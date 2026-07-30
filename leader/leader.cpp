// Trossen leader machine binary.
//
// Drives the Glide arm or leader arm in external_effort mode, applies force feedback,
// for glide corrects joint-frame conventions for joints 3/4/5, and publishes
// positions at --rate-hz.

#include "adamo/adamo.hpp"
#include "libtrossen_arm/trossen_arm.hpp"
#include "trossen_adamo/args.hpp"
#include "trossen_adamo/arm.hpp"
#include "trossen_adamo/handshake.hpp"
#include "trossen_adamo/loop.hpp"
#include "trossen_adamo/publisher.hpp"
#include "trossen_adamo/recovery.hpp"
#include "trossen_adamo/signal.hpp"
#include "trossen_adamo/subscriber.hpp"
#include "trossen_adamo/topics.hpp"
#include "trossen_adamo/wire.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace {

// Built-in defaults. Override order: CLI flag > environment variable > default.
constexpr const char* kDefaultRobot     = "wxai";
constexpr const char* kDefaultLeaderIp  = "192.168.1.2";

// Gripper force-feedback constants.
constexpr double LEADER_GRIPPER_MAX_EFFORT   = 20.0;
constexpr double FOLLOWER_GRIPPER_MAX_EFFORT = 100.0;
constexpr double GRIPPER_EFFORT_OFFSET       = 6.5;
constexpr double kDefaultJoint5Offset        = M_PI / 4.0;

struct Options {
    std::string api_key;
    std::string robot;        // resolved to kDefaultRobot if neither CLI nor env set
    std::string leader_ip;    // resolved to kDefaultLeaderIp if neither CLI nor env set
    std::string protocol_str = "quic";
    double teleoperation_time = 20.0;
    double force_feedback_gain = 0.1;
    double connect_timeout = 20.0;
    double ready_timeout = 60.0;
    double rate_hz = 100.0;
    double velocity_limit = 5.0;
    double stall_log_ms = 50.0;
    bool clear_error = false;
    std::string model_str = "glide_right";
};

trossen_arm::Model parse_model(const std::string& s) {
    if (s == "glide_right") return trossen_arm::Model::glide_right;
    if (s == "glide_left")  return trossen_arm::Model::glide_left;
    if (s == "wxai_v0")     return trossen_arm::Model::wxai_v0;
    throw std::runtime_error("invalid --model: " + s + " (expected glide_right|glide_left|wxai_v0)");
}

bool is_glide(const std::string& model_str) {
    return model_str == "glide_right" || model_str == "glide_left";
}

void usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Required:\n"
        "  --api-key KEY               (or ADAMO_API_KEY)\n"
        "\n"
        "Defaults (override via env or flag):\n"
        "  --robot NAME                (default: wxai;          env ADAMO_ROBOT_NAME)\n"
        "  --leader-ip IP              (default: 192.168.1.2;   env ADAMO_TROSSEN_LEADER_IP)\n"
        "\n"
        "Options:\n"
        "  --protocol quic|udp|tcp     (default: quic)\n"
        "  --teleoperation-time SEC    (default: 20)\n"
        "  --force-feedback-gain G     (default: 0.1)\n"
        "  --rate-hz HZ                (default: 100)\n"
        "  --velocity-limit RAD/S      (default: 5.0)\n"
        "  --connect-timeout SEC       (default: 20)\n"
        "  --ready-timeout SEC         (default: 60)\n"
        "  --stall-log-ms MS           (default: 50)\n"
        "  --clear-error               clear arm fault on connect\n"
        "  --model NAME                glide_right|glide_left|wxai_v0 (default: glide_right)\n",
        prog);
}

Options parse(int argc, char** argv) {
    namespace ta = trossen_adamo::args;
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if      (a == "--api-key")             o.api_key = ta::require_value(a, argc, argv, i);
        else if (a == "--robot")               o.robot = ta::require_value(a, argc, argv, i);
        else if (a == "--leader-ip")           o.leader_ip = ta::require_value(a, argc, argv, i);
        else if (a == "--protocol")            o.protocol_str = ta::require_value(a, argc, argv, i);
        else if (a == "--teleoperation-time")  o.teleoperation_time = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--force-feedback-gain") o.force_feedback_gain = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--rate-hz")             o.rate_hz = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--velocity-limit")      o.velocity_limit = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--connect-timeout")     o.connect_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--ready-timeout")       o.ready_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--stall-log-ms")        o.stall_log_ms = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--clear-error")         o.clear_error = true;
        else if (a == "--model")               o.model_str = ta::require_value(a, argc, argv, i);
        else if (a == "--help" || a == "-h")   { usage(argv[0]); std::exit(0); }
        else throw std::runtime_error("unknown option: " + a);
    }
    o.api_key   = ta::require_cli_or_env("--api-key",   "ADAMO_API_KEY", o.api_key);
    o.robot     = ta::cli_env_or_default(o.robot,     "ADAMO_ROBOT_NAME",        kDefaultRobot);
    o.leader_ip = ta::cli_env_or_default(o.leader_ip, "ADAMO_TROSSEN_LEADER_IP", kDefaultLeaderIp);
    if (o.rate_hz <= 0.0) throw std::runtime_error("--rate-hz must be positive");
    if (o.velocity_limit <= 0.0) throw std::runtime_error("--velocity-limit must be positive");
    parse_model(o.model_str);
    return o;
}

}  // namespace

int main(int argc, char** argv) try {
    namespace ta = trossen_adamo;
    ta::install_signal_handlers();

    const Options opt = parse(argc, argv);
    const adamo::Protocol protocol = ta::args::parse_protocol(opt.protocol_str);

    std::cout << "leader: configuring arm at " << opt.leader_ip << "\n";
    auto driver = ta::arm::configure(opt.leader_ip,
                                     trossen_arm::StandardEndEffector::wxai_v0_leader,
                                     opt.clear_error,
                                     opt.connect_timeout,
                                     parse_model(opt.model_str));

    const bool glide_leader = is_glide(opt.model_str);

    // Cap the gripper's max opening and widen the arm joints' velocity/effort
    // fault tolerance to their max.
    {
        auto joint_limits = driver->get_joint_limits();
        joint_limits.back().position_max = 0.05;
        for (int i = 0; i < 6; ++i) {
            joint_limits[i].velocity_tolerance = joint_limits[i].velocity_max;
            joint_limits[i].effort_tolerance = joint_limits[i].effort_max;
        }
        driver->set_joint_limits(joint_limits);
    }

    // From here on, any thrown exception (including handshake timeout,
    // driver fault, or external_effort-mode operation failures) must run
    // through safe_park before unwinding past main: the leader spends most
    // of its life in external_effort mode, and abandoning the arm there
    // with a stale torque is a real-world hazard.
    ta::arm::ArmParkGuard park_guard(*driver, "leader");

    std::cout << "leader: opening Adamo session (" << opt.protocol_str << ")\n";
    auto session = adamo::Session::open(opt.api_key, protocol);
    const auto state_topic         = ta::topics::state_of(opt.robot);
    const auto effort_topic        = ta::topics::effort_of(opt.robot);
    const auto leader_ready_topic  = ta::topics::leader_ready_of(opt.robot);
    const auto follower_ready_topic= ta::topics::follower_ready_of(opt.robot);
    const auto teleop_toggle_topic = ta::topics::teleop_toggle_of(opt.robot);
    const auto error_recover_topic = ta::topics::error_recover_of(opt.robot);
    // Effort feedback runs on the SDK's receive thread, off the control loop.
    ta::LatestSubscriber effort_sub(session, effort_topic);
    auto ready_sub = session.subscribe(follower_ready_topic);
    auto teleop_toggle_pub = session.publisher(teleop_toggle_topic, 250, true, false);
    auto error_recover_pub = session.publisher(error_recover_topic, 250, true, false);

    std::cout << "leader: moving to home\n";
    ta::arm::move_home(*driver);

    auto ready_pub = session.publisher(leader_ready_topic, /*priority=*/250,
                                       /*express=*/true, /*reliable=*/false);
    auto state_pub = session.publisher(state_topic, /*priority=*/250,
                                       /*express=*/true, /*reliable=*/false);
    ta::LatestPublisher state_latest(std::move(state_pub));

    ta::handshake::wait_for_peer_ready(ready_pub, ready_sub, opt.ready_timeout, "follower");

    std::cout << "leader: starting teleop\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    driver->set_all_modes(trossen_arm::Mode::external_effort);

    // Glide's gripper (finger) is force-controlled via effort mode
    if (glide_leader) {
        driver->set_gripper_mode(trossen_arm::Mode::effort);
        driver->set_gripper_effort(GRIPPER_EFFORT_OFFSET, 0.2, false);
    }

    const double teleop_started_at = ta::wire::now_seconds();
    const auto loop_end = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(opt.teleoperation_time);

    std::vector<std::uint8_t> effort_buf;     // reused; capacity stable after warm-up
    std::vector<double> applied(ta::wire::kNumJoints - 1, 0.0);  // joints 0-5

    // Glide-only: SEL_1 starts/stops the follower's teleop, SEL_2 clears a
    // follower fault and resumes. See trossen_adamo/recovery.hpp and
    // follower/follower.cpp's --button-gated mode.
    ta::recovery::ButtonTrigger teleop_toggle_button(/*bit=*/0);  // SEL_1
    ta::recovery::ButtonTrigger error_recover_button(/*bit=*/1);  // SEL_2

    while (!ta::stop_requested() && std::chrono::steady_clock::now() < loop_end) {
        const auto loop_start = std::chrono::steady_clock::now();

        // Apply force feedback if a fresh effort sample arrived. The actual
        // receive happens on the SDK's background thread; here we only do
        // a try_lock + swap, never a network/queue call.
        if (effort_sub.poll(effort_buf)) {
            try {
                const auto e = ta::wire::decode_efforts(effort_buf.data(), effort_buf.size());
                if (e.timestamp >= teleop_started_at) {
                    for (std::size_t i = 0; i < applied.size(); ++i) {
                        applied[i] = -opt.force_feedback_gain * e.efforts[i];
                    }
                    driver->set_arm_external_efforts(applied, 0.0, false);

                    // Gripper (joint 6): Glide uses normalised cubic fit;
                    // wxai_v0 applies scaled effort directly.
                    if (glide_leader) {
                        const double effort_norm =
                            std::min(std::abs(e.efforts[ta::wire::kNumJoints - 1]) /
                                         FOLLOWER_GRIPPER_MAX_EFFORT, 1.0);
                        const double gripper_effort =
                            LEADER_GRIPPER_MAX_EFFORT * std::pow(effort_norm, 3) +
                            GRIPPER_EFFORT_OFFSET;
                        driver->set_gripper_effort(gripper_effort, 0.1, false);
                    } else {
                        driver->set_gripper_external_effort(
                            -opt.force_feedback_gain * e.efforts[ta::wire::kNumJoints - 1],
                            0.2, false);
                    }
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "leader: bad effort payload: %s\n", e.what());
            }
        }

        // Read positions + velocities in a single Trossen daemon slot.
        // The driver serializes every getter against its background UDP
        // daemon via mutex_preempt_; calling get_all_positions() then
        // get_all_velocities() is two daemon cycles. get_robot_output()
        // returns both (and more) in one cycle, halving the per-tick
        // contention window with the daemon.
        const auto out = driver->get_robot_output();
        auto positions  = out.joint.all.positions;
        auto velocities  = out.joint.all.velocities;
        for (double& v : velocities) {
            v = std::clamp(v, -opt.velocity_limit, opt.velocity_limit);
        }
        if (glide_leader) {
            positions[3] = -positions[3];
            positions[4] = -positions[4];
            positions[5] += (opt.model_str == "glide_left") ? -kDefaultJoint5Offset : kDefaultJoint5Offset;
            velocities[3] = -velocities[3];
            velocities[4] = -velocities[4];
        }
        const auto payload = ta::wire::encode_state(ta::wire::now_seconds(), positions, velocities);
        state_latest.put(payload.data(), payload.size());

        // Glide-only: forward button presses to the follower.
        if (glide_leader) {
            if (teleop_toggle_button.poll(*driver)) {
                std::cout << "leader: start/stop button pressed, notifying follower\n";
                const auto p = ta::wire::encode_ready(ta::wire::now_seconds());
                teleop_toggle_pub.put(p.data(), p.size());
            }
            if (error_recover_button.poll(*driver)) {
                std::cout << "leader: error-recovery button pressed, notifying follower\n";
                const auto p = ta::wire::encode_ready(ta::wire::now_seconds());
                error_recover_pub.put(p.data(), p.size());
            }
        }

        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - loop_start).count();
        if (elapsed_ms > opt.stall_log_ms) {
            std::fprintf(stderr, "leader loop stall: %.1fms\n", elapsed_ms);
        }
        ta::sleep_until_next_tick(loop_start, opt.rate_hz);
    }

    state_latest.close();

    std::cout << "leader: returning home + sleep\n";
    // park_guard runs here as we return: position mode, home, sleep.
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "leader error: %s\n", e.what());
    return 1;
}
