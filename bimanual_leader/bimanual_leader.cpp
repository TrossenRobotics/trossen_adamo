// Trossen bimanual leader binary.
//
// Drives two Glide or wxai leader arms simultaneously in external_effort mode,
// applies force feedback per arm independently, corrects Glide joint-frame
// conventions per side (joints 3/4/5), and publishes each arm's positions on
// separate left/right Adamo topics at --rate-hz.
//
// Left arm  → leader_state_left  topic  / subscribes follower_effort_left
// Right arm → leader_state_right topic  / subscribes follower_effort_right
//
// For Glide arms:
//   Right Glide: positions[5] += joint5_offset
//   Left  Glide: positions[5] -= joint5_offset
//   Both sides:  positions[3,4] and velocities[3,4] are negated.

#include "adamo/adamo.hpp"
#include "libtrossen_arm/trossen_arm.hpp"
#include "trossen_adamo/args.hpp"
#include "trossen_adamo/arm.hpp"
#include "trossen_adamo/handshake.hpp"
#include "trossen_adamo/loop.hpp"
#include "trossen_adamo/publisher.hpp"
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
constexpr const char* kDefaultRobot          = "wxai";
constexpr const char* kDefaultLeftLeaderIp   = "192.168.1.4";
constexpr const char* kDefaultRightLeaderIp  = "192.168.1.2";

// Gripper force-feedback constants.
constexpr double LEADER_GRIPPER_MAX_EFFORT   = 27.0;
constexpr double FOLLOWER_GRIPPER_MAX_EFFORT = 87.5;
constexpr double GRIPPER_EFFORT_OFFSET       = 8.0;
constexpr double kDefaultJoint5Offset        = M_PI / 4.0;

struct Options {
    std::string api_key;
    std::string robot;            // resolved to kDefaultRobot if neither CLI nor env set
    std::string left_leader_ip;   // resolved to kDefaultLeftLeaderIp
    std::string right_leader_ip;  // resolved to kDefaultRightLeaderIp
    std::string left_model_str  = "glide_left";   // glide_left | wxai_v0
    std::string right_model_str = "glide_right";  // glide_right | wxai_v0
    std::string protocol_str = "quic";
    double teleoperation_time  = 20.0;
    double force_feedback_gain = 0.1;
    double connect_timeout     = 20.0;
    double ready_timeout       = 60.0;
    double rate_hz             = 100.0;
    double velocity_limit      = 5.0;
    double stall_log_ms        = 50.0;
    bool   clear_error         = false;
};

trossen_arm::Model parse_model(const std::string& s) {
    if (s == "glide_right") return trossen_arm::Model::glide_right;
    if (s == "glide_left")  return trossen_arm::Model::glide_left;
    if (s == "wxai_v0")     return trossen_arm::Model::wxai_v0;
    throw std::runtime_error("invalid model: " + s + " (expected glide_right|glide_left|wxai_v0)");
}

bool is_glide(const std::string& model_str) {
    return model_str == "glide_right" || model_str == "glide_left";
}

// Apply Glide joint-frame correction in-place.
// Right Glide: positions[5] += offset; Left Glide: positions[5] -= offset.
void apply_glide_transform(std::vector<double>& positions,
                            std::vector<double>& velocities,
                            bool is_right) {
    positions[3]  = -positions[3];
    positions[4]  = -positions[4];
    positions[5] += is_right ? kDefaultJoint5Offset : -kDefaultJoint5Offset;
    velocities[3] = -velocities[3];
    velocities[4] = -velocities[4];
}

void usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Required:\n"
        "  --api-key KEY                  (or ADAMO_API_KEY)\n"
        "\n"
        "Defaults (override via env or flag):\n"
        "  --robot NAME                   (default: wxai;          env ADAMO_ROBOT_NAME)\n"
        "  --left-leader-ip IP            (default: 192.168.1.4;   env ADAMO_TROSSEN_LEFT_LEADER_IP)\n"
        "  --right-leader-ip IP           (default: 192.168.1.2;   env ADAMO_TROSSEN_RIGHT_LEADER_IP)\n"
        "\n"
        "Options:\n"
        "  --protocol quic|udp|tcp        (default: quic)\n"
        "  --teleoperation-time SEC       (default: 20)\n"
        "  --force-feedback-gain G        (default: 0.1)\n"
        "  --rate-hz HZ                   (default: 100)\n"
        "  --velocity-limit RAD/S         (default: 5.0)\n"
        "  --connect-timeout SEC          (default: 20)\n"
        "  --ready-timeout SEC            (default: 60)\n"
        "  --stall-log-ms MS              (default: 50)\n"
        "  --clear-error                  clear arm fault on connect\n"
        "  --left-model NAME              glide_left|wxai_v0  (default: glide_left)\n"
        "  --right-model NAME             glide_right|wxai_v0 (default: glide_right)\n",
        prog);
}

Options parse(int argc, char** argv) {
    namespace ta = trossen_adamo::args;
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if      (a == "--api-key")             o.api_key = ta::require_value(a, argc, argv, i);
        else if (a == "--robot")               o.robot = ta::require_value(a, argc, argv, i);
        else if (a == "--left-leader-ip")      o.left_leader_ip = ta::require_value(a, argc, argv, i);
        else if (a == "--right-leader-ip")     o.right_leader_ip = ta::require_value(a, argc, argv, i);
        else if (a == "--protocol")            o.protocol_str = ta::require_value(a, argc, argv, i);
        else if (a == "--teleoperation-time")  o.teleoperation_time = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--force-feedback-gain") o.force_feedback_gain = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--rate-hz")             o.rate_hz = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--velocity-limit")      o.velocity_limit = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--connect-timeout")     o.connect_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--ready-timeout")       o.ready_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--stall-log-ms")        o.stall_log_ms = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--clear-error")         o.clear_error = true;
        else if (a == "--left-model")          o.left_model_str = ta::require_value(a, argc, argv, i);
        else if (a == "--right-model")         o.right_model_str = ta::require_value(a, argc, argv, i);
        else if (a == "--help" || a == "-h")   { usage(argv[0]); std::exit(0); }
        else throw std::runtime_error("unknown option: " + a);
    }
    o.api_key        = ta::require_cli_or_env("--api-key", "ADAMO_API_KEY", o.api_key);
    o.robot          = ta::cli_env_or_default(o.robot,          "ADAMO_ROBOT_NAME",              kDefaultRobot);
    o.left_leader_ip = ta::cli_env_or_default(o.left_leader_ip, "ADAMO_TROSSEN_LEFT_LEADER_IP",  kDefaultLeftLeaderIp);
    o.right_leader_ip= ta::cli_env_or_default(o.right_leader_ip,"ADAMO_TROSSEN_RIGHT_LEADER_IP", kDefaultRightLeaderIp);
    if (o.rate_hz <= 0.0)        throw std::runtime_error("--rate-hz must be positive");
    if (o.velocity_limit <= 0.0) throw std::runtime_error("--velocity-limit must be positive");
    parse_model(o.left_model_str);   // validate early
    parse_model(o.right_model_str);  // validate early
    return o;
}

}  // namespace

int main(int argc, char** argv) try {
    namespace ta = trossen_adamo;
    ta::install_signal_handlers();

    const Options opt = parse(argc, argv);
    const adamo::Protocol protocol = ta::args::parse_protocol(opt.protocol_str);

    // Configure both arms before creating park guards so that any configure
    // failure is clean (no guard to dismiss).
    std::cout << "bimanual_leader: configuring left arm at "  << opt.left_leader_ip  << "\n";
    auto left_driver = ta::arm::configure(opt.left_leader_ip,
                                          trossen_arm::StandardEndEffector::wxai_v0_leader,
                                          opt.clear_error,
                                          opt.connect_timeout,
                                          parse_model(opt.left_model_str));

    std::cout << "bimanual_leader: configuring right arm at " << opt.right_leader_ip << "\n";
    auto right_driver = ta::arm::configure(opt.right_leader_ip,
                                           trossen_arm::StandardEndEffector::wxai_v0_leader,
                                           opt.clear_error,
                                           opt.connect_timeout,
                                           parse_model(opt.right_model_str));

    // Park guards: declared after both configure() calls so they fire on
    // every exit path from here on (exception, signal-driven loop break, etc.).
    ta::arm::ArmParkGuard left_park(*left_driver,   "bimanual_leader_left");
    ta::arm::ArmParkGuard right_park(*right_driver, "bimanual_leader_right");

    std::cout << "bimanual_leader: opening Adamo session (" << opt.protocol_str << ")\n";
    auto session = adamo::Session::open(opt.api_key, protocol);

    const auto state_left_topic      = ta::topics::state_left_of(opt.robot);
    const auto state_right_topic     = ta::topics::state_right_of(opt.robot);
    const auto effort_left_topic     = ta::topics::effort_left_of(opt.robot);
    const auto effort_right_topic    = ta::topics::effort_right_of(opt.robot);
    const auto leader_ready_topic    = ta::topics::leader_ready_of(opt.robot);
    const auto follower_ready_topic  = ta::topics::follower_ready_of(opt.robot);

    // Effort feedback runs on the SDK's receive thread, off the control loop.
    ta::LatestSubscriber effort_left_sub(session,  effort_left_topic);
    ta::LatestSubscriber effort_right_sub(session, effort_right_topic);
    auto ready_sub = session.subscribe(follower_ready_topic);

    std::cout << "bimanual_leader: moving arms to home\n";
    ta::arm::move_home(*left_driver);
    ta::arm::move_home(*right_driver);

    auto ready_pub       = session.publisher(leader_ready_topic,  250, true, false);
    auto state_left_pub  = session.publisher(state_left_topic,    250, true, false);
    auto state_right_pub = session.publisher(state_right_topic,   250, true, false);
    ta::LatestPublisher state_left_latest(std::move(state_left_pub));
    ta::LatestPublisher state_right_latest(std::move(state_right_pub));

    ta::handshake::wait_for_peer_ready(ready_pub, ready_sub, opt.ready_timeout, "follower");

    std::cout << "bimanual_leader: starting teleop\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    left_driver->set_all_modes(trossen_arm::Mode::external_effort);
    right_driver->set_all_modes(trossen_arm::Mode::external_effort);

    const bool left_is_glide  = is_glide(opt.left_model_str);
    const bool right_is_glide = is_glide(opt.right_model_str);
    const double teleop_started_at = ta::wire::now_seconds();
    const auto loop_end = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(opt.teleoperation_time);

    std::vector<std::uint8_t> effort_left_buf;
    std::vector<std::uint8_t> effort_right_buf;
    std::vector<double> applied_left(ta::wire::kNumJoints - 1, 0.0);   // joints 0-5
    std::vector<double> applied_right(ta::wire::kNumJoints - 1, 0.0);

    while (!ta::stop_requested() && std::chrono::steady_clock::now() < loop_end) {
        const auto loop_start = std::chrono::steady_clock::now();

        // --- Left arm force feedback ---
        if (effort_left_sub.poll(effort_left_buf)) {
            try {
                const auto e = ta::wire::decode_efforts(effort_left_buf.data(), effort_left_buf.size());
                if (e.timestamp >= teleop_started_at) {
                    for (std::size_t i = 0; i < applied_left.size(); ++i) {
                        applied_left[i] = -opt.force_feedback_gain * e.efforts[i];
                    }
                    left_driver->set_arm_external_efforts(applied_left, 0.0, false);

                    // Gripper (joint 6): Glide uses normalised cubic fit;
                    // wxai_v0 applies scaled effort directly.
                    if (left_is_glide) {
                        const double effort_norm =
                            std::min(std::abs(e.efforts[ta::wire::kNumJoints - 1]) /
                                         FOLLOWER_GRIPPER_MAX_EFFORT, 1.0);
                        const double gripper_effort =
                            LEADER_GRIPPER_MAX_EFFORT * std::pow(effort_norm, 3) +
                            GRIPPER_EFFORT_OFFSET;
                        left_driver->set_gripper_external_effort(gripper_effort, 0.2, false);
                    } else {
                        left_driver->set_gripper_external_effort(
                            -opt.force_feedback_gain * e.efforts[ta::wire::kNumJoints - 1],
                            0.2, false);
                    }
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "bimanual_leader: bad left effort payload: %s\n", e.what());
            }
        }

        // --- Right arm force feedback ---
        if (effort_right_sub.poll(effort_right_buf)) {
            try {
                const auto e = ta::wire::decode_efforts(effort_right_buf.data(), effort_right_buf.size());
                if (e.timestamp >= teleop_started_at) {
                    for (std::size_t i = 0; i < applied_right.size(); ++i) {
                        applied_right[i] = -opt.force_feedback_gain * e.efforts[i];
                    }
                    right_driver->set_arm_external_efforts(applied_right, 0.0, false);

                    // Gripper (joint 6): Glide uses normalised cubic fit;
                    // wxai_v0 applies scaled effort directly.
                    if (right_is_glide) {
                        const double effort_norm =
                            std::min(std::abs(e.efforts[ta::wire::kNumJoints - 1]) /
                                         FOLLOWER_GRIPPER_MAX_EFFORT, 1.0);
                        const double gripper_effort =
                            LEADER_GRIPPER_MAX_EFFORT * std::pow(effort_norm, 3) +
                            GRIPPER_EFFORT_OFFSET;
                        right_driver->set_gripper_external_effort(gripper_effort, 0.2, false);
                    } else {
                        right_driver->set_gripper_external_effort(
                            -opt.force_feedback_gain * e.efforts[ta::wire::kNumJoints - 1],
                            0.2, false);
                    }
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "bimanual_leader: bad right effort payload: %s\n", e.what());
            }
        }

        // --- Read and publish left arm state ---
        {
            const auto out = left_driver->get_robot_output();
            auto positions  = out.joint.all.positions;
            auto velocities = out.joint.all.velocities;
            for (double& v : velocities) {
                v = std::clamp(v, -opt.velocity_limit, opt.velocity_limit);
            }
            if (left_is_glide) {
                apply_glide_transform(positions, velocities, /*is_right=*/false);
            }
            const auto payload = ta::wire::encode_state(ta::wire::now_seconds(), positions, velocities);
            state_left_latest.put(payload.data(), payload.size());
        }

        // --- Read and publish right arm state ---
        {
            const auto out = right_driver->get_robot_output();
            auto positions  = out.joint.all.positions;
            auto velocities = out.joint.all.velocities;
            for (double& v : velocities) {
                v = std::clamp(v, -opt.velocity_limit, opt.velocity_limit);
            }
            if (right_is_glide) {
                apply_glide_transform(positions, velocities, /*is_right=*/true);
            }
            const auto payload = ta::wire::encode_state(ta::wire::now_seconds(), positions, velocities);
            state_right_latest.put(payload.data(), payload.size());
        }

        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - loop_start).count();
        if (elapsed_ms > opt.stall_log_ms) {
            std::fprintf(stderr, "bimanual_leader loop stall: %.1fms\n", elapsed_ms);
        }
        ta::sleep_until_next_tick(loop_start, opt.rate_hz);
    }

    state_left_latest.close();
    state_right_latest.close();

    std::cout << "bimanual_leader: returning home + sleep\n";
    // Park guards run here as we return: position mode, home, sleep.
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "bimanual_leader error: %s\n", e.what());
    return 1;
}
