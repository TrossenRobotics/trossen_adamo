#include "adamo/adamo.hpp"
#include "trossen_vr/trossen_vr.hpp"
#include "libtrossen_arm/trossen_arm.hpp"

#include "trossen_adamo/args.hpp"
#include "trossen_adamo/arm.hpp"
#include "trossen_adamo/handshake.hpp"
#include "trossen_adamo/loop.hpp"
#include "trossen_adamo/subscriber.hpp"
#include "trossen_adamo/signal.hpp"
#include "trossen_adamo/topics.hpp"
#include "trossen_adamo/wire.hpp"

#include <iostream>
#include <chrono>
#include <stdexcept>
#include <unordered_map>
#include <variant>

namespace {

// Built-in defaults. Override order: CLI flag > environment variable > default.
// TODO(abhichothani42): robot name is a issue here, you something else for adamo topic name
constexpr const char* kDefaultRobot      = "vr_headset";
constexpr const char* kDefaultFollowerIp = "192.168.1.3";

struct Options {
    std::string api_key;
    std::string robot;          // resolved to kDefaultRobot if neither CLI nor env set
    std::string follower_ip;    // resolved to kDefaultFollowerIp if neither CLI nor env set
    std::string protocol_str = "quic";
    double teleoperation_time = 20.0;
    double connect_timeout = 20.0;
    double ready_timeout = 60.0;
    double rate_hz = 100.0;
    double stall_log_ms = 50.0;
    bool clear_error = false;

    // Smoothing / control-shaping parameters applied to the follower's
    // commanded positions.

    // make sure each one are being used and how
    double smooth_alpha = 0.35;          // EMA factor; 1.0 disables
    std::optional<double> max_step;      // per-update absolute joint delta (rad); unset = no clamp
    double initial_sync_time = 2.0;      // seconds to ramp into the first leader pose
    double command_time = 0.02;          // controller goal_time (s) during steady state
    double stats_interval_s = 1.0;       // periodic latency-stats interval; 0 disables
};

void usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Required:\n"
        "  --api-key KEY               (or ADAMO_API_KEY)\n"
        "\n"
        "Defaults (override via env or flag):\n"
        "  --robot NAME                (default: wxai;          env ADAMO_ROBOT_NAME)\n"
        "  --follower-ip IP            (default: 192.168.1.3;   env ADAMO_TROSSEN_FOLLOWER_IP)\n"
        "\n"
        "Teleop options:\n"
        "  --protocol quic|udp|tcp     (default: quic)\n"
        "  --teleoperation-time SEC    (default: 20)\n"
        "  --rate-hz HZ                (default: 100)\n"
        "  --connect-timeout SEC       (default: 20)\n"
        "  --ready-timeout SEC         (default: 60)\n"
        "  --stall-log-ms MS           (default: 50)\n"
        "  --clear-error               clear arm fault on connect\n"
        "\n"
        "Smoothing options (applied to commanded follower positions):\n"
        "  --smooth-alpha A            EMA factor in (0,1]; 1.0 disables (default: 0.35)\n"
        "  --max-step RAD              per-update absolute joint delta clamp (default: off)\n"
        "  --initial-sync-time SEC     ramp time into first leader pose (default: 2.0)\n"
        "  --command-time SEC          controller goal_time during steady state (default: 0.02)\n"
        "  --stats-interval SEC        latency stats print interval; 0 disables (default: 1.0)\n",
        prog);
}

Options parse(int argc, char** argv) {
    namespace ta = trossen_adamo::args;
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if      (a == "--api-key")             o.api_key = ta::require_value(a, argc, argv, i);
        else if (a == "--robot")               o.robot = ta::require_value(a, argc, argv, i);
        else if (a == "--follower-ip")         o.follower_ip = ta::require_value(a, argc, argv, i);
        else if (a == "--protocol")            o.protocol_str = ta::require_value(a, argc, argv, i);
        else if (a == "--teleoperation-time")  o.teleoperation_time = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--rate-hz")             o.rate_hz = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--connect-timeout")     o.connect_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--ready-timeout")       o.ready_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--stall-log-ms")        o.stall_log_ms = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--clear-error")         o.clear_error = true;
        else if (a == "--smooth-alpha")        o.smooth_alpha = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--max-step")            o.max_step = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--initial-sync-time")   o.initial_sync_time = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--command-time")        o.command_time = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--stats-interval")      o.stats_interval_s = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--help" || a == "-h")   { usage(argv[0]); std::exit(0); }
        else throw std::runtime_error("unknown option: " + a);
    }
    o.api_key     = ta::require_cli_or_env("--api-key",     "ADAMO_API_KEY", o.api_key);
    o.robot       = ta::cli_env_or_default(o.robot,       "ADAMO_ROBOT_NAME",          kDefaultRobot);
    o.follower_ip = ta::cli_env_or_default(o.follower_ip, "ADAMO_TROSSEN_FOLLOWER_IP", kDefaultFollowerIp);
    if (o.rate_hz <= 0.0) throw std::runtime_error("--rate-hz must be positive");
    if (!(o.smooth_alpha > 0.0 && o.smooth_alpha <= 1.0)) {
        throw std::runtime_error("--smooth-alpha must be in (0, 1]");
    }
    if (o.max_step.has_value() && *o.max_step <= 0.0) {
        throw std::runtime_error("--max-step must be positive");
    }
    if (o.initial_sync_time < 0.0) throw std::runtime_error("--initial-sync-time must be >= 0");
    if (o.command_time < 0.0)      throw std::runtime_error("--command-time must be >= 0");
    if (o.stats_interval_s < 0.0)  throw std::runtime_error("--stats-interval must be >= 0");
    return o;
}

}  // namespace

class EdgeDetector {
public:
    bool pressed(const trossen_vr::VRFrame& frame, const std::string& name) {
        auto it = frame.buttons.find(name);
        if (it == frame.buttons.end()) return false;
        if (!std::holds_alternative<bool>(it->second)) return false;

        bool current = std::get<bool>(it->second);
        bool previous = prev_[name];
        prev_[name] = current;
        return (current && !previous);
    }

    double analog(const trossen_vr::VRFrame& frame, const std::string& name) {
        auto it = frame.buttons.find(name);
        if (it == frame.buttons.end()) return 0.0;
        if (!std::holds_alternative<double>(it->second)) return 0.0;
        return std::get<double>(it->second);
    }

private:
    std::unordered_map<std::string, bool> prev_;
};

int main(int argc, char** argv) try {
    namespace ta = trossen_adamo;
    ta::install_signal_handlers();

    const Options opt = parse(argc, argv);
    const adamo::Protocol protocol = ta::args::parse_protocol(opt.protocol_str);

    // robot config
    const double gripper_max_m = 0.04;
    const double cmd_goal_time = 0.15;

    std::cout << "follower: configuring arm at " << opt.follower_ip << "\n";
    auto driver = ta::arm::configure(opt.follower_ip,
                                     trossen_arm::StandardEndEffector::wxai_v0_follower,
                                     opt.clear_error,
                                     opt.connect_timeout);

    // Park guard fires on every exit path (normal return, thrown
    // handshake timeout, decode/driver fault, signal-driven loop break).
    // Declared before the streamer so the camera stops first on
    // destruction; the arm parks in silence rather than holding the
    // RealSense pipeline open during the move.
    ta::arm::ArmParkGuard park_guard(*driver, "follower");

    std::cout << "follower: opening Adamo session (" << opt.protocol_str << ")\n";
    auto session = adamo::Session::open(opt.api_key, protocol);

    // TODO(abhichothani42): state_of topic name contains leader change it
    const auto state_topic          = ta::topics::state_of(opt.robot);
    const auto vr_headset_ready_topic   = ta::topics::vr_headset_ready_of(opt.robot);
    const auto follower_ready_topic = ta::topics::follower_ready_of(opt.robot);

    // Leader state runs on the SDK's receive thread, off the control loop.
    ta::LatestSubscriber state_sub(session, state_topic);
    auto ready_sub = session.subscribe(vr_headset_ready_topic);

    std::cout << "follower: moving to home\n";
    ta::arm::move_home(*driver);

    auto ready_pub  = session.publisher(follower_ready_topic, 250, true, false);
    
    ta::handshake::wait_for_peer_ready(ready_pub, ready_sub, opt.ready_timeout, "leader");

    std::cout << "follower: starting teleop\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    driver->set_all_modes(trossen_arm::Mode::position);

    const double teleop_started_at = ta::wire::now_seconds();
    const auto loop_end = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(opt.teleoperation_time);


    std::vector<std::uint8_t> state_buf;
    
    // trossen_vr teleop state
    bool teleop_active = false;
    Eigen::Matrix4d T_offset_right = Eigen::Matrix4d::Identity();
    bool offset_captured = false;
    
    EdgeDetector edges;

    double timestamp = 0.0;

    while (!ta::stop_requested() && std::chrono::steady_clock::now() < loop_end) {
        const auto loop_start = std::chrono::steady_clock::now();

        if (state_sub.poll(state_buf)) {
            try {
                const auto frame = ta::wire::decode_vr_frame(state_buf.data(), state_buf.size(), &timestamp);
            
                if (edges.pressed(frame, trossen_vr::ButtonNames::A)) {
                    teleop_active = !teleop_active;
                    if (teleop_active) {
                        offset_captured = false;
                        std::cout << "Teleop ENGAGED" << std::endl;
                    } else {
                        std::cout << "Teleop PAUSED" << std::endl;
                    }
                }

                if (edges.pressed(frame, trossen_vr::ButtonNames::B)) {
                    std::cout << "Exit requested via B button" << std::endl;
                    break;
                }
        
                double right_trig = edges.analog(frame, trossen_vr::ButtonNames::RightTrigger);
                driver->set_gripper_position(right_trig * gripper_max_m, 0.0, false);

                if (!teleop_active) continue;

                trossen_vr::Vec6 vr_right = trossen_vr::Vec6::Zero();
                bool right_valid = false;

                if (frame.right) {
                    vr_right = trossen_vr::unity_pose_to_vec6(frame.right->position, frame.right->rotation);
                    right_valid = true;
                }

                if (!offset_captured) {
                    if (right_valid) {
                        auto rp = driver->get_cartesian_positions();
                        Eigen::Map<Eigen::VectorXd> rs(rp.data(), 6);
                        T_offset_right = trossen_vr::vec6_to_T(rs) * trossen_vr::vec6_to_T(vr_right).inverse();
                    }
                    if (right_valid) {
                        offset_captured = true;
                        std::cout << "Offset captured — tracking active" << std::endl;
                    }
                    continue;
                }

                if (right_valid) {
                    Eigen::Matrix4d T_cmd = T_offset_right * trossen_vr::vec6_to_T(vr_right);
                    trossen_vr::Vec6 cmd = trossen_vr::T_to_vec6(T_cmd);
                    std::array<double, 6> goal{};
                    Eigen::VectorXd::Map(goal.data(), 6) = cmd;
                    driver->set_cartesian_positions(
                        goal, trossen_arm::InterpolationSpace::cartesian, cmd_goal_time, false);
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "follower: bad state payload: %s\n", e.what());
            }
        }

        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - loop_start).count();
        if (elapsed_ms > opt.stall_log_ms) {
            std::fprintf(stderr, "follower loop stall: %.1fms\n", elapsed_ms);
        }
        ta::sleep_until_next_tick(loop_start, opt.rate_hz);
    }

    std::cout << "follower: returning home + sleep\n";
    // park_guard runs here as we return: position mode, home, sleep.
    return 0;

} catch (const std::exception& e) {
    std::fprintf(stderr, "follower error: %s\n", e.what());
    return 1;
}