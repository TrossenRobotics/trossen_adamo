// VR follower machine binary.
//
// Drives the follower arm in `position` mode (cartesian control), subscribes
// to VR frames from vr_headset via Adamo pubsub, and mirrors VR controller
// movements to robot arm end-effector. Grip/hand trigger controls engagement
// (auto-engage when held, auto-disengage when released), button B exits,
// right index trigger controls gripper.

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

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using trossen_vr::pose6d_to_transform4d;
using trossen_vr::transform4d_to_pose6d;

namespace {

// Built-in defaults. Override order: CLI flag > environment variable > default.
constexpr const char* kDefaultRobot      = "vr_teleop";
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
};

void usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Required:\n"
        "  --api-key KEY               (or ADAMO_API_KEY)\n"
        "\n"
        "Defaults (override via env or flag):\n"
        "  --robot NAME                (default: vr_teleop;     env ADAMO_ROBOT_NAME)\n"
        "  --follower-ip IP            (default: 192.168.1.3;   env ADAMO_TROSSEN_FOLLOWER_IP)\n"
        "\n"
        "Options:\n"
        "  --protocol quic|udp|tcp     (default: quic)\n"
        "  --teleoperation-time SEC    (default: 20)\n"
        "  --rate-hz HZ                (default: 100)\n"
        "  --connect-timeout SEC       (default: 20)\n"
        "  --ready-timeout SEC         (default: 60)\n"
        "  --stall-log-ms MS           (default: 50)\n"
        "  --clear-error               clear arm fault on connect\n",
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
        else if (a == "--help" || a == "-h")   { usage(argv[0]); std::exit(0); }
        else throw std::runtime_error("unknown option: " + a);
    }
    o.api_key     = ta::require_cli_or_env("--api-key",     "ADAMO_API_KEY", o.api_key);
    o.robot       = ta::cli_env_or_default(o.robot,       "ADAMO_ROBOT_NAME",          kDefaultRobot);
    o.follower_ip = ta::cli_env_or_default(o.follower_ip, "ADAMO_TROSSEN_FOLLOWER_IP", kDefaultFollowerIp);
    if (o.rate_hz <= 0.0) throw std::runtime_error("--rate-hz must be positive");
    return o;
}

}  // namespace

int main(int argc, char** argv) try {
    namespace ta = trossen_adamo;
    ta::install_signal_handlers();

    const Options opt = parse(argc, argv);
    const adamo::Protocol protocol = ta::args::parse_protocol(opt.protocol_str);

    // Robot configuration constants.
    const double gripper_max_m = 0.04;    // Maximum gripper opening (meters).
    const double cmd_goal_time = 0.15;    // Cartesian command goal time (seconds).

    std::cout << "vr_follower: configuring arm at " << opt.follower_ip << "\n";
    auto driver = ta::arm::configure(opt.follower_ip,
                                     trossen_arm::StandardEndEffector::wxai_v0_follower,
                                     opt.clear_error,
                                     opt.connect_timeout);

    // Park guard ensures arm returns to home + sleep on all exit paths.
    // Declared before session so camera/network stops before arm parking.
    ta::arm::ArmParkGuard park_guard(*driver, "follower");

    std::cout << "vr_follower: opening Adamo session (" << opt.protocol_str << ")\n";
    auto session = adamo::Session::open(opt.api_key, protocol);

    const auto state_topic            = ta::topics::vr_state_of(opt.robot);
    const auto vr_headset_ready_topic = ta::topics::vr_headset_ready_of(opt.robot);
    const auto follower_ready_topic   = ta::topics::follower_ready_of(opt.robot);

    // VR frames run on the SDK's receive thread, off the control loop.
    ta::LatestSubscriber state_sub(session, state_topic);
    auto ready_sub = session.subscribe(vr_headset_ready_topic);

    std::cout << "vr_follower: moving to home\n";
    ta::arm::move_home(*driver);

    auto ready_pub = session.publisher(follower_ready_topic, 250, true, false);
    
    // Wait for vr_headset to be ready before starting teleop.
    ta::handshake::wait_for_peer_ready(ready_pub, ready_sub, opt.ready_timeout, "vr_headset");

    std::cout << "vr_follower: starting teleop\n";
    std::cout << "vr_follower: grip/hand trigger to engage, release to pause. Press B to exit\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    driver->set_all_modes(trossen_arm::Mode::position);

    const auto loop_end = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(opt.teleoperation_time);

    std::vector<std::uint8_t> state_buf;
    
    // VR teleop state - follows manual_polling pattern from trossen_vr demos.
    trossen_vr::Transform4D T_offset_right;
    bool offset_captured = false;
    
    // Track previous button/tracking state for edge detection.
    uint8_t prev_right_tracked = 0;
    uint8_t prev_button_b = 0;

    double timestamp = 0.0;

    while (!ta::stop_requested() && std::chrono::steady_clock::now() < loop_end) {
        const auto loop_start = std::chrono::steady_clock::now();

        if (state_sub.poll(state_buf)) {
            try {
                const auto frame = ta::wire::decode_vr_frame(state_buf.data(), state_buf.size(), &timestamp);
                
                bool right_tracked = frame.right_controller.is_tracked != 0;

                // Detect engage (tracking transition 0→1 from grip trigger press).
                if (right_tracked && !prev_right_tracked) {
                    offset_captured = false;
                    std::cout << "Right controller ENGAGED (grip held)\n";
                }

                // Detect release (tracking transition 1→0 from grip trigger release).
                if (!right_tracked && prev_right_tracked) {
                    std::cout << "Right controller PAUSED (grip released)\n";
                }
                prev_right_tracked = right_tracked;

                // Edge detect button B (exit).
                uint8_t button_b = frame.right_controller.buttons.two;
                if (button_b && !prev_button_b) {
                    std::cout << "Exit requested via B button\n";
                    break;
                }
                prev_button_b = button_b;

                // Update gripper (index trigger) every frame.
                driver->set_gripper_position(
                    frame.right_controller.triggers.index_trigger * gripper_max_m, 0.0, false);

                // Capture offset on first valid frame after tracking engages.
                if (!offset_captured && right_tracked) {
                    auto rp = driver->get_cartesian_positions();
                    trossen_vr::Pose6D robot_pose;
                    robot_pose.x = rp[0]; robot_pose.y = rp[1]; robot_pose.z = rp[2];
                    robot_pose.ax = rp[3]; robot_pose.ay = rp[4]; robot_pose.az = rp[5];

                    trossen_vr::Transform4D T_robot = pose6d_to_transform4d(robot_pose);
                    trossen_vr::Transform4D T_vr = pose6d_to_transform4d(frame.right_controller.pose6d);
                    T_offset_right = T_robot * T_vr.inverse();
                    
                    offset_captured = true;
                    continue;
                }

                // Send cartesian commands only when controller is tracked.
                if (offset_captured && right_tracked) {
                    trossen_vr::Transform4D T_vr = pose6d_to_transform4d(frame.right_controller.pose6d);
                    trossen_vr::Transform4D T_cmd = T_offset_right * T_vr;
                    trossen_vr::Pose6D cmd_pose = transform4d_to_pose6d(T_cmd);
                    
                    std::array<double, 6> goal{cmd_pose.x, cmd_pose.y, cmd_pose.z,
                                              cmd_pose.ax, cmd_pose.ay, cmd_pose.az};
                    driver->set_cartesian_positions(
                        goal, trossen_arm::InterpolationSpace::cartesian, cmd_goal_time, false);
                }
                    
            } catch (const std::exception& e) {
                std::fprintf(stderr, "vr_follower: bad state payload: %s\n", e.what());
            }
        }

        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - loop_start).count();
        if (elapsed_ms > opt.stall_log_ms) {
            std::fprintf(stderr, "vr_follower loop stall: %.1fms\n", elapsed_ms);
        }
        ta::sleep_until_next_tick(loop_start, opt.rate_hz);
    }

    std::cout << "vr_follower: returning home + sleep\n";
    // park_guard runs here as we return: position mode, home, sleep.
    return 0;

} catch (const std::exception& e) {
    std::fprintf(stderr, "vr_follower error: %s\n", e.what());
    return 1;
}
