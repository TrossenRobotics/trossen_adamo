// Trossen follower machine binary.
//
// Drives the follower arm in `position` mode, tracks leader joint state from
// the Adamo bus, publishes external efforts back, and (optionally) streams
// the host's RealSense camera through the Adamo C SDK.

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

#include "realsense_streamer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

// Built-in defaults. Override order: CLI flag > environment variable > default.
constexpr const char* kDefaultRobot      = "wxai";
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
    double smooth_alpha = 0.35;          // EMA factor; 1.0 disables
    std::optional<double> max_step;      // per-update absolute joint delta (rad); unset = no clamp
    double initial_sync_time = 2.0;      // seconds to ramp into the first leader pose
    double command_time = 0.02;          // controller goal_time (s) during steady state
    double stats_interval_s = 1.0;       // periodic latency-stats interval; 0 disables

    bool        camera_enabled = true;
    std::string camera_track = "main";
    std::string camera_serial;
    int         camera_width = 640;
    int         camera_height = 480;
    int         camera_fps = 30;
    int         camera_bitrate_kbps = 4000;
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
        "  --stats-interval SEC        latency stats print interval; 0 disables (default: 1.0)\n"
        "\n"
        "Camera options (RealSense color):\n"
        "  --no-camera                 disable the camera streamer\n"
        "  --camera-track NAME         (default: main; e.g. main/front/rear/head/overlay)\n"
        "  --camera-serial SERIAL      pin to a specific RealSense device\n"
        "  --camera-width N            (default: 640)\n"
        "  --camera-height N           (default: 480)\n"
        "  --camera-fps N              (default: 30)\n"
        "  --camera-bitrate-kbps N     (default: 4000)\n",
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
        else if (a == "--no-camera")           o.camera_enabled = false;
        else if (a == "--camera-track")        o.camera_track = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-serial")       o.camera_serial = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-width")        o.camera_width = ta::parse_int(ta::require_value(a, argc, argv, i), a);
        else if (a == "--camera-height")       o.camera_height = ta::parse_int(ta::require_value(a, argc, argv, i), a);
        else if (a == "--camera-fps")          o.camera_fps = ta::parse_int(ta::require_value(a, argc, argv, i), a);
        else if (a == "--camera-bitrate-kbps") o.camera_bitrate_kbps = ta::parse_int(ta::require_value(a, argc, argv, i), a);
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
    if (o.camera_enabled) {
        if (o.camera_width <= 0 || o.camera_height <= 0 || o.camera_fps <= 0 || o.camera_bitrate_kbps <= 0) {
            throw std::runtime_error("camera width/height/fps/bitrate must be positive");
        }
    }
    return o;
}

}  // namespace

int main(int argc, char** argv) try {
    namespace ta = trossen_adamo;
    ta::install_signal_handlers();

    const Options opt = parse(argc, argv);
    const adamo::Protocol protocol = ta::args::parse_protocol(opt.protocol_str);

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

    // Bring up the camera streamer first so the operator's video link is up
    // by the time teleop begins.
    std::unique_ptr<ta::camera::RealSenseStreamer> streamer;
    if (opt.camera_enabled) {
#ifdef ADAMO_HAS_VIDEO
        ta::camera::Config c;
        c.api_key      = opt.api_key;
        c.robot        = opt.robot;
        c.track        = opt.camera_track;
        c.serial       = opt.camera_serial;
        c.width        = opt.camera_width;
        c.height       = opt.camera_height;
        c.fps          = opt.camera_fps;
        c.bitrate_kbps = opt.camera_bitrate_kbps;
        c.protocol     = protocol;
        streamer = std::make_unique<ta::camera::RealSenseStreamer>(std::move(c));
        streamer->start();
#else
        std::fprintf(stderr,
            "follower: camera requested but Adamo was built without ADAMO_BUILD_VIDEO; "
            "rebuild with -DADAMO_BUILD_VIDEO=ON or pass --no-camera\n");
        return 1;
#endif
    }

    std::cout << "follower: opening Adamo session (" << opt.protocol_str << ")\n";
    auto session = adamo::Session::open(opt.api_key, protocol);
    const auto state_topic          = ta::topics::state_of(opt.robot);
    const auto effort_topic         = ta::topics::effort_of(opt.robot);
    const auto leader_ready_topic   = ta::topics::leader_ready_of(opt.robot);
    const auto follower_ready_topic = ta::topics::follower_ready_of(opt.robot);
    // Leader state runs on the SDK's receive thread, off the control loop.
    ta::LatestSubscriber state_sub(session, state_topic);
    auto ready_sub = session.subscribe(leader_ready_topic);

    std::cout << "follower: moving to home\n";
    ta::arm::move_home(*driver);

    auto ready_pub  = session.publisher(follower_ready_topic, 250, true, false);
    auto effort_pub = session.publisher(effort_topic,         250, true, false);
    ta::LatestPublisher effort_latest(std::move(effort_pub));

    ta::handshake::wait_for_peer_ready(ready_pub, ready_sub, opt.ready_timeout, "leader");

    std::cout << "follower: starting teleop\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    driver->set_all_modes(trossen_arm::Mode::position);

    const double teleop_started_at = ta::wire::now_seconds();
    const auto loop_end = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(opt.teleoperation_time);

    // Smoothing / stats state, scoped to the teleop loop.
    bool synced = false;
    std::vector<double> last_command;        // EMA history; populated on first sample
    std::vector<double> command_buf(ta::wire::kNumJoints, 0.0);
    std::vector<std::uint8_t> state_buf;     // reused; capacity stable after warm-up
    std::vector<double> latencies_ms;        // ring of leader-publish → here latencies
    latencies_ms.reserve(static_cast<std::size_t>(opt.rate_hz * opt.stats_interval_s) + 64);
    std::optional<std::chrono::steady_clock::time_point> next_stats;
    if (opt.stats_interval_s > 0.0) {
        next_stats = std::chrono::steady_clock::now() +
                     std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(opt.stats_interval_s));
    }

    while (!ta::stop_requested() && std::chrono::steady_clock::now() < loop_end) {
        const auto loop_start = std::chrono::steady_clock::now();

        // Always emit the latest effort reading.
        const auto efforts = driver->get_all_external_efforts();
        const auto effort_payload = ta::wire::encode_efforts(ta::wire::now_seconds(), efforts);
        effort_latest.put(effort_payload.data(), effort_payload.size());

        if (state_sub.poll(state_buf)) {
            try {
                const auto s = ta::wire::decode_state(state_buf.data(), state_buf.size());
                if (s.timestamp >= teleop_started_at) {
                    // Latency tracking based on the leader's publish timestamp.
                    const double latency_ms =
                        std::max(0.0, (ta::wire::now_seconds() - s.timestamp) * 1000.0);
                    latencies_ms.push_back(latency_ms);

                    if (!synced) {
                        // First fresh sample: ramp into the leader pose over
                        // initial-sync-time so we don't snap from home with a
                        // single 100 Hz step.
                        std::cout << "follower: syncing to first leader pose over "
                                  << opt.initial_sync_time << "s\n";
                        driver->set_all_positions(s.positions, opt.initial_sync_time, true);
                        last_command = s.positions;
                        synced = true;
                    } else {
                        // EMA toward the target, then optional per-tick clamp.
                        for (std::size_t i = 0; i < s.positions.size(); ++i) {
                            const double prev = last_command[i];
                            double value = prev + opt.smooth_alpha * (s.positions[i] - prev);
                            if (opt.max_step) {
                                const double max = *opt.max_step;
                                value = std::clamp(value, prev - max, prev + max);
                            }
                            command_buf[i] = value;
                        }
                        driver->set_all_positions(command_buf, opt.command_time, false, s.velocities);
                        last_command = command_buf;
                    }
                }
            } catch (const std::exception& e) {
                std::fprintf(stderr, "follower: bad state payload: %s\n", e.what());
            }
        }

        // Periodic latency stats (p50 / p95 / max). Cleared each interval.
        if (next_stats && std::chrono::steady_clock::now() >= *next_stats) {
            if (!latencies_ms.empty()) {
                std::sort(latencies_ms.begin(), latencies_ms.end());
                const std::size_t n = latencies_ms.size();
                const double p50 = latencies_ms[n / 2];
                const double p95 = (n >= 20)
                    ? latencies_ms[static_cast<std::size_t>(n * 0.95) - 1]
                    : latencies_ms.back();
                const double pmax = latencies_ms.back();
                std::fprintf(stderr,
                    "follower stats: samples=%zu latency_ms p50=%.1f p95=%.1f max=%.1f\n",
                    n, p50, p95, pmax);
                latencies_ms.clear();
            }
            *next_stats = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(opt.stats_interval_s));
        }

        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - loop_start).count();
        if (elapsed_ms > opt.stall_log_ms) {
            std::fprintf(stderr, "follower loop stall: %.1fms\n", elapsed_ms);
        }
        ta::sleep_until_next_tick(loop_start, opt.rate_hz);
    }

    effort_latest.close();

    std::cout << "follower: returning home + sleep\n";
    if (streamer) streamer->stop();
    // park_guard runs here as we return: position mode, home, sleep.
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "follower error: %s\n", e.what());
    return 1;
}
