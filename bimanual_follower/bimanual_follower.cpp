// Trossen bimanual follower binary.
//
// Drives two follower arms (wxai_v0 or pro) simultaneously in `position` mode,
// tracks left/right leader joint states from the Adamo bus, publishes external
// efforts for each arm back on separate left/right topics, and (optionally)
// streams multiple cameras through the Adamo C SDK.
//
// Left arm  → subscribes leader_state_left  / publishes follower_effort_left
// Right arm → subscribes leader_state_right / publishes follower_effort_right
//
// Camera backends:
//   realsense  – up to 4 cameras; use --num-cameras 1-4 (default: 3)
//   zed        – up to 3 cameras; use --num-cameras 1-3 (default: 3)
//
// Per-camera track and serial are configured via:
//   --camera-track-N NAME    (default: cam0, cam1, cam2, cam3)
//   --camera-serial-N SERIAL (default: empty = auto-detect)

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

#ifdef ADAMO_TROSSEN_HAS_REALSENSE
#include "trossen_adamo/realsense_streamer.hpp"
#endif
#ifdef ADAMO_TROSSEN_HAS_ZED
#include "trossen_adamo/zed_streamer.hpp"
#endif

#include <algorithm>
#include <array>
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
#include <utility>
#include <vector>

namespace {

// Built-in defaults. Override order: CLI flag > environment variable > default.
constexpr const char* kDefaultRobot           = "wxai";
constexpr const char* kDefaultLeftFollowerIp  = "192.168.1.5";
constexpr const char* kDefaultRightFollowerIp = "192.168.1.3";
constexpr int kMaxCamerasRealSense = 4;
constexpr int kMaxCamerasZed       = 3;

struct Options {
    std::string api_key;
    std::string robot;               // resolved to kDefaultRobot if neither CLI nor env set
    std::string left_follower_ip;    // resolved to kDefaultLeftFollowerIp
    std::string right_follower_ip;   // resolved to kDefaultRightFollowerIp
    std::string left_model_str  = "wxai_v0";  // wxai_v0 | pro
    std::string right_model_str = "wxai_v0";  // wxai_v0 | pro
    std::string protocol_str = "quic";
    double teleoperation_time  = 20.0;
    double connect_timeout     = 20.0;
    double ready_timeout       = 60.0;
    double rate_hz             = 100.0;
    double stall_log_ms        = 50.0;
    bool   clear_error         = false;

    // Glide-only: start stopped and wait for a leader button press before
    // tracking (applies to both sides).
    bool button_gated = false;

    // Smoothing / control-shaping parameters.
    double smooth_alpha      = 0.35;
    std::optional<double> max_step;
    double initial_sync_time = 2.0;
    double command_time      = 0.02;
    double stats_interval_s  = 1.0;

    // Camera options.
    bool        camera_enabled  = true;
#if defined(ADAMO_TROSSEN_HAS_REALSENSE)
    std::string camera_backend  = "realsense";  // "realsense" | "zed"
#elif defined(ADAMO_TROSSEN_HAS_ZED)
    std::string camera_backend  = "zed";        // "realsense" | "zed"
#else
    std::string camera_backend;                 // no backend compiled in
#endif
    int         num_cameras     = 3;
    // Per-camera track names and serial numbers (indices 0..num_cameras-1).
    std::array<std::string, kMaxCamerasRealSense> camera_tracks  = {"cam0", "cam1", "cam2", "cam3"};
    std::array<std::string, kMaxCamerasRealSense> camera_serials = {"", "", "", ""};
    // RealSense-specific.
    int         camera_width        = 640;
    int         camera_height       = 480;
    int         camera_fps          = 30;
    int         camera_bitrate_kbps = 4000;
    // ZED-specific.
    std::string camera_resolution_str = "HD1200";
};

struct FollowerModelConfig {
    trossen_arm::Model       model;
    trossen_arm::EndEffector end_effector;
};

FollowerModelConfig parse_follower_model(const std::string& s) {
    if (s == "wxai_v0") return {trossen_arm::Model::wxai_v0, trossen_arm::StandardEndEffector::wxai_v0_follower};
    if (s == "pro")     return {trossen_arm::Model::pro,     trossen_arm::StandardEndEffector::pro_follower};
    throw std::runtime_error("invalid --model: " + s + " (expected wxai_v0|pro)");
}

#ifdef ADAMO_TROSSEN_HAS_ZED
sl::RESOLUTION parse_zed_resolution(const std::string& s) {
    if (s == "HD2K")   return sl::RESOLUTION::HD2K;
    if (s == "HD1200") return sl::RESOLUTION::HD1200;
    if (s == "HD1080") return sl::RESOLUTION::HD1080;
    if (s == "HD720")  return sl::RESOLUTION::HD720;
    if (s == "SVGA")   return sl::RESOLUTION::SVGA;
    if (s == "VGA")    return sl::RESOLUTION::VGA;
    throw std::runtime_error("invalid --camera-resolution: " + s +
                             " (expected HD2K|HD1200|HD1080|HD720|SVGA|VGA)");
}
#endif

void usage(const char* prog) {
    std::fprintf(stderr,
        "Usage: %s [options]\n"
        "\n"
        "Required:\n"
        "  --api-key KEY                    (or ADAMO_API_KEY)\n"
        "\n"
        "Defaults (override via env or flag):\n"
        "  --robot NAME                     (default: wxai;          env ADAMO_ROBOT_NAME)\n"
        "  --left-follower-ip IP            (default: 192.168.1.5;   env ADAMO_TROSSEN_LEFT_FOLLOWER_IP)\n"
        "  --right-follower-ip IP           (default: 192.168.1.3;   env ADAMO_TROSSEN_RIGHT_FOLLOWER_IP)\n"
        "\n"
        "Teleop options:\n"
        "  --protocol quic|udp|tcp          (default: quic)\n"
        "  --teleoperation-time SEC         (default: 20)\n"
        "  --rate-hz HZ                     (default: 100)\n"
        "  --connect-timeout SEC            (default: 20)\n"
        "  --ready-timeout SEC              (default: 60)\n"
        "  --stall-log-ms MS                (default: 50)\n"
        "  --clear-error                    clear arm fault on connect\n"
        "  --left-model NAME                wxai_v0|pro (default: wxai_v0)\n"
        "  --right-model NAME               wxai_v0|pro (default: wxai_v0)\n"
        "  --button-gated                   Glide leaders only: start stopped; SEL_1 on each leader\n"
        "                                   starts/stops that side's teleop (stop moves it home),\n"
        "                                   SEL_2 clears a fault and resumes (independent per side)\n"
        "\n"
        "Smoothing options:\n"
        "  --smooth-alpha A                 EMA factor in (0,1]; 1.0 disables (default: 0.35)\n"
        "  --max-step RAD                   per-update absolute joint delta clamp (default: off)\n"
        "  --initial-sync-time SEC          ramp time into first leader pose (default: 2.0)\n"
        "  --command-time SEC               controller goal_time during steady state (default: 0.02)\n"
        "  --stats-interval SEC             latency stats print interval; 0 disables (default: 1.0)\n"
        "\n"
        "Camera options:\n"
        "  --no-camera                      disable all camera streamers\n"
#if defined(ADAMO_TROSSEN_HAS_REALSENSE) && defined(ADAMO_TROSSEN_HAS_ZED)
        "  --camera-backend NAME            realsense|zed (default: realsense)\n"
#elif defined(ADAMO_TROSSEN_HAS_REALSENSE)
        "  --camera-backend NAME            realsense (only backend this build supports)\n"
#elif defined(ADAMO_TROSSEN_HAS_ZED)
        "  --camera-backend NAME            zed (only backend this build supports)\n"
#else
        "  --camera-backend NAME            (none — this build has no camera backend compiled in)\n"
#endif
        "  --num-cameras N                  1-4 for realsense, 1-3 for zed (default: 3)\n"
        "  --camera-track-0 NAME            track name for camera 0 (default: cam0)\n"
        "  --camera-track-1 NAME            track name for camera 1 (default: cam1)\n"
        "  --camera-track-2 NAME            track name for camera 2 (default: cam2)\n"
        "  --camera-track-3 NAME            track name for camera 3 (default: cam3)\n"
        "  --camera-serial-0 SERIAL         serial for camera 0 (default: auto)\n"
        "  --camera-serial-1 SERIAL         serial for camera 1 (default: auto)\n"
        "  --camera-serial-2 SERIAL         serial for camera 2 (default: auto)\n"
        "  --camera-serial-3 SERIAL         serial for camera 3 (default: auto)\n"
        "  --camera-width N                 RealSense only (default: 640)\n"
        "  --camera-height N                RealSense only (default: 480)\n"
        "  --camera-resolution RES          ZED only: HD2K|HD1200|HD1080|HD720|SVGA|VGA (default: HD1200)\n"
        "  --camera-fps N                   (default: 30)\n"
        "  --camera-bitrate-kbps N          (default: 4000)\n",
        prog);
}

Options parse(int argc, char** argv) {
    namespace ta = trossen_adamo::args;
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if      (a == "--api-key")             o.api_key = ta::require_value(a, argc, argv, i);
        else if (a == "--robot")               o.robot = ta::require_value(a, argc, argv, i);
        else if (a == "--left-follower-ip")    o.left_follower_ip = ta::require_value(a, argc, argv, i);
        else if (a == "--right-follower-ip")   o.right_follower_ip = ta::require_value(a, argc, argv, i);
        else if (a == "--left-model")          o.left_model_str = ta::require_value(a, argc, argv, i);
        else if (a == "--right-model")         o.right_model_str = ta::require_value(a, argc, argv, i);
        else if (a == "--protocol")            o.protocol_str = ta::require_value(a, argc, argv, i);
        else if (a == "--teleoperation-time")  o.teleoperation_time = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--rate-hz")             o.rate_hz = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--connect-timeout")     o.connect_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--ready-timeout")       o.ready_timeout = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--stall-log-ms")        o.stall_log_ms = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--clear-error")         o.clear_error = true;
        else if (a == "--button-gated")        o.button_gated = true;
        else if (a == "--smooth-alpha")        o.smooth_alpha = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--max-step")            o.max_step = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--initial-sync-time")   o.initial_sync_time = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--command-time")        o.command_time = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--stats-interval")      o.stats_interval_s = ta::parse_double(ta::require_value(a, argc, argv, i), a);
        else if (a == "--no-camera")           o.camera_enabled = false;
        else if (a == "--camera-backend")      o.camera_backend = ta::require_value(a, argc, argv, i);
        else if (a == "--num-cameras")         o.num_cameras = ta::parse_int(ta::require_value(a, argc, argv, i), a);
        else if (a == "--camera-track-0")      o.camera_tracks[0] = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-track-1")      o.camera_tracks[1] = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-track-2")      o.camera_tracks[2] = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-track-3")      o.camera_tracks[3] = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-serial-0")     o.camera_serials[0] = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-serial-1")     o.camera_serials[1] = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-serial-2")     o.camera_serials[2] = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-serial-3")     o.camera_serials[3] = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-width")        o.camera_width = ta::parse_int(ta::require_value(a, argc, argv, i), a);
        else if (a == "--camera-height")       o.camera_height = ta::parse_int(ta::require_value(a, argc, argv, i), a);
        else if (a == "--camera-resolution")   o.camera_resolution_str = ta::require_value(a, argc, argv, i);
        else if (a == "--camera-fps")          o.camera_fps = ta::parse_int(ta::require_value(a, argc, argv, i), a);
        else if (a == "--camera-bitrate-kbps") o.camera_bitrate_kbps = ta::parse_int(ta::require_value(a, argc, argv, i), a);
        else if (a == "--help" || a == "-h")   { usage(argv[0]); std::exit(0); }
        else throw std::runtime_error("unknown option: " + a);
    }
    o.api_key           = ta::require_cli_or_env("--api-key", "ADAMO_API_KEY", o.api_key);
    o.robot             = ta::cli_env_or_default(o.robot,             "ADAMO_ROBOT_NAME",                kDefaultRobot);
    o.left_follower_ip  = ta::cli_env_or_default(o.left_follower_ip,  "ADAMO_TROSSEN_LEFT_FOLLOWER_IP",  kDefaultLeftFollowerIp);
    o.right_follower_ip = ta::cli_env_or_default(o.right_follower_ip, "ADAMO_TROSSEN_RIGHT_FOLLOWER_IP", kDefaultRightFollowerIp);
    if (o.rate_hz <= 0.0) throw std::runtime_error("--rate-hz must be positive");
    parse_follower_model(o.left_model_str);   // validate early
    parse_follower_model(o.right_model_str);  // validate early
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
#if defined(ADAMO_TROSSEN_HAS_REALSENSE) && defined(ADAMO_TROSSEN_HAS_ZED)
        if (o.camera_backend != "realsense" && o.camera_backend != "zed") {
            throw std::runtime_error("--camera-backend must be realsense or zed");
        }
#elif defined(ADAMO_TROSSEN_HAS_REALSENSE)
        if (o.camera_backend != "realsense") {
            throw std::runtime_error("--camera-backend must be realsense (this build was compiled without ZED support)");
        }
#elif defined(ADAMO_TROSSEN_HAS_ZED)
        if (o.camera_backend != "zed") {
            throw std::runtime_error("--camera-backend must be zed (this build was compiled without RealSense support)");
        }
#else
        throw std::runtime_error("camera requested but this build has no camera backend compiled in; pass --no-camera");
#endif
        const int max_cam = (o.camera_backend == "zed") ? kMaxCamerasZed : kMaxCamerasRealSense;
        if (o.num_cameras < 1 || o.num_cameras > max_cam) {
            throw std::runtime_error("--num-cameras must be 1-" + std::to_string(max_cam) +
                                     " for backend '" + o.camera_backend + "'");
        }
        if (o.camera_fps <= 0 || o.camera_bitrate_kbps <= 0) {
            throw std::runtime_error("camera fps/bitrate must be positive");
        }
        if (o.camera_backend == "realsense" && (o.camera_width <= 0 || o.camera_height <= 0)) {
            throw std::runtime_error("camera width/height must be positive");
        }
#ifdef ADAMO_TROSSEN_HAS_ZED
        if (o.camera_backend == "zed") {
            parse_zed_resolution(o.camera_resolution_str);  // validate early
        }
#endif
    }
    return o;
}

// Per-arm smoothing state used inside the teleop loop.
struct ArmSmoothState {
    bool synced = false;
    std::vector<double> last_command;
    std::vector<double> command_buf = std::vector<double>(trossen_adamo::wire::kNumJoints, 0.0);
    std::vector<double> latencies_ms;
    std::optional<std::chrono::steady_clock::time_point> next_stats;
    std::vector<double> sync_start;          // arm position when the current sync ramp began
    std::optional<std::chrono::steady_clock::time_point> sync_started_at;
};

// Process a decoded state sample for one arm, commanding the driver and updating
// smoothing state. Returns true if a command was issued.
bool process_arm_state(const trossen_adamo::wire::State& s,
                       double teleop_started_at,
                       trossen_arm::TrossenArmDriver& driver,
                       const Options& opt,
                       ArmSmoothState& ss,
                       const char* label) {
    if (s.timestamp < teleop_started_at) return false;

    const double latency_ms = std::max(0.0, (trossen_adamo::wire::now_seconds() - s.timestamp) * 1000.0);
    ss.latencies_ms.push_back(latency_ms);

    if (!ss.synced) {
        // Non-blocking ramp toward the leader's live pose (re-sampled each
        // tick) instead of one blocking move to a stale snapshot -- avoids
        // a jump when steady-state tracking takes over.
        if (!ss.sync_started_at) {
            ss.sync_start = driver.get_all_positions();
            ss.sync_started_at = std::chrono::steady_clock::now();
            std::cout << label << ": syncing to leader pose over "
                      << opt.initial_sync_time << "s\n";
        }
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - *ss.sync_started_at).count();
        const double frac = (opt.initial_sync_time > 0.0)
            ? std::clamp(elapsed / opt.initial_sync_time, 0.0, 1.0)
            : 1.0;
        for (std::size_t i = 0; i < s.positions.size(); ++i) {
            ss.command_buf[i] = ss.sync_start[i] + frac * (s.positions[i] - ss.sync_start[i]);
        }
        driver.set_all_positions(ss.command_buf, opt.command_time, false, s.velocities);
        ss.last_command = ss.command_buf;
        if (frac >= 1.0) {
            ss.synced = true;
            ss.sync_started_at.reset();
        }
    } else {
        for (std::size_t i = 0; i < s.positions.size(); ++i) {
            const double prev = ss.last_command[i];
            double value = prev + opt.smooth_alpha * (s.positions[i] - prev);
            if (opt.max_step) {
                const double max = *opt.max_step;
                value = std::clamp(value, prev - max, prev + max);
            }
            ss.command_buf[i] = value;
        }
        driver.set_all_positions(ss.command_buf, opt.command_time, false, s.velocities);
        ss.last_command = ss.command_buf;
    }
    return true;
}

// Print latency stats if the interval has elapsed.
void maybe_print_stats(ArmSmoothState& ss, const Options& opt, const char* label) {
    if (!ss.next_stats || std::chrono::steady_clock::now() < *ss.next_stats) return;
    if (!ss.latencies_ms.empty()) {
        std::sort(ss.latencies_ms.begin(), ss.latencies_ms.end());
        const std::size_t n = ss.latencies_ms.size();
        const double p50  = ss.latencies_ms[n / 2];
        const double p95  = (n >= 20)
            ? ss.latencies_ms[static_cast<std::size_t>(n * 0.95) - 1]
            : ss.latencies_ms.back();
        const double pmax = ss.latencies_ms.back();
        std::fprintf(stderr,
            "%s stats: samples=%zu latency_ms p50=%.1f p95=%.1f max=%.1f\n",
            label, n, p50, p95, pmax);
        ss.latencies_ms.clear();
    }
    *ss.next_stats = std::chrono::steady_clock::now() +
                     std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(opt.stats_interval_s));
}

}  // namespace

int main(int argc, char** argv) try {
    namespace ta = trossen_adamo;
    ta::install_signal_handlers();

    const Options opt = parse(argc, argv);
    const adamo::Protocol protocol = ta::args::parse_protocol(opt.protocol_str);

    // Configure both arms.
    std::cout << "bimanual_follower: configuring left arm at "  << opt.left_follower_ip  << "\n";
    const auto left_model_cfg  = parse_follower_model(opt.left_model_str);
    auto left_driver = ta::arm::configure(opt.left_follower_ip,
                                          left_model_cfg.end_effector,
                                          opt.clear_error,
                                          opt.connect_timeout,
                                          left_model_cfg.model);

    std::cout << "bimanual_follower: configuring right arm at " << opt.right_follower_ip << "\n";
    const auto right_model_cfg = parse_follower_model(opt.right_model_str);
    auto right_driver = ta::arm::configure(opt.right_follower_ip,
                                           right_model_cfg.end_effector,
                                           opt.clear_error,
                                           opt.connect_timeout,
                                           right_model_cfg.model);

    // Widen the arm joints' velocity/effort fault tolerance to their max.
    {
        auto left_joint_limits = left_driver->get_joint_limits();
        for (int i = 0; i < 6; ++i) {
            left_joint_limits[i].velocity_tolerance = left_joint_limits[i].velocity_max;
            left_joint_limits[i].effort_tolerance = left_joint_limits[i].effort_max;
        }
        left_driver->set_joint_limits(left_joint_limits);

        auto right_joint_limits = right_driver->get_joint_limits();
        for (int i = 0; i < 6; ++i) {
            right_joint_limits[i].velocity_tolerance = right_joint_limits[i].velocity_max;
            right_joint_limits[i].effort_tolerance = right_joint_limits[i].effort_max;
        }
        right_driver->set_joint_limits(right_joint_limits);
    }

    // Guards each side's driver calls against ErrorState faults
    // independently, so a fault on one side never affects the other.
    ta::recovery::ArmFaultTracker fault_tracker_left("bimanual_follower_left");
    ta::recovery::ArmFaultTracker fault_tracker_right("bimanual_follower_right");

    // Park guards: fire on every exit path from here on.
    ta::arm::ArmParkGuard left_park(*left_driver,   "bimanual_follower_left");
    ta::arm::ArmParkGuard right_park(*right_driver, "bimanual_follower_right");

    // Start camera streamers before opening the Adamo session so the operator's
    // video link is up by the time teleop begins. Each camera runs its own
    // dedicated streamer with a distinct track name. Each vector only exists
    // when its backend was compiled in (ADAMO_TROSSEN_HAS_REALSENSE/_ZED, see
    // bimanual_follower/CMakeLists.txt).
#ifdef ADAMO_TROSSEN_HAS_REALSENSE
    std::vector<std::unique_ptr<ta::camera::RealSenseStreamer>> rs_streamers;
#endif
#ifdef ADAMO_TROSSEN_HAS_ZED
    std::vector<std::unique_ptr<ta::camera::ZedStreamer>>       zed_streamers;
#endif

    if (opt.camera_enabled) {
#ifdef ADAMO_HAS_VIDEO
        bool started = false;
#ifdef ADAMO_TROSSEN_HAS_REALSENSE
        if (!started && opt.camera_backend == "realsense") {
            for (int idx = 0; idx < opt.num_cameras; ++idx) {
                ta::camera::Config c;
                c.api_key      = opt.api_key;
                c.robot        = opt.robot;
                c.track        = opt.camera_tracks[static_cast<std::size_t>(idx)];
                c.serial       = opt.camera_serials[static_cast<std::size_t>(idx)];
                c.width        = opt.camera_width;
                c.height       = opt.camera_height;
                c.fps          = opt.camera_fps;
                c.bitrate_kbps = opt.camera_bitrate_kbps;
                c.protocol     = protocol;
                auto streamer = std::make_unique<ta::camera::RealSenseStreamer>(std::move(c));
                streamer->start();
                rs_streamers.push_back(std::move(streamer));
            }
            started = true;
        }
#endif
#ifdef ADAMO_TROSSEN_HAS_ZED
        if (!started && opt.camera_backend == "zed") {
            for (int idx = 0; idx < opt.num_cameras; ++idx) {
                ta::camera::ZedConfig c;
                c.api_key      = opt.api_key;
                c.robot        = opt.robot;
                c.track        = opt.camera_tracks[static_cast<std::size_t>(idx)];
                c.serial       = opt.camera_serials[static_cast<std::size_t>(idx)];
                c.resolution   = parse_zed_resolution(opt.camera_resolution_str);
                c.fps          = opt.camera_fps;
                c.bitrate_kbps = opt.camera_bitrate_kbps;
                c.protocol     = protocol;
                auto streamer = std::make_unique<ta::camera::ZedStreamer>(std::move(c));
                streamer->start();
                zed_streamers.push_back(std::move(streamer));
            }
            started = true;
        }
#endif
        if (!started) {
            std::fprintf(stderr,
                "bimanual_follower: --camera-backend '%s' is not supported by this build\n",
                opt.camera_backend.c_str());
            return 1;
        }
#else
        std::fprintf(stderr,
            "bimanual_follower: camera requested but Adamo was built without ADAMO_BUILD_VIDEO; "
            "rebuild with -DADAMO_BUILD_VIDEO=ON or pass --no-camera\n");
        return 1;
#endif
    }

    std::cout << "bimanual_follower: opening Adamo session (" << opt.protocol_str << ")\n";
    auto session = adamo::Session::open(opt.api_key, protocol);

    const auto state_left_topic      = ta::topics::state_left_of(opt.robot);
    const auto state_right_topic     = ta::topics::state_right_of(opt.robot);
    const auto effort_left_topic     = ta::topics::effort_left_of(opt.robot);
    const auto effort_right_topic    = ta::topics::effort_right_of(opt.robot);
    const auto leader_ready_topic    = ta::topics::leader_ready_of(opt.robot);
    const auto follower_ready_topic  = ta::topics::follower_ready_of(opt.robot);
    const auto teleop_toggle_left_topic  = ta::topics::teleop_toggle_left_of(opt.robot);
    const auto teleop_toggle_right_topic = ta::topics::teleop_toggle_right_of(opt.robot);
    const auto error_recover_left_topic  = ta::topics::error_recover_left_of(opt.robot);
    const auto error_recover_right_topic = ta::topics::error_recover_right_of(opt.robot);

    // Leader state subscribers run on the SDK's receive thread.
    ta::LatestSubscriber state_left_sub(session,  state_left_topic);
    ta::LatestSubscriber state_right_sub(session, state_right_topic);
    ta::LatestSubscriber teleop_toggle_left_sub(session,  teleop_toggle_left_topic);
    ta::LatestSubscriber teleop_toggle_right_sub(session, teleop_toggle_right_topic);
    ta::LatestSubscriber error_recover_left_sub(session,  error_recover_left_topic);
    ta::LatestSubscriber error_recover_right_sub(session, error_recover_right_topic);
    auto ready_sub = session.subscribe(leader_ready_topic);

    std::cout << "bimanual_follower: moving arms to home\n";
    ta::arm::move_home(*left_driver);
    ta::arm::move_home(*right_driver);

    auto ready_pub        = session.publisher(follower_ready_topic, 250, true, false);
    auto effort_left_pub  = session.publisher(effort_left_topic,    250, true, false);
    auto effort_right_pub = session.publisher(effort_right_topic,   250, true, false);
    ta::LatestPublisher effort_left_latest(std::move(effort_left_pub));
    ta::LatestPublisher effort_right_latest(std::move(effort_right_pub));

    ta::handshake::wait_for_peer_ready(ready_pub, ready_sub, opt.ready_timeout, "leader");

    std::cout << "bimanual_follower: starting teleop\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    left_driver->set_all_modes(trossen_arm::Mode::position);
    right_driver->set_all_modes(trossen_arm::Mode::position);

    const double teleop_started_at = ta::wire::now_seconds();
    const auto loop_end = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(opt.teleoperation_time);

    ArmSmoothState left_ss;
    ArmSmoothState right_ss;
    std::vector<std::uint8_t> state_left_buf;
    std::vector<std::uint8_t> state_right_buf;

    const auto make_next_stats = [&]() -> std::optional<std::chrono::steady_clock::time_point> {
        if (opt.stats_interval_s <= 0.0) return std::nullopt;
        return std::chrono::steady_clock::now() +
               std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                   std::chrono::duration<double>(opt.stats_interval_s));
    };
    left_ss.next_stats  = make_next_stats();
    right_ss.next_stats = make_next_stats();

    // --button-gated state per side: a Glide leader's SEL_1 starts/stops
    // that side's teleop (stopping moves that arm home), SEL_2 clears that
    // side's fault and resumes. When not gated, both sides stay Active the
    // whole run (today's behaviour, unchanged) and each side's fault
    // tracker is bypassed entirely so an uncaught driver exception still
    // propagates as before.
    enum class TeleopState { Stopped, Active };
    TeleopState state_left  = opt.button_gated ? TeleopState::Stopped : TeleopState::Active;
    TeleopState state_right = opt.button_gated ? TeleopState::Stopped : TeleopState::Active;
    std::vector<std::uint8_t> teleop_toggle_left_buf, teleop_toggle_right_buf;
    std::vector<std::uint8_t> error_recover_left_buf, error_recover_right_buf;

    // Wraps a driver call: guarded (caught, fault-tracked) when
    // --button-gated, called directly (exceptions propagate as before)
    // otherwise. One per side since each has its own fault tracker.
    auto maybe_guard_left = [&](auto&& fn) {
        if (opt.button_gated) return fault_tracker_left.guard(std::forward<decltype(fn)>(fn));
        fn();
        return true;
    };
    auto maybe_guard_right = [&](auto&& fn) {
        if (opt.button_gated) return fault_tracker_right.guard(std::forward<decltype(fn)>(fn));
        fn();
        return true;
    };

    if (opt.button_gated) {
        std::cout << "bimanual_follower: --button-gated: waiting for each leader's SEL_1 button to start teleop\n";
    }

    while (!ta::stop_requested() && std::chrono::steady_clock::now() < loop_end) {
        const auto loop_start = std::chrono::steady_clock::now();

        // Publish latest efforts for both arms; guarded only in --button-gated mode.
        maybe_guard_left([&] {
            const auto efforts = left_driver->get_all_external_efforts();
            const auto payload = ta::wire::encode_efforts(ta::wire::now_seconds(), efforts);
            effort_left_latest.put(payload.data(), payload.size());
        });
        maybe_guard_right([&] {
            const auto efforts = right_driver->get_all_external_efforts();
            const auto payload = ta::wire::encode_efforts(ta::wire::now_seconds(), efforts);
            effort_right_latest.put(payload.data(), payload.size());
        });

        if (opt.button_gated) {
            // Left side: SEL_1 start/stop teleop, SEL_2 error recovery.
            if (teleop_toggle_left_sub.poll(teleop_toggle_left_buf)) {
                double ts = 0.0;
                if (ta::wire::decode_ready(teleop_toggle_left_buf.data(), teleop_toggle_left_buf.size(), &ts) &&
                    ts >= teleop_started_at) {
                    if (fault_tracker_left.faulted()) {
                        std::cout << "bimanual_follower: ignoring left start/stop — faulted; press the left error-recovery button first\n";
                    } else if (state_left == TeleopState::Stopped) {
                        state_left = TeleopState::Active;
                        left_ss.synced = false;
                        left_ss.sync_started_at.reset();
                        std::cout << "bimanual_follower: left teleop started (leader button)\n";
                    } else {
                        maybe_guard_left([&] { ta::arm::move_home(*left_driver); });
                        state_left = TeleopState::Stopped;
                        std::cout << "bimanual_follower: left teleop stopped (leader button), moved home\n";
                    }
                }
            }
            if (error_recover_left_sub.poll(error_recover_left_buf)) {
                double ts = 0.0;
                if (ta::wire::decode_ready(error_recover_left_buf.data(), error_recover_left_buf.size(), &ts) &&
                    ts >= teleop_started_at) {
                    if (!fault_tracker_left.faulted()) {
                        std::cout << "bimanual_follower: left error-recovery button pressed but there is no active fault\n";
                    } else if (fault_tracker_left.try_clear(*left_driver)) {
                        left_driver->set_all_modes(trossen_arm::Mode::position);
                        state_left = TeleopState::Active;
                        left_ss.synced = false;
                        left_ss.sync_started_at.reset();
                        std::cout << "bimanual_follower: left fault cleared (leader button), resuming teleop\n";
                    }
                }
            }

            // Right side: SEL_1 start/stop teleop, SEL_2 error recovery.
            if (teleop_toggle_right_sub.poll(teleop_toggle_right_buf)) {
                double ts = 0.0;
                if (ta::wire::decode_ready(teleop_toggle_right_buf.data(), teleop_toggle_right_buf.size(), &ts) &&
                    ts >= teleop_started_at) {
                    if (fault_tracker_right.faulted()) {
                        std::cout << "bimanual_follower: ignoring right start/stop — faulted; press the right error-recovery button first\n";
                    } else if (state_right == TeleopState::Stopped) {
                        state_right = TeleopState::Active;
                        right_ss.synced = false;
                        right_ss.sync_started_at.reset();
                        std::cout << "bimanual_follower: right teleop started (leader button)\n";
                    } else {
                        maybe_guard_right([&] { ta::arm::move_home(*right_driver); });
                        state_right = TeleopState::Stopped;
                        std::cout << "bimanual_follower: right teleop stopped (leader button), moved home\n";
                    }
                }
            }
            if (error_recover_right_sub.poll(error_recover_right_buf)) {
                double ts = 0.0;
                if (ta::wire::decode_ready(error_recover_right_buf.data(), error_recover_right_buf.size(), &ts) &&
                    ts >= teleop_started_at) {
                    if (!fault_tracker_right.faulted()) {
                        std::cout << "bimanual_follower: right error-recovery button pressed but there is no active fault\n";
                    } else if (fault_tracker_right.try_clear(*right_driver)) {
                        right_driver->set_all_modes(trossen_arm::Mode::position);
                        state_right = TeleopState::Active;
                        right_ss.synced = false;
                        right_ss.sync_started_at.reset();
                        std::cout << "bimanual_follower: right fault cleared (leader button), resuming teleop\n";
                    }
                }
            }
        }

        // Process left arm state.
        if (state_left == TeleopState::Active && state_left_sub.poll(state_left_buf)) {
            try {
                const auto s = ta::wire::decode_state(state_left_buf.data(), state_left_buf.size());
                maybe_guard_left([&] {
                    process_arm_state(s, teleop_started_at, *left_driver, opt, left_ss,
                                      "bimanual_follower_left");
                });
            } catch (const std::exception& e) {
                std::fprintf(stderr, "bimanual_follower: bad left state payload: %s\n", e.what());
            }
        }

        // Process right arm state.
        if (state_right == TeleopState::Active && state_right_sub.poll(state_right_buf)) {
            try {
                const auto s = ta::wire::decode_state(state_right_buf.data(), state_right_buf.size());
                maybe_guard_right([&] {
                    process_arm_state(s, teleop_started_at, *right_driver, opt, right_ss,
                                      "bimanual_follower_right");
                });
            } catch (const std::exception& e) {
                std::fprintf(stderr, "bimanual_follower: bad right state payload: %s\n", e.what());
            }
        }

        maybe_print_stats(left_ss,  opt, "bimanual_follower_left");
        maybe_print_stats(right_ss, opt, "bimanual_follower_right");

        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - loop_start).count();
        if (elapsed_ms > opt.stall_log_ms) {
            std::fprintf(stderr, "bimanual_follower loop stall: %.1fms\n", elapsed_ms);
        }
        ta::sleep_until_next_tick(loop_start, opt.rate_hz);
    }

    effort_left_latest.close();
    effort_right_latest.close();

    std::cout << "bimanual_follower: returning home + sleep\n";
#ifdef ADAMO_TROSSEN_HAS_REALSENSE
    for (auto& s : rs_streamers)  s->stop();
#endif
#ifdef ADAMO_TROSSEN_HAS_ZED
    for (auto& s : zed_streamers) s->stop();
#endif
    // Park guards run here as we return: position mode, home, sleep.
    return 0;
} catch (const std::exception& e) {
    std::fprintf(stderr, "bimanual_follower error: %s\n", e.what());
    return 1;
}
