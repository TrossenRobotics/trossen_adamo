// RealSense → Adamo video streamer encapsulated as a class. Owns its own
// `adamo::Robot` because video tracks attach to a Robot, while the teleop
// pubsub uses an `adamo::Session`.

#pragma once

#include "adamo/adamo.hpp"
#include "trossen_adamo/signal.hpp"

#include <librealsense2/rs.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace trossen_adamo::camera {

struct Config {
    std::string api_key;
    std::string robot;
    std::string track = "main";
    std::string serial;        // empty = first available
    int width = 640;
    int height = 480;
    int fps = 30;
    int bitrate_kbps = 4000;
    adamo::Protocol protocol = adamo::Protocol::Quic;
};

class RealSenseStreamer {
public:
    explicit RealSenseStreamer(Config cfg) : cfg_(std::move(cfg)) {}

    RealSenseStreamer(const RealSenseStreamer&)            = delete;
    RealSenseStreamer& operator=(const RealSenseStreamer&) = delete;

    ~RealSenseStreamer() { stop(); }

    void start() {
#ifndef ADAMO_HAS_VIDEO
        throw std::runtime_error("RealSense streamer requires Adamo built with ADAMO_BUILD_VIDEO=ON");
#else
        rs2::config rs_cfg;
        if (!cfg_.serial.empty()) rs_cfg.enable_device(cfg_.serial);
        rs_cfg.enable_stream(RS2_STREAM_COLOR, cfg_.width, cfg_.height, RS2_FORMAT_RGB8, cfg_.fps);

        auto profile = pipe_.start(rs_cfg);
        const auto color_profile =
            profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        const int width  = color_profile.width();
        const int height = color_profile.height();
        const int fps    = color_profile.fps();

        std::fprintf(stderr,
            "camera: opening Adamo robot '%s' for video track '%s' (%dx%d@%d, %d kbps)\n",
            cfg_.robot.c_str(), cfg_.track.c_str(), width, height, fps, cfg_.bitrate_kbps);

        auto robot = adamo::Robot::create(cfg_.api_key, cfg_.robot, cfg_.protocol);
        track_ = robot.video(
            cfg_.track,
            static_cast<std::uint32_t>(width),
            static_cast<std::uint32_t>(height),
            "BGRA",
            static_cast<std::uint32_t>(fps),
            static_cast<std::uint32_t>(cfg_.bitrate_kbps));

        // The Adamo robot run loop is blocking; drive it from a dedicated thread.
        //
        // The captures are deliberate. stop() detaches this thread, so it can
        // outlive the streamer: capturing `this` (or cfg_) would dangle. The
        // stop flag is a shared_ptr precisely so the thread can still set it
        // after the object is gone, and the track name is copied for the same
        // reason.
        adamo_thread_ = std::thread([r = std::move(robot),
                                     stop = local_stop_,
                                     track = cfg_.track]() mutable {
            std::fprintf(stderr, "camera: track '%s' starting Adamo robot run loop\n",
                         track.c_str());
            const int rc = std::move(r).run();
            // Stop THIS camera only. This used to set the process-wide stop
            // flag -- the same one Ctrl-C sets -- so one camera's session
            // dropping shut down arm teleop and every other camera with it,
            // indistinguishable from an operator interrupt.
            std::fprintf(stderr,
                "camera: track '%s' Adamo run loop exited with %d; stopping this camera, "
                "the rest of the robot keeps running\n", track.c_str(), rc);
            stop->store(true);
        });

        // Give the robot a moment to spin up before frames start flowing.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        capture_thread_ = std::thread([this, width, height] { capture_loop(width, height); });
        running_ = true;
#endif
    }

    void stop() noexcept {
#ifdef ADAMO_HAS_VIDEO
        // Not an early return on `!was_running`: start() can throw after the
        // run-loop thread exists, and the caller now catches that and destroys
        // the streamer. Leaving a joinable std::thread member for the
        // destructor would call std::terminate. Idempotent, so the usual
        // stop()-then-destructor path is still fine.
        const bool was_running = running_.exchange(false);
        local_stop_->store(true);
        if (was_running) { try { pipe_.stop(); } catch (...) {} }
        if (capture_thread_.joinable()) capture_thread_.join();
        track_ = adamo::VideoTrack{};
        // Detach the run-loop thread so it doesn't block process exit.
        if (adamo_thread_.joinable()) adamo_thread_.detach();
#endif
    }

private:
#ifdef ADAMO_HAS_VIDEO
    void capture_loop(int width, int height) {
        std::vector<std::uint8_t> bgra(static_cast<std::size_t>(width) * height * 4);
        std::uint64_t frames = 0;
        auto last_report = std::chrono::steady_clock::now();
        while (!local_stop_->load() && !stop_requested()) {
            rs2::frameset fs;
            if (!pipe_.poll_for_frames(&fs)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            rs2::video_frame color = fs.get_color_frame();
            if (!color) continue;
            rgb_to_bgra(color, bgra);
            try {
                track_.send(bgra.data(), bgra.size());
            } catch (const std::exception& e) {
                // Terminal for this camera: the loop exits and does not retry.
                // Named so it is attributable with several cameras running.
                std::fprintf(stderr,
                    "camera: track '%s' send failed, this camera is going dark "
                    "(no retry): %s\n", cfg_.track.c_str(), e.what());
                local_stop_->store(true);
                break;
            }
            ++frames;
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_report);
            if (elapsed.count() >= 5) {
                const double rate = static_cast<double>(frames) / static_cast<double>(elapsed.count());
                std::fprintf(stderr, "camera: send rate %.1f fps\n", rate);
                frames = 0;
                last_report = now;
            }
        }
    }

    static void rgb_to_bgra(const rs2::video_frame& frame, std::vector<std::uint8_t>& bgra) {
        const auto* rgb = static_cast<const std::uint8_t*>(frame.get_data());
        for (std::size_t i = 0, j = 0; i < bgra.size(); i += 4, j += 3) {
            bgra[i + 0] = rgb[j + 2];
            bgra[i + 1] = rgb[j + 1];
            bgra[i + 2] = rgb[j + 0];
            bgra[i + 3] = 255;
        }
    }

    rs2::pipeline pipe_;
    adamo::VideoTrack track_;
    std::thread adamo_thread_;
    std::thread capture_thread_;
    // shared_ptr, not a plain member: the detached Adamo run-loop thread sets
    // this and may outlive the streamer.
    std::shared_ptr<std::atomic<bool>> local_stop_ = std::make_shared<std::atomic<bool>>(false);
    std::atomic<bool> running_{false};
#endif

    Config cfg_;
};

}  // namespace trossen_adamo::camera
