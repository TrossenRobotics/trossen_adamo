// Owns its own `adamo::Robot` because video tracks
// attach to a Robot, while the teleop pubsub uses an `adamo::Session`.

#pragma once

#include "adamo/adamo.hpp"
#include "trossen_adamo/signal.hpp"

#include <sl/Camera.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace trossen_adamo::camera {

struct ZedConfig {
    std::string api_key;
    std::string robot;
    std::string track = "main";
    std::string serial;                                   // empty = first available
    sl::RESOLUTION resolution = sl::RESOLUTION::HD1200;    // ZED X (Nano) native resolution
    int fps = 30;
    int bitrate_kbps = 4000;
    adamo::Protocol protocol = adamo::Protocol::Quic;
};

class ZedStreamer {
public:
    explicit ZedStreamer(ZedConfig cfg) : cfg_(std::move(cfg)) {}

    ZedStreamer(const ZedStreamer&)            = delete;
    ZedStreamer& operator=(const ZedStreamer&) = delete;

    ~ZedStreamer() { stop(); }

    void start() {
#ifndef ADAMO_HAS_VIDEO
        throw std::runtime_error("ZED streamer requires Adamo built with ADAMO_BUILD_VIDEO=ON");
#else
        sl::InitParameters init_params;
        init_params.camera_resolution = cfg_.resolution;
        init_params.camera_fps = cfg_.fps;
        init_params.depth_mode = sl::DEPTH_MODE::NONE;  // color-only; no depth/point-cloud needed
        if (!cfg_.serial.empty()) {
            init_params.input.setFromSerialNumber(
                static_cast<unsigned int>(std::stoul(cfg_.serial)));
        }

        const sl::ERROR_CODE err = zed_.open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            std::ostringstream oss;
            oss << "ZED camera open failed: " << err;
            throw std::runtime_error(oss.str());
        }

        const auto& config = zed_.getCameraInformation().camera_configuration;
        const int width  = static_cast<int>(config.resolution.width);
        const int height = static_cast<int>(config.resolution.height);
        const int fps    = static_cast<int>(config.fps);

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
        adamo_thread_ = std::thread([r = std::move(robot)]() mutable {
            std::fprintf(stderr, "camera: starting Adamo robot run loop\n");
            const int rc = std::move(r).run();
            std::fprintf(stderr, "camera: Adamo robot run loop exited with %d\n", rc);
            stop_flag().store(true);
        });

        // Give the robot a moment to spin up before frames start flowing.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        capture_thread_ = std::thread([this, width, height] { capture_loop(width, height); });
        running_ = true;
#endif
    }

    void stop() noexcept {
#ifdef ADAMO_HAS_VIDEO
        if (!running_.exchange(false)) return;
        local_stop_.store(true);
        if (capture_thread_.joinable()) capture_thread_.join();
        zed_.close();
        track_ = adamo::VideoTrack{};
        // Detach the run-loop thread so it doesn't block process exit.
        if (adamo_thread_.joinable()) adamo_thread_.detach();
#endif
    }

private:
#ifdef ADAMO_HAS_VIDEO
    void capture_loop(int width, int height) {
        sl::Mat image;
        const std::size_t row_bytes = static_cast<std::size_t>(width) * 4;
        std::vector<std::uint8_t> scratch;  // only populated if rows are padded
        std::uint64_t frames = 0;
        auto last_report = std::chrono::steady_clock::now();
        while (!local_stop_.load() && !stop_requested()) {
            if (zed_.grab() != sl::ERROR_CODE::SUCCESS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            zed_.retrieveImage(image, sl::VIEW::LEFT, sl::MEM::CPU);

            const std::size_t step = image.getStepBytes(sl::MEM::CPU);
            auto* base = reinterpret_cast<const std::uint8_t*>(image.getPtr<sl::uchar1>(sl::MEM::CPU));

            const std::uint8_t* send_ptr = base;
            std::size_t send_len = row_bytes * static_cast<std::size_t>(height);
            if (step != row_bytes) {
                scratch.resize(send_len);
                for (int y = 0; y < height; ++y) {
                    std::memcpy(scratch.data() + static_cast<std::size_t>(y) * row_bytes,
                                base + static_cast<std::size_t>(y) * step,
                                row_bytes);
                }
                send_ptr = scratch.data();
            }

            try {
                track_.send(send_ptr, send_len);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "camera: send failed: %s\n", e.what());
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

    sl::Camera zed_;
    adamo::VideoTrack track_;
    std::thread adamo_thread_;
    std::thread capture_thread_;
    std::atomic<bool> local_stop_{false};
    std::atomic<bool> running_{false};
#endif

    ZedConfig cfg_;
};

}  // namespace trossen_adamo::camera
