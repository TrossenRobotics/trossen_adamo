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
    std::string serial;                          // empty = first available
    // Output resolution streamed to Adamo.  Camera always opens at HD1200
    // internally (CTI/Rogue Jetson board constraint); this is used only for downsampling.
    sl::RESOLUTION resolution = sl::RESOLUTION::SVGA;
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
        // Always open at native HD1200: CTI/Rogue Jetson board rigs only accept the sensor's
        // native resolution from nvargus-daemon.  Any other value causes SIGABRT.
        // Downsampling to cfg_.resolution happens in retrieveImage() below,
        // exactly as nvvidconv does in the GStreamer capture pipeline.
        init_params.camera_resolution = sl::RESOLUTION::HD1200;
        init_params.camera_fps = cfg_.fps;
        init_params.depth_mode = sl::DEPTH_MODE::NONE;
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

        const auto& cap_cfg = zed_.getCameraInformation().camera_configuration;
        const int fps = static_cast<int>(cap_cfg.fps);

        // Output resolution the user requested; downsampled from HD1200.
        const sl::Resolution out_res = resolution_to_dims(cfg_.resolution);
        const int out_w = static_cast<int>(out_res.width);
        const int out_h = static_cast<int>(out_res.height);

        std::fprintf(stderr,
            "camera: capture 1920x1200 -> output %dx%d @%d fps, %d kbps (robot '%s' track '%s')\n",
            out_w, out_h, fps, cfg_.bitrate_kbps, cfg_.robot.c_str(), cfg_.track.c_str());

        auto robot = adamo::Robot::create(cfg_.api_key, cfg_.robot, cfg_.protocol);
        track_ = robot.video(
            cfg_.track,
            static_cast<std::uint32_t>(out_w),
            static_cast<std::uint32_t>(out_h),
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

        capture_thread_ = std::thread([this, out_w, out_h] { capture_loop(out_w, out_h); });
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
    void capture_loop(int out_w, int out_h) {
        sl::Mat image;
        const std::size_t row_bytes = static_cast<std::size_t>(out_w) * 4;
        // Pass output size to retrieveImage; ZED SDK bilinearly resizes from
        // the HD1200 capture frame — equivalent to the nvvidconv step.
        const sl::Resolution out_res(static_cast<std::size_t>(out_w),
                                     static_cast<std::size_t>(out_h));
        std::vector<std::uint8_t> scratch;  // only populated if rows are padded
        std::uint64_t frames = 0;
        auto last_report = std::chrono::steady_clock::now();
        while (!local_stop_.load() && !stop_requested()) {
            if (zed_.grab() != sl::ERROR_CODE::SUCCESS) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            zed_.retrieveImage(image, sl::VIEW::LEFT, sl::MEM::CPU, out_res);

            const std::size_t step = image.getStepBytes(sl::MEM::CPU);
            auto* base = reinterpret_cast<const std::uint8_t*>(image.getPtr<sl::uchar1>(sl::MEM::CPU));

            const std::uint8_t* send_ptr = base;
            std::size_t send_len = row_bytes * static_cast<std::size_t>(out_h);
            if (step != row_bytes) {
                scratch.resize(send_len);
                for (int y = 0; y < out_h; ++y) {
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

    // Convert sl::RESOLUTION enum to pixel dimensions for retrieveImage().
    static sl::Resolution resolution_to_dims(sl::RESOLUTION r) {
        switch (r) {
            case sl::RESOLUTION::HD2K:   return sl::Resolution(2208, 1242);
            case sl::RESOLUTION::HD1200: return sl::Resolution(1920, 1200);
            case sl::RESOLUTION::HD1080: return sl::Resolution(1920, 1080);
            case sl::RESOLUTION::HD720:  return sl::Resolution(1280,  720);
            case sl::RESOLUTION::SVGA:   return sl::Resolution( 960,  600);
            case sl::RESOLUTION::VGA:    return sl::Resolution( 672,  376);
            default:                     return sl::Resolution( 960,  600);
        }
    }
#endif

    ZedConfig cfg_;
};

}  // namespace trossen_adamo::camera
