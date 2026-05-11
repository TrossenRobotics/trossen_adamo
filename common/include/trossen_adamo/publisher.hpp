// LatestPublisher decouples the teleop control loop from publish latency by
// keeping a single most-recent payload slot. A worker thread sends whatever
// is in the slot and clears it; producers always overwrite, so a slow
// publish never stalls the 100 Hz loop.

#pragma once

#include "adamo/adamo.hpp"

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace trossen_adamo {

class LatestPublisher {
public:
    explicit LatestPublisher(adamo::Publisher pub) : pub_(std::move(pub)) {
        worker_ = std::thread([this] { run(); });
    }

    LatestPublisher(const LatestPublisher&)            = delete;
    LatestPublisher& operator=(const LatestPublisher&) = delete;

    ~LatestPublisher() { close(); }

    void put(const std::uint8_t* data, std::size_t len) {
        {
            std::lock_guard<std::mutex> lk(mu_);
            pending_.assign(data, data + len);
            has_pending_ = true;
        }
        cv_.notify_one();
    }

    void close() noexcept {
        {
            std::lock_guard<std::mutex> lk(mu_);
            stop_ = true;
        }
        cv_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

private:
    void run() {
        std::vector<std::uint8_t> buf;
        while (true) {
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [&]{ return stop_ || has_pending_; });
                if (stop_ && !has_pending_) return;
                buf.swap(pending_);
                has_pending_ = false;
            }
            try {
                pub_.put(buf.data(), buf.size());
            } catch (const std::exception& e) {
                std::fprintf(stderr, "LatestPublisher publish failed: %s\n", e.what());
            }
            buf.clear();
        }
    }

    adamo::Publisher pub_;
    std::mutex mu_;
    std::condition_variable cv_;
    std::vector<std::uint8_t> pending_;
    bool has_pending_ = false;
    bool stop_ = false;
    std::thread worker_;
};

}  // namespace trossen_adamo
