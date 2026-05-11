// Callback-driven single-slot subscriber.
//
// The Adamo C SDK lets us register a callback that fires on the SDK's
// receive thread. We use that to keep the latest payload in a small
// mutex-guarded slot so the control loop can poll without ever blocking
// on the receive path — the loop never serialises against the SDK's
// network thread.
//
// Compared with calling `try_recv` inline on the control thread:
//   - the control loop's per-tick cost is a `try_lock` on a small mutex,
//     not a queue drain through the SDK boundary;
//   - older samples are dropped automatically (we keep only the newest);
//   - heap allocations are amortised — `poll()` swaps buffers with the
//     callback so both sides keep their capacity across iterations.

#pragma once

#include "adamo/adamo.hpp"

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace trossen_adamo {

class LatestSubscriber {
public:
    LatestSubscriber(adamo::Session& sess, const std::string& key) {
        // The lambda captures `this`. CallbackSubscriber's destructor
        // synchronously stops the subscriber before returning, so
        // callbacks are guaranteed to have stopped firing by the time
        // `this` is destroyed.
        sub_ = sess.subscribe_cb(key, [this](adamo::SampleView v) {
            if (v.is_delete) return;
            std::lock_guard<std::mutex> lk(mu_);
            pending_.assign(v.payload, v.payload + v.payload_len);
            has_pending_ = true;
        });
    }

    LatestSubscriber(const LatestSubscriber&)            = delete;
    LatestSubscriber& operator=(const LatestSubscriber&) = delete;
    LatestSubscriber(LatestSubscriber&&)                 = delete;
    LatestSubscriber& operator=(LatestSubscriber&&)      = delete;

    // If a fresh payload arrived since the last poll, swap it into
    // `out_buf` and return true. Otherwise leave `out_buf` untouched and
    // return false. Swap (not copy) keeps the inner buffer's capacity
    // across iterations after warm-up.
    bool poll(std::vector<std::uint8_t>& out_buf) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!has_pending_) return false;
        has_pending_ = false;
        out_buf.swap(pending_);
        return true;
    }

private:
    std::mutex mu_;
    std::vector<std::uint8_t> pending_;
    bool has_pending_ = false;
    adamo::CallbackSubscriber sub_;
};

}  // namespace trossen_adamo
