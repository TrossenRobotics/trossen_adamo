// Ready handshake: each side publishes a wall-clock timestamp on its own
// ready topic and waits for a fresh sample (timestamp >= our start time)
// on the peer's topic before entering the teleop loop.

#pragma once

#include "adamo/adamo.hpp"
#include "trossen_adamo/signal.hpp"
#include "trossen_adamo/wire.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace trossen_adamo::handshake {

// Drain queued samples and return the most recent one.
inline std::optional<adamo::Sample> drain_latest(adamo::Subscriber& sub) {
    std::optional<adamo::Sample> latest;
    while (auto s = sub.try_recv()) {
        latest = std::move(s);
    }
    return latest;
}

inline void wait_for_peer_ready(adamo::Publisher& self_ready_pub,
                                adamo::Subscriber& peer_ready_sub,
                                double ready_timeout_s,
                                const char* peer_label)
{
    const double started_at = wire::now_seconds();
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(ready_timeout_s);
    while (!stop_requested() && std::chrono::steady_clock::now() < deadline) {
        const auto self_payload = wire::encode_ready(wire::now_seconds());
        self_ready_pub.put(self_payload.data(), self_payload.size());

        if (auto sample = drain_latest(peer_ready_sub)) {
            double stamp = 0.0;
            if (wire::decode_ready(sample->payload(), sample->payload_len(), &stamp) &&
                stamp >= started_at) {
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    throw std::runtime_error(std::string("timed out waiting for ") + peer_label + "_ready");
}

}  // namespace trossen_adamo::handshake
