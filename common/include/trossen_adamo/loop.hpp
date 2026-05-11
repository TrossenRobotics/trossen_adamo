// Fixed-rate loop pacing.

#pragma once

#include <chrono>
#include <thread>

namespace trossen_adamo {

inline void sleep_until_next_tick(std::chrono::steady_clock::time_point start,
                                  double rate_hz)
{
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    const auto target = start + std::chrono::duration_cast<std::chrono::nanoseconds>(period);
    if (std::chrono::steady_clock::now() < target) {
        std::this_thread::sleep_until(target);
    }
}

}  // namespace trossen_adamo
