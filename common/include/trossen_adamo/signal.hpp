// Process-wide stop flag set by SIGINT / SIGTERM.
//
// The flag lives at namespace scope as an inline variable so it is
// constant-initialised at static-init time, before main runs. A
// function-local static would be initialised lazily on first call; if a
// signal arrived before any thread had ever called the accessor, the
// implicit __cxa_guard_acquire would run inside the signal handler and
// reach into pthread internals — not async-signal-safe.

#pragma once

#include <atomic>
#include <csignal>

namespace trossen_adamo {

namespace detail {
inline std::atomic<bool> stop_flag_storage{false};
}  // namespace detail

inline std::atomic<bool>& stop_flag() noexcept {
    return detail::stop_flag_storage;
}

inline void install_signal_handlers() {
    auto handler = [](int) noexcept { detail::stop_flag_storage.store(true); };
    std::signal(SIGINT,  handler);
    std::signal(SIGTERM, handler);
}

inline bool stop_requested() noexcept {
    return detail::stop_flag_storage.load(std::memory_order_relaxed);
}

}  // namespace trossen_adamo
