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

    // Ignore SIGPIPE: unhandled, a write to a closed socket (e.g. during
    // clear_error()'s TCP reconnect) kills the whole process instantly with
    // no exception. Ignoring it turns that into a plain EPIPE error, which
    // trossen_arm-source already handles as a catchable exception.
    std::signal(SIGPIPE, SIG_IGN);
}

inline bool stop_requested() noexcept {
    return detail::stop_flag_storage.load(std::memory_order_relaxed);
}

}  // namespace trossen_adamo
