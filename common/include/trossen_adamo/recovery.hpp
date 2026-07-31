// Glide-button-driven teleop control for the single-arm follower.
//
// Two independent buttons on a Glide leader (SEL_1/SEL_2, i.e. bits 0/1 of
// RobotOutput::InputReport::buttons):
//   - Pause/resume: toggles the follower between tracking the leader and
//     holding its current position. Resuming ramps back to the leader's
//     current pose rather than snapping.
//   - Error recovery: when the follower's driver has faulted (e.g.
//     joint_limit_exceeded), clears the fault (a blocking reconnect via
//     clear_error()) and resumes teleop the same way as a normal resume.
//
// Only meaningful when the follower is run with --button-gated and paired
// with a Glide leader (buttons are Glide-only hardware) — see
// follower/follower.cpp and leader/leader.cpp.

#pragma once

#include "libtrossen_arm/trossen_arm.hpp"

#include <cstdint>
#include <cstdio>
#include <exception>

namespace trossen_adamo::recovery {

// Wraps driver calls that may throw once the driver's ErrorState becomes
// non-`none`. Once faulted, guard() no-ops (skips the call) until try_clear()
// succeeds. clear_error() is a blocking reconnect (hundreds of ms to
// seconds) — call try_clear() only in direct response to a deliberate user
// action (the error-recovery button), not on every tick.
class ArmFaultTracker {
public:
    explicit ArmFaultTracker(const char* who) : who_(who) {}

    bool faulted() const noexcept { return faulted_; }

    template <class F>
    bool guard(F&& fn) {
        if (faulted_) return false;
        try {
            fn();
            return true;
        } catch (const std::exception& e) {
            std::fprintf(stderr,
                "%s: FAULT, press the error-recovery button to clear: %s\n", who_, e.what());
            faulted_ = true;
            return false;
        }
    }

    // Attempt driver.clear_error(). Returns true if the arm is (now) not
    // faulted: a no-op success if it wasn't faulted to begin with.
    bool try_clear(trossen_arm::TrossenArmDriver& driver) {
        if (!faulted_) return true;
        try {
            driver.clear_error();
            faulted_ = false;
            std::fprintf(stderr, "%s: error cleared\n", who_);
            return true;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "%s: clear_error failed, still faulted: %s\n", who_, e.what());
            return false;
        }
    }

private:
    const char* who_;
    bool faulted_ = false;
};

// Edge-detects a single button bit (0-indexed; SEL_1 = bit 0, SEL_2 = bit 1,
// ...) from a Glide leader's InputReport. Reads are wrapped defensively so a
// transient failure just skips that tick rather than propagating.
class ButtonTrigger {
public:
    explicit ButtonTrigger(int bit) : mask_(static_cast<std::uint8_t>(1u << bit)) {}

    // Returns true exactly once per press (rising edge).
    bool poll(trossen_arm::TrossenArmDriver& driver) {
        std::uint8_t buttons = 0;
        try {
            buttons = driver.get_input_report().buttons & mask_;
        } catch (const std::exception&) {
            return false;
        }
        const bool pressed = buttons != 0 && prev_ == 0;
        prev_ = buttons;
        return pressed;
    }

private:
    std::uint8_t mask_;
    std::uint8_t prev_ = 0;
};

// Wire status codes published by the follower (see topics::follower_status_of)
// and consumed by the leader to drive its Glide button LEDs. Doubles for
// wire::encode_status/decode_status consistency.
inline constexpr double kFollowerStatusStopped = 0.0;
inline constexpr double kFollowerStatusActive  = 1.0;
inline constexpr double kFollowerStatusFaulted = 2.0;

// Glide leader button-LED guide: SEL_1 = start/stop, SEL_2 = error recovery,
// SEL_3/4 unused. The LEDs are monochrome (per-button off/solid/breathe,
// one shared brightness) -- no color -- so "needs attention" is conveyed by
// breathing (pulsing) rather than a color change.
enum class LedState { Stopped, Active, Error };

// Stopped: SEL_1 breathes, inviting a press to start.
// Active: all four solid -- an unambiguous "teleop is live" signal.
// Error: SEL_2 breathes, inviting a press to recover; everything else off.
inline trossen_arm::InputCommand make_led_command(LedState state) {
    trossen_arm::InputCommand cmd;
    cmd.button_led_brightness = 200;
    switch (state) {
        case LedState::Stopped: cmd.button_led_effects = {2, 0, 0, 0}; break;
        case LedState::Active:  cmd.button_led_effects = {1, 1, 1, 1}; break;
        case LedState::Error:   cmd.button_led_effects = {0, 2, 0, 0}; break;
    }
    return cmd;
}

}  // namespace trossen_adamo::recovery
