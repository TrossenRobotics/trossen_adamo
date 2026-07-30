// Trossen arm helpers shared by leader and follower.

#pragma once

#include "libtrossen_arm/trossen_arm.hpp"
#include "trossen_adamo/wire.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace trossen_adamo::arm {

// Home pose
inline std::vector<double> home_pose() {
    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

inline std::unique_ptr<trossen_arm::TrossenArmDriver>
configure(const std::string& ip,
          const trossen_arm::EndEffector& end_effector,
          bool clear_error,
          double connect_timeout,
          trossen_arm::Model model = trossen_arm::Model::wxai_v0)
{
    auto driver = std::make_unique<trossen_arm::TrossenArmDriver>();
    driver->configure(model,
                      end_effector,
                      ip,
                      clear_error,
                      connect_timeout);
    if (driver->get_num_joints() != wire::kNumJoints) {
        throw std::runtime_error("unexpected joint count: " +
                                 std::to_string(driver->get_num_joints()));
    }
    return driver;
}

inline void move_home(trossen_arm::TrossenArmDriver& driver, double goal_time = 2.0) {
    driver.set_all_modes(trossen_arm::Mode::position);
    driver.set_all_positions(home_pose(), goal_time, true);
}

inline void move_sleep(trossen_arm::TrossenArmDriver& driver, double goal_time = 2.0) {
    driver.set_all_modes(trossen_arm::Mode::position);
    driver.set_all_positions(std::vector<double>(driver.get_num_joints(), 0.0),
                             goal_time, true);
}

// Best-effort park: position mode, then home, then sleep, swallowing every
// error so the caller can run this from a destructor or catch block without
// masking the original failure. Each step is independent: a failure in one
// stage does not skip later stages, and every failure is logged. Blocks
// for up to ~goal_time per move (default 2s + 2s).
inline void safe_park(trossen_arm::TrossenArmDriver& driver, const char* who) noexcept {
    const auto attempt = [&](auto&& fn, const char* what) noexcept {
        try { fn(); }
        catch (const std::exception& e) {
            std::fprintf(stderr, "%s: %s on shutdown: %s\n", who, what, e.what());
        } catch (...) {
            std::fprintf(stderr, "%s: %s on shutdown: unknown error\n", who, what);
        }
    };
    attempt([&]{ driver.set_all_modes(trossen_arm::Mode::position); },
            "set_all_modes(position)");
    attempt([&]{ move_home(driver); },  "move_home");
    attempt([&]{ move_sleep(driver); }, "move_sleep");
}

// RAII scope guard: runs safe_park on destruction unless dismissed. Use
// this to make sure the arm reaches a safe pose on every exit path —
// normal completion, std::exception unwinding, or a thrown handshake
// timeout / driver fault. Place the guard immediately after configure()
// so even early failures (move_home throw, publisher open failure,
// handshake timeout) trigger parking.
class ArmParkGuard {
public:
    ArmParkGuard(trossen_arm::TrossenArmDriver& driver, const char* who) noexcept
        : driver_(&driver), who_(who) {}

    ArmParkGuard(const ArmParkGuard&)            = delete;
    ArmParkGuard& operator=(const ArmParkGuard&) = delete;
    ArmParkGuard(ArmParkGuard&&)                 = delete;
    ArmParkGuard& operator=(ArmParkGuard&&)      = delete;

    ~ArmParkGuard() {
        if (!dismissed_) safe_park(*driver_, who_);
    }

    void dismiss() noexcept { dismissed_ = true; }

private:
    trossen_arm::TrossenArmDriver* driver_;
    const char* who_;
    bool dismissed_ = false;
};

}  // namespace trossen_adamo::arm
