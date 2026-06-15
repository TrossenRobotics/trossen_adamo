// Binary wire format for VR teleoperation (extends wire.hpp).
//
// VR-specific encoding/decoding functions isolated from the core wire.hpp
// to avoid forcing VR dependencies (trossen_vr headers) on non-VR binaries.
//
//   vr_frame: [timestamp, right_controller, left_controller] = 124 bytes

#pragma once

#include "trossen_adamo/wire.hpp"
#include "trossen_vr/vr_types.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace trossen_adamo::wire {

inline constexpr std::size_t kVrFrameBytes =
    sizeof(double) +                  // Timestamp (8 bytes)
    sizeof(uint8_t) +                 // Right is_tracked (1 byte)
    6 * sizeof(double) +              // Right Pose6D: x,y,z,ax,ay,az (48 bytes)
    sizeof(double) +                  // Right index_trigger (8 bytes)
    2 * sizeof(uint8_t) +             // Right buttons: one, two (2 bytes)
    sizeof(uint8_t) +                 // Left is_tracked (1 byte)
    6 * sizeof(double) +              // Left Pose6D: x,y,z,ax,ay,az (48 bytes)
    sizeof(double) +                  // Left index_trigger (8 bytes)
    2 * sizeof(uint8_t);              // Left buttons: one, two (2 bytes)
                                      // Total: 124 bytes

inline std::array<std::uint8_t, kVrFrameBytes> 
encode_vr_frame(double timestamp, const trossen_vr::VRFrame& frame) {
    std::array<std::uint8_t, kVrFrameBytes> out{};
    std::uint8_t* p = out.data();

    // Pack timestamp
    pack_be_double(timestamp, p);
    p += sizeof(double);

    // Pack right controller
    *p = frame.right_controller.is_tracked;
    p += sizeof(uint8_t);
    pack_be_double(frame.right_controller.pose6d.x, p); p += sizeof(double);
    pack_be_double(frame.right_controller.pose6d.y, p); p += sizeof(double);
    pack_be_double(frame.right_controller.pose6d.z, p); p += sizeof(double);
    pack_be_double(frame.right_controller.pose6d.ax, p); p += sizeof(double);
    pack_be_double(frame.right_controller.pose6d.ay, p); p += sizeof(double);
    pack_be_double(frame.right_controller.pose6d.az, p); p += sizeof(double);
    pack_be_double(frame.right_controller.triggers.index_trigger, p); p += sizeof(double);
    *p = frame.right_controller.buttons.one; p += sizeof(uint8_t);
    *p = frame.right_controller.buttons.two; p += sizeof(uint8_t);

    // Pack left controller
    *p = frame.left_controller.is_tracked;
    p += sizeof(uint8_t);
    pack_be_double(frame.left_controller.pose6d.x, p); p += sizeof(double);
    pack_be_double(frame.left_controller.pose6d.y, p); p += sizeof(double);
    pack_be_double(frame.left_controller.pose6d.z, p); p += sizeof(double);
    pack_be_double(frame.left_controller.pose6d.ax, p); p += sizeof(double);
    pack_be_double(frame.left_controller.pose6d.ay, p); p += sizeof(double);
    pack_be_double(frame.left_controller.pose6d.az, p); p += sizeof(double);
    pack_be_double(frame.left_controller.triggers.index_trigger, p); p += sizeof(double);
    *p = frame.left_controller.buttons.one; p += sizeof(uint8_t);
    *p = frame.left_controller.buttons.two; p += sizeof(uint8_t);

    return out;
}

inline trossen_vr::VRFrame 
decode_vr_frame(const std::uint8_t* data, std::size_t len, double* out_timestamp = nullptr) {
    if (len != kVrFrameBytes) {
        throw std::runtime_error("decode_vr_frame: Binary telemetry packet corrupted! Expected " + 
                                 std::to_string(kVrFrameBytes) + " bytes, but received: " + std::to_string(len));
    }

    trossen_vr::VRFrame frame;
    const std::uint8_t* p = data;

    // Unpack timestamp
    double ts = unpack_be_double(p);
    if (out_timestamp) {
        *out_timestamp = ts;
    }
    p += sizeof(double);

    // Unpack right controller
    frame.right_controller.is_tracked = *p;
    p += sizeof(uint8_t);
    frame.right_controller.pose6d.x = unpack_be_double(p); p += sizeof(double);
    frame.right_controller.pose6d.y = unpack_be_double(p); p += sizeof(double);
    frame.right_controller.pose6d.z = unpack_be_double(p); p += sizeof(double);
    frame.right_controller.pose6d.ax = unpack_be_double(p); p += sizeof(double);
    frame.right_controller.pose6d.ay = unpack_be_double(p); p += sizeof(double);
    frame.right_controller.pose6d.az = unpack_be_double(p); p += sizeof(double);
    frame.right_controller.triggers.index_trigger = unpack_be_double(p); p += sizeof(double);
    frame.right_controller.buttons.one = *p; p += sizeof(uint8_t);
    frame.right_controller.buttons.two = *p; p += sizeof(uint8_t);

    // Unpack left controller
    frame.left_controller.is_tracked = *p;
    p += sizeof(uint8_t);
    frame.left_controller.pose6d.x = unpack_be_double(p); p += sizeof(double);
    frame.left_controller.pose6d.y = unpack_be_double(p); p += sizeof(double);
    frame.left_controller.pose6d.z = unpack_be_double(p); p += sizeof(double);
    frame.left_controller.pose6d.ax = unpack_be_double(p); p += sizeof(double);
    frame.left_controller.pose6d.ay = unpack_be_double(p); p += sizeof(double);
    frame.left_controller.pose6d.az = unpack_be_double(p); p += sizeof(double);
    frame.left_controller.triggers.index_trigger = unpack_be_double(p); p += sizeof(double);
    frame.left_controller.buttons.one = *p; p += sizeof(uint8_t);
    frame.left_controller.buttons.two = *p; p += sizeof(uint8_t);

    return frame;
}

}  // namespace trossen_adamo::wire
