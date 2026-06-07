// Binary wire format for the leader/follower teleop link.
//
// All payloads are big-endian-packed IEEE-754 float64s — i.e. Python's
// `struct.pack('!d' * N, ...)` — so other-language implementations stay
// interoperable.
//
//   leader_state   : [timestamp, p0..p6, v0..v6]   = 15 doubles = 120 bytes
//   follower_effort: [timestamp, e0..e6]           =  8 doubles =  64 bytes
//   *_ready        : [timestamp]                   =  1 double  =   8 bytes

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include "trossen_vr/vr_types.hpp"

namespace trossen_adamo::wire {

inline constexpr int kNumJoints = 7;
inline constexpr std::size_t kStateBytes  = (1 + kNumJoints * 2) * sizeof(double);
inline constexpr std::size_t kEffortBytes = (1 + kNumJoints) * sizeof(double);
inline constexpr std::size_t kReadyBytes  = sizeof(double);

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

inline std::uint64_t bswap64(std::uint64_t v) noexcept {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_bswap64(v);
#else
    return ((v & 0x00000000000000FFULL) << 56) |
           ((v & 0x000000000000FF00ULL) << 40) |
           ((v & 0x0000000000FF0000ULL) << 24) |
           ((v & 0x00000000FF000000ULL) << 8)  |
           ((v & 0x000000FF00000000ULL) >> 8)  |
           ((v & 0x0000FF0000000000ULL) >> 24) |
           ((v & 0x00FF000000000000ULL) >> 40) |
           ((v & 0xFF00000000000000ULL) >> 56);
#endif
}

inline bool host_is_little_endian() noexcept {
    constexpr std::uint16_t probe = 0x0001;
    return *reinterpret_cast<const std::uint8_t*>(&probe) == 0x01;
}

inline void pack_be_double(double value, std::uint8_t* out) noexcept {
    std::uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    if (host_is_little_endian()) bits = bswap64(bits);
    std::memcpy(out, &bits, sizeof(bits));
}

inline double unpack_be_double(const std::uint8_t* in) noexcept {
    std::uint64_t bits;
    std::memcpy(&bits, in, sizeof(bits));
    if (host_is_little_endian()) bits = bswap64(bits);
    double value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline double now_seconds() noexcept {
    using namespace std::chrono;
    const auto since_epoch = system_clock::now().time_since_epoch();
    return duration_cast<duration<double>>(since_epoch).count();
}

inline std::array<std::uint8_t, kStateBytes>
encode_state(double timestamp,
             const std::vector<double>& positions,
             const std::vector<double>& velocities)
{
    if (positions.size() != kNumJoints || velocities.size() != kNumJoints) {
        throw std::runtime_error("encode_state: expected " +
                                 std::to_string(kNumJoints) + " joints");
    }
    std::array<std::uint8_t, kStateBytes> out{};
    std::uint8_t* p = out.data();
    pack_be_double(timestamp, p); p += sizeof(double);
    for (double v : positions)  { pack_be_double(v, p); p += sizeof(double); }
    for (double v : velocities) { pack_be_double(v, p); p += sizeof(double); }
    return out;
}

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

struct State {
    double timestamp = 0.0;
    std::vector<double> positions;
    std::vector<double> velocities;
};

inline State decode_state(const std::uint8_t* data, std::size_t len) {
    if (len != kStateBytes) {
        throw std::runtime_error("decode_state: bad payload size " + std::to_string(len));
    }
    State s;
    s.positions.resize(kNumJoints);
    s.velocities.resize(kNumJoints);
    const std::uint8_t* p = data;
    s.timestamp = unpack_be_double(p); p += sizeof(double);
    for (int i = 0; i < kNumJoints; ++i) { s.positions[i]  = unpack_be_double(p); p += sizeof(double); }
    for (int i = 0; i < kNumJoints; ++i) { s.velocities[i] = unpack_be_double(p); p += sizeof(double); }
    return s;
}

inline std::array<std::uint8_t, kEffortBytes>
encode_efforts(double timestamp, const std::vector<double>& efforts)
{
    if (efforts.size() != kNumJoints) {
        throw std::runtime_error("encode_efforts: expected " +
                                 std::to_string(kNumJoints) + " joints");
    }
    std::array<std::uint8_t, kEffortBytes> out{};
    std::uint8_t* p = out.data();
    pack_be_double(timestamp, p); p += sizeof(double);
    for (double v : efforts) { pack_be_double(v, p); p += sizeof(double); }
    return out;
}

struct Efforts {
    double timestamp = 0.0;
    std::vector<double> efforts;
};

inline Efforts decode_efforts(const std::uint8_t* data, std::size_t len) {
    if (len != kEffortBytes) {
        throw std::runtime_error("decode_efforts: bad payload size " + std::to_string(len));
    }
    Efforts e;
    e.efforts.resize(kNumJoints);
    const std::uint8_t* p = data;
    e.timestamp = unpack_be_double(p); p += sizeof(double);
    for (int i = 0; i < kNumJoints; ++i) { e.efforts[i] = unpack_be_double(p); p += sizeof(double); }
    return e;
}

inline std::array<std::uint8_t, kReadyBytes> encode_ready(double timestamp) {
    std::array<std::uint8_t, kReadyBytes> out{};
    pack_be_double(timestamp, out.data());
    return out;
}

inline bool decode_ready(const std::uint8_t* data, std::size_t len, double* out) {
    if (len != kReadyBytes || data == nullptr || out == nullptr) return false;
    *out = unpack_be_double(data);
    return true;
}

}  // namespace trossen_adamo::wire
