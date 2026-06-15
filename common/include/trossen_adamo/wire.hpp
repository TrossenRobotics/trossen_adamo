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

namespace trossen_adamo::wire {

inline constexpr int kNumJoints = 7;
inline constexpr std::size_t kStateBytes  = (1 + kNumJoints * 2) * sizeof(double);
inline constexpr std::size_t kEffortBytes = (1 + kNumJoints) * sizeof(double);
inline constexpr std::size_t kReadyBytes  = sizeof(double);

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
