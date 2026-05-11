// Tiny argv parser. Keeps the leader and follower binaries free of any
// hardcoded IPs, robot names, or paths — everything is either supplied on
// the command line or pulled from a documented environment variable.

#pragma once

#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "adamo/adamo.hpp"

namespace trossen_adamo::args {

inline std::string require_value(const std::string& opt, int argc, char** argv, int& i) {
    if (i + 1 >= argc) {
        throw std::runtime_error(opt + " requires a value");
    }
    return argv[++i];
}

inline std::optional<std::string> env(const char* name) {
    if (const char* v = std::getenv(name); v && *v) return std::string(v);
    return std::nullopt;
}

// CLI > env > error. For required values with no sensible default.
inline std::string require_cli_or_env(const std::string& flag,
                                      const char* env_name,
                                      const std::string& cli_value)
{
    if (!cli_value.empty()) return cli_value;
    if (auto v = env(env_name)) return *v;
    throw std::runtime_error("missing " + flag + " (set " + env_name + " or pass " + flag + ")");
}

// CLI flag wins; otherwise env; otherwise the supplied default.
// `cli_value` is whatever the option already holds: empty means not set on
// the CLI, anything else means the user passed it explicitly.
inline std::string cli_env_or_default(const std::string& cli_value,
                                      const char* env_name,
                                      const std::string& default_value)
{
    if (!cli_value.empty()) return cli_value;
    if (auto v = env(env_name)) return *v;
    return default_value;
}

inline adamo::Protocol parse_protocol(const std::string& s) {
    if (s == "quic") return adamo::Protocol::Quic;
    if (s == "udp")  return adamo::Protocol::Udp;
    if (s == "tcp")  return adamo::Protocol::Tcp;
    throw std::runtime_error("invalid --protocol: " + s + " (expected quic|udp|tcp)");
}

inline double parse_double(const std::string& s, const std::string& flag) {
    try { return std::stod(s); }
    catch (...) { throw std::runtime_error("invalid " + flag + ": " + s); }
}

inline int parse_int(const std::string& s, const std::string& flag) {
    try { return std::stoi(s); }
    catch (...) { throw std::runtime_error("invalid " + flag + ": " + s); }
}

}  // namespace trossen_adamo::args
