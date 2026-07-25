// Topic-name builder. Both the leader and follower derive their topic names
// from a single robot identifier so that the only thing that has to match
// between the two machines is the --robot argument.

#pragma once

#include <string>

namespace trossen_adamo::topics {

inline std::string state_of(const std::string& robot) {
    return robot + "/trossen/reference/leader_state";
}
inline std::string effort_of(const std::string& robot) {
    return robot + "/trossen/reference/follower_effort";
}
inline std::string leader_ready_of(const std::string& robot) {
    return robot + "/trossen/reference/leader_ready";
}
inline std::string follower_ready_of(const std::string& robot) {
    return robot + "/trossen/reference/follower_ready";
}
inline std::string vr_headset_ready_of(const std::string& robot) {
    return robot + "/trossen/reference/vr_headset_ready";
}
inline std::string vr_state_of(const std::string& robot) {
    return robot + "/trossen/reference/vr_state";
}

// Bimanual-specific topics: left and right arm channels.
inline std::string state_left_of(const std::string& robot) {
    return robot + "/trossen/reference/leader_state_left";
}
inline std::string state_right_of(const std::string& robot) {
    return robot + "/trossen/reference/leader_state_right";
}
inline std::string effort_left_of(const std::string& robot) {
    return robot + "/trossen/reference/follower_effort_left";
}
inline std::string effort_right_of(const std::string& robot) {
    return robot + "/trossen/reference/follower_effort_right";
}

}  // namespace trossen_adamo::topics
