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

}  // namespace trossen_adamo::topics
