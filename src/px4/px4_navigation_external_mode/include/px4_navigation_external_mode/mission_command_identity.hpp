#pragma once

#include <cstdint>
#include <string>

#include <navigation_contracts/navigation_command_contract.hpp>

namespace px4_navigation_external_mode {

inline bool missionCommandIdentityMatches(
    const navigation_contracts::msg::NavigationCommand& command,
    const std::string& mission_id, std::uint32_t active_waypoint_index,
    std::uint64_t active_request_id, bool mission_terminal,
    std::uint32_t last_completed_waypoint_index,
    std::uint64_t last_completed_request_id) {
  if (!mission_terminal) {
    return navigation_contracts::commandMissionIdentityMatches(
        command, mission_id, active_waypoint_index, active_request_id);
  }
  return command.mission_id == mission_id &&
         command.waypoint_index == last_completed_waypoint_index &&
         command.request_id == last_completed_request_id;
}

}  // namespace px4_navigation_external_mode
