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

// A pass-through transition can advance MissionController before the planner
// has reached a certified stop/retarget boundary. During that bounded window
// the previous BACKUP command remains the physical owner of the vehicle. It
// may refresh the External Mode command lease, but it must never be accepted
// as the new waypoint's nominal command. A retry for the same checkpoint is
// deliberately not bridged here: Runtime owns that retry and must preserve
// the exact mission/request identity instead of creating two owners.
inline bool priorSafetySuffixCommandIdentityMatches(
    const navigation_contracts::msg::NavigationCommand& command,
    const std::string& mission_id, std::uint32_t active_waypoint_index,
    std::uint64_t active_request_id, std::uint32_t suffix_waypoint_index,
    std::uint64_t suffix_request_id) noexcept {
  return command.mission_id == mission_id &&
         command.waypoint_index == suffix_waypoint_index &&
         command.request_id == suffix_request_id &&
         suffix_waypoint_index + 1U == active_waypoint_index &&
         suffix_request_id < active_request_id &&
         command.role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP &&
         (command.status == navigation_contracts::msg::NavigationCommand::STATUS_READY ||
          command.status == navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED);
}

// MissionController publishes the next pass-through goal as soon as the
// previous checkpoint is accepted.  The planner successor is intentionally a
// later execution-timeline activation, so one unfinished MAIN command from
// the immediately previous pass-through checkpoint must remain admissible in
// the interim.  This is an explicit continuity bridge, not a goal rebind:
// the command keeps its old {waypoint, request} identity and may not bridge a
// STOP, safety suffix, skipped waypoint, or completed command.
inline bool priorPassThroughMainCommandIdentityMatches(
    const navigation_contracts::msg::NavigationCommand& command,
    const std::string& mission_id, std::uint32_t active_waypoint_index,
    std::uint64_t active_request_id, bool previous_waypoint_is_pass_through) noexcept {
  return previous_waypoint_is_pass_through && active_waypoint_index > 0U &&
         command.mission_id == mission_id &&
         command.waypoint_index + 1U == active_waypoint_index &&
         command.request_id < active_request_id && command.request_id != 0U &&
         command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN &&
         command.status == navigation_contracts::msg::NavigationCommand::STATUS_READY;
}

}  // namespace px4_navigation_external_mode
