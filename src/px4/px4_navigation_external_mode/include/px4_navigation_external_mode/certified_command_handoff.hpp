#pragma once

#include <optional>

#include <navigation_contracts/msg/navigation_command.hpp>

namespace px4_navigation_external_mode {

enum class CertifiedCommandTransition { kRetain, kCommit, kInvalidate };

inline bool commandMayBeRetainedAcrossWaypointHandoff(
    const navigation_contracts::msg::NavigationCommand& command) noexcept {
  // A completed command is no longer the physical owner of the vehicle. This
  // includes a completed BACKUP: retaining it after MissionController
  // advances would replay an old endpoint under the new waypoint and can
  // trigger a false stale-command handover. Only an unfinished READY command
  // may bridge the callback ordering boundary; the next waypoint then waits
  // for its own identity-matching command.
  return command.status ==
         navigation_contracts::msg::NavigationCommand::STATUS_READY;
}

inline std::optional<navigation_contracts::msg::NavigationCommand>
transitionCertifiedCommand(
    const std::optional<navigation_contracts::msg::NavigationCommand>& current,
    const std::optional<navigation_contracts::msg::NavigationCommand>& candidate,
    CertifiedCommandTransition transition) {
  switch (transition) {
    case CertifiedCommandTransition::kRetain:
      return current;
    case CertifiedCommandTransition::kCommit:
      return candidate;
    case CertifiedCommandTransition::kInvalidate:
      return std::nullopt;
  }
  return std::nullopt;
}

}  // namespace px4_navigation_external_mode
