#pragma once

#include <optional>

#include <navigation_contracts/msg/navigation_command.hpp>

namespace px4_navigation_external_mode {

enum class CertifiedCommandTransition { kRetain, kCommit, kInvalidate };

inline bool commandMayBeRetainedAcrossWaypointHandoff(
    const navigation_contracts::msg::NavigationCommand& command,
    const bool terminal_successor = false) noexcept {
  // A completed command is no longer the physical owner of the vehicle. This
  // includes a completed BACKUP: retaining it after MissionController
  // advances would replay an old endpoint under the new waypoint and can
  // trigger a false stale-command handover. The sole exception is a completed
  // MAIN terminal endpoint when the next route identity is the coincident STOP
  // successor; that bounded endpoint is still the certified physical owner
  // until the successor transaction or its finite lease closes it.
  return command.status ==
             navigation_contracts::msg::NavigationCommand::STATUS_READY ||
         (terminal_successor &&
          command.status ==
              navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED &&
          command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN);
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
