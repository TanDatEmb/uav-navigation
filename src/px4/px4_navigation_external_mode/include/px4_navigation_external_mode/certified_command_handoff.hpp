#pragma once

#include <optional>

#include <navigation_contracts/msg/navigation_command.hpp>

namespace px4_navigation_external_mode {

enum class CertifiedCommandTransition { kRetain, kCommit, kInvalidate };

inline bool commandMayBeRetainedAcrossWaypointHandoff(
    const navigation_contracts::msg::NavigationCommand& command) noexcept {
  return command.status !=
         navigation_contracts::msg::NavigationCommand::STATUS_REJECTED;
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
