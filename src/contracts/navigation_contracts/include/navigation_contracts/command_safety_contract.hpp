#pragma once

#include <cstdint>

namespace navigation_contracts {

// Product command-envelope contract. This is deliberately a named contract
// constant rather than a node-local tuning parameter: the planner runtime and
// PX4 command consumer must enforce the same geometric acceptance limit.
inline constexpr double kCommandAnchorErrorLimitM = 0.75;

[[nodiscard]] constexpr bool estimatorHealthAllowsCommand(
    const bool typed_health_seen,
    const bool health_valid,
    const std::uint64_t command_localization_epoch,
    const std::uint64_t health_localization_epoch) noexcept {
  return typed_health_seen && health_valid && command_localization_epoch != 0U &&
         command_localization_epoch == health_localization_epoch;
}

}  // namespace navigation_contracts
