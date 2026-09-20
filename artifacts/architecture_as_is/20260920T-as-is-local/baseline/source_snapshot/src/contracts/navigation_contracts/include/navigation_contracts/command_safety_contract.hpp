#pragma once

#include <cstdint>

namespace navigation_contracts {

// Product command-envelope contract. This is deliberately a named contract
// constant rather than a node-local tuning parameter: the planner runtime and
// PX4 command consumer must enforce the same geometric acceptance limit.
inline constexpr double kCommandAnchorErrorLimitM = 0.75;

// Ordinary MAIN tracking separates progress from geometric path error. This
// finite phase window equals the established 100 ms planner cadence; both
// runtime projection and the PX4 longitudinal guard consume this contract.
// It does not extend command freshness, validity or recovery authority.
inline constexpr double kMainTrackingPhaseWindowS = 0.10;

[[nodiscard]] constexpr bool estimatorHealthAllowsCommand(
    const bool typed_health_seen,
    const bool health_valid,
    const std::uint64_t command_localization_epoch,
    const std::uint64_t health_localization_epoch) noexcept {
  return typed_health_seen && health_valid && command_localization_epoch != 0U &&
         command_localization_epoch == health_localization_epoch;
}

}  // namespace navigation_contracts
