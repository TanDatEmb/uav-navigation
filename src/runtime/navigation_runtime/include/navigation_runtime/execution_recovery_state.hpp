#pragma once

#include <cstdint>

namespace navigation_runtime {

enum class ExecutionRecoveryState : std::uint8_t {
  kInitialHold,
  kTrackMain,
  kTrackBackup,
  kEmergencyBrake,
  kStoppedRecovery,
  kPx4Hold,
};

[[nodiscard]] constexpr bool executionRecoveryStateKnown(
    ExecutionRecoveryState state) noexcept {
  return state == ExecutionRecoveryState::kInitialHold ||
         state == ExecutionRecoveryState::kTrackMain ||
         state == ExecutionRecoveryState::kTrackBackup ||
         state == ExecutionRecoveryState::kEmergencyBrake ||
         state == ExecutionRecoveryState::kStoppedRecovery ||
         state == ExecutionRecoveryState::kPx4Hold;
}

}  // namespace navigation_runtime
