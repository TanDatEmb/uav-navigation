#pragma once

#include <cstdint>
#include <navigation_execution/execution_recovery_state.hpp>

namespace navigation_execution {

enum class ExecutionPhase : std::uint8_t {
  // Internal physical execution lifecycle. Planning-worker activity remains
  // orthogonal to this episode.
  kInitialHold = 0,
  kTrackingMain = 1,
  kTrackingBackup = 2,
  kStoppedHold = 3,
  kPx4Hold = 4,
};

// The diagnostic key execution_episode_phase is a v1 telemetry field. Keep
// its historical wire codes at this publication boundary while the internal
// lifecycle enum remains contiguous and free to evolve independently.
constexpr std::uint8_t executionPhaseTelemetryCodeV1(
    ExecutionPhase phase) noexcept {
  switch (phase) {
    case ExecutionPhase::kInitialHold:
      return 0U;
    case ExecutionPhase::kTrackingMain:
      return 2U;
    case ExecutionPhase::kTrackingBackup:
      return 3U;
    case ExecutionPhase::kStoppedHold:
      return 4U;
    case ExecutionPhase::kPx4Hold:
      return 5U;
  }
  return 0xFFU;
}

// These are independent execution facts. A frozen safety suffix may sample
// MAIN before its BACKUP switch, and analytic hold can precede measured stop.
enum class ExecutionExposure : std::uint8_t {
  kUnavailable,
  kAvailable,
  kSuspended,
  kFailed,
};

enum class ExecutionSafetyOwnership : std::uint8_t {
  kNominal,
  kSafetySuffix,
};

enum class ExecutionRestartRequest : std::uint8_t {
  kNone,
  kFromRest,
};

struct ExecutionLifecycleState final {
  ExecutionPhase phase{ExecutionPhase::kInitialHold};
  ExecutionRecoveryState recovery{ExecutionRecoveryState::kInitialHold};
  ExecutionExposure exposure{ExecutionExposure::kUnavailable};
  ExecutionSafetyOwnership safety{ExecutionSafetyOwnership::kNominal};
  ExecutionRestartRequest restart{ExecutionRestartRequest::kNone};

  bool operator==(const ExecutionLifecycleState&) const = default;
};

}  // namespace navigation_execution
