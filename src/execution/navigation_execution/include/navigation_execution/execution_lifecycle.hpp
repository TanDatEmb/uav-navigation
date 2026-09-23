#pragma once

#include <cstdint>
#include <navigation_execution/execution_recovery_state.hpp>

namespace navigation_execution {

enum class ExecutionEpisodePhase : std::uint8_t {
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
constexpr std::uint8_t executionEpisodePhaseTelemetryCodeV1(
    ExecutionEpisodePhase phase) noexcept {
  switch (phase) {
    case ExecutionEpisodePhase::kInitialHold:
      return 0U;
    case ExecutionEpisodePhase::kTrackingMain:
      return 2U;
    case ExecutionEpisodePhase::kTrackingBackup:
      return 3U;
    case ExecutionEpisodePhase::kStoppedHold:
      return 4U;
    case ExecutionEpisodePhase::kPx4Hold:
      return 5U;
  }
  return 0xFFU;
}

struct ExecutionEpisodeSnapshot final {
  std::uint64_t localization_epoch{0U};
  // Desired identity can advance during a hot retarget while the active
  // command still belongs to the previous waypoint. Keep that execution
  // identity until the successor bundle is actually committed.
  std::uint64_t goal_epoch{0U};
  std::uint64_t request_id{0U};
  std::uint64_t active_command_goal_epoch{0U};
  std::uint64_t active_command_request_id{0U};
  std::uint64_t active_generation{0U};
  ExecutionEpisodePhase phase{ExecutionEpisodePhase::kInitialHold};
  bool command_available{false};
  bool failure_latched{false};
  bool safety_suffix_active{false};
  bool restart_from_rest{false};
  // Recovery policy is part of this same physical-execution snapshot. It is
  // intentionally distinct from sampled phase (for example, a terminal MAIN
  // can already be emitting STOPPED_HOLD while completion is still pending),
  // but it no longer has an independently mutable owner.
  ExecutionRecoveryState recovery_state{ExecutionRecoveryState::kInitialHold};
};

}  // namespace navigation_execution
