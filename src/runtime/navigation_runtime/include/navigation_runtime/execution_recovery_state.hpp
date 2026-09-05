#pragma once

#include <cstdint>
#include <atomic>

namespace navigation_runtime {

enum class ExecutionRecoveryState : std::uint8_t {
  kInitialHold,
  kTrackMain,
  kTrackBackup,
  kEmergencyBrake,
  kStoppedRecovery,
  kPx4Hold,
};

enum class ExecutionRecoveryEvent : std::uint8_t {
  kMainCommitted,
  kBackupActivated,
  kEmergencyCommitted,
  kCertifiedStopObserved,
  // A nominal terminal STOP is a distinct completion event. A safety
  // endpoint must never be promoted by completion geometry alone.
  kTerminalStopCompleted,
  kEmergencyCertificationFailed,
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

[[nodiscard]] constexpr bool nominalPlanningAllowed(
    ExecutionRecoveryState state) noexcept {
  return state == ExecutionRecoveryState::kInitialHold ||
         state == ExecutionRecoveryState::kTrackMain ||
         state == ExecutionRecoveryState::kStoppedRecovery;
}

// BACKUP and emergency braking are one-way until a fresh measured stop is
// observed. In particular, a late nominal result cannot replace either moving
// safety command. The caller owns freshness and the <=0.15 m/s measurement;
// this transition function owns only the public state semantics.
[[nodiscard]] constexpr ExecutionRecoveryState transitionExecutionRecovery(
    ExecutionRecoveryState state, ExecutionRecoveryEvent event) noexcept {
  if (!executionRecoveryStateKnown(state)) {
    return ExecutionRecoveryState::kPx4Hold;
  }
  if (state == ExecutionRecoveryState::kPx4Hold) {
    return state;
  }
  if (event == ExecutionRecoveryEvent::kEmergencyCertificationFailed) {
    return ExecutionRecoveryState::kPx4Hold;
  }
  switch (state) {
    case ExecutionRecoveryState::kInitialHold:
      return event == ExecutionRecoveryEvent::kMainCommitted
          ? ExecutionRecoveryState::kTrackMain : state;
    case ExecutionRecoveryState::kTrackMain:
      if (event == ExecutionRecoveryEvent::kBackupActivated) {
        return ExecutionRecoveryState::kTrackBackup;
      }
      if (event == ExecutionRecoveryEvent::kEmergencyCommitted) {
        return ExecutionRecoveryState::kEmergencyBrake;
      }
      if (event == ExecutionRecoveryEvent::kTerminalStopCompleted) {
        return ExecutionRecoveryState::kStoppedRecovery;
      }
      return state;
    case ExecutionRecoveryState::kTrackBackup:
    case ExecutionRecoveryState::kEmergencyBrake:
      return event == ExecutionRecoveryEvent::kCertifiedStopObserved
          ? ExecutionRecoveryState::kStoppedRecovery : state;
    case ExecutionRecoveryState::kStoppedRecovery:
      return event == ExecutionRecoveryEvent::kMainCommitted
          ? ExecutionRecoveryState::kTrackMain : state;
    case ExecutionRecoveryState::kPx4Hold:
      return state;
  }
  return ExecutionRecoveryState::kPx4Hold;
}

// The caller must hold the execution transition mutex.  Keeping the
// load/transition/store sequence here prevents event writers from silently
// bypassing the one-way recovery semantics while avoiding a second lock or a
// CAS loop inside the already-serialized transition boundary.
inline void applyExecutionRecoveryEventLocked(
    std::atomic<ExecutionRecoveryState>& state,
    ExecutionRecoveryEvent event) noexcept {
  state.store(
      transitionExecutionRecovery(state.load(std::memory_order_acquire), event),
      std::memory_order_release);
}

}  // namespace navigation_runtime
