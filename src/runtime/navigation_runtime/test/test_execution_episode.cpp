#include <gtest/gtest.h>

#include <navigation_runtime/execution_episode.hpp>

namespace {

navigation_planning::CandidateBundle bundle(
    navigation_planning::CandidateBundleKind kind,
    std::uint64_t generation) {
  navigation_planning::CandidateBundle value;
  value.kind = kind;
  value.bundle_generation = generation;
  return value;
}

TEST(ExecutionEpisode, KeepsOneAuthoritativeLifecycleSnapshot) {
  navigation_runtime::ExecutionEpisode episode;
  episode.reset(4U);
  episode.beginGoal(4U, 7U, 11U, false);
  auto initial = episode.snapshot();
  EXPECT_EQ(initial.localization_epoch, 4U);
  EXPECT_EQ(initial.goal_epoch, 7U);
  EXPECT_EQ(initial.request_id, 11U);
  EXPECT_EQ(initial.phase, navigation_runtime::ExecutionEpisodePhase::kInitialHold);
  EXPECT_EQ(initial.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kInitialHold);
  EXPECT_FALSE(initial.command_available);

  episode.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 20U));
  auto tracking = episode.snapshot();
  EXPECT_EQ(tracking.phase, navigation_runtime::ExecutionEpisodePhase::kTrackingMain);
  EXPECT_EQ(tracking.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
  EXPECT_TRUE(tracking.command_available);
  EXPECT_EQ(tracking.active_generation, 20U);

  episode.sampledSafetyRoleObserved(
      navigation_planning::CandidateRole::kBackup, 20U);
  const auto backup = episode.snapshot();
  EXPECT_EQ(backup.phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingBackup);
  EXPECT_EQ(backup.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackBackup);
  EXPECT_TRUE(backup.safety_suffix_active);
  episode.stoppedHold(20U);
  episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kCertifiedStopObserved);
  const auto stopped = episode.snapshot();
  EXPECT_EQ(stopped.phase,
            navigation_runtime::ExecutionEpisodePhase::kStoppedHold);
  EXPECT_EQ(stopped.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kStoppedRecovery);
}

TEST(ExecutionEpisode, SampledSafetyRoleCannotRewriteAnotherGeneration) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, false);
  episode.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U));

  episode.sampledSafetyRoleObserved(
      navigation_planning::CandidateRole::kBackup, 5U);
  const auto state = episode.snapshot();
  EXPECT_EQ(state.active_generation, 4U);
  EXPECT_EQ(state.phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingMain);
  EXPECT_EQ(state.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
  EXPECT_FALSE(state.safety_suffix_active);
}

TEST(ExecutionEpisode, CommitUpdatesLifecycleAndRecoveryInOneSnapshot) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, false);

  auto main = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U);
  main.role = navigation_planning::CandidateRole::kMain;
  episode.commandCommitted(main);
  auto state = episode.snapshot();
  EXPECT_EQ(state.phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingMain);
  EXPECT_EQ(state.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);

  auto emergency = bundle(
      navigation_planning::CandidateBundleKind::kEmergencyBrake, 5U);
  emergency.role = navigation_planning::CandidateRole::kEmergency;
  episode.commandCommitted(emergency);
  state = episode.snapshot();
  EXPECT_EQ(state.phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingBackup);
  EXPECT_EQ(state.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kEmergencyBrake);
}

TEST(ExecutionEpisode, UsesContiguousInternalPhasesAndVersionedTelemetryConversion) {
  using navigation_runtime::ExecutionEpisodePhase;
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionEpisodePhase::kInitialHold), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionEpisodePhase::kTrackingMain), 1U);
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionEpisodePhase::kTrackingBackup), 2U);
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionEpisodePhase::kStoppedHold), 3U);
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionEpisodePhase::kPx4Hold), 4U);
  EXPECT_EQ(navigation_runtime::executionEpisodePhaseTelemetryCodeV1(
                ExecutionEpisodePhase::kInitialHold), 0U);
  EXPECT_EQ(navigation_runtime::executionEpisodePhaseTelemetryCodeV1(
                ExecutionEpisodePhase::kTrackingMain), 2U);
  EXPECT_EQ(navigation_runtime::executionEpisodePhaseTelemetryCodeV1(
                ExecutionEpisodePhase::kTrackingBackup), 3U);
  EXPECT_EQ(navigation_runtime::executionEpisodePhaseTelemetryCodeV1(
                ExecutionEpisodePhase::kStoppedHold), 4U);
  EXPECT_EQ(navigation_runtime::executionEpisodePhaseTelemetryCodeV1(
                ExecutionEpisodePhase::kPx4Hold), 5U);
}

TEST(ExecutionEpisode, FailClosedClearsCommandExposure) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, true);
  episode.failClosed();
  const auto state = episode.snapshot();
  EXPECT_EQ(state.phase, navigation_runtime::ExecutionEpisodePhase::kPx4Hold);
  EXPECT_EQ(state.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kPx4Hold);
  EXPECT_FALSE(state.command_available);
  EXPECT_TRUE(state.failure_latched);
}

TEST(ExecutionEpisode, ObservationsCannotResurrectFailClosedEpisode) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, true);
  episode.failClosed();

  episode.roleObserved(navigation_planning::CandidateRole::kBackup, 7U);
  episode.setSafetySuffix(true);
  episode.requestRestartFromRest();
  episode.stoppedHold(7U);
  episode.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 8U));

  const auto state = episode.snapshot();
  EXPECT_EQ(state.phase, navigation_runtime::ExecutionEpisodePhase::kPx4Hold);
  EXPECT_FALSE(state.command_available);
  EXPECT_TRUE(state.failure_latched);
  EXPECT_FALSE(state.safety_suffix_active);
  EXPECT_FALSE(state.restart_from_rest);
  EXPECT_EQ(state.active_generation, 0U);
  EXPECT_EQ(state.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kPx4Hold);
}

TEST(ExecutionEpisode, StoppedHoldPreservesMeasuredRestartRequest) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, true);
  episode.requestRestartFromRest();

  episode.stoppedHold(4U);
  const auto state = episode.snapshot();
  EXPECT_EQ(state.phase, navigation_runtime::ExecutionEpisodePhase::kStoppedHold);
  EXPECT_TRUE(state.command_available);
  EXPECT_TRUE(state.restart_from_rest);
}

TEST(ExecutionEpisode, SuspendAndClearDoNotRetainCommandIdentity) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(8U, 9U, 10U, true);
  episode.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 31U));

  episode.suspendCommand();
  auto suspended = episode.snapshot();
  EXPECT_FALSE(suspended.command_available);
  EXPECT_EQ(suspended.active_generation, 31U);
  EXPECT_EQ(suspended.goal_epoch, 9U);

  episode.clearGoal(12U);
  const auto cleared = episode.snapshot();
  EXPECT_EQ(cleared.localization_epoch, 12U);
  EXPECT_EQ(cleared.goal_epoch, 0U);
  EXPECT_EQ(cleared.request_id, 0U);
  EXPECT_EQ(cleared.active_generation, 0U);
  EXPECT_FALSE(cleared.command_available);
  EXPECT_FALSE(cleared.failure_latched);
  EXPECT_EQ(cleared.phase,
            navigation_runtime::ExecutionEpisodePhase::kInitialHold);
  EXPECT_EQ(cleared.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kInitialHold);
}

TEST(ExecutionEpisode, RecoveryEventsRemainOneWayInsideTheLifecycleRecord) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, true);
  episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kEmergencyCommitted);
  episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kMainCommitted);
  EXPECT_EQ(episode.snapshot().recovery_state,
            navigation_runtime::ExecutionRecoveryState::kEmergencyBrake);

  episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kCertifiedStopObserved);
  EXPECT_EQ(episode.snapshot().recovery_state,
            navigation_runtime::ExecutionRecoveryState::kStoppedRecovery);
  episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kMainCommitted);
  EXPECT_EQ(episode.snapshot().recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
}

}  // namespace
