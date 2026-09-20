#include <gtest/gtest.h>

#include <navigation_runtime/execution_episode.hpp>

namespace {

navigation_planning::CandidateBundle bundle(
    navigation_planning::CandidateBundleKind kind,
    std::uint64_t generation,
    std::uint64_t localization_epoch = 1U,
    std::uint64_t goal_epoch = 2U,
    std::uint64_t request_id = 3U) {
  navigation_planning::CandidateBundle value;
  value.kind = kind;
  value.bundle_generation = generation;
  value.localization_epoch = localization_epoch;
  value.goal_epoch = goal_epoch;
  value.request_id = request_id;
  value.role = kind == navigation_planning::CandidateBundleKind::kBackupOnly
      ? navigation_planning::CandidateRole::kBackup
      : kind == navigation_planning::CandidateBundleKind::kEmergencyBrake
      ? navigation_planning::CandidateRole::kEmergency
      : navigation_planning::CandidateRole::kMain;
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

  const auto active = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup,
      20U, 4U, 7U, 11U);
  ASSERT_TRUE(episode.commandCommitted(active));
  auto tracking = episode.snapshot();
  EXPECT_EQ(tracking.phase, navigation_runtime::ExecutionEpisodePhase::kTrackingMain);
  EXPECT_EQ(tracking.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
  EXPECT_TRUE(tracking.command_available);
  EXPECT_EQ(tracking.active_generation, 20U);

  ASSERT_TRUE(episode.observeSampledSafetyRole(
      active, navigation_planning::CandidateRole::kBackup));
  const auto backup = episode.snapshot();
  EXPECT_EQ(backup.phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingBackup);
  EXPECT_EQ(backup.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackBackup);
  EXPECT_TRUE(backup.safety_suffix_active);
  ASSERT_TRUE(episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kCertifiedStopObserved,
      active));
  ASSERT_TRUE(episode.stoppedHold(active));
  const auto stopped = episode.snapshot();
  EXPECT_EQ(stopped.phase,
            navigation_runtime::ExecutionEpisodePhase::kStoppedHold);
  EXPECT_EQ(stopped.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kStoppedRecovery);
}

TEST(ExecutionEpisode, SampledSafetyRoleCannotRewriteAnotherGeneration) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, false);
  const auto active = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U);
  ASSERT_TRUE(episode.commandCommitted(active));

  EXPECT_FALSE(episode.observeSampledSafetyRole(
      bundle(navigation_planning::CandidateBundleKind::kMainWithBackup, 5U),
      navigation_planning::CandidateRole::kBackup));
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
  ASSERT_TRUE(episode.commandCommitted(main));
  auto state = episode.snapshot();
  EXPECT_EQ(state.phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingMain);
  EXPECT_EQ(state.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);

  auto emergency = bundle(
      navigation_planning::CandidateBundleKind::kEmergencyBrake, 5U);
  emergency.role = navigation_planning::CandidateRole::kEmergency;
  ASSERT_TRUE(episode.commandCommitted(emergency));
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

  const auto stale = bundle(
      navigation_planning::CandidateBundleKind::kBackupOnly, 7U);
  EXPECT_FALSE(episode.observeRetainedCommand(stale, true));
  EXPECT_FALSE(episode.preserveSafetySuffix(stale));
  EXPECT_FALSE(episode.requestRestartFromRest(stale));
  EXPECT_FALSE(episode.stoppedHold(stale));
  EXPECT_FALSE(episode.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 8U)));

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
  const auto active = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U);
  ASSERT_TRUE(episode.commandCommitted(active));
  ASSERT_TRUE(episode.requestRestartFromRest(active));
  ASSERT_TRUE(episode.stoppedHold(active));
  const auto state = episode.snapshot();
  EXPECT_EQ(state.phase, navigation_runtime::ExecutionEpisodePhase::kStoppedHold);
  EXPECT_TRUE(state.command_available);
  EXPECT_TRUE(state.restart_from_rest);
}

TEST(ExecutionEpisode, HotRetargetPreservesDrainingSuffixUntilSuccessorCommit) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, false);
  const auto active_backup = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U,
      1U, 2U, 3U);
  ASSERT_TRUE(episode.commandCommitted(active_backup));
  ASSERT_TRUE(episode.observeSampledSafetyRole(
      active_backup, navigation_planning::CandidateRole::kBackup));
  ASSERT_TRUE(episode.requestRestartFromRest(active_backup));

  episode.beginGoal(1U, 5U, 6U, true);
  auto state = episode.snapshot();
  EXPECT_EQ(state.goal_epoch, 5U);
  EXPECT_EQ(state.request_id, 6U);
  EXPECT_EQ(state.active_goal_epoch, 2U);
  EXPECT_EQ(state.active_request_id, 3U);
  EXPECT_EQ(state.active_generation, 4U);
  EXPECT_EQ(state.phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingBackup);
  EXPECT_TRUE(state.safety_suffix_active);
  EXPECT_TRUE(state.restart_from_rest);
  EXPECT_EQ(state.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackBackup);

  const auto successor = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 5U,
      1U, 5U, 6U);
  EXPECT_FALSE(episode.commandCommitted(successor));
  auto late_main_sample = active_backup;
  late_main_sample.role = navigation_planning::CandidateRole::kMain;
  EXPECT_TRUE(episode.observeRetainedCommand(late_main_sample, false));
  state = episode.snapshot();
  EXPECT_EQ(state.phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingBackup);
  EXPECT_TRUE(state.safety_suffix_active);
  EXPECT_EQ(state.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackBackup);
  EXPECT_FALSE(episode.commandCommitted(active_backup));

  ASSERT_TRUE(episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kCertifiedStopObserved,
      active_backup));
  EXPECT_FALSE(episode.snapshot().safety_suffix_active);
  EXPECT_EQ(episode.snapshot().phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingBackup);
  ASSERT_TRUE(episode.stoppedHold(active_backup));
  EXPECT_TRUE(episode.snapshot().restart_from_rest);
  ASSERT_TRUE(episode.commandCommitted(successor));

  EXPECT_FALSE(episode.observeRetainedCommand(active_backup, true));
  EXPECT_FALSE(episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kEmergencyCommitted,
      active_backup));
  state = episode.snapshot();
  EXPECT_EQ(state.goal_epoch, 5U);
  EXPECT_EQ(state.request_id, 6U);
  EXPECT_EQ(state.active_goal_epoch, 5U);
  EXPECT_EQ(state.active_request_id, 6U);
  EXPECT_EQ(state.active_generation, 5U);
  EXPECT_EQ(state.phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingMain);
  EXPECT_EQ(state.recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
  EXPECT_FALSE(state.safety_suffix_active);
  EXPECT_FALSE(state.restart_from_rest);
}

TEST(ExecutionEpisode, SuspendAndClearDoNotRetainCommandIdentity) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(8U, 9U, 10U, true);
  ASSERT_TRUE(episode.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 31U,
      8U, 9U, 10U)));

  EXPECT_TRUE(episode.suspendCommand(episode.snapshot()));
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
  const auto emergency = bundle(
      navigation_planning::CandidateBundleKind::kEmergencyBrake, 4U);
  ASSERT_TRUE(episode.commandCommitted(emergency));
  EXPECT_TRUE(episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kEmergencyCommitted,
      emergency));
  EXPECT_TRUE(episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kMainCommitted,
      emergency));
  EXPECT_EQ(episode.snapshot().recovery_state,
            navigation_runtime::ExecutionRecoveryState::kEmergencyBrake);

  EXPECT_TRUE(episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kCertifiedStopObserved,
      emergency));
  EXPECT_EQ(episode.snapshot().recovery_state,
            navigation_runtime::ExecutionRecoveryState::kStoppedRecovery);
  EXPECT_TRUE(episode.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kMainCommitted,
      emergency));
  EXPECT_EQ(episode.snapshot().recovery_state,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
}

}  // namespace
