#include <gtest/gtest.h>

#include "execution_authority_lifecycle_fixture.hpp"

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

TEST(ExecutionLifecycle, KeepsOneAuthoritativeLifecycleSnapshot) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.reset(4U);
  authority.beginGoal(4U, 7U, 11U, false);
  auto initial = authority.snapshot();
  EXPECT_EQ(initial.admission_localization_epoch, 4U);
  EXPECT_EQ(initial.admission_goal_epoch, 7U);
  // Desired request lives in MissionProgress; no active bundle exists yet.
  EXPECT_EQ(initial.admissionRequestId(), 0U);
  EXPECT_EQ(initial.lifecycle.phase, navigation_runtime::ExecutionPhase::kInitialHold);
  EXPECT_EQ(initial.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kInitialHold);
  EXPECT_FALSE(initial.commandAvailable());

  authority.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 20U,
      4U, 7U, 11U));
  auto tracking = authority.snapshot();
  EXPECT_EQ(tracking.lifecycle.phase, navigation_runtime::ExecutionPhase::kTrackingMain);
  EXPECT_EQ(tracking.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
  EXPECT_TRUE(tracking.commandAvailable());
  EXPECT_EQ(tracking.activeGeneration(), 20U);

  const auto active_main = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup,
      20U, 4U, 7U, 11U);
  EXPECT_TRUE(authority.observeSampledSafetyRole(
      active_main,
      navigation_planning::CandidateRole::kBackup));
  const auto backup = authority.snapshot();
  EXPECT_EQ(backup.lifecycle.phase,
            navigation_runtime::ExecutionPhase::kTrackingBackup);
  EXPECT_EQ(backup.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kTrackBackup);
  EXPECT_TRUE(backup.safetySuffixActive());
  EXPECT_TRUE(authority.stoppedHold(active_main));
  EXPECT_TRUE(authority.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kCertifiedStopObserved,
      active_main));
  const auto stopped = authority.snapshot();
  EXPECT_EQ(stopped.lifecycle.phase,
            navigation_runtime::ExecutionPhase::kStoppedHold);
  EXPECT_EQ(stopped.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kStoppedRecovery);
}

TEST(ExecutionLifecycle, SampledSafetyRoleCannotRewriteAnotherGeneration) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, false);
  authority.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U));

  EXPECT_FALSE(authority.observeSampledSafetyRole(
      bundle(navigation_planning::CandidateBundleKind::kMainWithBackup, 5U),
      navigation_planning::CandidateRole::kBackup));
  const auto state = authority.snapshot();
  EXPECT_EQ(state.activeGeneration(), 4U);
  EXPECT_EQ(state.lifecycle.phase,
            navigation_runtime::ExecutionPhase::kTrackingMain);
  EXPECT_EQ(state.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
  EXPECT_FALSE(state.safetySuffixActive());
}

TEST(ExecutionLifecycle, LateRetainedRoleFromAStaleCandidateCannotRewriteSuccessor) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, false);
  const auto candidate_a = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U);
  authority.commandCommitted(candidate_a);
  authority.beginGoal(1U, 5U, 6U, true);
  const auto candidate_b = bundle(
      navigation_planning::CandidateBundleKind::kBackupOnly, 7U,
      1U, 5U, 6U);
  EXPECT_EQ(authority.commandCommitted(candidate_b),
            navigation_execution::CommitDecision::kCommitted);

  EXPECT_FALSE(authority.observeRetainedCommand(candidate_a, true));
  EXPECT_FALSE(authority.observeSampledSafetyRole(
      candidate_a, navigation_planning::CandidateRole::kEmergency));
  EXPECT_FALSE(authority.preserveSafetySuffix(candidate_a));
  EXPECT_FALSE(authority.requestRestartFromRest(candidate_a));
  const auto state = authority.snapshot();
  EXPECT_EQ(state.admission_goal_epoch, 5U);
  EXPECT_EQ(state.admissionRequestId(), 6U);
  EXPECT_EQ(state.activeGeneration(), candidate_b.bundle_generation);
  EXPECT_EQ(state.lifecycle.phase,
            navigation_runtime::ExecutionPhase::kTrackingBackup);
  EXPECT_FALSE(state.restartFromRest());
  EXPECT_EQ(state.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kTrackBackup);
}

TEST(ExecutionLifecycle, HotRetargetKeepsActiveBackupUntilSuccessorCommits) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, false);
  const auto candidate_a = bundle(
      navigation_planning::CandidateBundleKind::kBackupOnly, 4U);
  authority.commandCommitted(candidate_a);

  // Goal B is desired, but A still owns the moving command while B is being
  // planned. A failed/stale B solve must not erase A's recovery phase.
  authority.beginGoal(1U, 5U, 6U, true);
  auto state = authority.snapshot();
  EXPECT_EQ(state.admission_goal_epoch, 5U);
  // Admission revision advanced, while the active request remains 3.
  EXPECT_EQ(state.admissionRequestId(), 0U);
  EXPECT_EQ(state.activeGoalEpoch(), 2U);
  EXPECT_EQ(state.activeRequestId(), 3U);
  EXPECT_EQ(state.activeGeneration(), candidate_a.bundle_generation);
  EXPECT_EQ(state.lifecycle.phase,
            navigation_runtime::ExecutionPhase::kTrackingBackup);
  EXPECT_TRUE(state.commandAvailable());
  EXPECT_TRUE(state.safetySuffixActive());
  EXPECT_TRUE(authority.observeSampledSafetyRole(
      candidate_a, navigation_planning::CandidateRole::kBackup));
  EXPECT_TRUE(authority.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kCertifiedStopObserved,
      candidate_a));

  const auto candidate_b = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 5U,
      1U, 5U, 6U);
  authority.commandCommitted(candidate_b);
  state = authority.snapshot();
  EXPECT_EQ(state.activeGoalEpoch(), 5U);
  EXPECT_EQ(state.activeRequestId(), 6U);
  EXPECT_EQ(state.activeGeneration(), candidate_b.bundle_generation);
  EXPECT_EQ(state.lifecycle.phase,
            navigation_runtime::ExecutionPhase::kTrackingMain);
  EXPECT_FALSE(state.safetySuffixActive());
  EXPECT_FALSE(authority.observeSampledSafetyRole(
      candidate_a, navigation_planning::CandidateRole::kBackup));
  EXPECT_EQ(authority.snapshot().activeGeneration(), candidate_b.bundle_generation);
}

TEST(ExecutionLifecycle, CommitUpdatesLifecycleAndRecoveryInOneSnapshot) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, false);

  auto main = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U);
  main.role = navigation_planning::CandidateRole::kMain;
  authority.commandCommitted(main);
  auto state = authority.snapshot();
  EXPECT_EQ(state.lifecycle.phase,
            navigation_runtime::ExecutionPhase::kTrackingMain);
  EXPECT_EQ(state.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);

  auto emergency = bundle(
      navigation_planning::CandidateBundleKind::kEmergencyBrake, 5U);
  emergency.role = navigation_planning::CandidateRole::kEmergency;
  authority.commandCommitted(emergency);
  state = authority.snapshot();
  EXPECT_EQ(state.lifecycle.phase,
            navigation_runtime::ExecutionPhase::kTrackingBackup);
  EXPECT_EQ(state.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kEmergencyBrake);
}

TEST(ExecutionLifecycle, UsesContiguousInternalPhasesAndVersionedTelemetryConversion) {
  using navigation_runtime::ExecutionPhase;
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionPhase::kInitialHold), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionPhase::kTrackingMain), 1U);
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionPhase::kTrackingBackup), 2U);
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionPhase::kStoppedHold), 3U);
  EXPECT_EQ(static_cast<std::uint8_t>(ExecutionPhase::kPx4Hold), 4U);
  EXPECT_EQ(navigation_runtime::executionPhaseTelemetryCodeV1(
                ExecutionPhase::kInitialHold), 0U);
  EXPECT_EQ(navigation_runtime::executionPhaseTelemetryCodeV1(
                ExecutionPhase::kTrackingMain), 2U);
  EXPECT_EQ(navigation_runtime::executionPhaseTelemetryCodeV1(
                ExecutionPhase::kTrackingBackup), 3U);
  EXPECT_EQ(navigation_runtime::executionPhaseTelemetryCodeV1(
                ExecutionPhase::kStoppedHold), 4U);
  EXPECT_EQ(navigation_runtime::executionPhaseTelemetryCodeV1(
                ExecutionPhase::kPx4Hold), 5U);
}

TEST(ExecutionLifecycle, FailClosedClearsCommandExposure) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, true);
  authority.failClosed();
  const auto state = authority.snapshot();
  EXPECT_EQ(state.lifecycle.phase, navigation_runtime::ExecutionPhase::kPx4Hold);
  EXPECT_EQ(state.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kPx4Hold);
  EXPECT_FALSE(state.commandAvailable());
  EXPECT_TRUE(state.failed());
}

TEST(ExecutionLifecycle, ObservationsCannotResurrectFailClosedExecution) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, true);
  authority.failClosed();

  const auto stale = bundle(
      navigation_planning::CandidateBundleKind::kBackupOnly, 7U);
  EXPECT_FALSE(authority.observeRetainedCommand(
      stale, true));
  EXPECT_FALSE(authority.preserveSafetySuffix(stale));
  EXPECT_FALSE(authority.requestRestartFromRest(stale));
  EXPECT_FALSE(authority.stoppedHold(stale));
  authority.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 8U));

  const auto state = authority.snapshot();
  EXPECT_EQ(state.lifecycle.phase, navigation_runtime::ExecutionPhase::kPx4Hold);
  EXPECT_FALSE(state.commandAvailable());
  EXPECT_TRUE(state.failed());
  EXPECT_FALSE(state.safetySuffixActive());
  EXPECT_FALSE(state.restartFromRest());
  EXPECT_EQ(state.activeGeneration(), 0U);
  EXPECT_EQ(state.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kPx4Hold);
}

TEST(ExecutionLifecycle, StoppedHoldPreservesMeasuredRestartRequest) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, true);

  const auto active = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U);
  authority.commandCommitted(active);
  EXPECT_TRUE(authority.requestRestartFromRest(active));
  EXPECT_TRUE(authority.stoppedHold(active));
  const auto state = authority.snapshot();
  EXPECT_EQ(state.lifecycle.phase, navigation_runtime::ExecutionPhase::kStoppedHold);
  EXPECT_TRUE(state.commandAvailable());
  EXPECT_TRUE(state.restartFromRest());
}

TEST(ExecutionLifecycle, LateCommitCannotRollbackSameRequestGeneration) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, false);
  const auto candidate_a = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U);
  authority.commandCommitted(candidate_a);
  const auto candidate_b = bundle(
      navigation_planning::CandidateBundleKind::kBackupOnly, 5U);
  EXPECT_EQ(authority.commandCommitted(candidate_b),
            navigation_execution::CommitDecision::kCommitted);

  authority.commandCommitted(candidate_a);
  const auto state = authority.snapshot();
  EXPECT_EQ(state.activeGeneration(), candidate_b.bundle_generation);
  EXPECT_EQ(state.lifecycle.phase, navigation_runtime::ExecutionPhase::kTrackingBackup);
  EXPECT_TRUE(state.safetySuffixActive());
  EXPECT_EQ(state.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kTrackBackup);
}

TEST(ExecutionLifecycle, SameGenerationForeignIdentityCannotMutateOwner) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, false);
  const auto active = bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 4U);
  authority.commandCommitted(active);

  auto foreign = active;
  foreign.request_id = 99U;
  EXPECT_FALSE(authority.observeRetainedCommand(foreign, true));
  EXPECT_FALSE(authority.observeSampledSafetyRole(
      foreign, navigation_planning::CandidateRole::kBackup));
  EXPECT_FALSE(authority.preserveSafetySuffix(foreign));
  EXPECT_FALSE(authority.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kEmergencyCommitted,
      foreign));
  const auto after = authority.snapshot();
  EXPECT_EQ(after.admissionRequestId(), active.request_id);
  EXPECT_EQ(after.activeGeneration(), active.bundle_generation);
  EXPECT_EQ(after.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
  EXPECT_FALSE(after.safetySuffixActive());
}

TEST(ExecutionLifecycle, SuspendAndClearDoNotRetainCommandIdentity) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(8U, 9U, 10U, true);
  authority.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 31U,
      8U, 9U, 10U));

  authority.suspendCommand();
  auto suspended = authority.snapshot();
  EXPECT_FALSE(suspended.commandAvailable());
  EXPECT_EQ(suspended.activeGeneration(), 31U);
  EXPECT_EQ(suspended.admission_goal_epoch, 9U);

  authority.clearGoal(12U);
  const auto cleared = authority.snapshot();
  EXPECT_EQ(cleared.admission_localization_epoch, 12U);
  EXPECT_EQ(cleared.admission_goal_epoch, 0U);
  EXPECT_EQ(cleared.admissionRequestId(), 0U);
  EXPECT_EQ(cleared.activeGeneration(), 0U);
  EXPECT_FALSE(cleared.commandAvailable());
  EXPECT_FALSE(cleared.failed());
  EXPECT_EQ(cleared.lifecycle.phase,
            navigation_runtime::ExecutionPhase::kInitialHold);
  EXPECT_EQ(cleared.lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kInitialHold);
}

TEST(ExecutionLifecycle, RecoveryEventsRemainOneWayInsideTheLifecycleRecord) {
  navigation_runtime::ExecutionLifecycleFixture authority;
  authority.beginGoal(1U, 2U, 3U, true);
  const auto active = bundle(
      navigation_planning::CandidateBundleKind::kEmergencyBrake, 4U);
  authority.commandCommitted(active);
  EXPECT_TRUE(authority.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kEmergencyCommitted,
      active));
  EXPECT_TRUE(authority.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kMainCommitted,
      active));
  EXPECT_EQ(authority.snapshot().lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kEmergencyBrake);

  EXPECT_TRUE(authority.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kCertifiedStopObserved,
      active));
  EXPECT_EQ(authority.snapshot().lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kStoppedRecovery);
  EXPECT_TRUE(authority.applyRecoveryEvent(
      navigation_runtime::ExecutionRecoveryEvent::kMainCommitted, active));
  EXPECT_EQ(authority.snapshot().lifecycle.recovery,
            navigation_runtime::ExecutionRecoveryState::kTrackMain);
}

}  // namespace
