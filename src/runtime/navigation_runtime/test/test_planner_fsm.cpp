#include "navigation_runtime/planner_fsm.hpp"
#include "navigation_runtime/commit_trace.hpp"
#include <navigation_planning/candidate_bundle.hpp>

#include <gtest/gtest.h>

namespace navigation_runtime {
namespace {

TEST(PlannerFsm, AcceptsSuccessfulPlannerResults) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kSuccess, true, false, true),
            PlannerResultDisposition::CommandReady);
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kNoNeed, false, true, false),
            PlannerResultDisposition::ValidateRetainedCommand);
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kFinished, false, true, true),
            PlannerResultDisposition::CommandReady);
}

TEST(PlannerFsm, ClearsForcedHotRetargetAfterEitherSuccessfulTransitionPath) {
  EXPECT_TRUE(clearHotGoalTransitionAfterCommit(true, false));
  EXPECT_TRUE(clearHotGoalTransitionAfterCommit(false, true));
  EXPECT_TRUE(clearHotGoalTransitionAfterCommit(true, true));
  EXPECT_FALSE(clearHotGoalTransitionAfterCommit(false, false));
}

TEST(PlannerFsm, DefersOptimizerWhileCertifiedMainHasRenewalMargin) {
  const auto decision = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 1.0, 4.0, 0.18, 0.2, 0.2);
  EXPECT_FALSE(decision.run_optimizer);
  EXPECT_EQ(decision.reason, PlannerRenewalReason::kRetainCertifiedMain);
  EXPECT_DOUBLE_EQ(decision.remaining_main_horizon_s, 3.0);
  EXPECT_NEAR(decision.required_lead_time_s, 0.58, 1.0e-15);
}

TEST(PlannerFsm, StartsAnchorRecoveryBeforeExecutionLeaseIsExhausted) {
  EXPECT_FALSE(commandAnchorRecoveryDue(
      false, navigation_planning::CandidateRole::kMain, 0.6, 0.75));
  EXPECT_FALSE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kBackup, 0.6, 0.75));
  EXPECT_TRUE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kMain, 0.375, 0.75));
  EXPECT_TRUE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kMain, 0.6, 0.75));
  EXPECT_FALSE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kMain,
      std::numeric_limits<double>::quiet_NaN(), 0.75));
}

TEST(PlannerFsm, StartsRecoveryBeforeCommandLeaseExpires) {
  EXPECT_FALSE(commandLeaseRenewalDue(true, 10'000'000'000LL,
                                      10'500'000'000LL, 0.49));
  EXPECT_TRUE(commandLeaseRenewalDue(true, 10'000'000'000LL,
                                     10'500'000'000LL, 0.50));
  EXPECT_TRUE(commandLeaseRenewalDue(true, 10'600'000'000LL,
                                     10'500'000'000LL, 0.20));
  EXPECT_FALSE(commandLeaseRenewalDue(false, 10'000'000'000LL,
                                      10'100'000'000LL, 0.20));
}

TEST(PlannerFsm, UsesBoundedSlowerVelocityEnvelopeAfterRestFailures) {
  EXPECT_DOUBLE_EQ(plannerRecoveryVelocityScale(0U), 1.0);
  EXPECT_DOUBLE_EQ(plannerRecoveryVelocityScale(1U), 0.75);
  EXPECT_DOUBLE_EQ(plannerRecoveryVelocityScale(2U), 0.50);
  EXPECT_DOUBLE_EQ(plannerRecoveryVelocityScale(3U), 0.35);
  EXPECT_DOUBLE_EQ(plannerRecoveryVelocityScale(100U), 0.35);
}

TEST(PlannerFsm, RenewsBeforeMainCanReachBackupDuringSchedulingAndSolve) {
  const auto before_boundary = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 3.41, 4.0, 0.18, 0.2, 0.2);
  EXPECT_FALSE(before_boundary.run_optimizer);

  const auto at_boundary = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 3.42, 4.0, 0.18, 0.2, 0.2);
  EXPECT_TRUE(at_boundary.run_optimizer);
  EXPECT_EQ(at_boundary.reason, PlannerRenewalReason::kRenewalDue);
}

TEST(PlannerFsm, NeverDefersRequiredTransitionsOrSafetyRecovery) {
  EXPECT_EQ(classifyPlannerRenewal(
                true, true, false, navigation_planning::CandidateRole::kMain,
                true, 0.0, 10.0, 0.18, 0.2, 0.2).reason,
            PlannerRenewalReason::kForcedTransition);
  EXPECT_EQ(classifyPlannerRenewal(
                false, false, false, navigation_planning::CandidateRole::kMain,
                false, 0.0, 0.0, 0.18, 0.2, 0.2).reason,
            PlannerRenewalReason::kNoCommand);
  EXPECT_EQ(classifyPlannerRenewal(
                false, true, true, navigation_planning::CandidateRole::kMain,
                true, 0.0, 10.0, 0.18, 0.2, 0.2).reason,
            PlannerRenewalReason::kSafetyRecovery);
  EXPECT_EQ(classifyPlannerRenewal(
                false, true, false, navigation_planning::CandidateRole::kEmergency,
                true, 0.0, 10.0, 0.18, 0.2, 0.2).reason,
            PlannerRenewalReason::kSafetyRecovery);
}

TEST(PlannerFsm, InvalidRenewalEvidenceRunsOptimizerFailClosed) {
  EXPECT_EQ(classifyPlannerRenewal(
                false, true, false, navigation_planning::CandidateRole::kMain,
                false, 0.0, 10.0, 0.18, 0.2, 0.2).reason,
            PlannerRenewalReason::kInvalidHorizon);
  EXPECT_EQ(classifyPlannerRenewal(
                false, true, false, navigation_planning::CandidateRole::kMain,
                true, std::numeric_limits<double>::quiet_NaN(), 10.0,
                0.18, 0.2, 0.2).reason,
            PlannerRenewalReason::kInvalidHorizon);
  EXPECT_EQ(classifyPlannerRenewal(
                false, true, false, navigation_planning::CandidateRole::kMain,
                true, 0.0, 10.0, 0.18, 0.2,
                std::numeric_limits<double>::infinity()).reason,
            PlannerRenewalReason::kInvalidHorizon);
}

TEST(PlannerFsm, NoNeedWithoutCommittedCommandFailsClosed) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kNoNeed, false, false, false),
            PlannerResultDisposition::FailClosed);
}

TEST(PlannerFsm, RetainedValidationPreservesValidStateAndFailsClosedOtherwise) {
  EXPECT_EQ(retainedValidationTransition(true),
            RetainedValidationTransition::PreserveExistingState);
  EXPECT_EQ(retainedValidationTransition(false),
            RetainedValidationTransition::FailClosed);
}

TEST(PlannerFsm, EmergencyBrakeCannotBeRearmedFromADriftingEmergency) {
  EXPECT_TRUE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true,
      navigation_planning::CandidateRole::kMain));
  EXPECT_TRUE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true,
      navigation_planning::CandidateRole::kBackup));
  EXPECT_FALSE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true,
      navigation_planning::CandidateRole::kEmergency));
  EXPECT_FALSE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, true, true, true, true,
      navigation_planning::CandidateRole::kMain));
}

TEST(PlannerFsm, PreservesObservedTerminalHoldAcrossRestRetry) {
  EXPECT_TRUE(terminalHoldIsPending(true, true, true, 17U));
  EXPECT_FALSE(terminalHoldIsPending(false, true, true, 17U));
  EXPECT_FALSE(terminalHoldIsPending(true, false, true, 17U));
  EXPECT_FALSE(terminalHoldIsPending(true, true, true, 0U));
  EXPECT_FALSE(terminalHoldIsPending(true, true, false, 17U));
}

TEST(PlannerFsm, ContinuesOnlyCompletedPassThroughGoalEndpoints) {
  EXPECT_TRUE(completedPassThroughRequiresContinuation(true, true, true));
  EXPECT_FALSE(completedPassThroughRequiresContinuation(false, true, true));
  EXPECT_FALSE(completedPassThroughRequiresContinuation(true, false, true));
  EXPECT_FALSE(completedPassThroughRequiresContinuation(true, true, false));
}

TEST(PlannerFsm, CertifiesMissedTerminalTickOnlyAtKnownFreeGoalEndpoint) {
  EXPECT_TRUE(terminalEndpointHoldIsCertified(true, true, true));
  EXPECT_FALSE(terminalEndpointHoldIsCertified(false, true, true));
  EXPECT_FALSE(terminalEndpointHoldIsCertified(true, false, true));
  EXPECT_FALSE(terminalEndpointHoldIsCertified(true, true, false));
}

TEST(PlannerFsm, ReplaysOnlyKnownFreeFrontierEndpointNearMeasuredState) {
  EXPECT_TRUE(expiredEndpointMayBeReplayed(true, true, true));
  EXPECT_FALSE(expiredEndpointMayBeReplayed(false, true, true));
  EXPECT_FALSE(expiredEndpointMayBeReplayed(true, false, true));
  EXPECT_FALSE(expiredEndpointMayBeReplayed(true, true, false));
}

TEST(PlannerFsm, ResumesOnlyExactFreshWorldRecertifiedGeneration) {
  EXPECT_TRUE(worldFreshnessSuspendedCommandMayResume(
      17U, 17U, 3U, 5U, 3U, 5U, 10'500, 10'000,
      true, false, true));
  EXPECT_FALSE(worldFreshnessSuspendedCommandMayResume(
      17U, 18U, 3U, 5U, 3U, 5U, 10'500, 10'000,
      true, false, true));
  EXPECT_FALSE(worldFreshnessSuspendedCommandMayResume(
      17U, 17U, 3U, 5U, 3U, 6U, 10'500, 10'000,
      true, false, true));
  EXPECT_FALSE(worldFreshnessSuspendedCommandMayResume(
      17U, 17U, 3U, 5U, 3U, 5U, 9'999, 10'000,
      true, false, true));
  EXPECT_FALSE(worldFreshnessSuspendedCommandMayResume(
      17U, 17U, 3U, 5U, 3U, 5U, 10'500, 10'000,
      true, true, true));
  EXPECT_FALSE(worldFreshnessSuspendedCommandMayResume(
      17U, 17U, 3U, 5U, 3U, 5U, 10'500, 10'000,
      true, false, false));
}

TEST(PlannerFsm, SupersedingCertifiedBundleDoesNotRevokeCommandAvailability) {
  EXPECT_TRUE(supersedingBundleMayRemainAvailable(
      41, 41, 7, 9, 7, 9, 1200, 1100, true, false, true));
  EXPECT_TRUE(supersedingBundleMayRemainAvailable(
      41, 42, 7, 9, 7, 9, 1200, 1100, true, false, true));

  EXPECT_FALSE(supersedingBundleMayRemainAvailable(
      41, 40, 7, 9, 7, 9, 1200, 1100, true, false, true));
  EXPECT_FALSE(supersedingBundleMayRemainAvailable(
      41, 41, 6, 9, 7, 9, 1200, 1100, true, false, true));
  EXPECT_FALSE(supersedingBundleMayRemainAvailable(
      41, 41, 7, 8, 7, 9, 1200, 1100, true, false, true));
  EXPECT_FALSE(supersedingBundleMayRemainAvailable(
      41, 41, 7, 9, 7, 9, 1099, 1100, true, false, true));
  EXPECT_FALSE(supersedingBundleMayRemainAvailable(
      41, 41, 7, 9, 7, 9, 1200, 1100, false, false, true));
  EXPECT_FALSE(supersedingBundleMayRemainAvailable(
      41, 41, 7, 9, 7, 9, 1200, 1100, true, true, true));
  EXPECT_FALSE(supersedingBundleMayRemainAvailable(
      41, 41, 7, 9, 7, 9, 1200, 1100, true, false, false));
}

TEST(PlannerFsm, SuccessWithoutNewCommittedGenerationFailsClosed) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kSuccess, false, true, false),
            PlannerResultDisposition::FailClosed);
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kFinished, true, false, false),
            PlannerResultDisposition::FailClosed);
}

TEST(PlannerFsm, AttributesOnlyTheCommitProducedByThisSolveCycle) {
  EXPECT_FALSE(commitObservedThisCycle(4U, 4U, 4U));
  EXPECT_TRUE(commitObservedThisCycle(4U, 5U, 5U));
  EXPECT_FALSE(commitObservedThisCycle(4U, 5U, 4U));
  EXPECT_FALSE(commitObservedThisCycle(5U, 4U, 4U));
}

TEST(PlannerFsm, ExecutionAgeUsesDeclaredSolveStartInstant) {
  EXPECT_DOUBLE_EQ(executionStateAgeMs(1'250'000'000LL, 1'000'000'000LL), 250.0);
  EXPECT_DOUBLE_EQ(executionStateAgeMs(900'000'000LL, 1'000'000'000LL), -100.0);
  EXPECT_TRUE(std::isnan(executionStateAgeMs(
      std::numeric_limits<std::int64_t>::max(),
      std::numeric_limits<std::int64_t>::min())));
}

TEST(PlannerFsm, SamplesDeclaredTerminalCandidateBeyondExecutionLease) {
  navigation_planning::CandidateBundle candidate;
  candidate.world_identity.localization_epoch = 1U;
  candidate.world_identity.generation = 1U;
  candidate.world_identity.revision = 1U;
  candidate.world_identity.observation_stamp_ns = 10000000000LL;
  candidate.pinned_world_identity = candidate.world_identity;
  candidate.localization_epoch = 1U;
  candidate.goal_epoch = 1U;
  candidate.request_id = 1U;
  candidate.bundle_generation = 1U;
  candidate.start_wall_time_s = 10.0;
  candidate.duration_s = 1.0;
  candidate.backup_start_time_s = 0.0;
  candidate.valid_from_ns = 10000000000LL;
  candidate.valid_until_ns = 10500000000LL;
  candidate.kind = navigation_planning::CandidateBundleKind::kTerminalStop;
  candidate.certificates = {true, true, true, true};
  candidate.protected_region.minimum = Eigen::Vector3d::Zero();
  candidate.protected_region.maximum = Eigen::Vector3d::Ones();
  candidate.role_schedule = {
      {0.0, 1.0, navigation_planning::CandidateRole::kMain}};
  candidate.evaluator = [](const std::int64_t stamp_ns,
                           navigation_planning::TrajectoryPoint& point) {
    point.position_world = Eigen::Vector3d{7.0, 0.0, 3.0};
    point.trajectory_time_s = static_cast<double>(stamp_ns - 10000000000LL) * 1.0e-9;
    return true;
  };

  EXPECT_FALSE(candidate.sample(11000000000LL).has_value());
  const auto endpoint = candidate.sampleAtDeclaredEnd();
  ASSERT_TRUE(endpoint.has_value());
  EXPECT_NEAR(endpoint->position_world.x(), 7.0, 1.0e-12);
  EXPECT_TRUE(endpoint->finished);
}

TEST(PlannerFsm, SamplesDeclaredTerminalMainOnlyCandidateWithoutBackupMetadata) {
  navigation_planning::CandidateBundle candidate;
  candidate.world_identity.localization_epoch = 1U;
  candidate.world_identity.generation = 1U;
  candidate.world_identity.revision = 1U;
  candidate.world_identity.observation_stamp_ns = 10000000000LL;
  candidate.pinned_world_identity = candidate.world_identity;
  candidate.localization_epoch = 1U;
  candidate.goal_epoch = 1U;
  candidate.request_id = 1U;
  candidate.bundle_generation = 1U;
  candidate.start_wall_time_s = 10.0;
  candidate.duration_s = 1.0;
  candidate.valid_from_ns = 10000000000LL;
  candidate.valid_until_ns = 10500000000LL;
  candidate.kind = navigation_planning::CandidateBundleKind::kTerminalStop;
  candidate.certificates = {true, true, true, true};
  candidate.protected_region.minimum = Eigen::Vector3d::Zero();
  candidate.protected_region.maximum = Eigen::Vector3d::Ones();
  candidate.role_schedule = {
      {0.0, 1.0, navigation_planning::CandidateRole::kMain}};
  candidate.evaluator = [](const std::int64_t,
                           navigation_planning::TrajectoryPoint& point) {
    point.position_world = Eigen::Vector3d{7.0, 0.0, 3.0};
    return true;
  };

  EXPECT_FALSE(candidate.hasTrajectoryMetadata());
  EXPECT_TRUE(candidate.hasDeclaredEndpointMetadata());
  const auto endpoint = candidate.sampleAtDeclaredEnd();
  ASSERT_TRUE(endpoint.has_value());
  EXPECT_TRUE(endpoint->finished);
}

TEST(PlannerFsm, RestartsAtLocalTrajectoryBoundary) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kRestartFromRest, false, true, false),
            PlannerResultDisposition::RestartFromRest);
}

TEST(PlannerFsm, RetriesTransientPlannerFailures) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kFailed, true, false, false),
            PlannerResultDisposition::RetryFromRest);
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kFailed, false, true, false),
            PlannerResultDisposition::RetainCommittedCommand);
}

TEST(PlannerFsm, MeasuredStateRestartRetainsCurrentCertifiedCommandOnSolveFailure) {
  EXPECT_EQ(classifyPlannerResult(
                navigation_planning::PlannerStatus::kFailed, true, true, false),
            PlannerResultDisposition::RetainCommittedCommand);
}

TEST(PlannerFsm, FailsClosedForEmergencyOrUnrecoverableFailures) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kFailed, false, false, false),
            PlannerResultDisposition::FailClosed);
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kEmergency, false, true, false),
            PlannerResultDisposition::FailClosed);
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kOptimizationFailed, true, false, false),
            PlannerResultDisposition::FailClosed);
}

TEST(PlannerFsm, RestToRestFailureBudgetExhaustsAtConfiguredLimit) {
  ConsecutiveFailureBudget budget(3U);

  EXPECT_FALSE(budget.recordFailure());
  EXPECT_EQ(budget.failureCount(), 1U);
  EXPECT_FALSE(budget.recordFailure());
  EXPECT_EQ(budget.failureCount(), 2U);
  EXPECT_TRUE(budget.recordFailure());
  EXPECT_TRUE(budget.exhausted());
  EXPECT_EQ(budget.failureCount(), 3U);
  EXPECT_TRUE(budget.recordFailure());
  EXPECT_EQ(budget.failureCount(), 3U);
}

TEST(PlannerFsm, RestToRestFailureBudgetResetsForSuccessfulOrNewGoal) {
  ConsecutiveFailureBudget budget(2U);
  EXPECT_FALSE(budget.recordFailure());
  budget.reset();
  EXPECT_EQ(budget.failureCount(), 0U);
  EXPECT_FALSE(budget.exhausted());
  EXPECT_FALSE(budget.recordFailure());
}

TEST(PlannerFsm, RestToRestFailureBudgetRejectsZeroLimit) {
  EXPECT_THROW(ConsecutiveFailureBudget(0U), std::invalid_argument);
}

TEST(PlannerFsm, AcceptsOnlyAContinuousValidCommittedSafetySuffix) {
  EXPECT_TRUE(committedSafetySuffixIsUsable(
      true, 1.0, 4.0, 2.0, 0.2, 0.75, true));
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      false, 1.0, 4.0, 2.0, 0.2, 0.75, true));
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      true, 1.0, 4.0, 0.5, 0.2, 0.75, true));
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      true, 1.0, 4.0, 2.0, 0.8, 0.75, true));
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      true, 1.0, 4.0, 2.0, 0.2, 0.75, false));
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      true, 4.0, 4.0, 4.0, 0.2, 0.75, true));
}

TEST(PlannerFsm, RetainedCommandCannotConsumeUncertifiedTrackingClearance) {
  EXPECT_DOUBLE_EQ(retainedCommandTrackingLimit(0.25, 0.75), 0.25);
  EXPECT_DOUBLE_EQ(retainedCommandTrackingLimit(1.0, 0.75), 0.75);
  EXPECT_TRUE(std::isnan(retainedCommandTrackingLimit(
      std::numeric_limits<double>::quiet_NaN(), 0.75)));

  // Generation 341 remained below the final 0.75 m execution rejection gate,
  // but at 0.301 m it had already consumed more tracking clearance than the
  // world certificate reserved. That state must trigger measured-state brake
  // recovery instead of retaining the detached nominal prefix.
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      true, 0.86, 8.361, 6.270, 0.301,
      retainedCommandTrackingLimit(0.25, 0.75), true));
}

TEST(PlannerFsm, RetainedCommandReservesClearanceUntilNextValidationBoundary) {
  EXPECT_DOUBLE_EQ(
      projectedRetainedAnchorErrorUpperBound(0.096, 0.8, 0.2), 0.256);
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      true, 0.64, 8.785, 7.681,
      projectedRetainedAnchorErrorUpperBound(0.096, 0.8, 0.2),
      retainedCommandTrackingLimit(0.25, 0.75), true));
  EXPECT_TRUE(committedSafetySuffixIsUsable(
      true, 0.64, 8.785, 7.681,
      projectedRetainedAnchorErrorUpperBound(0.096, 0.2, 0.2),
      retainedCommandTrackingLimit(0.25, 0.75), true));
  EXPECT_TRUE(std::isnan(projectedRetainedAnchorErrorUpperBound(
      0.1, 0.2, 0.0)));
}

TEST(PlannerFsm, RetainsVisibleMainOnlyTrajectoryAfterTransientReplanFailure) {
  EXPECT_TRUE(committedSafetySuffixIsUsable(
      false, 0.8, 1.395, 0.8, 0.187, 0.75, true));
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      false, 0.8, 1.395, 0.8, 0.187, 0.75, false));
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      false, 1.395, 1.395, 1.395, 0.187, 0.75, true));
}

TEST(PlannerFsm, PassThroughHotRetargetsFromCertifiedFiniteCommand) {
  EXPECT_TRUE(canHotRetargetAtWaypointTransition(false, true, true, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(true, true, true, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, false, true, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, true, false, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, true, true, true, false));
  EXPECT_TRUE(canHotRetargetAtWaypointTransition(false, true, true, false, true));
}

TEST(PlannerFsm, RebasesHotRetargetWhenThePreviousCommandIsAtItsBoundary) {
  EXPECT_TRUE(hotRetargetNeedsMeasuredStatePlan(
      true, true, false, 1.0, 1.1, 0.1, 0.2, 0.75));
  EXPECT_TRUE(hotRetargetNeedsMeasuredStatePlan(
      true, true, false, 1.2, 1.1, 0.1, 0.2, 0.75));
  EXPECT_FALSE(hotRetargetNeedsMeasuredStatePlan(
      true, true, true, 0.8, 1.1, 0.1, 0.2, 0.75));
  EXPECT_FALSE(hotRetargetNeedsMeasuredStatePlan(
      true, true, false, 0.8, 1.1, 0.1, 0.2, 0.75));
  EXPECT_FALSE(hotRetargetNeedsMeasuredStatePlan(
      false, true, true, 1.2, 1.1, 0.1, 0.2, 0.75));
  EXPECT_FALSE(hotRetargetNeedsMeasuredStatePlan(
      true, false, true, 1.2, 1.1, 0.1, 0.2, 0.75));
  EXPECT_FALSE(hotRetargetNeedsMeasuredStatePlan(
      true, true, false, 1.2, 1.1, 0.0, 0.2, 0.75));
  EXPECT_TRUE(hotRetargetNeedsMeasuredStatePlan(
      true, true, false, -0.1, 1.1, 0.1, 0.2, 0.75));
}

TEST(PlannerFsm, RebasesDivergedHotRetargetBeforeAtomicCommit) {
  EXPECT_TRUE(hotRetargetNeedsMeasuredStatePlan(
      true, true, true, 0.2, 2.0, 0.1, 0.751, 0.75));
  EXPECT_FALSE(hotRetargetNeedsMeasuredStatePlan(
      true, true, true, 0.2, 2.0, 0.1, 0.75, 0.75));
  EXPECT_FALSE(hotRetargetNeedsMeasuredStatePlan(
      false, true, true, 0.2, 2.0, 0.1, 2.0, 0.75));
}

TEST(PlannerFsm, AcceptsSafetySuffixWhenVehicleIsAlreadyOnBackup) {
  const double elapsed_s = 2.5;
  const double original_backup_start_s = 2.0;
  const double effective_safety_start_s =
      std::max(original_backup_start_s, elapsed_s);
  EXPECT_TRUE(committedSafetySuffixIsUsable(
      true, elapsed_s, 4.0, effective_safety_start_s, 0.2, 0.75, true));
}

TEST(PlannerFsm, RepeatedBackupFailuresRetainModeBeforeAnchorInvalidation) {
  // This models the product transition sequence observed in the Phase C
  // trace: planner backend returns FAILED after backup generation, so the current
  // committed generation is not replaced while its certified suffix remains
  // usable. Once the suffix anchor exceeds the hard bound, validation fails
  // closed rather than resurrecting an older bundle or synthesizing a MAIN.
  // The generation value here is an explicit retained-state model; this test
  // does not claim to invoke Planner::ReplanOnce or inspect its private
  // Planner command snapshot.
  constexpr std::uint64_t committed_generation = 8U;
  constexpr double total_duration_s = 2.0;
  constexpr double elapsed_s = 0.5;

  for (int failure = 0; failure < 3; ++failure) {
    EXPECT_EQ(classifyPlannerResult(
                  navigation_planning::PlannerStatus::kFailed, false, true, false),
              PlannerResultDisposition::RetainCommittedCommand);
    EXPECT_TRUE(committedSafetySuffixIsUsable(
        true, elapsed_s + 0.1 * failure, total_duration_s,
        0.9, 0.70, 0.75, true));
    EXPECT_EQ(committed_generation, 8U);
  }

  EXPECT_FALSE(committedSafetySuffixIsUsable(
      true, elapsed_s, total_duration_s, 0.9, 0.7500001, 0.75, true));
  EXPECT_EQ(retainedValidationTransition(false),
            RetainedValidationTransition::FailClosed);
  EXPECT_EQ(committed_generation, 8U);
}

}  // namespace
}  // namespace navigation_runtime
