#include "navigation_runtime/planner_fsm.hpp"
#include "navigation_runtime/commit_trace.hpp"
#include "navigation_runtime/runtime_boundaries.hpp"
#include <navigation_planning/candidate_bundle.hpp>

#include <gtest/gtest.h>
#include <barrier>
#include <thread>
#include <atomic>
#include <memory>

namespace navigation_runtime {
namespace {

navigation_contracts::msg::NavigationGoal goal(
    const char* mission_id, std::uint32_t waypoint_index,
    std::uint64_t request_id, std::uint64_t route_revision) {
  navigation_contracts::msg::NavigationGoal value;
  value.mission_id = mission_id;
  value.waypoint_index = waypoint_index;
  value.request_id = request_id;
  value.route.route_revision = route_revision;
  return value;
}

TEST(PlannerFsm, ClassifiesDesiredAndExecutingIdentityTransitions) {
  const auto executing = goal("mission", 2U, 3U, 7U);
  EXPECT_EQ(classifyGoalTransition(std::nullopt, std::nullopt),
            GoalTransitionKind::kSteady);
  EXPECT_EQ(classifyGoalTransition(std::nullopt, executing),
            GoalTransitionKind::kCancelOrLocalizationReset);
  EXPECT_EQ(classifyGoalTransition(goal("mission", 2U, 3U, 7U), std::nullopt),
            GoalTransitionKind::kInitialGoal);
  EXPECT_EQ(classifyGoalTransition(goal("mission", 2U, 3U, 7U), executing),
            GoalTransitionKind::kSteady);
  EXPECT_EQ(classifyGoalTransition(goal("mission", 3U, 4U, 7U), executing),
            GoalTransitionKind::kSameRouteWaypointAdvance);
  EXPECT_EQ(classifyGoalTransition(goal("mission", 3U, 4U, 8U), executing),
            GoalTransitionKind::kRouteReplacement);
  EXPECT_EQ(classifyGoalTransition(goal("mission", 2U, 4U, 7U), executing),
            GoalTransitionKind::kRouteReplacement);
  EXPECT_EQ(classifyGoalTransition(goal("mission", 2U, 3U, 8U), executing),
            GoalTransitionKind::kRouteReplacement);
  EXPECT_EQ(classifyGoalTransition(goal("other", 0U, 1U, 1U), executing),
            GoalTransitionKind::kMissionReplacement);
  EXPECT_STREQ(goalTransitionKindName(GoalTransitionKind::kSameRouteWaypointAdvance),
               "same_route_waypoint_advance");
}

TEST(PlannerFsm, PlannerSolveActivityDisarmsOnlyItsOwnedGeneration) {
  std::int64_t started_ns{0};
  std::uint64_t active_generation{0};
  std::mutex activity_mutex;
  {
    const PlannerSolveActivityScope activity(
        activity_mutex, started_ns, active_generation, 7U, 1234);
    EXPECT_EQ(started_ns, 1234);
    EXPECT_EQ(active_generation, 7U);
  }
  EXPECT_EQ(started_ns, 0);
  EXPECT_EQ(active_generation, 0U);

  {
    const PlannerSolveActivityScope stale_activity(
        activity_mutex, started_ns, active_generation, 8U, 2000);
    active_generation = 9U;
    started_ns = 3000;
  }
  EXPECT_EQ(active_generation, 9U);
  EXPECT_EQ(started_ns, 3000);
}

TEST(PlannerFsm, PlannerSolveActivityBlocksScopeExitDuringWatchdogDecision) {
  std::int64_t started_ns{0};
  std::uint64_t active_generation{0U};
  std::mutex activity_mutex;
  auto activity = std::make_unique<PlannerSolveActivityScope>(
      activity_mutex, started_ns, active_generation, 11U, 1000);

  std::unique_lock<std::mutex> watchdog_lock(activity_mutex);
  std::atomic_bool destructor_started{false};
  std::atomic_bool destructor_finished{false};
  std::thread stale_scope([&] {
    destructor_started.store(true, std::memory_order_release);
    activity.reset();
    destructor_finished.store(true, std::memory_order_release);
  });
  while (!destructor_started.load(std::memory_order_acquire)) std::this_thread::yield();
  std::this_thread::yield();
  EXPECT_FALSE(destructor_finished.load(std::memory_order_acquire));
  EXPECT_EQ(active_generation, 11U);
  EXPECT_EQ(started_ns, 1000);

  // The watchdog may now cancel and apply its generation-bound transition
  // while the worker scope is still the sole owner of the marker.
  watchdog_lock.unlock();
  stale_scope.join();
  EXPECT_EQ(active_generation, 0U);
  EXPECT_EQ(started_ns, 0);
}

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

TEST(PlannerFsm, RetainsOnlyCertifiedPassThroughTerminalAcknowledgement) {
  const PassThroughTerminalAckFacts valid{
      true, true, true, true, true, true, true, false, false, true, true};
  EXPECT_TRUE(passThroughTerminalAckMayRetainCommand(valid));

  auto facts = valid;
  facts.successful_terminal_status = false;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.status_matches_active_identity = false;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.active_goal_is_pass_through = false;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.outgoing_route_exists = false;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.certified_main_command = false;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.certified_continuation_boundary = false;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.execution_identity_current = false;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.failure_latched = true;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.safety_suffix_active = true;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.command_exposure_allowed = false;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
  facts = valid;
  facts.command_lease_valid = false;
  EXPECT_FALSE(passThroughTerminalAckMayRetainCommand(facts));
}

TEST(PlannerFsm, DefersOptimizerWhileCertifiedMainHasRenewalMargin) {
  const auto decision = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 1.0, 4.0,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS,
      navigation_planning::PlanningTimingContract::kStitchDurationS,
      navigation_planning::PlanningTimingContract::kPlannerPeriodS);
  EXPECT_FALSE(decision.run_optimizer);
  EXPECT_EQ(decision.reason, PlannerRenewalReason::kRetainCertifiedMain);
  EXPECT_DOUBLE_EQ(decision.remaining_main_horizon_s, 3.0);
  EXPECT_DOUBLE_EQ(
      decision.required_lead_time_s,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS +
          2.0 * navigation_planning::PlanningTimingContract::kStitchDurationS +
          navigation_planning::PlanningTimingContract::kPlannerPeriodS +
          navigation_planning::PlanningTimingContract::kCommitGuardS);
}

TEST(PlannerFsm, UsesCoherentTenHertzTimingContract) {
  EXPECT_DOUBLE_EQ(navigation_planning::PlanningTimingContract::kPlannerPeriodS, 0.10);
  EXPECT_DOUBLE_EQ(navigation_planning::PlanningTimingContract::kPlannerRateHz, 10.0);
  EXPECT_DOUBLE_EQ(navigation_planning::PlanningTimingContract::kSolveDeadlineS, 0.08);
  EXPECT_LT(navigation_planning::PlanningTimingContract::kSolveDeadlineS,
            navigation_planning::PlanningTimingContract::kPlannerPeriodS);
  EXPECT_DOUBLE_EQ(navigation_planning::PlanningTimingContract::kCommandPeriodS, 0.02);
  EXPECT_DOUBLE_EQ(navigation_planning::PlanningTimingContract::kCommandStreamTimeoutS, 0.10);
  EXPECT_DOUBLE_EQ(
      navigation_planning::PlanningTimingContract::kMinimumMainReserveS,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS +
          navigation_planning::PlanningTimingContract::kStitchDurationS +
          navigation_planning::PlanningTimingContract::kPlannerPeriodS +
          navigation_planning::PlanningTimingContract::kCommitGuardS);
}

TEST(PlannerFsm, ProductionRenewalLeadIncludesTwoForwardIntervals) {
  const auto decision = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 2.0, 3.0,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS,
      navigation_planning::PlanningTimingContract::kStitchDurationS,
      navigation_planning::PlanningTimingContract::kPlannerPeriodS);
  EXPECT_TRUE(decision.run_optimizer);
  EXPECT_EQ(decision.reason, PlannerRenewalReason::kRenewalDue);
  EXPECT_DOUBLE_EQ(
      decision.required_lead_time_s,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS +
          2.0 * navigation_planning::PlanningTimingContract::kStitchDurationS +
          navigation_planning::PlanningTimingContract::kPlannerPeriodS +
          navigation_planning::PlanningTimingContract::kCommitGuardS);
  EXPECT_DOUBLE_EQ(decision.remaining_main_horizon_s, 1.0);
}

TEST(PlannerFsm, ArmsOrdinaryFailureInjectionOnlyInFreshDueWindow) {
  const double required_lead_time =
      navigation_planning::PlanningTimingContract::kSolveDeadlineS +
      2.0 * navigation_planning::PlanningTimingContract::kStitchDurationS +
      navigation_planning::PlanningTimingContract::kPlannerPeriodS +
      navigation_planning::PlanningTimingContract::kCommitGuardS;
  const auto fresh_due = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 3.2 - (required_lead_time -
                   0.5 * navigation_planning::PlanningTimingContract::kPlannerPeriodS), 3.2,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS,
      navigation_planning::PlanningTimingContract::kStitchDurationS,
      navigation_planning::PlanningTimingContract::kPlannerPeriodS);
  EXPECT_EQ(fresh_due.reason, PlannerRenewalReason::kRenewalDue);
  EXPECT_TRUE(ordinaryRenewalFailureInjectionMayArm(
      fresh_due, navigation_planning::PlanningTimingContract::kPlannerPeriodS));

  const auto late_due = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 3.2 - (required_lead_time -
                   1.5 * navigation_planning::PlanningTimingContract::kPlannerPeriodS), 3.2,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS,
      navigation_planning::PlanningTimingContract::kStitchDurationS,
      navigation_planning::PlanningTimingContract::kPlannerPeriodS);
  EXPECT_EQ(late_due.reason, PlannerRenewalReason::kRenewalDue);
  EXPECT_FALSE(ordinaryRenewalFailureInjectionMayArm(
      late_due, navigation_planning::PlanningTimingContract::kPlannerPeriodS));

  const auto retained = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 1.0, 4.0,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS,
      navigation_planning::PlanningTimingContract::kStitchDurationS,
      navigation_planning::PlanningTimingContract::kPlannerPeriodS);
  EXPECT_EQ(retained.reason, PlannerRenewalReason::kRetainCertifiedMain);
  EXPECT_FALSE(ordinaryRenewalFailureInjectionMayArm(
      retained, navigation_planning::PlanningTimingContract::kPlannerPeriodS));

  const auto forced = classifyPlannerRenewal(
      true, true, false, navigation_planning::CandidateRole::kMain,
      true, 0.0, 10.0,
      navigation_planning::PlanningTimingContract::kSolveDeadlineS,
      navigation_planning::PlanningTimingContract::kStitchDurationS,
      navigation_planning::PlanningTimingContract::kPlannerPeriodS);
  EXPECT_EQ(forced.reason, PlannerRenewalReason::kForcedTransition);
  EXPECT_FALSE(ordinaryRenewalFailureInjectionMayArm(
      forced, navigation_planning::PlanningTimingContract::kPlannerPeriodS));
}

TEST(PlannerFsm, StartsAnchorRecoveryBeforeExecutionLeaseIsExhausted) {
  EXPECT_FALSE(commandAnchorRecoveryDue(
      false, navigation_planning::CandidateRole::kMain, 0.6, 0.75));
  EXPECT_FALSE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kBackup, 0.6, 0.75));
  EXPECT_FALSE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kMain, 0.375, 0.75));
  EXPECT_TRUE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kMain, 0.75, 0.75));
  EXPECT_FALSE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kMain,
      std::numeric_limits<double>::quiet_NaN(), 0.75));
}

TEST(PlannerFsm, AnchorRecoveryUsesRetainedTrackingCertificate) {
  const double retained_limit = retainedCommandTrackingLimit(0.25, 0.75);
  EXPECT_DOUBLE_EQ(retained_limit, 0.25);
  EXPECT_FALSE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kMain, 0.249, retained_limit));
  EXPECT_TRUE(commandAnchorRecoveryDue(
      true, navigation_planning::CandidateRole::kMain, 0.25, retained_limit));
}

TEST(PlannerFsm, DefersTerminalStopAnchorRecoveryOnlyInsideTrackingEnvelope) {
  EXPECT_TRUE(terminalStopMayDeferAnchorRecovery(
      true, true, true, true,
      navigation_planning::CandidateRole::kMain, true, 0.125, 0.25));
  EXPECT_FALSE(terminalStopMayDeferAnchorRecovery(
      true, true, true, false,
      navigation_planning::CandidateRole::kMain, true, 0.125, 0.25));
  EXPECT_FALSE(terminalStopMayDeferAnchorRecovery(
      true, true, true, true,
      navigation_planning::CandidateRole::kBackup, true, 0.125, 0.25));
  EXPECT_FALSE(terminalStopMayDeferAnchorRecovery(
      true, true, true, true,
      navigation_planning::CandidateRole::kMain, true, 0.251, 0.25));
  EXPECT_FALSE(terminalStopMayDeferAnchorRecovery(
      true, true, true, true,
      navigation_planning::CandidateRole::kMain, false, 0.20, 0.25));
  EXPECT_FALSE(terminalStopMayDeferAnchorRecovery(
      true, true, true, true,
      navigation_planning::CandidateRole::kMain, true, 0.20, 0.25, true));
}

TEST(PlannerFsm, TerminalStopCompletionRequiresMeasuredWaypointAcceptance) {
  using navigation_runtime::terminalStopCompletionObserved;

  EXPECT_TRUE(terminalStopCompletionObserved(true, true, true, 0.535, 0.314, 0.7));
  EXPECT_FALSE(terminalStopCompletionObserved(true, false, true, 0.535, 0.314, 0.7));
  EXPECT_FALSE(terminalStopCompletionObserved(true, true, true, 0.535, 0.701, 0.7));
  EXPECT_FALSE(terminalStopCompletionObserved(true, true, true, 0.701, 0.314, 0.7));
  EXPECT_FALSE(terminalStopCompletionObserved(true, true, false, 0.1, 0.1, 0.7));
  EXPECT_FALSE(terminalStopCompletionObserved(false, true, true, 0.1, 0.1, 0.7));
}

TEST(PlannerFsm, TerminalStopAcceptsCertifiedBackupEndpointButNotEmergency) {
  EXPECT_TRUE(terminalStopEndpointContractValid(
      true, navigation_planning::CandidateBundleKind::kMainWithBackup,
      navigation_planning::CandidateRole::kMain,
      navigation_planning::CandidateRole::kBackup));
  EXPECT_TRUE(terminalStopEndpointContractValid(
      true, navigation_planning::CandidateBundleKind::kTerminalStop,
      navigation_planning::CandidateRole::kMain,
      navigation_planning::CandidateRole::kMain));
  EXPECT_FALSE(terminalStopEndpointContractValid(
      true, navigation_planning::CandidateBundleKind::kMainWithBackup,
      navigation_planning::CandidateRole::kMain,
      navigation_planning::CandidateRole::kEmergency));
  EXPECT_FALSE(terminalStopEndpointContractValid(
      true, navigation_planning::CandidateBundleKind::kBackupOnly,
      navigation_planning::CandidateRole::kBackup,
      navigation_planning::CandidateRole::kBackup));
}

TEST(PlannerFsm, UsesStopDistanceOnlyForTerminalStopApproach) {
  const double braking_distance = plannerTerminalStopBrakingDistanceM(5.0, 2.0, 4.0);
  EXPECT_NEAR(braking_distance, 8.75, 1.0e-12);
  EXPECT_FALSE(plannerTerminalStopApproachDue(
      false, 0.0, 5.0, 2.0, 4.0));
  EXPECT_FALSE(plannerTerminalStopApproachDue(
      true, 140.0, 5.0, 2.0, 4.0));
  EXPECT_TRUE(plannerTerminalStopApproachDue(
      true, braking_distance, 5.0, 2.0, 4.0));
}

TEST(PlannerFsm, RenewsBeforeMainCanReachBackupDuringSchedulingAndSolve) {
  const auto before_boundary = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 2.79, 4.0, 0.18, 0.4, 0.2);
  EXPECT_FALSE(before_boundary.run_optimizer);

  const auto at_boundary = classifyPlannerRenewal(
      false, true, false, navigation_planning::CandidateRole::kMain,
      true, 2.80, 4.0, 0.18, 0.4, 0.2);
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

TEST(PlannerFsm, StoppedRecoveryFailureRemainsRetryableWithoutCommand) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kFailed,
                                  true, false, false),
            PlannerResultDisposition::RetryFromRest);
}

TEST(PlannerFsm, RetainedValidationPreservesValidStateAndFailsClosedOtherwise) {
  EXPECT_EQ(retainedValidationTransition(true),
            RetainedValidationTransition::PreserveExistingState);
  EXPECT_EQ(retainedValidationTransition(false),
            RetainedValidationTransition::FailClosed);
}

TEST(PlannerFsm, EmergencyBrakeCannotBeRearmedFromADriftingEmergency) {
  EXPECT_TRUE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true, true,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain));
  EXPECT_FALSE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true, false,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kBackup));
  EXPECT_FALSE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true, true,
      ExecutionRecoveryState::kEmergencyBrake,
      navigation_planning::CandidateRole::kEmergency));
  EXPECT_FALSE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, true, true, true, true, true,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain));
  EXPECT_FALSE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true, false,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain));
}

TEST(PlannerFsm, ProjectedAnchorCrossingRequiresKnownFreeState) {
  EXPECT_FALSE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, true, true, true, true, false,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain));
  EXPECT_TRUE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true, false,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain, true, true));
  EXPECT_TRUE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, true, true, true, true, false,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain, true, true, false));
  EXPECT_TRUE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, true, true, true, true, false,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain, true, true, false, true));
  EXPECT_FALSE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true, false,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain, true, false));
  EXPECT_FALSE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, true, true, true, true, false,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain, true, true, true));
  EXPECT_TRUE(measuredStateEmergencyMayReplaceCommittedCommand(
      false, false, true, true, true, true,
      ExecutionRecoveryState::kTrackMain,
      navigation_planning::CandidateRole::kMain));
}

TEST(PlannerFsm, EmergencyBoundaryDropsOnlyEstimatedHighOrderDerivatives) {
  navigation_planning::TrajectoryPoint source;
  source.position_world << 1.0, 2.0, 3.0;
  source.velocity_world << 0.4, -0.2, 0.1;
  source.acceleration_world << 2.0, 3.0, 4.0;
  source.jerk_world << -5.0, 6.0, -7.0;
  source.yaw = 0.3;
  source.yaw_rate = -0.1;

  const auto estimated = makeMeasuredEmergencyBoundary(source, true, true);
  EXPECT_TRUE(estimated.position_world.isApprox(source.position_world));
  EXPECT_TRUE(estimated.velocity_world.isApprox(source.velocity_world));
  EXPECT_TRUE(estimated.acceleration_world.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(estimated.jerk_world.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_DOUBLE_EQ(estimated.yaw, source.yaw);
  EXPECT_DOUBLE_EQ(estimated.yaw_rate, source.yaw_rate);

  const auto measured = makeMeasuredEmergencyBoundary(source, false, false);
  EXPECT_TRUE(measured.acceleration_world.isApprox(source.acceleration_world));
  EXPECT_TRUE(measured.jerk_world.isApprox(source.jerk_world));
}

TEST(PlannerFsm, EmergencyTerminalAltitudeUsesBoundedCertifiedAnchor) {
  EXPECT_DOUBLE_EQ(
      plannerEmergencyTerminalAltitude(2.0, 2.2, 0.25), 2.2);
  EXPECT_DOUBLE_EQ(
      plannerEmergencyTerminalAltitude(2.0, 3.0, 0.25), 2.25);
  EXPECT_DOUBLE_EQ(
      plannerEmergencyTerminalAltitude(2.0, 1.0, 0.25), 1.75);
  EXPECT_DOUBLE_EQ(
      plannerEmergencyTerminalAltitude(2.0, std::numeric_limits<double>::quiet_NaN(),
                                       0.25), 2.0);
  EXPECT_DOUBLE_EQ(
      plannerEmergencyTerminalAltitude(2.0, 3.0, -0.25), 2.0);
}

TEST(PlannerFsm, BackupAndEmergencyAreOneWayUntilCertifiedStop) {
  EXPECT_FALSE(nominalPlanningAllowed(ExecutionRecoveryState::kTrackBackup));
  EXPECT_FALSE(nominalPlanningAllowed(ExecutionRecoveryState::kEmergencyBrake));
  EXPECT_EQ(transitionExecutionRecovery(
                ExecutionRecoveryState::kTrackBackup,
                ExecutionRecoveryEvent::kMainCommitted),
            ExecutionRecoveryState::kTrackBackup);
  EXPECT_EQ(transitionExecutionRecovery(
                ExecutionRecoveryState::kEmergencyBrake,
                ExecutionRecoveryEvent::kEmergencyCommitted),
            ExecutionRecoveryState::kEmergencyBrake);
  EXPECT_EQ(transitionExecutionRecovery(
                ExecutionRecoveryState::kEmergencyBrake,
                ExecutionRecoveryEvent::kCertifiedStopObserved),
            ExecutionRecoveryState::kStoppedRecovery);
  EXPECT_EQ(transitionExecutionRecovery(
                ExecutionRecoveryState::kTrackMain,
                ExecutionRecoveryEvent::kTerminalStopCompleted),
            ExecutionRecoveryState::kStoppedRecovery);
  EXPECT_TRUE(nominalPlanningAllowed(ExecutionRecoveryState::kStoppedRecovery));
}

TEST(PlannerFsm, EmergencyCertificationFailureGoesDirectlyToPx4Hold) {
  EXPECT_EQ(transitionExecutionRecovery(
                ExecutionRecoveryState::kTrackMain,
                ExecutionRecoveryEvent::kEmergencyCertificationFailed),
            ExecutionRecoveryState::kPx4Hold);
}

TEST(PlannerFsm, SerializedRecoveryEventsHaveOneLinearOrder) {
  std::atomic<ExecutionRecoveryState> state{ExecutionRecoveryState::kTrackMain};
  std::mutex transition_mutex;
  std::barrier rendezvous(3);
  std::thread backup([&] {
    rendezvous.arrive_and_wait();
    std::lock_guard<std::mutex> lock(transition_mutex);
    applyExecutionRecoveryEventLocked(
        state, ExecutionRecoveryEvent::kBackupActivated);
  });
  std::thread emergency([&] {
    rendezvous.arrive_and_wait();
    std::lock_guard<std::mutex> lock(transition_mutex);
    applyExecutionRecoveryEventLocked(
        state, ExecutionRecoveryEvent::kEmergencyCommitted);
  });
  rendezvous.arrive_and_wait();
  backup.join();
  emergency.join();

  const auto result = state.load(std::memory_order_acquire);
  EXPECT_TRUE(result == ExecutionRecoveryState::kTrackBackup ||
              result == ExecutionRecoveryState::kEmergencyBrake);
}

TEST(PlannerFsm, SerializedFailClosedCannotBeResurrectedByNominalEvent) {
  std::atomic<ExecutionRecoveryState> state{ExecutionRecoveryState::kTrackMain};
  std::mutex transition_mutex;
  std::barrier rendezvous(3);
  std::thread nominal([&] {
    rendezvous.arrive_and_wait();
    std::lock_guard<std::mutex> lock(transition_mutex);
    applyExecutionRecoveryEventLocked(
        state, ExecutionRecoveryEvent::kBackupActivated);
  });
  std::thread fail_closed([&] {
    rendezvous.arrive_and_wait();
    std::lock_guard<std::mutex> lock(transition_mutex);
    state.store(ExecutionRecoveryState::kPx4Hold,
                std::memory_order_release);
  });
  rendezvous.arrive_and_wait();
  nominal.join();
  fail_closed.join();

  EXPECT_EQ(state.load(std::memory_order_acquire),
            ExecutionRecoveryState::kPx4Hold);
}

TEST(PlannerFsm, PreservesObservedTerminalHoldAcrossRestRetry) {
  EXPECT_TRUE(terminalHoldIsPending(true, true, true, 17U));
  EXPECT_FALSE(terminalHoldIsPending(false, true, true, 17U));
  EXPECT_FALSE(terminalHoldIsPending(true, false, true, 17U));
  EXPECT_FALSE(terminalHoldIsPending(true, true, true, 0U));
  EXPECT_FALSE(terminalHoldIsPending(true, true, false, 17U));
}

TEST(PlannerFsm, RetainsIdentityBoundTerminalBundleAcrossDerivedFlagRace) {
  EXPECT_TRUE(committedTerminalBundleHoldIsPending(true, true, true, true, true, 17U));
  EXPECT_FALSE(committedTerminalBundleHoldIsPending(true, true, false, true, true, 17U));
  EXPECT_FALSE(committedTerminalBundleHoldIsPending(true, true, true, false, true, 17U));
  EXPECT_FALSE(committedTerminalBundleHoldIsPending(true, true, true, true, false, 17U));
  EXPECT_FALSE(committedTerminalBundleHoldIsPending(true, false, true, true, true, 17U));
  EXPECT_FALSE(committedTerminalBundleHoldIsPending(true, true, true, true, true, 0U));
}

TEST(PlannerFsm, BackupOutsideAcceptanceSchedulesMeasuredRestart) {
  EXPECT_TRUE(backupStopNeedsMeasuredRestart(
      ExecutionRecoveryState::kTrackBackup, true, false));
  EXPECT_FALSE(backupStopNeedsMeasuredRestart(
      ExecutionRecoveryState::kTrackBackup, true, true));
  EXPECT_FALSE(backupStopNeedsMeasuredRestart(
      ExecutionRecoveryState::kTrackBackup, false, false));
  EXPECT_FALSE(backupStopNeedsMeasuredRestart(
      ExecutionRecoveryState::kEmergencyBrake, true, false));
}

TEST(PlannerFsm, RetainsPendingStatusWhileSafetySuffixDrains) {
  EXPECT_FALSE(pendingGoalTerminalStatusMayClear(true, true));
  EXPECT_TRUE(pendingGoalTerminalStatusMayClear(true, false));
  EXPECT_FALSE(pendingGoalTerminalStatusMayClear(false, true));
  EXPECT_FALSE(pendingGoalTerminalStatusMayClear(false, false));
}

TEST(PlannerFsm, PendingGoalOwnerLinearizesSupersedeAndExactConsumption) {
  PendingGoalHandoffOwner owner;
  navigation_contracts::msg::NavigationGoal active;
  active.mission_id = "mission";
  active.request_id = 10U;
  active.waypoint_index = 2U;
  active.route.route_revision = 4U;
  auto make_goal = [](std::uint64_t request, std::uint32_t waypoint,
                      std::uint64_t revision) {
    auto goal = std::make_shared<navigation_contracts::msg::NavigationGoal>();
    goal->mission_id = "mission";
    goal->request_id = request;
    goal->waypoint_index = waypoint;
    goal->route.route_revision = revision;
    return std::shared_ptr<const navigation_contracts::msg::NavigationGoal>(goal);
  };
  EXPECT_TRUE(owner.enqueueGoal(make_goal(11U, 3U, 5U), active, true));
  EXPECT_FALSE(owner.enqueueGoal(make_goal(14U, 6U, 8U), active, false));
  EXPECT_FALSE(owner.enqueueGoal(make_goal(10U, 3U, 5U), active, true));
  EXPECT_TRUE(owner.enqueueGoal(make_goal(12U, 4U, 6U), active, true));
  const auto promoted = owner.goalSnapshot();
  ASSERT_TRUE(promoted);
  EXPECT_EQ(promoted->request_id, 12U);
  EXPECT_TRUE(owner.consumeGoal(promoted));
  EXPECT_FALSE(owner.goalSnapshot());
  EXPECT_TRUE(owner.enqueueGoal(make_goal(13U, 5U, 7U), active, true));
  EXPECT_TRUE(owner.goalMatchesStatus("mission", 5U, 13U));
  EXPECT_FALSE(owner.goalMatchesStatus("mission", 5U, 12U));
  owner.clearGoal();
  EXPECT_FALSE(owner.goalSnapshot());
}

TEST(PlannerFsm, PendingGoalOwnerConcurrentSupersedeKeepsNewestOnly) {
  PendingGoalHandoffOwner owner;
  navigation_contracts::msg::NavigationGoal active;
  active.mission_id = "mission";
  active.request_id = 10U;
  active.waypoint_index = 1U;
  active.route.route_revision = 1U;
  auto make_goal = [](std::uint64_t request) {
    auto goal = std::make_shared<navigation_contracts::msg::NavigationGoal>();
    goal->mission_id = "mission";
    goal->request_id = request;
    goal->waypoint_index = static_cast<std::uint32_t>(request - 9U);
    goal->route.route_revision = request;
    return std::shared_ptr<const navigation_contracts::msg::NavigationGoal>(goal);
  };
  std::barrier rendezvous(3);
  std::thread older([&] {
    rendezvous.arrive_and_wait();
    (void)owner.enqueueGoal(make_goal(11U), active, true);
  });
  std::thread newer([&] {
    rendezvous.arrive_and_wait();
    (void)owner.enqueueGoal(make_goal(12U), active, true);
  });
  rendezvous.arrive_and_wait();
  older.join();
  newer.join();
  const auto pending = owner.goalSnapshot();
  ASSERT_TRUE(pending);
  EXPECT_EQ(pending->request_id, 12U);
}

TEST(PlannerFsm, ContinuesOnlyCompletedPassThroughGoalEndpoints) {
  EXPECT_TRUE(completedPassThroughRequiresContinuation(true, true, true, true));
  EXPECT_FALSE(completedPassThroughRequiresContinuation(false, true, true, true));
  EXPECT_FALSE(completedPassThroughRequiresContinuation(true, false, true, true));
  EXPECT_FALSE(completedPassThroughRequiresContinuation(true, true, true, false));
  EXPECT_FALSE(completedPassThroughRequiresContinuation(true, true, false, true));
}

TEST(PlannerFsm, RejectsFinitePassThroughWithoutOutgoingRouteLeg) {
  EXPECT_TRUE(waypointBehaviorContractValid(false, false));
  EXPECT_TRUE(waypointBehaviorContractValid(false, true));
  EXPECT_TRUE(waypointBehaviorContractValid(true, true));
  EXPECT_FALSE(waypointBehaviorContractValid(true, false));
}

TEST(PlannerFsm, FiveSecondTimeoutExistsOnlyWhileStationary) {
  EXPECT_TRUE(stoppedPlanningTimeoutMayFailClosed(
      ExecutionRecoveryState::kInitialHold, true, 5.0, 5.0));
  EXPECT_TRUE(stoppedPlanningTimeoutMayFailClosed(
      ExecutionRecoveryState::kStoppedRecovery, true, 5.1, 5.0));
  EXPECT_FALSE(stoppedPlanningTimeoutMayFailClosed(
      ExecutionRecoveryState::kTrackMain, true, 10.0, 5.0));
  EXPECT_FALSE(stoppedPlanningTimeoutMayFailClosed(
      ExecutionRecoveryState::kStoppedRecovery, false, 10.0, 5.0));
  EXPECT_FALSE(stoppedPlanningTimeoutMayFailClosed(
      ExecutionRecoveryState::kStoppedRecovery, true, 4.999, 5.0));
}

TEST(PlannerFsm, WatchdogKeepsBoundedStoppedRecoveryHoldForRetry) {
  EXPECT_TRUE(watchdogTimeoutMayRetainStoppedRecoveryHold(
      ExecutionRecoveryState::kStoppedRecovery, true, true));
  EXPECT_FALSE(watchdogTimeoutMayRetainStoppedRecoveryHold(
      ExecutionRecoveryState::kStoppedRecovery, true, false));
  EXPECT_FALSE(watchdogTimeoutMayRetainStoppedRecoveryHold(
      ExecutionRecoveryState::kTrackBackup, true, true));
  EXPECT_FALSE(watchdogTimeoutMayRetainStoppedRecoveryHold(
      ExecutionRecoveryState::kStoppedRecovery, false, true));
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

TEST(PlannerFsm, StaleCommandPublicationCannotMutateNewExecution) {
  EXPECT_EQ(
      classifyStaleCommandPublication(false, false),
      StaleCommandPublicationDisposition::kDropStale);
  EXPECT_EQ(
      classifyStaleCommandPublication(true, true),
      StaleCommandPublicationDisposition::kRetainSuperseding);
  EXPECT_EQ(
      classifyStaleCommandPublication(true, false),
      StaleCommandPublicationDisposition::kFailClosed);
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
  candidate.activation_stamp_ns = candidate.valid_from_ns;
  candidate.declared_start_ns = 10000000000LL;
  candidate.declared_end_ns = 11000000000LL;
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
  candidate.activation_stamp_ns = candidate.valid_from_ns;
  candidate.declared_start_ns = 10000000000LL;
  candidate.declared_end_ns = 11000000000LL;
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

TEST(PlannerFsm, SamplesExactFractionalMainWithBackupEndpointRole) {
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
  candidate.start_wall_time_s = 10.123456789;
  candidate.duration_s = 0.75;
  candidate.declared_start_ns = *navigation_common::secondsToNanoseconds(
      candidate.start_wall_time_s);
  candidate.declared_end_ns = *navigation_common::secondsSumToNanoseconds(
      candidate.start_wall_time_s, candidate.duration_s);
  candidate.valid_from_ns = candidate.declared_start_ns;
  candidate.valid_until_ns = candidate.declared_end_ns;
  candidate.activation_stamp_ns = candidate.valid_from_ns;
  candidate.role = navigation_planning::CandidateRole::kMain;
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.certificates = {true, true, true, false};
  candidate.protected_region.minimum = Eigen::Vector3d::Zero();
  candidate.protected_region.maximum = Eigen::Vector3d::Ones();
  candidate.role_schedule = {
      {0.0, 0.5, navigation_planning::CandidateRole::kMain},
      {0.5, 0.75, navigation_planning::CandidateRole::kBackup}};
  const auto start_ns = candidate.declared_start_ns;
  candidate.evaluator = [start_ns](const std::int64_t stamp_ns,
                                   navigation_planning::TrajectoryPoint& point) {
    point.position_world = Eigen::Vector3d{1.0, 2.0, 3.0};
    point.trajectory_time_s = static_cast<double>(stamp_ns - start_ns) * 1.0e-9;
    point.role = point.trajectory_time_s >= 0.5
        ? navigation_planning::CandidateRole::kBackup
        : navigation_planning::CandidateRole::kMain;
    return true;
  };

  const auto endpoint = candidate.sampleAtDeclaredEnd();
  ASSERT_TRUE(endpoint.has_value());
  EXPECT_EQ(endpoint->role, navigation_planning::CandidateRole::kBackup);
  EXPECT_TRUE(endpoint->finished);
  EXPECT_EQ(candidate.scheduledRole(candidate.duration_s - 1.0e-9),
            navigation_planning::CandidateRole::kBackup);
  EXPECT_EQ(candidate.scheduledRole(0.5 - 1.0e-9),
            navigation_planning::CandidateRole::kMain);
  EXPECT_EQ(candidate.scheduledRole(0.5), navigation_planning::CandidateRole::kBackup);

  auto inconsistent = candidate;
  inconsistent.declared_end_ns += 1;
  EXPECT_FALSE(inconsistent.valid());
}

TEST(PlannerFsm, AcceptsBoundedSourceTimeMainPhaseWithoutIncreasingTrackingTube) {
  navigation_planning::CandidateBundle candidate;
  candidate.world_identity.localization_epoch = 1U;
  candidate.world_identity.generation = 1U;
  candidate.world_identity.revision = 1U;
  candidate.world_identity.observation_stamp_ns = 1;
  candidate.pinned_world_identity = candidate.world_identity;
  candidate.localization_epoch = 1U;
  candidate.goal_epoch = 1U;
  candidate.request_id = 1U;
  candidate.bundle_generation = 1U;
  candidate.start_wall_time_s = 10.0;
  candidate.duration_s = 2.0;
  candidate.backup_start_time_s = 1.5;
  candidate.declared_start_ns = 10000000000LL;
  candidate.declared_end_ns = 12000000000LL;
  candidate.valid_from_ns = candidate.declared_start_ns;
  candidate.valid_until_ns = candidate.declared_end_ns;
  candidate.activation_stamp_ns = candidate.valid_from_ns;
  candidate.role = navigation_planning::CandidateRole::kMain;
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.certificates = {true, true, true, false};
  candidate.protected_region.minimum = Eigen::Vector3d::Constant(-10.0);
  candidate.protected_region.maximum = Eigen::Vector3d::Constant(10.0);
  candidate.role_schedule = {
      {0.0, 1.5, navigation_planning::CandidateRole::kMain},
      {1.5, 2.0, navigation_planning::CandidateRole::kBackup}};
  candidate.evaluator = [](const std::int64_t stamp_ns,
                           navigation_planning::TrajectoryPoint& point) {
    point.trajectory_time_s = static_cast<double>(stamp_ns - 10000000000LL) * 1.0e-9;
    point.position_world = Eigen::Vector3d{1.0 + 3.0 * point.trajectory_time_s, 0.0, 0.0};
    point.velocity_world = Eigen::Vector3d{3.0, 0.0, 0.0};
    point.role = point.trajectory_time_s < 1.5
        ? navigation_planning::CandidateRole::kMain
        : navigation_planning::CandidateRole::kBackup;
    return true;
  };

  const auto now = candidate.sampleAtDeclaredStamp(10500000000LL);
  const auto source = candidate.sampleAtDeclaredStamp(10476000000LL);
  ASSERT_TRUE(now.has_value());
  ASSERT_TRUE(source.has_value());
  const auto result = assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{2.428, -0.24, 0.0}, Eigen::Vector3d{3.0, 0.0, 0.0},
      10500000000LL, 10476000000LL,
      100000000LL, 0.1, 0.25, 0.75, true, true);
  EXPECT_TRUE(result.accepted());
  EXPECT_NEAR(result.source_time_error_m, 0.24, 1.0e-12);
  EXPECT_NEAR(result.predicted_source_error_m, 0.24, 1.0e-12);
  EXPECT_NEAR(result.raw_divergence_m, std::sqrt(0.072 * 0.072 + 0.24 * 0.24), 1.0e-12);
  EXPECT_GT(result.raw_divergence_m, 0.25);
  EXPECT_LT(result.raw_divergence_m, 0.75);

  // The same physical residual is initially admissible at .248 m, but a
  // transaction delayed by 80 ms must recompute the forecast rather than
  // carry the old witness across the next validation horizon.
  const auto initial_delayed_case = assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{2.44, -0.23, 0.0},
      Eigen::Vector3d{2.85, 0.0, 0.0}, 10500000000LL, 10480000000LL,
      100000000LL, 0.1, 0.25, 0.75, true, true);
  EXPECT_TRUE(initial_delayed_case.accepted());
  EXPECT_NEAR(initial_delayed_case.predicted_source_error_m, 0.248, 1.0e-12);
  EXPECT_FALSE(assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{2.44, -0.23, 0.0},
      Eigen::Vector3d{2.85, 0.0, 0.0}, 10580000000LL, 10480000000LL,
      100000000LL, 0.1, 0.25, 0.75, true, true).accepted());

  // A source-time point can be inside the unchanged tracking tube while a
  // large relative lateral velocity makes the next validation interval
  // unsafe. The bounded forecast must reject that case.
  EXPECT_FALSE(assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{2.464, -0.2443, 0.0},
      Eigen::Vector3d{3.0, 1.0, 0.0}, 10500000000LL, 10488000000LL,
      100000000LL, 0.1, 0.25, 0.75, true, true).accepted());

  // The phase witness cannot cross the immutable MAIN/BACKUP seam. It is not
  // a way to borrow a future role interval while the current sample is MAIN.
  EXPECT_FALSE(assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{5.434, 0.0, 0.0},
      Eigen::Vector3d{3.0, 0.0, 0.0}, 11490000000LL, 11478000000LL,
      100000000LL, 0.1, 0.25, 0.75, true, true).accepted());
}

TEST(PlannerFsm, PhaseCertificateRejectsUnsafeOrAmbiguousMotion) {
  navigation_planning::CandidateBundle candidate;
  candidate.world_identity.localization_epoch = 1U;
  candidate.world_identity.generation = 1U;
  candidate.world_identity.revision = 1U;
  candidate.world_identity.observation_stamp_ns = 1;
  candidate.pinned_world_identity = candidate.world_identity;
  candidate.localization_epoch = 1U;
  candidate.goal_epoch = 1U;
  candidate.request_id = 1U;
  candidate.bundle_generation = 1U;
  candidate.start_wall_time_s = 10.0;
  candidate.duration_s = 2.0;
  candidate.backup_start_time_s = 1.5;
  candidate.declared_start_ns = 10000000000LL;
  candidate.declared_end_ns = 12000000000LL;
  candidate.valid_from_ns = candidate.declared_start_ns;
  candidate.valid_until_ns = candidate.declared_end_ns;
  candidate.activation_stamp_ns = candidate.valid_from_ns;
  candidate.role = navigation_planning::CandidateRole::kMain;
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.certificates = {true, true, true, false};
  candidate.protected_region.minimum = Eigen::Vector3d::Constant(-10.0);
  candidate.protected_region.maximum = Eigen::Vector3d::Constant(10.0);
  candidate.role_schedule = {
      {0.0, 1.5, navigation_planning::CandidateRole::kMain},
      {1.5, 2.0, navigation_planning::CandidateRole::kBackup}};
  candidate.evaluator = [](const std::int64_t stamp_ns,
                           navigation_planning::TrajectoryPoint& point) {
    point.trajectory_time_s = static_cast<double>(stamp_ns - 10000000000LL) * 1.0e-9;
    point.position_world = Eigen::Vector3d{1.0 + 3.0 * point.trajectory_time_s, 0.0, 0.0};
    point.velocity_world = Eigen::Vector3d{3.0, 0.0, 0.0};
    point.role = point.trajectory_time_s < 1.5
        ? navigation_planning::CandidateRole::kMain
        : navigation_planning::CandidateRole::kBackup;
    return true;
  };
  const auto now = candidate.sampleAtDeclaredStamp(10500000000LL);
  const auto source = candidate.sampleAtDeclaredStamp(10488000000LL);
  ASSERT_TRUE(now.has_value());
  ASSERT_TRUE(source.has_value());

  EXPECT_FALSE(assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{2.464, 0.26, 0.0}, Eigen::Vector3d{3.0, 0.0, 0.0},
      10500000000LL, 10488000000LL,
      100000000LL, 0.1, 0.25, 0.75, true, true).accepted());
  EXPECT_FALSE(assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{2.464, 0.0, 0.0}, Eigen::Vector3d{-3.0, 0.0, 0.0},
      10500000000LL, 10488000000LL,
      100000000LL, 0.1, 0.25, 0.75, true, true).accepted());
  EXPECT_FALSE(assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{2.464, 0.0, 0.0}, Eigen::Vector3d{3.0, 0.0, 0.0},
      10500000000LL, 10488000000LL,
      100000000LL, 0.1, 0.25, 0.75, false, true).accepted());
  EXPECT_FALSE(assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{2.464, 0.0, 0.0}, Eigen::Vector3d{3.0, 0.0, 0.0},
      10500000000LL, 10488000000LL,
      100000000LL, 0.1, 0.25, 0.75, true, false).accepted());
  EXPECT_FALSE(assessPhaseExecutionCertificate(
      candidate, Eigen::Vector3d{2.464, 0.0, 0.0}, Eigen::Vector3d{3.0, 0.0, 0.0},
      10500000000LL, 10488000000LL, 10000000LL, 0.1, 0.25, 0.75,
      true, true).accepted());
}

TEST(PlannerFsm, PhaseBridgePreservesOnlyCurrentMainWithinLease) {
  const auto accepted = [&](bool plan_from_rest, ExecutionRecoveryState state,
                            std::uint64_t state_epoch, bool failure_latched,
                            std::int64_t valid_until_ns) {
    return phaseExecutionBridgeMayPreserveMain(
        true, true, true, true, plan_from_rest, state, state_epoch, 7U,
        failure_latched, 10000000000LL, valid_until_ns, 100000000LL);
  };

  EXPECT_TRUE(accepted(false, ExecutionRecoveryState::kTrackMain, 7U, false,
                      10200000000LL));
  EXPECT_FALSE(accepted(true, ExecutionRecoveryState::kTrackMain, 7U, false,
                       10200000000LL));
  EXPECT_FALSE(accepted(false, ExecutionRecoveryState::kStoppedRecovery, 7U, false,
                       10200000000LL));
  EXPECT_FALSE(accepted(false, ExecutionRecoveryState::kTrackMain, 8U, false,
                       10200000000LL));
  EXPECT_FALSE(accepted(false, ExecutionRecoveryState::kTrackMain, 7U, true,
                       10200000000LL));
  EXPECT_FALSE(accepted(false, ExecutionRecoveryState::kTrackMain, 7U, false,
                       10050000000LL));

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
            PlannerResultDisposition::RetainCommittedCommand);
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kEmergency, false, false, false),
            PlannerResultDisposition::FailClosed);
  EXPECT_EQ(classifyPlannerResult(navigation_planning::PlannerStatus::kOptimizationFailed, true, false, false),
            PlannerResultDisposition::FailClosed);
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

TEST(PlannerFsm, RetainsPredecessorAcrossDesiredWaypointEpochAdvance) {
  // The desired successor is already current, but the predecessor remains the
  // physical execution owner until its certified successor takes over.
  EXPECT_TRUE(retainedCommandMatchesExecutionIdentity(
      true, true, 4U, 4U, 7U, 7U, 10U, true, 10U, true));

  EXPECT_FALSE(retainedCommandMatchesExecutionIdentity(
      true, true, 4U, 4U, 7U, 8U, 10U, true, 10U, true));
  EXPECT_FALSE(retainedCommandMatchesExecutionIdentity(
      true, true, 4U, 4U, 7U, 7U, 10U, true, 11U, true));
  EXPECT_FALSE(retainedCommandMatchesExecutionIdentity(
      true, true, 4U, 4U, 7U, 7U, 10U, true, 10U, false));
  EXPECT_FALSE(retainedCommandMatchesExecutionIdentity(
      true, true, 4U, 5U, 7U, 7U, 10U, true, 10U, true));
}

TEST(PlannerFsm, TransfersOnlyAValidatedCoincidentTerminalSuccessorHold) {
  EXPECT_TRUE(terminalSuccessorHoldMayTransfer(
      true, true, true, true, true, true, true, true, true, true, true));

  for (const auto index : {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U}) {
    bool facts[] = {true, true, true, true, true, true, true, true, true, true, true};
    facts[index] = false;
    EXPECT_FALSE(terminalSuccessorHoldMayTransfer(
        facts[0], facts[1], facts[2], facts[3], facts[4], facts[5], facts[6],
        facts[7], facts[8], facts[9], facts[10])) << index;
  }
}

TEST(PlannerFsm, PassThroughHotRetargetsFromCertifiedFiniteCommand) {
  EXPECT_TRUE(canHotRetargetAtWaypointTransition(false, true, true, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(true, true, true, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, false, true, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, true, false, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, true, true, true, false));
  // A moving safety suffix remains owned by BACKUP/EMERGENCY until a
  // certified stop; a new nominal goal cannot rebind it as TrackMain.
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, true, true, false, true));
}

TEST(PlannerFsm, WatchdogRetainsOnlyMovingCertifiedSafetySuffix) {
  EXPECT_TRUE(watchdogTimeoutMayRetainSafetySuffix(
      ExecutionRecoveryState::kTrackBackup, true, true));
  EXPECT_TRUE(watchdogTimeoutMayRetainSafetySuffix(
      ExecutionRecoveryState::kEmergencyBrake, true, true));
  EXPECT_FALSE(watchdogTimeoutMayRetainSafetySuffix(
      ExecutionRecoveryState::kTrackMain, true, true));
  EXPECT_FALSE(watchdogTimeoutMayRetainSafetySuffix(
      ExecutionRecoveryState::kTrackBackup, false, true));
  EXPECT_FALSE(watchdogTimeoutMayRetainSafetySuffix(
      ExecutionRecoveryState::kTrackBackup, true, false));
}

TEST(PlannerFsm, HotRetargetUsesOnlyCertifiedFutureMainState) {
  EXPECT_TRUE(hotRetargetUsesCommittedFutureState(
      true, true, navigation_planning::CandidateRole::kMain, 0.75, 0.75));
  EXPECT_FALSE(hotRetargetUsesCommittedFutureState(
      true, true, navigation_planning::CandidateRole::kMain, 0.751, 0.75));
  EXPECT_FALSE(hotRetargetUsesCommittedFutureState(
      true, true, navigation_planning::CandidateRole::kBackup, 0.2, 0.75));
  EXPECT_FALSE(hotRetargetUsesCommittedFutureState(
      true, false, navigation_planning::CandidateRole::kMain, 0.2, 0.75));
  EXPECT_FALSE(hotRetargetUsesCommittedFutureState(
      false, true, navigation_planning::CandidateRole::kMain, 0.2, 0.75));
}

TEST(PlannerFsm, AcceptsSafetySuffixWhenVehicleIsAlreadyOnBackup) {
  const double elapsed_s = 2.5;
  const double original_backup_start_s = 2.0;
  const double effective_safety_start_s =
      std::max(original_backup_start_s, elapsed_s);
  EXPECT_TRUE(committedSafetySuffixIsUsable(
      true, elapsed_s, 4.0, effective_safety_start_s, 0.2, 0.75, true));
}

TEST(PlannerFsm, DoesNotEnterBackupBeforeRetainedBundleReachesBackupInterval) {
  EXPECT_FALSE(retainedSafetyTransitionMayActivateBackup(
      true, true, navigation_planning::CandidateRole::kMain));
  EXPECT_TRUE(retainedSafetyTransitionMayActivateBackup(
      true, true, navigation_planning::CandidateRole::kBackup));
  EXPECT_FALSE(retainedSafetyTransitionMayActivateBackup(
      false, true, navigation_planning::CandidateRole::kBackup));
  EXPECT_FALSE(retainedSafetyTransitionMayActivateBackup(
      true, false, navigation_planning::CandidateRole::kBackup));
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
