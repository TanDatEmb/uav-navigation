#include "navigation_runtime/planner_fsm.hpp"
#include "navigation_runtime/commit_trace.hpp"

#include <gtest/gtest.h>

namespace navigation_runtime {
namespace {

TEST(PlannerFsm, AcceptsSuccessfulPlannerResults) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::SUCCESS, true, false, true),
            PlannerResultDisposition::CommandReady);
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::NO_NEED, false, true, false),
            PlannerResultDisposition::ValidateRetainedCommand);
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::FINISH, false, true, true),
            PlannerResultDisposition::CommandReady);
}

TEST(PlannerFsm, NoNeedWithoutCommittedCommandFailsClosed) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::NO_NEED, false, false, false),
            PlannerResultDisposition::FailClosed);
}

TEST(PlannerFsm, RetainedValidationPreservesValidStateAndFailsClosedOtherwise) {
  EXPECT_EQ(retainedValidationTransition(true),
            RetainedValidationTransition::PreserveExistingState);
  EXPECT_EQ(retainedValidationTransition(false),
            RetainedValidationTransition::FailClosed);
}

TEST(PlannerFsm, SuccessWithoutNewCommittedGenerationFailsClosed) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::SUCCESS, false, true, false),
            PlannerResultDisposition::FailClosed);
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::FINISH, true, false, false),
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
}

TEST(PlannerFsm, RestartsAtLocalTrajectoryBoundary) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::NEW_TRAJ, false, true, false),
            PlannerResultDisposition::RestartFromRest);
}

TEST(PlannerFsm, RetriesTransientPlannerFailures) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::FAILED, true, false, false),
            PlannerResultDisposition::RetryFromRest);
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::FAILED, false, true, false),
            PlannerResultDisposition::RetainCommittedCommand);
}

TEST(PlannerFsm, FailsClosedForEmergencyOrUnrecoverableFailures) {
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::FAILED, false, false, false),
            PlannerResultDisposition::FailClosed);
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::EMER, false, true, false),
            PlannerResultDisposition::FailClosed);
  EXPECT_EQ(classifyPlannerResult(navigation_planning_backend::OPT_FAILED, true, false, false),
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

TEST(PlannerFsm, RetainsVisibleMainOnlyTrajectoryAfterTransientReplanFailure) {
  EXPECT_TRUE(committedSafetySuffixIsUsable(
      false, 0.8, 1.395, 0.8, 0.187, 0.75, true));
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      false, 0.8, 1.395, 0.8, 0.187, 0.75, false));
  EXPECT_FALSE(committedSafetySuffixIsUsable(
      false, 1.395, 1.395, 1.395, 0.187, 0.75, true));
}

TEST(PlannerFsm, PassThroughHotRetargetsOnlyFromNominalCommand) {
  EXPECT_TRUE(canHotRetargetAtWaypointTransition(false, true, true, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(true, true, true, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, false, true, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, true, false, false, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, true, true, true, false));
  EXPECT_FALSE(canHotRetargetAtWaypointTransition(false, true, true, false, true));
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
  // CmdTraj snapshot.
  constexpr std::uint64_t committed_generation = 8U;
  constexpr double total_duration_s = 2.0;
  constexpr double elapsed_s = 0.5;

  for (int failure = 0; failure < 3; ++failure) {
    EXPECT_EQ(classifyPlannerResult(
                  navigation_planning_backend::FAILED, false, true, false),
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
