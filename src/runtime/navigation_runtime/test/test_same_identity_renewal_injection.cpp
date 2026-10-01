#include "navigation_runtime/same_identity_renewal_injection.hpp"

#include <memory>
#include <atomic>
#include <future>
#include <latch>

#include <gtest/gtest.h>

#include <navigation_execution/execution_authority.hpp>
#include <navigation_runtime/execution_lifecycle_view.hpp>

namespace navigation_runtime {
namespace {

std::shared_ptr<const navigation_contracts::msg::NavigationGoal> goalFor(
    const std::shared_ptr<const navigation_planning::CandidateBundle>& bundle) {
  auto goal = std::make_shared<navigation_contracts::msg::NavigationGoal>();
  goal->mission_id = "test_mission";
  goal->request_id = bundle->request_id;
  return goal;
}

navigation_planning::CandidateBundle candidateFor(
    std::uint64_t goal_epoch, std::uint64_t revision) {
  navigation_planning::CandidateBundle candidate;
  candidate.world_identity.localization_epoch = 3;
  candidate.world_identity.generation = 4;
  candidate.world_identity.revision = revision;
  candidate.world_identity.observation_stamp_ns = static_cast<std::int64_t>(revision);
  candidate.pinned_world_identity = candidate.world_identity;
  candidate.localization_epoch = 3;
  candidate.goal_epoch = goal_epoch;
  candidate.request_id = goal_epoch + 10;
  candidate.bundle_generation = goal_epoch + 20;
  candidate.valid_from_ns = 1;
  candidate.valid_until_ns = 100;
  candidate.start_wall_time_s = 1.0e-9;
  candidate.duration_s = 399.0e-9;
  candidate.activation_stamp_ns = candidate.valid_from_ns;
  candidate.declared_start_ns = *navigation_common::secondsToNanoseconds(
      candidate.start_wall_time_s);
  candidate.declared_end_ns = *navigation_common::secondsSumToNanoseconds(
      candidate.start_wall_time_s, candidate.duration_s);
  candidate.kind = navigation_planning::CandidateBundleKind::kTerminalStop;
  candidate.certificates = {true, true, true, true};
  candidate.protected_region.minimum = Eigen::Vector3d::Zero();
  candidate.protected_region.maximum = Eigen::Vector3d::Ones();
  candidate.role_schedule = {
      {0.0, 399.0e-9, navigation_planning::CandidateRole::kMain}};
  candidate.evaluator = [](std::int64_t stamp, navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = static_cast<double>(stamp);
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    return true;
  };
  return candidate;
}

std::shared_ptr<const navigation_planning::CandidateBundle> successorFor(
    const navigation_execution::ExecutionAnchor& anchor,
    std::uint64_t goal_epoch) {
  auto successor = candidateFor(goal_epoch, anchor.command_world.revision);
  successor.localization_epoch = anchor.localization_epoch;
  successor.world_identity = anchor.command_world;
  successor.pinned_world_identity = anchor.command_world;
  successor.valid_from_ns = anchor.activation_stamp_ns;
  successor.activation_stamp_ns = anchor.activation_stamp_ns;
  successor.bundle_generation = anchor.active_bundle_generation + 100U;
  return std::make_shared<const navigation_planning::CandidateBundle>(successor);
}

bool publishWorldIdentityForTest(
    navigation_execution::ExecutionAuthority& store,
    const navigation_world_model::WorldSnapshotIdentity& identity) {
  const auto snapshot = store.snapshot();
  return store.publishWorldIdentityIfCurrent(
             identity, snapshot.version, snapshot.active, false, 0,
             snapshot.pending, false) ==
         navigation_world_model::WorldCommitDecision::kCommitted;
}

SameIdentityRenewalFacts eligibleFacts() {
  SameIdentityRenewalFacts facts;
  facts.start_mode = navigation_planning::PlanningStartMode::kCommittedFutureState;
  facts.transition_kind = GoalTransitionKind::kSteady;
  facts.recovery_state = ExecutionRecoveryState::kTrackMain;
  facts.execution_phase = ExecutionPhase::kTrackingMain;
  facts.desired_goal_valid = true;
  facts.executing_goal_valid = true;
  facts.desired_identity_matches_executing = true;
  facts.goal_epoch_matches_command = true;
  facts.active_bundle_valid = true;
  facts.active_bundle_is_main = true;
  facts.active_bundle_identity_current = true;
  facts.no_new_goal = true;
  facts.no_hot_goal_transition = true;
  facts.ordinary_renewal = true;
  facts.no_pending_successor = true;
  facts.command_available = true;
  facts.command_exposure_allowed = true;
  facts.execution_state_fresh = true;
  facts.world_fresh = true;
  facts.valid_future_anchor = true;
  facts.fresh_renewal_due_window = true;
  return facts;
}

TEST(WorldRevocationDelivery, FinalizesLifecycleBeforePublicationReturns) {
  navigation_execution::ExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(store, world));
  ASSERT_TRUE(store.setAdmissionGoalEpoch(7));
  const auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, goalFor(candidate), candidate),
            navigation_execution::CommitDecision::kCommitted);
  const auto before = store.snapshot();
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  unsigned int finalized = 0;
  ASSERT_EQ(store.publishWorldIdentityIfCurrentAndFinalizeRevocation(
                next_world, before.version, before.active, false,
                [&]() noexcept { ++finalized;  }),
            navigation_world_model::WorldCommitDecision::kCommitted);

  // The production callback used to require the old pointer AFTER the store
  // had revoked it. That condition is false, without any concurrent event.
  EXPECT_FALSE(store.load());
  EXPECT_FALSE(store.invalidateIfCurrent(before));
  EXPECT_EQ(finalized, 1U);
  const auto after = store.snapshot();
  EXPECT_TRUE(after.failed());
  EXPECT_FALSE(after.commandAvailable());
  EXPECT_EQ(after.lifecycle.phase, ExecutionPhase::kPx4Hold);
}

class WorldRevocationFixture : public testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(publishWorldIdentityForTest(store, world));
    ASSERT_TRUE(store.setAdmissionGoalEpoch(7));
    predecessor = std::make_shared<const navigation_planning::CandidateBundle>(
        candidateFor(7, 1));
    ASSERT_EQ(store.tryCommit({world, 7, 1}, goalFor(predecessor), predecessor),
              navigation_execution::CommitDecision::kCommitted);
  }

  navigation_execution::ExecutionAuthority store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  std::shared_ptr<const navigation_planning::CandidateBundle> predecessor;
};

TEST_F(WorldRevocationFixture, SupersededRevocationPreservesNewerExecution) {
  const auto before = store.snapshot();
  auto replacement = candidateFor(7, 1);
  ++replacement.bundle_generation;
  const auto newer = std::make_shared<const navigation_planning::CandidateBundle>(replacement);
  ASSERT_EQ(store.tryCommit({world, 7, 2}, goalFor(newer), newer),
            navigation_execution::CommitDecision::kCommitted);
  unsigned int finalized = 0;
  EXPECT_EQ(store.publishWorldIdentityIfCurrentAndFinalizeRevocation(
                next_world, before.version, before.active, false,
                [&]() noexcept { ++finalized;  }),
            navigation_world_model::WorldCommitDecision::kSuperseded);
  EXPECT_EQ(finalized, 0U);
  EXPECT_EQ(store.load(), newer);
  EXPECT_FALSE(store.snapshot().failed());
  EXPECT_EQ(store.snapshot().activeGeneration(), newer->bundle_generation);
}

TEST_F(WorldRevocationFixture, PendingOnlyRejectionKeepsActiveLifecycle) {
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  const auto successor = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, goalFor(successor), successor),
            navigation_execution::StageDecision::kStaged);
  const auto before = store.snapshot();
  ASSERT_TRUE(before.pending);
  unsigned int finalized = 0;
  EXPECT_EQ(store.publishWorldIdentityIfCurrentAndFinalizeRevocation(
                next_world, before.version, before.active, true,
                [&]() noexcept { ++finalized;  },
                0, before.pending, false),
            navigation_world_model::WorldCommitDecision::kCommitted);
  EXPECT_EQ(finalized, 0U);
  ASSERT_TRUE(store.load());
  EXPECT_EQ(store.load()->bundle_generation, predecessor->bundle_generation);
  EXPECT_FALSE(store.snapshot().pending);
  EXPECT_TRUE(store.snapshot().commandAvailable());
  EXPECT_FALSE(store.snapshot().failed());
}

TEST_F(WorldRevocationFixture, RejectedIdentityCannotFinalizeCurrentExecution) {
  const auto before = store.snapshot();
  auto invalid_world = next_world;
  invalid_world.localization_epoch = 0;
  unsigned int finalized = 0;
  EXPECT_EQ(store.publishWorldIdentityIfCurrentAndFinalizeRevocation(
                invalid_world, before.version, before.active, false,
                [&]() noexcept { ++finalized;  }),
            navigation_world_model::WorldCommitDecision::kCandidateRejected);
  EXPECT_EQ(store.publishWorldIdentityIfCurrentAndFinalizeRevocation(
                world, before.version, before.active, false,
                [&]() noexcept { ++finalized;  }),
            navigation_world_model::WorldCommitDecision::kWorldAdvanced);
  EXPECT_EQ(finalized, 0U);
  EXPECT_EQ(store.load(), predecessor);
  EXPECT_FALSE(store.snapshot().failed());
}

TEST_F(WorldRevocationFixture, RepeatedPublicationDoesNotRedeliverRevocation) {
  const auto before = store.snapshot();
  unsigned int finalized = 0;
  const auto finalize = [&]() noexcept { ++finalized;  };
  EXPECT_EQ(store.publishWorldIdentityIfCurrentAndFinalizeRevocation(
                next_world, before.version, before.active, false, finalize),
            navigation_world_model::WorldCommitDecision::kCommitted);
  EXPECT_EQ(store.publishWorldIdentityIfCurrentAndFinalizeRevocation(
                next_world, before.version, before.active, false, finalize),
            navigation_world_model::WorldCommitDecision::kSuperseded);
  EXPECT_EQ(finalized, 1U);
  EXPECT_TRUE(store.snapshot().failed());
}

TEST_F(WorldRevocationFixture, ConcurrentCommitCannotSplitRevocationAndLifecycle) {
  const auto before = store.snapshot();
  auto replacement = candidateFor(7, 2);
  ++replacement.bundle_generation;
  const auto newer = std::make_shared<const navigation_planning::CandidateBundle>(replacement);
  std::latch finalizer_entered{1};
  std::latch commit_attempted{1};
  std::atomic_bool finalized{false};
  std::atomic_bool committed{false};
  bool commit_seen_inside_finalizer = true;
  auto publication = std::async(std::launch::async, [&] {
    return store.publishWorldIdentityIfCurrentAndFinalizeRevocation(
        next_world, before.version, before.active, false,
        [&]() noexcept {
          finalizer_entered.count_down();
          commit_attempted.wait();
          commit_seen_inside_finalizer = committed.load();
          finalized.store(true);
        });
  });
  finalizer_entered.wait();
  auto commit = std::async(std::launch::async, [&] {
    commit_attempted.count_down();
    const auto result = store.tryCommit({next_world, 7, 2}, goalFor(newer), newer);
    committed.store(true);
    const bool finalized_before_commit = finalized.load();
      return std::make_pair(result, finalized_before_commit);
  });
  EXPECT_EQ(publication.get(), navigation_world_model::WorldCommitDecision::kCommitted);
  const auto commit_result = commit.get();
  EXPECT_EQ(commit_result.first, navigation_execution::CommitDecision::kCancelled);
  EXPECT_TRUE(commit_result.second);
  EXPECT_FALSE(commit_seen_inside_finalizer);
  // A later candidate cannot replace or resurrect failed execution authority.
  EXPECT_FALSE(store.snapshot().commandAvailable());
  EXPECT_TRUE(store.snapshot().failed());
}

TEST(SameIdentityRenewalInjection, H1EligibleOrdinaryRenewal) {
  EXPECT_TRUE(sameIdentityRenewalInjectionEligible(eligibleFacts()));
}

TEST(SameIdentityRenewalInjection, H2RejectsWaypointHandoff) {
  auto facts = eligibleFacts();
  facts.transition_kind = GoalTransitionKind::kSameRouteWaypointAdvance;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
  facts = eligibleFacts();
  facts.desired_identity_matches_executing = false;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
}

TEST(SameIdentityRenewalInjection, H3RejectsHotRetarget) {
  auto facts = eligibleFacts();
  facts.no_hot_goal_transition = false;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
}

TEST(SameIdentityRenewalInjection, H4RejectsPlanFromRest) {
  auto facts = eligibleFacts();
  facts.start_mode = navigation_planning::PlanningStartMode::kStoppedMeasuredState;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
}

TEST(SameIdentityRenewalInjection, H5RejectsBackupOwnership) {
  auto facts = eligibleFacts();
  facts.recovery_state = ExecutionRecoveryState::kTrackBackup;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
}

TEST(SameIdentityRenewalInjection, H6RejectsPendingSuccessor) {
  auto facts = eligibleFacts();
  facts.no_pending_successor = false;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
}

TEST(SameIdentityRenewalInjection, RejectsAnchorRecoveryAsOrdinaryRenewal) {
  auto facts = eligibleFacts();
  facts.ordinary_renewal = false;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
}

TEST(SameIdentityRenewalInjection, H7RejectsStaleEvidenceAndBodySupport) {
  auto facts = eligibleFacts();
  facts.world_fresh = false;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
  facts = eligibleFacts();
  facts.execution_state_fresh = false;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
  facts = eligibleFacts();
  facts.current_body_support_present = true;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
}

TEST(SameIdentityRenewalInjection, RejectsLateRenewalOutsideFreshDueWindow) {
  auto facts = eligibleFacts();
  facts.fresh_renewal_due_window = false;
  EXPECT_FALSE(sameIdentityRenewalInjectionEligible(facts));
}

TEST(SameIdentityRenewalInjection, H8OneShotFiresOnlyOnce) {
  SameIdentityRenewalInjectionController controller;
  controller.setTargetOrdinal(1U);
  const auto facts = eligibleFacts();

  const auto first = controller.observe(facts);
  ASSERT_EQ(first, 1U);
  EXPECT_TRUE(controller.shouldInject(first));
  controller.markInjected();

  const auto second = controller.observe(facts);
  EXPECT_EQ(second, 2U);
  EXPECT_FALSE(controller.shouldInject(second));
}

TEST(SameIdentityRenewalInjection, H9OrdinalTargetsOnlyRequestedEligibleRenewal) {
  SameIdentityRenewalInjectionController controller;
  controller.setTargetOrdinal(2U);
  const auto facts = eligibleFacts();

  const auto first = controller.observe(facts);
  EXPECT_EQ(first, 1U);
  EXPECT_FALSE(controller.shouldInject(first));

  const auto second = controller.observe(facts);
  EXPECT_EQ(second, 2U);
  EXPECT_TRUE(controller.shouldInject(second));
  controller.markInjected();

  const auto third = controller.observe(facts);
  EXPECT_EQ(third, 3U);
  EXPECT_FALSE(controller.shouldInject(third));
}

TEST(SameIdentityRenewalInjection,
     FailedRenewalEvidenceKeepsPredecessorUntilLaterAtomicActivation) {
  SameIdentityRenewalInjectionController controller;
  controller.setTargetOrdinal(1U);
  const auto facts = eligibleFacts();

  const auto ordinal = controller.observe(facts);
  ASSERT_TRUE(controller.shouldInject(ordinal));

  navigation_execution::ExecutionAuthority execution_authority;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(publishWorldIdentityForTest(execution_authority, world));
  ASSERT_TRUE(execution_authority.setAdmissionGoalEpoch(7U));
  const auto predecessor =
      std::make_shared<const navigation_planning::CandidateBundle>(candidateFor(7U, 1U));
  ASSERT_EQ(execution_authority.tryCommit({world, 7U, 1U}, goalFor(predecessor), predecessor),
            navigation_execution::CommitDecision::kCommitted);


  // The diagnostic failure is exposed after the real solve, so it must not
  // mutate any execution-owned predecessor state or identity.
  controller.markInjected();
  EXPECT_EQ(execution_authority.load(), predecessor);
  const auto after_failed_renewal = execution_authority.snapshot();
  EXPECT_EQ(after_failed_renewal.active, predecessor);
  EXPECT_FALSE(after_failed_renewal.pending);
  const auto predecessor_execution = execution_authority.snapshot();
  EXPECT_EQ(predecessor_execution.activeGeneration(), predecessor->bundle_generation);
  EXPECT_TRUE(predecessor_execution.commandAvailable());
  EXPECT_EQ(predecessor_execution.admission_goal_epoch, predecessor->goal_epoch);
  EXPECT_EQ(predecessor_execution.admissionRequestId(), predecessor->request_id);

  const auto later_ordinal = controller.observe(facts);
  EXPECT_FALSE(controller.shouldInject(later_ordinal));

  const auto anchor = execution_authority.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  const auto successor = successorFor(*anchor, 7U);
  ASSERT_EQ(execution_authority.stagePending(
                {world, 7U, 2U}, *anchor, goalFor(successor), successor),
            navigation_execution::StageDecision::kStaged);
  EXPECT_EQ(execution_authority.load(), predecessor);
  const auto pending_snapshot = execution_authority.snapshot();
  ASSERT_EQ(pending_snapshot.pending, successor);

  ASSERT_TRUE(execution_authority.activatePendingIfDueAndFinalize(
      50, pending_snapshot, [](std::uint64_t) { return true; }));
  EXPECT_EQ(execution_authority.load(), successor);
  const auto after_activation_snapshot = execution_authority.snapshot();
  const auto successor_execution = execution_authority.snapshot();
  EXPECT_EQ(after_activation_snapshot.activeGeneration(), successor->bundle_generation);
  EXPECT_EQ(successor_execution.activeGeneration(), successor->bundle_generation);
  EXPECT_TRUE(successor_execution.commandAvailable());
  EXPECT_FALSE(execution_authority.snapshot().pending);
}

}  // namespace
}  // namespace navigation_runtime
