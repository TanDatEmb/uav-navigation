#include <memory>

#include <gtest/gtest.h>

#include <navigation_execution/command_sampler.hpp>
#include <navigation_execution/execution_state_store.hpp>

namespace {

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
  candidate.backup_start_time_s = 0.0;
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

TEST(CommittedBundleStore, CommitRequiresCurrentWorldAndGoal) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));

  navigation_execution::CommitToken token{world, 7, 1};
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  EXPECT_EQ(store.tryCommit(token, candidate), navigation_execution::CommitDecision::kCommitted);

  auto advanced = world;
  advanced.revision = 2;
  advanced.observation_stamp_ns = 2;
  ASSERT_TRUE(store.publishWorldIdentity(advanced));
  EXPECT_EQ(store.tryCommit(token, candidate), navigation_execution::CommitDecision::kWorldAdvanced);
  EXPECT_FALSE(store.load());

  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 2));
  EXPECT_EQ(store.tryCommit({advanced, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);
  EXPECT_TRUE(store.load());
}

TEST(CommittedBundleStore, RejectsOutOfOrderTransactionIdentity) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto first = std::make_shared<const navigation_planning::CandidateBundle>(candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 2}, first),
            navigation_execution::CommitDecision::kCommitted);
  auto stale = std::make_shared<const navigation_planning::CandidateBundle>(candidateFor(7, 1));
  EXPECT_EQ(store.tryCommit({world, 7, 1}, stale),
            navigation_execution::CommitDecision::kCancelled);
  EXPECT_EQ(store.load(), first);
}

TEST(CommittedBundleStore, FinalizerFailureRestoresPreviousExecutionPointer) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));

  auto previous = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, previous),
            navigation_execution::CommitDecision::kCommitted);

  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  EXPECT_EQ(store.tryCommitAndFinalize(
                {world, 7, 2}, replacement, [] { return false; }),
            navigation_execution::CommitDecision::kFinalizationFailed);
  EXPECT_EQ(store.load(), previous);

  auto after_rollback = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  EXPECT_EQ(store.tryCommitAndFinalize(
                {world, 7, 2}, after_rollback, [] { return true; }),
            navigation_execution::CommitDecision::kCommitted);
  EXPECT_EQ(store.load(), after_rollback);
}

TEST(CommittedBundleStore, SuccessfulFinalizerKeepsReplacementPointer) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));

  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  bool finalized = false;
  EXPECT_EQ(store.tryCommitAndFinalize(
                {world, 7, 2}, replacement, [&finalized] {
                  finalized = true;
                  return true;
                }),
            navigation_execution::CommitDecision::kCommitted);
  EXPECT_TRUE(finalized);
  EXPECT_EQ(store.load(), replacement);
}

TEST(CommittedBundleStore, GoalReplacementInvalidatesAndSamplerDoesNotLockWorld) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  navigation_execution::CommandSampler sampler(store);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50)));
  ASSERT_TRUE(store.setActiveGoalEpoch(8));
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50)));
}

TEST(CommandSampler, RejectsRetainedBundleFromPreviousGoal) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  // Hot-retarget may retain the physical pointer while the new goal is
  // waiting for a fresh solve.  The sampler must not expose or relabel it.
  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  navigation_execution::CommandSampler sampler(store);
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50, 8)));
}

TEST(CommittedBundleStore, RetainedCommandStaysOldUntilSuccessorActivation) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  navigation_execution::CommandSampler sampler(store);
  const auto retained = store.load();
  ASSERT_TRUE(retained);
  EXPECT_EQ(retained->goal_epoch, 7U);
  EXPECT_EQ(retained->request_id, 17U);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50, 7)));
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50, 8)));

  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto successor = candidateFor(8, 1);
  successor.request_id = 20U;
  successor.valid_from_ns = 50;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(successor);
  ASSERT_EQ(navigation_execution::candidateMatchesAnchor(*successor_ptr, *anchor),
            navigation_execution::AnchorMatchResult::kMatch);
  ASSERT_EQ(store.stagePending({world, 8, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);
  EXPECT_EQ(store.load(), retained);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50, 7)));
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50, 8)));

  ASSERT_TRUE(store.activatePendingIfDue(50));
  EXPECT_EQ(store.load(), successor_ptr);
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50, 7)));
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50, 8)));
}

TEST(ExecutionTimelineStore, RejectsSuccessorWhenPredecessorAdvanced) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);

  auto replacement_data = candidateFor(7, 1);
  replacement_data.bundle_generation = active->bundle_generation + 1U;
  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      replacement_data);
  ASSERT_EQ(store.tryCommit({world, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);

  EXPECT_EQ(store.stagePending({world, 7, 3}, *anchor, successorFor(*anchor, 7)),
            navigation_execution::StageDecision::kPredecessorAdvanced);
  EXPECT_EQ(store.load(), replacement);
  EXPECT_FALSE(store.hasPending());
}

TEST(ExecutionTimelineStore, RejectsFinalizedSuccessorWhenPredecessorAdvanced) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);

  auto replacement_data = candidateFor(7, 1);
  replacement_data.bundle_generation = active->bundle_generation + 1U;
  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      replacement_data);
  ASSERT_EQ(store.tryCommit({world, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);

  bool finalized = false;
  EXPECT_EQ(store.stagePendingAndFinalize(
                {world, 7, 3}, *anchor, successorFor(*anchor, 7),
                [&finalized] {
                  finalized = true;
                  return true;
                }),
            navigation_execution::StageDecision::kPredecessorAdvanced);
  EXPECT_FALSE(finalized);
  EXPECT_EQ(store.load(), replacement);
  EXPECT_FALSE(store.hasPending());
}

TEST(ExecutionTimelineStore, FailedPendingFinalizationRestoresPriorPending) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);

  const auto first_anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(first_anchor);
  auto first_pending = successorFor(*first_anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *first_anchor, first_pending),
            navigation_execution::StageDecision::kStaged);

  const auto replacement_anchor = store.reserveAnchor(60, 70);
  ASSERT_TRUE(replacement_anchor);
  auto replacement_pending = successorFor(*replacement_anchor, 7);
  EXPECT_EQ(store.stagePendingAndFinalize(
                {world, 7, 3}, *replacement_anchor, replacement_pending,
                [] { return false; }),
            navigation_execution::StageDecision::kFinalizationFailed);
  EXPECT_EQ(store.loadPending(), first_pending);
  EXPECT_EQ(store.pendingActivationNs(), 50);
  EXPECT_TRUE(store.activatePendingIfDue(50));
  EXPECT_EQ(store.load(), first_pending);
}

TEST(ExecutionTimelineStore, AcceptsAnchorAtExactMainBackupBoundary) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active_data = candidateFor(7, 1);
  active_data.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  active_data.backup_available = true;
  active_data.backup_start_time_s = 49.0e-9;
  active_data.role_schedule = {
      {0.0, 49.0e-9, navigation_planning::CandidateRole::kMain},
      {49.0e-9, 399.0e-9, navigation_planning::CandidateRole::kBackup}};
  active_data.evaluator = [](std::int64_t stamp,
                             navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = static_cast<double>(stamp);
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    point.role = stamp >= 50 ? navigation_planning::CandidateRole::kBackup
                             : navigation_planning::CandidateRole::kMain;
    return true;
  };
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(active_data);
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);

  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  EXPECT_EQ(anchor->active_role, navigation_planning::CandidateRole::kBackup);
  EXPECT_EQ(store.stagePending({world, 7, 2}, *anchor, successorFor(*anchor, 7)),
            navigation_execution::StageDecision::kStaged);
}

TEST(ExecutionTimelineStore, RecertifiedPredecessorKeepsSuccessorActivationValid) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto successor = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor),
            navigation_execution::StageDecision::kStaged);
  const auto old_timeline = store.snapshot();
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};

  ASSERT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, old_timeline.version, old_timeline.active, true, 300,
                old_timeline.pending, true),
            navigation_world_model::WorldCommitDecision::kCommitted);
  EXPECT_NE(store.load(), active);
  EXPECT_TRUE(store.hasPending());
  EXPECT_TRUE(store.activatePendingIfDue(50));
  EXPECT_EQ(store.load()->bundle_generation, successor->bundle_generation);
}

TEST(ExecutionTimelineStore, RejectsActivationWhenActiveWasInvalidatedButPendingRetained) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto successor = successorFor(*anchor, 7);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor),
            navigation_execution::StageDecision::kStaged);
  const auto old_timeline = store.snapshot();
  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};

  ASSERT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, old_timeline.version, old_timeline.active, false, 0,
                old_timeline.pending, true),
            navigation_world_model::WorldCommitDecision::kCommitted);
  EXPECT_FALSE(store.load());
  EXPECT_TRUE(store.hasPending());
  EXPECT_FALSE(store.activatePendingIfDue(50));
  EXPECT_FALSE(store.hasPending());
}

TEST(CommandSampler, RetainsFutureBundleUntilItsSampleValidityBoundary) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  std::size_t evaluations = 0;
  auto candidate = candidateFor(7, 1);
  candidate.valid_from_ns = 100;
  candidate.valid_until_ns = 200;
  candidate.activation_stamp_ns = 100;
  candidate.start_wall_time_s = 100.0e-9;
  candidate.duration_s = 300.0e-9;
  candidate.declared_start_ns = 100;
  candidate.declared_end_ns = 400;
  candidate.role_schedule = {
      {0.0, 300.0e-9, navigation_planning::CandidateRole::kMain}};
  candidate.evaluator = [&evaluations](
      std::int64_t stamp, navigation_planning::TrajectoryPoint& point) {
    ++evaluations;
    point.position_world.x() = static_cast<double>(stamp);
    return true;
  };
  auto committed = std::make_shared<const navigation_planning::CandidateBundle>(candidate);
  ASSERT_EQ(store.tryCommit({world, 7, 1}, committed),
            navigation_execution::CommitDecision::kCommitted);

  navigation_execution::CommandSampler sampler(store);
  const auto before_activation = sampler.sample(99);
  EXPECT_FALSE(static_cast<bool>(before_activation));
  ASSERT_TRUE(before_activation.bundle);
  EXPECT_TRUE(before_activation.awaiting_activation);
  EXPECT_EQ(before_activation.status,
            navigation_execution::SampleStatus::kAwaitingActivation);
  EXPECT_EQ(evaluations, 0U);

  const auto active = sampler.sample(100);
  ASSERT_TRUE(static_cast<bool>(active));
  EXPECT_FALSE(active.awaiting_activation);
  EXPECT_EQ(active.status, navigation_execution::SampleStatus::kActiveSample);
  EXPECT_EQ(evaluations, 1U);
  EXPECT_DOUBLE_EQ(active.point->position_world.x(), 100.0);

  const auto expired = sampler.sample(201);
  EXPECT_FALSE(static_cast<bool>(expired));
  ASSERT_TRUE(expired.bundle);
  EXPECT_FALSE(expired.awaiting_activation);
  EXPECT_EQ(expired.status, navigation_execution::SampleStatus::kExpiredLease);
  EXPECT_EQ(evaluations, 1U);

  const auto planned_hold = sampler.sample(401);
  ASSERT_TRUE(static_cast<bool>(planned_hold));
  EXPECT_TRUE(planned_hold.planned_stop_hold);
  EXPECT_EQ(planned_hold.status, navigation_execution::SampleStatus::kStoppedHold);
  EXPECT_TRUE(planned_hold.point->finished);
  // The sampler returns the endpoint only as a typed hold fallback; it does
  // not evaluate the expired command again.
  EXPECT_EQ(evaluations, 2U);
}

TEST(CommandSampler, SamplesDeclaredMainToBackupBundleAcrossRoleBoundary) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = candidateFor(7, 1);
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.backup_start_time_s = 49.0e-9;
  candidate.role_schedule = {
      {0.0, 49.0e-9, navigation_planning::CandidateRole::kMain},
      {49.0e-9, 399.0e-9, navigation_planning::CandidateRole::kBackup}};
  candidate.evaluator = [](std::int64_t stamp,
                           navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = static_cast<double>(stamp);
    point.role = stamp < 50
                     ? navigation_planning::CandidateRole::kMain
                     : navigation_planning::CandidateRole::kBackup;
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    return true;
  };
  auto committed =
      std::make_shared<const navigation_planning::CandidateBundle>(candidate);
  ASSERT_EQ(store.tryCommit({world, 7, 1}, committed),
            navigation_execution::CommitDecision::kCommitted);

  navigation_execution::CommandSampler sampler(store);
  const auto main = sampler.sample(49, 7);
  ASSERT_TRUE(static_cast<bool>(main));
  EXPECT_EQ(main.point->role, navigation_planning::CandidateRole::kMain);
  const auto backup = sampler.sample(50, 7);
  ASSERT_TRUE(static_cast<bool>(backup));
  EXPECT_EQ(backup.point->role, navigation_planning::CandidateRole::kBackup);
}

TEST(ExecutionTimelineStore, StagesSuccessorUntilFutureAnchorActivation) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);

  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(navigation_execution::candidateMatchesAnchor(*successor_ptr, *anchor),
            navigation_execution::AnchorMatchResult::kMatch);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);
  EXPECT_EQ(store.load(), active);
  EXPECT_EQ(store.loadPending(), successor_ptr);

  navigation_execution::CommandSampler sampler(store);
  EXPECT_EQ(sampler.sample(49, 7).bundle, active);
  ASSERT_TRUE(store.activatePendingIfDue(50));
  const auto activated = sampler.sample(50, 7);
  ASSERT_TRUE(activated);
  EXPECT_EQ(activated.bundle, successor_ptr);
  EXPECT_FALSE(store.hasPending());
}

TEST(ExecutionTimelineStore, RenewsSuccessorsWithoutAnExecutionPointerGap) {
  navigation_execution::ExecutionTimelineStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));

  auto initial = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, initial),
            navigation_execution::CommitDecision::kCommitted);
  navigation_execution::CommandSampler sampler(store);

  const auto first_anchor = store.reserveAnchor(20, 50);
  ASSERT_TRUE(first_anchor);
  auto first_successor = candidateFor(7, 1);
  first_successor.bundle_generation = 28;
  first_successor.valid_from_ns = 50;
  first_successor.valid_until_ns = 90;
  first_successor.activation_stamp_ns = 50;
  auto first_successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      first_successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *first_anchor, first_successor_ptr),
            navigation_execution::StageDecision::kStaged);
  ASSERT_EQ(store.load(), initial);
  ASSERT_TRUE(store.activatePendingIfDue(50));
  ASSERT_EQ(store.load(), first_successor_ptr);

  const auto second_anchor = store.reserveAnchor(60, 70);
  ASSERT_TRUE(second_anchor);
  auto second_successor = first_successor;
  second_successor.bundle_generation = 29;
  second_successor.valid_from_ns = 70;
  second_successor.valid_until_ns = 95;
  second_successor.activation_stamp_ns = 70;
  auto second_successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      second_successor);
  ASSERT_EQ(store.stagePending({world, 7, 3}, *second_anchor, second_successor_ptr),
            navigation_execution::StageDecision::kStaged);
  EXPECT_EQ(store.load(), first_successor_ptr);
  EXPECT_TRUE(store.hasPending());
  ASSERT_TRUE(store.activatePendingIfDue(70));
  EXPECT_EQ(store.load(), second_successor_ptr);
  EXPECT_FALSE(store.hasPending());

  const auto sampled = sampler.sample(70, 7);
  ASSERT_TRUE(sampled);
  EXPECT_EQ(sampled.bundle, second_successor_ptr);
  EXPECT_EQ(sampled.point->position_world.x(), 70.0);
}

TEST(ExecutionTimelineStore, WorldAdvanceInvalidatesPendingSuccessor) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  ASSERT_TRUE(store.publishWorldIdentity(next_world));
  EXPECT_FALSE(store.hasPending());
  EXPECT_FALSE(store.load());
}

TEST(ExecutionTimelineStore, KeepsPendingSuccessorOnlyAfterExplicitWorldRecertification) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  const navigation_world_model::WorldSnapshotIdentity next_world{3, 4, 2, 2};
  EXPECT_FALSE(store.recertifyPendingWorldIdentity(next_world, nullptr));
  const auto recertified_pending = store.recertifyPendingWorldIdentity(
      next_world, successor_ptr);
  ASSERT_TRUE(recertified_pending);
  ASSERT_TRUE(store.publishWorldIdentity(
      next_world, {}, false, 0, *recertified_pending, true));
  const auto recertified = store.loadPending();
  ASSERT_TRUE(recertified);
  EXPECT_EQ(recertified->world_identity.revision, next_world.revision);
}

TEST(ExecutionTimelineStore, ActivationFinalizesPlannerOnlyAtSwapBoundary) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  std::uint64_t finalized_generation = 0;
  EXPECT_TRUE(store.activatePendingIfDueAndFinalize(50, [&](const std::uint64_t generation) {
    finalized_generation = generation;
    return true;
  }));
  EXPECT_EQ(finalized_generation, successor_ptr->bundle_generation);
  EXPECT_EQ(store.load(), successor_ptr);
  EXPECT_FALSE(store.hasPending());
}

TEST(ExecutionTimelineStore, ActivationFinalizerFailureKeepsOldAndDropsPending) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  EXPECT_FALSE(store.activatePendingIfDueAndFinalize(50, [](std::uint64_t) {
    return false;
  }));
  EXPECT_EQ(store.load(), active);
  EXPECT_FALSE(store.hasPending());
}

TEST(ExecutionTimelineStore, MissedActivationKeepsActiveCommandAndDropsSuccessor) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 60;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, successor_ptr),
            navigation_execution::StageDecision::kStaged);

  // Pending activation is an execution-store transition owned by the command
  // publisher; CommandSampler must remain read-only.
  EXPECT_FALSE(store.activatePendingIfDue(61));
  navigation_execution::CommandSampler sampler(store);
  const auto missed = sampler.sample(61, 7);
  ASSERT_TRUE(missed);
  EXPECT_EQ(missed.bundle, active);
  EXPECT_EQ(store.load(), active);
  EXPECT_FALSE(store.hasPending());
}

TEST(ExecutionTimelineStore, FinalizerFailureRestoresPendingTransactionWatermark) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor.has_value());
  auto successor = candidateFor(7, 1);
  successor.valid_from_ns = 50;
  successor.valid_until_ns = 90;
  successor.activation_stamp_ns = 50;
  auto successor_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      successor);
  EXPECT_EQ(store.stagePendingAndFinalize(
                {world, 7, 2}, *anchor, successor_ptr, [] { return false; }),
            navigation_execution::StageDecision::kFinalizationFailed);
  EXPECT_FALSE(store.hasPending());

  auto retry = std::make_shared<const navigation_planning::CandidateBundle>(successor);
  EXPECT_EQ(store.stagePending({world, 7, 2}, *anchor, retry),
            navigation_execution::StageDecision::kStaged);
}

TEST(CommittedBundleStore, ExposureRejectsBundleInvalidatedAfterSampling) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  const auto sampled = store.load();
  ASSERT_EQ(sampled.get(), candidate.get());
  auto advanced = world;
  advanced.revision = 2;
  advanced.observation_stamp_ns = 2;
  ASSERT_TRUE(store.publishWorldIdentity(advanced));

  bool exposed = false;
  EXPECT_FALSE(store.publishIfCurrent(sampled, 7, [&] { exposed = true; }));
  EXPECT_FALSE(exposed);
}

TEST(CommittedBundleStore, ExposureKeepsRetainedExecutionBundleUntilActivation) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  const auto sampled = store.load();
  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  bool exposed = false;
  EXPECT_TRUE(store.publishIfCurrent(sampled, 7, [&] { exposed = true; }));
  EXPECT_TRUE(exposed);
}

TEST(CommittedBundleStore, RecertifiesOnlyTheValidatedBundleOnWorldAdvance) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};
  ASSERT_TRUE(store.publishWorldIdentity(next_world, candidate, true));
  const auto recertified = store.load();
  ASSERT_TRUE(recertified);
  EXPECT_NE(recertified.get(), candidate.get());
  EXPECT_TRUE(navigation_world_model::sameWorldSnapshotIdentity(
      recertified->world_identity, next_world));
  EXPECT_TRUE(store.publishIfCurrent(recertified, 7, [] {}));
}

TEST(CommittedBundleStore, RecertificationRenewsOnlyTheValidatedExecutionWindow) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = candidateFor(7, 1);
  candidate.valid_until_ns = 100;
  auto committed = std::make_shared<const navigation_planning::CandidateBundle>(candidate);
  ASSERT_EQ(store.tryCommit({world, 7, 1}, committed),
            navigation_execution::CommitDecision::kCommitted);

  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};
  ASSERT_TRUE(store.publishWorldIdentity(next_world, committed, true, 300));
  const auto recertified = store.load();
  ASSERT_TRUE(recertified);
  EXPECT_EQ(recertified->valid_until_ns, 300);
  EXPECT_TRUE(store.publishIfCurrent(recertified, 7, [] {}));
}

TEST(CommittedBundleStore, RecertificationMismatchOrGoalChangeClearsBundle) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  const auto unrelated_bundle = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 99));
  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};
  ASSERT_TRUE(store.publishWorldIdentity(next_world, unrelated_bundle, true));
  EXPECT_FALSE(store.load());
}

TEST(ExecutionTimelineStore, SupersededWorldRefreshPreservesNewCommit) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto first = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, first),
            navigation_execution::CommitDecision::kCommitted);
  const auto old_timeline = store.snapshot();

  auto replacement = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 2));
  auto replacement_copy = std::make_shared<navigation_planning::CandidateBundle>(*replacement);
  replacement_copy->world_identity = world;
  replacement_copy->pinned_world_identity = world;
  replacement = std::shared_ptr<const navigation_planning::CandidateBundle>(
      std::move(replacement_copy));
  ASSERT_EQ(store.tryCommit({world, 7, 2}, replacement),
            navigation_execution::CommitDecision::kCommitted);
  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};
  EXPECT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, old_timeline.version, old_timeline.active, true, 300),
            navigation_world_model::WorldCommitDecision::kSuperseded);
  EXPECT_EQ(store.load(), replacement);
  EXPECT_EQ(store.snapshot().world_identity->revision, world.revision);
}

TEST(ExecutionTimelineStore, RefreshKeepsActiveWhenPendingIsInvalid) {
  navigation_execution::ExecutionTimelineStore store;
  const navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto active = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, active),
            navigation_execution::CommitDecision::kCommitted);
  const auto anchor = store.reserveAnchor(50, 50);
  ASSERT_TRUE(anchor);
  auto pending = candidateFor(7, 1);
  pending.valid_from_ns = 50;
  pending.valid_until_ns = 90;
  pending.activation_stamp_ns = 50;
  auto pending_ptr = std::make_shared<const navigation_planning::CandidateBundle>(pending);
  ASSERT_EQ(store.stagePending({world, 7, 2}, *anchor, pending_ptr),
            navigation_execution::StageDecision::kStaged);
  const auto old_timeline = store.snapshot();
  const auto next_world = navigation_world_model::WorldSnapshotIdentity{3, 4, 2, 2};

  ASSERT_EQ(store.publishWorldIdentityIfCurrent(
                next_world, old_timeline.version, old_timeline.active, true, 300,
                old_timeline.pending, false),
            navigation_world_model::WorldCommitDecision::kCommitted);
  ASSERT_TRUE(store.load());
  EXPECT_NE(store.load(), active);
  EXPECT_EQ(store.load()->valid_from_ns, active->valid_from_ns);
  EXPECT_FALSE(store.loadPending());
  EXPECT_EQ(store.snapshot().world_identity->revision, next_world.revision);
}

TEST(ExecutionStateStore, RejectsOldEpochAndClearsStateOnReset) {
  navigation_execution::ExecutionStateStore store;
  navigation_planning::KinematicState state;
  state.position_world.x() = 1.0;
  state.source_stamp_ns = 10;
  state.receive_stamp_ns = 20;
  state.localization_epoch = 4;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  ASSERT_TRUE(store.publish(state));
  ASSERT_TRUE(store.load());
  EXPECT_EQ(store.load()->ingress_sequence, 1U);
  EXPECT_FALSE(store.publish(state));
  store.resetForLocalizationEpoch(5);
  EXPECT_FALSE(store.load());
  state.localization_epoch = 4;
  state.source_stamp_ns = 11;
  EXPECT_FALSE(store.publish(state));
  state.localization_epoch = 5;
  EXPECT_TRUE(store.publish(state));
  ASSERT_TRUE(store.load());
  EXPECT_EQ(store.load()->ingress_sequence, 2U);
}

TEST(ExecutionStateStore, RejectsWildcardResetEpoch) {
  navigation_execution::ExecutionStateStore store;
  EXPECT_FALSE(store.resetForLocalizationEpoch(0));

  navigation_planning::KinematicState state;
  state.source_stamp_ns = 10;
  state.receive_stamp_ns = 20;
  state.localization_epoch = 4;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  ASSERT_TRUE(store.publish(state));
  EXPECT_TRUE(store.load());
  EXPECT_TRUE(store.resetForLocalizationEpoch(5));
  EXPECT_FALSE(store.load());
}

}  // namespace
