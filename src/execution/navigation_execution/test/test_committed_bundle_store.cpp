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

TEST(CommittedBundleStore, RebindsRetainedBundleOnlyAcrossExactGoalIdentity) {
  navigation_execution::CommittedBundleStore store;
  navigation_world_model::WorldSnapshotIdentity world{3, 4, 1, 1};
  ASSERT_TRUE(store.publishWorldIdentity(world));
  ASSERT_TRUE(store.setActiveGoalEpoch(7));
  auto candidate = std::make_shared<const navigation_planning::CandidateBundle>(
      candidateFor(7, 1));
  ASSERT_EQ(store.tryCommit({world, 7, 1}, candidate),
            navigation_execution::CommitDecision::kCommitted);

  ASSERT_TRUE(store.setActiveGoalEpoch(8, true));
  EXPECT_FALSE(store.rebindRetainedBundle(8, 20, 6));
  EXPECT_TRUE(store.rebindRetainedBundle(8, 20, 7));

  const auto rebound = store.load();
  ASSERT_TRUE(rebound);
  EXPECT_EQ(rebound->goal_epoch, 8U);
  EXPECT_EQ(rebound->request_id, 20U);
  navigation_execution::CommandSampler sampler(store);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50, 8)));
  EXPECT_FALSE(static_cast<bool>(sampler.sample(50, 7)));
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
  EXPECT_EQ(evaluations, 0U);

  const auto active = sampler.sample(100);
  ASSERT_TRUE(static_cast<bool>(active));
  EXPECT_FALSE(active.awaiting_activation);
  EXPECT_EQ(evaluations, 1U);
  EXPECT_DOUBLE_EQ(active.point->position_world.x(), 100.0);

  const auto expired = sampler.sample(201);
  EXPECT_FALSE(static_cast<bool>(expired));
  ASSERT_TRUE(expired.bundle);
  EXPECT_FALSE(expired.awaiting_activation);
  EXPECT_EQ(evaluations, 1U);
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

TEST(CommittedBundleStore, ExposureRejectsRetainedBundleAfterGoalChange) {
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
  EXPECT_FALSE(store.publishIfCurrent(sampled, 7, [&] { exposed = true; }));
  EXPECT_FALSE(exposed);
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
