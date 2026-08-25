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
  candidate.localization_epoch = 3;
  candidate.goal_epoch = goal_epoch;
  candidate.request_id = goal_epoch + 10;
  candidate.bundle_generation = goal_epoch + 20;
  candidate.valid_from_ns = 1;
  candidate.valid_until_ns = 100;
  candidate.evaluator = [](std::int64_t stamp, navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = static_cast<double>(stamp);
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
  ASSERT_TRUE(store.load());
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

}  // namespace
