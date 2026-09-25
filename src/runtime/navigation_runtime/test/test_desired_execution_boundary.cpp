#include <cstdint>
#include <memory>

#include <gtest/gtest.h>

#include <navigation_execution/command_sampler.hpp>
#include <navigation_execution/execution_authority.hpp>
#include <navigation_runtime/desired_planning_intent.hpp>

namespace navigation_runtime {
namespace {

using Goal = navigation_contracts::msg::NavigationGoal;
using navigation_execution::CommitDecision;
using navigation_execution::CommitToken;
using navigation_execution::ExecutionAuthority;
using navigation_execution::StageDecision;
using navigation_planning::CandidateBundle;

Goal makeGoal(const char* mission_id, std::uint64_t request_id) {
  Goal goal;
  goal.mission_id = mission_id;
  goal.request_id = request_id;
  return goal;
}

CandidateBundle candidateFor(
    const std::uint64_t goal_epoch, const std::uint64_t request_id,
    const navigation_world_model::WorldSnapshotIdentity& world) {
  CandidateBundle candidate;
  candidate.pinned_world_identity = world;
  candidate.world_identity = world;
  candidate.localization_epoch = world.localization_epoch;
  candidate.goal_epoch = goal_epoch;
  candidate.request_id = request_id;
  candidate.bundle_generation = goal_epoch + 20U;
  candidate.valid_from_ns = 1;
  candidate.valid_until_ns = 100;
  candidate.activation_stamp_ns = candidate.valid_from_ns;
  candidate.start_wall_time_s = 1.0e-9;
  candidate.duration_s = 399.0e-9;
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
  candidate.evaluator = [](
      const std::int64_t stamp,
      navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = static_cast<double>(stamp);
    point.trajectory_time_s = static_cast<double>(stamp - 1) * 1.0e-9;
    return true;
  };
  return candidate;
}

std::shared_ptr<const Goal> immutableGoal(const Goal& goal) {
  return std::make_shared<const Goal>(goal);
}

TEST(DesiredExecutionBoundary,
     DesiredSuccessorCoexistsWithPredecessorUntilAtomicCutover) {
  ExecutionAuthority authority;
  const navigation_world_model::WorldSnapshotIdentity world{3U, 4U, 1U, 1};
  const auto empty = authority.snapshot();
  ASSERT_EQ(authority.publishWorldIdentityIfCurrent(
                world, empty.version, empty.active, false),
            navigation_world_model::WorldCommitDecision::kCommitted);

  DesiredPlanningIntent desired;
  const auto predecessor_goal = makeGoal("mission", 17U);
  const auto successor_goal = makeGoal("mission", 18U);
  ASSERT_EQ(desired.advanceRevision(), 1U);
  desired.install(predecessor_goal, PlanningIntentTransition::kNewIntent);
  ASSERT_TRUE(desired.matches(predecessor_goal, 1U));

  ASSERT_TRUE(authority.beginGoal(world.localization_epoch, desired.revision(), false));
  auto predecessor = std::make_shared<const CandidateBundle>(
      candidateFor(1U, predecessor_goal.request_id, world));
  ASSERT_TRUE(predecessor->valid());
  ASSERT_EQ(authority.tryCommit(
                CommitToken{world, 1U, 1U}, immutableGoal(predecessor_goal), predecessor),
            CommitDecision::kCommitted);

  navigation_execution::CommandSampler sampler(authority);
  const auto before_retarget = authority.snapshot();
  ASSERT_EQ(before_retarget.active, predecessor);
  ASSERT_TRUE(static_cast<bool>(sampler.sample(50, 1U)));

  ASSERT_EQ(desired.advanceRevision(), 2U);
  desired.install(successor_goal, PlanningIntentTransition::kHotRetarget);
  ASSERT_TRUE(desired.matches(successor_goal, 2U));
  ASSERT_TRUE(desired.isHotRetarget());
  ASSERT_TRUE(authority.beginGoal(world.localization_epoch, desired.revision(), true));

  const auto retained = authority.snapshot();
  ASSERT_EQ(retained.active, predecessor);
  EXPECT_EQ(retained.admission_goal_epoch, 2U);
  EXPECT_EQ(retained.activeGoalEpoch(), 1U);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50, 1U)));
  EXPECT_EQ(sampler.sample(50, 2U).status,
            navigation_execution::SampleStatus::kGoalMismatch);

  // A result captured for desired revision 1 is rejected at the owner seam.
  // Rejection is observationally inert: it cannot replace or fail-close the
  // exact predecessor still authorized for publication.
  const auto before_stale_result = authority.snapshot();
  auto stale_candidate = std::make_shared<const CandidateBundle>(
      candidateFor(1U, predecessor_goal.request_id, world));
  EXPECT_EQ(authority.tryCommit(
                CommitToken{world, 1U, 2U}, immutableGoal(predecessor_goal), stale_candidate),
            CommitDecision::kGoalAdvanced);
  const auto after_stale_result = authority.snapshot();
  EXPECT_EQ(after_stale_result.version, before_stale_result.version);
  EXPECT_EQ(after_stale_result.active, predecessor);
  EXPECT_FALSE(after_stale_result.pending);
  EXPECT_EQ(after_stale_result.lifecycle, before_stale_result.lifecycle);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(50, 1U)));

  const auto anchor = authority.reserveAnchor(50, 60);
  ASSERT_TRUE(anchor);
  auto successor_value = candidateFor(2U, successor_goal.request_id, world);
  successor_value.world_identity = anchor->command_world;
  successor_value.pinned_world_identity = anchor->command_world;
  successor_value.valid_from_ns = anchor->activation_stamp_ns;
  successor_value.activation_stamp_ns = anchor->activation_stamp_ns;
  auto successor = std::make_shared<const CandidateBundle>(
      std::move(successor_value));
  ASSERT_TRUE(successor->valid());
  ASSERT_EQ(navigation_execution::candidateMatchesAnchor(*successor, *anchor),
            navigation_execution::AnchorMatchResult::kMatch);

  ASSERT_EQ(authority.stagePending(
                CommitToken{world, 2U, 2U}, *anchor,
                immutableGoal(successor_goal), successor),
            StageDecision::kStaged);
  EXPECT_EQ(authority.load(), predecessor);
  EXPECT_EQ(authority.snapshot().pending, successor);
  EXPECT_TRUE(static_cast<bool>(sampler.sample(55, 1U)));
  EXPECT_EQ(sampler.sample(55, 2U).status,
            navigation_execution::SampleStatus::kGoalMismatch);

  const auto activation = authority.snapshot();
  ASSERT_TRUE(authority.activatePendingIfDueAndFinalize(
      anchor->activation_stamp_ns, activation,
      [](const std::uint64_t) { return true; }));
  EXPECT_EQ(authority.load(), successor);
  EXPECT_EQ(sampler.sample(anchor->activation_stamp_ns, 1U).status,
            navigation_execution::SampleStatus::kGoalMismatch);
  EXPECT_TRUE(static_cast<bool>(
      sampler.sample(anchor->activation_stamp_ns, 2U)));
}

}  // namespace
}  // namespace navigation_runtime
