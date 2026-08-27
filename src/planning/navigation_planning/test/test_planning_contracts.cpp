#include <chrono>

#include <gtest/gtest.h>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_planning/kinematic_state.hpp>
#include <navigation_planning/planning_outcome.hpp>
#include <navigation_planning/planning_request.hpp>
#include <navigation_planning/planning_limits.hpp>
#include <navigation_planning/planner_diagnostics.hpp>

namespace {

navigation_planning::CandidateBundle validCandidate() {
  navigation_planning::CandidateBundle candidate;
  candidate.world_identity.localization_epoch = 4;
  candidate.world_identity.generation = 2;
  candidate.world_identity.revision = 8;
  candidate.world_identity.observation_stamp_ns = 100;
  candidate.localization_epoch = 4;
  candidate.goal_epoch = 7;
  candidate.request_id = 9;
  candidate.bundle_generation = 11;
  candidate.valid_from_ns = 100;
  candidate.valid_until_ns = 200;
  candidate.evaluator = [](std::int64_t, navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = 1.0;
    return true;
  };
  return candidate;
}

TEST(PlanningCandidate, RejectsInvalidProvenanceAndNonFiniteSamples) {
  auto candidate = validCandidate();
  ASSERT_TRUE(candidate.valid());
  EXPECT_TRUE(candidate.sample(150).has_value());
  EXPECT_FALSE(candidate.sample(99).has_value());

  candidate.world_identity.localization_epoch = 3;
  EXPECT_FALSE(candidate.valid());
}

TEST(PlanningOutcome, SuccessRequiresCandidateAndFailureDoesNotCarryOne) {
  navigation_planning::PlanningOutcome success;
  success.status = navigation_planning::PlanningStatus::kSuccess;
  success.candidate = validCandidate();
  EXPECT_TRUE(success.valid());

  navigation_planning::PlanningOutcome failure;
  failure.status = navigation_planning::PlanningStatus::kDeadline;
  EXPECT_TRUE(failure.valid());
  failure.candidate = validCandidate();
  EXPECT_FALSE(failure.valid());
}

TEST(PlanningBudget, UsesSteadyClockAndCancellation) {
  navigation_planning::PlanningBudget budget;
  budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
                    std::chrono::seconds(1);
  EXPECT_FALSE(budget.exhausted());
  std::stop_source source;
  budget.cancellation = source.get_token();
  source.request_stop();
  EXPECT_TRUE(budget.cancelled());
  EXPECT_TRUE(budget.exhausted());
}

TEST(KinematicState, RequiresTimeFrameAndYawContract) {
  navigation_planning::KinematicState state;
  state.source_stamp_ns = 100;
  state.receive_stamp_ns = 120;
  state.localization_epoch = 3;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  state.yaw_rad = 0.25;
  EXPECT_TRUE(state.finite());

  state.receive_stamp_ns = 0;
  EXPECT_FALSE(state.finite());
}

TEST(KinematicState, QuaternionFiniteCheckIsStableForLargeFiniteCoefficients) {
  navigation_planning::KinematicState state;
  state.source_stamp_ns = 100;
  state.receive_stamp_ns = 120;
  state.localization_epoch = 3;
  state.world_frame_id = "lio_odom";
  state.body_frame_id = "base_link";
  state.yaw_rad = 0.25;

  state.orientation_world_body = Eigen::Quaterniond(1.0e200, 1.0e200,
                                                      -1.0e200, 1.0e200);
  EXPECT_TRUE(state.finite());

  state.orientation_world_body = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  EXPECT_FALSE(state.finite());
}

TEST(DynamicLimits, HasOneProductOwnedValidationContract) {
  const navigation_planning::DynamicLimits valid{7.0, 5.0, 12.0};
  EXPECT_TRUE(valid.valid());

  const navigation_planning::DynamicLimits invalid{7.0, 0.0, 12.0};
  EXPECT_FALSE(invalid.valid());
}

TEST(TrajectorySnapshot, PreservesRoleAndFinishedStateAtProductBoundary) {
  navigation_planning::TrajectorySnapshot snapshot;
  snapshot.start_wall_time_s = 10.0;
  snapshot.duration_s = 2.0;
  snapshot.evaluator = [](double time_s, navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = time_s;
    point.trajectory_time_s = time_s;
    return true;
  };
  snapshot.role_evaluator = [](double time_s) {
    return time_s >= 1.0 ? navigation_planning::CandidateRole::kBackup
                         : navigation_planning::CandidateRole::kMain;
  };

  navigation_planning::TrajectoryPoint main_point;
  ASSERT_TRUE(snapshot.sample(0.5, main_point));
  EXPECT_EQ(main_point.role, navigation_planning::CandidateRole::kMain);
  EXPECT_FALSE(main_point.finished);

  navigation_planning::TrajectoryPoint backup_point;
  ASSERT_TRUE(snapshot.sample(2.0, backup_point));
  EXPECT_EQ(backup_point.role, navigation_planning::CandidateRole::kBackup);
  EXPECT_TRUE(backup_point.finished);
}

}  // namespace
