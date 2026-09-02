#include <chrono>

#include <gtest/gtest.h>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_planning/candidate_admission.hpp>
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
  candidate.pinned_world_identity = candidate.world_identity;
  candidate.localization_epoch = 4;
  candidate.goal_epoch = 7;
  candidate.request_id = 9;
  candidate.bundle_generation = 11;
  candidate.valid_from_ns = 100;
  candidate.valid_until_ns = 200;
  candidate.start_wall_time_s = 1.0e-7;
  candidate.duration_s = 1.0e-7;
  candidate.backup_start_time_s = 0.0;
  candidate.kind = navigation_planning::CandidateBundleKind::kTerminalStop;
  candidate.certificates = {true, true, true, true};
  candidate.protected_region.minimum = Eigen::Vector3d::Zero();
  candidate.protected_region.maximum = Eigen::Vector3d::Ones();
  candidate.role_schedule = {{0.0, 1.0e-7,
                              navigation_planning::CandidateRole::kMain}};
  candidate.evaluator = [](std::int64_t stamp,
                           navigation_planning::TrajectoryPoint& point) {
    point.position_world.x() = 1.0;
    point.trajectory_time_s = static_cast<double>(stamp - 100) * 1.0e-9;
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

TEST(PlanningCandidate, RejectsEvaluatorRoleMutationAndUnknownRole) {
  auto candidate = validCandidate();
  candidate.evaluator = [](std::int64_t, navigation_planning::TrajectoryPoint& point) {
    point.role = navigation_planning::CandidateRole::kBackup;
    return true;
  };
  EXPECT_FALSE(candidate.sample(150).has_value());
  candidate.role = static_cast<navigation_planning::CandidateRole>(255U);
  EXPECT_FALSE(candidate.valid());
}

TEST(PlanningCandidate, AllowsDeclaredMainToBackupRoleSchedule) {
  auto candidate = validCandidate();
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.backup_start_time_s = 7.5e-8;
  candidate.role_schedule = {
      {0.0, 7.5e-8, navigation_planning::CandidateRole::kMain},
      {7.5e-8, 1.0e-7, navigation_planning::CandidateRole::kBackup}};
  candidate.evaluator = [](std::int64_t stamp,
                           navigation_planning::TrajectoryPoint& point) {
    point.role = stamp < 175
                     ? navigation_planning::CandidateRole::kMain
                     : navigation_planning::CandidateRole::kBackup;
    point.trajectory_time_s = static_cast<double>(stamp - 100) * 1.0e-9;
    return true;
  };

  const auto main = candidate.sample(150);
  ASSERT_TRUE(main.has_value());
  EXPECT_EQ(main->role, navigation_planning::CandidateRole::kMain);
  const auto backup = candidate.sample(175);
  ASSERT_TRUE(backup.has_value());
  EXPECT_EQ(backup->role, navigation_planning::CandidateRole::kBackup);

  candidate.backup_available = false;
  EXPECT_FALSE(candidate.sample(175).has_value());
}

TEST(PlanningCandidate, RequiresExplicitTerminalStopSemanticForMainWithBackup) {
  auto candidate = validCandidate();
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.backup_start_time_s = 7.5e-8;
  candidate.certificates.terminal_stop = true;
  candidate.terminal_stop = true;
  candidate.role_schedule = {
      {0.0, 7.5e-8, navigation_planning::CandidateRole::kMain},
      {7.5e-8, 1.0e-7, navigation_planning::CandidateRole::kBackup}};
  candidate.evaluator = [](std::int64_t stamp,
                           navigation_planning::TrajectoryPoint& point) {
    point.role = stamp < 175
                     ? navigation_planning::CandidateRole::kMain
                     : navigation_planning::CandidateRole::kBackup;
    point.trajectory_time_s = static_cast<double>(stamp - 100) * 1.0e-9;
    return true;
  };
  EXPECT_TRUE(candidate.valid());
  EXPECT_TRUE(navigation_planning::certifiedTerminalStopAtEndpoint(candidate));

  candidate.certificates.terminal_stop = false;
  EXPECT_FALSE(candidate.valid());
}

TEST(PlanningCandidate, AdmissionRequiresEightHundredMillisecondsOfMain) {
  auto candidate = validCandidate();
  candidate.start_wall_time_s = 10.0;
  candidate.duration_s = 2.0;
  candidate.backup_start_time_s = 1.0;
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.certificates.terminal_stop = false;
  candidate.role_schedule = {
      {0.0, 1.0, navigation_planning::CandidateRole::kMain},
      {1.0, 2.0, navigation_planning::CandidateRole::kBackup}};
  candidate.evaluator = [](std::int64_t stamp,
                           navigation_planning::TrajectoryPoint& point) {
    point.trajectory_time_s = static_cast<double>(stamp) * 1.0e-9 - 10.0;
    point.role = point.trajectory_time_s < 1.0
        ? navigation_planning::CandidateRole::kMain
        : navigation_planning::CandidateRole::kBackup;
    return true;
  };
  EXPECT_TRUE(navigation_planning::candidateHasRequiredMainReserve(
      candidate, 10.20));
  EXPECT_FALSE(navigation_planning::candidateHasRequiredMainReserve(
      candidate, 10.200000002));
}

TEST(PlanningCandidate, CertifiedTerminalStopIsReserveExempt) {
  auto candidate = validCandidate();
  ASSERT_TRUE(navigation_planning::certifiedTerminalStopAtEndpoint(candidate));
  EXPECT_TRUE(navigation_planning::candidateHasRequiredMainReserve(
      candidate, 100.0));
  candidate.evaluator = [](std::int64_t,
                           navigation_planning::TrajectoryPoint& point) {
    point.velocity_world.x() = 0.01;
    point.trajectory_time_s = 1.0e-7;
    return true;
  };
  EXPECT_FALSE(navigation_planning::candidateHasRequiredMainReserve(
      candidate, 100.0));
}

TEST(PlanningCandidate, FinalRoleWinsWhenProducerOffsetRoundsOneNanosecondShort) {
  auto candidate = validCandidate();
  candidate.duration_s = 0.000019999400017999461;
  candidate.backup_available = true;
  candidate.kind = navigation_planning::CandidateBundleKind::kMainWithBackup;
  candidate.certificates.terminal_stop = false;
  // This reproduces a producer role endpoint whose independently rounded
  // offset is 19,999 ns while the declared duration rounds to 20,000 ns.
  candidate.role_schedule = {
      {0.0, 0.000009, navigation_planning::CandidateRole::kMain},
      {0.000009, 0.0000199994, navigation_planning::CandidateRole::kBackup}};
  const auto role = candidate.scheduledRole(candidate.duration_s);
  ASSERT_TRUE(role.has_value());
  EXPECT_EQ(*role, navigation_planning::CandidateRole::kBackup);
}

TEST(PlanningOutcome, SuccessRequiresCandidateAndFailureDoesNotCarryOne) {
  navigation_planning::PlanningOutcome success;
  success.outcome = navigation_planning::CompletePlanningOutcome::kBaselineCompleteBundle;
  success.failure_stage = navigation_planning::PlanningFailureStage::kNone;
  success.failure_reason = navigation_planning::PlanningFailureReason::kNone;
  success.candidate = validCandidate();
  EXPECT_TRUE(success.valid());

  navigation_planning::PlanningOutcome failure;
  failure.outcome = navigation_planning::CompletePlanningOutcome::kNoCompleteBundle;
  failure.failure_stage = navigation_planning::PlanningFailureStage::kDeadline;
  failure.failure_reason =
      navigation_planning::PlanningFailureReason::kNoCompleteBundleAtDeadline;
  EXPECT_TRUE(failure.valid());
  failure.candidate = validCandidate();
  EXPECT_FALSE(failure.valid());

  navigation_planning::PlanningOutcome retained;
  retained.outcome =
      navigation_planning::CompletePlanningOutcome::kRetainedCommittedBundle;
  retained.failure_stage = navigation_planning::PlanningFailureStage::kNone;
  retained.failure_reason = navigation_planning::PlanningFailureReason::kNone;
  EXPECT_TRUE(retained.valid());
}

TEST(RouteBoundary, RequiresExplicitVolumeEventAndUnitTangents) {
  navigation_planning::RouteBoundaryConstraint constraint;
  constraint.admissible_volume.minimum = Eigen::Vector3d(-1.0, -1.0, -1.0);
  constraint.admissible_volume.maximum = Eigen::Vector3d(1.0, 1.0, 1.0);
  constraint.incoming_tangent = Eigen::Vector3d::UnitX();
  constraint.outgoing_tangent = Eigen::Vector3d::UnitY();
  constraint.corner_speed_mps = 2.0;
  EXPECT_TRUE(constraint.valid());
  EXPECT_TRUE(constraint.contains(Eigen::Vector3d::Zero()));
  EXPECT_FALSE(constraint.contains(Eigen::Vector3d(2.0, 0.0, 0.0)));

  navigation_planning::RouteBoundaryEvent event;
  event.boundary_stamp_ns = 100;
  event.position_world = Eigen::Vector3d::Zero();
  event.incoming_tangent = Eigen::Vector3d::UnitX();
  event.outgoing_tangent = Eigen::Vector3d::UnitY();
  event.corner_speed_mps = 2.0;
  EXPECT_TRUE(event.valid());
  event.outgoing_tangent = Eigen::Vector3d(2.0, 0.0, 0.0);
  EXPECT_FALSE(event.valid());
}

TEST(PlanningRequest, KeyPinsEveryMutableIdentityAndStartMode) {
  navigation_planning::PlanningKey key;
  key.localization_epoch = 3;
  key.goal_epoch = 4;
  key.request_id = 5;
  key.route_revision = 6;
  key.committed_bundle_generation = 7;
  key.pinned_world_generation = 8;
  key.pinned_world_revision = 9;
  key.start_mode = navigation_planning::PlanningStartMode::kCommittedFutureState;
  key.anchor_stamp_ns = 10;
  key.dynamics_hash = 11;
  EXPECT_TRUE(key.valid());
  key.anchor_stamp_ns = 0;
  EXPECT_FALSE(key.valid());
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
  EXPECT_FALSE(state.finite());

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
