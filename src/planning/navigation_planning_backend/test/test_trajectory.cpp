#include <cmath>
#include <atomic>
#include <chrono>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "data_structure/base/trajectory.h"
#include <navigation_world_model/goal_contract.hpp>
#include "planner_core/planner.hpp"
#include "planner_core/absolute_deadline.hpp"
#include "planner_core/command_time.hpp"
#include "planner_core/ciri.h"
#include "planner_core/guide_endpoint.hpp"
#include "planner_core/replan_contract.hpp"
#include "planner_core/planning_stage.hpp"
#include "planner_core/trajectory_world_validator.hpp"
#include "traj_opt/trajectory_dynamics.hpp"
#include "traj_opt/yaw_traj_opt.h"
#include "data_structure/base/polytope.h"
#include "utils/geometry/geometry_utils.h"

namespace navigation_planning_backend {

struct CiriGeometryTestAccess {
  static bool findTangentPlaneOfSphere(
      const Eigen::Vector3d& center, double radius,
      const Eigen::Vector3d& pass_point, const Eigen::Vector3d& seed_point,
      Eigen::Vector4d& plane) {
    return CIRI::findTangentPlaneOfSphere(
        center, radius, pass_point, seed_point, plane);
  }
};

}  // namespace navigation_planning_backend

namespace {

class SweepWorld : public navigation_world_model::WorldModelView {
 public:
  double blocked_from_x{std::numeric_limits<double>::infinity()};
  bool endpoints_in_bounds{true};
  navigation_world_model::WorldGeometry geometry() const noexcept override {
    navigation_world_model::WorldGeometry value;
    value.inflated_resolution_m = 0.2;
    return value;
  }
  navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return {1, 1, 1, 1};
  }
  navigation_world_model::CellState classify(
      const navigation_world_model::Point3& p,
      navigation_world_model::GridLayer) const noexcept override {
    return p.x() >= blocked_from_x
        ? navigation_world_model::CellState::kOccupied
        : navigation_world_model::CellState::kUnknown;
  }
  bool contains(const navigation_world_model::Point3&) const noexcept override {
    return endpoints_in_bounds;
  }
  navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3& p,
      navigation_world_model::GridLayer) const noexcept override {
    return (p.array() / 0.2).floor().cast<int>();
  }
  navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3& index,
      navigation_world_model::GridLayer) const noexcept override {
    return (index.cast<double>().array() + 0.5).matrix() * 0.2;
  }
  std::optional<navigation_world_model::Point3> nearestNotOccupied(
      const navigation_world_model::Point3& p, navigation_world_model::GridLayer,
      double) const override { return p; }
  bool isSegmentTraversable(
      const navigation_world_model::Point3& a,
      const navigation_world_model::Point3& b,
      navigation_world_model::GridLayer,
      navigation_world_model::UnknownPolicy policy) const noexcept override {
    if (policy != navigation_world_model::UnknownPolicy::kAllowUnknown) return false;
    return std::max(a.x(), b.x()) < blocked_from_x;
  }
  navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& box) const noexcept override { return box; }
  navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox&) const override { return {}; }
};

class CurvedCellWorld final : public SweepWorld {
 public:
  navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer) const noexcept override {
    const navigation_world_model::Point3 occupied_cell_center(0.5, 0.3, 3.1);
    return (point - occupied_cell_center).norm() < 1.0e-3
        ? navigation_world_model::CellState::kOccupied
        : navigation_world_model::CellState::kUnknown;
  }
};

class DiagonalNeighborWorld final : public SweepWorld {
 public:
  explicit DiagonalNeighborWorld(
      const navigation_world_model::Point3& unknown_center)
      : unknown_center_(unknown_center) {}

  navigation_world_model::CellState classify(
      const navigation_world_model::Point3& point,
      navigation_world_model::GridLayer) const noexcept override {
    return (point - unknown_center_).norm() < 1.0e-9
        ? navigation_world_model::CellState::kUnknown
        : navigation_world_model::CellState::kKnownFree;
  }

 private:
  navigation_world_model::Point3 unknown_center_;
};

geometry_utils::Trajectory linearTrajectory(double duration, double start_wall_time) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 1.0;
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory result({duration}, {coefficients});
  result.start_WT = start_wall_time;
  return result;
}
}  // namespace

TEST(CiriGeometry, UsesNonCollinearSeedDirectionWithoutArtificialPerturbation) {
  Eigen::Vector4d plane;
  ASSERT_TRUE(navigation_planning_backend::CiriGeometryTestAccess::
                  findTangentPlaneOfSphere(
                      Eigen::Vector3d::Zero(), 1.0,
                      Eigen::Vector3d(2.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.005, 0.0), plane));

  // The positive-y seed selects the first tangent branch. A fixed
  // pass_point - pass_point test would always perturb the seed by 0.01 m,
  // crossing this deliberately narrow branch boundary and flipping plane.y.
  EXPECT_NEAR(plane.x(), 0.5, 1e-12);
  EXPECT_NEAR(plane.y(), std::sqrt(3.0) / 2.0, 1e-12);
  EXPECT_NEAR(plane.z(), 0.0, 1e-12);
  EXPECT_NEAR(plane.w(), -1.0, 1e-12);
}

TEST(CiriGeometry, RejectsInvalidConfigurationBeforeNumericalWork) {
  navigation_planning_backend::CIRI ciri;
  EXPECT_THROW(ciri.setupParams(0.0, 1), std::invalid_argument);
  EXPECT_THROW(ciri.setupParams(0.5, 0), std::invalid_argument);
  EXPECT_THROW(ciri.setupParams(std::numeric_limits<double>::quiet_NaN(), 1),
               std::invalid_argument);
}

TEST(CiriGeometry, RejectsDegenerateSeedSegmentBeforeEllipsoidConstruction) {
  navigation_planning_backend::CIRI ciri;
  ciri.setupParams(0.35, 1);

  Eigen::MatrixX4d bounds(6, 4);
  bounds <<
      1.0, 0.0, 0.0, -2.0,
     -1.0, 0.0, 0.0, -2.0,
      0.0, 1.0, 0.0, -2.0,
      0.0, -1.0, 0.0, -2.0,
      0.0, 0.0, 1.0, -2.0,
      0.0, 0.0, -1.0, -2.0;
  Eigen::Matrix3Xd obstacles(3, 1);
  obstacles.col(0) = Eigen::Vector3d(1.5, 1.5, 1.5);

  EXPECT_EQ(ciri.comvexDecomposition(
                bounds, obstacles, Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero()),
            navigation_math::INIT_ERROR);
}

TEST(PlannerTrajectory, PartialSlicePreservesPieceLocalTimeAndContinuity) {
  std::vector<double> durations{1.0, 1.0};
  std::vector<Eigen::MatrixXd> coefficients;
  for (double offset : {0.0, 1.0}) {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(3, 6);
    matrix(0, 4) = 1.0;  // x(t) = t + offset
    matrix(0, 5) = offset;
    matrix(2, 5) = 3.0;
    coefficients.push_back(matrix);
  }

  geometry_utils::Trajectory source(durations, coefficients);
  geometry_utils::Trajectory partial;
  ASSERT_TRUE(source.getPartialTrajectoryByTime(0.25, 1.5, partial));
  ASSERT_EQ(partial.getPieceNum(), 2);
  EXPECT_NEAR(partial.getTotalDuration(), 1.25, 1e-12);
  EXPECT_NEAR(partial.getPos(0.0).x(), 0.25, 1e-12);
  EXPECT_NEAR(partial.getPos(0.75).x(), 1.0, 1e-12);
  EXPECT_NEAR(partial.getPos(partial.getTotalDuration()).x(), 1.5, 1e-12);
  EXPECT_NEAR(partial.getVel(0.0).x(), 1.0, 1e-12);
  EXPECT_NEAR(partial.getVel(partial.getTotalDuration()).x(), 1.0, 1e-12);
  EXPECT_TRUE(partial.getPos(0.0).allFinite());
}

TEST(PlannerTrajectory, PartialSliceUsesHalfOpenStartAtPieceBoundary) {
  std::vector<double> durations{1.0, 1.0};
  std::vector<Eigen::MatrixXd> coefficients;
  for (double offset : {0.0, 1.0}) {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(3, 6);
    matrix(0, 4) = 1.0;
    matrix(0, 5) = offset;
    coefficients.push_back(matrix);
  }

  geometry_utils::Trajectory source(durations, coefficients);
  geometry_utils::Trajectory partial;
  ASSERT_TRUE(source.getPartialTrajectoryByTime(1.0, 1.5, partial));
  ASSERT_EQ(partial.getPieceNum(), 1);
  EXPECT_DOUBLE_EQ(partial[0].getDuration(), 0.5);
  EXPECT_DOUBLE_EQ(partial.start_WT, source.start_WT + 1.0);
  EXPECT_NEAR(partial.getPos(0.0).x(), 1.0, 1.0e-12);
  EXPECT_NEAR(partial.getPos(0.5).x(), 1.5, 1.0e-12);
  EXPECT_TRUE(partial.getState(0.0).allFinite());
}

TEST(PlannerTrajectory, PartialSliceEndingAtPieceBoundaryKeepsLocalDuration) {
  std::vector<double> durations{1.0, 1.0};
  std::vector<Eigen::MatrixXd> coefficients;
  for (double offset : {0.0, 1.0}) {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(3, 6);
    matrix(0, 4) = 1.0;
    matrix(0, 5) = offset;
    coefficients.push_back(matrix);
  }

  geometry_utils::Trajectory source(durations, coefficients);
  geometry_utils::Trajectory partial;
  ASSERT_TRUE(source.getPartialTrajectoryByTime(0.0, 1.0, partial));
  ASSERT_EQ(partial.getPieceNum(), 1);
  EXPECT_DOUBLE_EQ(partial[0].getDuration(), 1.0);
  EXPECT_NEAR(partial.getPos(partial.getTotalDuration()).x(), 1.0, 1.0e-12);
  EXPECT_TRUE(partial.getState(partial.getTotalDuration()).allFinite());
}

TEST(PlannerTrajectory, SweepUsesNextPieceAtExactBoundary) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 1.0;
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({0.5, 0.5}, {coefficients, coefficients});
  geometry_utils::Trajectory yaw({0.5, 0.5}, {coefficients, coefficients});
  position.start_WT = yaw.start_WT = 10.0;

  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = position;
  candidate.yaw = yaw;
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  const auto location = navigation_planning_backend::locatePieceForSweep(
      candidate.position, 0.5);
  ASSERT_TRUE(location);
  EXPECT_EQ(location->index, 1);
  EXPECT_DOUBLE_EQ(location->local_time, 0.0);
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      SweepWorld{}, candidate, 10.5,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, SweepAdvancesAcrossNumericalPieceBoundary) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 1.0;
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({0.2, 0.8}, {coefficients, coefficients});
  geometry_utils::Trajectory yaw({0.2, 0.8}, {coefficients, coefficients});
  position.start_WT = yaw.start_WT = 10.0;

  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = position;
  candidate.yaw = yaw;
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  const double just_before_boundary = 0.2 - 1.0e-15;
  const auto location = navigation_planning_backend::locatePieceForSweep(
      candidate.position, just_before_boundary);
  ASSERT_TRUE(location);
  EXPECT_EQ(location->index, 1);
  EXPECT_DOUBLE_EQ(location->local_time, 0.0);
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      SweepWorld{}, candidate, 10.2 - 1.0e-15,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, CommandTrajectoryTimePreservesEstablishedSamplingSemantics) {
  const auto before = navigation_planning_backend::commandTrajectoryTime(9.75, 10.0, 2.0);
  EXPECT_FALSE(before.finished);
  EXPECT_DOUBLE_EQ(before.trajectory_time_s, -0.25);

  const auto start = navigation_planning_backend::commandTrajectoryTime(10.0, 10.0, 2.0);
  EXPECT_FALSE(start.finished);
  EXPECT_DOUBLE_EQ(start.trajectory_time_s, 0.0);

  const auto middle = navigation_planning_backend::commandTrajectoryTime(11.25, 10.0, 2.0);
  EXPECT_FALSE(middle.finished);
  EXPECT_DOUBLE_EQ(middle.trajectory_time_s, 1.25);

  const auto finished = navigation_planning_backend::commandTrajectoryTime(12.1, 10.0, 2.0);
  EXPECT_TRUE(finished.finished);
  EXPECT_DOUBLE_EQ(finished.trajectory_time_s, 2.0);
}

TEST(PlannerTrajectory, OnlySuccessfulExpResultMayBuildAndCommitNewCandidate) {
  EXPECT_TRUE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::SUCCESS));
  EXPECT_FALSE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::NO_NEED));
  EXPECT_FALSE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::FAILED));
  EXPECT_FALSE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::NEW_TRAJ));
  EXPECT_FALSE(navigation_planning_backend::expResultMayBuildCommandCandidate(navigation_math::EMER));

  EXPECT_TRUE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::SUCCESS));
  EXPECT_TRUE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::NO_NEED));
  EXPECT_TRUE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::FINISH));
  EXPECT_FALSE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::FAILED));
  EXPECT_FALSE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::OPT_FAILED));
  EXPECT_FALSE(navigation_planning_backend::backupResultMayBuildCommandCandidate(navigation_math::EMER));
}

TEST(PlannerTrajectory, VisibleReplacementDoesNotEraseFutureBackupSuffix) {
  EXPECT_TRUE(navigation_planning_backend::shouldRetainBackupCapableCommand(
      false, true));
  EXPECT_FALSE(navigation_planning_backend::shouldRetainBackupCapableCommand(
      true, true));
  EXPECT_FALSE(navigation_planning_backend::shouldRetainBackupCapableCommand(
      false, false));
}

TEST(PlannerTrajectory, FlatnessGateRejectsExcessBodyRateAndThrust) {
  traj_opt::Config config;
  config.mass = 1.0;
  config.grav = 9.81;
  config.dh = 0.0;
  config.dv = 0.0;
  config.cp = 0.0;
  config.v_eps = 1.0e-4;
  config.max_omg = 2.0;
  config.min_acc_thr = 6.0;
  config.max_acc_thr = 15.0;
  config.quadrotot_flatness.reset(config.mass, config.grav, config.dh, config.dv,
                                  config.cp, config.v_eps);

  Eigen::MatrixXd hover_coefficients = Eigen::MatrixXd::Zero(3, 6);
  hover_coefficients(2, 5) = 3.0;
  geometry_utils::Trajectory hover({1.0}, {hover_coefficients});
  traj_opt::TrajectoryDynamicReport hover_report;
  EXPECT_TRUE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      hover, config, &hover_report));
  EXPECT_NEAR(hover_report.minimum_thrust_n, 9.81, 1.0e-9);

  // x(t)=10*t^3 creates 60 m/s^3 jerk at t=0, which maps to a body rate far
  // above the configured envelope even though thrust remains finite.
  Eigen::MatrixXd aggressive_coefficients = Eigen::MatrixXd::Zero(3, 6);
  aggressive_coefficients(0, 2) = 10.0;
  aggressive_coefficients(2, 5) = 3.0;
  geometry_utils::Trajectory aggressive({1.0}, {aggressive_coefficients});
  traj_opt::TrajectoryDynamicReport aggressive_report;
  EXPECT_FALSE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      aggressive, config, &aggressive_report));
  EXPECT_GT(aggressive_report.maximum_body_rate_rad_s, config.max_omg);

  Eigen::MatrixXd thrust_coefficients = Eigen::MatrixXd::Zero(3, 6);
  thrust_coefficients(2, 3) = 10.0;  // z(t)=10*t^2, constant 20 m/s^2 up.
  geometry_utils::Trajectory excessive_thrust({1.0}, {thrust_coefficients});
  traj_opt::TrajectoryDynamicReport thrust_report;
  EXPECT_FALSE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      excessive_thrust, config, &thrust_report));
  EXPECT_GT(thrust_report.maximum_thrust_n, config.max_acc_thr);

  Eigen::MatrixXd yaw_coefficients = Eigen::MatrixXd::Zero(3, 6);
  yaw_coefficients(0, 4) = 3.0;  // Constant 3 rad/s yaw rate.
  geometry_utils::Trajectory fast_yaw({1.0}, {yaw_coefficients});
  traj_opt::TrajectoryDynamicReport yaw_report;
  EXPECT_FALSE(traj_opt::trajectorySatisfiesFlatnessEnvelope(
      hover, config, &yaw_report, 0.01, &fast_yaw));
  EXPECT_GT(yaw_report.maximum_body_rate_rad_s, config.max_omg);
}

TEST(PlannerTrajectory, EmergencyBundleIsAtomicallyOwnedByBackup) {
  Eigen::MatrixXd position_coefficients = Eigen::MatrixXd::Zero(3, 8);
  position_coefficients(0, 6) = 1.0;
  position_coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {position_coefficients});
  position.start_WT = 42.0;

  Eigen::MatrixXd yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  yaw_coefficients(0, 7) = 0.4;
  geometry_utils::Trajectory yaw({1.0}, {yaw_coefficients});
  yaw.start_WT = 42.0;

  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  EXPECT_FALSE(command.empty());
  EXPECT_TRUE(command.backupTrajAvilibale());
  EXPECT_DOUBLE_EQ(command.getBackupTrajStartTT(), 0.0);
  EXPECT_TRUE(command.isTTOnBackupTraj(0.01));
  EXPECT_NEAR(command.getPos(0.5).x(), 0.5, 1.0e-12);
  EXPECT_NEAR(command.getYaw(0.5).x(), 0.4, 1.0e-12);
}

TEST(PlannerTrajectory, RobotStateHasDeterministicFiniteDefaults) {
  const navigation_math::RobotState state;
  EXPECT_TRUE(state.p.isZero());
  EXPECT_TRUE(state.v.isZero());
  EXPECT_TRUE(state.a.isZero());
  EXPECT_TRUE(state.j.isZero());
  EXPECT_TRUE(state.q.coeffs().allFinite());
  EXPECT_DOUBLE_EQ(state.q.norm(), 1.0);
  EXPECT_FALSE(state.rcv);
}

TEST(PlannerTrajectory, FailedCandidateLeavesEmergencyBundleUnchanged) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 7.0;
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  const auto generation = command.generation();
  const auto diagnostics_before = command.snapshot().diagnostics;
  const auto before = command.getPos(0.5);
  geometry_utils::Trajectory invalid_yaw;
  EXPECT_FALSE(command.setEmergencyBackup(position, invalid_yaw));
  EXPECT_EQ(command.generation(), generation);
  EXPECT_TRUE(command.getPos(0.5).isApprox(before));
  const auto diagnostics_after = command.snapshot().diagnostics;
  EXPECT_EQ(diagnostics_after.generation, diagnostics_before.generation);
  EXPECT_TRUE(diagnostics_after.candidate_start_pvaj.isApprox(
      diagnostics_before.candidate_start_pvaj));
}

TEST(PlannerTrajectory, CommitDiagnosticsDescribeExactOldToNewSplice) {
  auto first_position = linearTrajectory(1.0, 10.0);
  auto first_yaw = linearTrajectory(1.0, 10.0);
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(first_position, first_yaw));
  auto first = command.snapshot();
  EXPECT_EQ(first.generation, 1U);
  EXPECT_EQ(first.diagnostics.generation, first.generation);
  EXPECT_EQ(first.diagnostics.previous_generation, 0U);
  EXPECT_FALSE(first.diagnostics.previous_valid);
  EXPECT_TRUE(first.diagnostics.candidate_start_pvaj.isApprox(
      first_position.getState(0.0)));

  auto second_position = linearTrajectory(1.0, 10.5);
  auto second_yaw = linearTrajectory(1.0, 10.5);
  ASSERT_TRUE(command.setEmergencyBackup(second_position, second_yaw));
  const auto second = command.snapshot();
  ASSERT_EQ(second.generation, 2U);
  EXPECT_EQ(second.diagnostics.generation, second.generation);
  EXPECT_EQ(second.diagnostics.previous_generation, first.generation);
  EXPECT_TRUE(second.diagnostics.previous_valid);
  EXPECT_DOUBLE_EQ(second.diagnostics.previous_sample_tt, 0.5);
  EXPECT_TRUE(second.diagnostics.previous_pvaj.isApprox(
      first_position.getState(0.5)));
  EXPECT_TRUE(second.diagnostics.position_residual.isApprox(
      second_position.getPos(0.0) - first_position.getPos(0.5)));
  EXPECT_TRUE(second.diagnostics.velocity_residual.isApprox(
      second_position.getVel(0.0) - first_position.getVel(0.5)));
  EXPECT_TRUE(second.diagnostics.acceleration_residual.isApprox(
      second_position.getAcc(0.0) - first_position.getAcc(0.5)));
  EXPECT_TRUE(second.diagnostics.jerk_residual.isApprox(
      second_position.getJer(0.0) - first_position.getJer(0.5)));
  EXPECT_NEAR(second.diagnostics.yaw_residual, -0.5, 1.0e-12);
}

TEST(PlannerTrajectory, CommitDiagnosticsClampPriorSampleAtFinishedEnd) {
  navigation_planning_backend::CmdTraj command;
  auto first_position = linearTrajectory(1.0, 10.0);
  auto first_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(first_position, first_yaw));

  auto after_position = linearTrajectory(1.0, 20.0);
  auto after_yaw = linearTrajectory(1.0, 20.0);
  ASSERT_TRUE(command.setEmergencyBackup(after_position, after_yaw));
  EXPECT_DOUBLE_EQ(command.snapshot().diagnostics.previous_sample_tt, 1.0);
}

TEST(PlannerTrajectory, RegressedCandidateStartTimeCannotReplaceCommittedBundle) {
  navigation_planning_backend::CmdTraj command;
  auto current_position = linearTrajectory(1.0, 10.0);
  auto current_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(current_position, current_yaw));
  const auto before = command.snapshot();
  const bool before_role_at_start = command.isTTOnBackupTraj(0.0);
  const bool before_role_at_end = command.isTTOnBackupTraj(1.0);

  auto historical_position = linearTrajectory(1.0, 9.5);
  auto historical_yaw = linearTrajectory(1.0, 9.5);
  auto historical = navigation_planning_backend::CmdTraj::buildEmergencyCandidate(
      historical_position, historical_yaw);
  ASSERT_TRUE(historical);
  EXPECT_FALSE(command.commitCandidate(std::move(*historical), {}));

  const auto after = command.snapshot();
  EXPECT_EQ(after.generation, before.generation);
  EXPECT_DOUBLE_EQ(after.position.start_WT, before.position.start_WT);
  EXPECT_DOUBLE_EQ(after.yaw.start_WT, before.yaw.start_WT);
  EXPECT_TRUE(after.position.getState(0.25).isApprox(
      before.position.getState(0.25)));
  EXPECT_TRUE(after.yaw.getState(0.25).isApprox(before.yaw.getState(0.25)));
  EXPECT_EQ(after.empty, before.empty);
  EXPECT_EQ(after.backup_available, before.backup_available);
  EXPECT_DOUBLE_EQ(after.backup_start_tt, before.backup_start_tt);
  EXPECT_EQ(command.isTTOnBackupTraj(0.0), before_role_at_start);
  EXPECT_EQ(command.isTTOnBackupTraj(1.0), before_role_at_end);
  EXPECT_EQ(after.certificate.pinned_world.generation,
            before.certificate.pinned_world.generation);
  EXPECT_EQ(after.certificate.pinned_world.revision,
            before.certificate.pinned_world.revision);
  EXPECT_EQ(after.certificate.pinned_world.observation_stamp_ns,
            before.certificate.pinned_world.observation_stamp_ns);
  EXPECT_EQ(after.certificate.validated_world.generation,
            before.certificate.validated_world.generation);
  EXPECT_EQ(after.certificate.validated_world.revision,
            before.certificate.validated_world.revision);
  EXPECT_EQ(after.certificate.validated_world.observation_stamp_ns,
            before.certificate.validated_world.observation_stamp_ns);
  EXPECT_DOUBLE_EQ(after.certificate.validation_begin_tt,
                   before.certificate.validation_begin_tt);
  EXPECT_EQ(after.diagnostics.generation, before.diagnostics.generation);
  EXPECT_EQ(after.diagnostics.previous_generation,
            before.diagnostics.previous_generation);
  EXPECT_DOUBLE_EQ(after.diagnostics.candidate_start_wall_time,
                   before.diagnostics.candidate_start_wall_time);
  EXPECT_TRUE(after.diagnostics.candidate_start_pvaj.isApprox(
      before.diagnostics.candidate_start_pvaj));
  EXPECT_EQ(after.diagnostics.previous_valid, before.diagnostics.previous_valid);
  EXPECT_DOUBLE_EQ(after.diagnostics.previous_sample_tt,
                   before.diagnostics.previous_sample_tt);
  EXPECT_TRUE(after.diagnostics.previous_pvaj.isApprox(
      before.diagnostics.previous_pvaj));
  EXPECT_TRUE(after.diagnostics.position_residual.isApprox(
      before.diagnostics.position_residual));
  EXPECT_TRUE(after.diagnostics.velocity_residual.isApprox(
      before.diagnostics.velocity_residual));
  EXPECT_TRUE(after.diagnostics.acceleration_residual.isApprox(
      before.diagnostics.acceleration_residual));
  EXPECT_TRUE(after.diagnostics.jerk_residual.isApprox(
      before.diagnostics.jerk_residual));
  EXPECT_DOUBLE_EQ(after.diagnostics.candidate_start_yaw,
                   before.diagnostics.candidate_start_yaw);
  EXPECT_DOUBLE_EQ(after.diagnostics.candidate_start_yaw_rate,
                   before.diagnostics.candidate_start_yaw_rate);
  EXPECT_DOUBLE_EQ(after.diagnostics.previous_yaw,
                   before.diagnostics.previous_yaw);
  EXPECT_DOUBLE_EQ(after.diagnostics.previous_yaw_rate,
                   before.diagnostics.previous_yaw_rate);
  EXPECT_DOUBLE_EQ(after.diagnostics.yaw_residual,
                   before.diagnostics.yaw_residual);
  EXPECT_DOUBLE_EQ(after.diagnostics.yaw_rate_residual,
                   before.diagnostics.yaw_rate_residual);
}

TEST(PlannerTrajectory, EqualOrNewerCandidateStartTimeCanCommit) {
  navigation_planning_backend::CmdTraj command;
  auto first_position = linearTrajectory(1.0, 10.0);
  auto first_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(first_position, first_yaw));

  auto equal_position = linearTrajectory(1.0, 10.0);
  auto equal_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(equal_position, equal_yaw));
  auto newer_position = linearTrajectory(1.0, 10.1);
  auto newer_yaw = linearTrajectory(1.0, 10.1);
  ASSERT_TRUE(command.setEmergencyBackup(newer_position, newer_yaw));
  EXPECT_EQ(command.snapshot().generation, 3U);
}

TEST(PlannerTrajectory, CommitDiagnosticsWrapYawResidualAcrossPiBoundary) {
  auto position = linearTrajectory(1.0, 10.0);
  Eigen::MatrixXd first_yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  first_yaw_coefficients(0, 7) = std::acos(-1.0) - 0.1;
  geometry_utils::Trajectory first_yaw({1.0}, {first_yaw_coefficients});
  first_yaw.start_WT = 10.0;
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, first_yaw));

  Eigen::MatrixXd second_yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  second_yaw_coefficients(0, 7) = -std::acos(-1.0) + 0.1;
  geometry_utils::Trajectory second_yaw({1.0}, {second_yaw_coefficients});
  second_yaw.start_WT = 10.0;
  ASSERT_TRUE(command.setEmergencyBackup(position, second_yaw));
  EXPECT_NEAR(command.snapshot().diagnostics.yaw_residual, 0.2, 1.0e-12);
}

TEST(PlannerTrajectory, InheritedBackupPrefixSurvivesMainOnlyCommit) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw, 0.0, 0.4);
  navigation_planning_backend::CmdTraj command;
  auto candidate = navigation_planning_backend::CmdTraj::buildCandidate(
      exp, nullptr, navigation_planning_backend::BackupDisposition::NO_NEED);
  ASSERT_TRUE(candidate);
  ASSERT_TRUE(command.commitCandidate(std::move(*candidate), {}));
  EXPECT_FALSE(command.backupTrajAvilibale());
  EXPECT_TRUE(command.isTTOnBackupTraj(0.0));
  EXPECT_TRUE(command.isTTOnBackupTraj(0.4));
  EXPECT_FALSE(command.isTTOnBackupTraj(0.5));
}

TEST(PlannerTrajectory, CandidateBuilderPreservesInheritedAndNewBackupRoles) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw, 0.1, 0.3);
  navigation_planning_backend::BackupTraj backup;
  auto backup_position = linearTrajectory(0.5, 10.6);
  auto backup_yaw = linearTrajectory(0.5, 10.6);
  backup.setTrajectory(10.6, 0.6, backup_position, backup_yaw);

  auto candidate = navigation_planning_backend::CmdTraj::buildCandidate(
      exp, &backup, navigation_planning_backend::BackupDisposition::SUCCESS);
  ASSERT_TRUE(candidate);
  EXPECT_NEAR(candidate->position.getTotalDuration(), 1.1, 1.0e-12);
  ASSERT_EQ(candidate->roles.size(), 4U);
  EXPECT_EQ(candidate->roles[0].role, navigation_planning_backend::CandidateTrajectoryRole::MAIN);
  EXPECT_DOUBLE_EQ(candidate->roles[1].begin_tt, 0.1);
  EXPECT_DOUBLE_EQ(candidate->roles[1].end_tt, 0.3);
  EXPECT_EQ(candidate->roles[1].role, navigation_planning_backend::CandidateTrajectoryRole::BACKUP);
  EXPECT_DOUBLE_EQ(candidate->roles[3].begin_tt, 0.6);
  EXPECT_DOUBLE_EQ(candidate->roles[3].end_tt, 1.1);
  EXPECT_EQ(candidate->roles[3].role, navigation_planning_backend::CandidateTrajectoryRole::BACKUP);
  EXPECT_TRUE(candidate->backup_suffix_available);
}

TEST(PlannerTrajectory, CandidateBuilderDoesNotAdvertiseZeroLengthBackupSuffix) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw);

  navigation_planning_backend::BackupTraj backup;
  auto empty_position = linearTrajectory(0.0, 11.0);
  auto empty_yaw = linearTrajectory(0.0, 11.0);
  backup.setTrajectory(11.0, 1.0, empty_position, empty_yaw);

  // A non-null backup object is not sufficient evidence.  The executable
  // bundle must either contain a positive-duration final BACKUP interval or
  // reject construction before it can reach authorization.
  EXPECT_FALSE(navigation_planning_backend::CmdTraj::buildCandidate(
      exp, &backup, navigation_planning_backend::BackupDisposition::SUCCESS));
}

TEST(PlannerTrajectory, InheritedBackupIntersectionNeverCrossesNewSuffix) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  auto backup_position = linearTrajectory(0.5, 10.6);
  auto backup_yaw = linearTrajectory(0.5, 10.6);
  navigation_planning_backend::BackupTraj backup;
  backup.setTrajectory(10.6, 0.6, backup_position, backup_yaw);

  for (const auto [begin, end] : std::vector<std::pair<double, double>>{
           {0.7, 0.9}, {0.5, 0.8}, {0.6, 0.6}, {0.2, 0.6}}) {
    navigation_planning_backend::ExpTraj exp;
    exp.setTrajectory(10.0, position, yaw, begin, end);
    auto candidate = navigation_planning_backend::CmdTraj::buildCandidate(
        exp, &backup, navigation_planning_backend::BackupDisposition::SUCCESS);
    ASSERT_TRUE(candidate);
    double previous_end = 0.0;
    for (const auto& interval : candidate->roles) {
      EXPECT_LE(interval.begin_tt, interval.end_tt);
      EXPECT_DOUBLE_EQ(interval.begin_tt, previous_end);
      previous_end = interval.end_tt;
    }
    EXPECT_DOUBLE_EQ(previous_end, candidate->position.getTotalDuration());
    EXPECT_TRUE(std::any_of(candidate->roles.begin(), candidate->roles.end(),
                            [](const auto& interval) {
                              return interval.role ==
                                         navigation_planning_backend::CandidateTrajectoryRole::BACKUP &&
                                     interval.begin_tt <= 0.6 && interval.end_tt >= 0.6;
                            }));
    if (begin > 0.6) {
      const auto first_backup = std::find_if(
          candidate->roles.begin(), candidate->roles.end(), [](const auto& interval) {
            return interval.role == navigation_planning_backend::CandidateTrajectoryRole::BACKUP;
          });
      ASSERT_NE(first_backup, candidate->roles.end());
      EXPECT_DOUBLE_EQ(first_backup->begin_tt, 0.6);
    }
  }
}

TEST(PlannerTrajectory, ExpOnlyDispositionsAndEmergencyPreserveProvenance) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  navigation_planning_backend::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw, 0.0, 0.2);
  for (const auto disposition : {navigation_planning_backend::BackupDisposition::FINISH,
                                 navigation_planning_backend::BackupDisposition::NO_NEED}) {
    auto candidate = navigation_planning_backend::CmdTraj::buildCandidate(
        exp, nullptr, disposition);
    ASSERT_TRUE(candidate);
    EXPECT_EQ(candidate->backup_disposition, disposition);
    EXPECT_FALSE(candidate->backup_suffix_available);
    ASSERT_EQ(candidate->roles.size(), 2U);
    EXPECT_EQ(candidate->roles.front().role,
              navigation_planning_backend::CandidateTrajectoryRole::BACKUP);
    EXPECT_EQ(candidate->roles.back().role,
              navigation_planning_backend::CandidateTrajectoryRole::MAIN);
  }

  auto emergency = navigation_planning_backend::CmdTraj::buildEmergencyCandidate(position, yaw);
  ASSERT_TRUE(emergency);
  EXPECT_EQ(emergency->backup_disposition,
            navigation_planning_backend::BackupDisposition::EMERGENCY);
  ASSERT_EQ(emergency->roles.size(), 1U);
  EXPECT_EQ(emergency->roles.front().role,
            navigation_planning_backend::CandidateTrajectoryRole::BACKUP);
}

TEST(PlannerTrajectory, CandidateRejectsZeroDurationPiece) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  geometry_utils::Trajectory malformed_position(
      {0.0, 1.0}, {position[0].getCoeffMat(), position[0].getCoeffMat()});
  geometry_utils::Trajectory malformed_yaw(
      {0.0, 1.0}, {yaw[0].getCoeffMat(), yaw[0].getCoeffMat()});

  EXPECT_FALSE(navigation_planning_backend::CmdTraj::buildEmergencyCandidate(
      malformed_position, malformed_yaw));
}

TEST(PlannerTrajectory, LatestWorldSweepAllowsUnknownAndRejectsFutureObstacle) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };
  SweepWorld world;
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0, navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kRequireKnownFree).valid);
  world.blocked_from_x = 0.7;
  const auto blocked = navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0, navigation_world_model::UnknownPolicy::kAllowUnknown);
  EXPECT_FALSE(blocked.valid);
  // The continuous tube certificate may reject at the first segment whose
  // conservative voxel tube reaches the obstacle boundary.
  EXPECT_GT(blocked.first_blocked_tt, 0.0);
}

TEST(PlannerTrajectory, ExpiredCandidateCannotBeValidatedAtItsTerminalPoint) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  SweepWorld world;
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.5,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
  // Validation must reject a candidate whose complete executable interval is
  // already in the past; clamping to t=duration must not turn it into a
  // terminal-point certificate.
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 11.01,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, ContinuousTubeRejectsCurvePassingThroughOccupiedCell) {
  constexpr double vertex_time = 0.5 / 0.98;
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 0.98;
  coefficients(1, 5) = -0.3 / (vertex_time * vertex_time);
  coefficients(1, 6) = 0.6 / vertex_time;
  coefficients(2, 7) = 3.0;

  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = geometry_utils::Trajectory({1.0}, {coefficients});
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  CurvedCellWorld world;
  // The curve crosses the occupied voxel around t=0.5102 while its sampled
  // points remain 0.1 m below the voxel center. The tube certificate must
  // inspect the cell covered by the bounded curve deviation.
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, ContinuousTubeIgnoresOccupiedCellOutsideCurveTube) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  // This occupied cell is inside the segment's axis-aligned bounding box but
  // outside the actual swept tube. A box-only certificate would falsely
  // reject the candidate.
  CurvedCellWorld world;
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, ContinuousTubeDoesNotClassifyDiagonalCellOutsideTube) {
  const navigation_world_model::Point3 start(0.1, 0.21, 0.21);
  const navigation_world_model::Point3 end(1.1, 0.21, 0.21);
  DiagonalNeighborWorld world({0.1, 0.1, 0.1});

  // The old center-distance-plus-half-diagonal approximation included this
  // diagonal neighbour even though its voxel box is separated from the
  // zero-radius centerline by sqrt(0.01^2 + 0.01^2).
  EXPECT_TRUE(navigation_planning_backend::certificateTubeIsSafe(
      world, start, end, 0.0,
      navigation_world_model::UnknownPolicy::kRequireKnownFree, 0.2));
}

TEST(PlannerTrajectory, ContinuousTubeClassifiesCellTouchedByCenterline) {
  const navigation_world_model::Point3 start(0.1, 0.2, 0.2);
  const navigation_world_model::Point3 end(1.1, 0.2, 0.2);
  DiagonalNeighborWorld world({0.1, 0.1, 0.1});

  // The same neighbour is now touched at the voxel corner and must remain a
  // fail-closed UNKNOWN for a BACKUP certificate.
  EXPECT_FALSE(navigation_planning_backend::certificateTubeIsSafe(
      world, start, end, 0.0,
      navigation_world_model::UnknownPolicy::kRequireKnownFree, 0.2));
}

TEST(PlannerTrajectory, ContinuousTubeReportsActualBlockingCell) {
  const navigation_world_model::Point3 start(0.1, 0.2, 0.2);
  const navigation_world_model::Point3 end(1.1, 0.2, 0.2);
  const navigation_world_model::Point3 expected_blocker(0.1, 0.1, 0.1);
  DiagonalNeighborWorld world(expected_blocker);
  navigation_world_model::CellState blocked_state{
      navigation_world_model::CellState::kUndefined};
  navigation_world_model::Point3 blocked_position =
      navigation_world_model::Point3::Constant(
          std::numeric_limits<double>::quiet_NaN());

  EXPECT_FALSE(navigation_planning_backend::certificateTubeIsSafe(
      world, start, end, 0.0,
      navigation_world_model::UnknownPolicy::kRequireKnownFree, 0.2,
      &blocked_state, &blocked_position));
  EXPECT_EQ(blocked_state, navigation_world_model::CellState::kUnknown);
  EXPECT_TRUE(blocked_position.isApprox(expected_blocker, 1.0e-12));
}

TEST(PlannerTrajectory, CandidateValidationReportsActualTubeBlocker) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 0.5, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
      {0.5, 1.0, navigation_planning_backend::CandidateTrajectoryRole::BACKUP},
  };
  // The main prefix is allowed to cross UNKNOWN, while the backup interval
  // must report the first actual UNKNOWN voxel in its swept tube.
  SweepWorld world;

  const auto result = navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kAllowUnknown);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure,
            navigation_planning_backend::SweptValidationResult::Failure::
                kCertificateTubeBlocked);
  EXPECT_EQ(result.blocked_cell_state,
            navigation_world_model::CellState::kUnknown);
  EXPECT_TRUE(result.blocked_position.allFinite());
  EXPECT_NEAR(result.blocked_position.x(), 0.5, 1.0e-12);
}

TEST(PlannerTrajectory, BackupRoleRequiresKnownFreeEvidence) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 0.5, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
      {0.5, 1.0, navigation_planning_backend::CandidateTrajectoryRole::BACKUP},
  };

  SweepWorld world;
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
      world, candidate, 10.0,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
  EXPECT_TRUE(navigation_planning_backend::candidateHasBackupSuffix(candidate));

  candidate.roles.pop_back();
  EXPECT_FALSE(navigation_planning_backend::candidateHasBackupSuffix(candidate));
}

TEST(PlannerTrajectory, MainOnlyAllowUnknownRequiresKnownFreeCertificate) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  EXPECT_EQ(
      navigation_planning_backend::candidateCertificatePolicy(
          candidate, navigation_world_model::UnknownPolicy::kAllowUnknown),
      navigation_world_model::UnknownPolicy::kRequireKnownFree);

  candidate.roles = {
      {0.0, 0.5, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
      {0.5, 1.0, navigation_planning_backend::CandidateTrajectoryRole::BACKUP},
  };
  EXPECT_EQ(
      navigation_planning_backend::candidateCertificatePolicy(
          candidate, navigation_world_model::UnknownPolicy::kAllowUnknown),
      navigation_world_model::UnknownPolicy::kAllowUnknown);
}

TEST(PlannerTrajectory, MainOnlyRevalidationCannotReuseAllowUnknownPolicy) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };

  SweepWorld newer_world;
  const auto mission_policy = navigation_world_model::UnknownPolicy::kAllowUnknown;
  const auto certificate_policy =
      navigation_planning_backend::candidateCertificatePolicy(candidate, mission_policy);
  EXPECT_EQ(certificate_policy,
            navigation_world_model::UnknownPolicy::kRequireKnownFree);
  EXPECT_FALSE(navigation_planning_backend::validateExecutableCandidate(
                   newer_world, candidate, 10.0, certificate_policy)
                   .valid);
}

TEST(PlannerTrajectory, LatestWorldSweepIgnoresAlreadyExecutedBlockedPrefix) {
  navigation_planning_backend::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  class PrefixWorld final : public SweepWorld {
   public:
    navigation_world_model::CellState classify(
        const navigation_world_model::Point3& p,
        navigation_world_model::GridLayer) const noexcept override {
      return p.x() < 0.4 ? navigation_world_model::CellState::kOccupied
                         : navigation_world_model::CellState::kUnknown;
    }
    bool isSegmentTraversable(
        const navigation_world_model::Point3& a,
        const navigation_world_model::Point3&,
        navigation_world_model::GridLayer,
        navigation_world_model::UnknownPolicy) const noexcept override {
      return a.x() >= 0.4;
    }
  } prefix_world;
  candidate.roles = {
      {0.0, 1.0, navigation_planning_backend::CandidateTrajectoryRole::MAIN},
  };
  EXPECT_TRUE(navigation_planning_backend::validateExecutableCandidate(
      prefix_world, candidate, 10.5,
      navigation_world_model::UnknownPolicy::kAllowUnknown).valid);
}

TEST(PlannerTrajectory, NonFiniteYawCannotReplaceCommittedGeneration) {
  Eigen::MatrixXd position_coefficients = Eigen::MatrixXd::Zero(3, 8);
  position_coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {position_coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 5.0;
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  const auto generation = command.generation();

  Eigen::MatrixXd invalid_coefficients = Eigen::MatrixXd::Zero(3, 8);
  invalid_coefficients(0, 7) = std::numeric_limits<double>::quiet_NaN();
  geometry_utils::Trajectory invalid_yaw({1.0}, {invalid_coefficients});
  invalid_yaw.start_WT = 5.0;
  EXPECT_FALSE(command.setEmergencyBackup(position, invalid_yaw));
  EXPECT_EQ(command.generation(), generation);
}

TEST(PlannerTrajectory, ConcurrentCommitAndSnapshotNeverMixGenerations) {
  Eigen::MatrixXd position_coefficients = Eigen::MatrixXd::Zero(3, 8);
  position_coefficients(0, 6) = 1.0;
  position_coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {position_coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 1.0;
  navigation_planning_backend::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  std::atomic_bool failed{false};
  std::thread writer([&] {
    for (int index = 0; index < 500; ++index) {
      if (!command.setEmergencyBackup(position, yaw)) failed.store(true);
    }
  });
  std::thread reader([&] {
    std::uint64_t previous_generation = 0;
    for (int index = 0; index < 500; ++index) {
      const auto snapshot = command.snapshot();
      const auto state = snapshot.position.getPos(0.5);
      if (snapshot.generation < previous_generation || !state.allFinite() ||
          snapshot.diagnostics.generation != snapshot.generation ||
          snapshot.diagnostics.previous_generation + 1U != snapshot.generation) {
        failed.store(true);
      }
      previous_generation = snapshot.generation;
    }
  });
  writer.join();
  reader.join();
  EXPECT_FALSE(failed.load());
}

TEST(PlannerTrajectory, FreeYawIsProjectedIntoRateEnvelopeWithoutChangingDuration) {
  std::vector<double> durations{0.8, 0.8, 0.8};
  std::vector<Eigen::MatrixXd> coefficients;
  const std::vector<Eigen::Vector3d> starts{
      {0.0, 0.0, 3.0}, {0.8, 0.0, 3.0}, {0.8, 0.8, 3.0}};
  const std::vector<Eigen::Vector3d> velocities{
      {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {-1.0, 0.0, 0.0}};
  for (std::size_t index = 0; index < durations.size(); ++index) {
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(3, 6);
    matrix.col(4) = velocities[index];
    matrix.col(5) = starts[index];
    coefficients.push_back(matrix);
  }
  geometry_utils::Trajectory position(durations, coefficients);
  const navigation_math::Vec4f initial_yaw{0.0, 0.0, 0.0, 0.0};
  const navigation_math::Vec4f free_goal_yaw{0.0, 0.0, 0.0, 0.0};
  traj_opt::YawTrajOpt optimizer(1.0);
  geometry_utils::Trajectory yaw;

  ASSERT_TRUE(optimizer.optimize(initial_yaw, free_goal_yaw, position, yaw,
                                 3, false, true));
  EXPECT_NEAR(yaw.getTotalDuration(), position.getTotalDuration(), 1.0e-12);
  EXPECT_LE(yaw.getMaxVelRate(), 1.0 + 1.0e-6);
  EXPECT_TRUE(yaw.getState(0.0).allFinite());
  EXPECT_TRUE(yaw.getState(yaw.getTotalDuration()).allFinite());

  // A commanded terminal attitude is a hard contract and must never be
  // weakened by the free-yaw projection.
  const navigation_math::Vec4f fixed_goal_yaw{M_PI, 0.0, 0.0, 0.0};
  geometry_utils::Trajectory rejected_fixed_yaw;
  EXPECT_FALSE(optimizer.optimize(initial_yaw, fixed_goal_yaw, position,
                                  rejected_fixed_yaw, 3, false, false));
}

TEST(PlannerTrajectory, SearchFallbacksShareOneAbsoluteDeadline) {
  const navigation_planning_backend::AbsoluteDeadline deadline(100.0, 0.04);
  EXPECT_NEAR(deadline.remaining(100.01), 0.03, 1.0e-12);
  EXPECT_NEAR(deadline.remaining(100.039), 0.001, 1.0e-12);
  EXPECT_DOUBLE_EQ(deadline.remaining(100.04), 0.0);
  EXPECT_TRUE(deadline.expired(100.05));
  EXPECT_DOUBLE_EQ(deadline.remaining(std::numeric_limits<double>::quiet_NaN()), 0.0);
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  EXPECT_GT(deadline.steadyDeadlineNanoseconds(), now_ns);
  EXPECT_FALSE(deadline.steadyExpired());
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, -1.0}), std::invalid_argument);
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, std::numeric_limits<double>::quiet_NaN()}),
               std::invalid_argument);
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, std::numeric_limits<double>::infinity()}),
               std::invalid_argument);
  EXPECT_THROW((navigation_planning_backend::AbsoluteDeadline{0.0, std::numeric_limits<double>::max()}),
               std::invalid_argument);

  // Simulation time may remain frozen while the optimizer is still executing.
  // The monotonic budget must expire independently of the ROS-time contract.
  const navigation_planning_backend::AbsoluteDeadline frozen_sim_time(42.0, 0.001);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  EXPECT_TRUE(frozen_sim_time.steadyExpired());
  EXPECT_NEAR(frozen_sim_time.remaining(42.0), 0.001, 1.0e-12);
}

TEST(PlannerTrajectory, ConnectedGoalIsResolvedBeforeCorridorConstruction) {
  const navigation_math::Vec3f guide_endpoint(1.0, 2.0, 3.0);
  const navigation_math::Vec3f goal(1.1, 2.0, 3.0);
  const auto result = navigation_planning_backend::resolveGuideEndpoint(
      guide_endpoint, goal, 0.4);
  EXPECT_TRUE(result.goal_connected);
  EXPECT_TRUE(result.position.isApprox(goal));
}

TEST(PlannerTrajectory, SimplifySfcPreservesRouteBoundaryGate) {
  using geometry_utils::MatD4f;
  using geometry_utils::Polytope;
  using geometry_utils::PolytopeVec;
  using navigation_math::Vec3f;

  const auto make_box = [](double min_x, double max_x,
                           double min_y, double max_y,
                           double min_z, double max_z) {
    MatD4f planes(6, 4);
    planes <<
        1.0, 0.0, 0.0, -max_x,
       -1.0, 0.0, 0.0, min_x,
        0.0, 1.0, 0.0, -max_y,
        0.0,-1.0, 0.0, min_y,
        0.0, 0.0, 1.0, -max_z,
        0.0, 0.0,-1.0, min_z;
    return Polytope(planes);
  };

  Polytope gate = make_box(0.8, 1.2, -0.2, 0.2, 2.8, 3.2);
  gate.SetRouteBoundaryContract(Vec3f{1.0F, 0.0F, 3.0F}, 0.9);
  PolytopeVec sfcs{
      make_box(-1.0, 2.0, -1.0, 1.0, 2.0, 4.0),
      gate,
      make_box(0.0, 3.0, -1.0, 1.0, 2.0, 4.0)};

  ASSERT_TRUE(geometry_utils::SimplifySFC(
      Vec3f(0.0, 0.0, 3.0), Vec3f(2.5, 0.0, 3.0), sfcs));
  ASSERT_EQ(sfcs.size(), 3U);
  EXPECT_TRUE(sfcs[1].IsRouteBoundaryGate());
  EXPECT_TRUE(sfcs[1].GetRouteBoundaryPoint().isApprox(
      Vec3f{1.0F, 0.0F, 3.0F}));
  EXPECT_DOUBLE_EQ(sfcs[1].GetRouteBoundaryRadius(), 0.9);
}

TEST(PlannerTrajectory, GoalPoliciesRemainNamedAndShareProvisionalValue) {
  EXPECT_DOUBLE_EQ(navigation_world_model::kGoalConnectionToleranceM,
                   navigation_world_model::kGoalCompletionToleranceM);
  EXPECT_DOUBLE_EQ(navigation_world_model::kNearGoalShortcutToleranceM,
                   navigation_world_model::kGoalCompletionToleranceM);
}

TEST(PlannerTrajectory, NearGoalSegmentRejectsOccupiedAndOutOfMapEndpoints) {
  SweepWorld world;
  const navigation_world_model::Point3 start(0.0, 0.0, 0.0);
  const navigation_world_model::Point3 goal(1.0, 0.0, 0.0);
  EXPECT_TRUE(navigation_world_model::isGoalSegmentTraversable(world, start, goal));

  world.blocked_from_x = 0.5;
  EXPECT_FALSE(navigation_world_model::isGoalSegmentTraversable(world, start, goal));

  world.blocked_from_x = std::numeric_limits<double>::infinity();
  world.endpoints_in_bounds = false;
  EXPECT_FALSE(navigation_world_model::isGoalSegmentTraversable(world, start, goal));
}

TEST(PlannerTrajectory, TimeAllocatorHandlesSwitchingDistanceRoundoff) {
  double elapsed = std::numeric_limits<double>::quiet_NaN();
  double velocity = std::numeric_limits<double>::quiet_NaN();
  constexpr double distance = 5.608921464952064;
  geometry_utils::simplePMTimeAllocator(
      3.0, 2.0, 0.0, distance, distance, elapsed, velocity);
  EXPECT_TRUE(std::isfinite(elapsed));
  EXPECT_GE(elapsed, 0.0);
  EXPECT_TRUE(std::isfinite(velocity));
}

TEST(PlannerTrajectory, GuideTimeAllocationUsesPointAlignedTravelledDistance) {
  const navigation_math::Vec3f start(0.0, 0.0, 0.0);
  const navigation_math::vec_E<navigation_math::Vec3f> nonuniform_path{
      start,
      navigation_math::Vec3f(0.3, 0.0, 0.0),
      navigation_math::Vec3f(4.7, 0.0, 0.0),
      navigation_math::Vec3f(30.0, 0.0, 0.0),
  };

  double previous_total_time = std::numeric_limits<double>::infinity();
  for (const double maximum_velocity : {2.0, 5.0, 6.0, 10.0}) {
    geometry_utils::GuideTimeAllocation allocation;
    ASSERT_TRUE(geometry_utils::allocateGuideElapsedTimes(
        5.0, maximum_velocity, 0.0, start, nonuniform_path, allocation));
    ASSERT_EQ(allocation.points.size(), 3U);
    ASSERT_EQ(allocation.elapsed_s.size(), allocation.points.size());
    EXPECT_TRUE(allocation.points.front().isApprox(nonuniform_path[1]));
    EXPECT_TRUE(allocation.points.back().isApprox(nonuniform_path.back()));
    EXPECT_NEAR(allocation.path_length_m, 30.0, 1.0e-12);
    EXPECT_GT(allocation.elapsed_s.front(), 0.0);
    for (std::size_t index = 1; index < allocation.elapsed_s.size(); ++index) {
      EXPECT_GT(allocation.elapsed_s[index], allocation.elapsed_s[index - 1]);
    }

    double expected_total_time = std::numeric_limits<double>::quiet_NaN();
    double terminal_velocity = std::numeric_limits<double>::quiet_NaN();
    geometry_utils::simplePMTimeAllocator(
        5.0, maximum_velocity, 0.0, 30.0, 30.0,
        expected_total_time, terminal_velocity);
    EXPECT_NEAR(allocation.elapsed_s.back(), expected_total_time, 1.0e-12);
    EXPECT_LE(allocation.elapsed_s.back(), previous_total_time);
    previous_total_time = allocation.elapsed_s.back();
  }
}

TEST(PlannerTrajectory, GuideTimeAllocationPreservesNonZeroInitialSpeedOnShortPath) {
  const navigation_math::Vec3f start(0.0, 0.0, 0.0);
  const navigation_math::vec_Vec3f short_path{
      start, navigation_math::Vec3f(1.0, 0.0, 0.0)};
  geometry_utils::GuideTimeAllocation allocation;
  ASSERT_TRUE(geometry_utils::allocateGuideElapsedTimes(
      2.0, 3.0, 2.4, start, short_path, allocation));
  ASSERT_EQ(allocation.points.size(), 1U);
  EXPECT_NEAR(allocation.elapsed_s.back(), 0.5367, 0.005);
  EXPECT_NEAR(allocation.terminal_velocity_mps, 1.3266, 0.005);
  EXPECT_GE(allocation.terminal_velocity_mps, 0.0);
}

TEST(PlannerTrajectory, GuideTimeAllocationRejectsDegenerateOrInvalidInputs) {
  const navigation_math::Vec3f start(0.0, 0.0, 0.0);
  const navigation_math::vec_E<navigation_math::Vec3f> duplicate_path{start, start};
  geometry_utils::GuideTimeAllocation allocation;
  EXPECT_FALSE(geometry_utils::allocateGuideElapsedTimes(
      5.0, 10.0, 0.0, start, duplicate_path, allocation));
  EXPECT_FALSE(geometry_utils::allocateGuideElapsedTimes(
      0.0, 10.0, 0.0, start,
      navigation_math::vec_E<navigation_math::Vec3f>{navigation_math::Vec3f(1.0, 0.0, 0.0)},
      allocation));
}

TEST(PlannerTrajectory, GoalConnectionUsesInclusiveBoundary) {
  const navigation_math::Vec3f guide_endpoint(0.0, 0.0, 0.0);
  const navigation_math::Vec3f goal(0.2, 0.0, 0.0);
  const auto result = navigation_planning_backend::resolveGuideEndpoint(
      guide_endpoint, goal, 0.2);
  EXPECT_TRUE(result.goal_connected);
  EXPECT_TRUE(result.position.isApprox(goal));
}

TEST(PlannerTrajectory, SolveFailureCodesRemainDistinct) {
  EXPECT_EQ(navigation_planning_backend::PlannerResultCode_STR(
                navigation_planning_backend::PLANNER_SOLVE_TIMEOUT),
            "Solve deadline exhausted before a candidate could be committed");
  EXPECT_EQ(navigation_planning_backend::PlannerResultCode_STR(
                navigation_planning_backend::PLANNER_SOLVE_CANCELLED),
            "Solve was cancelled before a candidate could be committed");
  EXPECT_NE(navigation_planning_backend::PLANNER_SOLVE_TIMEOUT,
            navigation_planning_backend::PLANNER_BACKUP_FAILED);
  EXPECT_NE(navigation_planning_backend::PLANNER_SOLVE_CANCELLED,
            navigation_planning_backend::PLANNER_BACKUP_FAILED);
  EXPECT_NE(navigation_planning_backend::PLANNER_EXP_FAILED,
            navigation_planning_backend::PLANNER_BACKUP_FAILED);
  EXPECT_EQ(navigation_planning_backend::PlannerResultCode_STR(
                navigation_planning_backend::PLANNER_CANDIDATE_REJECTED),
            "Generated candidate failed construction or world validation");
}

TEST(PlannerTrajectory, BackupFailureKeepsActionableCause) {
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::TIME_OUT),
            navigation_planning_backend::PLANNER_SOLVE_TIMEOUT);
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::OPT_FAILED),
            navigation_planning_backend::PLANNER_BACKUP_OPTIMIZATION_FAILED);
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::INIT_ERROR),
            navigation_planning_backend::PLANNER_BACKUP_INITIALIZATION_FAILED);
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::NO_PATH),
            navigation_planning_backend::PLANNER_BACKUP_NO_PATH);
  EXPECT_EQ(navigation_planning_backend::classifyBackupResult(
                navigation_math::FAILED),
            navigation_planning_backend::PLANNER_BACKUP_FAILED);
}

TEST(PlannerTrajectory, UnconnectedOrInvalidGoalDoesNotMoveGuideEndpoint) {
  const navigation_math::Vec3f guide_endpoint(1.0, 2.0, 3.0);
  const navigation_math::Vec3f goal(2.0, 2.0, 3.0);
  const auto unconnected = navigation_planning_backend::resolveGuideEndpoint(
      guide_endpoint, goal, 0.4);
  EXPECT_FALSE(unconnected.goal_connected);
  EXPECT_TRUE(unconnected.position.isApprox(guide_endpoint));

  navigation_math::Vec3f invalid_goal = goal;
  invalid_goal.x() = std::numeric_limits<double>::quiet_NaN();
  const auto invalid = navigation_planning_backend::resolveGuideEndpoint(
      guide_endpoint, invalid_goal, 0.4);
  EXPECT_FALSE(invalid.goal_connected);
  EXPECT_TRUE(invalid.position.isApprox(guide_endpoint));
}

TEST(PlannerTrajectory, SolveStagesHaveStableDecisionTraceNames) {
  EXPECT_EQ(navigation_planning_backend::solveStageName(0), "idle");
  EXPECT_EQ(navigation_planning_backend::solveStageName(2), "astar");
  EXPECT_EQ(navigation_planning_backend::solveStageName(4), "main_minco");
  EXPECT_EQ(navigation_planning_backend::solveStageName(5), "backup");
  EXPECT_EQ(navigation_planning_backend::solveStageName(33), "corridor_iris");
  EXPECT_EQ(navigation_planning_backend::solveStageName(999), "unknown");
}

TEST(PlannerTrajectory, UnknownReturnCodeHasDeterministicDiagnostic) {
  EXPECT_EQ(navigation_planning_backend::PlannerResultCode_STR(42),
            "Unknown planner return code (42)");
}
