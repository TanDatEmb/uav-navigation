#include <cmath>
#include <atomic>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "data_structure/base/trajectory.h"
#include "super_core/super_planner.h"
#include "super_core/absolute_deadline.hpp"
#include "super_core/command_time.hpp"
#include "super_core/guide_endpoint.hpp"
#include "super_core/replan_contract.hpp"
#include "super_core/solve_stage.hpp"
#include "super_core/trajectory_world_validator.hpp"
#include "traj_opt/trajectory_dynamics.hpp"
#include "traj_opt/yaw_traj_opt.h"

namespace {
class SweepWorld : public navigation_world_model::WorldModelView {
 public:
  double blocked_from_x{std::numeric_limits<double>::infinity()};
  navigation_world_model::WorldGeometry geometry() const noexcept override {
    navigation_world_model::WorldGeometry value;
    value.inflated_resolution_m = 0.2;
    return value;
  }
  navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return {1, 1, 1};
  }
  navigation_world_model::CellState classify(
      const navigation_world_model::Point3& p,
      navigation_world_model::GridLayer) const noexcept override {
    return p.x() >= blocked_from_x
        ? navigation_world_model::CellState::kOccupied
        : navigation_world_model::CellState::kUnknown;
  }
  bool contains(const navigation_world_model::Point3&) const noexcept override { return true; }
  navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3&, navigation_world_model::GridLayer) const noexcept override {
    return {};
  }
  navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3&, navigation_world_model::GridLayer) const noexcept override {
    return {};
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

geometry_utils::Trajectory linearTrajectory(double duration, double start_wall_time) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 1.0;
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory result({duration}, {coefficients});
  result.start_WT = start_wall_time;
  return result;
}
}  // namespace

TEST(SuperTrajectory, PartialSlicePreservesPieceLocalTimeAndContinuity) {
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

TEST(SuperTrajectory, CommandTrajectoryTimePreservesEstablishedSamplingSemantics) {
  const auto before = super_planner::commandTrajectoryTime(9.75, 10.0, 2.0);
  EXPECT_FALSE(before.finished);
  EXPECT_DOUBLE_EQ(before.trajectory_time_s, -0.25);

  const auto start = super_planner::commandTrajectoryTime(10.0, 10.0, 2.0);
  EXPECT_FALSE(start.finished);
  EXPECT_DOUBLE_EQ(start.trajectory_time_s, 0.0);

  const auto middle = super_planner::commandTrajectoryTime(11.25, 10.0, 2.0);
  EXPECT_FALSE(middle.finished);
  EXPECT_DOUBLE_EQ(middle.trajectory_time_s, 1.25);

  const auto finished = super_planner::commandTrajectoryTime(12.1, 10.0, 2.0);
  EXPECT_TRUE(finished.finished);
  EXPECT_DOUBLE_EQ(finished.trajectory_time_s, 2.0);
}

TEST(SuperTrajectory, OnlySuccessfulExpResultMayBuildAndCommitNewCandidate) {
  EXPECT_TRUE(super_planner::expResultMayBuildCommandCandidate(super_utils::SUCCESS));
  EXPECT_FALSE(super_planner::expResultMayBuildCommandCandidate(super_utils::NO_NEED));
  EXPECT_FALSE(super_planner::expResultMayBuildCommandCandidate(super_utils::FAILED));
  EXPECT_FALSE(super_planner::expResultMayBuildCommandCandidate(super_utils::NEW_TRAJ));
  EXPECT_FALSE(super_planner::expResultMayBuildCommandCandidate(super_utils::EMER));

  EXPECT_TRUE(super_planner::backupResultMayBuildCommandCandidate(super_utils::SUCCESS));
  EXPECT_TRUE(super_planner::backupResultMayBuildCommandCandidate(super_utils::NO_NEED));
  EXPECT_TRUE(super_planner::backupResultMayBuildCommandCandidate(super_utils::FINISH));
  EXPECT_FALSE(super_planner::backupResultMayBuildCommandCandidate(super_utils::FAILED));
  EXPECT_FALSE(super_planner::backupResultMayBuildCommandCandidate(super_utils::OPT_FAILED));
  EXPECT_FALSE(super_planner::backupResultMayBuildCommandCandidate(super_utils::EMER));
}

TEST(SuperTrajectory, FlatnessGateRejectsExcessBodyRateAndThrust) {
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
  config.penna_margin = 0.0;
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

TEST(SuperTrajectory, EmergencyBundleIsAtomicallyOwnedByBackup) {
  Eigen::MatrixXd position_coefficients = Eigen::MatrixXd::Zero(3, 8);
  position_coefficients(0, 6) = 1.0;
  position_coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {position_coefficients});
  position.start_WT = 42.0;

  Eigen::MatrixXd yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  yaw_coefficients(0, 7) = 0.4;
  geometry_utils::Trajectory yaw({1.0}, {yaw_coefficients});
  yaw.start_WT = 42.0;

  super_planner::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  EXPECT_FALSE(command.empty());
  EXPECT_TRUE(command.backupTrajAvilibale());
  EXPECT_DOUBLE_EQ(command.getBackupTrajStartTT(), 0.0);
  EXPECT_TRUE(command.isTTOnBackupTraj(0.01));
  EXPECT_NEAR(command.getPos(0.5).x(), 0.5, 1.0e-12);
  EXPECT_NEAR(command.getYaw(0.5).x(), 0.4, 1.0e-12);
}

TEST(SuperTrajectory, RobotStateHasDeterministicFiniteDefaults) {
  const super_utils::RobotState state;
  EXPECT_TRUE(state.p.isZero());
  EXPECT_TRUE(state.v.isZero());
  EXPECT_TRUE(state.a.isZero());
  EXPECT_TRUE(state.j.isZero());
  EXPECT_TRUE(state.q.coeffs().allFinite());
  EXPECT_DOUBLE_EQ(state.q.norm(), 1.0);
  EXPECT_FALSE(state.rcv);
}

TEST(SuperTrajectory, FailedCandidateLeavesEmergencyBundleUnchanged) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 7.0;
  super_planner::CmdTraj command;
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

TEST(SuperTrajectory, CommitDiagnosticsDescribeExactOldToNewSplice) {
  auto first_position = linearTrajectory(1.0, 10.0);
  auto first_yaw = linearTrajectory(1.0, 10.0);
  super_planner::CmdTraj command;
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

TEST(SuperTrajectory, CommitDiagnosticsClampPriorSampleAtFinishedEnd) {
  super_planner::CmdTraj command;
  auto first_position = linearTrajectory(1.0, 10.0);
  auto first_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(first_position, first_yaw));

  auto after_position = linearTrajectory(1.0, 20.0);
  auto after_yaw = linearTrajectory(1.0, 20.0);
  ASSERT_TRUE(command.setEmergencyBackup(after_position, after_yaw));
  EXPECT_DOUBLE_EQ(command.snapshot().diagnostics.previous_sample_tt, 1.0);
}

TEST(SuperTrajectory, RegressedCandidateStartTimeCannotReplaceCommittedBundle) {
  super_planner::CmdTraj command;
  auto current_position = linearTrajectory(1.0, 10.0);
  auto current_yaw = linearTrajectory(1.0, 10.0);
  ASSERT_TRUE(command.setEmergencyBackup(current_position, current_yaw));
  const auto before = command.snapshot();
  const bool before_role_at_start = command.isTTOnBackupTraj(0.0);
  const bool before_role_at_end = command.isTTOnBackupTraj(1.0);

  auto historical_position = linearTrajectory(1.0, 9.5);
  auto historical_yaw = linearTrajectory(1.0, 9.5);
  auto historical = super_planner::CmdTraj::buildEmergencyCandidate(
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

TEST(SuperTrajectory, EqualOrNewerCandidateStartTimeCanCommit) {
  super_planner::CmdTraj command;
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

TEST(SuperTrajectory, CommitDiagnosticsWrapYawResidualAcrossPiBoundary) {
  auto position = linearTrajectory(1.0, 10.0);
  Eigen::MatrixXd first_yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  first_yaw_coefficients(0, 7) = std::acos(-1.0) - 0.1;
  geometry_utils::Trajectory first_yaw({1.0}, {first_yaw_coefficients});
  first_yaw.start_WT = 10.0;
  super_planner::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, first_yaw));

  Eigen::MatrixXd second_yaw_coefficients = Eigen::MatrixXd::Zero(3, 8);
  second_yaw_coefficients(0, 7) = -std::acos(-1.0) + 0.1;
  geometry_utils::Trajectory second_yaw({1.0}, {second_yaw_coefficients});
  second_yaw.start_WT = 10.0;
  ASSERT_TRUE(command.setEmergencyBackup(position, second_yaw));
  EXPECT_NEAR(command.snapshot().diagnostics.yaw_residual, 0.2, 1.0e-12);
}

TEST(SuperTrajectory, InheritedBackupPrefixSurvivesMainOnlyCommit) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  super_planner::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw, 0.0, 0.4);
  super_planner::CmdTraj command;
  command.setTrajectory(exp);
  EXPECT_FALSE(command.backupTrajAvilibale());
  EXPECT_TRUE(command.isTTOnBackupTraj(0.0));
  EXPECT_TRUE(command.isTTOnBackupTraj(0.4));
  EXPECT_FALSE(command.isTTOnBackupTraj(0.5));
}

TEST(SuperTrajectory, CandidateBuilderPreservesInheritedAndNewBackupRoles) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  super_planner::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw, 0.1, 0.3);
  super_planner::BackupTraj backup;
  auto backup_position = linearTrajectory(0.5, 10.6);
  auto backup_yaw = linearTrajectory(0.5, 10.6);
  backup.setTrajectory(10.6, 0.6, backup_position, backup_yaw);

  auto candidate = super_planner::CmdTraj::buildCandidate(
      exp, &backup, super_planner::BackupDisposition::SUCCESS);
  ASSERT_TRUE(candidate);
  EXPECT_NEAR(candidate->position.getTotalDuration(), 1.1, 1.0e-12);
  ASSERT_EQ(candidate->roles.size(), 4U);
  EXPECT_EQ(candidate->roles[0].role, super_planner::CandidateTrajectoryRole::MAIN);
  EXPECT_DOUBLE_EQ(candidate->roles[1].begin_tt, 0.1);
  EXPECT_DOUBLE_EQ(candidate->roles[1].end_tt, 0.3);
  EXPECT_EQ(candidate->roles[1].role, super_planner::CandidateTrajectoryRole::BACKUP);
  EXPECT_DOUBLE_EQ(candidate->roles[3].begin_tt, 0.6);
  EXPECT_DOUBLE_EQ(candidate->roles[3].end_tt, 1.1);
  EXPECT_EQ(candidate->roles[3].role, super_planner::CandidateTrajectoryRole::BACKUP);
}

TEST(SuperTrajectory, InheritedBackupIntersectionNeverCrossesNewSuffix) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  auto backup_position = linearTrajectory(0.5, 10.6);
  auto backup_yaw = linearTrajectory(0.5, 10.6);
  super_planner::BackupTraj backup;
  backup.setTrajectory(10.6, 0.6, backup_position, backup_yaw);

  for (const auto [begin, end] : std::vector<std::pair<double, double>>{
           {0.7, 0.9}, {0.5, 0.8}, {0.6, 0.6}, {0.2, 0.6}}) {
    super_planner::ExpTraj exp;
    exp.setTrajectory(10.0, position, yaw, begin, end);
    auto candidate = super_planner::CmdTraj::buildCandidate(
        exp, &backup, super_planner::BackupDisposition::SUCCESS);
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
                                         super_planner::CandidateTrajectoryRole::BACKUP &&
                                     interval.begin_tt <= 0.6 && interval.end_tt >= 0.6;
                            }));
    if (begin > 0.6) {
      const auto first_backup = std::find_if(
          candidate->roles.begin(), candidate->roles.end(), [](const auto& interval) {
            return interval.role == super_planner::CandidateTrajectoryRole::BACKUP;
          });
      ASSERT_NE(first_backup, candidate->roles.end());
      EXPECT_DOUBLE_EQ(first_backup->begin_tt, 0.6);
    }
  }
}

TEST(SuperTrajectory, ExpOnlyDispositionsAndEmergencyPreserveProvenance) {
  auto position = linearTrajectory(1.0, 10.0);
  auto yaw = linearTrajectory(1.0, 10.0);
  super_planner::ExpTraj exp;
  exp.setTrajectory(10.0, position, yaw, 0.0, 0.2);
  for (const auto disposition : {super_planner::BackupDisposition::FINISH,
                                 super_planner::BackupDisposition::NO_NEED}) {
    auto candidate = super_planner::CmdTraj::buildCandidate(
        exp, nullptr, disposition);
    ASSERT_TRUE(candidate);
    EXPECT_EQ(candidate->backup_disposition, disposition);
    EXPECT_FALSE(candidate->backup_suffix_available);
    ASSERT_EQ(candidate->roles.size(), 2U);
    EXPECT_EQ(candidate->roles.front().role,
              super_planner::CandidateTrajectoryRole::BACKUP);
    EXPECT_EQ(candidate->roles.back().role,
              super_planner::CandidateTrajectoryRole::MAIN);
  }

  auto emergency = super_planner::CmdTraj::buildEmergencyCandidate(position, yaw);
  ASSERT_TRUE(emergency);
  EXPECT_EQ(emergency->backup_disposition,
            super_planner::BackupDisposition::EMERGENCY);
  ASSERT_EQ(emergency->roles.size(), 1U);
  EXPECT_EQ(emergency->roles.front().role,
            super_planner::CandidateTrajectoryRole::BACKUP);
}

TEST(SuperTrajectory, LatestWorldSweepAllowsUnknownAndRejectsFutureObstacle) {
  super_planner::CandidateCommandBundle candidate;
  candidate.position = linearTrajectory(1.0, 10.0);
  candidate.yaw = linearTrajectory(1.0, 10.0);
  candidate.start_wall_time = 10.0;
  SweepWorld world;
  EXPECT_TRUE(super_planner::validateExecutableCandidate(world, candidate, 10.0).valid);
  world.blocked_from_x = 0.7;
  const auto blocked = super_planner::validateExecutableCandidate(world, candidate, 10.0);
  EXPECT_FALSE(blocked.valid);
  EXPECT_GE(blocked.first_blocked_tt, 0.6);
}

TEST(SuperTrajectory, LatestWorldSweepIgnoresAlreadyExecutedBlockedPrefix) {
  super_planner::CandidateCommandBundle candidate;
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
  EXPECT_TRUE(super_planner::validateExecutableCandidate(
      prefix_world, candidate, 10.5).valid);
}

TEST(SuperTrajectory, NonFiniteYawCannotReplaceCommittedGeneration) {
  Eigen::MatrixXd position_coefficients = Eigen::MatrixXd::Zero(3, 8);
  position_coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {position_coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 5.0;
  super_planner::CmdTraj command;
  ASSERT_TRUE(command.setEmergencyBackup(position, yaw));
  const auto generation = command.generation();

  Eigen::MatrixXd invalid_coefficients = Eigen::MatrixXd::Zero(3, 8);
  invalid_coefficients(0, 7) = std::numeric_limits<double>::quiet_NaN();
  geometry_utils::Trajectory invalid_yaw({1.0}, {invalid_coefficients});
  invalid_yaw.start_WT = 5.0;
  EXPECT_FALSE(command.setEmergencyBackup(position, invalid_yaw));
  EXPECT_EQ(command.generation(), generation);
}

TEST(SuperTrajectory, ConcurrentCommitAndSnapshotNeverMixGenerations) {
  Eigen::MatrixXd position_coefficients = Eigen::MatrixXd::Zero(3, 8);
  position_coefficients(0, 6) = 1.0;
  position_coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {position_coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 1.0;
  super_planner::CmdTraj command;
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

TEST(SuperTrajectory, FreeYawIsProjectedIntoRateEnvelopeWithoutChangingDuration) {
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
  const super_utils::Vec4f initial_yaw{0.0, 0.0, 0.0, 0.0};
  const super_utils::Vec4f free_goal_yaw{0.0, 0.0, 0.0, 0.0};
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
  const super_utils::Vec4f fixed_goal_yaw{M_PI, 0.0, 0.0, 0.0};
  geometry_utils::Trajectory rejected_fixed_yaw;
  EXPECT_FALSE(optimizer.optimize(initial_yaw, fixed_goal_yaw, position,
                                  rejected_fixed_yaw, 3, false, false));
}

TEST(SuperTrajectory, SearchFallbacksShareOneAbsoluteDeadline) {
  const super_planner::AbsoluteDeadline deadline(100.0, 0.04);
  EXPECT_NEAR(deadline.remaining(100.01), 0.03, 1.0e-12);
  EXPECT_NEAR(deadline.remaining(100.039), 0.001, 1.0e-12);
  EXPECT_DOUBLE_EQ(deadline.remaining(100.04), 0.0);
  EXPECT_TRUE(deadline.expired(100.05));
  EXPECT_DOUBLE_EQ(deadline.remaining(std::numeric_limits<double>::quiet_NaN()), 0.0);
  EXPECT_THROW((super_planner::AbsoluteDeadline{0.0, 0.0}), std::invalid_argument);
}

TEST(SuperTrajectory, ConnectedGoalIsResolvedBeforeCorridorConstruction) {
  const super_utils::Vec3f guide_endpoint(1.0, 2.0, 3.0);
  const super_utils::Vec3f goal(1.1, 2.0, 3.0);
  const auto result = super_planner::resolveGuideEndpoint(
      guide_endpoint, goal, 0.4);
  EXPECT_TRUE(result.goal_connected);
  EXPECT_TRUE(result.position.isApprox(goal));
}

TEST(SuperTrajectory, UnconnectedOrInvalidGoalDoesNotMoveGuideEndpoint) {
  const super_utils::Vec3f guide_endpoint(1.0, 2.0, 3.0);
  const super_utils::Vec3f goal(2.0, 2.0, 3.0);
  const auto unconnected = super_planner::resolveGuideEndpoint(
      guide_endpoint, goal, 0.4);
  EXPECT_FALSE(unconnected.goal_connected);
  EXPECT_TRUE(unconnected.position.isApprox(guide_endpoint));

  super_utils::Vec3f invalid_goal = goal;
  invalid_goal.x() = std::numeric_limits<double>::quiet_NaN();
  const auto invalid = super_planner::resolveGuideEndpoint(
      guide_endpoint, invalid_goal, 0.4);
  EXPECT_FALSE(invalid.goal_connected);
  EXPECT_TRUE(invalid.position.isApprox(guide_endpoint));
}

TEST(SuperTrajectory, SolveStagesHaveStableDecisionTraceNames) {
  EXPECT_EQ(super_planner::solveStageName(0), "idle");
  EXPECT_EQ(super_planner::solveStageName(2), "astar");
  EXPECT_EQ(super_planner::solveStageName(4), "main_minco");
  EXPECT_EQ(super_planner::solveStageName(5), "backup");
  EXPECT_EQ(super_planner::solveStageName(33), "corridor_iris");
  EXPECT_EQ(super_planner::solveStageName(999), "unknown");
}

TEST(SuperTrajectory, UnknownReturnCodeHasDeterministicDiagnostic) {
  EXPECT_EQ(super_planner::SUPER_RET_CODE_STR(42),
            "Unknown SUPER return code (42)");
}
