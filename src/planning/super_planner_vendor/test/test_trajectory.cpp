#include <cmath>
#include <atomic>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "data_structure/base/trajectory.h"
#include "super_core/super_planner.h"
#include "traj_opt/trajectory_dynamics.hpp"
#include "traj_opt/yaw_traj_opt.h"

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
  const auto before = command.getPos(0.5);
  geometry_utils::Trajectory invalid_yaw;
  EXPECT_FALSE(command.setEmergencyBackup(position, invalid_yaw));
  EXPECT_EQ(command.generation(), generation);
  EXPECT_TRUE(command.getPos(0.5).isApprox(before));
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
      command.lock();
      const auto generation = command.generation();
      const auto state = command.getPos(0.5);
      const bool backup = command.isTTOnBackupTraj(0.5);
      command.unlock();
      if (generation < previous_generation || !state.allFinite() || !backup) {
        failed.store(true);
      }
      previous_generation = generation;
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
