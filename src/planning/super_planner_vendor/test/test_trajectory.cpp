#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "data_structure/base/trajectory.h"
#include "super_core/super_planner.h"
#include "traj_opt/trajectory_dynamics.hpp"

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
