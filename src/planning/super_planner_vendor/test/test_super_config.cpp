#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <rog_map/rog_map_core/config.hpp>
#include <super_core/backup_braking.hpp>
#include <super_core/config.hpp>

TEST(SuperProductConfig, SatisfiesVisibilityInflationAndReplanBudgets) {
  const super_planner::Config planner(SUPER_PRODUCT_CONFIG_PATH);
  const rog_map::Config map(SUPER_PRODUCT_CONFIG_PATH);

  const double visibility_horizon = planner.sensing_horizon > 0.0
      ? std::min(planner.sensing_horizon, planner.safe_corridor_line_max_length)
      : planner.safe_corridor_line_max_length;
  const double required_horizon =
      super_planner::jerkLimitedStopDistance(
          planner.exp_traj_cfg.max_vel, planner.back_traj_cfg.max_acc,
          planner.back_traj_cfg.max_jerk) +
      2.0 * planner.exp_traj_cfg.max_vel * planner.replan_forward_dt +
      planner.robot_r;
  EXPECT_GE(visibility_horizon, required_horizon);
  EXPECT_GE(map.inflation_resolution * map.inflation_step, planner.robot_r);
  EXPECT_LE(planner.astar_search_time_limit_s, planner.replan_forward_dt * 0.25);
  EXPECT_GE(planner.astar_total_time_limit_s, planner.astar_search_time_limit_s);
  EXPECT_LT(planner.astar_total_time_limit_s, planner.solve_deadline_s);
  EXPECT_LE(planner.solve_deadline_s, planner.replan_forward_dt);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.corridor_plane_tolerance_m, 0.01);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.vertical_guide_tolerance_m, 0.05);
  EXPECT_DOUBLE_EQ(planner.back_traj_cfg.corridor_plane_tolerance_m, 0.01);
}

TEST(SuperProductConfig, MissionLimitsLowerButNeverRaiseProductEnvelope) {
  const super_planner::DynamicLimits mission{7.0, 5.0, 12.0};
  const super_planner::Config planner(SUPER_PRODUCT_CONFIG_PATH, mission);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.max_vel, 7.0);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.max_acc, 5.0);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.max_jerk, 12.0);
  EXPECT_DOUBLE_EQ(planner.back_traj_cfg.max_vel, 7.0);
  EXPECT_DOUBLE_EQ(planner.safe_corridor_line_max_length, 14.0);
  EXPECT_THROW(
      (super_planner::Config(
          SUPER_PRODUCT_CONFIG_PATH,
          super_planner::DynamicLimits{12.1, 5.0, 12.0})),
      std::invalid_argument);
}

TEST(SuperBackupBraking, UsesJerkLimitedTriangularAndTrapezoidalProfiles) {
  EXPECT_NEAR(super_planner::jerkLimitedStopTime(0.5, 2.0, 8.0), 0.5, 1.0e-12);
  EXPECT_NEAR(super_planner::jerkLimitedStopTime(2.0, 2.0, 8.0), 1.25, 1.0e-12);
  EXPECT_NEAR(super_planner::jerkLimitedStopDistance(2.0, 2.0, 8.0), 1.25,
              1.0e-12);
}

TEST(SuperBackupBraking, MinimumSnapSeedPreservesPVAJAndStopsWithinBounds) {
  super_utils::StatePVAJ initial;
  initial.setZero();
  initial.col(0) << 1.0, -2.0, 3.0;
  initial.col(1) << 2.0, 0.25, -0.1;
  initial.col(2) << 0.2, -0.1, 0.05;
  initial.col(3) << 0.1, 0.05, -0.02;

  const auto seed = super_planner::makeBackupBrakingSeed(
      0.6, initial, 3.0, 2.0, 8.0, 0.05, 0.05);
  ASSERT_TRUE(seed.feasible);
  ASSERT_GT(seed.duration_s, 0.0);
  const auto piece = super_planner::minimumSnapStopPiece(initial, seed.duration_s);
  const auto control_points =
      super_planner::minimumSnapStopBezierControlPoints(initial, seed.duration_s);
  EXPECT_TRUE(piece.getState(0.0).isApprox(initial, 1.0e-9));
  EXPECT_TRUE(piece.getPos(seed.duration_s).isApprox(seed.endpoint, 1.0e-9));
  EXPECT_NEAR(piece.getVel(seed.duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR(piece.getAcc(seed.duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR(piece.getJer(seed.duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_LE(seed.maximum_velocity_mps, 3.0 * 1.05);
  EXPECT_LE(seed.maximum_acceleration_mps2, 2.0 * 1.05);
  EXPECT_LE(seed.maximum_jerk_mps3, 8.0 * 1.05);
  EXPECT_TRUE(control_points.col(0).isApprox(initial.col(0), 1.0e-9));
  EXPECT_TRUE(control_points.col(7).isApprox(seed.endpoint, 1.0e-9));
  for (int sample = 0; sample <= 20; ++sample) {
    const double u = static_cast<double>(sample) / 20.0;
    Eigen::Vector3d bezier = Eigen::Vector3d::Zero();
    constexpr int choose7[8] = {1, 7, 21, 35, 35, 21, 7, 1};
    for (int i = 0; i <= 7; ++i) {
      bezier += choose7[i] * std::pow(u, i) * std::pow(1.0 - u, 7 - i) *
                control_points.col(i);
    }
    EXPECT_TRUE(bezier.isApprox(piece.getPos(u * seed.duration_s), 1.0e-8));
  }
}

TEST(SuperBackupBraking, RejectsNonFiniteState) {
  super_utils::StatePVAJ initial;
  initial.setZero();
  initial(0, 1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(super_planner::makeBackupBrakingSeed(
      0.0, initial, 3.0, 2.0, 8.0, 0.05, 0.05).feasible);
}

TEST(SuperBackupBraking, PreservesMeasuredOverspeedWithoutIncreasingIt) {
  super_utils::StatePVAJ initial = super_utils::StatePVAJ::Zero();
  initial.col(0) << 2.0, -1.0, 3.0;
  initial.col(1) << 13.0, 0.0, 0.0;
  initial.col(2).setZero();
  initial.col(3).setZero();

  const auto seed = super_planner::makeBackupBrakingSeed(
      0.0, initial, 12.0, 5.0, 12.0, 0.05, 0.0);
  ASSERT_TRUE(seed.feasible)
      << "duration=" << seed.duration_s
      << " peak_v=" << seed.maximum_velocity_mps
      << " peak_a=" << seed.maximum_acceleration_mps2
      << " peak_j=" << seed.maximum_jerk_mps3;
  EXPECT_TRUE(seed.initial_overspeed);
  EXPECT_DOUBLE_EQ(seed.initial_velocity_mps, 13.0);
  EXPECT_DOUBLE_EQ(seed.allowed_peak_velocity_mps, 13.0);
  EXPECT_LE(seed.maximum_velocity_mps, seed.initial_velocity_mps + 1.0e-9);
  EXPECT_LE(seed.maximum_acceleration_mps2, 5.0);
  EXPECT_LE(seed.maximum_jerk_mps3, 12.0);

  const auto piece = super_planner::minimumSnapStopPiece(initial, seed.duration_s);
  EXPECT_TRUE(piece.getState(0.0).isApprox(initial, 1.0e-9));
  EXPECT_NEAR(piece.getVel(seed.duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR(piece.getAcc(seed.duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR(piece.getJer(seed.duration_s).norm(), 0.0, 1.0e-8);
}

TEST(SuperBackupBraking, RefinementDurationCannotUndercutCertifiedSeed) {
  EXPECT_TRUE(super_planner::refinementDurationRespectsCertifiedFloor(2.0, 2.0));
  EXPECT_TRUE(super_planner::refinementDurationRespectsCertifiedFloor(2.5, 2.0));
  EXPECT_FALSE(super_planner::refinementDurationRespectsCertifiedFloor(1.99, 2.0));
  EXPECT_FALSE(super_planner::refinementDurationRespectsCertifiedFloor(
      std::numeric_limits<double>::quiet_NaN(), 2.0));
}
