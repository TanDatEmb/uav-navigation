#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <rog_map/rog_map_core/config.hpp>
#include <planner_core/backup_braking.hpp>
#include <planner_core/config.hpp>
#include <planner_core/corridor_plane_validation.hpp>
#include <utils/optimization/optimization_utils.h>

TEST(PlannerDurationParameterization, KeepsFreeDurationAboveLowerBound) {
  navigation_math::VecDf tau(5);
  tau << -3.0, -0.25, 0.0, 0.5, 2.0;

  using MappedVector = Eigen::Map<Eigen::VectorXd>;
  navigation_math::VecDf free_duration_s;
  optimization_utils::Gcopter<MappedVector>::forwardMapTauToT(tau, free_duration_s);

  navigation_math::VecDf duration_lower_bound_s(5);
  duration_lower_bound_s << 1.0, 1.5, 2.0, 2.5, 3.0;
  const navigation_math::VecDf total_duration_s =
      duration_lower_bound_s + free_duration_s;

  ASSERT_TRUE(free_duration_s.allFinite());
  ASSERT_TRUE(total_duration_s.allFinite());
  EXPECT_GT(free_duration_s.minCoeff(), 0.0);
  for (int index = 0; index < total_duration_s.size(); ++index) {
    EXPECT_GE(total_duration_s(index), duration_lower_bound_s(index));
  }
}

TEST(PlannerDurationParameterization, RoundTripsFreeDurationSeed) {
  navigation_math::VecDf free_duration_s(3);
  free_duration_s << 0.05, 1.0, 10.0;
  navigation_math::VecDf tau_storage(3);
  using MappedVector = Eigen::Map<Eigen::VectorXd>;
  MappedVector tau(tau_storage.data(), 3);
  optimization_utils::Gcopter<MappedVector>::backwardMapTToTau(free_duration_s, tau);

  navigation_math::VecDf reconstructed_free_duration_s;
  optimization_utils::Gcopter<MappedVector>::forwardMapTauToT(
      tau, reconstructed_free_duration_s);
  EXPECT_TRUE(reconstructed_free_duration_s.isApprox(free_duration_s, 1.0e-12));
}

TEST(PlannerProductConfig, SatisfiesVisibilityInflationAndReplanBudgets) {
  navigation_planning_backend::Config planner(PLANNER_PRODUCT_CONFIG_PATH);
  EXPECT_EQ(planner.unknown_space_policy,
            navigation_world_model::UnknownPolicy::kRequireKnownFree);
  const rog_map::Config map(PLANNER_PRODUCT_CONFIG_PATH);
  navigation_world_model::WorldGeometry world_geometry;
  world_geometry.evidence_resolution_m = map.resolution;
  world_geometry.inflated_resolution_m = map.inflation_resolution;
  world_geometry.occupied_inflation_radius_m =
      map.inflation_resolution * map.inflation_step;
  world_geometry.local_size_m = Eigen::Vector3d{110.0, 15.0, 6.0};
  world_geometry.effective_virtual_ground_m = -10.0;
  world_geometry.effective_virtual_ceiling_m = 10.0;
  planner.bindWorldGeometry(world_geometry);

  const double visibility_horizon = planner.sensing_horizon_m > 0.0
      ? std::min(planner.sensing_horizon_m, planner.visibility_horizon_m)
      : planner.visibility_horizon_m;
  const double required_horizon =
      navigation_planning_backend::jerkLimitedStopDistance(
          planner.exp_traj_cfg.max_vel, planner.back_traj_cfg.max_acc,
          planner.back_traj_cfg.max_jerk) +
      2.0 * planner.exp_traj_cfg.max_vel * planner.replan_forward_dt_s +
      planner.robot_r;
  EXPECT_GE(visibility_horizon, required_horizon);
  EXPECT_GE(map.inflation_resolution * map.inflation_step, planner.robot_r);
  EXPECT_LE(planner.astar_search_time_limit_s, planner.replan_forward_dt_s * 0.25);
  EXPECT_GE(planner.astar_total_time_limit_s, planner.astar_search_time_limit_s);
  EXPECT_LT(planner.astar_total_time_limit_s, planner.solve_deadline_s);
  EXPECT_LE(planner.solve_deadline_s, planner.replan_forward_dt_s);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.corridor_plane_tolerance_m, 0.01);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.route_reference_lateral_weight, 1.0);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.route_reference_vertical_weight, 1.0);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.route_reference_lateral_deadband_m, 0.05);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.route_reference_vertical_deadband_m, 0.05);
  EXPECT_GT(planner.exp_traj_cfg.feasibility_retry_max_iterations, 0);
  EXPECT_DOUBLE_EQ(planner.back_traj_cfg.corridor_plane_tolerance_m, 0.01);
}

TEST(PlannerProductConfig, MissionLimitsLowerButNeverRaiseProductEnvelope) {
  const navigation_planning::DynamicLimits mission{7.0, 5.0, 12.0};
  navigation_planning_backend::Config planner(PLANNER_PRODUCT_CONFIG_PATH, mission);
  navigation_world_model::WorldGeometry world_geometry;
  world_geometry.evidence_resolution_m = 0.2;
  world_geometry.inflated_resolution_m = 0.2;
  world_geometry.occupied_inflation_radius_m = 1.0;
  world_geometry.local_size_m = Eigen::Vector3d{110.0, 15.0, 6.0};
  world_geometry.effective_virtual_ground_m = -10.0;
  world_geometry.effective_virtual_ceiling_m = 10.0;
  planner.bindWorldGeometry(world_geometry);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.max_vel, 7.0);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.max_acc, 5.0);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.max_jerk, 12.0);
  EXPECT_DOUBLE_EQ(planner.back_traj_cfg.max_vel, 7.0);
  EXPECT_DOUBLE_EQ(planner.visibility_horizon_m, 14.0);
  EXPECT_THROW(
      (navigation_planning_backend::Config(
          PLANNER_PRODUCT_CONFIG_PATH,
          navigation_planning::DynamicLimits{12.1, 5.0, 12.0})),
      std::invalid_argument);
}

TEST(PlannerProductConfig, RejectsMapThatCannotContainConfiguredPlanningHorizon) {
  navigation_planning_backend::Config planner(PLANNER_PRODUCT_CONFIG_PATH);
  navigation_world_model::WorldGeometry world_geometry;
  world_geometry.evidence_resolution_m = 0.2;
  world_geometry.inflated_resolution_m = 0.2;
  world_geometry.occupied_inflation_radius_m = 1.0;
  world_geometry.local_size_m = Eigen::Vector3d{10.0, 10.0, 6.0};
  world_geometry.effective_virtual_ground_m = -10.0;
  world_geometry.effective_virtual_ceiling_m = 10.0;
  EXPECT_THROW(planner.bindWorldGeometry(world_geometry), std::invalid_argument);
}

TEST(PlannerCorridorPlanes, NormalizesFinitePlanesAndRejectsMalformedNormals) {
  navigation_math::PolyhedronH planes(2, 4);
  planes << 2.0, 0.0, 0.0, 4.0,
            0.0, -3.0, 0.0, 6.0;
  ASSERT_TRUE(navigation_planning_backend::normalizeCorridorPlanes(planes));
  EXPECT_DOUBLE_EQ(planes(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(planes(0, 3), 2.0);
  EXPECT_DOUBLE_EQ(planes(1, 1), -1.0);
  EXPECT_DOUBLE_EQ(planes(1, 3), 2.0);

  navigation_math::PolyhedronH small_scale = planes * 1.0e-12;
  ASSERT_TRUE(navigation_planning_backend::normalizeCorridorPlanes(small_scale));
  EXPECT_TRUE(small_scale.isApprox(planes, 1.0e-12));

  navigation_math::PolyhedronH large_scale = planes * 1.0e12;
  ASSERT_TRUE(navigation_planning_backend::normalizeCorridorPlanes(large_scale));
  EXPECT_TRUE(large_scale.isApprox(planes, 1.0e-12));

  for (const double invalid_normal : {
           std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::infinity(),
           0.0,
       }) {
    navigation_math::PolyhedronH invalid(1, 4);
    invalid << invalid_normal, 0.0, 0.0, 1.0;
    EXPECT_FALSE(navigation_planning_backend::normalizeCorridorPlanes(invalid));
  }

  navigation_math::PolyhedronH invalid_offset(1, 4);
  invalid_offset << 1.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(navigation_planning_backend::normalizeCorridorPlanes(invalid_offset));
  EXPECT_DOUBLE_EQ(invalid_offset(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(invalid_offset(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(invalid_offset(0, 2), 0.0);
  EXPECT_TRUE(std::isnan(invalid_offset(0, 3)));

  navigation_math::PolyhedronH overflow(1, 4);
  overflow << 1.0e-300, 0.0, 0.0, 1.0e308;
  const auto overflow_before = overflow;
  EXPECT_FALSE(navigation_planning_backend::normalizeCorridorPlanes(overflow));
  EXPECT_TRUE(overflow.isApprox(overflow_before));

  navigation_math::PolyhedronH empty;
  EXPECT_FALSE(navigation_planning_backend::normalizeCorridorPlanes(empty));

  navigation_math::MatDf wrong_shape(1, 3);
  wrong_shape.setOnes();
  EXPECT_FALSE(navigation_planning_backend::normalizeCorridorPlanes(wrong_shape));
}

TEST(PlannerCorridorPlanes, GeometricCertificateIsIndependentOfPenaltyWeight) {
  navigation_math::PolyhedronH planes(1, 4);
  planes << 1.0, 0.0, 0.0, -0.4;
  ASSERT_TRUE(navigation_planning_backend::normalizeCorridorPlanes(planes));

  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 1.0;
  geometry_utils::Trajectory trajectory({1.0}, {coefficients});

  EXPECT_NEAR(
      navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
          trajectory, planes),
      0.6, 1.0e-12);

  navigation_math::PolyhedronH rescaled = planes * 1.0e8;
  ASSERT_TRUE(navigation_planning_backend::normalizeCorridorPlanes(rescaled));
  EXPECT_NEAR(
      navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
          trajectory, rescaled),
      0.6, 1.0e-12);
}

TEST(PlannerCorridorPlanes, CertificateChecksPolynomialExtremaBetweenSamples) {
  navigation_math::PolyhedronH planes(1, 4);
  planes << 1.0, 0.0, 0.0, -0.4;
  ASSERT_TRUE(navigation_planning_backend::normalizeCorridorPlanes(planes));

  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  // x(t) = 2t - 2t^2: endpoints are clear, but x(0.5) = 0.5.
  coefficients(0, 6) = 2.0;
  coefficients(0, 5) = -2.0;
  geometry_utils::Trajectory trajectory({1.0}, {coefficients});

  EXPECT_NEAR(
      navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
          trajectory, planes),
      0.1, 1.0e-10);
}

TEST(PlannerBackupBraking, UsesJerkLimitedTriangularAndTrapezoidalProfiles) {
  EXPECT_NEAR(navigation_planning_backend::jerkLimitedStopTime(0.5, 2.0, 8.0), 0.5, 1.0e-12);
  EXPECT_NEAR(navigation_planning_backend::jerkLimitedStopTime(2.0, 2.0, 8.0), 1.25, 1.0e-12);
  EXPECT_NEAR(navigation_planning_backend::jerkLimitedStopDistance(2.0, 2.0, 8.0), 1.25,
              1.0e-12);
}

TEST(PlannerBackupBraking, MinimumSnapSeedPreservesPVAJAndStopsWithinBounds) {
  navigation_math::StatePVAJ initial;
  initial.setZero();
  initial.col(0) << 1.0, -2.0, 3.0;
  initial.col(1) << 2.0, 0.25, -0.1;
  initial.col(2) << 0.2, -0.1, 0.05;
  initial.col(3) << 0.1, 0.05, -0.02;

  const auto seed = navigation_planning_backend::makeBackupBrakingSeed(
      0.6, initial, 3.0, 2.0, 8.0, 0.05, 0.05);
  ASSERT_TRUE(seed.feasible);
  ASSERT_GT(seed.duration_s, 0.0);
  const auto piece = navigation_planning_backend::minimumSnapStopPiece(initial, seed.duration_s);
  const auto control_points =
      navigation_planning_backend::minimumSnapStopBezierControlPoints(initial, seed.duration_s);
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

TEST(PlannerBackupBraking, RejectsNonFiniteState) {
  navigation_math::StatePVAJ initial;
  initial.setZero();
  initial(0, 1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(navigation_planning_backend::makeBackupBrakingSeed(
      0.0, initial, 3.0, 2.0, 8.0, 0.05, 0.05).feasible);
}

TEST(PlannerBackupBraking, PreservesMeasuredOverspeedWithoutIncreasingIt) {
  navigation_math::StatePVAJ initial = navigation_math::StatePVAJ::Zero();
  initial.col(0) << 2.0, -1.0, 3.0;
  initial.col(1) << 13.0, 0.0, 0.0;
  initial.col(2).setZero();
  initial.col(3).setZero();

  const auto seed = navigation_planning_backend::makeBackupBrakingSeed(
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

  const auto piece = navigation_planning_backend::minimumSnapStopPiece(initial, seed.duration_s);
  EXPECT_TRUE(piece.getState(0.0).isApprox(initial, 1.0e-9));
  EXPECT_NEAR(piece.getVel(seed.duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR(piece.getAcc(seed.duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR(piece.getJer(seed.duration_s).norm(), 0.0, 1.0e-8);
}

TEST(PlannerBackupBraking, RefinementDurationCannotUndercutCertifiedSeed) {
  EXPECT_TRUE(navigation_planning_backend::refinementDurationRespectsCertifiedFloor(2.0, 2.0));
  EXPECT_TRUE(navigation_planning_backend::refinementDurationRespectsCertifiedFloor(2.5, 2.0));
  EXPECT_FALSE(navigation_planning_backend::refinementDurationRespectsCertifiedFloor(1.99, 2.0));
  EXPECT_FALSE(navigation_planning_backend::refinementDurationRespectsCertifiedFloor(
      std::numeric_limits<double>::quiet_NaN(), 2.0));
}
