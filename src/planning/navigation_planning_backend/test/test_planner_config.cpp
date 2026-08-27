#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <rog_map/rog_map_core/config.hpp>
#include <planner_core/backup_braking.hpp>
#include <planner_core/config.hpp>
#include <planner_core/corridor_plane_validation.hpp>
#include <planner_core/kinematic_state_boundary.hpp>
#include <planner_core/pass_through_terminal_velocity.hpp>
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
  EXPECT_EQ(planner.exp_traj_cfg.pos_constraint_type, traj_opt::CORRIDOR);
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
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.route_reference_vertical_weight, 10.0);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.route_reference_lateral_deadband_m, 0.05);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.route_reference_vertical_deadband_m, 0.05);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.optimization_dynamic_reserve_ratio, 0.98);
  EXPECT_DOUBLE_EQ(planner.exp_traj_cfg.dynamic_limit_tolerance_ratio, 0.0);
  EXPECT_DOUBLE_EQ(planner.back_traj_cfg.dynamic_limit_tolerance_ratio, 0.0);
  EXPECT_TRUE(planner.preserve_backup_altitude);
  EXPECT_GT(planner.exp_traj_cfg.feasibility_retry_max_iterations, 0);
  EXPECT_DOUBLE_EQ(planner.back_traj_cfg.corridor_plane_tolerance_m, 0.01);
}

TEST(PlannerProductConfig, RejectsDynamicLimitToleranceThatWeakensPhysicalEnvelope) {
  EXPECT_NO_THROW(traj_opt::validateDynamicLimitToleranceRatio(0.0));
  EXPECT_THROW(traj_opt::validateDynamicLimitToleranceRatio(0.01), std::invalid_argument);
  EXPECT_THROW(
      traj_opt::validateDynamicLimitToleranceRatio(std::numeric_limits<double>::infinity()),
      std::invalid_argument);
}

TEST(PlannerPassThrough, UsesBoundedOutgoingTerminalVelocity) {
  const auto terminal_velocity = navigation_planning_backend::passThroughTerminalVelocity(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{9.0, 0.0, 0.0}, 5.0, 2.0);
  ASSERT_TRUE(terminal_velocity.has_value());
  EXPECT_NEAR(terminal_velocity->x(), std::sqrt(18.0), 1.0e-12);
  EXPECT_DOUBLE_EQ(terminal_velocity->y(), 0.0);
  EXPECT_DOUBLE_EQ(terminal_velocity->z(), 0.0);

  const auto capped_velocity = navigation_planning_backend::passThroughTerminalVelocity(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{100.0, 0.0, 0.0}, 5.0, 2.0);
  ASSERT_TRUE(capped_velocity.has_value());
  EXPECT_DOUBLE_EQ(capped_velocity->norm(), 5.0);

  EXPECT_FALSE(navigation_planning_backend::passThroughTerminalVelocity(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 5.0, 2.0));
}

TEST(PlannerPassThrough, LimitsOrthogonalVelocityChangeByTransitionTime) {
  const Eigen::Vector3d incoming_velocity{5.0, 0.0, 0.0};
  const auto terminal_velocity =
      navigation_planning_backend::passThroughTerminalVelocity(
          Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 10.0, 0.0},
          incoming_velocity, 2.0, 5.0, 2.0, 4.0);
  ASSERT_TRUE(terminal_velocity.has_value());

  const double expected_delta =
      navigation_planning_backend::passThroughMaximumVelocityChange(
          2.0, 2.0, 4.0);
  EXPECT_NEAR((*terminal_velocity - incoming_velocity).norm(), expected_delta,
              1.0e-12);
  EXPECT_GT(terminal_velocity->y(), 0.0);
  EXPECT_GT(terminal_velocity->x(), 0.0);
  EXPECT_LE(terminal_velocity->norm(), 5.0);
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

TEST(PlannerProductConfig, RejectsTallOnlyMapWithoutHorizontalPlanningExtent) {
  navigation_planning_backend::Config planner(PLANNER_PRODUCT_CONFIG_PATH);
  navigation_world_model::WorldGeometry world_geometry;
  world_geometry.evidence_resolution_m = 0.2;
  world_geometry.inflated_resolution_m = 0.2;
  world_geometry.occupied_inflation_radius_m = 1.0;
  world_geometry.local_size_m = Eigen::Vector3d{10.0, 10.0, 40.0};
  world_geometry.effective_virtual_ground_m = -10.0;
  world_geometry.effective_virtual_ceiling_m = 30.0;
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

TEST(PlannerBackupBraking, PreservesTerminalAltitudeWithoutBreakingPVAJStop) {
  navigation_math::StatePVAJ initial = navigation_math::StatePVAJ::Zero();
  initial.col(0) << 1.0, -2.0, 2.4;
  initial.col(1) << 1.0, 0.0, 0.0;
  initial.col(2) << 0.1, -0.05, 0.0;
  initial.col(3) << 0.02, 0.01, 0.0;

  constexpr double duration_s = 2.0;
  constexpr double target_altitude_m = 3.0;
  const auto piece =
      navigation_planning_backend::minimumSnapStopPieceWithTerminalAltitude(
          initial, duration_s, target_altitude_m);
  const auto control_points =
      navigation_planning_backend::minimumSnapStopBezierControlPoints(piece);

  EXPECT_TRUE(piece.getState(0.0).isApprox(initial, 1.0e-9));
  EXPECT_NEAR(piece.getPos(duration_s).z(), target_altitude_m, 1.0e-9);
  EXPECT_NEAR(piece.getVel(duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR(piece.getAcc(duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR(piece.getJer(duration_s).norm(), 0.0, 1.0e-8);
  EXPECT_TRUE(control_points.col(0).isApprox(initial.col(0), 1.0e-9));
  EXPECT_NEAR(control_points.col(7).z(), target_altitude_m, 1.0e-9);
  EXPECT_TRUE(piece.getMaxVelRate() < 3.0);
  EXPECT_TRUE(piece.getMaxAccRate() < 2.0);
  EXPECT_TRUE(piece.getMaxJerRate() < 8.0);
}

TEST(PlannerBackupBraking, ExtendsCertifiedSeedWhenAltitudeNeedsMoreTime) {
  navigation_math::StatePVAJ initial = navigation_math::StatePVAJ::Zero();
  initial.col(0) << 0.0, 0.0, 2.0;
  initial.col(1) << 2.0, 0.0, 0.0;

  const auto seed =
      navigation_planning_backend::makeBackupBrakingSeedWithTerminalAltitude(
          0.0, initial, 3.0, 2.0, 8.0, 0.05, 0.0, 3.0);
  ASSERT_TRUE(seed.feasible);
  ASSERT_TRUE(seed.terminal_altitude_preserved);
  EXPECT_NEAR(seed.endpoint.z(), 3.0, 1.0e-9);
  EXPECT_LE(seed.maximum_velocity_mps, seed.allowed_peak_velocity_mps);
  EXPECT_LE(seed.maximum_acceleration_mps2, 2.0);
  EXPECT_LE(seed.maximum_jerk_mps3, 8.0);
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

TEST(PlannerKinematicStateBoundary, BoundsOnlyEstimatedHighOrderDerivatives) {
  const Eigen::Vector3d raw{0.0, 40.0, 0.0};
  const auto bounded = navigation_planning_backend::boundEstimatedDerivative(
      raw, true, 30.0);
  EXPECT_NEAR(bounded.norm(), 30.0, 1.0e-12);
  EXPECT_TRUE(bounded.isApprox(Eigen::Vector3d{0.0, 30.0, 0.0}, 1.0e-12));

  const auto measured = navigation_planning_backend::boundEstimatedDerivative(
      raw, false, 30.0);
  EXPECT_TRUE(measured.isApprox(raw, 1.0e-12));

  const auto already_bounded = navigation_planning_backend::boundEstimatedDerivative(
      Eigen::Vector3d{0.0, 2.0, 0.0}, true, 30.0);
  EXPECT_TRUE(already_bounded.isApprox(Eigen::Vector3d{0.0, 2.0, 0.0}, 1.0e-12));

  navigation_math::StatePVAJ initial = navigation_math::StatePVAJ::Zero();
  initial.col(1) << 0.0, 2.0, 0.0;
  initial.col(3) = raw;
  EXPECT_FALSE(navigation_planning_backend::makeBackupBrakingSeed(
      0.0, initial, 3.0, 5.0, 30.0, 0.05, 0.0).feasible);
  initial.col(3) = bounded;
  const auto bounded_seed = navigation_planning_backend::makeBackupBrakingSeed(
      0.0, initial, 3.0, 5.0, 30.0, 0.05, 0.0);
  EXPECT_TRUE(bounded_seed.feasible);
}
