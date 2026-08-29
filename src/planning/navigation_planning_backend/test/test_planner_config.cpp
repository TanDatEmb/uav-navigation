#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <rog_map/rog_map_core/config.hpp>
#include <planner_core/backup_braking.hpp>
#include <planner_core/boundary_velocity_recovery.hpp>
#include <planner_core/hot_replan_recovery.hpp>
#include <planner_core/config.hpp>
#include <planner_core/corridor_plane_validation.hpp>
#include <planner_core/guide_vertical_envelope.hpp>
#include <planner_core/kinematic_state_boundary.hpp>
#include <planner_core/pass_through_terminal_velocity.hpp>
#include <planner_core/route_boundary_timing.hpp>
#include <navigation_planning/planning_limits.hpp>
#include <utils/optimization/optimization_utils.h>

TEST(PlannerDynamicLimits, BoundaryAccountingIsUlpsOnly) {
  const double limit = 3.0;
  EXPECT_TRUE(navigation_planning::withinNumericalDynamicLimit(
      std::nextafter(limit, std::numeric_limits<double>::infinity()), limit));
  EXPECT_TRUE(navigation_planning::withinNumericalDynamicLimit(
      limit + 32.0 * std::numeric_limits<double>::epsilon() * limit, limit));
  EXPECT_FALSE(navigation_planning::withinNumericalDynamicLimit(
      limit + 1.0e-9, limit));
  EXPECT_FALSE(navigation_planning::withinNumericalDynamicLimit(
      std::numeric_limits<double>::infinity(), limit));
}

TEST(TrajOptConfig, RejectsMalformedDirectOptimizerConfiguration) {
  traj_opt::Config config;
  EXPECT_THROW(config.validate(), std::invalid_argument);

  // A valid loaded configuration can still be modified by a direct caller;
  // the optimizer constructor must not trust the YAML-loader validation that
  // happened before this mutation.
  config.mass = 1.0;
  config.grav = 9.81;
  config.v_eps = 1.0e-4;
  config.max_vel = 8.0;
  config.max_acc = 4.0;
  config.max_jerk = 12.0;
  config.max_omg = 5.0;
  config.max_acc_thr = 25.0;
  config.min_acc_thr = 6.0;
  config.integral_reso = 10;
  config.smooth_eps = 0.01;
  config.opt_accuracy = 1.0e-5;
  config.pos_constraint_type = traj_opt::CORRIDOR;
  config.dh = 0.35;
  config.dv = 0.35;
  config.cp = 0.001;
  config.validate();

  config.pos_constraint_type = 99;
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config.pos_constraint_type = traj_opt::CORRIDOR;
  config.max_omg = std::numeric_limits<double>::infinity();
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config.max_omg = 5.0;
  config.min_acc_thr = 26.0;
  EXPECT_THROW(config.validate(), std::invalid_argument);

  config.min_acc_thr = 6.0;
  config.integral_reso = traj_opt::Config::kMaximumIntegralResolution + 1;
  EXPECT_THROW(config.validate(), std::invalid_argument);
  config.integral_reso = 10;
  config.feasibility_retry_max_iterations =
      traj_opt::Config::kMaximumFeasibilityRetryIterations + 1;
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(GuideVerticalEnvelope, UsesOneInflatedVoxelAroundCertifiedGuide) {
  navigation_math::vec_Vec3f guide{
      navigation_math::Vec3f{0.0, 0.0, 3.0},
      navigation_math::Vec3f{5.0, 0.0, 3.4}};
  const auto envelope =
      navigation_planning_backend::deriveGuideVerticalEnvelope(guide, 0.2);
  ASSERT_TRUE(envelope.valid);
  EXPECT_DOUBLE_EQ(envelope.lower_z_m, 2.8);
  EXPECT_DOUBLE_EQ(envelope.upper_z_m, 3.6);
  EXPECT_DOUBLE_EQ(envelope.slack_m, 0.2);

  navigation_math::MatD4f wide_box(6, 4);
  wide_box << 1.0, 0.0, 0.0, -10.0,
             -1.0, 0.0, 0.0, -10.0,
              0.0, 1.0, 0.0, -10.0,
              0.0,-1.0, 0.0, -10.0,
              0.0, 0.0, 1.0, -10.0,
              0.0, 0.0,-1.0, -10.0;
  geometry_utils::Polytope polytope(wide_box);
  polytope.SetSeedLine({guide.front(), guide.back()});
  geometry_utils::PolytopeVec corridor{polytope};
  ASSERT_TRUE(navigation_planning_backend::applyGuideVerticalEnvelope(
      corridor, envelope));
  const auto bounded = corridor.front().GetPlanes();
  ASSERT_EQ(bounded.rows(), 8);
  EXPECT_DOUBLE_EQ(bounded(6, 2), 1.0);
  EXPECT_DOUBLE_EQ(bounded(6, 3), -3.6);
  EXPECT_DOUBLE_EQ(bounded(7, 2), -1.0);
  EXPECT_DOUBLE_EQ(bounded(7, 3), 2.8);
}

TEST(GuideVerticalEnvelope, FollowsEachGuideSegmentAndPreservesOverlap) {
  navigation_math::vec_Vec3f guide{
      navigation_math::Vec3f{0.0, 0.0, 3.0},
      navigation_math::Vec3f{5.0, 0.0, 3.0},
      navigation_math::Vec3f{10.0, 0.0, 4.0}};
  const auto envelope =
      navigation_planning_backend::deriveGuideVerticalEnvelope(guide, 0.2);
  ASSERT_TRUE(envelope.valid);

  navigation_math::MatD4f wide_box(6, 4);
  wide_box << 1.0, 0.0, 0.0, -20.0,
             -1.0, 0.0, 0.0, -20.0,
              0.0, 1.0, 0.0, -20.0,
              0.0,-1.0, 0.0, -20.0,
              0.0, 0.0, 1.0, -20.0,
              0.0, 0.0,-1.0, -20.0;
  geometry_utils::Polytope flat(wide_box);
  flat.SetSeedLine({guide[0], guide[1]});
  geometry_utils::Polytope climbing(wide_box);
  climbing.SetSeedLine({guide[1], guide[2]});
  geometry_utils::PolytopeVec corridor{flat, climbing};
  ASSERT_TRUE(navigation_planning_backend::applyGuideVerticalEnvelope(
      corridor, envelope));

  const auto flat_planes = corridor[0].GetPlanes();
  const auto climbing_planes = corridor[1].GetPlanes();
  EXPECT_DOUBLE_EQ(flat_planes(6, 3), -3.2);
  EXPECT_DOUBLE_EQ(flat_planes(7, 3), 2.8);
  EXPECT_DOUBLE_EQ(climbing_planes(6, 3), -4.2);
  EXPECT_DOUBLE_EQ(climbing_planes(7, 3), 2.8);
  EXPECT_GT(corridor[1].overlap_depth_with_last_one, 0.0);
  EXPECT_TRUE(corridor[1].interior_pt_with_last_one.allFinite());
}

TEST(GuideVerticalEnvelope, RejectsInvalidScaleOrGuide) {
  EXPECT_FALSE(navigation_planning_backend::deriveGuideVerticalEnvelope({}, 0.2).valid);
  navigation_math::vec_Vec3f guide{navigation_math::Vec3f::Zero()};
  EXPECT_FALSE(navigation_planning_backend::deriveGuideVerticalEnvelope(guide, 0.0).valid);
  guide.front().z() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(navigation_planning_backend::deriveGuideVerticalEnvelope(guide, 0.2).valid);
}

namespace {

geometry_utils::Trajectory makeLinearSpeedTrajectory(
    const double initial_speed_mps, const double acceleration_mps2,
    const double duration_s) {
  Eigen::Matrix<double, 3, 6> coefficients =
      Eigen::Matrix<double, 3, 6>::Zero();
  coefficients(0, 3) = 0.5 * acceleration_mps2;
  coefficients(0, 4) = initial_speed_mps;
  geometry_utils::Trajectory trajectory;
  trajectory.emplace_back(duration_s, coefficients);
  return trajectory;
}

}  // namespace

TEST(BoundaryVelocityRecovery, KeepsNormalStartsUnderMissionCap) {
  const auto report =
      navigation_planning_backend::certifyBoundaryVelocityRecovery(
          makeLinearSpeedTrajectory(2.9, 0.0, 1.0), 3.0, 2.0, 4.0);
  EXPECT_FALSE(report.initial_overspeed);
  EXPECT_TRUE(report.satisfied);
  EXPECT_DOUBLE_EQ(report.allowed_peak_speed_mps, 3.0);
}

TEST(BoundaryVelocityRecovery, AcceptsBoundedJerkLimitedRecovery) {
  const auto report =
      navigation_planning_backend::certifyBoundaryVelocityRecovery(
          makeLinearSpeedTrajectory(3.1, -0.5, 1.0), 3.0, 2.0, 4.0);
  EXPECT_TRUE(report.initial_overspeed);
  EXPECT_TRUE(report.peak_bounded);
  EXPECT_TRUE(report.recovered_by_deadline);
  EXPECT_TRUE(report.satisfied);
  EXPECT_DOUBLE_EQ(report.allowed_peak_speed_mps, 3.1);
  EXPECT_LE(report.suffix_maximum_speed_mps, 3.0);
}

TEST(BoundaryVelocityRecovery, RejectsWorseningOrLateOverspeed) {
  const auto worsening =
      navigation_planning_backend::certifyBoundaryVelocityRecovery(
          makeLinearSpeedTrajectory(3.1, 0.1, 1.0), 3.0, 2.0, 4.0);
  EXPECT_FALSE(worsening.peak_bounded);
  EXPECT_FALSE(worsening.satisfied);

  const auto late =
      navigation_planning_backend::certifyBoundaryVelocityRecovery(
          makeLinearSpeedTrajectory(3.1, -0.05, 1.0), 3.0, 2.0, 4.0);
  EXPECT_TRUE(late.peak_bounded);
  EXPECT_FALSE(late.recovered_by_deadline);
  EXPECT_FALSE(late.satisfied);
}

TEST(HotReplanTrackingRecovery, NeverBuildsAReverseConnectorToMeasuredHistory) {
  using navigation_planning_backend::HotReplanTrackingRecovery;
  using navigation_planning_backend::classifyHotReplanTrackingRecovery;

  EXPECT_EQ(classifyHotReplanTrackingRecovery(false, true),
            HotReplanTrackingRecovery::kContinueHotStitch);
  EXPECT_EQ(classifyHotReplanTrackingRecovery(true, true),
            HotReplanTrackingRecovery::kRebaseCurrentSolveOnMeasuredState);
  EXPECT_EQ(classifyHotReplanTrackingRecovery(true, false),
            HotReplanTrackingRecovery::kFailClosed);
}

TEST(HotReplanTrackingRecovery, RejectsSpliceOutsideNecessaryKinematicEnvelope) {
  using navigation_planning_backend::assessHotReplanSpliceCompatibility;

  // Runtime artifact generation 246 was inside the current 0.25 m position
  // budget but its command and measured Y velocities differed by about
  // 0.737 m/s. A 2 m/s^2 vehicle cannot close that mismatch over the 0.2 s
  // splice prefix.
  const auto artifact_failure = assessHotReplanSpliceCompatibility(
      0.236, 0.383, 0.737, 0.2, 2.0, 0.25);
  EXPECT_TRUE(artifact_failure.finite);
  EXPECT_TRUE(artifact_failure.current_position_within_budget);
  EXPECT_FALSE(artifact_failure.future_position_within_envelope);
  EXPECT_FALSE(artifact_failure.future_velocity_within_envelope);
  EXPECT_TRUE(artifact_failure.requiresMeasuredStateRestart());
  EXPECT_DOUBLE_EQ(artifact_failure.future_position_allowance_m, 0.29);
  EXPECT_DOUBLE_EQ(artifact_failure.future_velocity_allowance_mps, 0.4);
}

TEST(HotReplanTrackingRecovery, KeepsSpliceInsideNecessaryKinematicEnvelope) {
  using navigation_planning_backend::assessHotReplanSpliceCompatibility;

  // Passing these two scalar projections only means that this guard has no
  // reason to force a restart. It is not a joint P/V reachability proof: the
  // same bounded acceleration history must produce both boundary conditions,
  // which remains owned by the downstream continuous dynamics certificate.
  const auto compatible = assessHotReplanSpliceCompatibility(
      0.25, 0.29, 0.4, 0.2, 2.0, 0.25);
  EXPECT_TRUE(compatible.finite);
  EXPECT_TRUE(compatible.current_position_within_budget);
  EXPECT_TRUE(compatible.future_position_within_envelope);
  EXPECT_TRUE(compatible.future_velocity_within_envelope);
  EXPECT_FALSE(compatible.requiresMeasuredStateRestart());
}

TEST(HotReplanTrackingRecovery, FailsClosedOnInvalidCompatibilityInputs) {
  using navigation_planning_backend::assessHotReplanSpliceCompatibility;

  const auto invalid = assessHotReplanSpliceCompatibility(
      0.0, std::numeric_limits<double>::quiet_NaN(), 0.0,
      0.2, 2.0, 0.25);
  EXPECT_FALSE(invalid.finite);
  EXPECT_TRUE(invalid.requiresMeasuredStateRestart());
}

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

TEST(PlannerDurationParameterization, ReserveScaleIsNotAddedTwice) {
  navigation_math::VecDf nominal_duration_s(3);
  nominal_duration_s << 0.5, 1.0, 2.0;
  for (const double scale : {1.05, 1.5, 2.0, 4.0}) {
    const navigation_math::VecDf target_duration_s = nominal_duration_s * scale;
    navigation_math::VecDf tau_storage(3);
    using MappedVector = Eigen::Map<Eigen::VectorXd>;
    MappedVector tau(tau_storage.data(), 3);
    optimization_utils::Gcopter<MappedVector>::backwardMapTToTau(
        target_duration_s, tau);
    navigation_math::VecDf reconstructed_free_duration_s;
    optimization_utils::Gcopter<MappedVector>::forwardMapTauToT(
        tau, reconstructed_free_duration_s);
    ASSERT_TRUE(reconstructed_free_duration_s.allFinite());
    EXPECT_TRUE(reconstructed_free_duration_s.isApprox(target_duration_s, 1.0e-12));
    EXPECT_TRUE(reconstructed_free_duration_s.isApprox(nominal_duration_s * scale,
                                                       1.0e-12));
  }
}

TEST(PlannerProductConfig, SatisfiesVisibilityInflationAndReplanBudgets) {
  navigation_planning_backend::Config planner(PLANNER_PRODUCT_CONFIG_PATH);
  EXPECT_EQ(planner.exp_traj_cfg.pos_constraint_type, traj_opt::CORRIDOR);
  EXPECT_EQ(planner.unknown_space_policy,
            navigation_world_model::UnknownPolicy::kRequireKnownFree);
  const rog_map::Config map(PLANNER_PRODUCT_CONFIG_PATH);
  // ROG-Map rounds each dimension up to an odd voxel count after inflation;
  // assert the product contract without depending on the one-cell padding.
  EXPECT_GE(map.map_size_d.x(), 110.0);
  EXPECT_GE(map.map_size_d.y(), 30.0);
  EXPECT_GE(map.map_size_d.z(), 6.0);
  EXPECT_LE(map.map_size_d.x(), 110.0 + map.resolution + 1.0e-9);
  EXPECT_LE(map.map_size_d.y(), 30.0 + map.resolution + 1.0e-9);
  EXPECT_LE(map.map_size_d.z(), 6.0 + map.resolution + 1.0e-9);
  navigation_world_model::WorldGeometry world_geometry;
  world_geometry.evidence_resolution_m = map.resolution;
  world_geometry.inflated_resolution_m = map.inflation_resolution;
  world_geometry.occupied_inflation_radius_m =
      map.inflation_resolution * map.inflation_step;
  world_geometry.local_size_m = map.map_size_d.cast<double>();
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
  EXPECT_DOUBLE_EQ(planner.yaw_tracking_error_budget_rad, 0.35);
  EXPECT_GT(planner.exp_traj_cfg.feasibility_retry_max_iterations, 0);
  EXPECT_EQ(planner.exp_traj_cfg.lbfgs_memory_size, 32);
  EXPECT_EQ(planner.back_traj_cfg.lbfgs_memory_size, 32);
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

  const auto interior_velocity =
      navigation_planning_backend::passThroughTerminalVelocity(
          Eigen::Vector3d::Zero(), Eigen::Vector3d{100.0, 0.0, 0.0},
          5.0 * 0.98, 2.0);
  ASSERT_TRUE(interior_velocity.has_value());
  EXPECT_DOUBLE_EQ(interior_velocity->norm(), 4.9);
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

TEST(PlannerPassThrough, FrontierKeepsCurrentGuideTangent) {
  const auto terminal_velocity =
      navigation_planning_backend::frontierContinuationVelocity(
          Eigen::Vector3d{10.0, 0.0, 0.0},
          Eigen::Vector3d{5.0, 0.0, 0.0},
          Eigen::Vector3d{3.0, 0.0, 0.0},
          3.0, 5.0, 2.0, 4.0, 0.2);
  ASSERT_TRUE(terminal_velocity.has_value());
  EXPECT_NEAR(terminal_velocity->x(), 3.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(terminal_velocity->y(), 0.0);
  EXPECT_DOUBLE_EQ(terminal_velocity->z(), 0.0);
}

TEST(PlannerPassThrough, FrontierDoesNotTurnTowardFutureCorner) {
  const auto terminal_velocity =
      navigation_planning_backend::frontierContinuationVelocity(
          Eigen::Vector3d{10.0, 0.0, 0.0},
          Eigen::Vector3d{5.0, 0.0, 0.0},
          Eigen::Vector3d{3.0, 0.0, 0.0},
          3.0, 5.0, 2.0, 4.0, 0.2);
  ASSERT_TRUE(terminal_velocity.has_value());
  EXPECT_GT(terminal_velocity->x(), 0.0);
  EXPECT_DOUBLE_EQ(terminal_velocity->y(), 0.0);
}

TEST(PlannerPassThrough, FrontierRejectsDegenerateGuideTail) {
  EXPECT_FALSE(navigation_planning_backend::frontierContinuationVelocity(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d{3.0, 0.0, 0.0}, 3.0, 5.0, 2.0, 4.0, 0.2));
}

TEST(PlannerPassThrough, ShortCornerBoundaryRequiresLowerIncomingTerminalSpeed) {
  const Eigen::Vector3d incoming{0.0, -2.7, 0.0};
  const Eigen::Vector3d tangent{0.0, -1.0, 0.0};
  const double path_length = 1.99;
  const double duration = 1.04;
  const double terminal_cap =
      navigation_planning_backend::terminalSpeedCapForPath(
          path_length, duration, tangent.dot(incoming), 2.94);
  EXPECT_NEAR(terminal_cap, 1.126923076923077, 1.0e-12);
  EXPECT_LT(terminal_cap, incoming.norm());
}

TEST(PlannerPassThrough, CornerTerminalSpeedUsesAcceptanceRoomEnvelope) {
  const double cap = navigation_planning_backend::passThroughCornerSpeedCap(
      0.9, 2.0, 2.94);
  EXPECT_NEAR(cap, std::sqrt(1.8), 1.0e-12);
  EXPECT_LT(cap, 2.94);
  EXPECT_DOUBLE_EQ(
      navigation_planning_backend::passThroughCornerSpeedCap(0.9, 2.0, 1.0),
      1.0);
  EXPECT_DOUBLE_EQ(
      navigation_planning_backend::passThroughCornerSpeedCap(0.0, 2.0, 3.0),
      0.0);
}

TEST(PlannerPassThrough, RouteWindowMovesEndpointAlongOutgoingCornerTangent) {
  const auto endpoint = navigation_planning_backend::passThroughRouteWindowEndpoint(
      Eigen::Vector3d{50.0, 5.0, 3.0}, Eigen::Vector3d{50.0, -5.0, 3.0},
      Eigen::Vector3d{30.0, 0.0, 0.0}, 0.9, 0.2);
  ASSERT_TRUE(endpoint.has_value());
  EXPECT_NEAR(endpoint->x(), 50.0, 1.0e-12);
  EXPECT_NEAR(endpoint->y(), 4.325, 1.0e-12);
  EXPECT_NEAR((*endpoint - Eigen::Vector3d{50.0, 5.0, 3.0}).norm(), 0.675, 1.0e-12);

  const auto window = navigation_planning_backend::passThroughRouteWindow(
      Eigen::Vector3d{50.0, 5.0, 3.0}, Eigen::Vector3d{50.0, -5.0, 3.0},
      Eigen::Vector3d{30.0, 0.0, 0.0}, 0.9, 0.2);
  ASSERT_TRUE(window.has_value());
  EXPECT_NEAR(window->entry.x(), 49.325, 1.0e-12);
  EXPECT_NEAR(window->entry.y(), 5.0, 1.0e-12);
  EXPECT_NEAR(window->outgoing_blend.x(), 50.0, 1.0e-12);
  EXPECT_NEAR(window->outgoing_blend.y(), 4.6625, 1.0e-12);
  EXPECT_NEAR(window->endpoint.y(), 4.325, 1.0e-12);
  const double fillet_radius =
      (window->endpoint - Eigen::Vector3d{50.0, 5.0, 3.0}).norm();
  EXPECT_NEAR(
      navigation_planning_backend::passThroughCornerSpeedCap(
          fillet_radius, 2.0, 2.94),
      std::sqrt(1.35), 1.0e-12);
}

TEST(PlannerPassThrough, RouteWindowDoesNotChangeShallowBendOrTinyAcceptanceBall) {
  EXPECT_FALSE(navigation_planning_backend::passThroughGenuineCorner(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{10.0, 1.0, 0.0},
      Eigen::Vector3d{1.0, 0.0, 0.0}));
  EXPECT_TRUE(navigation_planning_backend::passThroughGenuineCorner(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 10.0, 0.0},
      Eigen::Vector3d{1.0, 0.0, 0.0}));
  EXPECT_FALSE(navigation_planning_backend::passThroughRouteWindowEndpoint(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{10.0, 1.0, 0.0},
      Eigen::Vector3d{1.0, 0.0, 0.0}, 0.9, 0.2));
  EXPECT_FALSE(navigation_planning_backend::passThroughRouteWindowEndpoint(
      Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 10.0, 0.0},
      Eigen::Vector3d{1.0, 0.0, 0.0}, 0.2, 0.2));
}

TEST(PlannerPassThrough, TerminalSpeedCapRejectsImpossibleShortBoundary) {
  EXPECT_NEAR(
      navigation_planning_backend::terminalSpeedCapForPath(1.0, 1.0, 2.4, 2.94),
      0.0, 1.0e-12);
  EXPECT_NEAR(
      navigation_planning_backend::terminalSpeedCapForPath(20.0, 7.0, 2.4, 2.94),
      2.94, 1.0e-12);
}

TEST(PlannerPassThrough, LookaheadDistanceCoversStoppingAndReplanEnvelope) {
  const double lookahead = navigation_planning_backend::passThroughLookaheadDistance(
      3.0, 3.0, 2.0, 4.0, 0.2, 3.0, 10.0);
  EXPECT_NEAR(lookahead, 7.2, 1.0e-12);
  EXPECT_DOUBLE_EQ(
      navigation_planning_backend::passThroughLookaheadDistance(
          3.0, 3.0, 2.0, 4.0, 0.2, 3.0, 4.0),
      4.0);
  EXPECT_DOUBLE_EQ(
      navigation_planning_backend::passThroughLookaheadDistance(
          -1.0, 3.0, 2.0, 4.0, 0.2, 3.0, 10.0),
      0.0);
}

TEST(PlannerPassThrough, SeparatesRequiredAndCertifiedLookahead) {
  const double required = navigation_planning_backend::passThroughRequiredLookaheadDistance(
      3.0, 3.0, 2.0, 4.0, 0.2, 3.0);
  EXPECT_NEAR(required, 7.2, 1.0e-12);
  EXPECT_TRUE(navigation_planning_backend::passThroughLookaheadComplete(required, 7.2));
  EXPECT_FALSE(navigation_planning_backend::passThroughLookaheadComplete(required, 6.0));
  EXPECT_FALSE(navigation_planning_backend::passThroughLookaheadComplete(required, -1.0));
}

TEST(PlannerPassThrough, CruiseLookaheadDoesNotCollapseAtLowMeasuredSpeed) {
  const double cruise_window =
      navigation_planning_backend::passThroughCruiseLookaheadDistance(
          3.0, 2.0, 4.0, 0.2, 3.0);
  EXPECT_NEAR(cruise_window, 7.2, 1.0e-12);
  EXPECT_GT(
      cruise_window,
      navigation_planning_backend::passThroughRequiredLookaheadDistance(
          0.0, 3.0, 2.0, 4.0, 0.2, 3.0));
}

TEST(PlannerPassThrough, VisibilityPrefixCannotMasqueradeAsMissionBoundary) {
  const Eigen::Vector3d mission_waypoint{85.0, -5.0, 3.0};
  EXPECT_FALSE(navigation_planning_backend::passThroughGuideReachesMissionBoundary(
      Eigen::Vector3d{70.0, -5.0, 3.0}, mission_waypoint, 0.2));
  EXPECT_TRUE(navigation_planning_backend::passThroughGuideReachesMissionBoundary(
      Eigen::Vector3d{84.9, -5.0, 3.0}, mission_waypoint, 0.2));
  EXPECT_FALSE(navigation_planning_backend::passThroughGuideReachesMissionBoundary(
      Eigen::Vector3d::Constant(NAN), mission_waypoint, 0.2));
}

TEST(PlannerPassThrough, CollinearPassThroughLegStillUsesLookaheadEnvelope) {
  const Eigen::Vector3d waypoint{20.0, 5.0, 3.0};
  const Eigen::Vector3d next_target{50.0, 5.0, 3.0};
  const Eigen::Vector3d incoming_tangent{20.0, 0.0, 0.0};
  EXPECT_FALSE(navigation_planning_backend::passThroughGenuineCorner(
      waypoint, next_target, incoming_tangent));

  const double lookahead = navigation_planning_backend::passThroughLookaheadDistance(
      3.0, 3.0, 2.0, 4.0, 0.2, 3.0,
      (next_target - waypoint).norm());
  EXPECT_GT(lookahead, 0.0);
  EXPECT_LT(lookahead, (next_target - waypoint).norm());
}

TEST(PlannerPassThrough, GenuineCornerRetainsLongOutgoingLookahead) {
  const Eigen::Vector3d waypoint{50.0, 5.0, 3.0};
  const Eigen::Vector3d next_target{50.0, -5.0, 3.0};
  const Eigen::Vector3d incoming_tangent{30.0, 0.0, 0.0};
  ASSERT_TRUE(navigation_planning_backend::passThroughGenuineCorner(
      waypoint, next_target, incoming_tangent));

  const double required =
      navigation_planning_backend::passThroughRequiredLookaheadDistance(
          3.0, 3.0, 2.0, 4.0, 0.2, 3.0);
  EXPECT_NEAR(required, 7.2, 1.0e-12);
  EXPECT_TRUE(navigation_planning_backend::passThroughOutgoingLookaheadEligible(
      required, (next_target - waypoint).norm(), 15.0, 0.2));
  EXPECT_FALSE(navigation_planning_backend::passThroughOutgoingLookaheadEligible(
      required, (next_target - waypoint).norm(), 0.4, 0.2));
}

TEST(PlannerPassThrough, RouteBoundaryTimingSplitsDirectEndpointInterval) {
  EXPECT_DOUBLE_EQ(
      navigation_planning_backend::routeBoundaryJunctionTime(
          true, 2, 3, 1, 3.0, 10.0, 10.0),
      6.5);
  EXPECT_DOUBLE_EQ(
      navigation_planning_backend::routeBoundaryJunctionTime(
          true, 2, 4, 1, 3.0, 10.0, 4.0),
      4.0);
  EXPECT_DOUBLE_EQ(
      navigation_planning_backend::routeBoundaryJunctionTime(
          false, 2, 3, 1, 3.0, 10.0, 10.0),
      10.0);
}

TEST(PlannerPassThrough, RepeatedGuideTimesUseNeighbouringTimeAnchors) {
  std::vector<double> interior_plateau{0.0, 3.0, 3.0, 3.0, 10.0};
  ASSERT_TRUE(navigation_planning_backend::spreadRepeatedGuideJunctionTimes(
      interior_plateau));
  EXPECT_DOUBLE_EQ(interior_plateau.front(), 0.0);
  EXPECT_DOUBLE_EQ(interior_plateau[1], 3.0);
  EXPECT_NEAR(interior_plateau[2], 16.0 / 3.0, 1.0e-12);
  EXPECT_NEAR(interior_plateau[3], 23.0 / 3.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(interior_plateau.back(), 10.0);

  std::vector<double> terminal_plateau{0.0, 3.0, 10.0, 10.0};
  ASSERT_TRUE(navigation_planning_backend::spreadRepeatedGuideJunctionTimes(
      terminal_plateau));
  EXPECT_DOUBLE_EQ(terminal_plateau.front(), 0.0);
  EXPECT_DOUBLE_EQ(terminal_plateau[1], 3.0);
  EXPECT_NEAR(terminal_plateau[2], 6.5, 1.0e-12);
  EXPECT_DOUBLE_EQ(terminal_plateau.back(), 10.0);

  std::vector<double> initial_plateau{0.0, 0.0, 0.0, 9.0};
  ASSERT_TRUE(navigation_planning_backend::spreadRepeatedGuideJunctionTimes(
      initial_plateau));
  EXPECT_DOUBLE_EQ(initial_plateau.front(), 0.0);
  EXPECT_NEAR(initial_plateau[1], 3.0, 1.0e-12);
  EXPECT_NEAR(initial_plateau[2], 6.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(initial_plateau.back(), 9.0);

  std::vector<double> decreasing{0.0, 3.0, 2.0, 10.0};
  EXPECT_FALSE(navigation_planning_backend::spreadRepeatedGuideJunctionTimes(
      decreasing));
  std::vector<double> degenerate{4.0, 4.0};
  EXPECT_FALSE(navigation_planning_backend::spreadRepeatedGuideJunctionTimes(
      degenerate));
}

TEST(PlannerProductConfig, MissionLimitsLowerButNeverRaiseProductEnvelope) {
  const navigation_planning::DynamicLimits mission{7.0, 5.0, 12.0};
  navigation_planning_backend::Config planner(PLANNER_PRODUCT_CONFIG_PATH, mission);
  const rog_map::Config map(PLANNER_PRODUCT_CONFIG_PATH);
  navigation_world_model::WorldGeometry world_geometry;
  world_geometry.evidence_resolution_m = 0.2;
  world_geometry.inflated_resolution_m = 0.2;
  world_geometry.occupied_inflation_radius_m = 1.0;
  world_geometry.local_size_m = map.map_size_d.cast<double>();
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

TEST(PlannerProductConfig, RejectsUnknownMissionSpacePolicy) {
  navigation_planning::DynamicLimits mission{7.0, 5.0, 12.0};
  mission.unknown_space_policy =
      static_cast<navigation_planning::UnknownSpacePolicy>(255U);
  EXPECT_THROW(
      (navigation_planning_backend::Config(PLANNER_PRODUCT_CONFIG_PATH, mission)),
      std::invalid_argument);
}

TEST(PlannerProductConfig, RejectsMapBelowThePlannerSafetyEnvelope) {
  navigation_planning_backend::Config planner(PLANNER_PRODUCT_CONFIG_PATH);
  navigation_world_model::WorldGeometry world_geometry;
  world_geometry.evidence_resolution_m = 0.2;
  world_geometry.inflated_resolution_m = 0.2;
  world_geometry.occupied_inflation_radius_m = 1.0;
  world_geometry.local_size_m = Eigen::Vector3d{1.0, 10.0, 6.0};
  world_geometry.effective_virtual_ground_m = -10.0;
  world_geometry.effective_virtual_ceiling_m = 10.0;
  EXPECT_THROW(planner.bindWorldGeometry(world_geometry), std::invalid_argument);
}

TEST(PlannerProductConfig, RejectsUnboundedSafetyNeighbourGeneration) {
  navigation_planning_backend::Config planner(PLANNER_PRODUCT_CONFIG_PATH);
  navigation_world_model::WorldGeometry world_geometry;
  world_geometry.evidence_resolution_m = 1.0e-300;
  world_geometry.inflated_resolution_m = 1.0e-300;
  world_geometry.occupied_inflation_radius_m = 1.0;
  world_geometry.local_size_m = Eigen::Vector3d{110.0, 15.0, 6.0};
  world_geometry.effective_virtual_ground_m = -10.0;
  world_geometry.effective_virtual_ceiling_m = 10.0;
  EXPECT_THROW(planner.bindWorldGeometry(world_geometry), std::invalid_argument);
}

TEST(PlannerProductConfig, DirectionalSupportUsesTheFirstAxisAlignedBoundary) {
  navigation_world_model::WorldGeometry world_geometry;
  world_geometry.local_center_m = Eigen::Vector3d::Zero();
  world_geometry.local_size_m = Eigen::Vector3d{110.0, 15.0, 6.0};

  const auto x_support = navigation_world_model::directionalSupportToLocalBoundary(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), world_geometry);
  const auto y_support = navigation_world_model::directionalSupportToLocalBoundary(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY(), world_geometry);
  const auto diagonal_support =
      navigation_world_model::directionalSupportToLocalBoundary(
          Eigen::Vector3d::Zero(), Eigen::Vector3d{1.0, 1.0, 0.0}, world_geometry);

  ASSERT_TRUE(x_support.has_value());
  ASSERT_TRUE(y_support.has_value());
  ASSERT_TRUE(diagonal_support.has_value());
  EXPECT_DOUBLE_EQ(*x_support, 55.0);
  EXPECT_DOUBLE_EQ(*y_support, 7.5);
  EXPECT_NEAR(*diagonal_support, 7.5 * std::sqrt(2.0), 1.0e-12);
  EXPECT_FALSE(navigation_world_model::directionalSupportToLocalBoundary(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), world_geometry));
  EXPECT_DOUBLE_EQ(
      *navigation_world_model::directionalSupportToLocalBoundary(
          Eigen::Vector3d{0.0, 20.0, 0.0}, Eigen::Vector3d::UnitY(), world_geometry),
      0.0);

  world_geometry.local_center_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::max());
  world_geometry.local_size_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::max());
  EXPECT_FALSE(navigation_world_model::directionalSupportToLocalBoundary(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), world_geometry));
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

TEST(PlannerKinematicStateBoundary, FreshRestPlanDropsEstimatedHighOrderDerivatives) {
  navigation_math::RobotState state;
  state.p << 1.0, 2.0, 3.0;
  state.v << 0.5, -0.25, 0.1;
  state.a << 1.0, 2.0, 3.0;
  state.j << -4.0, 5.0, -6.0;

  const auto boundary = navigation_planning_backend::makeCommandBoundaryPVAJ(
      state, true, true);
  EXPECT_TRUE(boundary.col(0).isApprox(state.p));
  EXPECT_TRUE(boundary.col(1).isApprox(state.v));
  EXPECT_TRUE(boundary.col(2).isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(boundary.col(3).isApprox(Eigen::Vector3d::Zero()));

  const auto measured_boundary =
      navigation_planning_backend::makeCommandBoundaryPVAJ(state, false, false);
  EXPECT_TRUE(measured_boundary.col(2).isApprox(state.a));
  EXPECT_TRUE(measured_boundary.col(3).isApprox(state.j));
}
