#include "planner_core/corridor_bezier_seed.hpp"
#include "planner_core/deterministic_nominal_seed.hpp"

#include <gtest/gtest.h>

namespace {

navigation_math::PolyhedronH box(
    const Eigen::Vector3d& minimum, const Eigen::Vector3d& maximum) {
  navigation_math::PolyhedronH planes(6, 4);
  planes <<
      1.0, 0.0, 0.0, -maximum.x(),
     -1.0, 0.0, 0.0,  minimum.x(),
      0.0, 1.0, 0.0, -maximum.y(),
      0.0,-1.0, 0.0,  minimum.y(),
      0.0, 0.0, 1.0, -maximum.z(),
      0.0, 0.0,-1.0,  minimum.z();
  return planes;
}

navigation_math::StatePVAJ state(
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero()) {
  navigation_math::StatePVAJ output = navigation_math::StatePVAJ::Zero();
  output.col(0) = position;
  output.col(1) = velocity;
  return output;
}

}  // namespace

TEST(CorridorBezierSeed, BuildsStraightC3BaselineInsideOverlappingCorridors) {
  navigation_math::PolyhedraH corridors{
      box({-1.0, -2.0, 0.0}, {11.0, 2.0, 6.0}),
      box({9.0, -2.0, 0.0}, {21.0, 2.0, 6.0})};
  navigation_math::Mat3Df junctions(3, 1);
  junctions.col(0) = Eigen::Vector3d{10.0, 0.0, 3.0};
  navigation_math::VecDf durations(2);
  durations << 4.0, 4.0;
  navigation_math::VecDi mapping(2);
  mapping << 0, 1;
  const auto result =
      navigation_planning_backend::buildCorridorContainedBezierSeed(
          state({0.0, 0.0, 3.0}, {2.0, 0.0, 0.0}),
          state({20.0, 0.0, 3.0}), junctions, durations, corridors, mapping,
          3.0, 1.0e-8);
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.trajectory.getPieceNum(), 2);
  EXPECT_NEAR((result.trajectory.getPos(0.0) -
               Eigen::Vector3d{0.0, 0.0, 3.0}).norm(), 0.0, 1.0e-9);
  EXPECT_NEAR((result.trajectory.getVel(0.0) -
               Eigen::Vector3d{2.0, 0.0, 0.0}).norm(), 0.0, 1.0e-9);
  EXPECT_NEAR((result.trajectory[0].getPos(4.0) -
               result.trajectory[1].getPos(0.0)).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR((result.trajectory[0].getVel(4.0) -
               result.trajectory[1].getVel(0.0)).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR((result.trajectory[0].getAcc(4.0) -
               result.trajectory[1].getAcc(0.0)).norm(), 0.0, 1.0e-8);
  EXPECT_NEAR((result.trajectory[0].getJer(4.0) -
               result.trajectory[1].getJer(0.0)).norm(), 0.0, 1.0e-8);

  traj_opt::Config config(PLANNER_CORRIDOR_BEZIER_CONFIG_PATH, "exp_traj");
  std::vector<unsigned char> route_gates(corridors.size(), 0U);
  std::vector<navigation_math::Vec3f> route_points(
      corridors.size(), navigation_math::Vec3f::Zero());
  std::vector<double> route_radii(corridors.size(), 0.0);
  const auto certificate =
      navigation_planning_backend::certifyDeterministicNominalSeed(
          result.trajectory, corridors, mapping, route_gates, route_points,
          route_radii,
          state({0.0, 0.0, 3.0}, {2.0, 0.0, 0.0}),
          state({20.0, 0.0, 3.0}), config);
  EXPECT_TRUE(certificate.valid)
      << "failure stage=" << static_cast<int>(certificate.failure_stage)
      << " velocity=" << certificate.maximum_velocity_mps
      << " acceleration=" << certificate.maximum_acceleration_mps2
      << " jerk=" << certificate.maximum_jerk_mps3;
}

TEST(CorridorBezierSeed, MatchesStraightJunctionVelocityToPieceTiming) {
  navigation_math::PolyhedraH corridors{
      box({-0.1, -1.0, 2.0}, {1.1, 1.0, 4.0}),
      box({0.9, -1.0, 2.0}, {3.1, 1.0, 4.0})};
  navigation_math::Mat3Df junctions(3, 1);
  junctions.col(0) = Eigen::Vector3d{1.0, 0.0, 3.0};
  navigation_math::VecDf durations(2);
  durations << 0.2, 0.4;
  navigation_math::VecDi mapping(2);
  mapping << 0, 1;

  const auto result =
      navigation_planning_backend::buildCorridorContainedBezierSeed(
          state({0.0, 0.0, 3.0}, {5.0, 0.0, 0.0}),
          state({3.0, 0.0, 3.0}, {5.0, 0.0, 0.0}), junctions, durations,
          corridors, mapping, 8.0, 1.0e-8);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.minimum_internal_velocity_scale, 1.0, 1.0e-12);
  EXPECT_NEAR(result.trajectory[0].getVel(0.2).x(), 5.0, 1.0e-9);
  EXPECT_NEAR(result.trajectory.getMaxVelRate(), 5.0, 1.0e-7);
  EXPECT_NEAR(result.trajectory.getMaxAccRate(), 0.0, 1.0e-6);
  EXPECT_NEAR(result.trajectory.getMaxJerRate(), 0.0, 1.0e-5);
}

TEST(CorridorBezierSeed, ReducesJunctionVelocityToContainACorner) {
  navigation_math::PolyhedraH corridors{
      box({-1.0, -0.1, 2.0}, {10.5, 2.0, 4.0}),
      box({8.0, -0.1, 2.0}, {11.0, 11.0, 4.0})};
  navigation_math::Mat3Df junctions(3, 1);
  junctions.col(0) = Eigen::Vector3d{9.0, 0.0, 3.0};
  navigation_math::VecDf durations(2);
  durations << 3.0, 3.0;
  navigation_math::VecDi mapping(2);
  mapping << 0, 1;
  const auto result =
      navigation_planning_backend::buildCorridorContainedBezierSeed(
          state({0.0, 0.0, 3.0}), state({9.0, 10.0, 3.0}),
          junctions, durations, corridors, mapping, 8.0, 1.0e-8);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.minimum_internal_velocity_scale, 1.0);
  for (int piece = 0; piece < result.trajectory.getPieceNum(); ++piece) {
    for (int sample = 0; sample <= 100; ++sample) {
      const double time = durations(piece) * sample / 100.0;
      EXPECT_TRUE(navigation_planning_backend::corridor_bezier_detail::pointInside(
          corridors[static_cast<std::size_t>(piece)],
          result.trajectory[piece].getPos(time), 1.0e-7));
    }
  }
}

TEST(CorridorBezierSeed, BoundedDurationRetryCanRecoverDynamicCertificate) {
  navigation_math::PolyhedraH corridors{
      box({-1.0, -2.0, 2.0}, {11.0, 2.0, 4.0})};
  navigation_math::Mat3Df junctions(3, 0);
  navigation_math::VecDf durations(1);
  durations << 1.0;
  navigation_math::VecDi mapping(1);
  mapping << 0;
  const auto initial =
      navigation_planning_backend::buildCorridorContainedBezierSeed(
          state({0.0, 0.0, 3.0}), state({4.0, 0.0, 3.0}), junctions,
          durations, corridors, mapping, 5.0, 1.0e-8);
  ASSERT_TRUE(initial.valid);

  traj_opt::Config config(PLANNER_CORRIDOR_BEZIER_CONFIG_PATH, "exp_traj");
  config.max_vel = 5.0;
  config.max_acc = 2.0;
  config.max_jerk = 4.0;
  std::vector<unsigned char> route_gates(corridors.size(), 0U);
  std::vector<navigation_math::Vec3f> route_points(
      corridors.size(), navigation_math::Vec3f::Zero());
  std::vector<double> route_radii(corridors.size(), 0.0);
  const auto initial_certificate =
      navigation_planning_backend::certifyDeterministicNominalSeed(
          initial.trajectory, corridors, mapping, route_gates, route_points,
          route_radii, state({0.0, 0.0, 3.0}), state({4.0, 0.0, 3.0}),
          config);
  ASSERT_FALSE(initial_certificate.valid);
  ASSERT_EQ(initial_certificate.failure_stage,
            navigation_planning_backend::
                DeterministicNominalSeedFailureStage::kDynamics);

  const auto scales = navigation_planning_backend::
      boundedDynamicDurationRetryScales(initial_certificate, config);
  ASSERT_FALSE(scales.empty());
  bool recovered = false;
  navigation_planning_backend::DeterministicNominalSeedCertificate
      last_certificate = initial_certificate;
  for (const double scale : scales) {
    const auto retry =
        navigation_planning_backend::buildCorridorContainedBezierSeed(
            state({0.0, 0.0, 3.0}), state({4.0, 0.0, 3.0}), junctions,
            durations * scale, corridors, mapping, 5.0, 1.0e-8);
    if (!retry.valid) continue;
    const auto certificate =
        navigation_planning_backend::certifyDeterministicNominalSeed(
            retry.trajectory, corridors, mapping, route_gates, route_points,
            route_radii, state({0.0, 0.0, 3.0}),
            state({4.0, 0.0, 3.0}), config);
    last_certificate = certificate;
    recovered = recovered || certificate.valid;
  }
  EXPECT_TRUE(recovered)
      << "last_stage=" << static_cast<int>(last_certificate.failure_stage)
      << " vel=" << last_certificate.maximum_velocity_mps
      << " acc=" << last_certificate.maximum_acceleration_mps2
      << " jerk=" << last_certificate.maximum_jerk_mps3
      << " corridor=" << last_certificate.maximum_corridor_violation_m;
}

TEST(CorridorBezierSeed, RejectsImmutableBoundaryDerivativeOutsideCorridor) {
  navigation_math::PolyhedraH corridors{
      box({0.0, -1.0, 2.0}, {10.0, 1.0, 4.0})};
  navigation_math::Mat3Df junctions(3, 0);
  navigation_math::VecDf durations(1);
  durations << 2.0;
  navigation_math::VecDi mapping(1);
  mapping << 0;
  const auto result =
      navigation_planning_backend::buildCorridorContainedBezierSeed(
          state({0.0, 0.0, 3.0}, {-2.0, 0.0, 0.0}),
          state({8.0, 0.0, 3.0}), junctions, durations, corridors, mapping,
          3.0, 1.0e-8);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_stage,
            navigation_planning_backend::CorridorBezierSeedFailureStage::
                kBoundaryControl);
}
