#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"
#include "fast_lio_core/registration/residual_builder.hpp"

namespace uav::nav::lio {
namespace {

std::vector<Eigen::Vector3d> makePlanePoints() {
  std::vector<Eigen::Vector3d> points;
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      points.emplace_back(0.25 * static_cast<double>(x), 0.25 * static_cast<double>(y), 0.0);
    }
  }
  return points;
}

TEST(ResidualBuilderTest, BuildsPointToPlaneJacobianAndGatesOutlier) {
  IkdTreeRegistrationMapConfig map_config;
  map_config.voxel_size_m = 0.02;
  IkdTreeRegistrationMap map(map_config);
  const std::vector<Eigen::Vector3d> plane = makePlanePoints();
  ASSERT_EQ(map.insert(plane), plane.size());

  ResidualBuilderConfig config;
  config.correspondence_search.maximum_neighbor_distance_m = 0.8;
  config.residual_gate.maximum_absolute_distance_m = 0.3;
  ResidualBuilder builder(config);

  ManifoldState state;
  state.set_position_odom_imu_m({0.0, 0.0, 0.1});
  const std::vector<Eigen::Vector3d> scan{
      {-0.5, -0.5, 0.0}, {0.0, -0.5, 0.0}, {0.5, -0.5, 0.0}, {-0.5, 0.0, 0.0}, {0.0, 0.0, 0.0},
      {0.5, 0.0, 0.0},   {-0.5, 0.5, 0.0}, {0.0, 0.5, 0.0},  {0.5, 0.5, 0.0},
  };

  const ResidualBuildResult result = builder.build(scan, state, map);

  ASSERT_EQ(result.measurement.residual_m.size(), scan.size());
  ASSERT_TRUE(result.measurement.valid());
  EXPECT_EQ(result.diagnostics.accepted_residual_count, scan.size());
  EXPECT_NEAR(result.diagnostics.residual_rms_m, 0.1, 1e-9);
  for (Eigen::Index row = 0; row < result.measurement.jacobian.rows(); ++row) {
    EXPECT_NEAR(std::abs(result.measurement.jacobian(row, ManifoldState::kPositionOffset + 2)), 1.0,
                1e-9);
  }

  ManifoldState outlier_state = state;
  outlier_state.set_position_odom_imu_m({0.0, 0.0, 0.5});
  const ResidualBuildResult rejected = builder.build(scan, outlier_state, map);
  EXPECT_EQ(rejected.measurement.residual_m.size(), 0);
  EXPECT_EQ(rejected.diagnostics.rejected_residual_count, scan.size());
}

TEST(ResidualBuilderTest, PointToPlaneJacobianMatchesFiniteDifferenceAndFixedExtrinsicIsZero) {
  IkdTreeRegistrationMap map({0.02});
  ASSERT_GT(map.insert(makePlanePoints()), 0U);
  ResidualBuilderConfig config;
  config.correspondence_search.maximum_neighbor_distance_m = 0.8;
  config.residual_gate.maximum_absolute_distance_m = 0.3;
  config.estimate_extrinsic = false;
  ResidualBuilder builder(config);
  const std::vector<Eigen::Vector3d> scan{{0.35, -0.2, 0.1}};
  ManifoldState state;
  state.set_position_odom_imu_m({0.0, 0.0, 0.05});
  state.set_orientation_odom_imu(
      Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX())));
  state.set_position_imu_lidar_m({-0.019391, -0.000278, 0.080926});

  const auto nominal = builder.build(scan, state, map);
  ASSERT_EQ(nominal.measurement.residual_m.size(), 1);
  constexpr double kStep = 1e-7;
  for (int axis = 0; axis < 3; ++axis) {
    ManifoldState perturbed = state;
    Eigen::Vector3d position = state.position_odom_imu_m();
    position[axis] += kStep;
    perturbed.set_position_odom_imu_m(position);
    const auto measured = builder.build(scan, perturbed, map);
    ASSERT_EQ(measured.measurement.residual_m.size(), 1);
    const double numerical =
        (measured.measurement.residual_m[0] -
         nominal.measurement.residual_m[0]) /
        kStep;
    EXPECT_NEAR(
        numerical,
        nominal.measurement.jacobian(
            0, ManifoldState::kPositionOffset + axis),
        2e-6);
  }
  for (int axis = 0; axis < 3; ++axis) {
    ManifoldState perturbed = state;
    Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
    tangent[axis] = kStep;
    perturbed.set_orientation_odom_imu(
        state.orientation_odom_imu() *
        Eigen::Quaterniond(Eigen::AngleAxisd(
            tangent.norm(), tangent.normalized())));
    const auto measured = builder.build(scan, perturbed, map);
    ASSERT_EQ(measured.measurement.residual_m.size(), 1);
    const double numerical =
        (measured.measurement.residual_m[0] -
         nominal.measurement.residual_m[0]) /
        kStep;
    EXPECT_NEAR(
        numerical,
        nominal.measurement.jacobian(
            0, ManifoldState::kOrientationOffset + axis),
        2e-6);
  }
  EXPECT_TRUE((nominal.measurement.jacobian
                  .block<1, 6>(
                      0, ManifoldState::kExtrinsicRotationOffset)
                  .isZero(1e-14)));
  EXPECT_GT(nominal.measurement.residual_m[0], 0.0);
}

}  // namespace
}  // namespace uav::nav::lio
