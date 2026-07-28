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

}  // namespace
}  // namespace uav::nav::lio
