#include <gtest/gtest.h>

#include <Eigen/Core>
#include <string_view>
#include <vector>

#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"
#include "fast_lio_core/mapping/local_map_manager.hpp"
#include "fast_lio_core/mapping/map_insertion_policy.hpp"

namespace uav::nav::lio {
namespace {

TEST(RegistrationMapTest, DownsamplesInOdomAndQueriesDeterministically) {
  IkdTreeRegistrationMapConfig config;
  config.voxel_size_m = 1.0;
  IkdTreeRegistrationMap map(config);
  const std::vector<Eigen::Vector3d> points{
      {0.1, 0.1, 0.1},
      {0.3, 0.1, 0.1},
      {2.0, 0.0, 0.0},
      {-2.0, 0.0, 0.0},
  };

  EXPECT_EQ(map.frameId(), std::string_view("odom"));
  EXPECT_EQ(map.backend(), RegistrationMapBackend::kDeterministicVoxelBruteForce);
  EXPECT_EQ(map.insert(points), 3U);
  EXPECT_EQ(map.size(), 3U);

  const NearestNeighborResult nearest = map.nearestNeighbors({0.0, 0.0, 0.0}, 3U, 3.0);
  ASSERT_TRUE(nearest.complete(3U));
  EXPECT_NEAR(nearest.points_odom_m.front().x(), 0.2, 1e-12);
  // Equal-distance ties are resolved lexicographically.
  EXPECT_DOUBLE_EQ(nearest.points_odom_m[1].x(), -2.0);
  EXPECT_DOUBLE_EQ(nearest.points_odom_m[2].x(), 2.0);
}

TEST(RegistrationMapTest, CropsAroundOdomCenter) {
  IkdTreeRegistrationMapConfig config;
  config.voxel_size_m = 0.1;
  IkdTreeRegistrationMap map(config);
  const std::vector<Eigen::Vector3d> points{
      {0.0, 0.0, 0.0},
      {1.0, 1.0, 1.0},
      {10.0, 0.0, 0.0},
  };
  static_cast<void>(map.insert(points));

  EXPECT_EQ(map.cropLocal({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}), 1U);
  EXPECT_EQ(map.size(), 2U);
}

TEST(RegistrationMapTest, InsertionPolicyRequiresCorrectedTrackingUpdate) {
  MapInsertionPolicyConfig config;
  config.minimum_point_count = 3;
  MapInsertionPolicy policy(config);
  MapInsertionContext context;
  context.estimator_tracking = true;
  context.lidar_update_successful = true;
  context.converged = true;
  context.transform_finite = true;
  context.filtered_point_count = 3;

  EXPECT_TRUE(policy.permits(context));
  context.converged = false;
  EXPECT_FALSE(policy.permits(context));
  context.converged = true;
  context.estimator_tracking = false;
  EXPECT_FALSE(policy.permits(context));
}

TEST(RegistrationMapTest, LocalManagerReportsRemovedPoints) {
  IkdTreeRegistrationMapConfig map_config;
  map_config.voxel_size_m = 0.1;
  IkdTreeRegistrationMap map(map_config);
  const std::vector<Eigen::Vector3d> points{{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  static_cast<void>(map.insert(points));

  LocalMapManagerConfig local_config;
  local_config.half_extent_m = {2.0, 2.0, 2.0};
  local_config.crop_trigger_distance_m = 1.0;
  LocalMapManager manager(local_config);
  const LocalMapUpdate update = manager.update(map, {0.0, 0.0, 0.0});

  EXPECT_TRUE(update.crop_performed);
  EXPECT_EQ(update.removed_point_count, 1U);
  EXPECT_EQ(map.size(), 1U);
}

}  // namespace
}  // namespace uav::nav::lio
