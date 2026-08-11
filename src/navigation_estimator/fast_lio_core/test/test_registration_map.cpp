#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <cstddef>
#include <random>
#include <string_view>
#include <vector>

#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"
#include "fast_lio_core/mapping/local_map_manager.hpp"
#include "fast_lio_core/mapping/map_insertion_policy.hpp"

namespace uav::nav::lio {
namespace {

IkdTreeRegistrationMapConfig deterministicMapConfig() {
  IkdTreeRegistrationMapConfig config;
  config.enable_asynchronous_rebuild = false;
  return config;
}

TEST(RegistrationMapTest, UsesUpstreamIncrementalTreeAndDownsampling) {
  auto config = deterministicMapConfig();
  config.voxel_size_m = 1.0;
  IkdTreeRegistrationMap map(config);
  const std::vector<Eigen::Vector3d> points{
      {0.1, 0.1, 0.1}, {0.3, 0.1, 0.1}, {2.0, 0.0, 0.0},
      {-2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, -2.0, 0.0}};

  EXPECT_EQ(map.frameId(), std::string_view("lio_odom"));
  EXPECT_EQ(map.backend(), RegistrationMapBackend::kUpstreamIkdTree);
  EXPECT_EQ(map.insert(points), 5U);
  EXPECT_EQ(map.size(), 5U);

  NeighborSet neighbors;
  ASSERT_TRUE(map.nearestSearch({0.0, 0.0, 0.0}, 3.0, neighbors));
  ASSERT_EQ(neighbors.count, NeighborSet::kCapacity);
  EXPECT_NEAR(neighbors.points.front().x(), 0.3, 1e-6);

  const std::vector<Eigen::Vector3d> closer_to_center{{0.4, 0.1, 0.1}};
  EXPECT_EQ(map.insert(closer_to_center), 0U);
  NeighborSet replaced;
  ASSERT_TRUE(map.nearestSearch({0.0, 0.0, 0.0}, 3.0, replaced));
  EXPECT_NEAR(replaced.points.front().x(), 0.4, 1e-6);
}

TEST(RegistrationMapTest, FixedQueryMatchesBruteForceNearestPrefix) {
  auto config = deterministicMapConfig();
  config.voxel_size_m = 0.001;
  IkdTreeRegistrationMap map(config);
  std::mt19937 generator(0x5A17U);
  std::uniform_real_distribution<double> coordinate(-20.0, 20.0);
  std::vector<Eigen::Vector3d> inserted_points;
  inserted_points.reserve(256U);
  for (std::size_t index = 0; index < 256U; ++index) {
    inserted_points.emplace_back(coordinate(generator), coordinate(generator),
                                 coordinate(generator));
  }
  ASSERT_EQ(map.insert(inserted_points), inserted_points.size());
  const std::vector<Eigen::Vector3d> represented_points = map.snapshot();

  const Eigen::Vector3d query(coordinate(generator), coordinate(generator),
                              coordinate(generator));
  std::vector<Eigen::Vector3d> sorted = represented_points;
  std::sort(sorted.begin(), sorted.end(), [&](const auto& left, const auto& right) {
    return (left - query).squaredNorm() < (right - query).squaredNorm();
  });
  NeighborSet actual;
  ASSERT_TRUE(map.nearestSearch(query, 12.0, actual));
  ASSERT_EQ(actual.count, NeighborSet::kCapacity);
  for (std::size_t index = 0; index < NeighborSet::kCapacity; ++index) {
    EXPECT_NEAR((actual.points[index] - sorted[index]).norm(), 0.0, 1e-5);
  }
}

TEST(RegistrationMapTest, LocalCropDeletesOutsidePointsThroughUpstreamBoxes) {
  auto config = deterministicMapConfig();
  config.voxel_size_m = 0.01;
  IkdTreeRegistrationMap map(config);
  const std::vector<Eigen::Vector3d> points{
      {0.0, 0.0, 0.0}, {-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0},
      {0.0, -2.0, 0.0}, {0.0, 2.0, 0.0}, {0.0, 0.0, -2.0},
      {0.0, 0.0, 2.0}, {-2.1, 0.0, 0.0}, {2.1, 0.0, 0.0},
      {0.0, -2.1, 0.0}, {0.0, 2.1, 0.0}, {0.0, 0.0, -2.1},
      {0.0, 0.0, 2.1}};
  ASSERT_EQ(map.insert(points), points.size());
  EXPECT_EQ(map.cropLocal({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}), 6U);
  EXPECT_EQ(map.size(), 7U);
  for (const Eigen::Vector3d& point : map.snapshot()) {
    EXPECT_LE(point.cwiseAbs().maxCoeff(), 2.0);
  }
}

TEST(RegistrationMapTest, ClearAllowsFreshUpstreamBuild) {
  IkdTreeRegistrationMap map(deterministicMapConfig());
  const std::vector<Eigen::Vector3d> first{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  ASSERT_EQ(map.insert(first), 2U);
  map.clear();
  EXPECT_EQ(map.size(), 0U);
  EXPECT_TRUE(map.snapshot().empty());
  const std::vector<Eigen::Vector3d> second{{5.0, 0.0, 0.0}};
  EXPECT_EQ(map.insert(second), 1U);
  EXPECT_EQ(map.size(), 1U);
}

TEST(RegistrationMapTest, LocalManagerUsesOnlyGeometricCropAndGuard) {
  auto map_config = deterministicMapConfig();
  map_config.voxel_size_m = 0.1;
  IkdTreeRegistrationMap map(map_config);
  const std::vector<Eigen::Vector3d> points{{0.0, 0.0, 0.0},
                                             {20.0, 0.0, 0.0}};
  ASSERT_EQ(map.insert(points), 2U);
  LocalMapManagerConfig config;
  config.half_extent_m = {2.0, 2.0, 2.0};
  config.crop_trigger_distance_m = 1.0;
  config.absolute_map_point_guard = 3U;
  LocalMapManager manager(config);
  const LocalMapUpdate update = manager.update(map, {0.0, 0.0, 0.0});
  EXPECT_TRUE(update.crop_performed);
  EXPECT_EQ(update.removed_point_count, 1U);
  EXPECT_TRUE(manager.insertionAllowed());
  EXPECT_EQ(map.size(), 1U);
}

TEST(RegistrationMapTest, GuardFreezesInsertionWhenCropCannotRecover) {
  auto map_config = deterministicMapConfig();
  map_config.voxel_size_m = 0.01;
  IkdTreeRegistrationMap map(map_config);
  std::vector<Eigen::Vector3d> points;
  for (int index = 0; index < 10; ++index) {
    points.emplace_back(static_cast<double>(index), 0.0, 0.0);
  }
  ASSERT_EQ(map.insert(points), points.size());
  LocalMapManagerConfig config;
  config.half_extent_m = {100.0, 100.0, 100.0};
  config.absolute_map_point_guard = 5U;
  LocalMapManager manager(config);
  const auto update = manager.update(map, Eigen::Vector3d::Zero());
  EXPECT_TRUE(update.absolute_guard_triggered);
  EXPECT_TRUE(update.absolute_guard_recovery_failed);
  EXPECT_TRUE(update.insertion_frozen);
  EXPECT_FALSE(manager.insertionAllowed());
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
}

}  // namespace
}  // namespace uav::nav::lio
