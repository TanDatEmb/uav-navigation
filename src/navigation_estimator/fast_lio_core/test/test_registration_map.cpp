#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>
#include <string_view>
#include <tuple>
#include <vector>

#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"
#include "fast_lio_core/mapping/local_map_manager.hpp"
#include "fast_lio_core/mapping/map_insertion_policy.hpp"

namespace uav::nav::lio {
namespace {

NearestNeighborResult bruteForceNearest(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector3d& query,
    std::size_t neighbor_count,
    double maximum_distance_m) {
  struct Candidate {
    Eigen::Vector3d point;
    double squared_distance_m2{0.0};
  };
  std::vector<Candidate> candidates;
  const double maximum_squared_distance_m2 =
      maximum_distance_m * maximum_distance_m;
  for (const Eigen::Vector3d& point : points) {
    const double squared_distance_m2 =
        (point - query).squaredNorm();
    if (squared_distance_m2 <= maximum_squared_distance_m2) {
      candidates.push_back({point, squared_distance_m2});
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& left, const Candidate& right) {
              if (left.squared_distance_m2 !=
                  right.squared_distance_m2) {
                return left.squared_distance_m2 <
                       right.squared_distance_m2;
              }
              return std::tie(left.point.x(), left.point.y(),
                              left.point.z()) <
                     std::tie(right.point.x(), right.point.y(),
                              right.point.z());
            });
  candidates.resize(std::min(neighbor_count, candidates.size()));

  NearestNeighborResult result;
  for (const Candidate& candidate : candidates) {
    result.points_odom_m.push_back(candidate.point);
    result.squared_distances_m2.push_back(
        candidate.squared_distance_m2);
  }
  return result;
}

TEST(RegistrationMapTest, UsesUpstreamIncrementalTreeAndDownsampling) {
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
  EXPECT_EQ(map.backend(),
            RegistrationMapBackend::kUpstreamIkdTree);
  EXPECT_EQ(map.insert(points), 3U);
  EXPECT_EQ(map.size(), 3U);

  // Upstream Add_Points downsampling keeps the point closest to the voxel
  // center; it does not synthesize a centroid.
  const NearestNeighborResult nearest =
      map.nearestNeighbors({0.0, 0.0, 0.0}, 3U, 3.0);
  ASSERT_TRUE(nearest.complete(3U));
  EXPECT_NEAR(nearest.points_odom_m.front().x(), 0.3, 1e-6);
  EXPECT_NEAR(nearest.points_odom_m[1].x(), -2.0, 1e-6);
  EXPECT_NEAR(nearest.points_odom_m[2].x(), 2.0, 1e-6);

  const std::vector<Eigen::Vector3d> closer_to_center{
      {0.4, 0.1, 0.1}};
  EXPECT_EQ(map.insert(closer_to_center), 0U);
  EXPECT_EQ(map.size(), 3U);
  const NearestNeighborResult replaced =
      map.nearestNeighbors({0.0, 0.0, 0.0}, 1U, 1.0);
  ASSERT_TRUE(replaced.complete(1U));
  EXPECT_NEAR(replaced.points_odom_m.front().x(), 0.4, 1e-6);
}

TEST(RegistrationMapTest, KnnMatchesBruteForceDifferentialOracle) {
  IkdTreeRegistrationMapConfig config;
  config.voxel_size_m = 0.001;
  IkdTreeRegistrationMap map(config);
  std::mt19937 generator(0x5A17U);
  std::uniform_real_distribution<double> coordinate(-20.0, 20.0);
  std::vector<Eigen::Vector3d> inserted_points;
  inserted_points.reserve(256U);
  for (std::size_t index = 0; index < 256U; ++index) {
    inserted_points.emplace_back(
        coordinate(generator), coordinate(generator),
        coordinate(generator));
  }
  ASSERT_EQ(map.insert(inserted_points), inserted_points.size());
  const std::vector<Eigen::Vector3d> represented_points =
      map.snapshot();
  ASSERT_EQ(represented_points.size(), inserted_points.size());

  for (std::size_t query_index = 0; query_index < 32U;
       ++query_index) {
    const Eigen::Vector3d query(
        coordinate(generator), coordinate(generator),
        coordinate(generator));
    const NearestNeighborResult expected =
        bruteForceNearest(represented_points, query, 7U, 12.0);
    const NearestNeighborResult actual =
        map.nearestNeighbors(query, 7U, 12.0);
    ASSERT_EQ(actual.points_odom_m.size(),
              expected.points_odom_m.size());
    ASSERT_EQ(actual.squared_distances_m2.size(),
              expected.squared_distances_m2.size());
    for (std::size_t index = 0;
         index < expected.points_odom_m.size(); ++index) {
      EXPECT_NEAR(
          (actual.points_odom_m[index] -
           expected.points_odom_m[index])
              .norm(),
          0.0, 1e-6);
      EXPECT_NEAR(actual.squared_distances_m2[index],
                  expected.squared_distances_m2[index], 1e-6);
    }
  }
}

TEST(RegistrationMapTest,
     LocalCropDeletesOutsidePointsThroughUpstreamBoxes) {
  IkdTreeRegistrationMapConfig config;
  config.voxel_size_m = 0.01;
  IkdTreeRegistrationMap map(config);
  const std::vector<Eigen::Vector3d> points{
      {0.0, 0.0, 0.0},
      {-2.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {0.0, -2.0, 0.0},
      {0.0, 2.0, 0.0},
      {0.0, 0.0, -2.0},
      {0.0, 0.0, 2.0},
      {-2.1, 0.0, 0.0},
      {2.1, 0.0, 0.0},
      {0.0, -2.1, 0.0},
      {0.0, 2.1, 0.0},
      {0.0, 0.0, -2.1},
      {0.0, 0.0, 2.1},
  };
  ASSERT_EQ(map.insert(points), points.size());

  EXPECT_EQ(
      map.cropLocal({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}),
      6U);
  EXPECT_EQ(map.size(), 7U);
  const std::vector<Eigen::Vector3d> remaining = map.snapshot();
  ASSERT_EQ(remaining.size(), 7U);
  for (const Eigen::Vector3d& point : remaining) {
    EXPECT_LE(point.cwiseAbs().maxCoeff(), 2.0);
  }
}

TEST(RegistrationMapTest, ClearAllowsFreshUpstreamBuild) {
  IkdTreeRegistrationMap map;
  const std::vector<Eigen::Vector3d> first{
      {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  ASSERT_EQ(map.insert(first), 2U);

  map.clear();

  EXPECT_EQ(map.size(), 0U);
  EXPECT_TRUE(map.snapshot().empty());
  const std::vector<Eigen::Vector3d> second{
      {5.0, 0.0, 0.0}};
  EXPECT_EQ(map.insert(second), 1U);
  const NearestNeighborResult nearest =
      map.nearestNeighbors({5.1, 0.0, 0.0}, 1U, 1.0);
  ASSERT_TRUE(nearest.complete(1U));
  EXPECT_NEAR(nearest.points_odom_m.front().x(), 5.0, 1e-6);
}

TEST(RegistrationMapTest, RepeatedLifecycleUsesStableSignedCounts) {
  IkdTreeRegistrationMapConfig config;
  config.voxel_size_m = 0.01;
  for (int iteration = 0; iteration < 25; ++iteration) {
    IkdTreeRegistrationMap map(config);
    std::vector<Eigen::Vector3d> points;
    points.reserve(2000U);
    for (int index = 0; index < 2000; ++index) {
      points.emplace_back(static_cast<double>(index) * 0.02,
                          static_cast<double>(iteration), 0.0);
    }
    EXPECT_EQ(map.insert(points), points.size());
    EXPECT_EQ(map.size(), points.size());
    map.clear();
    EXPECT_EQ(map.size(), 0U);
  }
}

TEST(RegistrationMapTest, InsertRebuildsAfterCropDeletesEntireTree) {
  IkdTreeRegistrationMap map;
  const std::vector<Eigen::Vector3d> old_points{
      {10.0, 0.0, 0.0}, {11.0, 0.0, 0.0}};
  ASSERT_EQ(map.insert(old_points), 2U);
  ASSERT_EQ(
      map.cropLocal({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}),
      2U);
  ASSERT_EQ(map.size(), 0U);

  const std::vector<Eigen::Vector3d> new_points{
      {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}};
  EXPECT_EQ(map.insert(new_points), 2U);
  EXPECT_EQ(map.size(), 2U);
  const NearestNeighborResult nearest =
      map.nearestNeighbors({0.1, 0.0, 0.0}, 1U, 1.0);
  ASSERT_TRUE(nearest.complete(1U));
  EXPECT_NEAR(nearest.points_odom_m.front().x(), 0.0, 1e-6);
}

TEST(RegistrationMapTest,
     InsertionPolicyRequiresCorrectedTrackingUpdate) {
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
  const std::vector<Eigen::Vector3d> points{
      {0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  static_cast<void>(map.insert(points));

  LocalMapManagerConfig local_config;
  local_config.half_extent_m = {2.0, 2.0, 2.0};
  local_config.crop_trigger_distance_m = 1.0;
  LocalMapManager manager(local_config);
  const LocalMapUpdate update =
      manager.update(map, {0.0, 0.0, 0.0});

  EXPECT_TRUE(update.crop_performed);
  EXPECT_EQ(update.removed_point_count, 1U);
  EXPECT_EQ(map.size(), 1U);
}

TEST(RegistrationMapTest, LocalManagerLeavesMapBelowSoftLimitUntouched) {
  IkdTreeRegistrationMapConfig map_config;
  map_config.voxel_size_m = 0.01;
  IkdTreeRegistrationMap map(map_config);
  const std::vector<Eigen::Vector3d> points{
      {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  ASSERT_EQ(map.insert(points), 3U);
  LocalMapManagerConfig config;
  config.half_extent_m = {100.0, 100.0, 100.0};
  config.soft_point_limit = 5;
  config.hard_point_limit = 8;
  config.target_point_count_after_prune = 3;
  LocalMapManager manager(config);
  const auto update = manager.update(map, {0.0, 0.0, 0.0});
  EXPECT_EQ(update.distance_pruned_count, 0U);
  EXPECT_EQ(map.size(), 3U);
}

TEST(RegistrationMapTest,
     LocalManagerPrunesFarthestToTargetAndUsesHysteresis) {
  IkdTreeRegistrationMapConfig map_config;
  map_config.voxel_size_m = 0.01;
  IkdTreeRegistrationMap map(map_config);
  std::vector<Eigen::Vector3d> points;
  for (int index = 0; index < 10; ++index) {
    points.emplace_back(static_cast<double>(index), 0.0, 0.0);
  }
  ASSERT_EQ(map.insert(points), 10U);
  LocalMapManagerConfig config;
  config.half_extent_m = {100.0, 100.0, 100.0};
  config.crop_trigger_distance_m = 1.0;
  config.soft_point_limit = 5;
  config.hard_point_limit = 8;
  config.target_point_count_after_prune = 3;
  config.distance_shell_size_m = 1.0;
  LocalMapManager manager(config);
  const auto first = manager.update(map, {0.0, 0.0, 0.0});
  EXPECT_EQ(first.distance_pruned_count, 7U);
  EXPECT_EQ(first.removed_point_count, 7U);
  EXPECT_EQ(map.size(), 3U);
  const auto remaining = map.snapshot();
  ASSERT_EQ(remaining.size(), 3U);
  EXPECT_LE(std::max_element(
                remaining.begin(), remaining.end(),
                [](const auto& left, const auto& right) {
                  return left.norm() < right.norm();
                })
                ->norm(),
            2.01);
  const auto second = manager.update(map, {0.1, 0.0, 0.0});
  EXPECT_FALSE(second.crop_performed);
  EXPECT_EQ(second.distance_pruned_count, 0U);
}

TEST(RegistrationMapTest, LocalManagerDoesNotPruneAtInvalidPosition) {
  IkdTreeRegistrationMapConfig map_config;
  map_config.voxel_size_m = 0.01;
  IkdTreeRegistrationMap map(map_config);
  std::vector<Eigen::Vector3d> points(10, Eigen::Vector3d::Zero());
  for (std::size_t index = 0; index < points.size(); ++index) {
    points[index].x() = static_cast<double>(index);
  }
  static_cast<void>(map.insert(points));
  LocalMapManagerConfig config;
  config.soft_point_limit = 5;
  config.hard_point_limit = 8;
  config.target_point_count_after_prune = 3;
  LocalMapManager manager(config);
  const auto update = manager.update(
      map, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  EXPECT_FALSE(update.crop_performed);
  EXPECT_EQ(update.distance_pruned_count, 0U);
  EXPECT_EQ(map.size(), 10U);
}

TEST(RegistrationMapTest, RejectsInvalidLocalMapLimitOrdering) {
  LocalMapManagerConfig config;
  config.target_point_count_after_prune = config.soft_point_limit;
  EXPECT_THROW(static_cast<void>(LocalMapManager{config}),
               std::invalid_argument);
}

}  // namespace
}  // namespace uav::nav::lio
