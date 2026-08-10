#include <gtest/gtest.h>

#include "rog_map_core/voxel_occupancy_map.hpp"
#include "rog_map_core/voxel_visualization.hpp"

namespace uav::nav::rog {
namespace {

VoxelMapConfig testConfig() {
  VoxelMapConfig config;
  config.resolution_m = 1.0;
  config.size_m = {20.0, 20.0, 20.0};
  config.inflation_radius_m = 1.0;
  config.raycast_min_range_m = 0.1;
  config.raycast_max_range_m = 10.0;
  config.shift_threshold_m = 2.0;
  return config;
}

TEST(VoxelOccupancyMapTest, RaycastProducesFreeEndpointOccupiedAndUnknown) {
  VoxelOccupancyMap map(testConfig());
  map.setValidity(MapValidity::kActive);
  (void)map.update({0.0, 0.0, 0.0}, {{4.2, 0.2, 0.2}});
  EXPECT_EQ(map.query({1.2, 0.2, 0.2}), VoxelState::kFree);
  EXPECT_EQ(map.query({4.2, 0.2, 0.2}), VoxelState::kOccupied);
  EXPECT_EQ(map.query({8.2, 8.2, 8.2}), VoxelState::kUnknown);
}

TEST(VoxelOccupancyMapTest, TruncatedRayIsFreeOnly) {
  auto config = testConfig();
  config.raycast_max_range_m = 5.0;
  VoxelOccupancyMap map(config);
  map.setValidity(MapValidity::kActive);

  const auto stats = map.update({0.0, 0.0, 0.0}, {{12.2, 0.2, 0.2}});

  EXPECT_EQ(stats.occupied_voxels_updated, 0U);
  EXPECT_GT(stats.free_voxels_updated, 0U);
  EXPECT_EQ(map.occupiedVoxelCount(), 0U);
  EXPECT_EQ(map.query({4.2, 0.2, 0.2}), VoxelState::kFree);
  EXPECT_EQ(map.query({5.2, 0.2, 0.2}), VoxelState::kUnknown);
}

TEST(VoxelOccupancyMapTest, BelowMinimumRangeDoesNotChangeMap) {
  auto config = testConfig();
  config.raycast_min_range_m = 0.50;
  VoxelOccupancyMap map(config);
  map.setValidity(MapValidity::kActive);

  const auto stats = map.update({0.0, 0.0, 0.0}, {{0.49, 0.0, 0.0}});

  EXPECT_EQ(stats.free_voxels_updated, 0U);
  EXPECT_EQ(stats.occupied_voxels_updated, 0U);
  EXPECT_EQ(stats.points_integrated, 0U);
  EXPECT_EQ(map.occupiedVoxelCount(), 0U);
  EXPECT_EQ(map.allocatedVoxelCount(), 0U);
}

TEST(VoxelOccupancyMapTest, InflationIsBoundedByConfiguredRadius) {
  VoxelOccupancyMap map(testConfig());
  map.setValidity(MapValidity::kActive);
  (void)map.update({0.0, 0.0, 0.0}, {{4.2, 0.2, 0.2}});
  EXPECT_TRUE(map.isInflatedOccupied({3.5, 0.5, 0.5}));
  EXPECT_FALSE(map.isInflatedOccupied({6.5, 0.5, 0.5}));
}

TEST(VoxelOccupancyMapTest, ExportsOccupiedAndInflatedCentersForVisualization) {
  VoxelOccupancyMap map(testConfig());
  map.setValidity(MapValidity::kActive);
  (void)map.update({0.0, 0.0, 0.0}, {{4.2, 0.2, 0.2}});

  const auto occupied = map.occupiedVoxelCenters();
  EXPECT_EQ(occupied.size(), map.occupiedVoxelCount());
  EXPECT_FALSE(occupied.empty());
  const auto inflated = deriveInflatedVoxelSet(
      occupied, map.localBounds(), map.resolution(), map.inflationRadius());
  EXPECT_GE(inflated.size(), occupied.size());
  EXPECT_GE(map.inflatedVoxelUpperBound(), inflated.size());
}

TEST(VoxelOccupancyMapTest, EmptyMapHasZeroInflationUpperBound) {
  VoxelOccupancyMap map(testConfig());
  map.setValidity(MapValidity::kActive);
  const auto stats = map.update({0.0, 0.0, 0.0}, {});
  EXPECT_EQ(stats.occupied_voxel_count, 0U);
  EXPECT_EQ(stats.inflated_voxel_upper_bound, 0U);
  EXPECT_EQ(map.inflatedVoxelUpperBound(), 0U);
}

TEST(VoxelOccupancyMapTest, SlidingWindowMovesOnFixedWorldAxes) {
  VoxelOccupancyMap map(testConfig());
  map.setValidity(MapValidity::kActive);
  (void)map.update({0.0, 0.0, 0.0}, {{4.2, 0.2, 0.2}});
  EXPECT_EQ(map.query({4.2, 0.2, 0.2}), VoxelState::kOccupied);
  (void)map.update({9.0, 0.0, 0.0}, {{9.2, 0.2, 0.2}});
  EXPECT_EQ(map.query({9.2, 0.2, 0.2}), VoxelState::kOccupied);
  EXPECT_GT(map.shiftCount(), 0U);
}

TEST(VoxelOccupancyMapTest, ValidityIsFailClosed) {
  VoxelOccupancyMap map(testConfig());
  EXPECT_EQ(map.validity(), MapValidity::kWaitingForLio);
  map.setValidity(MapValidity::kInvalid);
  EXPECT_EQ(map.validity(), MapValidity::kInvalid);
}

TEST(VoxelVisualizationTest, SphericalInflationAndSurfaceUseExplicitSet) {
  const MapBounds bounds{{-10.0, -10.0, -10.0}, {10.0, 10.0, 10.0}};
  const auto inflated = deriveInflatedVoxelSet({{0.5, 0.5, 0.5}}, bounds, 1.0, 1.0);
  EXPECT_EQ(inflated.size(), 7U);
  EXPECT_TRUE(inflated.contains({0, 0, 0}));
  EXPECT_TRUE(inflated.contains({1, 0, 0}));
  EXPECT_FALSE(inflated.contains({1, 1, 1}));
  const auto surface = extractInflationSurface(inflated, bounds, 1.0);
  EXPECT_EQ(surface.size(), 6U);
  for (const auto& voxel : surface) EXPECT_TRUE(inflated.contains(voxel));
}

TEST(VoxelVisualizationTest, SolidThreeByThreeByThreeHasTwentySixSurfaceVoxels) {
  const MapBounds bounds{{-10.0, -10.0, -10.0}, {10.0, 10.0, 10.0}};
  std::vector<Eigen::Vector3d> centers;
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      for (int z = 0; z < 3; ++z) centers.emplace_back(x + 0.5, y + 0.5, z + 0.5);
    }
  }
  const auto solid = visualizationSetFromCenters(centers, 1.0);
  const auto surface = extractInflationSurface(solid, bounds, 1.0);
  EXPECT_EQ(solid.size(), 27U);
  EXPECT_EQ(surface.size(), 26U);
  EXPECT_FALSE(surface.contains({1, 1, 1}));
  for (const auto& voxel : surface) EXPECT_TRUE(solid.contains(voxel));
}

TEST(VoxelVisualizationTest, BoundaryVoxelIsSurfaceAndNoDuplicates) {
  const MapBounds bounds{{0.0, 0.0, 0.0}, {3.0, 3.0, 3.0}};
  const auto solid = visualizationSetFromCenters({{0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}}, 1.0);
  const auto surface = extractInflationSurface(solid, bounds, 1.0);
  EXPECT_EQ(solid.size(), 1U);
  EXPECT_EQ(surface.size(), 1U);
  EXPECT_TRUE(surface.contains({0, 0, 0}));
}

}  // namespace
}  // namespace uav::nav::rog
