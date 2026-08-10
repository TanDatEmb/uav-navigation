#include <gtest/gtest.h>

#include "rog_map_core/voxel_occupancy_map.hpp"

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
  const auto inflated = map.inflatedVoxelCenters();

  EXPECT_EQ(occupied.size(), map.occupiedVoxelCount());
  EXPECT_FALSE(occupied.empty());
  EXPECT_GE(inflated.size(), occupied.size());
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

}  // namespace
}  // namespace uav::nav::rog
