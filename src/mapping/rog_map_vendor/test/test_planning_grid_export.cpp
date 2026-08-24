#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>

#include "test_rog_map_fixture.hpp"

namespace {

using navigation_mapping::test::TestRogMap;

std::string testConfigPath() {
  return std::string(ROG_MAP_VENDOR_TEST_FIXTURE_DIR) + "/rog_map_test.yaml";
}

std::size_t offsetOf(const rog_map::PlanningGridLayoutExport& layout,
                     const Eigen::Vector3i& index) {
  const Eigen::Vector3i local = index - layout.global_min_index;
  return static_cast<std::size_t>(
      (local.x() * layout.dimensions.y() + local.y()) * layout.dimensions.z() + local.z());
}

Eigen::Vector3d centerOf(const rog_map::PlanningGridLayoutExport& layout,
                         const Eigen::Vector3i& index) {
  return (index.cast<double>() + Eigen::Vector3d::Constant(0.5)) * layout.resolution_m;
}

}  // namespace

TEST(RogMapPlanningGridExport, OwnsDetachedLogicalStateWithExactOrdering) {
  TestRogMap map;
  map.loadConfigAndInit(testConfigPath());
  const auto started = std::chrono::steady_clock::now();
  const auto snapshot = map.exportPlanningGrid();
  const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - started).count();

  const auto base_cells = static_cast<std::size_t>(snapshot.base_layout.dimensions.prod());
  const auto inflated_cells =
      static_cast<std::size_t>(snapshot.inflated.layout.dimensions.prod());
  EXPECT_EQ(snapshot.base_state.size(), base_cells);
  EXPECT_EQ(snapshot.inflated.occupied.size(), inflated_cells);
  EXPECT_EQ(snapshot.unknown_inflation_enabled, !snapshot.inflated.unknown.empty());
  EXPECT_EQ(snapshot.byteSize(), snapshot.base_state.size() +
                                    snapshot.inflated.occupied.size() +
                                    snapshot.inflated.unknown.size() +
                                    snapshot.nearest_offsets.size() * sizeof(Eigen::Vector3i));
  RecordProperty("export_elapsed_us", elapsed_us);
  RecordProperty("export_bytes", snapshot.byteSize());

  const auto& config = map.getMapConfig();
  const Eigen::Vector3i base_max =
      snapshot.base_layout.global_min_index + snapshot.base_layout.dimensions;
  for (int x = snapshot.base_layout.global_min_index.x(); x < base_max.x(); ++x) {
    for (int y = snapshot.base_layout.global_min_index.y(); y < base_max.y(); ++y) {
      for (int z = snapshot.base_layout.global_min_index.z(); z < base_max.z(); ++z) {
        const Eigen::Vector3i index{x, y, z};
        const Eigen::Vector3d point = centerOf(snapshot.base_layout, index);
        if (point.z() <= config.virtual_ground_height ||
            point.z() >= config.virtual_ceil_height) {
          continue;
        }
        EXPECT_EQ(snapshot.base_state[offsetOf(snapshot.base_layout, index)],
                  static_cast<std::uint8_t>(map.getGridType(point)));
      }
    }
  }

  const Eigen::Vector3i inflated_max = snapshot.inflated.layout.global_min_index +
                                       snapshot.inflated.layout.dimensions;
  for (int x = snapshot.inflated.layout.global_min_index.x(); x < inflated_max.x(); ++x) {
    for (int y = snapshot.inflated.layout.global_min_index.y(); y < inflated_max.y(); ++y) {
      for (int z = snapshot.inflated.layout.global_min_index.z(); z < inflated_max.z(); ++z) {
        const Eigen::Vector3i index{x, y, z};
        const Eigen::Vector3d point = centerOf(snapshot.inflated.layout, index);
        if (point.z() <= config.virtual_ground_height ||
            point.z() >= config.virtual_ceil_height) {
          continue;
        }
        EXPECT_EQ(snapshot.inflated.occupied[offsetOf(snapshot.inflated.layout, index)] != 0U,
                  map.isOccupiedInflate(point));
      }
    }
  }
}

TEST(RogMapPlanningGridExport, EarlierValueDoesNotAliasLaterMapUpdate) {
  TestRogMap map;
  map.loadConfigAndInit(testConfigPath());
  const auto before = map.exportPlanningGrid();
  const auto before_state = before.base_state;

  rog_map::PointCloud cloud;
  pcl::PointXYZI point;
  point.x = 3.0F;
  point.y = 0.0F;
  point.z = 0.0F;
  point.intensity = 0.0F;
  cloud.push_back(point);
  const rog_map::Pose pose{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  for (int iteration = 0; iteration < 8; ++iteration) {
    map.updateMap(cloud, pose);
  }
  const auto after = map.exportPlanningGrid();

  EXPECT_EQ(before.base_state, before_state);
  EXPECT_NE(before.base_state, after.base_state);
}
