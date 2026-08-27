#include <gtest/gtest.h>

#include <chrono>
#include <array>
#include <cstddef>
#include <cstdint>

#include "test_rog_map_fixture.hpp"

namespace {

using navigation_mapping::test::TestRogMap;

std::string testConfigPath() {
  return std::string(ROG_MAP_VENDOR_TEST_FIXTURE_DIR) + "/rog_map_test.yaml";
}

std::string noVirtualPlanesConfigPath() {
  return std::string(ROG_MAP_VENDOR_TEST_FIXTURE_DIR) +
      "/rog_map_no_virtual_planes.yaml";
}

std::string raycastingBoundaryConfigPath() {
  return std::string(ROG_MAP_VENDOR_TEST_FIXTURE_DIR) +
      "/rog_map_raycasting_boundary.yaml";
}

rog_map::PointCloud singlePointCloud(const Eigen::Vector3d& point) {
  rog_map::PointCloud cloud;
  pcl::PointXYZI sample;
  sample.x = static_cast<float>(point.x());
  sample.y = static_cast<float>(point.y());
  sample.z = static_cast<float>(point.z());
  sample.intensity = 0.0F;
  cloud.push_back(sample);
  return cloud;
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

TEST(RogMapPlanningGridExport, UpdateOutcomeTruthTableIsExplicit) {
  EXPECT_TRUE(rog_map::mapUpdateAdvancedWorld(rog_map::MapUpdateOutcome::UPDATED));
  EXPECT_TRUE(rog_map::mapUpdateAdvancedWorld(rog_map::MapUpdateOutcome::SLIDE_ONLY));
  for (const auto outcome : {
           rog_map::MapUpdateOutcome::ACCUMULATED,
           rog_map::MapUpdateOutcome::EMPTY_CLOUD,
           rog_map::MapUpdateOutcome::CALLBACK_OWNED,
           rog_map::MapUpdateOutcome::BELOW_GROUND,
           rog_map::MapUpdateOutcome::ABOVE_CEILING}) {
    EXPECT_FALSE(rog_map::mapUpdateAdvancedWorld(outcome));
    EXPECT_STRNE(rog_map::mapUpdateOutcomeName(outcome), "UNKNOWN");
  }
}

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
                                    snapshot.nearest_offsets->size() * sizeof(Eigen::Vector3i));
  EXPECT_EQ(snapshot.ownedByteSize(), snapshot.base_state.size() +
                                         snapshot.inflated.occupied.size() +
                                         snapshot.inflated.unknown.size());
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
    EXPECT_EQ(map.updateMap(cloud, pose), rog_map::MapUpdateOutcome::UPDATED);
  }
  const auto after = map.exportPlanningGrid();

  EXPECT_EQ(before.base_state, before_state);
  EXPECT_NE(before.base_state, after.base_state);
  EXPECT_EQ(before.nearest_offsets.get(), after.nearest_offsets.get());
  EXPECT_NE(static_cast<const void*>(before.nearest_offsets.get()),
            static_cast<const void*>(&map.getMapConfig().spherical_neighbor));
}

TEST(RogMapPlanningGridExport, CircularLogicalOrderingMatchesAfterSignedAxisSlides) {
  TestRogMap map;
  map.loadConfigAndInit(testConfigPath());
  const auto& config = map.getMapConfig();
  const std::array<Eigen::Vector3d, 6> slide_poses{
      Eigen::Vector3d{2.1, 0.0, 0.0}, Eigen::Vector3d{-2.1, 0.0, 0.0},
      Eigen::Vector3d{0.0, 2.1, 0.0}, Eigen::Vector3d{0.0, -2.1, 0.0},
      Eigen::Vector3d{0.0, 0.0, 2.1}, Eigen::Vector3d{0.0, 0.0, -2.1}};
  for (const auto& position : slide_poses) {
    const auto cloud = singlePointCloud(position + Eigen::Vector3d{1.0, 0.0, 0.0});
    EXPECT_EQ(map.updateMap(
                  cloud, rog_map::Pose{position, Eigen::Quaterniond::Identity()}),
              rog_map::MapUpdateOutcome::UPDATED);
    const auto exported = map.exportPlanningGrid();
    const Eigen::Vector3i base_max =
        exported.base_layout.global_min_index + exported.base_layout.dimensions;
    for (int x = exported.base_layout.global_min_index.x(); x < base_max.x(); ++x) {
      for (int y = exported.base_layout.global_min_index.y(); y < base_max.y(); ++y) {
        for (int z = exported.base_layout.global_min_index.z(); z < base_max.z(); ++z) {
          const Eigen::Vector3i index{x, y, z};
          const Eigen::Vector3d point = centerOf(exported.base_layout, index);
          EXPECT_EQ(exported.base_state[offsetOf(exported.base_layout, index)],
                    static_cast<std::uint8_t>(map.getGridType(point)))
              << "slide=" << position.transpose() << " index=" << index.transpose();
        }
      }
    }
    const Eigen::Vector3i inflated_max =
        exported.inflated.layout.global_min_index + exported.inflated.layout.dimensions;
    for (int x = exported.inflated.layout.global_min_index.x(); x < inflated_max.x(); ++x) {
      for (int y = exported.inflated.layout.global_min_index.y(); y < inflated_max.y(); ++y) {
        for (int z = exported.inflated.layout.global_min_index.z(); z < inflated_max.z(); ++z) {
          const Eigen::Vector3i index{x, y, z};
          const Eigen::Vector3d point = centerOf(exported.inflated.layout, index);
          if (point.z() <= config.virtual_ground_height ||
              point.z() >= config.virtual_ceil_height) {
            continue;
          }
          EXPECT_EQ(exported.inflated.occupied[offsetOf(exported.inflated.layout, index)] != 0U,
                    map.isOccupiedInflate(point))
              << "slide=" << position.transpose() << " index=" << index.transpose();
        }
      }
    }
  }
}

TEST(RogMapPlanningGridExport, EnabledVirtualPlaneRejectsWithoutMutatingMap) {
  TestRogMap map;
  map.loadConfigAndInit(testConfigPath());
  const auto before = map.exportPlanningGrid();
  const Eigen::Vector3d pose_position{0.0, 0.0, -6.0};
  const auto outcome = map.updateMap(
      singlePointCloud(pose_position + Eigen::Vector3d{1.0, 0.0, 0.0}),
      rog_map::Pose{pose_position, Eigen::Quaterniond::Identity()});
  const auto after = map.exportPlanningGrid();

  EXPECT_EQ(outcome, rog_map::MapUpdateOutcome::BELOW_GROUND);
  EXPECT_TRUE(after.virtual_ground_ceiling_enabled);
  EXPECT_EQ(after.base_layout.global_min_index, before.base_layout.global_min_index);
  EXPECT_EQ(after.base_state, before.base_state);
  EXPECT_EQ(after.inflated.occupied, before.inflated.occupied);
}

TEST(RogMapPlanningGridExport, DisabledVirtualPlanesAllowNegativeZAndMoveLocalWindow) {
  TestRogMap map;
  map.loadConfigAndInit(noVirtualPlanesConfigPath());
  const auto before = map.exportPlanningGrid();
  const Eigen::Vector3d pose_position{0.0, 0.0, -4.0};
  const auto outcome = map.updateMap(
      singlePointCloud(pose_position + Eigen::Vector3d{1.0, 0.0, 0.0}),
      rog_map::Pose{pose_position, Eigen::Quaterniond::Identity()});
  const auto after = map.exportPlanningGrid();

  EXPECT_EQ(outcome, rog_map::MapUpdateOutcome::UPDATED);
  EXPECT_EQ(map.lastDiagnostics().map_slide_count, 1U);
  EXPECT_FALSE(after.virtual_ground_ceiling_enabled);
  EXPECT_NE(after.base_layout.global_min_index.z(), before.base_layout.global_min_index.z());
  EXPECT_LT(after.virtual_ground_m, before.virtual_ground_m);
  EXPECT_LT(after.virtual_ceiling_m, before.virtual_ceiling_m);
  EXPECT_NE(map.getGridType(pose_position), rog_map::GridType::OCCUPIED);
  EXPECT_FALSE(map.isOccupiedInflate(pose_position));
  // The earlier detached value remains immutable after the vertical slide.
  EXPECT_EQ(before.base_layout.local_center_m, Eigen::Vector3d::Zero());
}

TEST(RogMapPlanningGridExport, FirstFrameClearClipsToFiniteMapWindow) {
  TestRogMap map;
  map.loadConfigAndInit(raycastingBoundaryConfigPath());

  const Eigen::Vector3d pose_position{0.0, 0.0, 2.9};
  const auto outcome = map.updateMap(
      singlePointCloud(pose_position + Eigen::Vector3d{2.0, 0.0, 0.0}),
      rog_map::Pose{pose_position, Eigen::Quaterniond::Identity()});

  ASSERT_EQ(outcome, rog_map::MapUpdateOutcome::UPDATED);
  // The body-clear sphere reaches above the finite local map.  Every point
  // retained inside the map must be known free, while an interior point that
  // was not cleared remains unknown.  Most importantly, the boundary
  // clipping must not corrupt the occupancy buffer.
  EXPECT_EQ(map.getGridType(Eigen::Vector3d{0.0, 0.0, 2.9}),
            rog_map::GridType::KNOWN_FREE);
  EXPECT_EQ(map.getInfGridType(Eigen::Vector3d{0.0, 0.0, 2.9}),
            rog_map::GridType::KNOWN_FREE);
  EXPECT_EQ(map.getGridType(Eigen::Vector3d{0.0, 0.0, 1.5}),
            rog_map::GridType::UNKNOWN);
}

TEST(RogMapPlanningGridExport, MapSlideClearsFreshBodyNeighborhood) {
  TestRogMap map;
  map.loadConfigAndInit(raycastingBoundaryConfigPath());

  const Eigen::Vector3d initial_pose{0.0, 0.0, 0.0};
  ASSERT_EQ(map.updateMap(
                singlePointCloud(initial_pose + Eigen::Vector3d{2.0, 0.0, 0.0}),
                rog_map::Pose{initial_pose, Eigen::Quaterniond::Identity()}),
            rog_map::MapUpdateOutcome::UPDATED);
  EXPECT_EQ(map.getGridType(initial_pose), rog_map::GridType::KNOWN_FREE);

  // This exceeds the 1.5 m slide threshold and exposes a new local window
  // around a takeoff-adjacent pose.  The newly exposed robot cell must be
  // known free, while a point outside the body-clear sphere remains unknown.
  const Eigen::Vector3d slid_pose{0.0, 0.0, 2.9};
  ASSERT_EQ(map.updateMap(
                singlePointCloud(slid_pose + Eigen::Vector3d{2.0, 0.0, 0.0}),
                rog_map::Pose{slid_pose, Eigen::Quaterniond::Identity()}),
            rog_map::MapUpdateOutcome::UPDATED);
  EXPECT_EQ(map.getGridType(slid_pose), rog_map::GridType::KNOWN_FREE);
  EXPECT_EQ(map.getInfGridType(slid_pose), rog_map::GridType::KNOWN_FREE);
  EXPECT_EQ(map.getGridType(Eigen::Vector3d{0.0, 0.0, 1.5}),
            rog_map::GridType::UNKNOWN);
}
