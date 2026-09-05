#include <gtest/gtest.h>

#include <chrono>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

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

void expectPatchMatchesFull(const rog_map::PlanningGridPatchExport& patch,
                            const rog_map::PlanningGridExport& full) {
  ASSERT_FALSE(patch.base_state.empty());
  ASSERT_FALSE(patch.inflated.occupied.empty());
  const Eigen::Vector3i base_max =
      patch.base_layout.global_min_index + patch.base_layout.dimensions;
  for (int x = patch.base_layout.global_min_index.x(); x < base_max.x(); ++x) {
    for (int y = patch.base_layout.global_min_index.y(); y < base_max.y(); ++y) {
      for (int z = patch.base_layout.global_min_index.z(); z < base_max.z(); ++z) {
        const Eigen::Vector3i index{x, y, z};
        EXPECT_EQ(patch.base_state[offsetOf(patch.base_layout, index)],
                  full.base_state[offsetOf(full.base_layout, index)])
            << "base index=" << index.transpose();
      }
    }
  }
  const Eigen::Vector3i inflated_max =
      patch.inflated.layout.global_min_index + patch.inflated.layout.dimensions;
  for (int x = patch.inflated.layout.global_min_index.x(); x < inflated_max.x(); ++x) {
    for (int y = patch.inflated.layout.global_min_index.y(); y < inflated_max.y(); ++y) {
      for (int z = patch.inflated.layout.global_min_index.z(); z < inflated_max.z(); ++z) {
        const Eigen::Vector3i index{x, y, z};
        const auto patch_offset = offsetOf(patch.inflated.layout, index);
        const auto full_offset = offsetOf(full.inflated.layout, index);
        EXPECT_EQ(patch.inflated.occupied[patch_offset],
                  full.inflated.occupied[full_offset])
            << "inflated occupied index=" << index.transpose();
        if (!patch.inflated.unknown.empty()) {
          ASSERT_FALSE(full.inflated.unknown.empty());
          EXPECT_EQ(patch.inflated.unknown[patch_offset],
                    full.inflated.unknown[full_offset])
              << "inflated unknown index=" << index.transpose();
        }
      }
    }
  }
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

TEST(RogMapPlanningGridExport, RegionMatchesFullExportAfterSignedAxisSlides) {
  TestRogMap map;
  map.loadConfigAndInit(testConfigPath());
  const std::array<Eigen::Vector3d, 6> slide_poses{
      Eigen::Vector3d{2.1, 0.0, 0.0}, Eigen::Vector3d{-2.1, 0.0, 0.0},
      Eigen::Vector3d{0.0, 2.1, 0.0}, Eigen::Vector3d{0.0, -2.1, 0.0},
      Eigen::Vector3d{0.0, 0.0, 2.1}, Eigen::Vector3d{0.0, 0.0, -2.1}};
  for (const auto& position : slide_poses) {
    const auto cloud = singlePointCloud(position + Eigen::Vector3d{1.0, 0.3, 0.2});
    EXPECT_EQ(map.updateMap(
                  cloud, rog_map::Pose{position, Eigen::Quaterniond::Identity()}),
              rog_map::MapUpdateOutcome::UPDATED);
    const auto full = map.exportPlanningGrid();
    const auto patch = map.exportPlanningGridRegion(
        position - Eigen::Vector3d{1.35, 1.15, 1.05},
        position + Eigen::Vector3d{1.25, 1.45, 1.35});
    const auto estimate = map.estimatePlanningGridRegionSize(
        position - Eigen::Vector3d{1.35, 1.15, 1.05},
        position + Eigen::Vector3d{1.25, 1.45, 1.35});
    ASSERT_TRUE(estimate.valid);
    EXPECT_EQ(estimate.base_cell_count, patch.base_state.size());
    EXPECT_EQ(estimate.inflated_cell_count, patch.inflated.occupied.size());
    EXPECT_EQ(estimate.owned_byte_count,
              patch.base_state.size() + patch.inflated.occupied.size() +
                  patch.inflated.unknown.size());
    expectPatchMatchesFull(patch, full);
  }
}

TEST(RogMapPlanningGridExport, RegionSizeEstimateRejectsMalformedOrEmptyBounds) {
  TestRogMap map;
  map.loadConfigAndInit(testConfigPath());

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(map.estimatePlanningGridRegionSize(
                      Eigen::Vector3d{nan, 0.0, 0.0},
                      Eigen::Vector3d{1.0, 1.0, 1.0})
                   .valid);
  EXPECT_FALSE(map.estimatePlanningGridRegionSize(
                      Eigen::Vector3d{1.0, 1.0, 1.0},
                      Eigen::Vector3d{-1.0, -1.0, -1.0})
                   .valid);
  EXPECT_FALSE(map.estimatePlanningGridRegionSize(
                      Eigen::Vector3d{1'000'000.0, 1'000'000.0, 1'000'000.0},
                      Eigen::Vector3d{1'000'001.0, 1'000'001.0, 1'000'001.0})
                   .valid);
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

TEST(RogMapPlanningGridExport, SensorRaycastDoesNotFabricateBodyFreeSpace) {
  TestRogMap map;
  map.loadConfigAndInit(raycastingBoundaryConfigPath());

  const Eigen::Vector3d pose_position{0.0, 0.0, 2.9};
  const auto outcome = map.updateMap(
      singlePointCloud(pose_position + Eigen::Vector3d{2.0, 0.0, 0.0}),
      rog_map::Pose{pose_position, Eigen::Quaterniond::Identity()});

  ASSERT_EQ(outcome, rog_map::MapUpdateOutcome::UPDATED);
  // A sensor minimum range is not vehicle-presence evidence.  The endpoint
  // ray still contributes its own sensor free cells, but the base pose cell
  // is not cleared by a synthetic sphere.
  EXPECT_EQ(map.getGridType(Eigen::Vector3d{0.0, 0.0, 2.9}),
            rog_map::GridType::UNKNOWN);
  EXPECT_EQ(map.getInfGridType(Eigen::Vector3d{0.0, 0.0, 2.9}),
            rog_map::GridType::UNKNOWN);
  EXPECT_EQ(map.getGridType(Eigen::Vector3d{0.0, 0.0, 1.5}),
            rog_map::GridType::UNKNOWN);
}

TEST(RogMapPlanningGridExport, MapSlideDoesNotClearFreshBodyNeighborhood) {
  TestRogMap map;
  map.loadConfigAndInit(raycastingBoundaryConfigPath());

  const Eigen::Vector3d initial_pose{0.0, 0.0, 0.0};
  ASSERT_EQ(map.updateMap(
                singlePointCloud(initial_pose + Eigen::Vector3d{2.0, 0.0, 0.0}),
                rog_map::Pose{initial_pose, Eigen::Quaterniond::Identity()}),
            rog_map::MapUpdateOutcome::UPDATED);
  EXPECT_EQ(map.getGridType(initial_pose), rog_map::GridType::UNKNOWN);

  // This exceeds the 1.5 m slide threshold and exposes a new local window.
  // Sliding must not turn the new base pose into probabilistic free evidence.
  const Eigen::Vector3d slid_pose{0.0, 0.0, 2.9};
  ASSERT_EQ(map.updateMap(
                singlePointCloud(slid_pose + Eigen::Vector3d{2.0, 0.0, 0.0}),
                rog_map::Pose{slid_pose, Eigen::Quaterniond::Identity()}),
            rog_map::MapUpdateOutcome::UPDATED);
  EXPECT_EQ(map.getGridType(slid_pose), rog_map::GridType::UNKNOWN);
  EXPECT_EQ(map.getInfGridType(slid_pose), rog_map::GridType::UNKNOWN);
  EXPECT_EQ(map.getGridType(Eigen::Vector3d{0.0, 0.0, 1.5}),
            rog_map::GridType::UNKNOWN);
}

TEST(RogMapPlanningGridExport, RefreshesBodyNeighborhoodAsPoseMovesWithinWindow) {
  TestRogMap map;
  map.loadConfigAndInit(raycastingBoundaryConfigPath());

  const Eigen::Vector3d initial_pose{0.0, 0.0, 0.0};
  ASSERT_EQ(map.updateMap(
                singlePointCloud(Eigen::Vector3d{0.0, 2.0, 0.0}),
                rog_map::Pose{initial_pose, Eigen::Quaterniond::Identity()}),
            rog_map::MapUpdateOutcome::UPDATED);
  EXPECT_EQ(map.getGridType(Eigen::Vector3d{1.0, 0.0, 0.0}),
            rog_map::GridType::UNKNOWN);

  // This pose remains inside the existing sliding window. Its sensor-minimum
  // neighborhood was not cleared by the first frame, so a one-metre motion
  // must refresh the execution anchor without relying on a map slide.
  const Eigen::Vector3d moved_pose{1.0, 0.0, 0.0};
  ASSERT_EQ(map.updateMap(
                singlePointCloud(Eigen::Vector3d{1.0, 2.0, 0.0}),
                rog_map::Pose{moved_pose, Eigen::Quaterniond::Identity()}),
            rog_map::MapUpdateOutcome::UPDATED);
  EXPECT_EQ(map.getGridType(moved_pose), rog_map::GridType::UNKNOWN);
  EXPECT_EQ(map.getInfGridType(moved_pose), rog_map::GridType::UNKNOWN);
  EXPECT_EQ(map.lastDiagnostics().body_neighborhood_cells_cleared, 0U);
  EXPECT_LE(map.lastDiagnostics().changed_region_min.x(), moved_pose.x() + 1.0e-5);
  EXPECT_GE(map.lastDiagnostics().changed_region_max.x(), moved_pose.x() - 1.0e-5);
}

TEST(RogMapPlanningGridExport, OccupiedEvidenceIsNotErasedByVehiclePresence) {
  TestRogMap map;
  map.loadConfigAndInit(raycastingBoundaryConfigPath());
  const Eigen::Vector3d base{0.0, 0.0, 0.0};
  const Eigen::Vector3d obstacle{0.8, 0.0, 0.0};
  ASSERT_EQ(map.updateMap(singlePointCloud(obstacle),
                          rog_map::Pose{base, Eigen::Quaterniond::Identity()}),
            rog_map::MapUpdateOutcome::UPDATED);
  EXPECT_EQ(map.getGridType(obstacle), rog_map::GridType::OCCUPIED);
  EXPECT_EQ(map.getInfGridType(obstacle), rog_map::GridType::OCCUPIED);
  EXPECT_EQ(map.lastDiagnostics().body_neighborhood_cells_cleared, 0U);
}

TEST(RogMapPlanningGridExport, RaycastUsesSensorOriginNotBaseOrigin) {
  TestRogMap map;
  map.loadConfigAndInit(raycastingBoundaryConfigPath());
  const Eigen::Vector3d base{0.0, 0.0, 0.0};
  const rog_map::Vec3f sensor_origin{0.5F, 0.0F, 0.0F};
  const Eigen::Vector3d obstacle{3.0, 0.0, 0.0};
  ASSERT_EQ(map.updateMap(singlePointCloud(obstacle),
                          rog_map::Pose{base, Eigen::Quaterniond::Identity()},
                          sensor_origin),
            rog_map::MapUpdateOutcome::UPDATED);
  EXPECT_EQ(map.getGridType(Eigen::Vector3d{0.4, 0.0, 0.0}),
            rog_map::GridType::UNKNOWN);
  EXPECT_EQ(map.getGridType(Eigen::Vector3d{1.5, 0.0, 0.0}),
            rog_map::GridType::KNOWN_FREE);
}
