
#include <gtest/gtest.h>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "test_rog_map_fixture.hpp"

namespace {

using navigation_mapping::test::TestRogMap;

std::string testConfigPath() {
  return std::string(ROG_MAP_VENDOR_TEST_FIXTURE_DIR) + "/rog_map_test.yaml";
}

rog_map::PointCloud singlePointCloud(const rog_map::Vec3f& point) {
  rog_map::PointCloud cloud;
  pcl::PointXYZI p;
  p.x = static_cast<float>(point.x());
  p.y = static_cast<float>(point.y());
  p.z = static_cast<float>(point.z());
  p.intensity = 0.0F;
  cloud.push_back(p);
  return cloud;
}

rog_map::PointCloud noReturnEndpoints(const rog_map::Vec3f& point) {
  return singlePointCloud(point);
}

}  // namespace

TEST(RogMapVendorSmoke, ConstructAndInit) {
  TestRogMap map;
  ASSERT_NO_THROW(map.loadConfigAndInit(testConfigPath()));
}

TEST(RogMapVendorSmoke, EndpointOnlyGeometryOccupiedUnknown) {
  TestRogMap map;
  map.loadConfigAndInit(testConfigPath());

  const rog_map::Vec3f sensor_origin(0.0, 0.0, 0.0);
  const rog_map::Vec3f obstacle(3.0, 0.0, 0.0);
  const rog_map::Pose pose(sensor_origin, Eigen::Quaterniond::Identity());

  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(map.updateMap(singlePointCloud(obstacle), pose),
              rog_map::MapUpdateOutcome::UPDATED);
  }

  EXPECT_FALSE(map.isKnownFree(rog_map::Vec3f(1.0, 0.0, 0.0)));
  EXPECT_FALSE(map.isLineKnownFree(sensor_origin, rog_map::Vec3f(1.0, 0.0, 0.0)));
  EXPECT_EQ(map.getInfGridType(rog_map::Vec3f(1.0, 0.0, 0.0)),
            rog_map::GridType::UNKNOWN);
  // The endpoint itself -> occupied.
  EXPECT_TRUE(map.isOccupied(obstacle));
  EXPECT_EQ(map.getInfGridType(obstacle), rog_map::GridType::OCCUPIED);
  // The endpoint-only visibility contract allows UNKNOWN, but the
  // inflated query must still reject an obstacle inside the vehicle tube even
  // when the base-grid centre line does not cross the occupied voxel.
  const rog_map::Vec3f offset_start(0.0, 0.25, 0.0);
  const rog_map::Vec3f offset_end(4.0, 0.25, 0.0);
  EXPECT_TRUE(map.isLineFree(offset_start, offset_end, false, false));
  EXPECT_FALSE(map.isLineFree(offset_start, offset_end, true, false));
  // Space behind (beyond) the endpoint -> remains unknown (never raycast).
  EXPECT_TRUE(map.isUnknown(rog_map::Vec3f(4.5, 0.0, 0.0)));
  EXPECT_FALSE(map.isLineKnownFree(sensor_origin, rog_map::Vec3f(4.5, 0.0, 0.0)));
  EXPECT_FALSE(map.isLineFree(sensor_origin, rog_map::Vec3f(100.0, 0.0, 0.0),
                              true, false));
  EXPECT_FALSE(map.isLineKnownFree(obstacle, obstacle));
  EXPECT_FALSE(map.isLineKnownFree(rog_map::Vec3f(100.0, 0.0, 0.0),
                                   rog_map::Vec3f(100.0, 0.0, 0.0)));
  // Same-cell queries must still classify the endpoint; a skipped DDA is not
  // permission to accept an occupied cell.
  EXPECT_FALSE(map.isLineFree(obstacle, obstacle, false, false));
  EXPECT_TRUE(map.isLineFree(sensor_origin, sensor_origin, false, false));
  rog_map::Vec3f same_cell_goal;
  const rog_map::vec_Vec3i empty_neighbors;
  EXPECT_FALSE(map.isLineFree(obstacle, obstacle, same_cell_goal, 0.0,
                              empty_neighbors));
  EXPECT_TRUE(map.isLineFree(sensor_origin, sensor_origin, same_cell_goal, 0.0,
                             empty_neighbors));
  EXPECT_FALSE(map.isLineFree(sensor_origin, rog_map::Vec3f(2.0, 0.0, 0.0), 1.0,
                              empty_neighbors));
  EXPECT_FALSE(map.isLineFree(sensor_origin,
                              rog_map::Vec3f(std::numeric_limits<float>::infinity(), 0.0F, 0.0F),
                              false, false));
  const rog_map::Vec3f huge_point(
      std::numeric_limits<float>::max(), 0.0F, 0.0F);
  EXPECT_FALSE(map.isLineKnownFree(huge_point, huge_point));
}

TEST(RogMapVendorSmoke, ExplicitNoReturnEndpointAddsMissOnlyEvidence) {
  TestRogMap map;
  map.loadConfigAndInit(
      std::string(ROG_MAP_VENDOR_TEST_FIXTURE_DIR) +
      "/rog_map_raycasting_boundary.yaml");

  const rog_map::Vec3f sensor_origin(0.0, 0.0, 0.0);
  const rog_map::Pose pose(sensor_origin, Eigen::Quaterniond::Identity());
  const auto free_endpoint = noReturnEndpoints(rog_map::Vec3f(4.0, 0.0, 0.0));
  rog_map::PointCloud hits;
  for (int i = 0; i < 30; ++i) {
    EXPECT_EQ(map.updateMap(hits, free_endpoint, pose),
              rog_map::MapUpdateOutcome::UPDATED);
  }

  const auto& diagnostics = map.lastDiagnostics();
  EXPECT_EQ(diagnostics.endpoint_count, 0U);
  EXPECT_EQ(diagnostics.free_space_endpoint_count, 1U);
  EXPECT_EQ(diagnostics.free_space_attempt_count, 1U);
  EXPECT_EQ(diagnostics.free_space_processed_count, 1U);
  EXPECT_EQ(diagnostics.free_space_skipped_count, 0U);
  EXPECT_LT(map.getMapValue(rog_map::Vec3f(1.0, 0.0, 0.0)), -0.0039);
  // The DDA intentionally excludes the endpoint voxel itself.  A voxel
  // strictly before the endpoint must nevertheless become known free.
  EXPECT_TRUE(map.isKnownFree(rog_map::Vec3f(1.0, 0.0, 0.0)));
  EXPECT_FALSE(map.isOccupied(rog_map::Vec3f(4.0, 0.0, 0.0)));
  EXPECT_TRUE(map.isLineKnownFree(rog_map::Vec3f(1.0, 0.0, 0.0),
                                  rog_map::Vec3f(1.0, 0.0, 0.0)));
}
