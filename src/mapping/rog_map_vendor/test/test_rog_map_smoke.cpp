
#include <gtest/gtest.h>
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
    map.updateMap(singlePointCloud(obstacle), pose);
  }

  EXPECT_FALSE(map.isKnownFree(rog_map::Vec3f(1.0, 0.0, 0.0)));
  EXPECT_FALSE(map.isLineKnownFree(sensor_origin, rog_map::Vec3f(1.0, 0.0, 0.0)));
  EXPECT_EQ(map.getInfGridType(rog_map::Vec3f(1.0, 0.0, 0.0)),
            rog_map::GridType::UNKNOWN);
  // The endpoint itself -> occupied.
  EXPECT_TRUE(map.isOccupied(obstacle));
  EXPECT_EQ(map.getInfGridType(obstacle), rog_map::GridType::OCCUPIED);
  // SUPER's endpoint-only visibility contract allows UNKNOWN, but the
  // inflated query must still reject an obstacle inside the vehicle tube even
  // when the base-grid centre line does not cross the occupied voxel.
  const rog_map::Vec3f offset_start(0.0, 0.25, 0.0);
  const rog_map::Vec3f offset_end(4.0, 0.25, 0.0);
  EXPECT_TRUE(map.isLineFree(offset_start, offset_end, false, false));
  EXPECT_FALSE(map.isLineFree(offset_start, offset_end, true, false));
  // Space behind (beyond) the endpoint -> remains unknown (never raycast).
  EXPECT_TRUE(map.isUnknown(rog_map::Vec3f(4.5, 0.0, 0.0)));
  EXPECT_FALSE(map.isLineKnownFree(sensor_origin, rog_map::Vec3f(4.5, 0.0, 0.0)));
}
