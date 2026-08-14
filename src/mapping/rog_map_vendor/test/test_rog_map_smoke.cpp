// Vendor-level smoke test: ROG-Map must construct, initialize from a YAML
// config, accept synthetic point clouds via the manual updateMap() API, and
// produce the expected occupied/free/unknown raycast semantics. This is a
// vendor sanity check; frame/transform correctness is covered at the
// navigation_mapping package level (see P1 test plan section 21).
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

TEST(RogMapVendorSmoke, RaycastGeometryFreeOccupiedUnknown) {
  TestRogMap map;
  map.loadConfigAndInit(testConfigPath());

  const rog_map::Vec3f sensor_origin(0.0, 0.0, 0.0);
  const rog_map::Vec3f obstacle(3.0, 0.0, 0.0);
  const rog_map::Pose pose(sensor_origin, Eigen::Quaterniond::Identity());

  // Repeated observations let the probabilistic log-odds filter cross the
  // occupied/free thresholds, matching real sequential-scan behavior.
  for (int i = 0; i < 8; ++i) {
    map.updateMap(singlePointCloud(obstacle), pose);
  }

  // Cell well before the obstacle along the ray -> free.
  EXPECT_TRUE(map.isKnownFree(rog_map::Vec3f(1.0, 0.0, 0.0)));
  // The endpoint itself -> occupied.
  EXPECT_TRUE(map.isOccupied(obstacle));
  // Space behind (beyond) the endpoint -> remains unknown (never raycast).
  EXPECT_TRUE(map.isUnknown(rog_map::Vec3f(4.5, 0.0, 0.0)));
}
