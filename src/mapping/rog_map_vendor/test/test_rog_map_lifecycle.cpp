// Regression test for the P1 lifecycle patch (see UPSTREAM.md, "Lifecycle
// fix"). Upstream's original function-local static init guard made a second
// ROGMap instance in the same process always throw, which is incompatible
// with the public-frame-generation reset contract (destroy + reconstruct in
// the same mapper process). This test fails if that regresses.
#include <gtest/gtest.h>

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "test_rog_map_fixture.hpp"

namespace {

using navigation_mapping::test::TestRogMap;

std::string testConfigPath() {
  return std::string(ROG_MAP_VENDOR_TEST_FIXTURE_DIR) + "/rog_map_test.yaml";
}

}  // namespace

TEST(RogMapVendorLifecycle, CanResetAndReinitializeRepeatedlyInSameProcess) {
  for (int generation = 0; generation < 5; ++generation) {
    auto map = std::make_unique<TestRogMap>();
    ASSERT_NO_THROW(map->loadConfigAndInit(testConfigPath()))
        << "generation " << generation;
    // The reconstructed map must be immediately usable, not merely
    // constructible.
    const rog_map::Pose pose(rog_map::Vec3f(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
    rog_map::PointCloud cloud;
    pcl::PointXYZI p;
    p.x = 2.0F;
    p.y = 0.0F;
    p.z = 0.0F;
    cloud.push_back(p);
    EXPECT_NO_THROW(map->updateMap(cloud, pose)) << "generation " << generation;
    // Destroyed at end of scope; the next iteration constructs a fresh
    // instance in the same process.
  }
}

TEST(RogMapVendorLifecycle, SameInstanceDoubleInitStillRejected) {
  TestRogMap map;
  map.loadConfigAndInit(testConfigPath());
  EXPECT_THROW(map.init(), std::runtime_error);
}
