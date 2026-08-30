#include <gtest/gtest.h>

#include <limits>

#include <gz/msgs/laserscan.pb.h>

#include "uav_simulation/visibility_cloud.hpp"

namespace uav::simulation {

gz::msgs::LaserScan makeScan() {
  gz::msgs::LaserScan scan;
  scan.set_frame("livox_frame");
  scan.mutable_header()->mutable_stamp()->set_sec(3);
  scan.mutable_header()->mutable_stamp()->set_nsec(42);
  scan.set_angle_min(0.0);
  scan.set_angle_max(1.5707963267948966);
  scan.set_angle_step(1.5707963267948966);
  scan.set_count(2U);
  scan.set_vertical_angle_min(0.0);
  scan.set_vertical_angle_step(1.5707963267948966);
  scan.set_vertical_count(2U);
  scan.set_range_min(0.1);
  scan.set_range_max(10.0);
  scan.add_ranges(2.0);
  scan.add_ranges(std::numeric_limits<double>::infinity());
  scan.add_ranges(10.0);
  scan.add_ranges(2.0);
  return scan;
}

TEST(VisibilityCloud, ConvertsOnlyExplicitNoReturnRaysInThreeDimensions) {
  const auto result = makeVisibilityCloud(makeScan(), "livox_frame");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->endpoints.size(), 2U);
  EXPECT_EQ(result->stamp_sec, 3);
  EXPECT_EQ(result->stamp_nanosec, 42U);
  EXPECT_NEAR(result->endpoints[0].x, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[0].y, 10.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[0].z, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[1].x, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[1].y, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[1].z, 10.0F, 1.0e-5F);
}

TEST(VisibilityCloud, RejectsIncompleteFlattenedScan) {
  auto scan = makeScan();
  scan.mutable_ranges()->RemoveLast();
  EXPECT_FALSE(makeVisibilityCloud(scan, "livox_frame").has_value());
}

TEST(VisibilityCloud, RejectsUnexpectedFrame) {
  EXPECT_FALSE(makeVisibilityCloud(makeScan(), "other_frame").has_value());
}

TEST(VisibilityCloud, DoesNotTurnAllOccupiedScanIntoEvidence) {
  auto scan = makeScan();
  scan.mutable_ranges()->Clear();
  for (int index = 0; index < 4; ++index) scan.add_ranges(2.0);
  const auto result = makeVisibilityCloud(scan, "livox_frame");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->source_ray_count, 4U);
  EXPECT_TRUE(result->endpoints.empty());
}

}  // namespace uav::simulation
