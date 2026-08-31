#include <gtest/gtest.h>

#include <cstring>
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

sensor_msgs::msg::PointCloud2 makeOrganizedCloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "livox_frame";
  cloud.header.stamp.sec = 4;
  cloud.header.stamp.nanosec = 24U;
  cloud.width = 2U;
  cloud.height = 2U;
  cloud.point_step = 12U;
  cloud.row_step = 24U;
  cloud.is_dense = false;
  for (std::uint32_t index = 0U; index < 3U; ++index) {
    sensor_msgs::msg::PointField field;
    field.name = index == 0U ? "x" : (index == 1U ? "y" : "z");
    field.offset = index * 4U;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1U;
    cloud.fields.push_back(field);
  }
  cloud.data.resize(48U);
  const float values[4][3] = {
      {1.0F, 0.0F, 0.0F},
      {std::numeric_limits<float>::infinity(),
       -std::numeric_limits<float>::infinity(),
       std::numeric_limits<float>::infinity()},
      {10.0F, 0.0F, 0.0F},
      {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F},
  };
  for (std::size_t point = 0U; point < 4U; ++point) {
    std::memcpy(cloud.data.data() + point * 12U, values[point], 12U);
  }
  return cloud;
}

TEST(VisibilityCloud, ReconstructsOnlyExplicitInfiniteOrganizedRays) {
  const OrganizedVisibilityConfig config{
      2U, 2U, 0.0, 1.5707963267948966, 0.0, 1.5707963267948966, 10.0};
  const auto result = makeVisibilityCloud(
      makeOrganizedCloud(), config, "livox_frame");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->endpoints.size(), 1U);
  EXPECT_EQ(result->source_ray_count, 4U);
  EXPECT_EQ(result->stamp_sec, 4);
  EXPECT_EQ(result->stamp_nanosec, 24U);
  EXPECT_NEAR(result->endpoints[0].x, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[0].y, 10.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[0].z, 0.0F, 1.0e-5F);
}

TEST(VisibilityCloud, RejectsMismatchedOrganizedGrid) {
  auto cloud = makeOrganizedCloud();
  const OrganizedVisibilityConfig config{
      720U, 28U, -3.14, 3.14, -0.12, 0.90, 40.0};
  EXPECT_FALSE(makeVisibilityCloud(cloud, config, "livox_frame").has_value());
}

}  // namespace uav::simulation
