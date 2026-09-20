#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <set>

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
  EXPECT_EQ(result->source_ray_count, 4U);
  EXPECT_EQ(result->detected_no_return_count, 2U);
  EXPECT_EQ(result->selected_no_return_count, 2U);
  EXPECT_EQ(result->sampling_policy, kVisibilitySamplingFull);
  EXPECT_EQ(result->stamp_sec, 3);
  EXPECT_EQ(result->stamp_nanosec, 42U);
  EXPECT_NEAR(result->endpoints[0].x, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[0].y, 10.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[0].z, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[1].x, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[1].y, 0.0F, 1.0e-5F);
  EXPECT_NEAR(result->endpoints[1].z, 10.0F, 1.0e-5F);
}

TEST(VisibilityCloud, StratifiedSamplingIsDeterministicAndCoversTwoAxes) {
  auto scan = makeScan();
  scan.set_count(8U);
  scan.set_angle_step(0.1);
  scan.set_vertical_count(4U);
  scan.set_vertical_angle_step(0.1);
  scan.mutable_ranges()->Clear();
  for (int index = 0; index < 32; ++index) {
    scan.add_ranges(std::numeric_limits<double>::infinity());
  }
  const auto first = makeVisibilityCloud(scan, "livox_frame", 8U);
  const auto second = makeVisibilityCloud(scan, "livox_frame", 8U);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  ASSERT_EQ(first->endpoints.size(), 8U);
  EXPECT_EQ(first->detected_no_return_count, 32U);
  EXPECT_EQ(first->selected_no_return_count, 8U);
  EXPECT_EQ(first->sampling_cap, 8U);
  EXPECT_EQ(first->sampling_policy, kVisibilitySamplingStratified2D);
  ASSERT_EQ(first->endpoints.size(), second->endpoints.size());
  for (std::size_t index = 0U; index < first->endpoints.size(); ++index) {
    EXPECT_EQ(first->endpoints[index].source_ray_index,
              second->endpoints[index].source_ray_index);
  }
  EXPECT_EQ(first->endpoints.front().elevation_index, 0U);
  EXPECT_EQ(first->endpoints.back().elevation_index, 3U);
  const auto min_azimuth = std::min_element(
      first->endpoints.begin(), first->endpoints.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.azimuth_index < rhs.azimuth_index;
      });
  const auto max_azimuth = std::max_element(
      first->endpoints.begin(), first->endpoints.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.azimuth_index < rhs.azimuth_index;
      });
  ASSERT_NE(min_azimuth, first->endpoints.end());
  ASSERT_NE(max_azimuth, first->endpoints.end());
  EXPECT_LT(min_azimuth->azimuth_index, max_azimuth->azimuth_index);
  EXPECT_GE(max_azimuth->azimuth_index - min_azimuth->azimuth_index, 4U);
}

TEST(VisibilityCloud, UnequalRowsKeepCoverageAndExactCap) {
  auto scan = makeScan();
  scan.set_count(8U);
  scan.set_angle_step(0.1);
  scan.set_vertical_count(4U);
  scan.set_vertical_angle_step(0.1);
  scan.mutable_ranges()->Clear();
  for (std::size_t row = 0U; row < 4U; ++row) {
    const std::size_t no_return_count = 8U - row;
    for (std::size_t column = 0U; column < 8U; ++column) {
      scan.add_ranges(column < no_return_count
          ? std::numeric_limits<double>::infinity() : 2.0);
    }
  }
  const auto result = makeVisibilityCloud(scan, "livox_frame", 8U);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->detected_no_return_count, 26U);
  EXPECT_EQ(result->selected_no_return_count, 8U);
  std::set<std::uint32_t> rows;
  for (const auto& endpoint : result->endpoints) {
    EXPECT_LT(endpoint.source_ray_index, 32U);
    rows.insert(endpoint.elevation_index);
  }
  EXPECT_EQ(rows.size(), 4U);

  const auto small = makeVisibilityCloud(scan, "livox_frame", 2U);
  ASSERT_TRUE(small.has_value());
  EXPECT_EQ(small->selected_no_return_count, 2U);
  EXPECT_LE(small->endpoints.size(), 2U);
  std::set<std::uint32_t> small_rows;
  for (const auto& endpoint : small->endpoints) small_rows.insert(endpoint.elevation_index);
  ASSERT_EQ(small_rows.size(), 2U);
  EXPECT_GE(*small_rows.rbegin() - *small_rows.begin(), 2U);
}

TEST(VisibilityCloud, RejectsIncompleteFlattenedScan) {
  auto scan = makeScan();
  scan.mutable_ranges()->RemoveLast();
  EXPECT_FALSE(makeVisibilityCloud(scan, "livox_frame").has_value());
}

TEST(VisibilityCloud, RejectsUnexpectedFrame) {
  EXPECT_FALSE(makeVisibilityCloud(makeScan(), "other_frame").has_value());
}

TEST(VisibilityCloud, RejectsEndpointCapBeyondRayBound) {
  EXPECT_FALSE(makeVisibilityCloud(makeScan(), "livox_frame", 262145U).has_value());
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
