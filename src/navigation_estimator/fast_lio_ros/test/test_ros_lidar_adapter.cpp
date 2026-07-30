#include <gtest/gtest.h>

#include <cstring>
#include <limits>

#include "fast_lio_ros/ros_lidar_adapter.hpp"

namespace uav::nav::lio {
namespace {

sensor_msgs::msg::PointCloud2 makeCloud(std::uint32_t width = 2U,
                                        std::uint32_t height = 1U,
                                        std::uint32_t padding = 0U) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar_link";
  cloud.header.stamp.sec = 1;
  cloud.height = height;
  cloud.width = width;
  cloud.point_step = 32;
  cloud.row_step = width * cloud.point_step + padding;
  cloud.fields = {
      sensor_msgs::msg::PointField{}.set__name("x").set__offset(0).set__datatype(
          sensor_msgs::msg::PointField::FLOAT32).set__count(1),
      sensor_msgs::msg::PointField{}.set__name("y").set__offset(4).set__datatype(
          sensor_msgs::msg::PointField::FLOAT32).set__count(1),
      sensor_msgs::msg::PointField{}.set__name("z").set__offset(8).set__datatype(
          sensor_msgs::msg::PointField::FLOAT32).set__count(1),
      sensor_msgs::msg::PointField{}.set__name("time").set__offset(12).set__datatype(
          sensor_msgs::msg::PointField::UINT32).set__count(1),
      sensor_msgs::msg::PointField{}.set__name("timestamp").set__offset(16).set__datatype(
          sensor_msgs::msg::PointField::FLOAT64).set__count(1)};
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * height);
  for (std::uint32_t row = 0; row < height; ++row) {
    for (std::uint32_t column = 0; column < width; ++column) {
      auto* point = cloud.data.data() + row * cloud.row_step + column * cloud.point_step;
      const float xyz[3]{1.0F + column, 2.0F, 3.0F};
      const std::uint32_t relative = column * 10U;
      const double absolute = 1'000'000'000.0 + relative;
      std::memcpy(point, xyz, sizeof(xyz));
      std::memcpy(point + 12, &relative, sizeof(relative));
      std::memcpy(point + 16, &absolute, sizeof(absolute));
    }
  }
  return cloud;
}

PointTimeConfig absoluteConfig() {
  return PointTimeConfig{
      "timestamp", PointTimeEncoding::kFloat64AbsoluteNanoseconds,
      ScanReference::kMinimumPointTime, 200'000'000, 200'000'000, true};
}

void setAbsolute(sensor_msgs::msg::PointCloud2& cloud, std::size_t index,
                 double value) {
  const auto row = index / cloud.width;
  const auto column = index % cloud.width;
  auto* point = cloud.data.data() + row * cloud.row_step +
                column * cloud.point_step;
  std::memcpy(point + 16, &value, sizeof(value));
}

void setX(sensor_msgs::msg::PointCloud2& cloud, std::size_t index,
          float value) {
  const auto row = index / cloud.width;
  const auto column = index % cloud.width;
  auto* point = cloud.data.data() + row * cloud.row_step +
                column * cloud.point_step;
  std::memcpy(point, &value, sizeof(value));
}

}  // namespace

TEST(RosLidarAdapterTest, SimultaneousScanDoesNotInventPointTime) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar_link";
  cloud.height = 1;
  cloud.width = 1;
  cloud.point_step = 12;
  cloud.row_step = 12;
  cloud.fields = {sensor_msgs::msg::PointField{}
                      .set__name("x")
                      .set__offset(0)
                      .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
                      .set__count(1),
                  sensor_msgs::msg::PointField{}
                      .set__name("y")
                      .set__offset(4)
                      .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
                      .set__count(1),
                  sensor_msgs::msg::PointField{}
                      .set__name("z")
                      .set__offset(8)
                      .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
                      .set__count(1)};
  cloud.data.resize(12);
  const float coordinates[3]{1.0F, 2.0F, 3.0F};
  std::memcpy(cloud.data.data(), coordinates, sizeof(coordinates));
  const auto scan =
      RosLidarAdapter{"lidar_link", LidarTimingMode::kSimultaneousScan,
                      ClockDomain::kSimulationTime}
          .convert(cloud);
  ASSERT_EQ(scan.points.size(), 1U);
  EXPECT_EQ(scan.start_time.clock_domain(), ClockDomain::kSimulationTime);
  EXPECT_EQ(scan.points.front().relative_time_ns, 0U);
  EXPECT_FALSE(scan.has_per_point_time);
}

TEST(RosLidarAdapterTest, RealModeRequiresPerPointTime) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar_link";
  cloud.height = 1;
  cloud.width = 0;
  cloud.point_step = 12;
  cloud.fields = {sensor_msgs::msg::PointField{}
                      .set__name("x")
                      .set__offset(0)
                      .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
                      .set__count(1),
                  sensor_msgs::msg::PointField{}
                      .set__name("y")
                      .set__offset(4)
                      .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
                      .set__count(1),
                  sensor_msgs::msg::PointField{}
                      .set__name("z")
                      .set__offset(8)
                      .set__datatype(sensor_msgs::msg::PointField::FLOAT32)
                      .set__count(1)};
  const RosLidarAdapter adapter{"lidar_link", LidarTimingMode::kPerPoint};
  EXPECT_THROW(static_cast<void>(adapter.convert(cloud)), std::invalid_argument);
}

TEST(RosLidarAdapterTest, AcceptsConfiguredUint32RelativeNanoseconds) {
  const auto scan =
      RosLidarAdapter{"lidar_link", LidarTimingMode::kPerPoint}.convert(makeCloud());
  ASSERT_EQ(scan.points.size(), 2U);
  EXPECT_EQ(scan.points[1].relative_time_ns, 10U);
}

TEST(RosLidarAdapterTest, AcceptsFloat64AbsoluteTimestampAndMinimumReference) {
  auto cloud = makeCloud();
  setAbsolute(cloud, 0, 1'000'000'020.0);
  setAbsolute(cloud, 1, 1'000'000'000.0);
  const auto scan =
      RosLidarAdapter{"lidar_link", LidarTimingMode::kPerPoint,
                      ClockDomain::kRosTime, absoluteConfig()}
          .convert(cloud);
  EXPECT_EQ(scan.start_time.nanoseconds(), 1'000'000'000);
  EXPECT_EQ(scan.points[0].relative_time_ns, 20U);
  EXPECT_EQ(scan.points[1].relative_time_ns, 0U);
}

TEST(RosLidarAdapterTest, PreservesTimestampIndexWhenNanXyzIsRemoved) {
  auto cloud = makeCloud(3);
  setAbsolute(cloud, 0, 1'000'000'000.0);
  setAbsolute(cloud, 1, 1'000'000'010.0);
  setAbsolute(cloud, 2, 1'000'000'020.0);
  setX(cloud, 1, std::numeric_limits<float>::quiet_NaN());
  const auto scan =
      RosLidarAdapter{"lidar_link", LidarTimingMode::kPerPoint,
                      ClockDomain::kRosTime, absoluteConfig()}
          .convert(cloud);
  ASSERT_EQ(scan.points.size(), 2U);
  EXPECT_EQ(scan.points[0].relative_time_ns, 0U);
  EXPECT_EQ(scan.points[1].relative_time_ns, 20U);
}

TEST(RosLidarAdapterTest, DeterministicallyTrimsBoundedBoundaryOverlap) {
  auto config = absoluteConfig();
  config.maximum_boundary_overlap_ns = 20;
  config.minimum_points_after_overlap_trim = 1;
  RosLidarAdapter adapter{"lidar_link", LidarTimingMode::kPerPoint,
                          ClockDomain::kRosTime, config};
  auto first = makeCloud();
  setAbsolute(first, 0, 1'000'000'000.0);
  setAbsolute(first, 1, 1'000'000'010.0);
  static_cast<void>(adapter.convert(first));
  auto second = makeCloud();
  setAbsolute(second, 0, 1'000'000'005.0);
  setAbsolute(second, 1, 1'000'000'020.0);
  const auto scan = adapter.convert(second);
  ASSERT_EQ(scan.points.size(), 1U);
  EXPECT_EQ(scan.start_time.nanoseconds(), 1'000'000'020);
  EXPECT_EQ(scan.points.front().relative_time_ns, 0U);
  const auto stats = adapter.normalizationStatistics();
  EXPECT_EQ(stats.input_point_count, 4U);
  EXPECT_EQ(stats.emitted_point_count, 3U);
  EXPECT_EQ(stats.dropped_overlapping_point_count, 1U);
}

TEST(RosLidarAdapterTest, RejectsNonFiniteAndOutOfRangeAbsoluteTimestamp) {
  for (const double invalid :
       {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), 1.0e20}) {
    auto cloud = makeCloud();
    setAbsolute(cloud, 0, invalid);
    EXPECT_THROW(
        RosLidarAdapter("lidar_link", LidarTimingMode::kPerPoint,
                        ClockDomain::kRosTime, absoluteConfig()).convert(cloud),
        std::invalid_argument);
  }
}

TEST(RosLidarAdapterTest, RejectsNegativeRelativeTimestampWithHeaderReference) {
  auto config = absoluteConfig();
  config.scan_reference = ScanReference::kHeaderStamp;
  auto cloud = makeCloud();
  setAbsolute(cloud, 0, 999'999'999.0);
  EXPECT_THROW(
      RosLidarAdapter("lidar_link", LidarTimingMode::kPerPoint,
                      ClockDomain::kRosTime, config).convert(cloud),
      std::invalid_argument);
}

TEST(RosLidarAdapterTest, RejectsDurationAndHeaderOffsetBeyondLimits) {
  auto cloud = makeCloud();
  setAbsolute(cloud, 1, 1'300'000'000.0);
  EXPECT_THROW(
      RosLidarAdapter("lidar_link", LidarTimingMode::kPerPoint,
                      ClockDomain::kRosTime, absoluteConfig()).convert(cloud),
      std::invalid_argument);
  auto config = absoluteConfig();
  config.maximum_header_offset_ns = 5;
  setAbsolute(cloud, 0, 1'000'000'010.0);
  setAbsolute(cloud, 1, 1'000'000'020.0);
  EXPECT_THROW(
      RosLidarAdapter("lidar_link", LidarTimingMode::kPerPoint,
                      ClockDomain::kRosTime, config).convert(cloud),
      std::invalid_argument);
}

TEST(RosLidarAdapterTest, SupportsOrganizedCloudWithRowPadding) {
  const auto scan =
      RosLidarAdapter{"lidar_link", LidarTimingMode::kPerPoint}
          .convert(makeCloud(2, 2, 16));
  EXPECT_EQ(scan.points.size(), 4U);
}

TEST(RosLidarAdapterTest, RejectsInvalidStorageAndEmptyCloud) {
  auto cloud = makeCloud();
  cloud.row_step = cloud.width * cloud.point_step - 1;
  EXPECT_THROW(RosLidarAdapter("lidar_link", LidarTimingMode::kPerPoint).convert(cloud),
               std::invalid_argument);
  cloud = makeCloud();
  cloud.point_step = 8;
  EXPECT_THROW(RosLidarAdapter("lidar_link", LidarTimingMode::kPerPoint).convert(cloud),
               std::invalid_argument);
  cloud = makeCloud(0);
  EXPECT_THROW(RosLidarAdapter("lidar_link", LidarTimingMode::kPerPoint).convert(cloud),
               std::invalid_argument);
}

TEST(RosLidarAdapterTest, RejectsFrameMismatch) {
  auto cloud = makeCloud();
  cloud.header.frame_id = "wrong";
  EXPECT_THROW(RosLidarAdapter("lidar_link", LidarTimingMode::kPerPoint).convert(cloud),
               std::invalid_argument);
}

TEST(RosLidarAdapterTest, RejectsScanLevelTimestampRegression) {
  RosLidarAdapter adapter{"lidar_link", LidarTimingMode::kPerPoint,
                          ClockDomain::kRosTime, absoluteConfig()};
  auto first = makeCloud();
  setAbsolute(first, 0, 1'000'000'100.0);
  setAbsolute(first, 1, 1'000'000'110.0);
  static_cast<void>(adapter.convert(first));
  EXPECT_THROW(static_cast<void>(adapter.convert(makeCloud())), std::invalid_argument);
}

}  // namespace uav::nav::lio
