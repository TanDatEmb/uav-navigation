#include <gtest/gtest.h>

#include <cstring>

#include "fast_lio_ros/ros_lidar_adapter.hpp"

namespace uav::nav::lio {

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
      RosLidarAdapter{"lidar_link", LidarTimingMode::kSimultaneousScan}.convert(cloud);
  ASSERT_EQ(scan.points.size(), 1U);
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

}  // namespace uav::nav::lio
