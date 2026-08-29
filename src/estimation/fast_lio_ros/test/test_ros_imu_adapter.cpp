#include <gtest/gtest.h>

#include "fast_lio_ros/ros_imu_adapter.hpp"

namespace uav::nav::lio {

TEST(RosImuAdapterTest, PreservesFluComponentsAndMeasurementTime) {
  sensor_msgs::msg::Imu message;
  message.header.frame_id = "livox_imu_frame";
  message.header.stamp.sec = 2;
  message.angular_velocity.x = 1.0;
  message.angular_velocity.y = 2.0;
  message.angular_velocity.z = 3.0;
  message.linear_acceleration.x = 4.0;
  message.linear_acceleration.y = 5.0;
  message.linear_acceleration.z = 6.0;
  const auto sample =
      RosImuAdapter{"livox_imu_frame", ClockDomain::kSensorTime}.convert(message);
  EXPECT_EQ(sample.time.nanoseconds(), 2'000'000'000LL);
  EXPECT_EQ(sample.time.clock_domain(), ClockDomain::kSensorTime);
  EXPECT_EQ(sample.angular_velocity_imu_rad_s, Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_EQ(sample.linear_acceleration_imu_m_s2, Eigen::Vector3d(4.0, 5.0, 6.0));
}

TEST(RosImuAdapterTest, RejectsUnexpectedFrame) {
  sensor_msgs::msg::Imu message;
  message.header.frame_id = "wrong";
  EXPECT_THROW(RosImuAdapter{"livox_imu_frame"}.convert(message), std::invalid_argument);
}

TEST(RosImuAdapterTest, RejectsInvalidConfiguration) {
  EXPECT_THROW((RosImuAdapter{"", ClockDomain::kRosTime}), std::invalid_argument);
  EXPECT_THROW((RosImuAdapter{"livox_imu_frame", static_cast<ClockDomain>(255)}),
               std::invalid_argument);
}

}  // namespace uav::nav::lio
