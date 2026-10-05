#include "fast_lio_ros/ros_imu_adapter.hpp"

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

RosImuAdapter::RosImuAdapter(std::string expected_frame,
                             ClockDomain clock_domain)
    : expected_frame_(std::move(expected_frame)),
      clock_domain_(clock_domain) {
  if (expected_frame_.empty()) {
    throw std::invalid_argument("IMU expected frame must not be empty");
  }
  if (toString(clock_domain_) == "unknown") {
    throw std::invalid_argument("IMU clock domain is invalid");
  }
}

ImuSample RosImuAdapter::convert(const sensor_msgs::msg::Imu& message) const {
  if (message.header.frame_id != expected_frame_) {
    throw std::invalid_argument("IMU frame does not match configured imu frame");
  }
  const Eigen::Vector3d gyro{message.angular_velocity.x, message.angular_velocity.y,
                             message.angular_velocity.z};
  const Eigen::Vector3d accel{message.linear_acceleration.x, message.linear_acceleration.y,
                              message.linear_acceleration.z};
  if (!gyro.allFinite() || !accel.allFinite()) {
    throw std::invalid_argument("IMU measurement contains non-finite values");
  }
  return ImuSample{
      RosTimeConverter::fromRos(message.header.stamp, clock_domain_),
      gyro, accel};
}

}  // namespace uav::nav::lio
