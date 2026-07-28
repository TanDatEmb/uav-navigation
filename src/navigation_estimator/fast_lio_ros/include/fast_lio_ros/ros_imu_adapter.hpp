#pragma once

#include <sensor_msgs/msg/imu.hpp>

#include "fast_lio_core/sensor/imu_sample.hpp"

namespace uav::nav::lio {

class RosImuAdapter {
 public:
  explicit RosImuAdapter(std::string expected_frame);
  [[nodiscard]] ImuSample convert(const sensor_msgs::msg::Imu& message) const;

 private:
  std::string expected_frame_;
};

}  // namespace uav::nav::lio
