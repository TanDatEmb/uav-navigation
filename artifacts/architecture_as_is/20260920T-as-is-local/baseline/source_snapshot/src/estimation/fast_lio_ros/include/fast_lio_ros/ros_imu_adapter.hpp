#pragma once

#include <sensor_msgs/msg/imu.hpp>

#include "fast_lio_core/sensor/imu_sample.hpp"

namespace uav::nav::lio {

class RosImuAdapter {
 public:
  explicit RosImuAdapter(
      std::string expected_frame,
      ClockDomain clock_domain = ClockDomain::kRosTime);
  [[nodiscard]] ImuSample convert(const sensor_msgs::msg::Imu& message) const;

 private:
  std::string expected_frame_;
  ClockDomain clock_domain_;
};

}  // namespace uav::nav::lio
