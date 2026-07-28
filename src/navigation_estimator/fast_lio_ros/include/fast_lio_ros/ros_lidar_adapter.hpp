#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio_core/sensor/lidar_scan.hpp"

namespace uav::nav::lio {

enum class LidarTimingMode { kSimultaneousScan, kPerPoint };

class RosLidarAdapter {
 public:
  RosLidarAdapter(std::string expected_frame, LidarTimingMode timing_mode);
  [[nodiscard]] LidarScan convert(const sensor_msgs::msg::PointCloud2& message) const;

 private:
  std::string expected_frame_;
  LidarTimingMode timing_mode_;
};

}  // namespace uav::nav::lio
