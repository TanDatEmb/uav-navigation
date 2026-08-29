#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <cstdint>
#include <mutex>
#include <string>

#include "fast_lio_core/sensor/lidar_scan.hpp"
#include "fast_lio_ros/ros_lidar_adapter.hpp"

namespace uav::nav::lio {

enum class LivoxTimestampPolicy {
  // Production policy for official livox_ros_driver2: header.stamp must equal
  // timebase, and timebase is the authoritative first-point time.
  kRequireHeaderMatchesTimebase,
  // Explicit legacy/offline policy. The header is validated structurally but
  // timebase remains authoritative when the two values differ.
  kTimebaseAuthoritative,
};

class RosLivoxCustomAdapter {
 public:
  RosLivoxCustomAdapter(
      std::string expected_frame, ClockDomain clock_domain,
      LivoxTimestampPolicy timestamp_policy =
          LivoxTimestampPolicy::kRequireHeaderMatchesTimebase,
      PointTimeConfig point_time = {});

  [[nodiscard]] LidarScan convert(
      const livox_ros_driver2::msg::CustomMsg& message) const;

 private:
  std::string expected_frame_;
  ClockDomain clock_domain_;
  LivoxTimestampPolicy timestamp_policy_;
  PointTimeConfig point_time_;
  mutable std::mutex state_mutex_;
  mutable std::int64_t previous_scan_start_ns_{-1};
};

}  // namespace uav::nav::lio
