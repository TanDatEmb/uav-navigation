#pragma once

#include <cstdint>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio_core/sensor/lidar_scan.hpp"

namespace uav::nav::lio {

enum class LidarTimingMode { kSimultaneousScan, kPerPoint };
enum class PointTimeEncoding {
  kUint32RelativeNanoseconds,
  kFloat64AbsoluteNanoseconds,
};
enum class ScanReference { kHeaderStamp, kMinimumPointTime };

struct PointTimeConfig {
  std::string field{"time"};
  PointTimeEncoding encoding{PointTimeEncoding::kUint32RelativeNanoseconds};
  ScanReference scan_reference{ScanReference::kHeaderStamp};
  std::int64_t maximum_scan_duration_ns{200'000'000};
  std::int64_t maximum_header_offset_ns{200'000'000};
  bool reject_scan_timestamp_regression{true};
};

class RosLidarAdapter {
 public:
  RosLidarAdapter(
      std::string expected_frame, LidarTimingMode timing_mode,
      ClockDomain clock_domain = ClockDomain::kRosTime,
      PointTimeConfig point_time = {});
  [[nodiscard]] LidarScan convert(const sensor_msgs::msg::PointCloud2& message) const;

 private:
  std::string expected_frame_;
  LidarTimingMode timing_mode_;
  ClockDomain clock_domain_;
  PointTimeConfig point_time_;
  mutable std::int64_t previous_scan_start_ns_{-1};
};

}  // namespace uav::nav::lio
