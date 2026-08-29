#pragma once

#include <cstdint>
#include <mutex>
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
  std::int64_t maximum_boundary_overlap_ns{0};
  std::size_t minimum_points_after_overlap_trim{1};
};

struct PointTimeNormalizationStatistics {
  std::size_t input_point_count{0};
  std::size_t emitted_point_count{0};
  std::size_t dropped_overlapping_point_count{0};
};

class RosLidarAdapter {
 public:
  RosLidarAdapter(
      std::string expected_frame, LidarTimingMode timing_mode,
      ClockDomain clock_domain = ClockDomain::kRosTime,
      PointTimeConfig point_time = {});
  [[nodiscard]] LidarScan convert(const sensor_msgs::msg::PointCloud2& message) const;
  [[nodiscard]] PointTimeNormalizationStatistics normalizationStatistics() const noexcept;

 private:
  std::string expected_frame_;
  LidarTimingMode timing_mode_;
  ClockDomain clock_domain_;
  PointTimeConfig point_time_;
  mutable std::int64_t previous_scan_start_ns_{-1};
  mutable std::int64_t previous_emitted_end_ns_{-1};
  mutable std::mutex state_mutex_;
  mutable PointTimeNormalizationStatistics normalization_statistics_{};
};

}  // namespace uav::nav::lio
