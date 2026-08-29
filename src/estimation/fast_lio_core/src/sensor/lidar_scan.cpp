#include "fast_lio_core/sensor/lidar_scan.hpp"

#include <algorithm>
#include <cstdint>

namespace uav::nav::lio {

Status LidarScan::validate() const {
  if (start_time.nanoseconds() <= 0 || end_time.nanoseconds() <= 0) {
    return Status(StatusCode::kInvalidArgument, "LiDAR scan timestamps must be positive");
  }
  const auto scan_duration = checkedDifference(end_time, start_time);
  if (!scan_duration.ok()) {
    return scan_duration.status();
  }
  if (scan_duration.value().nanoseconds() <= 0) {
    return Status(StatusCode::kTimestampRegression, "LiDAR scan end precedes scan start");
  }
  if (points.empty()) {
    return Status(StatusCode::kInvalidArgument, "LiDAR scan contains no points");
  }
  const auto duration_ns = static_cast<std::uint64_t>(scan_duration.value().nanoseconds());
  for (const auto& point : points) {
    if (!point.allFinite()) {
      return Status(StatusCode::kInvalidArgument, "LiDAR scan contains a non-finite point");
    }
    if (!has_per_point_time && point.relative_time_ns != 0U) {
      return Status(StatusCode::kInvalidArgument,
                    "Point relative time must be zero for a simultaneous scan");
    }
    if (has_per_point_time && static_cast<std::uint64_t>(point.relative_time_ns) > duration_ns) {
      return Status(StatusCode::kOutOfRange, "Point relative time lies outside scan interval");
    }
  }
  return Status::Ok();
}

Result<Timestamp> LidarScan::pointTime(const LidarPoint& point) const {
  if (!has_per_point_time && point.relative_time_ns != 0U) {
    return Status(StatusCode::kInvalidArgument,
                  "A simultaneous scan cannot contain non-zero point time");
  }
  const auto time = checkedAdd(start_time, Duration(point.relative_time_ns));
  if (!time.ok()) {
    return time.status();
  }
  const auto compare = checkedCompare(time.value(), end_time);
  if (!compare.ok()) {
    return compare.status();
  }
  if (compare.value() > 0) {
    return Status(StatusCode::kOutOfRange, "Point relative time lies after scan end");
  }
  return time.value();
}

}  // namespace uav::nav::lio
