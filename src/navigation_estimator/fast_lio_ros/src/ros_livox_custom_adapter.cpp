#include "fast_lio_ros/ros_livox_custom_adapter.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

RosLivoxCustomAdapter::RosLivoxCustomAdapter(
    std::string expected_frame, ClockDomain clock_domain,
    LivoxTimestampPolicy timestamp_policy)
    : expected_frame_(std::move(expected_frame)),
      clock_domain_(clock_domain),
      timestamp_policy_(timestamp_policy) {}

LidarScan RosLivoxCustomAdapter::convert(
    const livox_ros_driver2::msg::CustomMsg& message) const {
  if (message.header.frame_id != expected_frame_) {
    throw std::invalid_argument(
        "Livox CustomMsg frame does not match configured lidar frame");
  }
  if (message.point_num != message.points.size()) {
    throw std::invalid_argument(
        "Livox CustomMsg point_num does not match points size");
  }
  if (message.points.empty()) {
    throw std::invalid_argument("Livox CustomMsg contains no points");
  }
  if (message.timebase >
      static_cast<std::uint64_t>(
          std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument(
        "Livox CustomMsg timebase exceeds core int64 nanosecond range");
  }

  const Timestamp header_time =
      RosTimeConverter::fromRos(message.header.stamp, clock_domain_);
  const auto timebase_ns = static_cast<std::int64_t>(message.timebase);
  if (timestamp_policy_ ==
          LivoxTimestampPolicy::kRequireHeaderMatchesTimebase &&
      header_time.nanoseconds() != timebase_ns) {
    throw std::invalid_argument(
        "Livox CustomMsg header.stamp does not equal timebase");
  }

  LidarScan scan;
  scan.start_time = Timestamp(timebase_ns, clock_domain_);
  scan.end_time = scan.start_time;
  scan.has_per_point_time = true;
  scan.points.reserve(message.points.size());
  std::uint32_t maximum_offset_time_ns = 0U;
  for (const auto& point : message.points) {
    LidarPoint converted;
    converted.relative_time_ns = point.offset_time;
    converted.position_lidar_m = {point.x, point.y, point.z};
    converted.reflectivity = point.reflectivity;
    converted.tag = point.tag;
    converted.line = point.line;
    if (!converted.allFinite()) {
      throw std::invalid_argument(
          "Livox CustomMsg contains a non-finite point");
    }
    maximum_offset_time_ns =
        std::max(maximum_offset_time_ns, point.offset_time);
    scan.points.push_back(converted);
  }
  const auto end_time =
      checkedAdd(scan.start_time, Duration(maximum_offset_time_ns));
  if (!end_time.ok()) {
    throw std::invalid_argument(end_time.status().message());
  }
  scan.end_time = end_time.value();
  const Status scan_status = scan.validate();
  if (!scan_status.ok()) {
    throw std::invalid_argument(scan_status.message());
  }
  return scan;
}

}  // namespace uav::nav::lio
