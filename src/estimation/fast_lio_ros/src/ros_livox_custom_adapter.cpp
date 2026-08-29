#include "fast_lio_ros/ros_livox_custom_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include <navigation_common/time.hpp>

#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

namespace {

bool validClockDomain(const ClockDomain domain) {
  switch (domain) {
    case ClockDomain::kRosTime:
    case ClockDomain::kSimulationTime:
    case ClockDomain::kSensorTime:
    case ClockDomain::kSystemTime:
    case ClockDomain::kSteadyTime:
      return true;
  }
  return false;
}

bool validTimestampPolicy(const LivoxTimestampPolicy policy) {
  return policy == LivoxTimestampPolicy::kRequireHeaderMatchesTimebase ||
         policy == LivoxTimestampPolicy::kTimebaseAuthoritative;
}

}  // namespace

RosLivoxCustomAdapter::RosLivoxCustomAdapter(
    std::string expected_frame, ClockDomain clock_domain,
    LivoxTimestampPolicy timestamp_policy, PointTimeConfig point_time)
    : expected_frame_(std::move(expected_frame)),
      clock_domain_(clock_domain),
      timestamp_policy_(timestamp_policy),
      point_time_(std::move(point_time)) {
  if (expected_frame_.empty() || !validClockDomain(clock_domain_) ||
      !validTimestampPolicy(timestamp_policy_) ||
      point_time_.maximum_scan_duration_ns <= 0 ||
      point_time_.maximum_header_offset_ns < 0) {
    throw std::invalid_argument("invalid Livox CustomMsg adapter configuration");
  }
}

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
  if (message.points.size() > kMaximumLidarPointCount) {
    throw std::invalid_argument(
        "Livox CustomMsg point count is outside the supported bound");
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
  const auto header_offset_ns = navigation_common::checkedDifference(
      header_time.nanoseconds(), timebase_ns);
  if (!header_offset_ns ||
      std::abs(static_cast<long double>(*header_offset_ns)) >
          static_cast<long double>(point_time_.maximum_header_offset_ns)) {
    throw std::invalid_argument(
        "Livox CustomMsg header.stamp exceeds configured timebase offset");
  }
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
  if (static_cast<std::int64_t>(maximum_offset_time_ns) >
      point_time_.maximum_scan_duration_ns) {
    throw std::invalid_argument("Livox CustomMsg scan duration exceeds configured limit");
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
