#include "fast_lio_ros/ros_livox_custom_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

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
      point_time_.maximum_header_offset_ns < 0 ||
      point_time_.maximum_boundary_overlap_ns < 0 ||
      point_time_.minimum_points_after_overlap_trim == 0U) {
    throw std::invalid_argument("invalid Livox CustomMsg adapter configuration");
  }
}

LidarScan RosLivoxCustomAdapter::convert(
    const livox_ros_driver2::msg::CustomMsg& message) const {
  std::lock_guard lock(state_mutex_);
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
  normalization_statistics_.input_point_count += message.points.size();
  if (message.timebase == 0U || message.timebase >
      static_cast<std::uint64_t>(
          std::numeric_limits<std::int64_t>::max())) {
    throw std::invalid_argument(
        "Livox CustomMsg timebase exceeds core int64 nanosecond range");
  }

  const Timestamp header_time =
      RosTimeConverter::fromRos(message.header.stamp, clock_domain_);
  const auto timebase_ns = static_cast<std::int64_t>(message.timebase);
  if (point_time_.reject_scan_timestamp_regression &&
      previous_scan_start_ns_ > 0 && timebase_ns <= previous_scan_start_ns_) {
    throw std::invalid_argument(
        "Livox CustomMsg timestamp is not strictly increasing");
  }
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
  std::vector<std::int64_t> absolute_times;
  absolute_times.reserve(message.points.size());
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
    const auto absolute_time = checkedAdd(
        scan.start_time, Duration(static_cast<std::uint64_t>(point.offset_time)));
    if (!absolute_time.ok()) {
      throw std::invalid_argument(absolute_time.status().message());
    }
    absolute_times.push_back(absolute_time.value().nanoseconds());
    scan.points.push_back(converted);
  }

  if (previous_emitted_end_ns_ >= 0 && !absolute_times.empty()) {
    const auto minimum_time = *std::min_element(absolute_times.begin(), absolute_times.end());
    const long double overlap_ns =
        static_cast<long double>(previous_emitted_end_ns_) -
        static_cast<long double>(minimum_time);
    if (overlap_ns > static_cast<long double>(point_time_.maximum_boundary_overlap_ns)) {
      throw std::invalid_argument(
          "Livox CustomMsg boundary overlap exceeds configured limit");
    }
    if (overlap_ns > 0.0L) {
      std::vector<std::size_t> emitted_indices;
      std::vector<std::int64_t> emitted_times;
      emitted_indices.reserve(message.points.size());
      emitted_times.reserve(absolute_times.size());
      for (std::size_t index = 0; index < absolute_times.size(); ++index) {
        if (absolute_times[index] <= previous_emitted_end_ns_) {
          ++normalization_statistics_.dropped_overlapping_point_count;
          continue;
        }
        emitted_indices.push_back(index);
        emitted_times.push_back(absolute_times[index]);
      }
      if (emitted_indices.size() < point_time_.minimum_points_after_overlap_trim) {
        throw std::invalid_argument(
            "Livox CustomMsg overlap trim left too few points");
      }
      std::vector<LidarPoint> emitted_points;
      emitted_points.reserve(emitted_indices.size());
      for (const auto index : emitted_indices) {
        emitted_points.push_back(scan.points[index]);
      }
      scan.points = std::move(emitted_points);
      absolute_times = std::move(emitted_times);
    }
  }

  if (scan.points.empty() || absolute_times.empty()) {
    throw std::invalid_argument("Livox CustomMsg overlap trim left no points");
  }
  const auto minimum_time = *std::min_element(absolute_times.begin(), absolute_times.end());
  const std::int64_t scan_start_ns =
      point_time_.scan_reference == ScanReference::kMinimumPointTime
          ? minimum_time
          : timebase_ns;
  scan.start_time = Timestamp(scan_start_ns, clock_domain_);
  const auto scan_header_offset = checkedDifference(header_time, scan.start_time);
  if (!scan_header_offset.ok() ||
      std::abs(static_cast<long double>(scan_header_offset.value().nanoseconds())) >
          static_cast<long double>(point_time_.maximum_header_offset_ns)) {
    throw std::invalid_argument(
        "Livox CustomMsg header.stamp exceeds configured scan offset");
  }
  std::uint32_t maximum_relative_time_ns = 0U;
  for (std::size_t index = 0; index < scan.points.size(); ++index) {
    const auto relative_time = checkedDifference(
        Timestamp(absolute_times[index], clock_domain_), scan.start_time);
    if (!relative_time.ok() || relative_time.value().nanoseconds() < 0 ||
        relative_time.value().nanoseconds() > point_time_.maximum_scan_duration_ns ||
        static_cast<std::uint64_t>(relative_time.value().nanoseconds()) >
            std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("Livox CustomMsg point timestamp is invalid");
    }
    scan.points[index].relative_time_ns =
        static_cast<std::uint32_t>(relative_time.value().nanoseconds());
    maximum_relative_time_ns =
        std::max(maximum_relative_time_ns, scan.points[index].relative_time_ns);
  }
  const auto end_time =
      checkedAdd(scan.start_time, Duration(maximum_relative_time_ns));
  if (!end_time.ok()) {
    throw std::invalid_argument(end_time.status().message());
  }
  scan.end_time = end_time.value();
  const Status scan_status = scan.validate();
  if (!scan_status.ok()) {
    throw std::invalid_argument(scan_status.message());
  }
  previous_scan_start_ns_ = scan.start_time.nanoseconds();
  previous_emitted_end_ns_ = scan.end_time.nanoseconds();
  normalization_statistics_.emitted_point_count += scan.points.size();
  return scan;
}

PointTimeNormalizationStatistics
RosLivoxCustomAdapter::normalizationStatistics() const noexcept {
  std::lock_guard lock(state_mutex_);
  return normalization_statistics_;
}

}  // namespace uav::nav::lio
