#include "fast_lio_ros/ros_lidar_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {
namespace {

const sensor_msgs::msg::PointField& requireField(const sensor_msgs::msg::PointCloud2& message,
                                                 std::string_view name, std::uint8_t datatype) {
  const auto field = std::find_if(message.fields.begin(), message.fields.end(),
                                  [name](const auto& candidate) { return candidate.name == name; });
  if (field == message.fields.end() || field->datatype != datatype || field->count != 1U) {
    throw std::invalid_argument("PointCloud2 has a missing or invalid field");
  }
  return *field;
}

template <typename T>
T readScalar(const std::uint8_t* point, std::uint32_t offset, std::uint32_t point_step) {
  if (offset + sizeof(T) > point_step) {
    throw std::invalid_argument("PointCloud2 field exceeds point_step");
  }
  T value{};
  std::memcpy(&value, point + offset, sizeof(T));
  return value;
}

}  // namespace

RosLidarAdapter::RosLidarAdapter(std::string expected_frame,
                                 LidarTimingMode timing_mode,
                                 ClockDomain clock_domain,
                                 PointTimeConfig point_time)
    : expected_frame_(std::move(expected_frame)),
      timing_mode_(timing_mode),
      clock_domain_(clock_domain),
      point_time_(std::move(point_time)) {
  if (point_time_.field.empty() ||
      point_time_.maximum_scan_duration_ns <= 0 ||
      point_time_.maximum_header_offset_ns < 0 ||
      point_time_.maximum_boundary_overlap_ns < 0 ||
      point_time_.minimum_points_after_overlap_trim == 0U) {
    throw std::invalid_argument("invalid PointCloud2 point-time configuration");
  }
}

LidarScan RosLidarAdapter::convert(const sensor_msgs::msg::PointCloud2& message) const {
  if (message.header.frame_id != expected_frame_) {
    throw std::invalid_argument("point cloud frame does not match configured lidar frame");
  }
  if (message.is_bigendian) {
    throw std::invalid_argument("big-endian PointCloud2 is unsupported");
  }
  const auto& x = requireField(message, "x", sensor_msgs::msg::PointField::FLOAT32);
  const auto& y = requireField(message, "y", sensor_msgs::msg::PointField::FLOAT32);
  const auto& z = requireField(message, "z", sensor_msgs::msg::PointField::FLOAT32);
  const sensor_msgs::msg::PointField* time = nullptr;
  if (timing_mode_ == LidarTimingMode::kPerPoint) {
    const auto datatype =
        point_time_.encoding == PointTimeEncoding::kUint32RelativeNanoseconds
            ? sensor_msgs::msg::PointField::UINT32
            : sensor_msgs::msg::PointField::FLOAT64;
    time = &requireField(message, point_time_.field, datatype);
  }
  if (message.point_step == 0U ||
      static_cast<std::uint64_t>(message.point_step) * message.width >
          std::numeric_limits<std::uint32_t>::max() ||
      message.row_step <
          static_cast<std::uint64_t>(message.point_step) * message.width ||
      static_cast<std::uint64_t>(message.row_step) * message.height >
          message.data.size()) {
    throw std::invalid_argument("PointCloud2 storage layout is inconsistent");
  }
  const auto point_count =
      static_cast<std::size_t>(message.width) * message.height;
  if (point_count == 0U) {
    throw std::invalid_argument("PointCloud2 must contain at least one point");
  }

  const auto header_time =
      RosTimeConverter::fromRos(message.header.stamp, clock_domain_);
  std::vector<std::int64_t> absolute_times;
  if (time != nullptr &&
      point_time_.encoding == PointTimeEncoding::kFloat64AbsoluteNanoseconds) {
    absolute_times.reserve(point_count);
  }
  LidarScan scan{header_time, header_time, {}, time != nullptr};
  scan.points.reserve(point_count);
  for (std::uint32_t row = 0; row < message.height; ++row) {
    const auto* row_data = message.data.data() + row * message.row_step;
    for (std::uint32_t column = 0; column < message.width; ++column) {
      const auto* point = row_data + column * message.point_step;
      LidarPoint converted;
      converted.position_lidar_m = {readScalar<float>(point, x.offset, message.point_step),
                                    readScalar<float>(point, y.offset, message.point_step),
                                    readScalar<float>(point, z.offset, message.point_step)};
      std::int64_t absolute_time = 0;
      if (time != nullptr &&
          point_time_.encoding == PointTimeEncoding::kUint32RelativeNanoseconds) {
        converted.relative_time_ns =
            readScalar<std::uint32_t>(point, time->offset, message.point_step);
      } else if (time != nullptr) {
        const double raw =
            readScalar<double>(point, time->offset, message.point_step);
        if (!std::isfinite(raw) ||
            static_cast<long double>(raw) <
                static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
            static_cast<long double>(raw) >
                static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
          throw std::invalid_argument("point timestamp is non-finite or outside int64");
        }
        // std::round specifies half-away-from-zero; conversion happens before
        // timestamp subtraction so absolute doubles never enter fast_lio_core.
        absolute_time = static_cast<std::int64_t>(std::round(raw));
      }
      if (!converted.position_lidar_m.allFinite()) {
        continue;
      }
      if (time != nullptr &&
          point_time_.encoding == PointTimeEncoding::kFloat64AbsoluteNanoseconds) {
        absolute_times.push_back(absolute_time);
      }
      scan.points.push_back(converted);
    }
  }
  if (scan.points.empty()) {
    throw std::invalid_argument("PointCloud2 contains no finite XYZ point");
  }
  normalization_statistics_.input_point_count += scan.points.size();

  if (!absolute_times.empty() && previous_emitted_end_ns_ >= 0) {
    const auto minimum_time =
        *std::min_element(absolute_times.begin(), absolute_times.end());
    const std::int64_t overlap_ns = previous_emitted_end_ns_ - minimum_time;
    if (overlap_ns > 0 &&
        overlap_ns <= point_time_.maximum_boundary_overlap_ns) {
      std::vector<LidarPoint> emitted_points;
      std::vector<std::int64_t> emitted_times;
      emitted_points.reserve(scan.points.size());
      emitted_times.reserve(absolute_times.size());
      for (std::size_t index = 0; index < scan.points.size(); ++index) {
        if (absolute_times[index] <= previous_emitted_end_ns_) {
          ++normalization_statistics_.dropped_overlapping_point_count;
          continue;
        }
        emitted_points.push_back(scan.points[index]);
        emitted_times.push_back(absolute_times[index]);
      }
      if (emitted_points.size() <
          point_time_.minimum_points_after_overlap_trim) {
        throw std::invalid_argument(
            "PointCloud2 overlap trim left too few points");
      }
      scan.points = std::move(emitted_points);
      absolute_times = std::move(emitted_times);
    }
  }
  normalization_statistics_.emitted_point_count += scan.points.size();

  std::int64_t scan_start_ns = header_time.nanoseconds();
  if (!absolute_times.empty()) {
    scan_start_ns = point_time_.scan_reference == ScanReference::kMinimumPointTime
                        ? *std::min_element(absolute_times.begin(), absolute_times.end())
                        : header_time.nanoseconds();
    if (std::abs(static_cast<long double>(header_time.nanoseconds()) -
                 static_cast<long double>(scan_start_ns)) >
        point_time_.maximum_header_offset_ns) {
      throw std::invalid_argument("PointCloud2 header/point timestamp offset exceeds limit");
    }
    for (std::size_t index = 0; index < scan.points.size(); ++index) {
      const auto difference =
          checkedDifference(Timestamp{absolute_times[index], clock_domain_},
                            Timestamp{scan_start_ns, clock_domain_});
      if (!difference.ok()) {
        throw std::invalid_argument("point relative timestamp overflows int64");
      }
      const auto relative = difference.value().nanoseconds();
      if (relative < 0 ||
          relative > point_time_.maximum_scan_duration_ns ||
          relative > std::numeric_limits<std::uint32_t>::max()) {
        throw std::invalid_argument("point relative timestamp is invalid");
      }
      scan.points[index].relative_time_ns =
          static_cast<std::uint32_t>(relative);
    }
  }
  std::uint32_t maximum_relative_time = 0U;
  for (const auto& point : scan.points) {
    maximum_relative_time =
        std::max(maximum_relative_time, point.relative_time_ns);
  }
  if (maximum_relative_time >
      static_cast<std::uint64_t>(point_time_.maximum_scan_duration_ns)) {
    throw std::invalid_argument("PointCloud2 scan duration exceeds configured limit");
  }
  if (point_time_.reject_scan_timestamp_regression &&
      previous_scan_start_ns_ >= 0 && scan_start_ns < previous_scan_start_ns_) {
    throw std::invalid_argument("PointCloud2 scan timestamp regressed");
  }
  scan.start_time = Timestamp{scan_start_ns, clock_domain_};
  previous_scan_start_ns_ = scan_start_ns;
  const auto end_time = checkedAdd(scan.start_time, Duration{maximum_relative_time});
  if (!end_time.ok()) {
    throw std::invalid_argument(end_time.status().message());
  }
  scan.end_time = end_time.value();
  previous_emitted_end_ns_ = scan.end_time.nanoseconds();
  return scan;
}

PointTimeNormalizationStatistics
RosLidarAdapter::normalizationStatistics() const noexcept {
  return normalization_statistics_;
}

}  // namespace uav::nav::lio
