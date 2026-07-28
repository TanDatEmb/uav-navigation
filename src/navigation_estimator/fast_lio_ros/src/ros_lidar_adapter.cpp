#include "fast_lio_ros/ros_lidar_adapter.hpp"

#include <algorithm>
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

RosLidarAdapter::RosLidarAdapter(std::string expected_frame, LidarTimingMode timing_mode)
    : expected_frame_(std::move(expected_frame)), timing_mode_(timing_mode) {}

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
  for (const auto& field : message.fields) {
    if (field.name == "time" || field.name == "t") {
      if (field.datatype != sensor_msgs::msg::PointField::UINT32 || field.count != 1U) {
        throw std::invalid_argument("point time must be uint32 nanoseconds");
      }
      time = &field;
      break;
    }
  }
  if (timing_mode_ == LidarTimingMode::kPerPoint && time == nullptr) {
    throw std::invalid_argument("per_point timing requires uint32 time field");
  }
  if (message.row_step < message.point_step * static_cast<std::uint64_t>(message.width) ||
      message.data.size() < message.row_step * static_cast<std::uint64_t>(message.height)) {
    throw std::invalid_argument("PointCloud2 storage layout is inconsistent");
  }

  const auto start_time = RosTimeConverter::fromRos(message.header.stamp);
  LidarScan scan{start_time, start_time, {}, time != nullptr};
  scan.points.reserve(static_cast<std::size_t>(message.width) * message.height);
  std::uint32_t maximum_relative_time = 0U;
  for (std::uint32_t row = 0; row < message.height; ++row) {
    const auto* row_data = message.data.data() + row * message.row_step;
    for (std::uint32_t column = 0; column < message.width; ++column) {
      const auto* point = row_data + column * message.point_step;
      LidarPoint converted;
      converted.position_lidar_m = {readScalar<float>(point, x.offset, message.point_step),
                                    readScalar<float>(point, y.offset, message.point_step),
                                    readScalar<float>(point, z.offset, message.point_step)};
      if (!converted.position_lidar_m.allFinite()) {
        continue;
      }
      converted.relative_time_ns =
          time == nullptr ? 0U : readScalar<std::uint32_t>(point, time->offset, message.point_step);
      maximum_relative_time = std::max(maximum_relative_time, converted.relative_time_ns);
      scan.points.push_back(converted);
    }
  }
  const auto end_time = checkedAdd(scan.start_time, Duration{maximum_relative_time});
  if (!end_time.ok()) {
    throw std::invalid_argument(end_time.status().message());
  }
  scan.end_time = end_time.value();
  return scan;
}

}  // namespace uav::nav::lio
