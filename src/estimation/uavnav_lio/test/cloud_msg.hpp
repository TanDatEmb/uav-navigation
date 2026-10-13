#pragma once

// Test-only message builders for the lio node tests: a Gazebo-layout PointCloud2 (float32 x,y,z at 0/4/8,
// float32 intensity at 16, uint16 ring at 24, point_step 32, frame livox_frame; the layout the Task 3 probe
// recorded) and a sensor_msgs/Imu in the IMU frame.

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "uavnav/lio_node/limits.hpp"

namespace uavnav::lio_node::testmsg {

inline constexpr std::uint32_t kPointStep = 32;

inline builtin_interfaces::msg::Time Stamp(std::int64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<std::int32_t>(ns / 1'000'000'000);
  t.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000);
  return t;
}

inline sensor_msgs::msg::PointField Field(const std::string& name, std::uint32_t offset, std::uint8_t type) {
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  return f;
}

/// One unorganised cloud (height 1) of `points` in the Gazebo layout, stamped `stamp_ns`.
inline sensor_msgs::msg::PointCloud2 Cloud(const std::vector<Eigen::Vector3f>& points, std::int64_t stamp_ns) {
  using sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 c;
  c.header.stamp = Stamp(stamp_ns);
  c.header.frame_id = std::string(limits::kLidarFrameId);
  c.height = 1;
  c.width = static_cast<std::uint32_t>(points.size());
  c.fields = {Field("x", 0, PointField::FLOAT32), Field("y", 4, PointField::FLOAT32),
              Field("z", 8, PointField::FLOAT32), Field("intensity", 16, PointField::FLOAT32),
              Field("ring", 24, PointField::UINT16)};
  c.is_bigendian = false;
  c.point_step = kPointStep;
  c.row_step = kPointStep * c.width;
  c.is_dense = false;
  c.data.assign(static_cast<std::size_t>(c.row_step), 0);
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::uint8_t* p = c.data.data() + i * kPointStep;
    std::memcpy(p, points[i].data(), 3 * sizeof(float));
    const float intensity = 100.0F;
    std::memcpy(p + 16, &intensity, sizeof(float));
  }
  return c;
}

inline sensor_msgs::msg::Imu Imu(std::int64_t stamp_ns, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel) {
  sensor_msgs::msg::Imu m;
  m.header.stamp = Stamp(stamp_ns);
  m.header.frame_id = std::string(limits::kImuFrameId);
  m.angular_velocity.x = gyro.x();
  m.angular_velocity.y = gyro.y();
  m.angular_velocity.z = gyro.z();
  m.linear_acceleration.x = accel.x();
  m.linear_acceleration.y = accel.y();
  m.linear_acceleration.z = accel.z();
  m.orientation.w = 1.0;
  return m;
}

}  // namespace uavnav::lio_node::testmsg
