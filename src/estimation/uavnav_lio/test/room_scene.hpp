#pragma once

// Test-only synthetic data for the lio ingest (and node) tests: a 10 m box room seen by a LiDAR that casts a fixed,
// deterministic set of rays (a Fibonacci sphere), and a stationary 200 Hz IMU stream. No randomness, no clock.
// Adapted from uavnav_lio_core/test/synthetic_scene.hpp (a package does not include another package's test
// headers).

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>

#include "fast_lio_core/sensor/lidar_point.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/types.hpp"

namespace uavnav::lio_node::room {

/// Room bounds in the room frame (z up, floor at z = 0): 10 x 10 x 4 m.
inline constexpr double kHalfX = 5.0;
inline constexpr double kHalfY = 5.0;
inline constexpr double kHeight = 4.0;
inline constexpr std::size_t kRays = 3000;
inline constexpr double kGravity = 9.80665;

inline constexpr time::Duration kImuPeriod = time::milliseconds(5);     // 200 Hz
inline constexpr time::Duration kScanPeriod = time::milliseconds(100);  // 10 Hz

/// The IMU pose in the room frame (^room T_imu): stationary, level, 1.5 m above the floor.
inline Eigen::Isometry3d room_T_imu() {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(0.3, -0.2, 1.5);
  return t;
}

/// Distance along unit ray d from o (inside the room) to the first wall, floor or ceiling.
inline double cast(const Eigen::Vector3d& o, const Eigen::Vector3d& d) {
  double best = std::numeric_limits<double>::infinity();
  const double lo[3] = {-kHalfX, -kHalfY, 0.0};
  const double hi[3] = {kHalfX, kHalfY, kHeight};
  for (int a = 0; a < 3; ++a) {
    if (std::abs(d[a]) < 1e-12) continue;
    const double s = ((d[a] > 0.0 ? hi[a] : lo[a]) - o[a]) / d[a];
    if (s > 0.0 && s < best) best = s;
  }
  return best;
}

/// Ray i of kRays, a Fibonacci sphere (deterministic, near-uniform directions), in the LiDAR frame.
inline Eigen::Vector3d ray(std::size_t i) {
  const double golden = std::numbers::pi * (3.0 - std::sqrt(5.0));
  const double z = 1.0 - 2.0 * (static_cast<double>(i) + 0.5) / static_cast<double>(kRays);
  const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
  const double phi = golden * static_cast<double>(i);
  return Eigen::Vector3d(r * std::cos(phi), r * std::sin(phi), z);
}

/// One Gazebo-style scan (O20: one instant, start = end = t, no per-point time) of the room from the stationary
/// IMU pose, in the LiDAR frame given ^imu T_lidar.
inline lio::ScanInput scan(time::SensorTime t, const Eigen::Isometry3d& imu_T_lidar) {
  const Eigen::Isometry3d room_T_lidar = room_T_imu() * imu_T_lidar;
  lio::ScanInput s{t, t, {}, false};
  s.points.reserve(kRays);
  for (std::size_t i = 0; i < kRays; ++i) {
    const Eigen::Vector3d d_lidar = ray(i);
    const double range = cast(room_T_lidar.translation(), room_T_lidar.linear() * d_lidar);
    if (!std::isfinite(range)) continue;
    uav::nav::lio::LidarPoint p;
    p.position_lidar_m = (d_lidar * range).cast<float>();
    s.points.push_back(p);
  }
  return s;
}

/// A sample of the stationary IMU stream: no rotation, specific force = -gravity (z up).
inline lio::ImuInput imu(time::SensorTime t) {
  return lio::ImuInput{t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, kGravity)};
}

/// Sample k of the 200 Hz stream that starts at t0.
inline time::SensorTime imu_time(time::SensorTime t0, std::int64_t k) { return t0 + time::Duration{k * kImuPeriod.ns}; }

}  // namespace uavnav::lio_node::room
