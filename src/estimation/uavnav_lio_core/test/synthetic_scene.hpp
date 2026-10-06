#pragma once

// Test-only synthetic scene for LioEstimator: an axis-aligned box room, 20 x 20 x 6 m, seen by a LiDAR that
// casts a fixed, deterministic set of rays (a Fibonacci sphere) from its pose. No randomness, no clock.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

#include "fast_lio_core/sensor/lidar_point.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/estimator.hpp"

namespace uavnav::lio::scene {

/// Room bounds in the room frame (z up, floor at z = 0).
inline constexpr double kHalfX = 10.0;
inline constexpr double kHalfY = 10.0;
inline constexpr double kHeight = 6.0;
inline constexpr std::size_t kRays = 3000;
inline constexpr double kGravity = 9.80665;

/// ^imu T_lidar: the beta extrinsic of config/lio/sim.yaml (Mid-360), identity rotation.
inline Eigen::Isometry3d imu_T_lidar() {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(-0.011, -0.02329, 0.04412);
  return t;
}

/// ^base T_imu used by the tests: IMU 10 cm forward, 5 cm up of base_link, no rotation.
inline Eigen::Isometry3d base_T_imu() {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(0.10, 0.0, 0.05);
  return t;
}

/// IMU pose in the room frame (^room T_imu).
struct Pose {
  Eigen::Vector3d p_m{0.0, 0.0, 1.5};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
};

/// IMU trajectory: level, constant velocity from p0 at t0 (v = 0: stationary).
struct Trajectory {
  Eigen::Vector3d p0_m{0.0, 0.0, 1.5};
  Eigen::Vector3d v_mps{Eigen::Vector3d::Zero()};
  time::SensorTime t0{};

  Pose at(time::SensorTime t) const {
    Pose pose;
    pose.p_m = p0_m + v_mps * time::to_seconds(t - t0);
    return pose;
  }
};

/// Distance along unit ray d from o (inside the room) to the first wall, floor or ceiling, and which face.
/// kWallX: the walls x = +-kHalfX (normal along x); kWallY: y = +-kHalfY.
enum class Face : std::uint8_t { kWallX, kWallY, kFloor, kCeiling };

inline double cast(const Eigen::Vector3d& o, const Eigen::Vector3d& d, Face& face) {
  double best = std::numeric_limits<double>::infinity();
  const double lo[3] = {-kHalfX, -kHalfY, 0.0};
  const double hi[3] = {kHalfX, kHalfY, kHeight};
  for (int a = 0; a < 3; ++a) {
    if (std::abs(d[a]) < 1e-12) continue;
    const double bound = d[a] > 0.0 ? hi[a] : lo[a];
    const double s = (bound - o[a]) / d[a];
    if (s > 0.0 && s < best) {
      best = s;
      face = a == 0 ? Face::kWallX : (a == 1 ? Face::kWallY : (d[a] > 0.0 ? Face::kCeiling : Face::kFloor));
    }
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

/// Points of every ray that hits one of `faces` (bit mask of Face values), in the LiDAR frame.
inline ScanInput scan_of(const Pose& imu_pose, time::SensorTime t_start, time::SensorTime t_end, unsigned faces) {
  const Eigen::Isometry3d room_T_imu = Eigen::Translation3d(imu_pose.p_m) * imu_pose.q;
  const Eigen::Isometry3d room_T_lidar = room_T_imu * imu_T_lidar();
  ScanInput scan{t_start, t_end, {}, false};
  scan.points.reserve(kRays);
  for (std::size_t i = 0; i < kRays; ++i) {
    const Eigen::Vector3d d_lidar = ray(i);
    const Eigen::Vector3d d_room = room_T_lidar.linear() * d_lidar;
    Face face = Face::kWallX;
    const double range = cast(room_T_lidar.translation(), d_room, face);
    if (!std::isfinite(range)) continue;
    if ((faces & (1U << static_cast<unsigned>(face))) == 0U) continue;
    uav::nav::lio::LidarPoint p;
    p.position_lidar_m = (d_lidar * range).cast<float>();
    scan.points.push_back(p);
  }
  return scan;
}

inline constexpr unsigned bit(Face f) { return 1U << static_cast<unsigned>(f); }
inline constexpr unsigned kAllFaces = 0b1111U;
inline constexpr unsigned kFloorOnly = bit(Face::kFloor);
/// Floor, ceiling and the two x walls: nothing constrains translation along y, while every rotation axis is
/// constrained (floor/ceiling: roll and pitch; x walls: pitch and yaw). Used to tell the blocks apart.
inline constexpr unsigned kCorridor = bit(Face::kFloor) | bit(Face::kCeiling) | bit(Face::kWallX);

/// About 3000 points on the walls, floor and ceiling of the room (no per-point time).
inline ScanInput make_scan(const Pose& imu_pose, time::SensorTime t_start, time::SensorTime t_end) {
  return scan_of(imu_pose, t_start, t_end, kAllFaces);
}

/// Floor points only: every point on one plane (the degenerate case).
inline ScanInput make_plane_only_scan(const Pose& imu_pose, time::SensorTime t_start, time::SensorTime t_end) {
  return scan_of(imu_pose, t_start, t_end, kFloorOnly);
}

/// Corridor along y (see kCorridor): translation-degenerate, rotation well constrained.
inline ScanInput make_corridor_scan(const Pose& imu_pose, time::SensorTime t_start, time::SensorTime t_end) {
  return scan_of(imu_pose, t_start, t_end, kCorridor);
}

/// IMU sample at t for a level, constant-velocity trajectory: no rotation, specific force = -gravity.
inline ImuInput make_imu(const Trajectory& /*trajectory*/, time::SensorTime t) {
  return ImuInput{t, Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, kGravity)};
}

}  // namespace uavnav::lio::scene
