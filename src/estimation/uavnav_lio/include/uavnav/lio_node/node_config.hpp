#pragma once

#include <array>
#include <numbers>

#include <Eigen/Geometry>

#include "uavnav/core/config.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/lio/config.hpp"

// Tier-(b) configuration of the lio node (SYSTEM_DESIGN §6.2, D22, D33/O20): the base_link <- LiDAR mount, loaded
// from its own file (config/lio/node_sim.yaml) against kLioNodeSpecs. The LIO core's keys (kLioSpecs, including
// the IMU <- LiDAR extrinsic) stay in config/lio/sim.yaml; nothing is read twice.
namespace uavnav::lio_node {

/// The six mount keys: the LiDAR pose in base_link (base_T_lidar). Angles are fixed-axis roll-pitch-yaw as in
/// SDF/URDF poses: R = Rz(yaw) * Ry(pitch) * Rx(roll). Bounds:
///   translation +-1.0 m: a LiDAR on the airframe (x500: 0.28 m mast, 0.5 m arm diagonal); a centimetre value
///   typed for metres (28) is refused;
///   roll, yaw [-pi, pi] (every angle once); pitch [-pi/2, pi/2] (the RPY convention's unique range).
inline constexpr auto kLioNodeSpecs = std::to_array<config::ParamSpec>({
    {"extrinsic_base_lidar_x_m", config::Unit::kMeters, -1.0, 1.0},
    {"extrinsic_base_lidar_y_m", config::Unit::kMeters, -1.0, 1.0},
    {"extrinsic_base_lidar_z_m", config::Unit::kMeters, -1.0, 1.0},
    {"extrinsic_base_lidar_roll_rad", config::Unit::kRadians, -std::numbers::pi, std::numbers::pi},
    {"extrinsic_base_lidar_pitch_rad", config::Unit::kRadians, -std::numbers::pi / 2.0, std::numbers::pi / 2.0},
    {"extrinsic_base_lidar_yaw_rad", config::Unit::kRadians, -std::numbers::pi, std::numbers::pi},
});

struct LioNodeConfig {
  Eigen::Isometry3d base_T_lidar;  ///< maps LiDAR-frame points into base_link
};

/// Builds the mount from values loaded against kLioNodeSpecs (config::load_params_file). Every key is re-checked
/// (hand-built values never went through load_params): kMissingKey, kNotFinite, kOutOfRange naming the key.
Result<LioNodeConfig, config::ConfigError> load_lio_node_config(const config::ParamValues& values);

/// base_T_imu = base_T_lidar * imu_T_lidar^-1 (O20), with imu_T_lidar the LIO core's extrinsic: translation
/// lio.math.t_imu_lidar_m (the LiDAR origin in the IMU frame), identity rotation, exactly as LioRunner::create
/// checks it (backend_math.cpp check_frames).
Eigen::Isometry3d imu_T_lidar(const lio::LioConfig& lio);
Eigen::Isometry3d compose_base_T_imu(const LioNodeConfig& node, const lio::LioConfig& lio);

}  // namespace uavnav::lio_node
