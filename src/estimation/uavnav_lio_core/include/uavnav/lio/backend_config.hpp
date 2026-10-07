#pragma once

#include <Eigen/Core>
#include <cstdint>

#include "uavnav/core/time.hpp"
#include "uavnav/lio/degeneracy.hpp"

// Configuration of the LIO backend (estimator thread), SYSTEM_DESIGN §3.5, D32. Loaded through config.hpp
// (kLioSpecs, load_lio_config) and handed over with backend_config(). This header must never include the
// lifecycle or the output predictor, directly or indirectly: backend sources fail to compile when the
// lifecycle header is reachable (D32 marker UAVNAV_LIO_LIFECYCLE_HPP_INCLUDED).
namespace uavnav::lio {

/// Values handed to the reused FAST-LIO math (fast_lio_core) by the backend. Beta values (from the survey
/// of `main`, config/runtime/sim.yaml) in parentheses. Every other fast_lio_core knob keeps the default of
/// its own config struct (single source: fast_lio_core), see backend.cpp.
struct EstimatorMathConfig {
  Eigen::Vector3d t_imu_lidar_m;         ///< IMU -> LiDAR translation, IMU frame ((-0.011, -0.02329, 0.04412))
  double preprocess_min_range_m;         ///< points closer than this are dropped (0.5)
  double preprocess_max_range_m;         ///< points farther than this are dropped (40)
  double preprocess_voxel_m;             ///< scan voxel filter edge (0.2)
  double map_voxel_m;                    ///< ikd-tree downsample voxel edge (0.3)
  Eigen::Vector3d map_half_extent_m;     ///< local map half extents x, y, z ((30, 30, 15))
  std::uint32_t registration_max_iterations;  ///< IKFoM iterations per scan (4)
  std::uint32_t imu_init_min_samples;    ///< stationary samples for gravity + bias init (200)
  time::Duration imu_max_gap;            ///< largest IMU step integrated / reported as a gap (0.02 s)
};

struct BackendConfig {
  EstimatorMathConfig math;
  DegeneracyConfig degeneracy;
};

}  // namespace uavnav::lio
