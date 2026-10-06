#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fast_lio_core/deskew/scan_deskewer.hpp"
#include "fast_lio_core/estimation/ikfom_estimator.hpp"
#include "fast_lio_core/initialization/imu_initializer.hpp"
#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"
#include "fast_lio_core/mapping/local_map_manager.hpp"
#include "fast_lio_core/preprocessing/point_cloud_preprocessor.hpp"
#include "fast_lio_core/registration/residual_builder.hpp"

namespace uav::nav::lio {

struct ExtrinsicConfig {
  Eigen::Quaterniond rotation_imu_lidar{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d translation_imu_lidar_m{Eigen::Vector3d::Zero()};
};

// Configuration of the math components only. Lifecycle, tracking, buffering
// and synchronization policy belong to uavnav_lio_core.
struct EstimatorConfig {
  ImuInitializerConfig initialization{};
  IkfomEstimatorConfig ikfom{};
  ScanDeskewerConfig deskew{};
  PointCloudPreprocessorConfig preprocessing{};
  ResidualBuilderConfig residual_builder{};
  IkdTreeRegistrationMapConfig registration_map{};
  LocalMapManagerConfig local_map{};
  ExtrinsicConfig extrinsic{};
};

}  // namespace uav::nav::lio
