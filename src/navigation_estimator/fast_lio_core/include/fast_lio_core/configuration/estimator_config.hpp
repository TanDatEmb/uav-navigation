#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>

#include "fast_lio_core/deskew/scan_deskewer.hpp"
#include "fast_lio_core/estimation/ikfom_estimator.hpp"
#include "fast_lio_core/initialization/imu_initializer.hpp"
#include "fast_lio_core/mapping/ikd_tree_registration_map.hpp"
#include "fast_lio_core/mapping/dynamic_map_evidence.hpp"
#include "fast_lio_core/mapping/local_map_manager.hpp"
#include "fast_lio_core/mapping/map_insertion_policy.hpp"
#include "fast_lio_core/preprocessing/point_cloud_preprocessor.hpp"
#include "fast_lio_core/registration/residual_builder.hpp"
#include "fast_lio_core/synchronization/measurement_buffer.hpp"
#include "fast_lio_core/synchronization/measurement_synchronizer.hpp"

namespace uav::nav::lio {

struct ExtrinsicConfig {
  Eigen::Quaterniond rotation_imu_lidar{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d translation_imu_lidar_m{Eigen::Vector3d::Zero()};
  bool estimate_online{false};
};

struct LifecycleConfig {
  std::size_t maximum_initial_map_registration_failures{10};
  std::size_t degraded_after_registration_failures{1};
  std::size_t lost_after_registration_failures{5};
  std::size_t local_map_snapshot_period_scans{10};
};

struct EstimatorConfig {
  MeasurementBufferConfig measurement_buffer{};
  MeasurementSynchronizerConfig synchronization{};
  ImuInitializerConfig initialization{};
  IkfomEstimatorConfig ikfom{};
  ScanDeskewerConfig deskew{};
  PointCloudPreprocessorConfig preprocessing{};
  ResidualBuilderConfig residual_builder{};
  IkdTreeRegistrationMapConfig registration_map{};
  LocalMapManagerConfig local_map{};
  DynamicFilterConfig dynamic_filter{};
  MapInsertionPolicyConfig insertion_policy{};
  ExtrinsicConfig extrinsic{};
  LifecycleConfig lifecycle{};
};

}  // namespace uav::nav::lio
