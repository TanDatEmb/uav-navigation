#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/pipeline/estimator_diagnostics.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

struct ProcessResult {
  EstimatorStatus status_before{EstimatorStatus::kWaitingForSensors};
  EstimatorStatus status_after{EstimatorStatus::kWaitingForSensors};
  std::optional<Timestamp> scan_time;
  bool lidar_correction_successful{false};
  bool has_corrected_odometry{false};
  ManifoldState corrected_state{};
  ManifoldState::Covariance corrected_covariance{ManifoldState::Covariance::Identity()};
  // Both point arrays are explicitly expressed in odom.
  std::vector<Eigen::Vector3d> registered_points_odom_m;
  std::vector<Eigen::Vector3d> local_map_points_odom_m;
  EstimatorDiagnostics diagnostics;
  std::string rejection_reason;
};

}  // namespace uav::nav::lio
