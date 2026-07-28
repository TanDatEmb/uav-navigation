#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/pipeline/estimator_diagnostics.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

struct StateEstimate {
  Timestamp time;
  ManifoldState state;
  ManifoldState::Covariance covariance{ManifoldState::Covariance::Identity()};

  [[nodiscard]] bool allFinite() const noexcept {
    return state.allFinite() && covariance.allFinite();
  }
};

struct ProcessResult {
  EstimatorStatus status_before{EstimatorStatus::kWaitingForSensors};
  EstimatorStatus status_after{EstimatorStatus::kWaitingForSensors};
  std::optional<Timestamp> scan_time;
  EstimateValidity estimate_validity{EstimateValidity::kUnavailable};
  LidarUpdateStatus lidar_update_status{LidarUpdateStatus::kNotAttempted};
  std::optional<StateEstimate> predicted_estimate;
  std::optional<StateEstimate> corrected_estimate;
  std::optional<Timestamp> last_lidar_correction_time;
  // Both point arrays are explicitly expressed in odom.
  std::vector<Eigen::Vector3d> registered_points_odom_m;
  std::vector<Eigen::Vector3d> local_map_points_odom_m;
  EstimatorDiagnostics diagnostics;
  std::string rejection_reason;

  [[nodiscard]] bool hasPredictedOutput() const noexcept {
    return predicted_estimate.has_value() && predicted_estimate->allFinite() &&
           estimate_validity != EstimateValidity::kUnavailable;
  }

  [[nodiscard]] bool hasCorrectedOutput() const noexcept {
    return corrected_estimate.has_value() && corrected_estimate->allFinite() &&
           estimate_validity == EstimateValidity::kCorrected &&
           lidar_update_status == LidarUpdateStatus::kSucceeded;
  }

  [[nodiscard]] bool hasRegisteredScanOutput() const noexcept {
    return hasCorrectedOutput() && !registered_points_odom_m.empty();
  }
};

}  // namespace uav::nav::lio
