#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "fast_lio_core/estimation/state_estimate.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/navigation/kinematic_state_estimate.hpp"
#include "fast_lio_core/pipeline/estimator_diagnostics.hpp"

namespace uav::nav::lio {

struct ProcessResult {
  EstimatorStatus status_before{EstimatorStatus::kWaitingForSensors};
  EstimatorStatus status_after{EstimatorStatus::kWaitingForSensors};
  std::optional<Timestamp> scan_time;
  EstimateValidity estimate_validity{EstimateValidity::kUnavailable};
  LidarUpdateStatus lidar_update_status{LidarUpdateStatus::kNotAttempted};
  std::optional<StateEstimate> predicted_estimate;
  std::optional<StateEstimate> corrected_estimate;
  std::optional<KinematicStateEstimate> corrected_kinematic_estimate;
  std::optional<Timestamp> last_lidar_correction_time;
  // Both point arrays are explicitly expressed in odom.
  std::vector<Eigen::Vector3d> registered_points_odom_m;
  // Common-filtered (pre estimator-voxelization) points for the navigation
  // world model, expressed in the sensor (livox) frame at `scan_time`. Only
  // populated when EstimatorConfig::preprocessing.retain_mapping_candidate is
  // set and preprocessing succeeded; consumers must additionally gate on
  // hasCorrectedOutput() before treating this as a valid mapping observation.
  std::vector<Eigen::Vector3d> mapping_candidate_points_lidar_m;
  // ^lio_odom T_livox at exactly `scan_time`, derived from the same corrected
  // estimator state as `corrected_estimate`. Only set alongside a successful
  // correction; never derived from predicted/propagated state.
  std::optional<RigidTransform> sensor_pose_odom_lidar;
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

  // The mapping observation contract additionally requires a valid corrected
  // sensor pose. It deliberately does not require non-empty points: an empty
  // filtered scan is still a valid (if uninformative) observation epoch.
  [[nodiscard]] bool hasMappingObservationOutput() const noexcept {
    return hasCorrectedOutput() && sensor_pose_odom_lidar.has_value() &&
           sensor_pose_odom_lidar->allFinite();
  }
};

}  // namespace uav::nav::lio

