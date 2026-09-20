#include "fast_lio_core/pipeline/estimator_diagnostics.hpp"

namespace uav::nav::lio {

const char* toString(EstimatorStatus status) noexcept {
  switch (status) {
    case EstimatorStatus::kWaitingForSensors:
      return "WAITING_FOR_SENSORS";
    case EstimatorStatus::kCollectingImu:
      return "COLLECTING_IMU";
    case EstimatorStatus::kInitializingImu:
      return "INITIALIZING_IMU";
    case EstimatorStatus::kInitializingMap:
      return "INITIALIZING_MAP";
    case EstimatorStatus::kTracking:
      return "TRACKING";
    case EstimatorStatus::kDegraded:
      return "DEGRADED";
    case EstimatorStatus::kLost:
      return "LOST";
    case EstimatorStatus::kResetting:
      return "RESETTING";
  }
  return "UNKNOWN";
}

const char* toString(EstimateValidity validity) noexcept {
  switch (validity) {
    case EstimateValidity::kUnavailable:
      return "UNAVAILABLE";
    case EstimateValidity::kPredictedOnly:
      return "PREDICTED_ONLY";
    case EstimateValidity::kCorrected:
      return "CORRECTED";
  }
  return "UNKNOWN";
}

const char* toString(LidarUpdateStatus status) noexcept {
  switch (status) {
    case LidarUpdateStatus::kNotAttempted:
      return "NOT_ATTEMPTED";
    case LidarUpdateStatus::kSucceeded:
      return "SUCCEEDED";
    case LidarUpdateStatus::kRejected:
      return "REJECTED";
  }
  return "UNKNOWN";
}

const char* toString(LidarUpdateFailureClass failure_class) noexcept {
  switch (failure_class) {
    case LidarUpdateFailureClass::kNone:
      return "none";
    case LidarUpdateFailureClass::kSynchronization:
      return "synchronization";
    case LidarUpdateFailureClass::kPrediction:
      return "prediction";
    case LidarUpdateFailureClass::kDeskew:
      return "deskew";
    case LidarUpdateFailureClass::kPreprocessing:
      return "preprocessing";
    case LidarUpdateFailureClass::kInsufficientPoints:
      return "insufficient_points";
    case LidarUpdateFailureClass::kRegistration:
      return "registration";
    case LidarUpdateFailureClass::kPropagationDiscontinuity:
      return "propagation_discontinuity";
    case LidarUpdateFailureClass::kStateCorruption:
      return "state_corruption";
  }
  return "unknown";
}

}  // namespace uav::nav::lio
