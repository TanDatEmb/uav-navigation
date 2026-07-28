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

}  // namespace uav::nav::lio
