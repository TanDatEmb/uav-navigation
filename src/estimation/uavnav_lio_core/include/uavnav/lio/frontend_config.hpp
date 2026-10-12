#pragma once

#include "uavnav/core/time.hpp"
#include "uavnav/lio/lifecycle.hpp"
#include "uavnav/lio/output_predictor.hpp"

// Configuration of the LIO frontend (ingest thread), SYSTEM_DESIGN §3.5, D32: the lifecycle
// thresholds, the output predictor's time constants and the IMU-gap event threshold of the ingest
// checks. Loaded through config.hpp (kLioSpecs, load_lio_config) and handed over with
// frontend_config(). Must not include backend_config.hpp or config.hpp.
namespace uavnav::lio {

struct FrontendConfig {
  LifecycleConfig lifecycle;
  PredictorConfig predictor;
  /// An IMU step longer than this is reported as an "ImuGap" event (S1a ingest check). The same key
  /// as the backend's largest integration step (imu_max_gap_s, EstimatorMathConfig::imu_max_gap):
  /// one source.
  time::Duration imu_max_gap;
};

}  // namespace uavnav::lio
