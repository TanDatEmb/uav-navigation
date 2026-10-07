#pragma once

#include "uavnav/lio/lifecycle.hpp"
#include "uavnav/lio/output_predictor.hpp"

// Configuration of the LIO frontend (ingest thread), SYSTEM_DESIGN §3.5, D32: the lifecycle thresholds and
// the output predictor's time constants. Loaded through config.hpp (kLioSpecs, load_lio_config) and handed
// over with frontend_config(). Must not include backend_config.hpp or config.hpp.
namespace uavnav::lio {

struct FrontendConfig {
  LifecycleConfig lifecycle;
  PredictorConfig predictor;
};

}  // namespace uavnav::lio
