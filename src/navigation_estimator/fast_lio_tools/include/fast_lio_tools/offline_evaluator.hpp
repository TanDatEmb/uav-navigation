#pragma once

#include "fast_lio_tools/evaluation_config.hpp"
#include "fast_lio_tools/evaluation_metrics.hpp"

namespace uav::nav::lio {

class OfflineEvaluator {
 public:
  explicit OfflineEvaluator(EvaluationConfig config);
  [[nodiscard]] EvaluationMetrics run();

 private:
  EvaluationConfig config_;
};

}  // namespace uav::nav::lio
