#pragma once

#include <string_view>

#include "fast_lio_core/registration/correspondence.hpp"

namespace uav::nav::lio {

enum class ResidualRejectionReason {
  kAccepted,
  kNonFinite,
  kLowPlanarity,
  kDistanceTooLarge,
};

struct ResidualGateConfig {
  double maximum_absolute_distance_m{0.30};
  double minimum_planarity{0.90};
  double huber_delta_m{0.10};
};

struct ResidualGateDecision {
  bool accepted{false};
  double robust_weight{0.0};
  ResidualRejectionReason reason{ResidualRejectionReason::kNonFinite};
};

class ResidualGate {
 public:
  explicit ResidualGate(ResidualGateConfig config = {});

  [[nodiscard]] ResidualGateDecision evaluate(const Plane& plane,
                                              double signed_distance_m) const noexcept;

 private:
  ResidualGateConfig config_;
};

[[nodiscard]] std::string_view toString(ResidualRejectionReason reason) noexcept;

}  // namespace uav::nav::lio
