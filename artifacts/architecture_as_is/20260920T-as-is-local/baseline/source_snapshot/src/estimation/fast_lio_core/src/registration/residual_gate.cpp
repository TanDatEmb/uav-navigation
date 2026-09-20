#include "fast_lio_core/registration/residual_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uav::nav::lio {

ResidualGate::ResidualGate(ResidualGateConfig config) : config_(config) {
  if (!(config_.maximum_absolute_distance_m > 0.0) || !(config_.minimum_planarity >= 0.0) ||
      config_.minimum_planarity > 1.0 || !(config_.huber_delta_m > 0.0)) {
    throw std::invalid_argument("invalid residual gate configuration");
  }
}

ResidualGateDecision ResidualGate::evaluate(const Plane& plane,
                                            double signed_distance_m) const noexcept {
  ResidualGateDecision decision;
  if (!plane.centroid_odom_m.allFinite() || !plane.normal_odom.allFinite() ||
      !std::isfinite(plane.planarity) || !std::isfinite(signed_distance_m)) {
    decision.reason = ResidualRejectionReason::kNonFinite;
    return decision;
  }
  if (plane.planarity < config_.minimum_planarity) {
    decision.reason = ResidualRejectionReason::kLowPlanarity;
    return decision;
  }
  const double absolute_distance = std::abs(signed_distance_m);
  if (absolute_distance > config_.maximum_absolute_distance_m) {
    decision.reason = ResidualRejectionReason::kDistanceTooLarge;
    return decision;
  }
  decision.accepted = true;
  decision.reason = ResidualRejectionReason::kAccepted;
  decision.robust_weight = absolute_distance <= config_.huber_delta_m
                               ? 1.0
                               : config_.huber_delta_m / std::max(absolute_distance, 1e-12);
  return decision;
}

std::string_view toString(ResidualRejectionReason reason) noexcept {
  switch (reason) {
    case ResidualRejectionReason::kAccepted:
      return "ACCEPTED";
    case ResidualRejectionReason::kNonFinite:
      return "NON_FINITE";
    case ResidualRejectionReason::kLowPlanarity:
      return "LOW_PLANARITY";
    case ResidualRejectionReason::kDistanceTooLarge:
      return "DISTANCE_TOO_LARGE";
  }
  return "UNKNOWN";
}

}  // namespace uav::nav::lio
