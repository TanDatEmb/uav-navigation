#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <planner_core/backup_braking.hpp>

namespace navigation_planning_backend {

struct EvidenceSpeedLimit final {
  double speed_mps{0.0};
  double support_m{0.0};
  bool sufficient{false};
};

// The same jerk-limited stopping model used by BACKUP is inverted over the
// weakest available support. The fixed iteration/tolerance bounds make this
// deterministic and prevent an evidence gap from becoming an optimistic cap.
[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimit(
    const navigation_math::StatePVAJ& state,
    const navigation_planning::DynamicLimits& dynamics,
    const std::array<double, 4>& support_m) noexcept {
  EvidenceSpeedLimit result;
  if (!state.allFinite() || !dynamics.valid()) return result;
  result.support_m = std::numeric_limits<double>::infinity();
  for (const double support : support_m) {
    if (!std::isfinite(support) || support <= 0.0) {
      result.support_m = 0.0;
      return result;
    }
    result.support_m = std::min(result.support_m, support);
  }
  const Eigen::Vector3d direction = state.col(1).norm() > 1.0e-9
      ? state.col(1).normalized() : Eigen::Vector3d::UnitX();
  double lower = 0.0;
  double upper = dynamics.intent.requested_cruise_speed_mps;
  auto candidate = [&](const double speed_mps) {
    auto candidate_state = state;
    candidate_state.col(1) = direction * speed_mps;
    return evaluateStopReachability(candidate_state, dynamics,
                                    result.support_m)
        .stopping_distance_m;
  };
  if (candidate(upper) <= result.support_m) {
    result.speed_mps = upper;
    result.sufficient = true;
    return result;
  }
  for (int iteration = 0; iteration < 32 && upper - lower > 1.0e-3;
       ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (candidate(middle) <= result.support_m) lower = middle;
    else upper = middle;
  }
  result.speed_mps = lower;
  result.sufficient = lower > 0.0 && candidate(lower) <= result.support_m + 1.0e-9;
  return result;
}

[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimit(
    double configured_speed_mps, double max_acceleration_mps2,
    double max_jerk_mps3, const std::array<double, 4>& support_m) noexcept {
  EvidenceSpeedLimit result;
  if (!std::isfinite(configured_speed_mps) || configured_speed_mps <= 0.0 ||
      !std::isfinite(max_acceleration_mps2) || max_acceleration_mps2 <= 0.0 ||
      !std::isfinite(max_jerk_mps3) || max_jerk_mps3 <= 0.0) {
    return result;
  }
  result.support_m = std::numeric_limits<double>::infinity();
  for (const double support : support_m) {
    if (!std::isfinite(support) || support <= 0.0) {
      result.support_m = 0.0;
      return result;
    }
    result.support_m = std::min(result.support_m, support);
  }
  navigation_math::StatePVAJ state = navigation_math::StatePVAJ::Zero();
  state.col(1).x() = configured_speed_mps;
  navigation_planning::DynamicLimits dynamics;
  dynamics.vehicle.maximum_velocity_mps = configured_speed_mps;
  dynamics.vehicle.maximum_acceleration_mps2 = max_acceleration_mps2;
  dynamics.vehicle.maximum_jerk_mps3 = max_jerk_mps3;
  dynamics.intent.requested_cruise_speed_mps = configured_speed_mps;
  return evidenceAwareSpeedLimit(state, dynamics, support_m);
}

}  // namespace navigation_planning_backend
