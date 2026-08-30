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
  double lower = 0.0;
  double upper = configured_speed_mps;
  if (jerkLimitedStopDistance(upper, max_acceleration_mps2, max_jerk_mps3) <=
      result.support_m) {
    result.speed_mps = upper;
    result.sufficient = true;
    return result;
  }
  for (int iteration = 0; iteration < 32 && upper - lower > 1.0e-3;
       ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (jerkLimitedStopDistance(
            middle, max_acceleration_mps2, max_jerk_mps3) <= result.support_m) {
      lower = middle;
    } else {
      upper = middle;
    }
  }
  result.speed_mps = lower;
  result.sufficient = lower > 0.0 &&
      jerkLimitedStopDistance(lower, max_acceleration_mps2, max_jerk_mps3) <=
          result.support_m + 1.0e-9;
  return result;
}

}  // namespace navigation_planning_backend
