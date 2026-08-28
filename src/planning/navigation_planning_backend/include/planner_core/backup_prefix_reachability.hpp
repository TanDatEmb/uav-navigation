#pragma once

#include <cmath>

#include <Eigen/Core>

namespace navigation_planning_backend {

struct BackupPrefixAdvance {
  bool admissible{false};
  double accumulated_length_m{0.0};
};

// Advance a backup-reachable MAIN prefix along its actual curve. The supplied
// segment oracle owns inflated-map/UNKNOWN semantics; this helper owns only
// finite geometry and the cumulative visibility-distance budget.
template <typename SegmentKnownFree>
BackupPrefixAdvance advanceBackupReachablePrefix(
    const Eigen::Vector3d& previous,
    const Eigen::Vector3d& next,
    const double accumulated_length_m,
    const double maximum_length_m,
    const double numerical_allowance_m,
    SegmentKnownFree&& segment_known_free) {
  BackupPrefixAdvance result;
  if (!previous.allFinite() || !next.allFinite() ||
      !std::isfinite(accumulated_length_m) || accumulated_length_m < 0.0 ||
      !std::isfinite(maximum_length_m) ||
      !std::isfinite(numerical_allowance_m) || numerical_allowance_m < 0.0) {
    return result;
  }
  const double segment_length_m = (next - previous).norm();
  result.accumulated_length_m = accumulated_length_m + segment_length_m;
  if (!std::isfinite(segment_length_m) ||
      (maximum_length_m > 0.0 &&
       result.accumulated_length_m > maximum_length_m + numerical_allowance_m) ||
      !segment_known_free(previous, next)) {
    return result;
  }
  result.admissible = true;
  return result;
}

}  // namespace navigation_planning_backend
