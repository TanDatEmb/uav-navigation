#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace navigation_planning_backend {

// Produce a small deterministic search between the analytically required
// reserve and the former coarse 1.5x jump. Fixed non-zero PVAJ boundaries make
// dynamic extrema non-monotone under time scaling, so every returned proposal
// remains only a candidate for the complete corridor and dynamic certificate.
inline std::vector<double> boundedTimeStretchReserveScales(
    double initial_scale, double maximum_scale = 4.0) {
  if (!std::isfinite(initial_scale) || !std::isfinite(maximum_scale) ||
      initial_scale <= 1.0 || maximum_scale <= 1.0) {
    return {};
  }
  initial_scale = std::min(initial_scale, maximum_scale);
  std::vector<double> scales;
  for (const double multiplier : {1.0, 1.10, 1.25, 1.50}) {
    const double candidate = std::min(maximum_scale, initial_scale * multiplier);
    if (candidate <= 1.0 ||
        (!scales.empty() && candidate <= scales.back() + 1.0e-9)) {
      continue;
    }
    scales.push_back(candidate);
  }
  if (scales.empty() || maximum_scale > scales.back() + 1.0e-9) {
    scales.push_back(maximum_scale);
  }
  return scales;
}

}  // namespace navigation_planning_backend
