#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_planning {

// The world-model contract owns the meaning of UNKNOWN. Keep this alias for
// planning callers while preventing a second, silently divergent enum.
using UnknownSpacePolicy = navigation_world_model::UnknownPolicy;

struct DynamicLimits {
  double max_velocity_mps{0.0};
  double max_acceleration_mps2{0.0};
  double max_jerk_mps3{0.0};
  UnknownSpacePolicy unknown_space_policy{UnknownSpacePolicy::kRequireKnownFree};

  [[nodiscard]] bool valid() const noexcept {
    return std::isfinite(max_velocity_mps) && std::isfinite(max_acceleration_mps2) &&
           std::isfinite(max_jerk_mps3) && max_velocity_mps > 0.0 &&
           max_acceleration_mps2 > 0.0 && max_jerk_mps3 > 0.0;
  }
};

// A trajectory extremum is evaluated through several polynomial transforms
// and can land a handful of ULPs above a limit that it reaches exactly. This
// is numerical boundary accounting, not a physical allowance: the accepted
// margin is bounded to a fixed number of representable double values and is
// far below any mission-configurable unit. Material overspeed or
// over-acceleration remains rejected.
inline bool withinNumericalDynamicLimit(const double value,
                                        const double limit) noexcept {
  if (!std::isfinite(value) || !std::isfinite(limit) || limit < 0.0) {
    return false;
  }
  constexpr double kBoundaryUlps = 64.0;
  const double scale = std::max({1.0, std::abs(value), std::abs(limit)});
  const double numerical_margin =
      kBoundaryUlps * std::numeric_limits<double>::epsilon() * scale;
  return value <= limit + numerical_margin;
}

}  // namespace navigation_planning
