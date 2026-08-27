#pragma once

#include <cmath>

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

}  // namespace navigation_planning
