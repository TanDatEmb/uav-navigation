#pragma once

#include <cmath>

namespace navigation_planning {

struct DynamicLimits {
  double max_velocity_mps{0.0};
  double max_acceleration_mps2{0.0};
  double max_jerk_mps3{0.0};

  [[nodiscard]] bool finitePositive() const noexcept {
    return std::isfinite(max_velocity_mps) && std::isfinite(max_acceleration_mps2) &&
           std::isfinite(max_jerk_mps3) && max_velocity_mps > 0.0 &&
           max_acceleration_mps2 > 0.0 && max_jerk_mps3 > 0.0;
  }
};

}  // namespace navigation_planning
