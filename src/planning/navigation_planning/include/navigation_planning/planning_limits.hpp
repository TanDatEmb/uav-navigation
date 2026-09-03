#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_planning {

// The world-model contract owns the meaning of UNKNOWN. Keep this alias for
// planning callers while preventing a second, silently divergent enum.
using UnknownSpacePolicy = navigation_world_model::UnknownPolicy;

struct VehicleDynamicModel {
  double maximum_velocity_mps{12.0};
  double maximum_acceleration_mps2{12.0};
  double maximum_jerk_mps3{30.0};
  double maximum_body_rate_rad_s{5.0};
  double maximum_yaw_rate_rad_s{5.0};
  double maximum_yaw_acceleration_rad_s2{0.3};
  double minimum_thrust_n{6.0 * 1.64};
  double maximum_thrust_n{25.0 * 1.64};
  double mass_kg{1.64};

  [[nodiscard]] bool valid() const noexcept {
    return std::isfinite(maximum_velocity_mps) && maximum_velocity_mps > 0.0 &&
           std::isfinite(maximum_acceleration_mps2) && maximum_acceleration_mps2 > 0.0 &&
           std::isfinite(maximum_jerk_mps3) && maximum_jerk_mps3 > 0.0 &&
           std::isfinite(maximum_body_rate_rad_s) && maximum_body_rate_rad_s > 0.0 &&
           std::isfinite(maximum_yaw_rate_rad_s) && maximum_yaw_rate_rad_s > 0.0 &&
           std::isfinite(maximum_yaw_acceleration_rad_s2) &&
           maximum_yaw_acceleration_rad_s2 > 0.0 &&
           std::isfinite(minimum_thrust_n) && minimum_thrust_n >= 0.0 &&
           std::isfinite(maximum_thrust_n) && maximum_thrust_n > minimum_thrust_n &&
           std::isfinite(mass_kg) && mass_kg > 0.0;
  }
};

struct MotionIntent {
  double requested_cruise_speed_mps{0.0};

  [[nodiscard]] bool valid(const VehicleDynamicModel& model) const noexcept {
    return model.valid() && std::isfinite(requested_cruise_speed_mps) &&
           requested_cruise_speed_mps > 0.0 &&
           requested_cruise_speed_mps <= model.maximum_velocity_mps;
  }
};

// Solve-time bundle: the physical model is immutable authority; intent only
// requests a cruise speed and never changes acceleration/jerk limits.
struct DynamicLimits {
  VehicleDynamicModel vehicle{};
  MotionIntent intent{};
  UnknownSpacePolicy unknown_space_policy{UnknownSpacePolicy::kRequireKnownFree};

  [[nodiscard]] bool valid() const noexcept {
    return intent.valid(vehicle) &&
           (unknown_space_policy == UnknownSpacePolicy::kAllowUnknown ||
            unknown_space_policy == UnknownSpacePolicy::kRequireKnownFree);
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
