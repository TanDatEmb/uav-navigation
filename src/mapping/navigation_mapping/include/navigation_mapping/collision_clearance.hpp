#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace navigation_mapping {

// The LiDAR endpoint range filter and vehicle collision envelope are separate
// physical contracts. An unavailable value is deliberate until a model-owned
// vehicle geometry source is added to the repository.
struct CollisionClearanceConfig {
  double vehicle_radius_m{std::numeric_limits<double>::quiet_NaN()};
  double safety_margin_m{std::numeric_limits<double>::quiet_NaN()};

  [[nodiscard]] bool configured() const noexcept {
    return std::isfinite(vehicle_radius_m) && vehicle_radius_m >= 0.0 &&
           std::isfinite(safety_margin_m) && safety_margin_m >= 0.0;
  }

  [[nodiscard]] double clearanceRadiusOrNaN() const noexcept {
    return configured() ? vehicle_radius_m + safety_margin_m
                        : std::numeric_limits<double>::quiet_NaN();
  }

  void validate() const {
    const bool any_configured = std::isfinite(vehicle_radius_m) ||
                                std::isfinite(safety_margin_m);
    if (!any_configured) return;
    if (!configured()) {
      throw std::invalid_argument(
          "CollisionClearanceConfig: both radii must be finite and non-negative");
    }
    if (!std::isfinite(vehicle_radius_m + safety_margin_m)) {
      throw std::invalid_argument("CollisionClearanceConfig: clearance radius overflow");
    }
  }
};

// ROG creates spherical coarse-cell neighbors using integer offsets. For the
// metric contract, one vendor step is one inflation-resolution radius; round
// upward so the configured radius is never smaller than the requested one.
inline int minimumRogInflationStep(double clearance_radius_m,
                                   double inflation_resolution_m) {
  if (!std::isfinite(clearance_radius_m) || clearance_radius_m < 0.0 ||
      !std::isfinite(inflation_resolution_m) || inflation_resolution_m <= 0.0) {
    throw std::invalid_argument(
        "minimumRogInflationStep: radii must be finite and non-negative");
  }
  return std::max(1, static_cast<int>(std::ceil(
                         clearance_radius_m / inflation_resolution_m)));
}

inline double guaranteedRogInflationRadius(int inflation_step,
                                           double inflation_resolution_m) {
  if (inflation_step <= 0 || !std::isfinite(inflation_resolution_m) ||
      inflation_resolution_m <= 0.0) {
    throw std::invalid_argument(
        "guaranteedRogInflationRadius: invalid vendor inflation geometry");
  }
  return static_cast<double>(inflation_step) * inflation_resolution_m;
}

}  // namespace navigation_mapping
