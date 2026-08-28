#pragma once

#include <cstdint>

#include <Eigen/Core>

#include <navigation_mission/route_progress.hpp>

namespace navigation_planning_backend {

struct RouteYawConfig {
  double minimum_lookahead_m{2.0};
  double maximum_lookahead_m{12.0};
  double lookahead_time_s{1.5};
  double minimum_horizontal_speed_mps{0.3};
  double minimum_horizontal_support_m{0.5};
  double reversal_threshold_rad{2.6179938779914944};  // 150 degrees.

  [[nodiscard]] bool valid() const noexcept;
};

enum class RouteYawSource : std::uint8_t {
  kRouteLookahead = 0U,
  kHoldLowSpeed = 1U,
  kHoldNoHorizontalSupport = 2U,
  kRouteTurnInPlace = 3U,
  kInvalidRoute = 4U,
};

struct RouteYawReference {
  bool valid{false};
  double target_yaw_rad{0.0};
  double lookahead_m{0.0};
  double progress_arc_m{0.0};
  Eigen::Vector3d target_point{Eigen::Vector3d::Zero()};
  RouteYawSource source{RouteYawSource::kInvalidRoute};
};

// Derive semantic yaw from measured position and immutable mission-route
// geometry. Position-trajectory shape is deliberately absent from this API.
[[nodiscard]] RouteYawReference computeRouteYawReference(
    const navigation_mission::ImmutableRouteSnapshot& route,
    const Eigen::Vector3d& measured_position,
    const Eigen::Vector3d& measured_velocity,
    double measured_yaw_rad,
    const RouteYawConfig& config = {}) noexcept;

}  // namespace navigation_planning_backend
