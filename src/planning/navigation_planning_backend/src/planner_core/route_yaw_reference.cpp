#include "planner_core/route_yaw_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation_planning_backend {
namespace {

constexpr double kGeometryTieM2 = 1.0e-12;

double unwrapNear(const double anchor, const double target) noexcept {
  if (!std::isfinite(anchor) || !std::isfinite(target)) return anchor;
  // Reduce both operands before subtraction.  A finite but extreme measured
  // yaw can otherwise overflow target-anchor and poison the reference with
  // NaN even though the desired route bearing is representable.
  const double normalized_anchor = std::remainder(anchor, 2.0 * M_PI);
  const double normalized_target = std::remainder(target, 2.0 * M_PI);
  const double result = normalized_anchor +
      std::remainder(normalized_target - normalized_anchor, 2.0 * M_PI);
  return std::isfinite(result) ? result : anchor;
}

double projectedProgressArc(
    const navigation_mission::ImmutableRouteSnapshot& route,
    const Eigen::Vector3d& position) noexcept {
  if (route.segments.empty()) return 0.0;
  double best_distance_m2 = std::numeric_limits<double>::infinity();
  double best_arc = route.measured_progress.progress_arc_m;
  double best_progress_distance = std::numeric_limits<double>::infinity();
  for (const auto& segment : route.segments) {
    const Eigen::Vector3d delta = position - segment.start;
    const double fraction = std::clamp(
        delta.dot(segment.tangent) / segment.length_m, 0.0, 1.0);
    const Eigen::Vector3d projected =
        segment.start + fraction * (segment.end - segment.start);
    const double distance_m2 = (position - projected).squaredNorm();
    const double arc = segment.start_arc_m + fraction * segment.length_m;
    if (!std::isfinite(distance_m2) || !std::isfinite(arc)) continue;
    const double progress_distance =
        std::abs(arc - route.measured_progress.progress_arc_m);
    const bool geometrically_better = distance_m2 + kGeometryTieM2 < best_distance_m2;
    const bool tied_but_progress_better =
        std::abs(distance_m2 - best_distance_m2) <= kGeometryTieM2 &&
        (progress_distance + 1.0e-9 < best_progress_distance ||
         (std::abs(progress_distance - best_progress_distance) <= 1.0e-9 &&
          arc > best_arc));
    if (!geometrically_better && !tied_but_progress_better) continue;
    best_distance_m2 = distance_m2;
    best_progress_distance = progress_distance;
    best_arc = arc;
  }
  const double active_boundary =
      route.waypoint_arc_lengths_m[route.active_waypoint_index];
  return std::min(active_boundary,
                  std::max(route.measured_progress.progress_arc_m, best_arc));
}

RouteYawReference holdReference(
    const double yaw, const double progress, const Eigen::Vector3d& point,
    const RouteYawSource source) noexcept {
  RouteYawReference output;
  output.valid = std::isfinite(yaw) && source != RouteYawSource::kInvalidRoute;
  output.target_yaw_rad = std::isfinite(yaw) ? yaw : 0.0;
  output.progress_arc_m = progress;
  output.target_point = point;
  output.source = source;
  return output;
}

}  // namespace

bool RouteYawConfig::valid() const noexcept {
  return std::isfinite(minimum_lookahead_m) && minimum_lookahead_m > 0.0 &&
         std::isfinite(maximum_lookahead_m) &&
         maximum_lookahead_m >= minimum_lookahead_m &&
         std::isfinite(lookahead_time_s) && lookahead_time_s > 0.0 &&
         std::isfinite(minimum_horizontal_speed_mps) &&
         minimum_horizontal_speed_mps >= 0.0 &&
         std::isfinite(minimum_horizontal_support_m) &&
         minimum_horizontal_support_m > 0.0 &&
         std::isfinite(reversal_threshold_rad) &&
         reversal_threshold_rad > M_PI_2 && reversal_threshold_rad <= M_PI;
}

RouteYawReference computeRouteYawReference(
    const navigation_mission::ImmutableRouteSnapshot& route,
    const Eigen::Vector3d& measured_position,
    const Eigen::Vector3d& measured_velocity,
    const double measured_yaw_rad,
    const RouteYawConfig& config,
    const std::optional<Eigen::Vector3d>& mission_start_position_enu) noexcept {
  if (!route.valid() || !config.valid() || !measured_position.allFinite() ||
      !measured_velocity.allFinite() || !std::isfinite(measured_yaw_rad)) {
    return holdReference(measured_yaw_rad, 0.0, measured_position,
                         RouteYawSource::kInvalidRoute);
  }
  const double progress = projectedProgressArc(route, measured_position);
  const auto progress_point = route.pointAtArc(progress);
  if (!progress_point.has_value()) {
    return holdReference(measured_yaw_rad, progress, measured_position,
                         RouteYawSource::kInvalidRoute);
  }

  // The active waypoint is the current mission-owned target.  Waypoint zero
  // is the fixed start pose, so the first executable leg is zero -> one;
  // after activation, each subsequent leg is (active - 1) -> active.  This
  // makes a waypoint transition update the heading target immediately and
  // never consumes a heading from a queued future waypoint or a lookahead
  // point from the previous leg.
  const std::size_t target_index = route.active_waypoint_index;
  if (target_index >= route.waypoints.size()) {
    return holdReference(measured_yaw_rad, progress, *progress_point,
                         RouteYawSource::kHoldNoHorizontalSupport);
  }
  Eigen::Vector3d direction_origin = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  if (target_index == 0U) {
    if (!mission_start_position_enu.has_value() ||
        !mission_start_position_enu->allFinite()) {
      return holdReference(measured_yaw_rad, progress,
                           route.waypoints[target_index].position_enu,
                           RouteYawSource::kHoldNoHorizontalSupport);
    }
    direction_origin = *mission_start_position_enu;
  } else {
    direction_origin = route.waypoints[target_index - 1U].position_enu;
  }
  const Eigen::Vector3d& target = route.waypoints[target_index].position_enu;
  if (!direction_origin.allFinite() || !target.allFinite()) {
    return holdReference(measured_yaw_rad, progress, *progress_point,
                         RouteYawSource::kInvalidRoute);
  }
  const Eigen::Vector2d direction =
      (target.head<2>() - direction_origin.head<2>()).eval();
  if (!direction.allFinite() ||
      direction.norm() < config.minimum_horizontal_support_m) {
    return holdReference(measured_yaw_rad, progress, target,
                         RouteYawSource::kHoldNoHorizontalSupport);
  }

  RouteYawReference output;
  output.valid = true;
  output.target_yaw_rad = unwrapNear(
      measured_yaw_rad, std::atan2(direction.y(), direction.x()));
  if (!std::isfinite(output.target_yaw_rad)) {
    return holdReference(measured_yaw_rad, progress, target,
                         RouteYawSource::kInvalidRoute);
  }
  output.lookahead_m = direction.norm();
  output.progress_arc_m = progress;
  output.target_point = target;
  output.source = RouteYawSource::kRouteLookahead;
  return output;
}

BoundedHeadingStep stepBoundedHeading(
    const double current_yaw_rad, const double current_yaw_rate_rad_s,
    const double target_yaw_rad, const double command_period_s,
    const double maximum_yaw_rate_rad_s,
    const double maximum_yaw_acceleration_rad_s2) noexcept {
  BoundedHeadingStep output;
  if (!std::isfinite(current_yaw_rad) ||
      !std::isfinite(current_yaw_rate_rad_s) ||
      !std::isfinite(target_yaw_rad) ||
      !std::isfinite(command_period_s) || command_period_s <= 0.0 ||
      !std::isfinite(maximum_yaw_rate_rad_s) || maximum_yaw_rate_rad_s <= 0.0 ||
      !std::isfinite(maximum_yaw_acceleration_rad_s2) ||
      maximum_yaw_acceleration_rad_s2 <= 0.0 ||
      std::abs(current_yaw_rate_rad_s) > maximum_yaw_rate_rad_s + 1.0e-9) {
    return output;
  }

  const double delta = std::remainder(
      target_yaw_rad - current_yaw_rad, 2.0 * M_PI);
  const double maximum_rate_change =
      maximum_yaw_acceleration_rad_s2 * command_period_s;
  // Once the target lies inside the one-step braking distance, finish the
  // step at the target and explicitly zero the rate.  This is bounded by the
  // same acceleration limit and avoids a tiny-target sign flip/oscillation in
  // the discrete command loop.
  const double one_step_braking_distance =
      std::abs(current_yaw_rate_rad_s) * command_period_s +
      0.5 * maximum_yaw_acceleration_rad_s2 * command_period_s *
          command_period_s;
  const bool rate_points_toward_target =
      std::abs(current_yaw_rate_rad_s) <= 1.0e-12 ||
      current_yaw_rate_rad_s * delta >= 0.0;
  if (rate_points_toward_target &&
      std::abs(current_yaw_rate_rad_s) <= maximum_rate_change + 1.0e-12 &&
      std::abs(delta) <= one_step_braking_distance + 1.0e-12) {
    output.valid = true;
    output.yaw_rad = current_yaw_rad + delta;
    output.yaw_rate_rad_s = 0.0;
    output.yaw_acceleration_rad_s2 =
        -current_yaw_rate_rad_s / command_period_s;
    return output;
  }
  const double stopping_limited_rate = std::sqrt(std::max(
      0.0, 2.0 * maximum_yaw_acceleration_rad_s2 * std::abs(delta)));
  const double desired_rate =
      std::copysign(std::min(maximum_yaw_rate_rad_s, stopping_limited_rate),
                    delta);
  const double rate_change = std::clamp(
      desired_rate - current_yaw_rate_rad_s,
      -maximum_rate_change, maximum_rate_change);
  const double next_rate = std::clamp(
      current_yaw_rate_rad_s + rate_change,
      -maximum_yaw_rate_rad_s, maximum_yaw_rate_rad_s);

  output.valid = true;
  output.yaw_rate_rad_s = next_rate;
  output.yaw_acceleration_rad_s2 = rate_change / command_period_s;
  output.yaw_rad = current_yaw_rad +
      0.5 * (current_yaw_rate_rad_s + next_rate) * command_period_s;
  return output;
}

}  // namespace navigation_planning_backend
