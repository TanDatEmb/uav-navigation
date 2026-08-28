#include "planner_core/route_yaw_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace navigation_planning_backend {
namespace {

constexpr double kGeometryTieM2 = 1.0e-12;

double unwrapNear(const double anchor, const double target) noexcept {
  if (!std::isfinite(anchor) || !std::isfinite(target)) return anchor;
  return anchor + std::remainder(target - anchor, 2.0 * M_PI);
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

bool activeWaypointIsReversal(
    const navigation_mission::ImmutableRouteSnapshot& route,
    const double threshold_rad) noexcept {
  const std::size_t active = route.active_waypoint_index;
  const navigation_mission::RouteSegment* incoming = nullptr;
  const navigation_mission::RouteSegment* outgoing = nullptr;
  for (const auto& segment : route.segments) {
    if (segment.end_waypoint_index == active) incoming = &segment;
    if (segment.start_waypoint_index == active) {
      outgoing = &segment;
      break;
    }
  }
  if (incoming == nullptr || outgoing == nullptr) return false;
  const double incoming_horizontal_norm = incoming->tangent.head<2>().norm();
  const double outgoing_horizontal_norm = outgoing->tangent.head<2>().norm();
  if (incoming_horizontal_norm < 1.0e-6 || outgoing_horizontal_norm < 1.0e-6) {
    return false;
  }
  const double cosine = std::clamp(
      incoming->tangent.head<2>().dot(outgoing->tangent.head<2>()) /
          (incoming_horizontal_norm * outgoing_horizontal_norm),
      -1.0, 1.0);
  return std::acos(cosine) >= threshold_rad;
}

bool previousWaypointWasReversal(
    const navigation_mission::ImmutableRouteSnapshot& route,
    const double threshold_rad) noexcept {
  if (route.active_waypoint_index == 0U) return false;
  auto previous = route;
  previous.active_waypoint_index = route.active_waypoint_index - 1U;
  return activeWaypointIsReversal(previous, threshold_rad);
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
    const RouteYawConfig& config) noexcept {
  if (!route.valid() || !config.valid() || !measured_position.allFinite() ||
      !measured_velocity.allFinite() || !std::isfinite(measured_yaw_rad)) {
    return holdReference(measured_yaw_rad, 0.0, measured_position,
                         RouteYawSource::kInvalidRoute);
  }
  const double horizontal_speed = measured_velocity.head<2>().norm();
  const double progress = projectedProgressArc(route, measured_position);
  const auto progress_point = route.pointAtArc(progress);
  if (!progress_point.has_value()) {
    return holdReference(measured_yaw_rad, progress, measured_position,
                         RouteYawSource::kInvalidRoute);
  }
  const bool reversal_turn_in_place =
      previousWaypointWasReversal(route, config.reversal_threshold_rad);
  if ((!std::isfinite(horizontal_speed) ||
       horizontal_speed < config.minimum_horizontal_speed_mps) &&
      !reversal_turn_in_place) {
    return holdReference(measured_yaw_rad, progress, *progress_point,
                         RouteYawSource::kHoldLowSpeed);
  }

  const double lookahead = std::clamp(
      std::max(config.minimum_lookahead_m,
               horizontal_speed * config.lookahead_time_s),
      config.minimum_lookahead_m, config.maximum_lookahead_m);
  double target_arc = std::min(route.total_length_m, progress + lookahead);
  if (activeWaypointIsReversal(route, config.reversal_threshold_rad)) {
    target_arc = std::min(
        target_arc, route.waypoint_arc_lengths_m[route.active_waypoint_index]);
  }
  const auto target_point = route.pointAtArc(target_arc);
  if (!target_point.has_value()) {
    return holdReference(measured_yaw_rad, progress, *progress_point,
                         RouteYawSource::kInvalidRoute);
  }
  // Heading is a property of the mission route, not of the cross-track
  // correction.  Using measured_position -> target_point makes yaw point back
  // at the waypoint after a small overshoot, producing a near-180 degree turn
  // while the position controller performs a short terminal correction.
  // Follow the route chord instead.  At the terminal arc there is no forward
  // chord, so use the incoming chord and preserve the final-leg heading.
  Eigen::Vector3d direction_origin = *progress_point;
  if (target_arc <= progress + config.minimum_horizontal_support_m) {
    const auto trailing_point = route.pointAtArc(
        std::max(0.0, target_arc - lookahead));
    if (trailing_point.has_value()) direction_origin = *trailing_point;
  }
  const Eigen::Vector2d direction =
      (target_point->head<2>() - direction_origin.head<2>()).eval();
  if (!direction.allFinite() ||
      direction.norm() < config.minimum_horizontal_support_m) {
    return holdReference(measured_yaw_rad, progress, *target_point,
                         RouteYawSource::kHoldNoHorizontalSupport);
  }

  RouteYawReference output;
  output.valid = true;
  output.target_yaw_rad = unwrapNear(
      measured_yaw_rad, std::atan2(direction.y(), direction.x()));
  output.lookahead_m = lookahead;
  output.progress_arc_m = progress;
  output.target_point = *target_point;
  output.source = reversal_turn_in_place &&
          horizontal_speed < config.minimum_horizontal_speed_mps
      ? RouteYawSource::kRouteTurnInPlace
      : RouteYawSource::kRouteLookahead;
  return output;
}

}  // namespace navigation_planning_backend
