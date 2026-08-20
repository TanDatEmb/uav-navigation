#include "navigation_planning/horizon_policy.hpp"

#include <algorithm>
#include <cmath>

namespace navigation_planning {
namespace {

constexpr double kEpsilon = 1e-9;

bool finiteNonNegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool validConfig(const HorizonPolicyConfig& config) {
  return finiteNonNegative(config.minimum_distance_m) &&
         finiteNonNegative(config.maximum_distance_m) &&
         config.maximum_distance_m >= config.minimum_distance_m &&
         finiteNonNegative(config.preview_time_s) &&
         finiteNonNegative(config.stopping_distance_factor) &&
         finiteNonNegative(config.map_boundary_margin_m) &&
         finiteNonNegative(config.projection_tolerance_m);
}

PlanningHorizon failure(HorizonFailureCode code, double start_arc) {
  PlanningHorizon result;
  result.failure = code;
  result.start_arc_length_m = start_arc;
  result.endpoint_arc_length_m = start_arc;
  return result;
}

}  // namespace

HorizonPolicy::HorizonPolicy(HorizonPolicyConfig config) : config_(config) {}

PlanningHorizon HorizonPolicy::compute(const HorizonRequest& request) const noexcept {
  const auto& route = request.route;
  const double start_arc = route.projected_arc_length_m;
  if (!validConfig(config_) || !std::isfinite(start_arc) ||
      !std::isfinite(route.route_length_m) || route.route_length_m < 0.0 ||
      start_arc < -config_.projection_tolerance_m ||
      start_arc > route.route_length_m + config_.projection_tolerance_m ||
      !finiteNonNegative(request.speed_mps) ||
      !std::isfinite(request.max_deceleration_mps2) ||
      request.max_deceleration_mps2 <= 0.0 ||
      !finiteNonNegative(request.pipeline_latency_s) ||
      !finiteNonNegative(request.stop_margin_m) ||
      (std::isinf(route.usable_forward_distance_m) &&
       route.usable_forward_distance_m < 0.0) ||
      (!std::isinf(route.usable_forward_distance_m) &&
       !finiteNonNegative(route.usable_forward_distance_m))) {
    return failure(HorizonFailureCode::InvalidInput, start_arc);
  }

  if (route.previous_projected_arc_length_m.has_value()) {
    const double previous = *route.previous_projected_arc_length_m;
    if (!std::isfinite(previous) || previous < 0.0 ||
        previous > route.route_length_m + config_.projection_tolerance_m) {
      return failure(HorizonFailureCode::InvalidInput, start_arc);
    }
    if (start_arc + config_.projection_tolerance_m < previous) {
      return failure(HorizonFailureCode::BackwardProjection, start_arc);
    }
  }

  if (request.terminal_waypoint_arc_length_m.has_value()) {
    const double terminal = *request.terminal_waypoint_arc_length_m;
    if (!std::isfinite(terminal) || terminal < 0.0 ||
        terminal > route.route_length_m + config_.projection_tolerance_m) {
      return failure(HorizonFailureCode::InvalidInput, start_arc);
    }
    if (terminal + config_.projection_tolerance_m < start_arc) {
      return failure(HorizonFailureCode::TerminalBehindProjection, start_arc);
    }
  }

  const double stop_distance =
      request.speed_mps * request.speed_mps / (2.0 * request.max_deceleration_mps2) +
      request.speed_mps * request.pipeline_latency_s + request.stop_margin_m;
  const double preview_distance = request.speed_mps * config_.preview_time_s;
  const double unconstrained = std::clamp(
      std::max({config_.minimum_distance_m, preview_distance,
                config_.stopping_distance_factor * stop_distance}),
      config_.minimum_distance_m, config_.maximum_distance_m);

  double usable = std::max(0.0, route.route_length_m - start_arc);
  if (std::isfinite(route.usable_forward_distance_m)) {
    usable = std::min(usable,
                      route.usable_forward_distance_m - config_.map_boundary_margin_m);
    usable = std::max(0.0, usable);
  }

  PlanningHorizon result;
  result.start_arc_length_m = start_arc;
  result.unconstrained_distance_m = unconstrained;
  result.usable_forward_distance_m = usable;

  const bool terminal_usable =
      request.terminal_waypoint_arc_length_m.has_value() &&
      *request.terminal_waypoint_arc_length_m - start_arc <=
          usable + config_.projection_tolerance_m;
  if (terminal_usable) {
    const double distance = std::max(
        0.0, *request.terminal_waypoint_arc_length_m - start_arc);
    result.forward_distance_m = std::min(unconstrained, distance);
    result.terminal_waypoint_clamped =
        distance <= unconstrained + config_.projection_tolerance_m;
  } else {
    result.forward_distance_m = std::min(unconstrained, usable);
  }

  if (result.forward_distance_m <= kEpsilon && !terminal_usable) {
    return failure(HorizonFailureCode::NoUsableForwardDistance, start_arc);
  }
  result.endpoint_arc_length_m = start_arc + result.forward_distance_m;
  result.shortened_by_usable_space =
      result.forward_distance_m + config_.projection_tolerance_m < unconstrained &&
      !result.terminal_waypoint_clamped;
  result.success = true;
  return result;
}

}  // namespace navigation_planning
