#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "navigation_mapping/world_model.hpp"

namespace navigation_planning {

struct BsplineSample {
  navigation_mapping::Vec3 position{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 velocity{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 acceleration{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 jerk{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 snap{navigation_mapping::Vec3::Zero()};
};

struct BsplineTrajectory {
  int degree{5};
  double knot_dt_s{0.2};
  std::vector<navigation_mapping::Vec3> control_points;

  [[nodiscard]] bool valid() const noexcept {
    return degree >= 1 && std::isfinite(knot_dt_s) && knot_dt_s > 0.0 &&
           control_points.size() >= static_cast<std::size_t>(degree + 1) &&
           std::all_of(control_points.begin(), control_points.end(),
                       [](const auto& point) { return point.allFinite(); });
  }

  [[nodiscard]] std::size_t spanCount() const noexcept {
    if (!valid()) return 0U;
    return control_points.size() - static_cast<std::size_t>(degree);
  }

  [[nodiscard]] double duration() const noexcept {
    return static_cast<double>(spanCount()) * knot_dt_s;
  }

 private:
  [[nodiscard]] const std::vector<double>& knots() const {
    const std::size_t span_count = spanCount();
    if (cached_span_count == span_count && cached_degree == degree &&
        cached_knots.size() == control_points.size() + static_cast<std::size_t>(degree) + 1U) {
      return cached_knots;
    }
    cached_knots.assign(control_points.size() + static_cast<std::size_t>(degree) + 1U, 0.0);
    for (std::size_t index = static_cast<std::size_t>(degree) + 1U;
         index < span_count + static_cast<std::size_t>(degree) + 1U; ++index) {
      cached_knots[index] = static_cast<double>(index - static_cast<std::size_t>(degree));
    }
    const double last = static_cast<double>(span_count);
    for (std::size_t index = span_count + static_cast<std::size_t>(degree) + 1U;
         index < cached_knots.size(); ++index) {
      cached_knots[index] = last;
    }
    cached_span_count = span_count;
    cached_degree = degree;
    return cached_knots;
  }

  mutable std::vector<double> cached_knots;
  mutable std::size_t cached_span_count{0U};
  mutable int cached_degree{-1};

  [[nodiscard]] static double basisDerivative(std::size_t index, int order, int derivative,
                                              double parameter,
                                              const std::vector<double>& knots) {
    if (derivative == 0) {
      if (order == 0) {
        const bool in_span = knots[index] <= parameter && parameter < knots[index + 1U];
        const bool at_end = parameter == knots.back() && index + 2U == knots.size();
        return (in_span || at_end) ? 1.0 : 0.0;
      }
      const double left_denominator = knots[index + static_cast<std::size_t>(order)] - knots[index];
      const double right_denominator =
          knots[index + static_cast<std::size_t>(order) + 1U] - knots[index + 1U];
      const double left = left_denominator > 0.0
                              ? (parameter - knots[index]) / left_denominator *
                                    basisDerivative(index, order - 1, 0, parameter, knots)
                              : 0.0;
      const double right = right_denominator > 0.0
                               ? (knots[index + static_cast<std::size_t>(order) + 1U] - parameter) /
                                     right_denominator *
                                     basisDerivative(index + 1U, order - 1, 0, parameter, knots)
                               : 0.0;
      return left + right;
    }
    if (derivative > order) return 0.0;
    const double left_denominator = knots[index + static_cast<std::size_t>(order)] - knots[index];
    const double right_denominator =
        knots[index + static_cast<std::size_t>(order) + 1U] - knots[index + 1U];
    const double left = left_denominator > 0.0
                            ? static_cast<double>(order) / left_denominator *
                                  basisDerivative(index, order - 1, derivative - 1, parameter, knots)
                            : 0.0;
    const double right = right_denominator > 0.0
                             ? static_cast<double>(order) / right_denominator *
                                   basisDerivative(index + 1U, order - 1, derivative - 1, parameter,
                                                   knots)
                             : 0.0;
    return left - right;
  }

 public:
  [[nodiscard]] BsplineSample evaluate(double time_s) const {
    BsplineSample result;
    if (!valid()) return result;
    const double clamped_time = std::clamp(time_s, 0.0, duration());
    double parameter = clamped_time / knot_dt_s;
    if (parameter >= static_cast<double>(spanCount())) {
      parameter = std::nextafter(static_cast<double>(spanCount()), 0.0);
    }
    const auto knot_vector = knots();
    const auto evaluateDerivative = [&](int derivative) {
      navigation_mapping::Vec3 value = navigation_mapping::Vec3::Zero();
      const double scale = std::pow(1.0 / knot_dt_s, derivative);
      for (std::size_t index = 0; index < control_points.size(); ++index) {
        value += scale * basisDerivative(index, degree, derivative, parameter, knot_vector) *
                 control_points[index];
      }
      return value;
    };
    result.position = evaluateDerivative(0);
    result.velocity = evaluateDerivative(1);
    result.acceleration = evaluateDerivative(2);
    result.jerk = evaluateDerivative(3);
    result.snap = evaluateDerivative(4);
    if (clamped_time <= 0.0) result.position = control_points.front();
    if (clamped_time >= duration()) result.position = control_points.back();
    return result;
  }

  // Collision checks only need position. Avoid calculating four derivative
  // orders for every dense verifier sample during each rolling replan.
  [[nodiscard]] navigation_mapping::Vec3 evaluatePosition(double time_s) const {
    if (!valid()) return navigation_mapping::Vec3::Zero();
    const double clamped_time = std::clamp(time_s, 0.0, duration());
    double parameter = clamped_time / knot_dt_s;
    if (parameter >= static_cast<double>(spanCount())) {
      parameter = std::nextafter(static_cast<double>(spanCount()), 0.0);
    }
    const auto& knot_vector = knots();
    navigation_mapping::Vec3 value = navigation_mapping::Vec3::Zero();
    for (std::size_t index = 0; index < control_points.size(); ++index) {
      value += basisDerivative(index, degree, 0, parameter, knot_vector) * control_points[index];
    }
    if (clamped_time <= 0.0) return control_points.front();
    if (clamped_time >= duration()) return control_points.back();
    return value;
  }
};

struct BsplineGenerationConfig {
  int degree{5};
  double knot_dt_s{0.2};
  // <= 0 selects topology-only control density. A positive value is an
  // explicit geometry fallback used when a compact spline cannot certify a
  // tight obstacle corridor.
  double control_point_spacing_m{0.0};
  double sample_dt_s{0.02};
  int smoothing_iterations{12};
  double smoothing_step{0.04};
  double snap_weight{1.0};
  double path_length_weight{0.10};
  double reference_weight{0.35};
  double max_velocity_mps{2.0};
  double max_acceleration_mps2{3.0};
  double max_deceleration_mps2{3.0};
  double max_jerk_mps3{6.0};
};

struct BsplineGenerationResult {
  bool success{false};
  BsplineTrajectory trajectory{};
  std::uint64_t sampled_point_count{0U};
  double maximum_velocity_mps{0.0};
  double maximum_acceleration_mps2{0.0};
  double maximum_deceleration_mps2{0.0};
  double maximum_jerk_mps3{0.0};
  double integrated_squared_jerk{0.0};
  double integrated_squared_snap{0.0};
  double objective_cost{0.0};
  double geometric_length_m{0.0};
};

namespace detail {

inline navigation_mapping::Vec3 interpolatePolyline(
    const std::vector<navigation_mapping::Vec3>& waypoints, double arc_length) {
  if (waypoints.empty()) return navigation_mapping::Vec3::Zero();
  if (waypoints.size() == 1U) return waypoints.front();
  double remaining = std::max(0.0, arc_length);
  for (std::size_t index = 1; index < waypoints.size(); ++index) {
    const auto delta = waypoints[index] - waypoints[index - 1U];
    const double length = delta.norm();
    if (length <= 1e-9) continue;
    if (remaining <= length || index + 1U == waypoints.size()) {
      return waypoints[index - 1U] + std::clamp(remaining / length, 0.0, 1.0) * delta;
    }
    remaining -= length;
  }
  return waypoints.back();
}

inline std::vector<navigation_mapping::Vec3> samplePolylinePreservingWaypoints(
    const std::vector<navigation_mapping::Vec3>& waypoints, std::size_t control_count) {
  if (waypoints.size() < 2U || control_count < 2U) return {};
  const std::size_t segment_count = waypoints.size() - 1U;
  if (control_count < segment_count + 1U) return {};

  std::vector<double> segment_lengths(segment_count, 0.0);
  std::vector<std::size_t> intervals(segment_count, 1U);
  for (std::size_t segment = 0; segment < segment_count; ++segment) {
    segment_lengths[segment] = (waypoints[segment + 1U] - waypoints[segment]).norm();
  }

  // Boundary-condition enforcement rewrites the first two and last two
  // control points. Reserve three intervals at each end when there is an
  // internal corner, otherwise that corner can be selected correctly and then
  // silently erased by the endpoint derivative constraints.
  const std::size_t total_intervals = control_count - 1U;
  const bool has_internal_corner = segment_count >= 2U &&
                                   total_intervals >= segment_count + 4U;
  if (has_internal_corner) {
    intervals.front() = 3U;
    intervals.back() = 3U;
  }

  // Every reference segment gets at least one knot interval. The remaining
  // degree-dependent controls are allocated to the currently longest
  // interval, while the protected boundary allocation keeps the first/last
  // corners available after endpoint conditions are imposed.
  std::size_t allocated_intervals = 0U;
  for (const auto count : intervals) allocated_intervals += count;
  std::size_t remaining_intervals = total_intervals - allocated_intervals;
  while (remaining_intervals > 0U) {
    std::size_t selected = 0U;
    double selected_spacing = -1.0;
    for (std::size_t segment = 0; segment < segment_count; ++segment) {
      if (has_internal_corner && segment == segment_count - 1U) continue;
      const double spacing = segment_lengths[segment] /
                             static_cast<double>(intervals[segment]);
      if (spacing > selected_spacing) {
        selected_spacing = spacing;
        selected = segment;
      }
    }
    ++intervals[selected];
    --remaining_intervals;
  }

  std::vector<navigation_mapping::Vec3> controls;
  controls.reserve(control_count);
  controls.push_back(waypoints.front());
  for (std::size_t segment = 0; segment < segment_count; ++segment) {
    const auto delta = waypoints[segment + 1U] - waypoints[segment];
    for (std::size_t interval = 1; interval <= intervals[segment]; ++interval) {
      controls.push_back(waypoints[segment] +
                         (static_cast<double>(interval) /
                          static_cast<double>(intervals[segment])) * delta);
    }
  }
  return controls;
}

inline double polylineLength(const std::vector<navigation_mapping::Vec3>& waypoints) {
  double length = 0.0;
  for (std::size_t index = 1; index < waypoints.size(); ++index) {
    length += (waypoints[index] - waypoints[index - 1U]).norm();
  }
  return length;
}

inline bool finiteLimits(const BsplineGenerationConfig& config) {
  return config.degree >= 1 && std::isfinite(config.knot_dt_s) && config.knot_dt_s > 0.0 &&
         std::isfinite(config.control_point_spacing_m) && config.control_point_spacing_m >= 0.0 &&
         std::isfinite(config.sample_dt_s) && config.sample_dt_s > 0.0 &&
         config.smoothing_iterations >= 0 && std::isfinite(config.smoothing_step) &&
         config.smoothing_step >= 0.0 && std::isfinite(config.reference_weight) &&
         config.reference_weight >= 0.0 && std::isfinite(config.snap_weight) &&
         config.snap_weight >= 0.0 && std::isfinite(config.path_length_weight) &&
         config.path_length_weight >= 0.0 && std::isfinite(config.max_velocity_mps) &&
         config.max_velocity_mps > 0.0 && std::isfinite(config.max_acceleration_mps2) &&
         config.max_acceleration_mps2 > 0.0 && std::isfinite(config.max_deceleration_mps2) &&
         config.max_deceleration_mps2 > 0.0 && std::isfinite(config.max_jerk_mps3) &&
         config.max_jerk_mps3 > 0.0;
}

inline void enforceBoundaryConditions(BsplineTrajectory& trajectory,
                                       const navigation_mapping::Vec3& start_velocity,
                                       const navigation_mapping::Vec3& start_acceleration,
                                       const navigation_mapping::Vec3& end_velocity,
                                       const navigation_mapping::Vec3& end_acceleration) {
  const int degree = trajectory.degree;
  const double dt = trajectory.knot_dt_s;
  const double degree_scale = static_cast<double>(degree);
  const double second_scale = static_cast<double>(degree * (degree - 1));
  if (trajectory.control_points.size() < static_cast<std::size_t>(degree + 1) ||
      second_scale <= 0.0) {
    return;
  }
  auto& points = trajectory.control_points;
  points[1] = points[0] + start_velocity * dt / degree_scale;
  // The first two spans of the open-uniform degree-five basis produce
  // C''(0) = 2 p (p - 1) (P2 - 3 P1 + 2 P0) / dt². Solve that relation
  // directly so a non-zero measured acceleration is preserved at splice.
  points[2] = start_acceleration * (2.0 * dt * dt / second_scale) +
              3.0 * points[1] - 2.0 * points[0];

  const std::size_t last = points.size() - 1U;
  points[last - 1U] = points[last] - end_velocity * dt / degree_scale;
  points[last - 2U] = end_acceleration * (2.0 * dt * dt / second_scale) +
                      3.0 * points[last - 1U] - 2.0 * points[last];
}

inline void smoothControlPoints(BsplineTrajectory& trajectory,
                                const std::vector<navigation_mapping::Vec3>& reference,
                                const BsplineGenerationConfig& config,
                                const navigation_mapping::Vec3& start_velocity,
                                const navigation_mapping::Vec3& start_acceleration,
                                const navigation_mapping::Vec3& end_velocity,
                                const navigation_mapping::Vec3& end_acceleration) {
  auto& points = trajectory.control_points;
  const int degree = trajectory.degree;
  if (points.size() <= static_cast<std::size_t>(2 * degree)) return;
  for (int iteration = 0; iteration < config.smoothing_iterations; ++iteration) {
    std::vector<navigation_mapping::Vec3> next = points;
    for (std::size_t index = static_cast<std::size_t>(degree);
         index + static_cast<std::size_t>(degree) < points.size(); ++index) {
      const auto fourth_difference = points[index - 2U] - 4.0 * points[index - 1U] +
                                     6.0 * points[index] - 4.0 * points[index + 1U] +
                                     points[index + 2U];
      const auto path_length_gradient = 2.0 * points[index] - points[index - 1U] -
                                        points[index + 1U];
      navigation_mapping::Vec3 reference_error = navigation_mapping::Vec3::Zero();
      if (!reference.empty()) {
        reference_error = reference[index % reference.size()] - points[index];
      }
      next[index] = points[index] - config.smoothing_step *
                        (config.snap_weight * fourth_difference +
                         config.path_length_weight * path_length_gradient) +
                    config.smoothing_step * config.reference_weight * reference_error;
    }
    points = std::move(next);
    enforceBoundaryConditions(trajectory, start_velocity, start_acceleration, end_velocity,
                              end_acceleration);
  }
}

}  // namespace detail

inline BsplineGenerationResult generateBsplineTrajectory(
    const std::vector<navigation_mapping::Vec3>& waypoints,
    const navigation_mapping::Vec3& start_velocity,
    const navigation_mapping::Vec3& start_acceleration,
    const navigation_mapping::Vec3& end_velocity,
    const navigation_mapping::Vec3& end_acceleration,
    const BsplineGenerationConfig& config = {}) {
  BsplineGenerationResult result;
  if (waypoints.size() < 2U || !detail::finiteLimits(config) ||
      !start_velocity.allFinite() || !start_acceleration.allFinite() ||
      !end_velocity.allFinite() || !end_acceleration.allFinite() ||
      !std::all_of(waypoints.begin(), waypoints.end(),
                   [](const auto& point) { return point.allFinite(); })) {
    return result;
  }

  const double length = detail::polylineLength(waypoints);
  if (!std::isfinite(length) || length <= 1e-9) return result;
  const int degree = std::max(1, config.degree);
  // The knot count is a temporal/shape discretisation, not a collision
  // sampling density. Tying it to metres forced a 10 m straight corridor
  // into thirteen spans; the dynamic-limit scaler then stretched every span
  // and reduced the executed cruise speed to roughly 1.5 m/s despite a 3 m/s
  // contract. Preserve one span per reference segment (with a minimum of two
  // for the degree-five boundary conditions); the planner/verifier still
  // samples the complete spline densely for collision and dynamic limits.
  const int reference_span_count = std::max(
      1, static_cast<int>(waypoints.size()) - 1);
  const int geometry_span_count = config.control_point_spacing_m > 0.0
                                      ? static_cast<int>(std::ceil(
                                            length / config.control_point_spacing_m))
                                      : 0;
  const int interior_count = std::max({2, reference_span_count, geometry_span_count});
  const std::size_t control_count = static_cast<std::size_t>(interior_count + degree);
  result.trajectory.degree = degree;
  result.trajectory.knot_dt_s = config.knot_dt_s;
  result.trajectory.control_points.resize(control_count);
  // The control polygon must cover the complete reference polyline.  A
  // uniform arc-length sample can skip a short but safety-critical corner;
  // preserve every reference waypoint and distribute only the surplus
  // degree-dependent controls over the longest segments.
  result.trajectory.control_points =
      detail::samplePolylinePreservingWaypoints(waypoints, control_count);
  if (result.trajectory.control_points.size() != control_count) return result;
  result.trajectory.control_points.front() = waypoints.front();
  result.trajectory.control_points.back() = waypoints.back();

  std::vector<navigation_mapping::Vec3> reference = result.trajectory.control_points;
  detail::enforceBoundaryConditions(result.trajectory, start_velocity, start_acceleration,
                                     end_velocity, end_acceleration);
  detail::smoothControlPoints(result.trajectory, reference, config, start_velocity,
                              start_acceleration, end_velocity, end_acceleration);

  if (!result.trajectory.valid()) return result;
  const int samples = std::max(
      2, static_cast<int>(std::ceil(result.trajectory.duration() / config.sample_dt_s)));
  BsplineSample previous;
  bool has_previous = false;
  for (int sample = 0; sample <= samples; ++sample) {
    const double time = result.trajectory.duration() * static_cast<double>(sample) /
                        static_cast<double>(samples);
    const auto value = result.trajectory.evaluate(time);
    if (!value.position.allFinite() || !value.velocity.allFinite() ||
        !value.acceleration.allFinite() || !value.jerk.allFinite() || !value.snap.allFinite()) {
      return BsplineGenerationResult{};
    }
    ++result.sampled_point_count;
    result.maximum_velocity_mps = std::max(result.maximum_velocity_mps, value.velocity.norm());
    result.maximum_acceleration_mps2 =
        std::max(result.maximum_acceleration_mps2, value.acceleration.norm());
    if (value.velocity.norm() > 1e-9) {
      result.maximum_deceleration_mps2 = std::max(
          result.maximum_deceleration_mps2,
          std::max(0.0, -value.acceleration.dot(value.velocity.normalized())));
    }
    result.maximum_jerk_mps3 = std::max(result.maximum_jerk_mps3, value.jerk.norm());
    if (has_previous) {
      result.geometric_length_m += (value.position - previous.position).norm();
      const double dt = result.trajectory.duration() / static_cast<double>(samples);
      result.integrated_squared_snap += 0.5 *
          (previous.snap.squaredNorm() + value.snap.squaredNorm()) * dt;
      result.integrated_squared_jerk += 0.5 *
          (previous.jerk.squaredNorm() + value.jerk.squaredNorm()) * dt;
    }
    previous = value;
    has_previous = true;
  }
  result.objective_cost = result.integrated_squared_snap + 0.10 * result.geometric_length_m;
  result.success = result.maximum_velocity_mps <= config.max_velocity_mps * 0.995 + 1e-9 &&
                   result.maximum_acceleration_mps2 <=
                       std::max(config.max_acceleration_mps2, config.max_deceleration_mps2) *
                           0.995 + 1e-9 &&
                   result.maximum_deceleration_mps2 <= config.max_deceleration_mps2 * 0.995 +
                       1e-9 &&
                   result.maximum_jerk_mps3 <= config.max_jerk_mps3 * 0.995 + 1e-9;
  return result;
}

}  // namespace navigation_planning
