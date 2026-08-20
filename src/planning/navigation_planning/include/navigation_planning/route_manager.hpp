#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "navigation_mapping/world_model.hpp"

namespace navigation_planning {

struct PolylineRoute {
  std::vector<navigation_mapping::Vec3> points;
  std::vector<double> cumulative_arc_length_m;
  double length_m{0.0};

  [[nodiscard]] bool valid() const noexcept {
    return points.size() >= 2U &&
           cumulative_arc_length_m.size() == points.size() &&
           std::isfinite(length_m) && length_m > 0.0 &&
           cumulative_arc_length_m.front() == 0.0 &&
           std::abs(cumulative_arc_length_m.back() - length_m) <= 1e-9;
  }
};

struct RouteProjection {
  bool success{false};
  std::size_t segment_index{0U};
  double arc_length_m{0.0};
  double distance_m{std::numeric_limits<double>::infinity()};
  navigation_mapping::Vec3 position{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 tangent{navigation_mapping::Vec3::Zero()};
};

class RouteManager final {
 public:
  static std::optional<PolylineRoute> build(
      const std::vector<navigation_mapping::Vec3>& points,
      double minimum_segment_length_m = 1e-6) {
    if (!std::isfinite(minimum_segment_length_m) || minimum_segment_length_m < 0.0) {
      return std::nullopt;
    }

    PolylineRoute route;
    route.points.reserve(points.size());
    for (const auto& point : points) {
      if (!point.allFinite()) return std::nullopt;
      if (!route.points.empty() &&
          (point - route.points.back()).norm() <= minimum_segment_length_m) {
        continue;
      }
      route.points.push_back(point);
    }
    if (route.points.size() < 2U) return std::nullopt;

    route.cumulative_arc_length_m.resize(route.points.size(), 0.0);
    for (std::size_t i = 1U; i < route.points.size(); ++i) {
      const double length = (route.points[i] - route.points[i - 1U]).norm();
      if (!std::isfinite(length) || length <= minimum_segment_length_m) {
        return std::nullopt;
      }
      route.length_m += length;
      route.cumulative_arc_length_m[i] = route.length_m;
    }
    return route;
  }

  static RouteProjection project(
      const PolylineRoute& route, const navigation_mapping::Vec3& position,
      std::optional<double> previous_arc_length_m = std::nullopt,
      double tolerance_m = 1e-6) noexcept {
    RouteProjection result;
    if (!route.valid() || !position.allFinite() || !std::isfinite(tolerance_m) ||
        tolerance_m < 0.0 ||
        (previous_arc_length_m.has_value() &&
         (!std::isfinite(*previous_arc_length_m) || *previous_arc_length_m < 0.0 ||
          *previous_arc_length_m > route.length_m + tolerance_m))) {
      return result;
    }

    for (std::size_t i = 0U; i + 1U < route.points.size(); ++i) {
      const auto delta = route.points[i + 1U] - route.points[i];
      const double squared_length = delta.squaredNorm();
      if (!std::isfinite(squared_length) || squared_length <= 1e-12) continue;
      const double alpha = std::clamp(
          (position - route.points[i]).dot(delta) / squared_length, 0.0, 1.0);
      const auto projected = route.points[i] + alpha * delta;
      const double distance = (position - projected).norm();
      const double arc = route.cumulative_arc_length_m[i] + alpha * std::sqrt(squared_length);
      if (!std::isfinite(distance) || !std::isfinite(arc)) continue;

      const bool better = distance + 1e-12 < result.distance_m ||
                          (!result.success && std::abs(distance - result.distance_m) <= 1e-12);
      if (better) {
        result.success = true;
        result.segment_index = i;
        result.arc_length_m = arc;
        result.distance_m = distance;
        result.position = projected;
        result.tangent = delta.normalized();
      }
    }
    if (result.success && previous_arc_length_m.has_value() &&
        result.arc_length_m + tolerance_m < *previous_arc_length_m) {
      return RouteProjection{};
    }
    return result;
  }

  static std::optional<navigation_mapping::Vec3> sample(
      const PolylineRoute& route, double arc_length_m) noexcept {
    if (!route.valid() || !std::isfinite(arc_length_m)) return std::nullopt;
    const double arc = std::clamp(arc_length_m, 0.0, route.length_m);
    const auto upper = std::upper_bound(route.cumulative_arc_length_m.begin(),
                                        route.cumulative_arc_length_m.end(), arc);
    const std::size_t index = upper == route.cumulative_arc_length_m.begin()
                                  ? 0U
                                  : std::min<std::size_t>(
                                        static_cast<std::size_t>(std::distance(
                                            route.cumulative_arc_length_m.begin(), upper)) - 1U,
                                        route.points.size() - 2U);
    const double start = route.cumulative_arc_length_m[index];
    const double segment_length = route.cumulative_arc_length_m[index + 1U] - start;
    const double alpha = segment_length > 0.0 ? (arc - start) / segment_length : 0.0;
    return route.points[index] + std::clamp(alpha, 0.0, 1.0) *
                                   (route.points[index + 1U] - route.points[index]);
  }

  static std::optional<PolylineRoute> trimConsumedPrefix(
      const PolylineRoute& route, double consumed_arc_length_m) {
    if (!route.valid() || !std::isfinite(consumed_arc_length_m) ||
        consumed_arc_length_m < 0.0 || consumed_arc_length_m >= route.length_m) {
      return std::nullopt;
    }

    const auto start = sample(route, consumed_arc_length_m);
    if (!start.has_value()) return std::nullopt;
    std::vector<navigation_mapping::Vec3> points{*start};
    for (std::size_t i = 1U; i < route.points.size(); ++i) {
      if (route.cumulative_arc_length_m[i] > consumed_arc_length_m) {
        points.push_back(route.points[i]);
      }
    }
    return build(points);
  }
};

}  // namespace navigation_planning
