#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <Eigen/Core>

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_planning {

enum class RouteBoundaryEventKind : std::uint8_t {
  kPassThrough,
  kTerminalStop,
};

// A route event is an executable boundary, not a diagnostic waypoint label.
// Its timestamp is producer-declared and is consumed with the same integer
// wall-clock contract as CandidateBundle endpoints.
struct RouteBoundaryEvent final {
  RouteBoundaryEventKind kind{RouteBoundaryEventKind::kPassThrough};
  std::size_t junction_index{0U};
  std::int64_t boundary_stamp_ns{0};
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d incoming_tangent{Eigen::Vector3d::Zero()};
  Eigen::Vector3d outgoing_tangent{Eigen::Vector3d::Zero()};
  double corner_speed_mps{0.0};

  [[nodiscard]] bool valid() const noexcept {
    const double incoming_norm = incoming_tangent.norm();
    const double outgoing_norm = outgoing_tangent.norm();
    return boundary_stamp_ns > 0 && position_world.allFinite() &&
           incoming_tangent.allFinite() && outgoing_tangent.allFinite() &&
           std::isfinite(incoming_norm) && incoming_norm > 1.0e-9 &&
           std::isfinite(outgoing_norm) && outgoing_norm > 1.0e-9 &&
           std::abs(incoming_norm - 1.0) <= 1.0e-6 &&
           std::abs(outgoing_norm - 1.0) <= 1.0e-6 &&
           std::isfinite(corner_speed_mps) && corner_speed_mps >= 0.0;
  }
};

// The admissible volume is deliberately explicit. A pass-through may not be
// represented by an endpoint replay or by a soft radius check performed only
// in mission code; the producer must provide the volume and the junction
// tangents used to construct and certify the crossing.
struct RouteBoundaryConstraint final {
  navigation_world_model::AxisAlignedBox admissible_volume{};
  std::size_t junction_index{0U};
  Eigen::Vector3d incoming_tangent{Eigen::Vector3d::Zero()};
  Eigen::Vector3d outgoing_tangent{Eigen::Vector3d::Zero()};
  double corner_speed_mps{0.0};

  [[nodiscard]] bool contains(const Eigen::Vector3d& point) const noexcept {
    return valid() && point.allFinite() &&
           (point.array() >= admissible_volume.minimum.array()).all() &&
           (point.array() <= admissible_volume.maximum.array()).all();
  }

  [[nodiscard]] bool valid() const noexcept {
    const double incoming_norm = incoming_tangent.norm();
    const double outgoing_norm = outgoing_tangent.norm();
    return admissible_volume.valid() && incoming_tangent.allFinite() &&
           outgoing_tangent.allFinite() && std::isfinite(incoming_norm) &&
           std::isfinite(outgoing_norm) && incoming_norm > 1.0e-9 &&
           outgoing_norm > 1.0e-9 && std::abs(incoming_norm - 1.0) <= 1.0e-6 &&
           std::abs(outgoing_norm - 1.0) <= 1.0e-6 &&
           std::isfinite(corner_speed_mps) && corner_speed_mps >= 0.0;
  }
};

}  // namespace navigation_planning
