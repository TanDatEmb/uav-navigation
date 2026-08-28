#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <planner_core/corridor_plane_validation.hpp>
#include <data_structure/base/trajectory.h>
#include <utils/header/type_utils.hpp>

namespace navigation_planning_backend {

enum class CorridorBezierSeedFailureStage {
  kNone = 0,
  kInput = 1,
  kJunction = 2,
  kBoundaryControl = 3,
  kInternalVelocity = 4,
  kCoefficient = 5,
};

struct CorridorBezierSeedResult {
  bool valid{false};
  CorridorBezierSeedFailureStage failure_stage{
      CorridorBezierSeedFailureStage::kInput};
  double minimum_internal_velocity_scale{0.0};
  geometry_utils::Trajectory trajectory;
};

namespace corridor_bezier_detail {

constexpr int kDegree = 7;

inline bool pointInsideNormalized(
    const navigation_math::PolyhedronH& planes,
    const Eigen::Vector3d& point,
    const double tolerance_m) {
  if (!point.allFinite() || planes.rows() == 0 || planes.cols() != 4) {
    return false;
  }
  const Eigen::VectorXd values =
      planes.leftCols(3) * point + planes.col(3);
  return values.allFinite() && values.maxCoeff() <= tolerance_m;
}

inline bool pointInside(
    const navigation_math::PolyhedronH& source_planes,
    const Eigen::Vector3d& point,
    const double tolerance_m) {
  auto planes = source_planes;
  return normalizeCorridorPlanes(planes) &&
      pointInsideNormalized(planes, point, tolerance_m);
}

inline std::array<Eigen::Vector3d, kDegree + 1> controlPoints(
    const navigation_math::StatePVAJ& start,
    const navigation_math::StatePVAJ& end,
    const double duration_s) noexcept {
  std::array<Eigen::Vector3d, kDegree + 1> controls;
  const double n = static_cast<double>(kDegree);
  const double acceleration_denominator = n * (n - 1.0);
  const double jerk_denominator = acceleration_denominator * (n - 2.0);
  const double duration_squared = duration_s * duration_s;
  const double duration_cubed = duration_squared * duration_s;
  controls[0] = start.col(0);
  controls[1] = start.col(0) + start.col(1) * duration_s / n;
  controls[2] = start.col(0) + 2.0 * start.col(1) * duration_s / n +
      start.col(2) * duration_squared / acceleration_denominator;
  controls[3] = start.col(0) + 3.0 * start.col(1) * duration_s / n +
      3.0 * start.col(2) * duration_squared / acceleration_denominator +
      start.col(3) * duration_cubed / jerk_denominator;
  controls[7] = end.col(0);
  controls[6] = end.col(0) - end.col(1) * duration_s / n;
  controls[5] = end.col(0) - 2.0 * end.col(1) * duration_s / n +
      end.col(2) * duration_squared / acceleration_denominator;
  controls[4] = end.col(0) - 3.0 * end.col(1) * duration_s / n +
      3.0 * end.col(2) * duration_squared / acceleration_denominator -
      end.col(3) * duration_cubed / jerk_denominator;
  return controls;
}

inline bool controlsInside(
    const std::array<Eigen::Vector3d, kDegree + 1>& controls,
    const navigation_math::PolyhedronH& planes,
    const double tolerance_m) {
  return std::all_of(
      controls.begin(), controls.end(),
      [&planes, tolerance_m](const Eigen::Vector3d& point) {
        return pointInsideNormalized(planes, point, tolerance_m);
      });
}

inline Eigen::MatrixXd powerCoefficients(
    const std::array<Eigen::Vector3d, kDegree + 1>& controls,
    const double duration_s) {
  Eigen::MatrixXd coefficients(3, kDegree + 1);
  coefficients.setZero();
  constexpr std::array<double, kDegree + 1> choose_degree{
      1.0, 7.0, 21.0, 35.0, 35.0, 21.0, 7.0, 1.0};
  constexpr std::array<std::array<double, kDegree + 1>, kDegree + 1> choose{
      std::array<double, 8>{1, 0, 0, 0, 0, 0, 0, 0},
      std::array<double, 8>{1, 1, 0, 0, 0, 0, 0, 0},
      std::array<double, 8>{1, 2, 1, 0, 0, 0, 0, 0},
      std::array<double, 8>{1, 3, 3, 1, 0, 0, 0, 0},
      std::array<double, 8>{1, 4, 6, 4, 1, 0, 0, 0},
      std::array<double, 8>{1, 5, 10, 10, 5, 1, 0, 0},
      std::array<double, 8>{1, 6, 15, 20, 15, 6, 1, 0},
      std::array<double, 8>{1, 7, 21, 35, 35, 21, 7, 1}};
  double duration_power = 1.0;
  for (int power = 0; power <= kDegree; ++power) {
    Eigen::Vector3d coefficient = Eigen::Vector3d::Zero();
    for (int control = 0; control <= power; ++control) {
      const double sign = ((power - control) % 2 == 0) ? 1.0 : -1.0;
      coefficient += sign * choose_degree[power] * choose[power][control] *
          controls[control];
    }
    coefficients.col(kDegree - power) = coefficient / duration_power;
    duration_power *= duration_s;
  }
  return coefficients;
}

}  // namespace corridor_bezier_detail

// Build a C3 piecewise degree-seven baseline whose Bernstein control points
// are all inside each piece's assigned convex corridor. Internal velocity is
// reduced deterministically until both adjacent pieces retain convex-hull
// containment. Endpoint PVAJ is immutable and never scaled.
inline CorridorBezierSeedResult buildCorridorContainedBezierSeed(
    const navigation_math::StatePVAJ& head_state,
    const navigation_math::StatePVAJ& tail_state,
    const navigation_math::Mat3Df& junction_positions,
    const navigation_math::VecDf& durations_s,
    const navigation_math::PolyhedraH& corridor_planes,
    const navigation_math::VecDi& piece_to_corridor,
    const double desired_internal_speed_mps,
    const double corridor_tolerance_m) {
  CorridorBezierSeedResult output;
  const int piece_count = static_cast<int>(durations_s.size());
  if (piece_count <= 0 || junction_positions.rows() != 3 ||
      junction_positions.cols() != piece_count - 1 ||
      piece_to_corridor.size() != piece_count || corridor_planes.empty() ||
      !head_state.allFinite() || !tail_state.allFinite() ||
      !junction_positions.allFinite() || !durations_s.allFinite() ||
      durations_s.minCoeff() <= 0.0 ||
      !std::isfinite(desired_internal_speed_mps) ||
      desired_internal_speed_mps < 0.0 ||
      !std::isfinite(corridor_tolerance_m) || corridor_tolerance_m < 0.0) {
    return output;
  }
  navigation_math::PolyhedraH normalized_corridors = corridor_planes;
  if (!std::all_of(
          normalized_corridors.begin(), normalized_corridors.end(),
          [](navigation_math::PolyhedronH& planes) {
            return normalizeCorridorPlanes(planes);
          })) {
    return output;
  }

  std::vector<navigation_math::StatePVAJ> states(
      static_cast<std::size_t>(piece_count + 1),
      navigation_math::StatePVAJ::Zero());
  states.front() = head_state;
  states.back() = tail_state;
  for (int junction = 1; junction < piece_count; ++junction) {
    states[static_cast<std::size_t>(junction)].col(0) =
        junction_positions.col(junction - 1);
  }
  for (int piece = 0; piece < piece_count; ++piece) {
    const int corridor = piece_to_corridor(piece);
    if (corridor < 0 || corridor >= static_cast<int>(corridor_planes.size()) ||
        !corridor_bezier_detail::pointInsideNormalized(
            normalized_corridors[static_cast<std::size_t>(corridor)],
            states[static_cast<std::size_t>(piece)].col(0),
            corridor_tolerance_m) ||
        !corridor_bezier_detail::pointInsideNormalized(
            normalized_corridors[static_cast<std::size_t>(corridor)],
            states[static_cast<std::size_t>(piece + 1)].col(0),
            corridor_tolerance_m)) {
      output.failure_stage = CorridorBezierSeedFailureStage::kJunction;
      return output;
    }
  }

  output.minimum_internal_velocity_scale = 1.0;
  constexpr std::array<double, 8> velocity_scales{
      1.0, 0.75, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.0};
  for (int junction = 1; junction < piece_count; ++junction) {
    const Eigen::Vector3d chord =
        states[static_cast<std::size_t>(junction + 1)].col(0) -
        states[static_cast<std::size_t>(junction - 1)].col(0);
    const double chord_norm = chord.norm();
    Eigen::Vector3d desired_velocity = Eigen::Vector3d::Zero();
    if (chord_norm > 1.0e-9) {
      desired_velocity = chord * (desired_internal_speed_mps / chord_norm);
    }
    bool found = false;
    for (const double scale : velocity_scales) {
      states[static_cast<std::size_t>(junction)].col(1) =
          scale * desired_velocity;
      const int previous_corridor = piece_to_corridor(junction - 1);
      const int next_corridor = piece_to_corridor(junction);
      const auto previous_controls = corridor_bezier_detail::controlPoints(
          states[static_cast<std::size_t>(junction - 1)],
          states[static_cast<std::size_t>(junction)], durations_s(junction - 1));
      const auto next_controls = corridor_bezier_detail::controlPoints(
          states[static_cast<std::size_t>(junction)],
          states[static_cast<std::size_t>(junction + 1)], durations_s(junction));
      if (corridor_bezier_detail::controlsInside(
              previous_controls,
              normalized_corridors[static_cast<std::size_t>(previous_corridor)],
              corridor_tolerance_m) &&
          corridor_bezier_detail::controlsInside(
              next_controls,
              normalized_corridors[static_cast<std::size_t>(next_corridor)],
              corridor_tolerance_m)) {
        output.minimum_internal_velocity_scale = std::min(
            output.minimum_internal_velocity_scale, scale);
        found = true;
        break;
      }
    }
    if (!found) {
      output.failure_stage = CorridorBezierSeedFailureStage::kInternalVelocity;
      return output;
    }
  }

  output.trajectory.clear();
  output.trajectory.reserve(piece_count);
  for (int piece = 0; piece < piece_count; ++piece) {
    const int corridor = piece_to_corridor(piece);
    const auto controls = corridor_bezier_detail::controlPoints(
        states[static_cast<std::size_t>(piece)],
        states[static_cast<std::size_t>(piece + 1)], durations_s(piece));
    if (!corridor_bezier_detail::controlsInside(
            controls, normalized_corridors[static_cast<std::size_t>(corridor)],
            corridor_tolerance_m)) {
      output.trajectory.clear();
      output.failure_stage = piece == 0 || piece + 1 == piece_count
          ? CorridorBezierSeedFailureStage::kBoundaryControl
          : CorridorBezierSeedFailureStage::kInternalVelocity;
      return output;
    }
    const Eigen::MatrixXd coefficients =
        corridor_bezier_detail::powerCoefficients(controls, durations_s(piece));
    if (!coefficients.allFinite()) {
      output.trajectory.clear();
      output.failure_stage = CorridorBezierSeedFailureStage::kCoefficient;
      return output;
    }
    output.trajectory.emplace_back(durations_s(piece), coefficients);
  }
  output.valid = !output.trajectory.empty();
  output.failure_stage = output.valid
      ? CorridorBezierSeedFailureStage::kNone
      : CorridorBezierSeedFailureStage::kCoefficient;
  return output;
}

}  // namespace navigation_planning_backend
