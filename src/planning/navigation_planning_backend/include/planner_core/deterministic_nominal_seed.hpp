#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <planner_core/corridor_plane_validation.hpp>
#include <navigation_planning/planning_limits.hpp>
#include <traj_opt/config.hpp>
#include <traj_opt/trajectory_dynamics.hpp>
#include <utils/header/type_utils.hpp>

namespace navigation_planning_backend {

enum class DeterministicNominalSeedFailureStage {
  kNone = 0,
  kInput = 1,
  kCorridor = 2,
  kBoundary = 3,
  kRouteBoundary = 4,
  kDynamics = 5,
  kFlatness = 6,
};

struct DeterministicNominalSeedCertificate {
  bool valid{false};
  DeterministicNominalSeedFailureStage failure_stage{
      DeterministicNominalSeedFailureStage::kInput};
  double maximum_corridor_violation_m{
      std::numeric_limits<double>::infinity()};
  double maximum_boundary_residual{
      std::numeric_limits<double>::infinity()};
  double maximum_boundary_roundoff_bound{
      std::numeric_limits<double>::infinity()};
  double maximum_velocity_mps{std::numeric_limits<double>::infinity()};
  double maximum_acceleration_mps2{std::numeric_limits<double>::infinity()};
  double maximum_jerk_mps3{std::numeric_limits<double>::infinity()};
  traj_opt::TrajectoryDynamicReport flatness_report{};
};

// Return a finite set of uniform duration multipliers for a geometrically
// certified seed that failed only its V/A/J certificate.  Uniform time scaling
// reduces derivative orders by s, s^2 and s^3 respectively when boundary
// derivatives scale with the path timing.  The rebuilt trajectory must still
// pass the complete certificate because endpoint derivatives remain immutable.
inline std::vector<double> boundedDynamicDurationRetryScales(
    const DeterministicNominalSeedCertificate& certificate,
    const traj_opt::Config& config,
    const double maximum_scale = 4.0) {
  if (certificate.failure_stage !=
          DeterministicNominalSeedFailureStage::kDynamics ||
      !std::isfinite(maximum_scale) || maximum_scale <= 1.0 ||
      !std::isfinite(config.max_vel) || config.max_vel <= 0.0 ||
      !std::isfinite(config.max_acc) || config.max_acc <= 0.0 ||
      !std::isfinite(config.max_jerk) || config.max_jerk <= 0.0) {
    return {};
  }
  const double required_scale = std::max({
      1.0,
      certificate.maximum_velocity_mps / config.max_vel,
      std::sqrt(certificate.maximum_acceleration_mps2 / config.max_acc),
      std::cbrt(certificate.maximum_jerk_mps3 / config.max_jerk)});
  if (!std::isfinite(required_scale) || required_scale <= 1.0) return {};

  const double first = std::min(maximum_scale, required_scale * 1.05);
  const double second = std::min(maximum_scale, first * 1.5);
  std::vector<double> scales;
  if (first > 1.0) scales.push_back(first);
  if (second > first + 1.0e-9) scales.push_back(second);
  if (maximum_scale > second + 1.0e-9) scales.push_back(maximum_scale);
  return scales;
}

// Prefer local time allocation before uniformly slowing an entire path.  Each
// multiplier is derived from the exact polynomial piece extrema and remains a
// proposal only; rebuilding changes shared junction derivatives, so the caller
// must repeat the complete trajectory certificate.
inline navigation_math::VecDf boundedPieceDurationRetryScales(
    const geometry_utils::Trajectory& trajectory,
    const traj_opt::Config& config,
    const double maximum_scale = 4.0) {
  const int piece_count = trajectory.getPieceNum();
  if (piece_count <= 0 || !std::isfinite(maximum_scale) || maximum_scale <= 1.0 ||
      !std::isfinite(config.max_vel) || config.max_vel <= 0.0 ||
      !std::isfinite(config.max_acc) || config.max_acc <= 0.0 ||
      !std::isfinite(config.max_jerk) || config.max_jerk <= 0.0) {
    return {};
  }
  navigation_math::VecDf scales(piece_count);
  for (int piece = 0; piece < piece_count; ++piece) {
    const double velocity = trajectory[piece].getMaxVelRate();
    const double acceleration = trajectory[piece].getMaxAccRate();
    const double jerk = trajectory[piece].getMaxJerRate();
    const double required = std::max({
        1.0, velocity / config.max_vel,
        std::sqrt(acceleration / config.max_acc),
        std::cbrt(jerk / config.max_jerk)});
    if (!std::isfinite(required)) return {};
    scales(piece) = required > 1.0
        ? std::min(maximum_scale, required * 1.05)
        : 1.0;
  }
  return scales;
}

enum class NominalCandidateSelection {
  kOptimized,
  kCertifiedSeed,
  kRejected,
};

// Selection copies the already-certified immutable object. It must never
// reconstruct a fallback from optimizer state after L-BFGS has started.
inline NominalCandidateSelection selectNominalCandidate(
    const geometry_utils::Trajectory& optimized_candidate,
    const bool optimized_candidate_certified,
    const geometry_utils::Trajectory& immutable_seed,
    const DeterministicNominalSeedCertificate& seed_certificate,
    geometry_utils::Trajectory& selected_candidate) {
  if (optimized_candidate_certified && !optimized_candidate.empty()) {
    selected_candidate = optimized_candidate;
    return NominalCandidateSelection::kOptimized;
  }
  if (seed_certificate.valid && !immutable_seed.empty()) {
    selected_candidate = immutable_seed;
    return NominalCandidateSelection::kCertifiedSeed;
  }
  selected_candidate.clear();
  return NominalCandidateSelection::kRejected;
}

inline navigation_math::StatePVAJ pieceState(
    const geometry_utils::Piece& piece, const double time_s) {
  navigation_math::StatePVAJ state;
  state.col(0) = piece.getPos(time_s);
  state.col(1) = piece.getVel(time_s);
  state.col(2) = piece.getAcc(time_s);
  state.col(3) = piece.getJer(time_s);
  return state;
}

// Bound the unavoidable forward error when Piece evaluates a power-basis
// polynomial in double precision. Long pieces can have large, cancelling
// coefficients even when their physical endpoint state is exactly constrained.
// A fixed absolute epsilon therefore confuses representation conditioning with
// a PVAJ discontinuity. The bound follows the absolute term sum and operation
// count; it is derived from machine precision, not tuned from flight data.
inline navigation_math::StatePVAJ pieceStateRoundoffBound(
    const geometry_utils::Piece& piece, const double time_s) {
  navigation_math::StatePVAJ bound = navigation_math::StatePVAJ::Zero();
  const auto& coefficients = piece.getCoeffMat();
  const int degree = piece.getDegree();
  if (!coefficients.allFinite() || !std::isfinite(time_s) || degree < 0) {
    bound.setConstant(std::numeric_limits<double>::infinity());
    return bound;
  }
  constexpr double kEvaluationOperationsPerTerm = 4.0;
  const double epsilon = std::numeric_limits<double>::epsilon();
  for (int derivative = 0; derivative <= 3; ++derivative) {
    const int term_count = degree - derivative + 1;
    if (term_count <= 0) continue;
    const double operation_count =
        kEvaluationOperationsPerTerm * static_cast<double>(term_count);
    const double denominator = 1.0 - operation_count * epsilon;
    if (!(denominator > 0.0)) {
      bound.col(derivative).setConstant(
          std::numeric_limits<double>::infinity());
      continue;
    }
    const double gamma = operation_count * epsilon / denominator;
    for (int axis = 0; axis < 3; ++axis) {
      long double absolute_term_sum = 0.0L;
      for (int column = 0; column < coefficients.cols(); ++column) {
        const int power = degree - column;
        if (power < derivative) continue;
        long double multiplier = 1.0L;
        for (int order = 0; order < derivative; ++order) {
          multiplier *= static_cast<long double>(power - order);
        }
        const long double time_power = std::pow(
            static_cast<long double>(std::abs(time_s)), power - derivative);
        absolute_term_sum += std::abs(
            static_cast<long double>(coefficients(axis, column)) *
            multiplier * time_power);
      }
      const long double numerical_bound =
          static_cast<long double>(gamma) * absolute_term_sum;
      bound(axis, derivative) = numerical_bound <=
              static_cast<long double>(std::numeric_limits<double>::max())
          ? static_cast<double>(numerical_bound)
          : std::numeric_limits<double>::infinity();
    }
  }
  return bound;
}

// Certify an immutable pre-optimizer trajectory. Each piece must retain its
// exact SFC provenance; geometric union/sampling checks are not substitutes.
inline DeterministicNominalSeedCertificate certifyDeterministicNominalSeed(
    const geometry_utils::Trajectory& seed,
    const navigation_math::PolyhedraH& corridor_planes,
    const navigation_math::VecDi& piece_to_corridor,
    const std::vector<unsigned char>& route_boundary_gates,
    const std::vector<navigation_math::Vec3f>& route_boundary_points,
    const std::vector<double>& route_boundary_radii,
    const navigation_math::StatePVAJ& expected_initial_state,
    const navigation_math::StatePVAJ& expected_terminal_state,
    const traj_opt::Config& config) {
  DeterministicNominalSeedCertificate result;
  const int piece_count = seed.getPieceNum();
  if (piece_count <= 0 || piece_to_corridor.size() != piece_count ||
      corridor_planes.empty() ||
      route_boundary_gates.size() != corridor_planes.size() ||
      route_boundary_points.size() != corridor_planes.size() ||
      route_boundary_radii.size() != corridor_planes.size() ||
      !expected_initial_state.allFinite() || !expected_terminal_state.allFinite() ||
      !std::isfinite(config.corridor_plane_tolerance_m) ||
      config.corridor_plane_tolerance_m < 0.0) {
    return result;
  }

  result.failure_stage = DeterministicNominalSeedFailureStage::kCorridor;
  result.maximum_corridor_violation_m =
      -std::numeric_limits<double>::infinity();
  result.maximum_boundary_residual = 0.0;
  for (int piece_index = 0; piece_index < piece_count; ++piece_index) {
    const int corridor_index = piece_to_corridor(piece_index);
    if (corridor_index < 0 ||
        corridor_index >= static_cast<int>(corridor_planes.size())) {
      return result;
    }
    auto normalized_planes = corridor_planes[static_cast<std::size_t>(corridor_index)];
    if (!normalizeCorridorPlanes(normalized_planes)) return result;
    const double violation = maximumContinuousCorridorPlaneViolation(
        seed[piece_index], normalized_planes);
    if (!std::isfinite(violation)) return result;
    result.maximum_corridor_violation_m = std::max(
        result.maximum_corridor_violation_m, violation);
  }
  if (result.maximum_corridor_violation_m >
      config.corridor_plane_tolerance_m) {
    return result;
  }

  result.failure_stage = DeterministicNominalSeedFailureStage::kBoundary;
  const auto initial_state = pieceState(seed[0], 0.0);
  const auto terminal_state = pieceState(
      seed[piece_count - 1], seed[piece_count - 1].getDuration());
  const auto initial_roundoff = pieceStateRoundoffBound(seed[0], 0.0);
  const auto terminal_roundoff = pieceStateRoundoffBound(
      seed[piece_count - 1], seed[piece_count - 1].getDuration());
  const auto initial_residual = (initial_state - expected_initial_state).cwiseAbs();
  const auto terminal_residual = (terminal_state - expected_terminal_state).cwiseAbs();
  result.maximum_boundary_residual = std::max(
      initial_residual.maxCoeff(), terminal_residual.maxCoeff());
  result.maximum_boundary_roundoff_bound = std::max(
      initial_roundoff.maxCoeff(), terminal_roundoff.maxCoeff());
  if (!initial_residual.allFinite() || !terminal_residual.allFinite() ||
      !initial_roundoff.allFinite() || !terminal_roundoff.allFinite() ||
      (initial_residual.array() > initial_roundoff.array()).any() ||
      (terminal_residual.array() > terminal_roundoff.array()).any()) {
    return result;
  }
  for (int piece_index = 0; piece_index + 1 < piece_count; ++piece_index) {
    const auto left = pieceState(
        seed[piece_index], seed[piece_index].getDuration());
    const auto right = pieceState(seed[piece_index + 1], 0.0);
    const navigation_math::StatePVAJ left_roundoff = pieceStateRoundoffBound(
        seed[piece_index], seed[piece_index].getDuration());
    const navigation_math::StatePVAJ right_roundoff =
        pieceStateRoundoffBound(seed[piece_index + 1], 0.0);
    const navigation_math::StatePVAJ junction_roundoff =
        (left_roundoff + right_roundoff).eval();
    const auto junction_residual = (left - right).cwiseAbs();
    result.maximum_boundary_residual = std::max(
        result.maximum_boundary_residual,
        junction_residual.maxCoeff());
    result.maximum_boundary_roundoff_bound = std::max(
        result.maximum_boundary_roundoff_bound,
        junction_roundoff.maxCoeff());
    if (!junction_residual.allFinite() || !junction_roundoff.allFinite() ||
        (junction_residual.array() > junction_roundoff.array()).any()) {
      return result;
    }
  }
  if (!std::isfinite(result.maximum_boundary_residual) ||
      !std::isfinite(result.maximum_boundary_roundoff_bound)) {
    return result;
  }

  result.failure_stage = DeterministicNominalSeedFailureStage::kRouteBoundary;
  for (std::size_t gate_index = 0; gate_index < route_boundary_gates.size(); ++gate_index) {
    if (route_boundary_gates[gate_index] == 0U) continue;
    const auto& point = route_boundary_points[gate_index];
    const double radius = route_boundary_radii[gate_index];
    if (!point.allFinite() || !std::isfinite(radius) || radius <= 0.0) {
      return result;
    }
    bool reached = false;
    for (const int junction : {static_cast<int>(gate_index) - 1,
                               static_cast<int>(gate_index)}) {
      if (junction < 0 || junction >= piece_count) continue;
      const double distance =
          (seed.getJuncPos(junction) - point.cast<double>()).norm();
      reached = reached || (std::isfinite(distance) && distance <= radius + 1.0e-6);
    }
    if (!reached) return result;
  }

  result.failure_stage = DeterministicNominalSeedFailureStage::kDynamics;
  result.maximum_velocity_mps = seed.getMaxVelRate();
  result.maximum_acceleration_mps2 = seed.getMaxAccRate();
  result.maximum_jerk_mps3 = seed.getMaxJerRate();
  if (!navigation_planning::withinNumericalDynamicLimit(
          result.maximum_velocity_mps, config.max_vel) ||
      !navigation_planning::withinNumericalDynamicLimit(
          result.maximum_acceleration_mps2, config.max_acc) ||
      !navigation_planning::withinNumericalDynamicLimit(
          result.maximum_jerk_mps3, config.max_jerk)) {
    return result;
  }

  result.failure_stage = DeterministicNominalSeedFailureStage::kFlatness;
  if (!traj_opt::trajectorySatisfiesFlatnessEnvelope(
          seed, config, &result.flatness_report)) {
    return result;
  }
  result.valid = true;
  result.failure_stage = DeterministicNominalSeedFailureStage::kNone;
  return result;
}

}  // namespace navigation_planning_backend
