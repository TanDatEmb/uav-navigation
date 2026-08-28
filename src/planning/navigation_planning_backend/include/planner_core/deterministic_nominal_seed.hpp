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
  double maximum_velocity_mps{std::numeric_limits<double>::infinity()};
  double maximum_acceleration_mps2{std::numeric_limits<double>::infinity()};
  double maximum_jerk_mps3{std::numeric_limits<double>::infinity()};
  traj_opt::TrajectoryDynamicReport flatness_report{};
};

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
  constexpr double kStateTolerance = 1.0e-8;
  const auto initial_state = pieceState(seed[0], 0.0);
  const auto terminal_state = pieceState(
      seed[piece_count - 1], seed[piece_count - 1].getDuration());
  result.maximum_boundary_residual = std::max(
      (initial_state - expected_initial_state).cwiseAbs().maxCoeff(),
      (terminal_state - expected_terminal_state).cwiseAbs().maxCoeff());
  for (int piece_index = 0; piece_index + 1 < piece_count; ++piece_index) {
    const auto left = pieceState(
        seed[piece_index], seed[piece_index].getDuration());
    const auto right = pieceState(seed[piece_index + 1], 0.0);
    result.maximum_boundary_residual = std::max(
        result.maximum_boundary_residual,
        (left - right).cwiseAbs().maxCoeff());
  }
  if (!std::isfinite(result.maximum_boundary_residual) ||
      result.maximum_boundary_residual > kStateTolerance) {
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
