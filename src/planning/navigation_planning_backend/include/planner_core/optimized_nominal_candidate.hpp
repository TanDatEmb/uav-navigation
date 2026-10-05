#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <navigation_planning/planning_limits.hpp>
#include <planner_core/boundary_velocity_recovery.hpp>
#include <planner_core/corridor_plane_validation.hpp>
#include <traj_opt/config.hpp>
#include <traj_opt/trajectory_dynamics.hpp>
#include <utils/header/type_utils.hpp>

namespace navigation_planning_backend {

enum class OptimizedNominalCandidateFailureStage {
  kNone = 0,
  kInput = 1,
  kCorridor = 2,
  kRouteBoundary = 3,
  kDynamics = 4,
  kFlatness = 5,
};

struct OptimizedNominalCandidateCertificate {
  bool valid{false};
  OptimizedNominalCandidateFailureStage failure_stage{
      OptimizedNominalCandidateFailureStage::kInput};
  int failing_piece_index{-1};
  int failing_corridor_index{-1};
  int failing_route_boundary_index{-1};
  double maximum_corridor_violation_m{
      std::numeric_limits<double>::infinity()};
  double maximum_velocity_mps{std::numeric_limits<double>::infinity()};
  double maximum_acceleration_mps2{
      std::numeric_limits<double>::infinity()};
  double maximum_jerk_mps3{std::numeric_limits<double>::infinity()};
  BoundaryVelocityRecoveryReport velocity_recovery{};
  traj_opt::TrajectoryDynamicReport flatness{};
};

// Certify a trajectory produced by the current MINCO transaction.  Endpoint
// and junction continuity are construction invariants of that transaction;
// unlike an independently supplied deterministic seed, the optimized
// candidate must not be reclassified using the seed-import roundoff policy.
// Every externally meaningful hard gate is nevertheless evaluated here from
// the immutable trajectory value.
inline OptimizedNominalCandidateCertificate certifyOptimizedNominalCandidate(
    const geometry_utils::Trajectory& candidate,
    const navigation_math::PolyhedraH& corridor_planes,
    const navigation_math::VecDi& piece_to_corridor,
    const std::vector<unsigned char>& route_boundary_gates,
    const std::vector<navigation_math::Vec3f>& route_boundary_points,
    const std::vector<double>& route_boundary_radii,
    const traj_opt::Config& config) {
  OptimizedNominalCandidateCertificate result;
  const int piece_count = candidate.getPieceNum();
  if (piece_count <= 0 || piece_to_corridor.size() != piece_count ||
      corridor_planes.empty() ||
      route_boundary_gates.size() != corridor_planes.size() ||
      route_boundary_points.size() != corridor_planes.size() ||
      route_boundary_radii.size() != corridor_planes.size() ||
      !std::isfinite(config.corridor_plane_tolerance_m) ||
      config.corridor_plane_tolerance_m < 0.0) {
    return result;
  }

  result.failure_stage = OptimizedNominalCandidateFailureStage::kCorridor;
  result.maximum_corridor_violation_m =
      -std::numeric_limits<double>::infinity();
  for (int piece_index = 0; piece_index < piece_count; ++piece_index) {
    const int corridor_index = piece_to_corridor(piece_index);
    if (corridor_index < 0 ||
        corridor_index >= static_cast<int>(corridor_planes.size())) {
      result.failing_piece_index = piece_index;
      result.failing_corridor_index = corridor_index;
      return result;
    }
    const double violation = maximumContinuousCorridorPlaneViolation(
        candidate[piece_index],
        corridor_planes[static_cast<std::size_t>(corridor_index)]);
    if (!std::isfinite(violation)) {
      result.failing_piece_index = piece_index;
      result.failing_corridor_index = corridor_index;
      return result;
    }
    if (violation > result.maximum_corridor_violation_m) {
      result.maximum_corridor_violation_m = violation;
      result.failing_piece_index = piece_index;
      result.failing_corridor_index = corridor_index;
    }
  }
  if (result.maximum_corridor_violation_m >
      config.corridor_plane_tolerance_m) {
    return result;
  }

  result.failure_stage = OptimizedNominalCandidateFailureStage::kRouteBoundary;
  for (std::size_t gate_index = 0;
       gate_index < route_boundary_gates.size(); ++gate_index) {
    if (route_boundary_gates[gate_index] == 0U) continue;
    const auto& point = route_boundary_points[gate_index];
    const double radius = route_boundary_radii[gate_index];
    if (!point.allFinite() || !std::isfinite(radius) || radius <= 0.0) {
      result.failing_route_boundary_index = static_cast<int>(gate_index);
      return result;
    }
    bool reached = false;
    for (const int junction_index : {
             static_cast<int>(gate_index) - 1,
             static_cast<int>(gate_index)}) {
      if (junction_index < 0 || junction_index >= piece_count) continue;
      const double distance =
          (candidate.getJuncPos(junction_index) - point.cast<double>()).norm();
      reached = reached ||
          (std::isfinite(distance) && distance <= radius + 1.0e-6);
    }
    if (!reached) {
      result.failing_route_boundary_index = static_cast<int>(gate_index);
      return result;
    }
  }

  result.failure_stage = OptimizedNominalCandidateFailureStage::kDynamics;
  result.maximum_velocity_mps = candidate.getMaxVelRate();
  result.maximum_acceleration_mps2 = candidate.getMaxAccRate();
  result.maximum_jerk_mps3 = candidate.getMaxJerRate();
  result.velocity_recovery = certifyBoundaryVelocityRecovery(
      candidate, config.max_vel, config.max_acc, config.max_jerk);
  if (!result.velocity_recovery.satisfied ||
      !std::isfinite(result.maximum_velocity_mps) ||
      !navigation_planning::withinNumericalDynamicLimit(
          result.maximum_acceleration_mps2, config.max_acc) ||
      !navigation_planning::withinNumericalDynamicLimit(
          result.maximum_jerk_mps3, config.max_jerk)) {
    return result;
  }

  result.failure_stage = OptimizedNominalCandidateFailureStage::kFlatness;
  if (!traj_opt::trajectorySatisfiesFlatnessEnvelope(
          candidate, config, &result.flatness)) {
    return result;
  }

  result.valid = true;
  result.failure_stage = OptimizedNominalCandidateFailureStage::kNone;
  return result;
}

}  // namespace navigation_planning_backend
