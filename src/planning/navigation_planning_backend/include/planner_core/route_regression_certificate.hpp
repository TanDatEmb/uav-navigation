#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>

#include <data_structure/cmd_traj.h>
#include <navigation_mission/route_progress.hpp>
#include <utils/optimization/root_finder.h>

namespace navigation_planning_backend {

struct RouteRegressionCertificate {
  bool applicable{false};
  bool valid{false};
  double maximum_regression_m{std::numeric_limits<double>::infinity()};
  double first_violation_time_s{std::numeric_limits<double>::quiet_NaN()};
};

inline RouteRegressionCertificate certifyMainRouteRegression(
    const CandidateCommandBundle& candidate,
    const navigation_mission::ImmutableRouteSnapshot& route,
    const double validation_begin_tt,
    const double regression_tolerance_m,
    const bool bounded_terminal_stop_recovery = false) noexcept {
  RouteRegressionCertificate result;
  if (!route.valid() || route.active_waypoint_index == 0U ||
      route.active_waypoint_index >= route.waypoints.size()) {
    return result;
  }
  const auto active_segment_it = std::find_if(
      route.segments.begin(), route.segments.end(),
      [&route](const navigation_mission::RouteSegment& segment) {
        return segment.end_waypoint_index == route.active_waypoint_index;
      });
  if (active_segment_it == route.segments.end()) return result;
  const auto active_behavior =
      route.waypoints[route.active_waypoint_index].behavior;
  const bool bounded_stop_recovery =
      bounded_terminal_stop_recovery && candidate.terminal_stop &&
      active_behavior == navigation_mission::MissionWaypoint::Behavior::Stop;
  const bool bounded_pass_through_recovery =
      bounded_terminal_stop_recovery && !candidate.terminal_stop &&
      candidate.backup_suffix_available &&
      active_behavior == navigation_mission::MissionWaypoint::Behavior::PassThrough;
  if (bounded_stop_recovery || bounded_pass_through_recovery) {
    // A measured emergency stop can leave the vehicle just beyond a waypoint
    // acceptance ball. A bounded correction back toward that same waypoint is
    // recovery geometry, not an arbitrary nominal route fold. The caller
    // proves exact command identity, emergency provenance, proximity to the
    // mission-owned acceptance region, and that MAIN ends inside acceptance;
    // world, dynamic, tracking, and command-continuity certificates still run.
    result.applicable = false;
    result.valid = true;
    return result;
  }
  const bool has_unexecuted_main = std::any_of(
      candidate.roles.begin(), candidate.roles.end(),
      [validation_begin_tt](const CandidateRoleInterval& role) {
        return role.role == CandidateTrajectoryRole::MAIN &&
               role.end_tt + 1.0e-12 >= validation_begin_tt;
      });
  if (!has_unexecuted_main) return result;
  result.applicable = true;
  if (candidate.position.empty() || candidate.roles.empty() ||
      !std::isfinite(validation_begin_tt) || validation_begin_tt < 0.0 ||
      !std::isfinite(regression_tolerance_m) || regression_tolerance_m < 0.0) {
    return result;
  }

  const auto& active_segment = *active_segment_it;
  if (!active_segment.tangent.allFinite() ||
      std::abs(active_segment.tangent.norm() - 1.0) > 1.0e-6) {
    return result;
  }

  // A pass-through candidate changes route ownership inside the mission-owned
  // acceptance ball. Requiring the junction to equal the waypoint centre
  // makes a nonzero-speed C3 turn through two intersecting corridors
  // geometrically impossible. Before the closest in-ball junction progress
  // belongs to the incoming segment; after it progress belongs to the
  // outgoing segment. Keeping one incoming tangent across both phases falsely
  // classifies a physically valid fillet as a reverse manoeuvre.
  const auto outgoing_segment_it = std::find_if(
      route.segments.begin(), route.segments.end(),
      [&route](const navigation_mission::RouteSegment& segment) {
        return segment.start_waypoint_index == route.active_waypoint_index;
      });
  std::optional<int> boundary_piece_index;
  if (outgoing_segment_it != route.segments.end() &&
      route.waypoints[route.active_waypoint_index].behavior ==
          navigation_mission::MissionWaypoint::Behavior::PassThrough &&
      outgoing_segment_it->tangent.allFinite() &&
      std::abs(outgoing_segment_it->tangent.norm() - 1.0) <= 1.0e-6) {
    const Eigen::Vector3d boundary =
        route.waypoints[route.active_waypoint_index].position_enu;
    const double acceptance_radius_m =
        route.waypoints[route.active_waypoint_index].acceptance_radius_m;
    double closest_junction_distance_m = std::numeric_limits<double>::infinity();
    for (int piece_index = 0;
         piece_index + 1 < candidate.position.getPieceNum(); ++piece_index) {
      const auto& piece = candidate.position[piece_index];
      const Eigen::Vector3d junction = piece.getPos(piece.getDuration());
      const double junction_distance_m = (junction - boundary).norm();
      if (junction.allFinite() && std::isfinite(acceptance_radius_m) &&
          acceptance_radius_m > 0.0 && std::isfinite(junction_distance_m) &&
          junction_distance_m <= acceptance_radius_m + 1.0e-6 &&
          junction_distance_m < closest_junction_distance_m) {
        boundary_piece_index = piece_index;
        closest_junction_distance_m = junction_distance_m;
      }
    }
  }

  const double duration = candidate.position.getTotalDuration();
  if (!std::isfinite(duration) || validation_begin_tt > duration + 1.0e-9) {
    return result;
  }

  double high_water_m = -std::numeric_limits<double>::infinity();
  double maximum_regression_m = 0.0;
  bool evaluated = false;
  double piece_begin_tt = 0.0;
  for (int piece_index = 0; piece_index < candidate.position.getPieceNum();
       ++piece_index) {
    const auto& piece = candidate.position[piece_index];
    const double piece_duration = piece.getDuration();
    const double piece_end_tt = piece_begin_tt + piece_duration;
    if (!std::isfinite(piece_duration) || piece_duration <= 0.0) return result;

    const bool outgoing_phase = boundary_piece_index.has_value() &&
        piece_index > *boundary_piece_index;
    const auto& progress_segment = outgoing_phase
        ? *outgoing_segment_it : active_segment;
    const Eigen::Vector3d tangent = progress_segment.tangent;

    for (const auto& role : candidate.roles) {
      if (role.role != CandidateTrajectoryRole::MAIN) continue;
      const double begin_tt = std::max({validation_begin_tt, piece_begin_tt,
                                        role.begin_tt});
      const double end_tt = std::min({duration, piece_end_tt, role.end_tt});
      if (end_tt + 1.0e-12 < begin_tt) continue;

      const double local_begin = std::clamp(
          begin_tt - piece_begin_tt, 0.0, piece_duration);
      const double local_end = std::clamp(
          end_tt - piece_begin_tt, 0.0, piece_duration);
      std::set<double> candidate_times{local_begin, local_end};
      const Eigen::VectorXd normalized_velocity =
          (tangent.transpose() * piece.normalizeVelCoeffMat()).transpose();
      const double tau_begin = local_begin / piece_duration;
      const double tau_end = local_end / piece_duration;
      for (const double root : math_utils::RootFinder::solvePolynomial(
               normalized_velocity, tau_begin - 1.0e-12,
               tau_end + 1.0e-12, 1.0e-10)) {
        if (std::isfinite(root) && root >= tau_begin - 1.0e-10 &&
            root <= tau_end + 1.0e-10) {
          candidate_times.insert(std::clamp(root, tau_begin, tau_end) *
                                 piece_duration);
        }
      }

      for (const double local_time : candidate_times) {
        const double progress_m =
            progress_segment.start_arc_m +
            tangent.dot(piece.getPos(local_time) - progress_segment.start);
        if (!std::isfinite(progress_m)) return result;
        evaluated = true;
        high_water_m = std::max(high_water_m, progress_m);
        maximum_regression_m =
            std::max(maximum_regression_m, high_water_m - progress_m);
        if (maximum_regression_m > regression_tolerance_m + 1.0e-9 &&
            !std::isfinite(result.first_violation_time_s)) {
          result.first_violation_time_s = piece_begin_tt + local_time;
        }
      }
    }
    piece_begin_tt = piece_end_tt;
  }

  result.maximum_regression_m = maximum_regression_m;
  result.valid = evaluated &&
      maximum_regression_m <= regression_tolerance_m + 1.0e-9;
  return result;
}

}  // namespace navigation_planning_backend
