#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <data_structure/cmd_traj.h>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_planning_backend {

struct SweptValidationResult {
  bool valid{false};
  double begin_tt{0.0};
  double first_blocked_tt{0.0};
  std::size_t sample_count{0};
  std::size_t segment_count{0};
  navigation_world_model::AxisAlignedBox protected_region{};
  enum class Failure : std::uint8_t {
    kNone,
    kInvalidTimeWindow,
    kInvalidRoleSchedule,
    kInvalidWorldGeometry,
    kNonFiniteTrajectory,
    kInitialPointBlocked,
    kRoleBoundaryMissing,
    kPieceLookupFailed,
    kCertificateTubeBlocked,
    kEndpointBlocked,
    kSegmentBlocked,
  } failure{Failure::kNone};
  CandidateTrajectoryRole blocked_role{CandidateTrajectoryRole::MAIN};
  navigation_world_model::CellState blocked_cell_state{
      navigation_world_model::CellState::kUndefined};
  navigation_world_model::Point3 blocked_position{
      navigation_world_model::Point3::Constant(
          std::numeric_limits<double>::quiet_NaN())};
};

inline const char* sweptValidationFailureName(
        const SweptValidationResult::Failure failure) noexcept {
    switch (failure) {
      case SweptValidationResult::Failure::kNone: return "none";
      case SweptValidationResult::Failure::kInvalidTimeWindow:
        return "invalid_time_window";
      case SweptValidationResult::Failure::kInvalidRoleSchedule:
        return "invalid_role_schedule";
      case SweptValidationResult::Failure::kInvalidWorldGeometry:
        return "invalid_world_geometry";
      case SweptValidationResult::Failure::kNonFiniteTrajectory:
        return "non_finite_trajectory";
      case SweptValidationResult::Failure::kInitialPointBlocked:
        return "initial_point_blocked";
      case SweptValidationResult::Failure::kRoleBoundaryMissing:
        return "role_boundary_missing";
      case SweptValidationResult::Failure::kPieceLookupFailed:
        return "piece_lookup_failed";
      case SweptValidationResult::Failure::kCertificateTubeBlocked:
        return "certificate_tube_blocked";
      case SweptValidationResult::Failure::kEndpointBlocked:
        return "endpoint_blocked";
      case SweptValidationResult::Failure::kSegmentBlocked:
        return "segment_blocked";
    }
    return "unknown";
}

inline double pointAabbDistanceSquared(
        const navigation_world_model::Point3& point,
        const navigation_world_model::Point3& minimum,
        const navigation_world_model::Point3& maximum) noexcept {
    double distance_squared = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        double distance = 0.0;
        if (point[axis] < minimum[axis]) {
            distance = minimum[axis] - point[axis];
        } else if (point[axis] > maximum[axis]) {
            distance = point[axis] - maximum[axis];
        }
        distance_squared += distance * distance;
    }
    return distance_squared;
}

// The squared distance from a line segment to an axis-aligned cell is a
// convex piecewise-quadratic function. Split the segment where an axis enters
// or leaves the cell, then minimize the corresponding quadratic on each
// interval. This avoids the center-plus-half-diagonal over-approximation while
// remaining deterministic and allocation-free in the hot validation loop.
inline double segmentAabbDistanceSquared(
        const navigation_world_model::Point3& start,
        const navigation_world_model::Point3& end,
        const navigation_world_model::Point3& minimum,
        const navigation_world_model::Point3& maximum) noexcept {
    if (!start.allFinite() || !end.allFinite() || !minimum.allFinite() ||
        !maximum.allFinite() || (maximum.array() < minimum.array()).any()) {
        return std::numeric_limits<double>::infinity();
    }
    const auto direction = end - start;
    // Two segment endpoints plus two boundary crossings for each of the
    // three axes. The sentinel 2.0 sorts after the [0, 1] segment domain.
    std::array<double, 8> breakpoints{0.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    for (int axis = 0; axis < 3; ++axis) {
        if (direction[axis] != 0.0) {
            for (int boundary_index = 0; boundary_index < 2; ++boundary_index) {
                const double boundary = boundary_index == 0
                    ? minimum[axis] : maximum[axis];
                const double parameter = (boundary - start[axis]) / direction[axis];
                if (std::isfinite(parameter) && parameter > 0.0 && parameter < 1.0) {
                    breakpoints[2 + 2 * axis + boundary_index] = parameter;
                }
            }
        }
    }
    std::sort(breakpoints.begin(), breakpoints.end());

    auto evaluate = [&](const double parameter) {
        return pointAabbDistanceSquared(
            start + parameter * direction, minimum, maximum);
    };
    double best = std::min(evaluate(0.0), evaluate(1.0));
    for (std::size_t interval = 0; interval + 1U < breakpoints.size(); ++interval) {
        const double lower = breakpoints[interval];
        const double upper = breakpoints[interval + 1U];
        if (lower >= 1.0) break;
        const double middle = 0.5 * (lower + upper);
        double quadratic = 0.0;
        double linear = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            const double middle_value = start[axis] + direction[axis] * middle;
            double offset = 0.0;
            if (middle_value < minimum[axis]) {
                offset = start[axis] - minimum[axis];
            } else if (middle_value > maximum[axis]) {
                offset = start[axis] - maximum[axis];
            } else {
                continue;
            }
            quadratic += direction[axis] * direction[axis];
            linear += direction[axis] * offset;
        }
        const double minimizer = quadratic > 0.0
            ? std::clamp(-linear / quadratic, lower, upper)
            : lower;
        best = std::min(best, evaluate(minimizer));
    }
    return best;
}

inline constexpr double kMaximumCurveDeviationFraction = 0.25;
inline constexpr std::size_t kMaximumCertificateCellsPerSegment = 4096U;

inline double polynomialAccelerationBound(
        const geometry_utils::Piece& piece) noexcept {
    const double duration = piece.getDuration();
    const auto& coefficients = piece.getCoeffMat();
    const int degree = piece.getDegree();
    if (!std::isfinite(duration) || duration < 0.0 || degree < 2 ||
        coefficients.rows() != 3 || coefficients.cols() != degree + 1 ||
        !coefficients.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }

    double bound = 0.0;
    for (int column = 0; column <= degree - 2; ++column) {
        const int power = degree - column;
        const double coefficient = static_cast<double>(power * (power - 1));
        const double time_bound = std::pow(duration, power - 2);
        const double term = coefficient * coefficients.col(column).norm() * time_bound;
        if (!std::isfinite(term)) return std::numeric_limits<double>::infinity();
        bound += term;
        if (!std::isfinite(bound)) return std::numeric_limits<double>::infinity();
    }
    return bound;
}

inline bool certificateTubeIsSafe(
        const navigation_world_model::WorldModelView& world,
        const navigation_world_model::Point3& start,
        const navigation_world_model::Point3& end,
        const double curve_deviation_m,
        const navigation_world_model::UnknownPolicy unknown_policy,
        const double resolution_m,
        navigation_world_model::CellState* blocked_cell_state = nullptr,
        navigation_world_model::Point3* blocked_position = nullptr) noexcept {
    if (blocked_cell_state != nullptr) {
        *blocked_cell_state = navigation_world_model::CellState::kUndefined;
    }
    if (blocked_position != nullptr) {
        *blocked_position = navigation_world_model::Point3::Constant(
            std::numeric_limits<double>::quiet_NaN());
    }
    if (!start.allFinite() || !end.allFinite() ||
        !std::isfinite(curve_deviation_m) || curve_deviation_m < 0.0 ||
        !std::isfinite(resolution_m) || resolution_m <= 0.0) {
        return false;
    }

    // The polynomial arc is within curve_deviation_m of its chord. Expand the
    // index enumeration by half a voxel diagonal so every cell whose box can
    // meet that tube is represented. The exact cell-box distance filter below
    // is important: a cell center can be close to the chord while its voxel
    // box is separated from the actual tube, especially at a diagonal corner.
    const double cell_half_diagonal = 0.5 * std::sqrt(3.0) * resolution_m;
    const double expansion = curve_deviation_m +
        cell_half_diagonal;
    const double tube_radius_squared = curve_deviation_m * curve_deviation_m;
    if (!std::isfinite(cell_half_diagonal) || !std::isfinite(expansion) ||
        !std::isfinite(tube_radius_squared)) return false;
    const auto minimum = start.cwiseMin(end).array() - expansion;
    const auto maximum = start.cwiseMax(end).array() + expansion;
    const auto minimum_index = world.positionToIndex(
        minimum.matrix(), navigation_world_model::GridLayer::kInflated);
    const auto maximum_index = world.positionToIndex(
        maximum.matrix(), navigation_world_model::GridLayer::kInflated);

    std::array<std::int64_t, 3> lower{};
    std::array<std::int64_t, 3> upper{};
    for (int axis = 0; axis < 3; ++axis) {
        lower[axis] = std::min<std::int64_t>(minimum_index(axis), maximum_index(axis));
        upper[axis] = std::max<std::int64_t>(minimum_index(axis), maximum_index(axis));
    }
    std::uint64_t cell_count = 1U;
    for (int axis = 0; axis < 3; ++axis) {
        const auto span = static_cast<std::uint64_t>(upper[axis] - lower[axis]) + 1U;
        if (span == 0U || cell_count >
                kMaximumCertificateCellsPerSegment / span) {
            return false;
        }
        cell_count *= span;
    }

    for (std::int64_t x = lower[0]; x <= upper[0]; ++x) {
        for (std::int64_t y = lower[1]; y <= upper[1]; ++y) {
            for (std::int64_t z = lower[2]; z <= upper[2]; ++z) {
                const navigation_world_model::GridIndex3 index(
                    static_cast<int>(x), static_cast<int>(y), static_cast<int>(z));
                const auto cell_center = world.indexToPosition(
                    index, navigation_world_model::GridLayer::kInflated);
                const auto cell_minimum = cell_center -
                    navigation_world_model::Point3::Constant(0.5 * resolution_m);
                const auto cell_maximum = cell_center +
                    navigation_world_model::Point3::Constant(0.5 * resolution_m);
                const double distance_squared = segmentAabbDistanceSquared(
                    start, end, cell_minimum, cell_maximum);
                const double distance_tolerance =
                    64.0 * std::numeric_limits<double>::epsilon() *
                    std::max(1.0, tube_radius_squared);
                if (!std::isfinite(distance_squared) ||
                    distance_squared > tube_radius_squared + distance_tolerance) {
                    if (z == upper[2]) break;
                    continue;
                }
                const auto state = world.classify(
                    cell_center, navigation_world_model::GridLayer::kInflated);
                if (!navigation_world_model::isCellTraversable(state, unknown_policy)) {
                    if (blocked_cell_state != nullptr) *blocked_cell_state = state;
                    if (blocked_position != nullptr) *blocked_position = cell_center;
                    return false;
                }
                if (z == upper[2]) break;
            }
            if (y == upper[1]) break;
        }
        if (x == upper[0]) break;
    }
    return true;
}

inline bool candidateRoleScheduleIsComplete(
        const CandidateCommandBundle& candidate) {
    const double duration = candidate.position.getTotalDuration();
    if (!std::isfinite(duration) || duration < 0.0 || candidate.roles.empty()) {
        return false;
    }
    double cursor = 0.0;
    for (const auto& interval : candidate.roles) {
        if (!std::isfinite(interval.begin_tt) || !std::isfinite(interval.end_tt) ||
            interval.begin_tt < 0.0 ||
            interval.end_tt <= interval.begin_tt ||
            interval.begin_tt != cursor ||
            (interval.role != CandidateTrajectoryRole::MAIN &&
             interval.role != CandidateTrajectoryRole::BACKUP)) {
            return false;
        }
        cursor = interval.end_tt;
    }
    return cursor == duration;
}

struct TrajectoryPieceLocation {
    int index{-1};
    double local_time{0.0};
    double end_time{0.0};
};

// Trajectory::locatePieceIdx() is a legacy endpoint evaluator: at an exact
// piece boundary it returns the piece that has just ended.  A swept validator
// needs the half-open interval convention instead, otherwise the computed
// next time is equal to the current time and a valid candidate is rejected
// without inspecting the next piece.
inline std::optional<TrajectoryPieceLocation> locatePieceForSweep(
        const Trajectory& trajectory, const double time_s) {
    if (!std::isfinite(time_s) || time_s < 0.0 || trajectory.empty()) {
        return std::nullopt;
    }
    const int piece_count = trajectory.getPieceNum();
    double piece_begin = 0.0;
    for (int index = 0; index < piece_count; ++index) {
        const double piece_duration = trajectory[index].getDuration();
        if (!std::isfinite(piece_duration) || piece_duration <= 0.0) {
            return std::nullopt;
        }
        const double piece_end = piece_begin + piece_duration;
        if (!std::isfinite(piece_end)) return std::nullopt;
        // The authorization clock is formed by subtracting two wall-time
        // doubles. Near a polynomial boundary that subtraction can leave a
        // time a few ulps before the boundary. Computing the remaining time
        // as `piece_end - time_s` then rounds to zero, so the sweep would
        // report piece_lookup_failed instead of advancing to the next piece.
        // Treat only this machine-scale boundary band as the next half-open
        // interval; trajectory geometry and safety policies are unchanged.
        const double boundary_tolerance = 32.0 * std::numeric_limits<double>::epsilon() *
            std::max({1.0, std::abs(time_s), std::abs(piece_begin), std::abs(piece_end)});
        if (index != piece_count - 1 &&
            time_s >= piece_end - boundary_tolerance) {
            piece_begin = piece_end;
            continue;
        }
        if (time_s < piece_end || index == piece_count - 1) {
            return TrajectoryPieceLocation{
                index,
                std::clamp(time_s - piece_begin, 0.0, piece_duration),
                piece_end};
        }
        piece_begin = piece_end;
    }
    return std::nullopt;
}

// The mission-owned UNKNOWN policy is applied to every swept segment. OCCUPIED
// and OUT_OF_MAP always fail closed; the caller must choose explicitly whether
// UNKNOWN is admissible for this candidate.
inline SweptValidationResult validateExecutableCandidate(
        const navigation_world_model::WorldModelView& world,
        const CandidateCommandBundle& candidate,
        double authorization_wall_time,
        navigation_world_model::UnknownPolicy unknown_policy) {
    SweptValidationResult result;
    const double duration = candidate.position.getTotalDuration();
    if (!std::isfinite(duration) || duration < 0.0 ||
        !std::isfinite(candidate.start_wall_time) ||
        !std::isfinite(authorization_wall_time)) {
        result.failure = SweptValidationResult::Failure::kInvalidTimeWindow;
        return result;
    }
    const double end_wall_time = candidate.start_wall_time + duration;
    if (!std::isfinite(end_wall_time) || authorization_wall_time > end_wall_time) {
        // A finished candidate cannot be made safe by clamping validation to
        // its terminal point. It has no remaining executable horizon.
        result.failure = SweptValidationResult::Failure::kInvalidTimeWindow;
        return result;
    }
    result.begin_tt = std::clamp(
        authorization_wall_time - candidate.start_wall_time, 0.0, duration);
    result.first_blocked_tt = result.begin_tt;
    if (!std::isfinite(result.begin_tt)) {
        result.failure = SweptValidationResult::Failure::kInvalidTimeWindow;
        return result;
    }

    if (!candidateRoleScheduleIsComplete(candidate)) {
        result.failure = SweptValidationResult::Failure::kInvalidRoleSchedule;
        return result;
    }

    const auto geometry = world.geometry();
    if (!std::isfinite(geometry.inflated_resolution_m) ||
        geometry.inflated_resolution_m <= 0.0) {
        result.failure = SweptValidationResult::Failure::kInvalidWorldGeometry;
        return result;
    }
    const double spatial_step = 0.5 * geometry.inflated_resolution_m;
    const double curve_deviation_tolerance =
        kMaximumCurveDeviationFraction * geometry.inflated_resolution_m;
    if (!std::isfinite(spatial_step) || spatial_step <= 0.0 ||
        !std::isfinite(curve_deviation_tolerance) || curve_deviation_tolerance <= 0.0) {
        result.failure = SweptValidationResult::Failure::kInvalidWorldGeometry;
        return result;
    }
    double t = result.begin_tt;
    const auto role_at = [&candidate, duration](const double time_s)
        -> std::optional<CandidateTrajectoryRole> {
        for (const auto& interval : candidate.roles) {
            const bool in_interval = time_s >= interval.begin_tt &&
                (time_s < interval.end_tt ||
                 (time_s == duration && time_s == interval.end_tt));
            if (in_interval) return std::optional<CandidateTrajectoryRole>{interval.role};
        }
        return std::nullopt;
    };
    const auto policy_for_role = [unknown_policy](const CandidateTrajectoryRole role) {
        return role == CandidateTrajectoryRole::BACKUP
            ? navigation_world_model::UnknownPolicy::kRequireKnownFree
            : unknown_policy;
    };
    const auto next_role_boundary = [&candidate, duration](const double time_s) {
        double boundary = duration;
        for (const auto& interval : candidate.roles) {
            if (interval.begin_tt > time_s) {
                boundary = std::min(boundary, interval.begin_tt);
            }
        }
        return boundary;
    };
    auto previous = candidate.position.getPos(t);
    if (!previous.allFinite()) {
        result.failure = SweptValidationResult::Failure::kNonFiniteTrajectory;
        return result;
    }

    const auto include_protected_segment =
        [&result, resolution = geometry.inflated_resolution_m](
            const navigation_world_model::Point3& start,
            const navigation_world_model::Point3& end,
            const double curve_deviation_m) -> bool {
          const double cell_half_diagonal = 0.5 * std::sqrt(3.0) * resolution;
          const double expansion = curve_deviation_m + cell_half_diagonal;
          if (!start.allFinite() || !end.allFinite() ||
              !std::isfinite(expansion) || expansion < 0.0) {
            return false;
          }
          const auto minimum = start.cwiseMin(end).array() - expansion;
          const auto maximum = start.cwiseMax(end).array() + expansion;
          if (!minimum.allFinite() || !maximum.allFinite()) return false;
          if (result.sample_count == 0U && result.segment_count == 0U) {
            result.protected_region.minimum = minimum.matrix();
            result.protected_region.maximum = maximum.matrix();
          } else {
            result.protected_region.minimum =
                result.protected_region.minimum.cwiseMin(minimum.matrix());
            result.protected_region.maximum =
                result.protected_region.maximum.cwiseMax(maximum.matrix());
          }
          return result.protected_region.minimum.allFinite() &&
              result.protected_region.maximum.allFinite();
        };
    if (!include_protected_segment(previous, previous, 0.0)) {
        result.failure = SweptValidationResult::Failure::kNonFiniteTrajectory;
        return result;
    }

    const auto point_safe = [&world](const auto& point,
                                     const navigation_world_model::UnknownPolicy policy) {
        const auto state = world.classify(
            point, navigation_world_model::GridLayer::kInflated);
        return navigation_world_model::isCellTraversable(state, policy);
    };
    const auto initial_role = role_at(t);
    if (!initial_role) {
        result.failure = SweptValidationResult::Failure::kRoleBoundaryMissing;
        return result;
    }
    if (!point_safe(previous, policy_for_role(*initial_role))) {
        result.failure = SweptValidationResult::Failure::kInitialPointBlocked;
        result.blocked_role = *initial_role;
        result.blocked_position = previous;
        result.blocked_cell_state = world.classify(
            previous, navigation_world_model::GridLayer::kInflated);
        return result;
    }
    ++result.sample_count;

    while (t < duration) {
        const auto role = role_at(t);
        if (!role) {
            result.failure = SweptValidationResult::Failure::kRoleBoundaryMissing;
            return result;
        }
        result.blocked_role = *role;
        const auto segment_policy = policy_for_role(*role);
        const auto piece_location = locatePieceForSweep(candidate.position, t);
        if (!piece_location) {
            result.failure = SweptValidationResult::Failure::kPieceLookupFailed;
            return result;
        }
        const int piece_index = piece_location->index;
        const auto& piece = candidate.position[piece_index];
        const double acceleration_bound = polynomialAccelerationBound(piece);
        if (!std::isfinite(acceleration_bound)) {
            result.failure = SweptValidationResult::Failure::kNonFiniteTrajectory;
            return result;
        }
        const double speed_norm = candidate.position.getVel(t).norm();
        if (!std::isfinite(speed_norm)) {
            result.failure = SweptValidationResult::Failure::kNonFiniteTrajectory;
            return result;
        }
        const double speed = std::max(0.1, speed_norm);
        double dt = std::clamp(spatial_step / speed, 0.002, 0.05);
        // Use the cumulative boundary returned by the same half-open lookup
        // instead of reconstructing it as `t + duration - local_time`.
        // The latter can round back to `t` after a wall-time-to-command-time
        // subtraction near a piece boundary and falsely reject an otherwise
        // finite candidate with piece_lookup_failed.
        double next_t = std::min({duration, t + dt,
                                  piece_location->end_time,
                                  next_role_boundary(t)});
        if (!(next_t > t)) {
            result.failure = SweptValidationResult::Failure::kPieceLookupFailed;
            return result;
        }
        auto next = candidate.position.getPos(next_t);
        while (next.allFinite() && (next - previous).norm() > spatial_step &&
               dt > 0.002 + 1.0e-12) {
            dt = std::max(0.002, 0.5 * dt);
            next_t = std::min({duration, t + dt,
                               piece_location->end_time,
                               next_role_boundary(t)});
            next = candidate.position.getPos(next_t);
        }
        double segment_dt = next_t - t;
        double curve_deviation_bound =
            acceleration_bound * segment_dt * segment_dt / 8.0;
        while (std::isfinite(curve_deviation_bound) &&
               curve_deviation_bound > curve_deviation_tolerance &&
               dt > 0.002 + 1.0e-12) {
            dt = std::max(0.002, 0.5 * dt);
            next_t = std::min({duration, t + dt,
                               piece_location->end_time,
                               next_role_boundary(t)});
            if (!(next_t > t)) {
                result.failure = SweptValidationResult::Failure::kPieceLookupFailed;
                return result;
            }
            next = candidate.position.getPos(next_t);
            segment_dt = next_t - t;
            curve_deviation_bound =
                acceleration_bound * segment_dt * segment_dt / 8.0;
        }
        navigation_world_model::CellState tube_blocked_cell_state{
            navigation_world_model::CellState::kUndefined};
        navigation_world_model::Point3 tube_blocked_position =
            navigation_world_model::Point3::Constant(
                std::numeric_limits<double>::quiet_NaN());
        const bool tube_safe = certificateTubeIsSafe(
            world, previous, next, curve_deviation_bound, segment_policy,
            geometry.inflated_resolution_m, &tube_blocked_cell_state,
            &tube_blocked_position);
        if (!std::isfinite(curve_deviation_bound) ||
            curve_deviation_bound > curve_deviation_tolerance || !tube_safe) {
            result.failure = SweptValidationResult::Failure::kCertificateTubeBlocked;
            if (tube_blocked_position.allFinite()) {
                result.blocked_position = tube_blocked_position;
                result.blocked_cell_state = tube_blocked_cell_state;
            } else {
                result.blocked_position = next;
            }
            if (result.blocked_cell_state ==
                    navigation_world_model::CellState::kUndefined && next.allFinite()) {
                result.blocked_cell_state = world.classify(
                    next, navigation_world_model::GridLayer::kInflated);
            }
            return result;
        }
        if (!include_protected_segment(previous, next, curve_deviation_bound)) {
            result.failure = SweptValidationResult::Failure::kNonFiniteTrajectory;
            return result;
        }
        result.first_blocked_tt = next_t;
        const bool endpoint_safe = next.allFinite() && point_safe(next, segment_policy);
        const bool segment_safe = endpoint_safe && world.isSegmentTraversable(
            previous, next, navigation_world_model::GridLayer::kInflated,
            segment_policy);
        if (!endpoint_safe || !segment_safe) {
            result.failure = !next.allFinite()
                ? SweptValidationResult::Failure::kNonFiniteTrajectory
                : (!endpoint_safe
                    ? SweptValidationResult::Failure::kEndpointBlocked
                    : SweptValidationResult::Failure::kSegmentBlocked);
            result.blocked_position = next;
            if (next.allFinite()) {
                result.blocked_cell_state = world.classify(
                    next, navigation_world_model::GridLayer::kInflated);
            }
            return result;
        }
        ++result.sample_count;
        ++result.segment_count;
        previous = next;
        t = next_t;
    }
    result.valid = true;
    result.first_blocked_tt = duration;
    return result;
}

inline bool candidateHasBackupSuffix(const CandidateCommandBundle& candidate) {
    const double duration = candidate.position.getTotalDuration();
    if (!candidateRoleScheduleIsComplete(candidate)) return false;
    const auto& final_interval = candidate.roles.back();
    return final_interval.role == CandidateTrajectoryRole::BACKUP &&
           std::isfinite(final_interval.begin_tt) &&
           std::isfinite(final_interval.end_tt) &&
           final_interval.end_tt > final_interval.begin_tt &&
           final_interval.end_tt == duration;
}

// A main-only candidate is safe under an allow-unknown mission only when the
// complete executable trajectory is independently known-free.  A candidate
// with a backup suffix may use the mission policy for MAIN, while BACKUP is
// always tightened by validateExecutableCandidate().
inline navigation_world_model::UnknownPolicy candidateCertificatePolicy(
        const CandidateCommandBundle& candidate,
        const navigation_world_model::UnknownPolicy mission_policy) {
    return candidateHasBackupSuffix(candidate)
        ? mission_policy
        : navigation_world_model::UnknownPolicy::kRequireKnownFree;
}

}  // namespace navigation_planning_backend
