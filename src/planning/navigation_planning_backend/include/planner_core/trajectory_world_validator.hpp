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
};

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
        const double resolution_m) noexcept {
    if (!start.allFinite() || !end.allFinite() ||
        !std::isfinite(curve_deviation_m) || curve_deviation_m < 0.0 ||
        !std::isfinite(resolution_m) || resolution_m <= 0.0) {
        return false;
    }

    // The polynomial arc is within curve_deviation_m of its chord. Expand by
    // half a voxel diagonal so every voxel touched by that tube has its center
    // represented by the conservative cell query below. The distance filter
    // below is important: checking the complete axis-aligned box would reject
    // occupied cells near a diagonal box corner that the trajectory cannot
    // touch.
    const double cell_half_diagonal = 0.5 * std::sqrt(3.0) * resolution_m;
    const double expansion = curve_deviation_m +
        cell_half_diagonal;
    const double tube_radius_squared = expansion * expansion;
    if (!std::isfinite(cell_half_diagonal) || !std::isfinite(expansion) ||
        !std::isfinite(tube_radius_squared)) return false;
    const auto segment = end - start;
    const double segment_length_squared = segment.squaredNorm();
    if (!std::isfinite(segment_length_squared)) return false;
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
                const auto from_start = cell_center - start;
                const double projection = segment_length_squared > 0.0
                    ? std::clamp(from_start.dot(segment) / segment_length_squared, 0.0, 1.0)
                    : 0.0;
                const auto nearest_point = start + projection * segment;
                const double distance_squared =
                    (cell_center - nearest_point).squaredNorm();
                if (!std::isfinite(distance_squared) ||
                    distance_squared > tube_radius_squared) {
                    if (z == upper[2]) break;
                    continue;
                }
                const auto state = world.classify(
                    cell_center, navigation_world_model::GridLayer::kInflated);
                if (!navigation_world_model::isCellTraversable(state, unknown_policy)) {
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
        return result;
    }
    const double end_wall_time = candidate.start_wall_time + duration;
    if (!std::isfinite(end_wall_time) || authorization_wall_time > end_wall_time) {
        // A finished candidate cannot be made safe by clamping validation to
        // its terminal point. It has no remaining executable horizon.
        return result;
    }
    result.begin_tt = std::clamp(
        authorization_wall_time - candidate.start_wall_time, 0.0, duration);
    result.first_blocked_tt = result.begin_tt;
    if (!std::isfinite(result.begin_tt)) return result;

    if (!candidateRoleScheduleIsComplete(candidate)) return result;

    const auto geometry = world.geometry();
    if (!std::isfinite(geometry.inflated_resolution_m) ||
        geometry.inflated_resolution_m <= 0.0) return result;
    const double spatial_step = 0.5 * geometry.inflated_resolution_m;
    const double curve_deviation_tolerance =
        kMaximumCurveDeviationFraction * geometry.inflated_resolution_m;
    if (!std::isfinite(spatial_step) || spatial_step <= 0.0 ||
        !std::isfinite(curve_deviation_tolerance) || curve_deviation_tolerance <= 0.0) {
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
    if (!previous.allFinite()) return result;

    const auto include_protected_segment =
        [&result, resolution = geometry.inflated_resolution_m](
            const navigation_world_model::Point3& start,
            const navigation_world_model::Point3& end,
            const double curve_deviation_m) {
          const double cell_half_diagonal = 0.5 * std::sqrt(3.0) * resolution;
          const double expansion = curve_deviation_m + cell_half_diagonal;
          if (!start.allFinite() || !end.allFinite() ||
              !std::isfinite(expansion) || expansion < 0.0) {
            result.protected_region.minimum =
                navigation_world_model::Point3::Constant(
                    std::numeric_limits<double>::quiet_NaN());
            result.protected_region.maximum = result.protected_region.minimum;
            return;
          }
          const auto minimum = start.cwiseMin(end).array() - expansion;
          const auto maximum = start.cwiseMax(end).array() + expansion;
          if (result.sample_count == 0U && result.segment_count == 0U) {
            result.protected_region.minimum = minimum.matrix();
            result.protected_region.maximum = maximum.matrix();
          } else {
            result.protected_region.minimum =
                result.protected_region.minimum.cwiseMin(minimum.matrix());
            result.protected_region.maximum =
                result.protected_region.maximum.cwiseMax(maximum.matrix());
          }
        };
    include_protected_segment(previous, previous, 0.0);

    const auto point_safe = [&world](const auto& point,
                                     const navigation_world_model::UnknownPolicy policy) {
        const auto state = world.classify(
            point, navigation_world_model::GridLayer::kInflated);
        return navigation_world_model::isCellTraversable(state, policy);
    };
    const auto initial_role = role_at(t);
    if (!initial_role || !point_safe(previous, policy_for_role(*initial_role))) return result;
    ++result.sample_count;

    while (t < duration) {
        const auto role = role_at(t);
        if (!role) return result;
        const auto segment_policy = policy_for_role(*role);
        double piece_local_t = t;
        const int piece_index = candidate.position.locatePieceIdx(piece_local_t);
        if (piece_index < 0 || piece_index >= candidate.position.getPieceNum()) return result;
        const auto& piece = candidate.position[piece_index];
        const double acceleration_bound = polynomialAccelerationBound(piece);
        if (!std::isfinite(acceleration_bound)) return result;
        const double speed = std::max(0.1, candidate.position.getVel(t).norm());
        double dt = std::clamp(spatial_step / speed, 0.002, 0.05);
        double next_t = std::min({duration, t + dt,
                                  t + piece.getDuration() - piece_local_t,
                                  next_role_boundary(t)});
        if (!(next_t > t)) return result;
        auto next = candidate.position.getPos(next_t);
        while (next.allFinite() && (next - previous).norm() > spatial_step &&
               dt > 0.002 + 1.0e-12) {
            dt = std::max(0.002, 0.5 * dt);
            next_t = std::min({duration, t + dt,
                               t + piece.getDuration() - piece_local_t,
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
                               t + piece.getDuration() - piece_local_t,
                               next_role_boundary(t)});
            if (!(next_t > t)) return result;
            next = candidate.position.getPos(next_t);
            segment_dt = next_t - t;
            curve_deviation_bound =
                acceleration_bound * segment_dt * segment_dt / 8.0;
        }
        if (!std::isfinite(curve_deviation_bound) ||
            curve_deviation_bound > curve_deviation_tolerance ||
            !certificateTubeIsSafe(world, previous, next, curve_deviation_bound,
                                   segment_policy, geometry.inflated_resolution_m)) {
            return result;
        }
        include_protected_segment(previous, next, curve_deviation_bound);
        result.first_blocked_tt = next_t;
        if (!next.allFinite() || !point_safe(next, segment_policy) ||
            !world.isSegmentTraversable(
                previous, next, navigation_world_model::GridLayer::kInflated,
                segment_policy)) {
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
    return std::any_of(candidate.roles.begin(), candidate.roles.end(), [duration](const auto& interval) {
        return interval.role == CandidateTrajectoryRole::BACKUP &&
               std::isfinite(interval.begin_tt) && std::isfinite(interval.end_tt) &&
               interval.end_tt > interval.begin_tt &&
               interval.end_tt == duration;
    });
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
