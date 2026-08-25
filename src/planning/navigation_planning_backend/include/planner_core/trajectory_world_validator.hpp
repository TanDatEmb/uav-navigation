#pragma once

#include <algorithm>
#include <cmath>

#include <data_structure/cmd_traj.h>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_planning_backend {

struct SweptValidationResult {
    bool valid{false};
    double begin_tt{0.0};
    double first_blocked_tt{0.0};
    std::size_t sample_count{0};
    std::size_t segment_count{0};
};

// WM-3B preserves the current endpoint-only profile: UNKNOWN is traversable
// for both roles, while OCCUPIED and OUT_OF_MAP fail closed. BACKUP role
// provenance remains in the bundle and is not promoted to persisted-known-free
// until the product visibility-certificate behavior batch.
inline SweptValidationResult validateExecutableCandidate(
        const navigation_world_model::WorldModelView& world,
        const CandidateCommandBundle& candidate,
        double authorization_wall_time) {
    SweptValidationResult result;
    const double duration = candidate.position.getTotalDuration();
    result.begin_tt = std::clamp(
        authorization_wall_time - candidate.start_wall_time, 0.0, duration);
    result.first_blocked_tt = result.begin_tt;
    if (!std::isfinite(duration) || duration < 0.0 ||
        !std::isfinite(result.begin_tt)) return result;

    const auto geometry = world.geometry();
    if (!std::isfinite(geometry.inflated_resolution_m) ||
        geometry.inflated_resolution_m <= 0.0) return result;
    const double spatial_step = std::max(0.02, 0.5 * geometry.inflated_resolution_m);
    double t = result.begin_tt;
    auto previous = candidate.position.getPos(t);
    if (!previous.allFinite()) return result;

    const auto point_safe = [&world](const auto& point) {
        const auto state = world.classify(
            point, navigation_world_model::GridLayer::kInflated);
        return state != navigation_world_model::CellState::kOccupied &&
               state != navigation_world_model::CellState::kOutOfMap;
    };
    if (!point_safe(previous)) return result;
    ++result.sample_count;

    while (t < duration) {
        const double speed = std::max(0.1, candidate.position.getVel(t).norm());
        double dt = std::clamp(spatial_step / speed, 0.002, 0.05);
        double next_t = std::min(duration, t + dt);
        auto next = candidate.position.getPos(next_t);
        while (next.allFinite() && (next - previous).norm() > spatial_step &&
               dt > 0.002 + 1.0e-12) {
            dt = std::max(0.002, 0.5 * dt);
            next_t = std::min(duration, t + dt);
            next = candidate.position.getPos(next_t);
        }
        result.first_blocked_tt = next_t;
        if (!next.allFinite() || !point_safe(next) ||
            !world.isSegmentTraversable(
                previous, next, navigation_world_model::GridLayer::kInflated,
                navigation_world_model::UnknownPolicy::kAllowUnknown)) {
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

}  // namespace navigation_planning_backend
