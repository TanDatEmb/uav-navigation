#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>

#include <Eigen/Core>

#include <navigation_planning/planning_request.hpp>

namespace navigation_planning {

enum class CandidateRole : std::uint8_t { kMain, kBackup, kEmergency };

[[nodiscard]] constexpr bool candidateRoleValid(CandidateRole role) noexcept {
  return role == CandidateRole::kMain || role == CandidateRole::kBackup ||
         role == CandidateRole::kEmergency;
}

struct TrajectoryPoint {
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk_world{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
  CandidateRole role{CandidateRole::kMain};
  bool finished{false};
  double trajectory_time_s{0.0};

  [[nodiscard]] bool finite() const noexcept {
    return position_world.allFinite() && velocity_world.allFinite() &&
           acceleration_world.allFinite() && jerk_world.allFinite() &&
           std::isfinite(yaw) && std::isfinite(yaw_rate) &&
           std::isfinite(trajectory_time_s);
  }
};

// A candidate owns no mutable planner state.  The evaluator is an immutable,
// product-owned callable supplied by a planner implementation and is never
// invoked while a world or commit mutex is held.
struct CandidateBundle {
  navigation_world_model::WorldSnapshotIdentity world_identity;
  std::uint64_t localization_epoch{0};
  std::uint64_t goal_epoch{0};
  std::uint64_t request_id{0};
  std::uint64_t bundle_generation{0};
  std::int64_t valid_from_ns{0};
  std::int64_t valid_until_ns{0};
  CandidateRole role{CandidateRole::kMain};
  // Optional trajectory metadata used by execution diagnostics and handover
  // checks. The evaluator remains the only source for a sampled point.
  double start_wall_time_s{std::numeric_limits<double>::quiet_NaN()};
  double duration_s{std::numeric_limits<double>::quiet_NaN()};
  double backup_start_time_s{std::numeric_limits<double>::quiet_NaN()};
  bool backup_available{false};
  std::function<bool(std::int64_t, TrajectoryPoint&)> evaluator;

  [[nodiscard]] bool valid() const noexcept {
    return localization_epoch != 0 && goal_epoch != 0 && request_id != 0 &&
           bundle_generation != 0 && valid_from_ns > 0 &&
           valid_until_ns >= valid_from_ns && static_cast<bool>(evaluator) &&
           candidateRoleValid(role) &&
           world_identity.localization_epoch == localization_epoch &&
           world_identity.generation != 0 && world_identity.revision != 0 &&
           world_identity.observation_stamp_ns > 0;
  }

  [[nodiscard]] bool hasTrajectoryMetadata() const noexcept {
    return std::isfinite(start_wall_time_s) && start_wall_time_s > 0.0 &&
           std::isfinite(duration_s) && duration_s >= 0.0 &&
           std::isfinite(backup_start_time_s) && backup_start_time_s >= 0.0 &&
           backup_start_time_s <= duration_s + 1.0e-9;
  }

  // The declared endpoint exists for every executable trajectory, including a
  // main-only candidate that has no backup suffix metadata. Keep this query
  // separate from hasTrajectoryMetadata(), which is also used by the
  // retained-backup safety path and therefore intentionally requires a valid
  // backup-start declaration.
  [[nodiscard]] bool hasDeclaredEndpointMetadata() const noexcept {
    return std::isfinite(start_wall_time_s) && start_wall_time_s > 0.0 &&
           std::isfinite(duration_s) && duration_s >= 0.0;
  }

  [[nodiscard]] std::optional<TrajectoryPoint> sample(std::int64_t stamp_ns) const {
    if (!valid() || stamp_ns < valid_from_ns || stamp_ns > valid_until_ns) {
      return std::nullopt;
    }
    TrajectoryPoint point;
    point.role = role;
    if (!evaluator(stamp_ns, point) || !point.finite() || point.role != role) {
      return std::nullopt;
    }
    return point;
  }

  // The declared trajectory endpoint can lie beyond the short execution lease
  // used for incremental replanning. This metadata query intentionally samples
  // the evaluator at the terminal trajectory timestamp without widening the
  // executable validity interval used by sample().
  [[nodiscard]] std::optional<TrajectoryPoint> sampleAtDeclaredEnd() const {
    if (!valid() || !hasDeclaredEndpointMetadata()) return std::nullopt;
    const double end_wall_time_s = start_wall_time_s + duration_s;
    if (!std::isfinite(end_wall_time_s) || end_wall_time_s <= 0.0 ||
        end_wall_time_s > static_cast<double>(std::numeric_limits<std::int64_t>::max()) * 1.0e-9) {
      return std::nullopt;
    }
    TrajectoryPoint point;
    point.role = role;
    const auto end_stamp_ns = static_cast<std::int64_t>(end_wall_time_s * 1.0e9);
    if (!evaluator(end_stamp_ns, point) || !point.finite() || point.role != role) {
      return std::nullopt;
    }
    // The evaluator's runtime completion predicate is intentionally strict
    // (it marks a sample finished only after the declared end). This API is
    // explicitly the declared endpoint, so normalize its lifecycle flag for
    // consumers performing terminal handover checks.
    point.finished = true;
    return point;
  }
};

}  // namespace navigation_planning
