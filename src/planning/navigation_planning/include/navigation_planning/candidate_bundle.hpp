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
           world_identity.localization_epoch == localization_epoch &&
           world_identity.generation != 0 && world_identity.revision != 0;
  }

  [[nodiscard]] bool hasTrajectoryMetadata() const noexcept {
    return std::isfinite(start_wall_time_s) && start_wall_time_s > 0.0 &&
           std::isfinite(duration_s) && duration_s >= 0.0 &&
           std::isfinite(backup_start_time_s) && backup_start_time_s >= 0.0 &&
           backup_start_time_s <= duration_s + 1.0e-9;
  }

  [[nodiscard]] std::optional<TrajectoryPoint> sample(std::int64_t stamp_ns) const {
    if (!valid() || stamp_ns < valid_from_ns || stamp_ns > valid_until_ns) {
      return std::nullopt;
    }
    TrajectoryPoint point;
    point.role = role;
    if (!evaluator(stamp_ns, point) || !point.finite()) return std::nullopt;
    return point;
  }
};

}  // namespace navigation_planning
