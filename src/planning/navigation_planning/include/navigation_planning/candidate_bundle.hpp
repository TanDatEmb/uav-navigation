#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
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

  [[nodiscard]] bool finite() const noexcept {
    return position_world.allFinite() && velocity_world.allFinite() &&
           acceleration_world.allFinite() && jerk_world.allFinite() &&
           std::isfinite(yaw) && std::isfinite(yaw_rate);
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
  std::function<bool(std::int64_t, TrajectoryPoint&)> evaluator;

  [[nodiscard]] bool valid() const noexcept {
    return localization_epoch != 0 && goal_epoch != 0 && request_id != 0 &&
           bundle_generation != 0 && valid_from_ns > 0 &&
           valid_until_ns >= valid_from_ns && static_cast<bool>(evaluator) &&
           world_identity.localization_epoch == localization_epoch &&
           world_identity.generation != 0 && world_identity.revision != 0;
  }

  [[nodiscard]] std::optional<TrajectoryPoint> sample(std::int64_t stamp_ns) const {
    if (!valid() || stamp_ns < valid_from_ns || stamp_ns > valid_until_ns) {
      return std::nullopt;
    }
    TrajectoryPoint point;
    if (!evaluator(stamp_ns, point) || !point.finite()) return std::nullopt;
    return point;
  }
};

}  // namespace navigation_planning
