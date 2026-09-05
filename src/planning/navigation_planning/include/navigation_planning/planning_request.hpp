#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>

#include <navigation_planning/kinematic_state.hpp>
#include <navigation_planning/execution_anchor.hpp>
#include <navigation_planning/planning_budget.hpp>
#include <navigation_planning/planning_limits.hpp>
#include <navigation_mission/route_progress.hpp>
#include <navigation_world_model/world_model_view.hpp>

namespace navigation_planning {

enum class PlanningStartMode : std::uint8_t {
  kCommittedFutureState,
  kStoppedMeasuredState,
  kMeasuredEmergencyBrake,
};

[[nodiscard]] constexpr bool planningStartModeKnown(
    PlanningStartMode mode) noexcept {
  return mode == PlanningStartMode::kCommittedFutureState ||
         mode == PlanningStartMode::kStoppedMeasuredState ||
         mode == PlanningStartMode::kMeasuredEmergencyBrake;
}

struct PlanningKey {
  std::uint64_t localization_epoch{0};
  std::uint64_t goal_epoch{0};
  std::uint64_t request_id{0};
  std::uint64_t route_revision{0};
  std::uint64_t committed_bundle_generation{0};
  std::uint64_t pinned_world_generation{0};
  std::uint64_t pinned_world_revision{0};
  PlanningStartMode start_mode{PlanningStartMode::kStoppedMeasuredState};
  std::int64_t anchor_stamp_ns{0};
  std::uint64_t dynamics_hash{0};

  [[nodiscard]] bool valid() const noexcept {
    return localization_epoch != 0 && goal_epoch != 0 && request_id != 0 &&
           route_revision != 0 && pinned_world_generation != 0 &&
           pinned_world_revision != 0 && planningStartModeKnown(start_mode) &&
           anchor_stamp_ns > 0 && dynamics_hash != 0;
  }

  friend bool operator==(const PlanningKey&, const PlanningKey&) = default;
};

struct GoalIdentity {
  std::uint64_t localization_epoch{0};
  std::uint64_t goal_epoch{0};
  std::string mission_id;
  std::uint32_t waypoint_index{0};
  std::uint64_t request_id{0};

  [[nodiscard]] bool valid() const noexcept {
    return localization_epoch != 0 && goal_epoch != 0 && !mission_id.empty() && request_id != 0;
  }
};

struct PlanningHistory {
  std::uint64_t previous_bundle_generation{0};
  Eigen::Vector3d previous_velocity_world{Eigen::Vector3d::Zero()};

  // A zero generation means that there is no prior executable bundle. In
  // that case the velocity field must not be interpreted as prior-command
  // history; the measured start state is carried separately by the request.
  [[nodiscard]] bool valid() const noexcept {
    return previous_velocity_world.allFinite() &&
           (previous_bundle_generation != 0 ||
            previous_velocity_world.isZero(1.0e-12));
  }
};

struct PlanningRequest {
  PlanningKey key;
  GoalIdentity goal;
  std::optional<ExecutionAnchor> anchor;
  std::int64_t activation_stamp_ns{0};
  KinematicState start_state;
  PlanningHistory history;
  // The route is part of the immutable solve snapshot. A planner solve must
  // not read a separately mutable route member after this request is queued.
  navigation_mission::ImmutableRouteSnapshot route_snapshot;
  navigation_world_model::WorldModelViewPtr world;
  DynamicLimits dynamics;
  PlanningBudget budget;

  [[nodiscard]] bool startModeContractValid() const noexcept {
    const bool successor_anchor_valid =
        key.start_mode != PlanningStartMode::kCommittedFutureState ||
        (anchor.has_value() && anchor->valid() && activation_stamp_ns > 0 &&
         anchor->activation_stamp_ns == activation_stamp_ns &&
         anchor->active_bundle_generation == key.committed_bundle_generation &&
         anchor->localization_epoch == key.localization_epoch &&
         anchor->command_world.generation == key.pinned_world_generation &&
         anchor->command_world.revision == key.pinned_world_revision);
    const bool stopped_activation_valid =
        key.start_mode == PlanningStartMode::kCommittedFutureState
            ? activation_stamp_ns > key.anchor_stamp_ns
            : !anchor.has_value() && activation_stamp_ns == 0;
    return successor_anchor_valid && stopped_activation_valid;
  }

  [[nodiscard]] bool valid() const noexcept {
    const bool route_contract_valid =
        route_snapshot.valid() &&
        route_snapshot.mission_id == goal.mission_id &&
        route_snapshot.route_revision == key.route_revision &&
        route_snapshot.request_id == goal.request_id &&
        route_snapshot.active_waypoint_index == goal.waypoint_index;
    return key.valid() && goal.valid() && route_contract_valid &&
           key.localization_epoch == goal.localization_epoch &&
           key.goal_epoch == goal.goal_epoch && key.request_id == goal.request_id &&
           key.anchor_stamp_ns == start_state.source_stamp_ns &&
           start_state.finite() &&
           static_cast<bool>(world) && world->identity().localization_epoch ==
               goal.localization_epoch &&
           world->identity().generation == key.pinned_world_generation &&
           world->identity().revision == key.pinned_world_revision &&
           world->identity().observation_stamp_ns > 0 &&
           history.valid() &&
           dynamics.valid() &&
           !budget.exhausted();
  }
};

}  // namespace navigation_planning
