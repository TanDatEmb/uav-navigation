#pragma once

#include <cstdint>

#include "navigation_runtime/planning_key.hpp"

namespace navigation_runtime {

enum class PlanningPriority : std::uint8_t {
  kQualityRefinement = 0,
  kNormalRenewal = 1,
  kSafetyRenewal = 2,
  kGoalTransition = 3,
  kEmergency = 4,
  kLocalizationReset = 5,
};

[[nodiscard]] constexpr bool planningPriorityKnown(
    PlanningPriority priority) noexcept {
  return priority >= PlanningPriority::kQualityRefinement &&
         priority <= PlanningPriority::kLocalizationReset;
}

[[nodiscard]] constexpr bool higherPriority(
    PlanningPriority lhs, PlanningPriority rhs) noexcept {
  return static_cast<std::uint8_t>(lhs) > static_cast<std::uint8_t>(rhs);
}

// Defines the changes that supersede an active solve. A newer odometry anchor
// or map revision is intentionally omitted: both are checked again by the
// commit boundary, and cancelling for either would starve a bounded solve on
// normal 50 Hz state and 10 Hz map publication.
[[nodiscard]] constexpr bool samePlanningCancellationIdentity(
    const PlanningKey& lhs, const PlanningKey& rhs) noexcept {
  return lhs.localization_epoch == rhs.localization_epoch &&
         lhs.goal_epoch == rhs.goal_epoch &&
         lhs.request_id == rhs.request_id &&
         lhs.route_revision == rhs.route_revision &&
         lhs.committed_bundle_generation == rhs.committed_bundle_generation &&
         lhs.pinned_world_generation == rhs.pinned_world_generation &&
         lhs.start_mode == rhs.start_mode &&
         lhs.dynamics_hash == rhs.dynamics_hash;
}

class PlanningSupervisor {
 public:
  [[nodiscard]] static constexpr PlanningPriority classifyPriority(
      bool localization_reset, bool emergency, bool goal_transition,
      bool safety_renewal) noexcept {
    if (localization_reset) return PlanningPriority::kLocalizationReset;
    if (emergency) return PlanningPriority::kEmergency;
    if (goal_transition) return PlanningPriority::kGoalTransition;
    if (safety_renewal) return PlanningPriority::kSafetyRenewal;
    return PlanningPriority::kNormalRenewal;
  }

  // A result may be recertified across a revision of the same immutable world
  // generation. Every ownership identity and committed-generation dependency
  // must still match exactly before the result can reach atomic commit.
  [[nodiscard]] static constexpr bool resultStillCurrent(
      const PlanningKey& request, const PlanningKey& current) noexcept {
    return samePlanningCancellationIdentity(request, current) &&
           request.committed_bundle_generation ==
               current.committed_bundle_generation &&
           request.pinned_world_generation == current.pinned_world_generation;
  }
};

}  // namespace navigation_runtime
