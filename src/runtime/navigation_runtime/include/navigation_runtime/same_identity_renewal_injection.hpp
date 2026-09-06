#pragma once

#include <cstdint>

#include <navigation_planning/planning_request.hpp>

#include "navigation_runtime/execution_episode.hpp"
#include "navigation_runtime/execution_recovery_state.hpp"
#include "navigation_runtime/runtime_boundaries.hpp"

namespace navigation_runtime {

// Facts captured at the ordinary renewal boundary.  This is deliberately a
// value object: the diagnostic hook must be admitted only from one coherent
// identity snapshot and must never infer eligibility from a scheduler cycle.
struct SameIdentityRenewalFacts final {
  navigation_planning::PlanningStartMode start_mode{
      navigation_planning::PlanningStartMode::kStoppedMeasuredState};
  GoalTransitionKind transition_kind{GoalTransitionKind::kCancelOrLocalizationReset};
  ExecutionRecoveryState recovery_state{ExecutionRecoveryState::kPx4Hold};
  ExecutionEpisodePhase execution_phase{ExecutionEpisodePhase::kInitialHold};
  bool desired_goal_valid{false};
  bool executing_goal_valid{false};
  bool desired_identity_matches_executing{false};
  bool goal_epoch_matches_command{false};
  bool active_bundle_valid{false};
  bool active_bundle_is_main{false};
  bool active_bundle_identity_current{false};
  bool no_new_goal{false};
  bool no_hot_goal_transition{false};
  bool ordinary_renewal{false};
  bool no_pending_successor{false};
  bool command_available{false};
  bool failure_latched{false};
  bool safety_suffix_active{false};
  bool restart_from_rest{false};
  bool command_exposure_allowed{false};
  bool execution_state_fresh{false};
  bool world_fresh{false};
  bool valid_future_anchor{false};
  bool current_body_support_present{false};
  bool terminal_hold_pending{false};
};

[[nodiscard]] inline bool sameIdentityRenewalInjectionEligible(
    const SameIdentityRenewalFacts& facts) noexcept {
  return facts.start_mode ==
             navigation_planning::PlanningStartMode::kCommittedFutureState &&
         facts.transition_kind == GoalTransitionKind::kSteady &&
         facts.recovery_state == ExecutionRecoveryState::kTrackMain &&
         facts.execution_phase == ExecutionEpisodePhase::kTrackingMain &&
         facts.desired_goal_valid && facts.executing_goal_valid &&
         facts.desired_identity_matches_executing &&
         facts.goal_epoch_matches_command && facts.active_bundle_valid &&
         facts.active_bundle_is_main && facts.active_bundle_identity_current &&
         facts.no_new_goal && facts.no_hot_goal_transition &&
         facts.ordinary_renewal &&
         facts.no_pending_successor && facts.command_available &&
         !facts.failure_latched && !facts.safety_suffix_active &&
         !facts.restart_from_rest && facts.command_exposure_allowed &&
         facts.execution_state_fresh && facts.world_fresh &&
         facts.valid_future_anchor && !facts.current_body_support_present &&
         !facts.terminal_hold_pending;
}

// Counts only eligible ordinary renewals.  The target ordinal is diagnostic
// configuration; zero leaves the counter observable but disables injection.
class SameIdentityRenewalInjectionController final {
 public:
  void setTargetOrdinal(const std::uint64_t ordinal) noexcept {
    target_ordinal_ = ordinal;
  }

  [[nodiscard]] std::uint64_t observe(
      const SameIdentityRenewalFacts& facts) noexcept {
    if (sameIdentityRenewalInjectionEligible(facts)) ++eligible_ordinal_;
    return eligible_ordinal_;
  }

  [[nodiscard]] bool shouldInject(const std::uint64_t observed_ordinal) const noexcept {
    return target_ordinal_ != 0U && !injection_fired_ &&
           observed_ordinal == target_ordinal_;
  }

  void markInjected() noexcept { injection_fired_ = true; }

  [[nodiscard]] std::uint64_t targetOrdinal() const noexcept {
    return target_ordinal_;
  }

  [[nodiscard]] std::uint64_t eligibleOrdinal() const noexcept {
    return eligible_ordinal_;
  }

  [[nodiscard]] bool injectionFired() const noexcept {
    return injection_fired_;
  }

 private:
  std::uint64_t target_ordinal_{0U};
  std::uint64_t eligible_ordinal_{0U};
  bool injection_fired_{false};
};

}  // namespace navigation_runtime
