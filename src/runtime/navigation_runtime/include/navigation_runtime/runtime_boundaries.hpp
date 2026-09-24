#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string_view>
#include <utility>

#include <navigation_contracts/msg/navigation_goal.hpp>
#include <navigation_execution/execution_authority.hpp>
#include <navigation_execution/execution_state_lease.hpp>
#include <navigation_planning/planning_timing.hpp>
#include "navigation_runtime/planning_key.hpp"

namespace navigation_runtime {

// This is an observability contract for the two identities involved in a
// retarget.  The desired mission may advance before its successor becomes the
// executable command; callers must not infer activation from the desired goal.
enum class GoalTransitionKind : std::uint8_t {
  kSteady,
  kInitialGoal,
  kSameRouteWaypointAdvance,
  kRouteReplacement,
  kMissionReplacement,
  kCancelOrLocalizationReset,
};

[[nodiscard]] inline bool sameGoalIdentity(
    const std::optional<navigation_contracts::msg::NavigationGoal>& lhs,
    const std::optional<navigation_contracts::msg::NavigationGoal>& rhs) noexcept {
  return lhs && rhs && lhs->mission_id == rhs->mission_id &&
         lhs->route.route_revision == rhs->route.route_revision &&
         lhs->waypoint_index == rhs->waypoint_index &&
         lhs->request_id == rhs->request_id;
}

// An in-flight sample belongs to the executing trajectory, not the desired
// mission gate. During a PASS_THROUGH handoff the desired goal may advance
// while the predecessor remains the certified execution owner. The caller
// still checks the exact bundle pointer, world, state and finite leases at
// the final publication transaction.
[[nodiscard]] inline bool sameExecutionPublicationIdentity(
    const std::optional<navigation_contracts::msg::NavigationGoal>& sampled_execution,
    const std::optional<navigation_contracts::msg::NavigationGoal>& current_execution,
    std::uint64_t sampled_command_goal_epoch,
    std::uint64_t current_command_goal_epoch,
    std::uint64_t sampled_localization_epoch,
    std::uint64_t current_localization_epoch) noexcept {
  return sampled_command_goal_epoch != 0U &&
         sampled_command_goal_epoch == current_command_goal_epoch &&
         sampled_localization_epoch != 0U &&
         sampled_localization_epoch == current_localization_epoch &&
         sameGoalIdentity(sampled_execution, current_execution);
}

[[nodiscard]] constexpr const char* goalTransitionKindName(
    const GoalTransitionKind kind) noexcept {
  switch (kind) {
    case GoalTransitionKind::kSteady: return "steady";
    case GoalTransitionKind::kInitialGoal: return "initial_goal";
    case GoalTransitionKind::kSameRouteWaypointAdvance:
      return "same_route_waypoint_advance";
    case GoalTransitionKind::kRouteReplacement: return "route_replacement";
    case GoalTransitionKind::kMissionReplacement: return "mission_replacement";
    case GoalTransitionKind::kCancelOrLocalizationReset:
      return "cancel_or_localization_reset";
  }
  return "unknown";
}

[[nodiscard]] inline GoalTransitionKind classifyGoalTransition(
    const std::optional<navigation_contracts::msg::NavigationGoal>& desired,
    const std::optional<navigation_contracts::msg::NavigationGoal>& executing) noexcept {
  if (!desired) {
    return executing ? GoalTransitionKind::kCancelOrLocalizationReset
                     : GoalTransitionKind::kSteady;
  }
  if (!executing) return GoalTransitionKind::kInitialGoal;
  if (desired->mission_id != executing->mission_id) {
    return GoalTransitionKind::kMissionReplacement;
  }
  if (sameGoalIdentity(desired, executing)) {
    return GoalTransitionKind::kSteady;
  }
  if (desired->route.route_revision == executing->route.route_revision &&
      desired->waypoint_index != executing->waypoint_index) {
    return GoalTransitionKind::kSameRouteWaypointAdvance;
  }
  if (desired->route.route_revision != executing->route.route_revision ||
      desired->waypoint_index == executing->waypoint_index ||
      desired->request_id != executing->request_id) {
    return GoalTransitionKind::kRouteReplacement;
  }
  return GoalTransitionKind::kSteady;
}

// Diagnostic-only, one-shot fault selection. This is a predicate over the
// immutable solve context; it grants no command or retained-command authority.
// HG-023 remains the sole validator after the real result classifier runs.
struct ExactOptimizationFailureTarget final {
  std::uint64_t predecessor_request{0};
  std::uint64_t successor_request{0};

  [[nodiscard]] bool valid() const noexcept {
    return predecessor_request != 0 && successor_request != 0 &&
           predecessor_request != successor_request;
  }
};

[[nodiscard]] inline bool exactOptimizationFailureHotHandoffEligible(
    const ExactOptimizationFailureTarget target,
    const PlanningKey& key,
    const navigation_contracts::msg::NavigationGoal& desired,
    const navigation_execution::ExecutionAuthoritySnapshot& execution,
    const GoalTransitionKind transition,
    const bool desired_current,
    const bool execution_current,
    const bool state_fresh,
    const bool world_fresh,
    const std::int64_t now_ns) noexcept {
  return target.valid() && desired_current && execution_current && state_fresh &&
         world_fresh && now_ns > 0 && key.valid() &&
         key.start_mode == PlanningStartMode::kCommittedFutureState &&
         transition == GoalTransitionKind::kSameRouteWaypointAdvance &&
         desired.request_id == target.successor_request &&
         desired.behavior == desired.BEHAVIOR_PASS_THROUGH &&
         key.request_id == desired.request_id &&
         key.route_revision == desired.route.route_revision &&
         key.goal_epoch == execution.admission_goal_epoch &&
         key.localization_epoch == execution.admission_localization_epoch &&
         execution.active && execution.active_goal &&
         execution.activeGoalEpoch() != key.goal_epoch &&
         execution.activeRequestId() == target.predecessor_request &&
         execution.active_goal->mission_id == desired.mission_id &&
         execution.active_goal->route.route_revision == key.route_revision &&
         execution.active_goal->waypoint_index < desired.waypoint_index &&
         execution.active->localization_epoch == key.localization_epoch &&
         execution.activeGeneration() == key.committed_bundle_generation &&
         execution.active->role == navigation_planning::CandidateRole::kMain &&
         execution.active->backup_available && !execution.active->terminal_stop &&
         execution.active->valid() && execution.active->valid_from_ns <= now_ns &&
         now_ns <= execution.active->valid_until_ns &&
         execution.commandAvailable() && !execution.failed() &&
         !execution.safetySuffixActive() && !execution.restartFromRest();
}

class ExactOptimizationFailureInjection final {
 public:
  void setTarget(const ExactOptimizationFailureTarget target) noexcept {
    target_ = target.valid() ? target : ExactOptimizationFailureTarget{};
    consumed_ = false;
  }
  [[nodiscard]] bool armed() const noexcept { return target_.valid() && !consumed_; }
  [[nodiscard]] bool consumed() const noexcept { return consumed_; }
  [[nodiscard]] ExactOptimizationFailureTarget target() const noexcept { return target_; }
  [[nodiscard]] bool consumeIfEligible(const bool eligible) noexcept {
    if (!armed() || !eligible) return false;
    consumed_ = true;
    return true;
  }

 private:
  ExactOptimizationFailureTarget target_{};
  bool consumed_{false};
};

inline std::optional<std::uint64_t> advanceMonotonicId(
    std::atomic_uint64_t& value) noexcept {
  auto current = value.load(std::memory_order_acquire);
  while (current != std::numeric_limits<std::uint64_t>::max()) {
    const auto next = current + 1U;
    if (value.compare_exchange_weak(
            current, next, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      return next;
    }
  }
  return std::nullopt;
}

// Ephemeral solve provenance, never an authority mirror. The watchdog may
// cancel this solve, but may mutate execution only while this exact desired,
// pending, world and execution context is still current.
struct PlannerSolveFailureWitness final {
  PlanningKey key;
  navigation_execution::ExecutionAuthoritySnapshot execution;
  navigation_contracts::msg::NavigationGoal::ConstSharedPtr pending_goal;
};

// Pointer identity is the immutable ingress witness. A later lease with a
// newer source timestamp cannot be revoked by a callback that examined L1.
[[nodiscard]] inline bool failedExecutionLeaseIsCurrent(
    const std::shared_ptr<const navigation_execution::ExecutionStateLease>& failed,
    const std::shared_ptr<const navigation_execution::ExecutionStateLease>& current)
    noexcept {
  return failed.get() == current.get();
}

// Own the watchdog marker for exactly the backend solve scope.  A late scope
// destructor must not clear a newer solve that has reused the same node.
class PlannerSolveActivityScope final {
 public:
  PlannerSolveActivityScope(
      std::mutex& activity_mutex,
      std::int64_t& started_steady_ns,
      std::uint64_t& active_generation,
      const std::uint64_t generation,
      const std::int64_t started_ns,
      std::optional<PlannerSolveFailureWitness>* witness_slot = nullptr,
      std::optional<PlannerSolveFailureWitness> witness = std::nullopt) noexcept
      : activity_mutex_(activity_mutex),
        started_steady_ns_(started_steady_ns),
        active_generation_(active_generation),
        generation_(generation),
        witness_slot_(witness_slot) {
    std::lock_guard<std::mutex> lock(activity_mutex_);
    started_steady_ns_ = started_ns;
    active_generation_ = generation_;
    if (witness_slot_) *witness_slot_ = std::move(witness);
  }

  PlannerSolveActivityScope(const PlannerSolveActivityScope&) = delete;
  PlannerSolveActivityScope& operator=(const PlannerSolveActivityScope&) = delete;

  ~PlannerSolveActivityScope() noexcept {
    std::lock_guard<std::mutex> lock(activity_mutex_);
    if (active_generation_ == generation_) {
      active_generation_ = 0U;
      started_steady_ns_ = 0;
      if (witness_slot_) witness_slot_->reset();
    }
  }

 private:
  std::mutex& activity_mutex_;
  std::int64_t& started_steady_ns_;
  std::uint64_t& active_generation_;
  std::uint64_t generation_;
  std::optional<PlannerSolveFailureWitness>* witness_slot_;
};

inline std::optional<std::chrono::nanoseconds> ratePeriodNanoseconds(
    const double rate_hz) noexcept {
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) return std::nullopt;
  const long double period_ns = 1.0e9L / static_cast<long double>(rate_hz);
  if (!std::isfinite(period_ns) || period_ns < 1.0L ||
      period_ns > static_cast<long double>(
                      std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  return std::chrono::nanoseconds{static_cast<std::int64_t>(period_ns)};
}

inline bool plannerPeriodCoversSolveBudget(
    const double planner_rate_hz, const double solve_deadline_s) noexcept {
  if (!std::isfinite(planner_rate_hz) || planner_rate_hz <= 0.0 ||
      !std::isfinite(solve_deadline_s) || solve_deadline_s <= 0.0) {
    return false;
  }
  return solve_deadline_s < 1.0 / planner_rate_hz;
}

inline bool plannerSolveDeadlineMatchesContract(
    const double solve_deadline_s) noexcept {
  return std::isfinite(solve_deadline_s) &&
         std::abs(solve_deadline_s -
                  navigation_planning::PlanningTimingContract::kSolveDeadlineS) <=
             1.0e-9;
}

inline std::optional<std::size_t> boundedTrajectorySampleCount(
    const double duration_s, const double sample_period_s,
    const std::size_t maximum_points) noexcept {
  if (!std::isfinite(duration_s) || duration_s <= 0.0 ||
      !std::isfinite(sample_period_s) || sample_period_s <= 0.0 ||
      maximum_points < 2U) {
    return std::nullopt;
  }
  const long double ratio = static_cast<long double>(duration_s) /
                            static_cast<long double>(sample_period_s);
  const long double cap = static_cast<long double>(maximum_points - 1U);
  if (!std::isfinite(ratio) || ratio >= cap) return maximum_points;
  const long double intervals = std::ceil(ratio);
  if (!std::isfinite(intervals) || intervals < 1.0L || intervals > cap) {
    return std::nullopt;
  }
  if (intervals == cap) return maximum_points;
  return static_cast<std::size_t>(intervals) + 1U;
}

}  // namespace navigation_runtime
