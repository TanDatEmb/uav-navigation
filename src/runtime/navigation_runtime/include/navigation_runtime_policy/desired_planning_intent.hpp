#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

#include <navigation_contracts/msg/navigation_goal.hpp>

namespace navigation_runtime {

// The transition records why the current desired goal still needs planning
// handling. It is deliberately distinct from GoalTransitionKind, which
// classifies desired-versus-executing identity and can remain non-steady after
// this one-shot planning disposition has been consumed.
enum class PlanningIntentTransition : std::uint8_t {
  kNone,
  kNewIntent,
  kHotRetarget,
};

// A value held under the RuntimeNode input lock. The revision is atomic because
// existing lock-free readers observe the monotonic goal epoch; goal and
// transition access remain protected by the caller's existing lock.
class DesiredPlanningIntent final {
 public:
  using Goal = navigation_contracts::msg::NavigationGoal;

  DesiredPlanningIntent() = default;
  DesiredPlanningIntent(const DesiredPlanningIntent&) = delete;
  DesiredPlanningIntent& operator=(const DesiredPlanningIntent&) = delete;
  DesiredPlanningIntent(DesiredPlanningIntent&&) = delete;
  DesiredPlanningIntent& operator=(DesiredPlanningIntent&&) = delete;

  [[nodiscard]] const std::optional<Goal>& goal() const noexcept {
    return goal_;
  }

  [[nodiscard]] std::uint64_t revision() const noexcept {
    return revision_.load(std::memory_order_acquire);
  }

  // Advances exactly once, returning nullopt at exhaustion without wrapping.
  [[nodiscard]] std::optional<std::uint64_t> advanceRevision() noexcept {
    auto current = revision_.load(std::memory_order_acquire);
    while (current != std::numeric_limits<std::uint64_t>::max()) {
      const auto next = current + 1U;
      if (revision_.compare_exchange_weak(
              current, next, std::memory_order_acq_rel,
              std::memory_order_acquire)) {
        return next;
      }
    }
    return std::nullopt;
  }

  // Passing a transition replaces the current planning disposition. Omitting
  // it updates the goal payload while preserving the pending disposition.
  void install(
      Goal goal,
      std::optional<PlanningIntentTransition> transition = std::nullopt) {
    goal_ = std::move(goal);
    if (transition) transition_ = *transition;
  }

  void clearGoal() noexcept {
    goal_.reset();
    transition_ = PlanningIntentTransition::kNone;
  }

  void rearmAfterLocalizationReset() noexcept {
    transition_ = goal_ ? PlanningIntentTransition::kNewIntent
                        : PlanningIntentTransition::kNone;
  }

  [[nodiscard]] bool isNewIntent() const noexcept {
    return transition_ == PlanningIntentTransition::kNewIntent;
  }

  [[nodiscard]] bool isHotRetarget() const noexcept {
    return transition_ == PlanningIntentTransition::kHotRetarget;
  }

  [[nodiscard]] bool hasTransition() const noexcept {
    return transition_ != PlanningIntentTransition::kNone;
  }

  [[nodiscard]] PlanningIntentTransition transition() const noexcept {
    return transition_;
  }

  void clearTransition() noexcept {
    transition_ = PlanningIntentTransition::kNone;
  }

  [[nodiscard]] bool consumeNewIntent() noexcept {
    if (!isNewIntent()) return false;
    clearTransition();
    return true;
  }

  [[nodiscard]] bool consumeHotRetarget() noexcept {
    if (!isHotRetarget()) return false;
    clearTransition();
    return true;
  }

  // This identity intentionally excludes route_revision, matching the existing
  // desired-goal gate. Callers that need route identity check it separately.
  [[nodiscard]] bool matches(
      const Goal& goal, const std::uint64_t revision) const noexcept {
    const auto current_revision = revision_.load(std::memory_order_acquire);
    return goal_ && current_revision == revision &&
           goal_->mission_id == goal.mission_id &&
           goal_->waypoint_index == goal.waypoint_index &&
           goal_->request_id == goal.request_id;
  }

 private:
  std::optional<Goal> goal_;
  std::atomic_uint64_t revision_{0U};
  PlanningIntentTransition transition_{PlanningIntentTransition::kNone};
};

}  // namespace navigation_runtime
