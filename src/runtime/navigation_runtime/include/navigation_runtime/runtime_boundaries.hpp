#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <string_view>

#include <navigation_contracts/msg/navigation_goal.hpp>

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

// Own the watchdog marker for exactly the backend solve scope.  A late scope
// destructor must not clear a newer solve that has reused the same node.
class PlannerSolveActivityScope final {
 public:
  PlannerSolveActivityScope(
      std::mutex& activity_mutex,
      std::int64_t& started_steady_ns,
      std::uint64_t& active_generation,
      const std::uint64_t generation,
      const std::int64_t started_ns) noexcept
      : activity_mutex_(activity_mutex),
        started_steady_ns_(started_steady_ns),
        active_generation_(active_generation),
        generation_(generation) {
    std::lock_guard<std::mutex> lock(activity_mutex_);
    started_steady_ns_ = started_ns;
    active_generation_ = generation_;
  }

  PlannerSolveActivityScope(const PlannerSolveActivityScope&) = delete;
  PlannerSolveActivityScope& operator=(const PlannerSolveActivityScope&) = delete;

  ~PlannerSolveActivityScope() noexcept {
    std::lock_guard<std::mutex> lock(activity_mutex_);
    if (active_generation_ == generation_) {
      active_generation_ = 0U;
      started_steady_ns_ = 0;
    }
  }

 private:
  std::mutex& activity_mutex_;
  std::int64_t& started_steady_ns_;
  std::uint64_t& active_generation_;
  std::uint64_t generation_;
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
