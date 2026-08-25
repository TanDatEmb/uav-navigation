#pragma once

#include <chrono>
#include <stop_token>

namespace navigation_planning {

struct PlanningBudget {
  using Clock = std::chrono::steady_clock;

  Clock::time_point deadline{};
  std::stop_token cancellation{};

  [[nodiscard]] bool cancelled() const noexcept { return cancellation.stop_requested(); }
  [[nodiscard]] bool expired() const noexcept {
    return deadline != Clock::time_point{} && Clock::now() >= deadline;
  }
  [[nodiscard]] bool exhausted() const noexcept { return cancelled() || expired(); }
};

}  // namespace navigation_planning
