#pragma once

#include <chrono>
#include <cstdint>
#include <stop_token>

namespace navigation_planning {

struct PlanningBudget {
  using Clock = std::chrono::steady_clock;

  Clock::time_point deadline{};
  // Absolute monotonic deadline copied from the runtime transaction. Zero
  // keeps compatibility callers on the legacy local-budget path.
  std::int64_t steady_deadline_ns{0};
  std::stop_token cancellation{};

  [[nodiscard]] bool cancelled() const noexcept { return cancellation.stop_requested(); }
  [[nodiscard]] bool expired() const noexcept {
    return deadline != Clock::time_point{} && Clock::now() >= deadline;
  }
  [[nodiscard]] bool exhausted() const noexcept { return cancelled() || expired(); }
  [[nodiscard]] bool steadyDeadlineValid() const noexcept {
    return steady_deadline_ns > 0;
  }
};

}  // namespace navigation_planning
