#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace navigation_planning_backend {

class AbsoluteDeadline {
public:
    AbsoluteDeadline(const double start_time, const double budget_seconds)
        : deadline_(makeSimulationDeadline(start_time, budget_seconds)),
          steady_deadline_(makeSteadyDeadline(budget_seconds)) {}

    [[nodiscard]] double remaining(const double now) const {
        if (!std::isfinite(now)) return 0.0;
        return std::max(0.0, deadline_ - now);
    }

    [[nodiscard]] bool expired(const double now) const {
        return remaining(now) <= 0.0;
    }

    [[nodiscard]] double conservativeRemaining(const double now) const noexcept {
        if (!std::isfinite(now)) return 0.0;
        const double simulation_remaining = remaining(now);
        const auto steady_remaining = std::chrono::duration<double>(
            steady_deadline_ - std::chrono::steady_clock::now()).count();
        if (!std::isfinite(steady_remaining)) return 0.0;
        return std::max(0.0, std::min(simulation_remaining, steady_remaining));
    }

    // Solver cancellation must not depend on ROS/simulation time.  During a
    // Gazebo or bridge stall ROS time may stop advancing while the optimizer
    // continues to consume CPU and holds the planning callback.  Keep the
    // original simulation-time contract for trajectory timestamps, but expose
    // a monotonic steady-clock budget for cancellation.
    [[nodiscard]] bool steadyExpired() const noexcept {
        return std::chrono::steady_clock::now() >= steady_deadline_;
    }

    [[nodiscard]] std::int64_t steadyDeadlineNanoseconds() const noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   steady_deadline_.time_since_epoch()).count();
    }

private:
    static double makeSimulationDeadline(const double start_time,
                                         const double budget_seconds) {
        if (!std::isfinite(start_time) || !std::isfinite(budget_seconds) ||
            budget_seconds <= 0.0 ||
            !std::isfinite(start_time + budget_seconds)) {
            throw std::invalid_argument("absolute deadline requires finite positive budget");
        }
        return start_time + budget_seconds;
    }

    static std::chrono::steady_clock::time_point makeSteadyDeadline(
        const double budget_seconds) {
        if (!std::isfinite(budget_seconds) || budget_seconds <= 0.0) {
            throw std::invalid_argument("absolute deadline requires finite positive budget");
        }
        using Duration = std::chrono::steady_clock::duration;
        const auto now = std::chrono::steady_clock::now();
        const long double ticks =
            static_cast<long double>(budget_seconds) *
            static_cast<long double>(std::chrono::steady_clock::period::den) /
            static_cast<long double>(std::chrono::steady_clock::period::num);
        const long double available =
            static_cast<long double>(Duration::max().count()) -
            static_cast<long double>(now.time_since_epoch().count());
        if (!std::isfinite(ticks) || ticks > available ||
            ticks > static_cast<long double>(Duration::max().count())) {
            throw std::invalid_argument("absolute deadline exceeds steady-clock range");
        }
        const auto duration = std::chrono::duration_cast<Duration>(
            std::chrono::duration<long double>(budget_seconds));
        return now + duration;
    }

    double deadline_;
    std::chrono::steady_clock::time_point steady_deadline_;
};

}  // namespace navigation_planning_backend
