#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace super_planner {

class AbsoluteDeadline {
public:
    AbsoluteDeadline(const double start_time, const double budget_seconds)
        : deadline_(start_time + budget_seconds) {
        if (!std::isfinite(start_time) || !std::isfinite(budget_seconds) ||
            budget_seconds <= 0.0 || !std::isfinite(deadline_)) {
            throw std::invalid_argument("absolute deadline requires finite positive budget");
        }
    }

    [[nodiscard]] double remaining(const double now) const {
        if (!std::isfinite(now)) return 0.0;
        return std::max(0.0, deadline_ - now);
    }

    [[nodiscard]] bool expired(const double now) const {
        return remaining(now) <= 0.0;
    }

private:
    double deadline_;
};

}  // namespace super_planner
