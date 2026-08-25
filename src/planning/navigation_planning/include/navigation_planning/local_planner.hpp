#pragma once

#include <navigation_planning/planning_outcome.hpp>
#include <navigation_planning/planning_request.hpp>

namespace navigation_planning {

class LocalPlanner {
 public:
  virtual ~LocalPlanner() = default;

  [[nodiscard]] virtual PlanningOutcome solve(const PlanningRequest& request) = 0;
};

}  // namespace navigation_planning
