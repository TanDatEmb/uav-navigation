#pragma once

#include <cstdint>
#include <optional>

#include <navigation_planning/candidate_bundle.hpp>
#include <navigation_planning/planning_status.hpp>

namespace navigation_planning {

struct PlanningTrace {
  std::uint32_t expanded_nodes{0};
  std::uint32_t optimizer_attempts{0};
  std::int64_t elapsed_steady_ns{0};
};

struct PlanningOutcome {
  PlanningStatus status{PlanningStatus::kInvalidRequest};
  std::optional<CandidateBundle> candidate;
  PlanningTrace trace;

  [[nodiscard]] bool valid() const noexcept {
    return planningStatusKnown(status) && trace.elapsed_steady_ns >= 0 &&
           planningSucceeded(status) == candidate.has_value() &&
           (!candidate || candidate->valid());
  }
};

}  // namespace navigation_planning
