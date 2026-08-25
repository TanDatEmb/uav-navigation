#pragma once

#include <cstdint>

namespace navigation_planning {

enum class PlanningStatus : std::uint8_t {
  kSuccess,
  kNoNeed,
  kNoPath,
  kFrontier,
  kInvalidRequest,
  kCancelled,
  kDeadline,
  kFailed,
};

[[nodiscard]] constexpr bool planningSucceeded(PlanningStatus status) noexcept {
  return status == PlanningStatus::kSuccess || status == PlanningStatus::kNoNeed ||
         status == PlanningStatus::kFrontier;
}

}  // namespace navigation_planning
