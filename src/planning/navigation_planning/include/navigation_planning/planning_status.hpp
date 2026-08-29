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

[[nodiscard]] constexpr bool planningStatusKnown(PlanningStatus status) noexcept {
  switch (status) {
    case PlanningStatus::kSuccess:
    case PlanningStatus::kNoNeed:
    case PlanningStatus::kNoPath:
    case PlanningStatus::kFrontier:
    case PlanningStatus::kInvalidRequest:
    case PlanningStatus::kCancelled:
    case PlanningStatus::kDeadline:
    case PlanningStatus::kFailed:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr bool planningSucceeded(PlanningStatus status) noexcept {
  return status == PlanningStatus::kSuccess || status == PlanningStatus::kNoNeed ||
         status == PlanningStatus::kFrontier;
}

}  // namespace navigation_planning
