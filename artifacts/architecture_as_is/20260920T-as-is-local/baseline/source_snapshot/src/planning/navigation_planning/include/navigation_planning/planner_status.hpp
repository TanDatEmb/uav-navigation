#pragma once

#include <cstdint>

namespace navigation_planning {

// Product-level result vocabulary. Backend-specific integer codes are mapped
// at the implementation boundary and must not cross into runtime policy code.
enum class PlannerStatus : std::uint8_t {
  kSuccess,
  kFinished,
  kNoNeed,
  kRestartFromRest,
  kFailed,
  kEmergency,
  kOptimizationFailed,
};

}  // namespace navigation_planning
