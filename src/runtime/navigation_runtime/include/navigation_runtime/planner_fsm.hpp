#pragma once

#include <navigation_runtime_policy/planning_policy.hpp>

namespace navigation_runtime {

// The legacy ordinary failure hook is a scheduler diagnostic, not a second
// renewal policy. Arm it only on the first scheduler-period-sized window after
// the normal lead-time boundary. The lower edge is derived from the scheduler
// interval and the command-clock conversion tolerance; it deliberately does
// not introduce an independent safety horizon.
inline bool ordinaryRenewalFailureInjectionMayArm(
    const PlannerRenewalDecision& decision,
    const double scheduling_interval_s) noexcept {
  if (decision.reason != PlannerRenewalReason::kRenewalDue ||
      !std::isfinite(decision.remaining_main_horizon_s) ||
      decision.remaining_main_horizon_s <= 0.0 ||
      !std::isfinite(decision.required_lead_time_s) ||
      decision.required_lead_time_s <= 0.0 ||
      !std::isfinite(scheduling_interval_s) || scheduling_interval_s <= 0.0) {
    return false;
  }
  constexpr double kCommandClockToleranceS = 1.0e-9;
  return decision.remaining_main_horizon_s >=
      decision.required_lead_time_s - scheduling_interval_s -
      kCommandClockToleranceS;
}

}  // namespace navigation_runtime
