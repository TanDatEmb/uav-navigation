#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include <planner_core/evidence_speed_governor.hpp>

int main() {
  using navigation_math::StatePVAJ;
  using navigation_planning_backend::evidenceAwareSpeedLimit;
  using navigation_planning_backend::evaluateStopReachability;

  const std::vector<double> supports{0.401, 0.45, 0.6, 1.0, 2.0, 5.0};
  const std::vector<double> requested_speeds{1.0, 3.0, 5.0, 8.0};
  std::size_t grid_rejects_with_swept_witness = 0U;
  std::size_t checked = 0U;

  for (const double support : supports) {
    for (const double requested : requested_speeds) {
      navigation_planning::DynamicLimits limits;
      limits.vehicle.maximum_velocity_mps = 12.0;
      limits.vehicle.maximum_acceleration_mps2 = 12.0;
      limits.vehicle.maximum_jerk_mps3 = 30.0;
      limits.intent.requested_cruise_speed_mps = requested;
      const StatePVAJ measured = StatePVAJ::Zero();
      const auto grid = evidenceAwareSpeedLimit(
          measured, limits, {support, support, support, support}, 0.05,
          [] { return false; });

      bool swept_witness = false;
      double first_witness = 0.0;
      // Independent speed enumeration over the same production stop
      // certificate; this checks the governor's sampled speed choices, not
      // collision/world validity or continuous mathematical completeness.
      for (int step = 1; step <= static_cast<int>(std::ceil(requested * 1000.0)); ++step) {
        const double speed = static_cast<double>(step) / 1000.0;
        if (speed > requested) break;
        StatePVAJ candidate = StatePVAJ::Zero();
        candidate.col(1).x() = speed;
        const auto stop = evaluateStopReachability(candidate, limits, support, 0.05);
        if (stop.feasible) {
          swept_witness = true;
          first_witness = speed;
          break;
        }
      }
      ++checked;
      if (!grid.sufficient && swept_witness) ++grid_rejects_with_swept_witness;
      std::cout << "support_m=" << support << " requested_mps=" << requested
                << " governor_sufficient=" << grid.sufficient
                << " governor_speed_mps=" << grid.speed_mps
                << " fine_sweep_feasible=" << swept_witness
                << " first_fine_speed_mps=" << first_witness << '\n';
    }
  }

  // Positive control and a lower support boundary control.
  navigation_planning::DynamicLimits control_limits;
  control_limits.vehicle.maximum_velocity_mps = 12.0;
  control_limits.vehicle.maximum_acceleration_mps2 = 12.0;
  control_limits.vehicle.maximum_jerk_mps3 = 30.0;
  control_limits.intent.requested_cruise_speed_mps = 5.0;
  const StatePVAJ measured = StatePVAJ::Zero();
  const auto positive = evidenceAwareSpeedLimit(
      measured, control_limits, {5.0, 5.0, 5.0, 5.0}, 0.05,
      [] { return false; });
  const auto missing = evidenceAwareSpeedLimit(
      measured, control_limits, {0.0, 5.0, 5.0, 5.0}, 0.05,
      [] { return false; });
  std::cout << "positive_control=" << positive.sufficient
            << " missing_support_control=" << missing.sufficient << '\n';
  std::cout << "checked=" << checked
            << " grid_rejects_with_fine_sweep_witness="
            << grid_rejects_with_swept_witness << '\n';
  return positive.sufficient && !missing.sufficient ? 0 : 1;
}
