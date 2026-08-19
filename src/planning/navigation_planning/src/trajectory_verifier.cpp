#include "navigation_planning/trajectory_verifier.hpp"

namespace navigation_planning {

TrajectoryVerificationResult TrajectoryVerifier::verify(
    const TimeParameterizedTrajectory& trajectory, const VehicleState& state, PlanRole role,
    const navigation_mapping::WorldModel& world) const {
  return detail::verifyModel(config_, trajectory, state, role, world);
}

DualPlanVerificationResult TrajectoryVerifier::verifyDual(
    const TimeParameterizedTrajectory& nominal, const TimeParameterizedTrajectory& safety,
    const VehicleState& state, const navigation_mapping::WorldModel& world) const {
  return detail::verifyDualModel(config_, nominal, safety, state, world);
}

}  // namespace navigation_planning
