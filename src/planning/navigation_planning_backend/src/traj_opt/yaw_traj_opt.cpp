/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#include "traj_opt/yaw_traj_opt.h"

#include <cmath>
#include <utility>

#include <utils/optimization/polynomial_interpolation.h>

using namespace geometry_utils;

namespace traj_opt {

YawTrajOpt::YawTrajOpt(const double &maximum_yaw_rate_rad_s,
                       const double &maximum_yaw_acceleration_rad_s2)
    : yaw_rate_max_rad_s_(maximum_yaw_rate_rad_s),
      yaw_acceleration_max_rad_s2_(maximum_yaw_acceleration_rad_s2) {}

bool YawTrajOpt::optimizeToTarget(
    const Vec4f &initial_state_in, double target_yaw_rad,
    const Trajectory &position_trajectory,
    Trajectory &output_trajectory) {
  const double duration = position_trajectory.getTotalDuration();
  if (!initial_state_in.allFinite() || !std::isfinite(target_yaw_rad) ||
      !std::isfinite(duration) || duration <= 0.0 ||
      !std::isfinite(yaw_rate_max_rad_s_) || yaw_rate_max_rad_s_ <= 0.0 ||
      !std::isfinite(yaw_acceleration_max_rad_s2_) ||
      yaw_acceleration_max_rad_s2_ <= 0.0 ||
      std::abs(initial_state_in(1)) > yaw_rate_max_rad_s_ + 1.0e-6 ||
      std::abs(initial_state_in(2)) >
          yaw_acceleration_max_rad_s2_ + 1.0e-6) {
    return false;
  }
  Vec4f initial_state = initial_state_in;
  normalizeNextYaw(initial_state(0), target_yaw_rad);
  const double requested_delta = target_yaw_rad - initial_state(0);
  const VecDf times = VecDf::Constant(1, duration);
  const VecDf no_waypoints;
  const auto interpolate = [&](const double scale) {
    const navigation_math::Vec3f initial = initial_state.head(3);
    navigation_math::Vec3f terminal;
    terminal << initial_state(0) + scale * requested_delta, 0.0, 0.0;
    return poly_interpo::minimumJerkInterpolation<1>(
        initial, terminal, no_waypoints, times);
  };
  const auto feasible = [&](const Trajectory &candidate) {
    const double rate = candidate.getMaxVelRate();
    const double acceleration = candidate.getMaxAccRate();
    return !candidate.empty() && std::isfinite(rate) &&
           std::isfinite(acceleration) &&
           rate <= yaw_rate_max_rad_s_ + 1.0e-6 &&
           acceleration <= yaw_acceleration_max_rad_s2_ + 1.0e-6;
  };

  Trajectory selected = interpolate(1.0);
  if (!feasible(selected)) {
    selected = interpolate(0.0);
    if (!feasible(selected)) return false;
    double feasible_scale = 0.0;
    double infeasible_scale = 1.0;
    for (int iteration = 0; iteration < 32; ++iteration) {
      const double scale = 0.5 * (feasible_scale + infeasible_scale);
      Trajectory candidate = interpolate(scale);
      if (feasible(candidate)) {
        feasible_scale = scale;
        selected = std::move(candidate);
      } else {
        infeasible_scale = scale;
      }
    }
  }
  selected.start_WT = position_trajectory.start_WT;
  output_trajectory = std::move(selected);
  return true;
}

}  // namespace traj_opt
