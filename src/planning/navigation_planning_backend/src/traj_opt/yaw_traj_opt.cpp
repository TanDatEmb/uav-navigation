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
  last_diagnostics_ = {};
  last_diagnostics_.initial_state = initial_state_in;
  last_diagnostics_.target_yaw_rad = target_yaw_rad;
  last_diagnostics_.duration_s = duration;
  if (!initial_state_in.allFinite() || !std::isfinite(target_yaw_rad) ||
      !std::isfinite(duration) || duration <= 0.0 ||
      !std::isfinite(yaw_rate_max_rad_s_) || yaw_rate_max_rad_s_ <= 0.0 ||
      !std::isfinite(yaw_acceleration_max_rad_s2_) ||
      yaw_acceleration_max_rad_s2_ <= 0.0 ||
      std::abs(initial_state_in(1)) > yaw_rate_max_rad_s_ + 1.0e-6 ||
      std::abs(initial_state_in(2)) >
          yaw_acceleration_max_rad_s2_ + 1.0e-6) {
    last_diagnostics_.failure = YawOptimizationFailure::kInvalidInput;
    return false;
  }
  Vec4f initial_state = initial_state_in;
  normalizeNextYaw(initial_state(0), target_yaw_rad);
  const double requested_delta = target_yaw_rad - initial_state(0);
  last_diagnostics_.target_yaw_rad = target_yaw_rad;
  last_diagnostics_.requested_delta_rad = requested_delta;
  const VecDf no_waypoints;
  const auto interpolate_delta = [&](const double yaw_delta_rad,
                                     const double transition_duration) {
    const navigation_math::Vec3f initial = initial_state.head(3);
    navigation_math::Vec3f terminal;
    terminal << initial_state(0) + yaw_delta_rad, 0.0, 0.0;
    const VecDf times = VecDf::Constant(1, transition_duration);
    return poly_interpo::minimumJerkInterpolation<1>(
        initial, terminal, no_waypoints, times);
  };
  const auto appendTargetHold = [&](Trajectory transition,
                                    const double transition_duration) {
    if (transition.empty()) return transition;
    const double hold_duration = duration - transition_duration;
    if (hold_duration > 1.0e-9) {
      Eigen::MatrixXd hold_coefficients = Eigen::MatrixXd::Zero(3, 6);
      // Piece coefficients are ordered [t^5 ... t 1]; the last column is
      // the constant term of the certified terminal heading.
      hold_coefficients(0, hold_coefficients.cols() - 1) =
          transition.getPos(transition_duration).x();
      transition.emplace_back(hold_duration, hold_coefficients);
    }
    transition.start_WT = position_trajectory.start_WT;
    return transition;
  };
  const auto feasible = [&](const Trajectory &candidate) {
    if (candidate.empty()) return false;
    const double rate = candidate.getMaxVelRate();
    const double acceleration = candidate.getMaxAccRate();
    return std::isfinite(rate) && std::isfinite(acceleration) &&
           rate <= yaw_rate_max_rad_s_ + 1.0e-6 &&
           acceleration <= yaw_acceleration_max_rad_s2_ + 1.0e-6;
  };

  // A route heading is a local execution reference.  Stretching the yaw
  // transition over the complete position horizon makes a short WP-to-WP
  // turn visibly lag behind the translation, even when the yaw envelope has
  // ample capacity.  Find the shortest certified transition and hold the
  // exact target for the remainder of the same position horizon.  The search
  // is bounded and keeps the original rate/acceleration certificates.
  const auto shortestFeasible = [&](const double yaw_delta_rad,
                                    Trajectory& selected,
                                    double& selected_duration) {
    const auto candidateAt = [&](const double transition_duration) {
      const auto transition = interpolate_delta(
          yaw_delta_rad, transition_duration);
      return feasible(transition)
          ? appendTargetHold(transition, transition_duration)
          : Trajectory{};
    };
    if (duration <= 1.0e-9) return false;
    Trajectory full = candidateAt(duration);
    if (full.empty()) return false;

    double feasible_duration = duration;
    double failed_duration = duration;
    bool found_failed_shorter = false;
    for (int exponent = 1; exponent <= 12; ++exponent) {
      const double probe = duration / std::ldexp(1.0, exponent);
      if (probe <= 1.0e-4) break;
      Trajectory candidate = candidateAt(probe);
      if (!candidate.empty()) {
        feasible_duration = probe;
        selected = std::move(candidate);
      } else {
        failed_duration = probe;
        found_failed_shorter = true;
        break;
      }
    }
    if (!found_failed_shorter) {
      selected_duration = feasible_duration;
      if (selected.empty()) selected = std::move(full);
      return true;
    }
    for (int iteration = 0; iteration < 32; ++iteration) {
      const double probe = 0.5 * (feasible_duration + failed_duration);
      Trajectory candidate = candidateAt(probe);
      if (!candidate.empty()) {
        feasible_duration = probe;
        selected = std::move(candidate);
      } else {
        failed_duration = probe;
      }
    }
    selected_duration = feasible_duration;
    return !selected.empty();
  };

  Trajectory selected;
  double selected_duration = duration;
  const auto full_transition = interpolate_delta(requested_delta, duration);
  if (!full_transition.empty()) {
    last_diagnostics_.full_turn_max_rate_rad_s =
        full_transition.getMaxVelRate();
    last_diagnostics_.full_turn_max_acceleration_rad_s2 =
        full_transition.getMaxAccRate();
  }
  if (feasible(full_transition) &&
      shortestFeasible(requested_delta, selected, selected_duration)) {
  } else {
    const Trajectory hold_transition = interpolate_delta(0.0, duration);
    if (!hold_transition.empty()) {
      last_diagnostics_.hold_max_rate_rad_s =
          hold_transition.getMaxVelRate();
      last_diagnostics_.hold_max_acceleration_rad_s2 =
          hold_transition.getMaxAccRate();
    }
    if (!feasible(hold_transition)) {
      // A rotating state cannot generally finish at the exact same angle with
      // zero rate/acceleration without reversing part of its motion. The
      // free-terminal-position minimum-jerk stop advances by this analytic
      // displacement; it preserves the initial state and reaches zero
      // derivatives without inventing an opposite turn.
      const double stopping_displacement =
          0.5 * initial_state(1) * duration +
          initial_state(2) * duration * duration / 12.0;
      Trajectory stopping = interpolate_delta(stopping_displacement, duration);
      last_diagnostics_.stopping_displacement_rad = stopping_displacement;
      if (!stopping.empty()) {
        last_diagnostics_.stopping_max_rate_rad_s =
            stopping.getMaxVelRate();
        last_diagnostics_.stopping_max_acceleration_rad_s2 =
            stopping.getMaxAccRate();
      }
      if (!feasible(stopping)) {
        last_diagnostics_.failure = YawOptimizationFailure::kNoFeasibleHold;
        return false;
      }
      last_diagnostics_.used_stopping_displacement = true;
      selected = std::move(stopping);
      selected.start_WT = position_trajectory.start_WT;
      output_trajectory = std::move(selected);
      return true;
    }
    double feasible_scale = 0.0;
    double infeasible_scale = 1.0;
    selected = appendTargetHold(hold_transition, duration);
    for (int iteration = 0; iteration < 32; ++iteration) {
      const double scale = 0.5 * (feasible_scale + infeasible_scale);
      Trajectory candidate;
      const double delta = scale * requested_delta;
      if (shortestFeasible(delta, candidate, selected_duration)) {
        feasible_scale = scale;
        selected = std::move(candidate);
      } else {
        infeasible_scale = scale;
      }
    }
  }
  last_diagnostics_.selected_turn_duration_s = selected_duration;
  last_diagnostics_.holds_target_after_turn =
      selected_duration + 1.0e-9 < duration &&
      !last_diagnostics_.used_stopping_displacement;
  selected.start_WT = position_trajectory.start_WT;
  output_trajectory = std::move(selected);
  return true;
}

}  // namespace traj_opt
