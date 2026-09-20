#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <data_structure/base/trajectory.h>
#include <navigation_planning/planning_limits.hpp>
#include <planner_core/backup_braking.hpp>

namespace navigation_planning_backend {

struct BoundaryVelocityRecoveryReport {
  bool initial_overspeed{false};
  bool finite{false};
  bool peak_bounded{false};
  bool recovered_by_deadline{false};
  bool satisfied{false};
  double initial_speed_mps{std::numeric_limits<double>::infinity()};
  double allowed_peak_speed_mps{std::numeric_limits<double>::infinity()};
  double maximum_speed_mps{std::numeric_limits<double>::infinity()};
  double recovery_deadline_s{0.0};
  double suffix_maximum_speed_mps{std::numeric_limits<double>::infinity()};
};

// A measured boundary can already be slightly above the mission speed cap;
// rejecting every continuous trajectory cannot remove that physical state.
// Keep the cap strict for normal starts. For an overspeed start, permit only a
// non-worsening peak and require the exact polynomial suffix to be back under
// the normal cap by the same jerk-limited recovery scale used by backup
// braking. Piece extrema remain analytic; no sampled gate authorizes motion.
inline BoundaryVelocityRecoveryReport certifyBoundaryVelocityRecovery(
    const geometry_utils::Trajectory& trajectory,
    const double maximum_velocity_mps,
    const double maximum_acceleration_mps2,
    const double maximum_jerk_mps3) {
  BoundaryVelocityRecoveryReport report;
  if (trajectory.empty() || !std::isfinite(maximum_velocity_mps) ||
      !std::isfinite(maximum_acceleration_mps2) ||
      !std::isfinite(maximum_jerk_mps3) || maximum_velocity_mps <= 0.0 ||
      maximum_acceleration_mps2 <= 0.0 || maximum_jerk_mps3 <= 0.0) {
    return report;
  }

  report.initial_speed_mps = trajectory.getVel(0.0).norm();
  report.maximum_speed_mps = trajectory.getMaxVelRate();
  report.initial_overspeed = report.initial_speed_mps > maximum_velocity_mps;
  report.allowed_peak_speed_mps = std::max(
      report.initial_speed_mps, maximum_velocity_mps);
  report.finite = std::isfinite(report.initial_speed_mps) &&
      std::isfinite(report.maximum_speed_mps);
  if (!report.finite) return report;

  report.peak_bounded = navigation_planning::withinNumericalDynamicLimit(
      report.maximum_speed_mps, report.allowed_peak_speed_mps);
  if (!report.initial_overspeed) {
    report.recovered_by_deadline = report.peak_bounded;
    report.suffix_maximum_speed_mps = report.maximum_speed_mps;
    report.satisfied = report.peak_bounded;
    return report;
  }

  constexpr double kRecoveryFeasibilityMargin = 1.15;
  const double excess_speed_mps =
      report.initial_speed_mps - maximum_velocity_mps;
  report.recovery_deadline_s = kRecoveryFeasibilityMargin *
      jerkLimitedStopTime(excess_speed_mps, maximum_acceleration_mps2,
                          maximum_jerk_mps3);
  const double duration_s = trajectory.getTotalDuration();
  if (!std::isfinite(report.recovery_deadline_s) ||
      report.recovery_deadline_s <= 0.0 ||
      !std::isfinite(duration_s) ||
      report.recovery_deadline_s >= duration_s) {
    return report;
  }

  geometry_utils::Trajectory suffix;
  if (!trajectory.getPartialTrajectoryByTime(
          report.recovery_deadline_s, duration_s, suffix) || suffix.empty()) {
    return report;
  }
  report.suffix_maximum_speed_mps = suffix.getMaxVelRate();
  report.recovered_by_deadline =
      std::isfinite(report.suffix_maximum_speed_mps) &&
      navigation_planning::withinNumericalDynamicLimit(
          report.suffix_maximum_speed_mps, maximum_velocity_mps);
  report.satisfied = report.peak_bounded && report.recovered_by_deadline;
  return report;
}

}  // namespace navigation_planning_backend
