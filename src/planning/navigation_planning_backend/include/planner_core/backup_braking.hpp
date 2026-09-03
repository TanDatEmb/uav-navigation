#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <data_structure/base/piece.h>
#include <navigation_planning/planning_limits.hpp>
#include <navigation_planning/kinematic_state.hpp>
#include <utils/header/eigen_alias.hpp>

namespace navigation_planning_backend {

enum class StopFailureReason : std::uint8_t {
  kNone,
  kInvalidState,
  kInvalidDynamics,
  kInsufficientSupport,
};

struct StopReachability {
  bool feasible{false};
  double stopping_time_s{0.0};
  double stopping_distance_m{0.0};
  double admissible_entry_speed_mps{0.0};
  double latest_safe_switch_time_s{0.0};
  Eigen::Vector3d stop_position{Eigen::Vector3d::Zero()};
  StopFailureReason failure{StopFailureReason::kNone};
};

// Conservative full-state envelope shared by speed authorization and the
// deterministic BACKUP builder. Positive acceleration along the current
// velocity is retained; transverse acceleration is also charged through its
// norm because the stop polynomial must inherit the complete PVAJ boundary.
struct StopKinematicEnvelope {
  double effective_speed_mps{0.0};
  double acceleration_release_s{0.0};
  double stopping_time_s{0.0};
  double stopping_distance_m{0.0};
};

struct BackupBrakingSeed {
  double switch_time_s{0.0};
  double duration_s{0.0};
  navigation_math::Vec3f endpoint{navigation_math::Vec3f::Zero()};
  double initial_velocity_mps{std::numeric_limits<double>::infinity()};
  double allowed_peak_velocity_mps{std::numeric_limits<double>::infinity()};
  double maximum_velocity_mps{std::numeric_limits<double>::infinity()};
  double maximum_acceleration_mps2{std::numeric_limits<double>::infinity()};
  double maximum_jerk_mps3{std::numeric_limits<double>::infinity()};
  bool initial_overspeed{false};
  bool terminal_altitude_preserved{false};
  bool feasible{false};
};

inline double jerkLimitedStopTime(double speed_mps, double max_acc_mps2,
                                  double max_jerk_mps3) {
  if (!std::isfinite(speed_mps) || !std::isfinite(max_acc_mps2) ||
      !std::isfinite(max_jerk_mps3) || speed_mps < 0.0 ||
      max_acc_mps2 <= 0.0 || max_jerk_mps3 <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double velocity_at_full_accel =
      max_acc_mps2 * max_acc_mps2 / max_jerk_mps3;
  if (speed_mps <= velocity_at_full_accel) {
    return 2.0 * std::sqrt(speed_mps / max_jerk_mps3);
  }
  return speed_mps / max_acc_mps2 + max_acc_mps2 / max_jerk_mps3;
}

inline double jerkLimitedStopDistance(double speed_mps, double max_acc_mps2,
                                      double max_jerk_mps3) {
  const double stop_time_s =
      jerkLimitedStopTime(speed_mps, max_acc_mps2, max_jerk_mps3);
  return std::isfinite(stop_time_s)
             ? 0.5 * speed_mps * stop_time_s
             : std::numeric_limits<double>::infinity();
}

inline StopKinematicEnvelope stopKinematicEnvelope(
    const navigation_math::StatePVAJ& state, double max_acc_mps2,
    double max_jerk_mps3) noexcept {
  StopKinematicEnvelope result;
  if (!state.allFinite() || !std::isfinite(max_acc_mps2) ||
      max_acc_mps2 <= 0.0 || !std::isfinite(max_jerk_mps3) ||
      max_jerk_mps3 <= 0.0) {
    result.stopping_time_s = std::numeric_limits<double>::infinity();
    result.stopping_distance_m = std::numeric_limits<double>::infinity();
    return result;
  }
  const Eigen::Vector3d velocity = state.col(1);
  const double speed_mps = velocity.norm();
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  if (speed_mps > 1.0e-9) direction = velocity / speed_mps;
  const double longitudinal_acceleration =
      std::max(0.0, state.col(2).dot(direction));
  const double acceleration_norm = state.col(2).norm();
  result.acceleration_release_s = acceleration_norm / max_jerk_mps3;
  result.effective_speed_mps = speed_mps +
      0.5 * acceleration_norm * result.acceleration_release_s +
      0.5 * longitudinal_acceleration * result.acceleration_release_s;
  result.stopping_time_s = result.acceleration_release_s +
      jerkLimitedStopTime(result.effective_speed_mps, max_acc_mps2,
                          max_jerk_mps3);
  result.stopping_distance_m =
      speed_mps * result.acceleration_release_s +
      jerkLimitedStopDistance(result.effective_speed_mps, max_acc_mps2,
                              max_jerk_mps3);
  return result;
}

// `known_free_support_m` is the minimum certified support along the planned
// stop direction/corridor. Geometry remains owned by the world model; this
// function only combines that evidence with the same full-state stop model
// used to construct BACKUP.
inline StopReachability evaluateStopReachability(
    const navigation_math::StatePVAJ& state,
    const navigation_planning::DynamicLimits& dynamics,
    double known_free_support_m) noexcept {
  StopReachability result;
  if (state.allFinite()) {
    result.stop_position = state.col(0);
  } else {
    result.stop_position =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  if (!state.allFinite()) {
    result.failure = StopFailureReason::kInvalidState;
    return result;
  }
  if (!dynamics.valid()) {
    result.failure = StopFailureReason::kInvalidDynamics;
    return result;
  }
  if (!std::isfinite(known_free_support_m) || known_free_support_m <= 0.0) {
    result.failure = StopFailureReason::kInsufficientSupport;
    return result;
  }
  const auto envelope = stopKinematicEnvelope(
      state, dynamics.vehicle.maximum_acceleration_mps2,
      dynamics.vehicle.maximum_jerk_mps3);
  result.stopping_time_s = envelope.stopping_time_s;
  result.stopping_distance_m = envelope.stopping_distance_m;
  result.admissible_entry_speed_mps = envelope.effective_speed_mps;
  result.feasible = std::isfinite(result.stopping_distance_m) &&
      result.stopping_distance_m <= known_free_support_m;
  if (!result.feasible) result.failure = StopFailureReason::kInsufficientSupport;
  return result;
}

inline bool refinementDurationRespectsCertifiedFloor(
    double candidate_duration_s, double certified_duration_s,
    double tolerance_s = 1.0e-9) {
  return std::isfinite(candidate_duration_s) &&
      std::isfinite(certified_duration_s) &&
      std::isfinite(tolerance_s) && certified_duration_s > 0.0 &&
      tolerance_s >= 0.0 &&
      candidate_duration_s + tolerance_s >= certified_duration_s;
}

inline geometry_utils::Piece minimumSnapStopPiece(
    const navigation_math::StatePVAJ &initial_state, double duration_s) {
  Eigen::Matrix<double, 3, 8> coefficients;
  coefficients.setZero();
  const auto position = initial_state.col(0);
  const auto velocity = initial_state.col(1);
  const auto acceleration = initial_state.col(2);
  const auto jerk = initial_state.col(3);
  const double t2 = duration_s * duration_s;
  const double t3 = t2 * duration_s;
  const double t4 = t3 * duration_s;
  const double t5 = t4 * duration_s;

  // Piece stores descending powers [t^7 ... t 1]. These coefficients are the
  // free-end-position minimum-snap polynomial with terminal V/A/J equal zero.
  coefficients.col(0).setZero();
  coefficients.col(1) =
      (-t2 * jerk / 12.0 - duration_s * acceleration / 2.0 - velocity) / t5;
  coefficients.col(2) =
      (3.0 * t2 * jerk + 16.0 * duration_s * acceleration + 30.0 * velocity) /
      (10.0 * t4);
  coefficients.col(3) =
      (-3.0 * t2 * jerk - 12.0 * duration_s * acceleration - 20.0 * velocity) /
      (8.0 * t3);
  coefficients.col(4) = jerk / 6.0;
  coefficients.col(5) = acceleration / 2.0;
  coefficients.col(6) = velocity;
  coefficients.col(7) = position;
  return geometry_utils::Piece(duration_s, coefficients);
}

inline geometry_utils::Piece minimumSnapStopPieceWithTerminalAltitude(
    const navigation_math::StatePVAJ &initial_state, double duration_s,
    double terminal_altitude_m) {
  const auto free_end_piece = minimumSnapStopPiece(initial_state, duration_s);
  if (!std::isfinite(duration_s) || duration_s <= 0.0 ||
      !std::isfinite(terminal_altitude_m) || !initial_state.allFinite()) {
    return free_end_piece;
  }

  const double free_end_altitude_m = free_end_piece.getPos(duration_s).z();
  const double altitude_delta_m = terminal_altitude_m - free_end_altitude_m;
  const double t2 = duration_s * duration_s;
  const double t4 = t2 * t2;
  const double t5 = t4 * duration_s;
  const double t6 = t5 * duration_s;
  const double t7 = t6 * duration_s;
  if (!std::isfinite(altitude_delta_m) || !std::isfinite(t4) ||
      !std::isfinite(t5) || !std::isfinite(t6) || !std::isfinite(t7) ||
      t4 <= 0.0 || t5 <= 0.0 || t6 <= 0.0 || t7 <= 0.0) {
    return free_end_piece;
  }

  // Add a C^3 smoothstep to the free-end minimum-snap polynomial. The
  // smoothstep and its first three derivatives vanish at both endpoints, so
  // the measured PVAJ handover and terminal V/A/J=0 remain exact while only
  // the terminal altitude is changed. Piece stores descending powers
  // [t^7 ... t 1].
  auto coefficients = free_end_piece.getCoeffMat();
  coefficients(2, 0) += -20.0 * altitude_delta_m / t7;
  coefficients(2, 1) += 70.0 * altitude_delta_m / t6;
  coefficients(2, 2) += -84.0 * altitude_delta_m / t5;
  coefficients(2, 3) += 35.0 * altitude_delta_m / t4;
  return geometry_utils::Piece(duration_s, coefficients);
}

inline Eigen::Matrix<double, 3, 8> minimumSnapStopBezierControlPoints(
    const geometry_utils::Piece &piece) {
  const double duration_s = piece.getDuration();
  const auto &coefficients = piece.getCoeffMat();
  Eigen::Matrix<double, 3, 8> power;
  double duration_power = 1.0;
  for (int degree = 0; degree <= 7; ++degree) {
    power.col(degree) = coefficients.col(7 - degree) * duration_power;
    duration_power *= duration_s;
  }
  constexpr int binomial[8][8] = {
      {1, 0, 0, 0, 0, 0, 0, 0},
      {1, 1, 0, 0, 0, 0, 0, 0},
      {1, 2, 1, 0, 0, 0, 0, 0},
      {1, 3, 3, 1, 0, 0, 0, 0},
      {1, 4, 6, 4, 1, 0, 0, 0},
      {1, 5, 10, 10, 5, 1, 0, 0},
      {1, 6, 15, 20, 15, 6, 1, 0},
      {1, 7, 21, 35, 35, 21, 7, 1},
  };
  Eigen::Matrix<double, 3, 8> control_points;
  control_points.setZero();
  for (int i = 0; i <= 7; ++i) {
    for (int k = 0; k <= i; ++k) {
      control_points.col(i) += power.col(k) *
          static_cast<double>(binomial[i][k]) /
          static_cast<double>(binomial[7][k]);
    }
  }
  return control_points;
}

inline Eigen::Matrix<double, 3, 8> minimumSnapStopBezierControlPoints(
    const navigation_math::StatePVAJ &initial_state, double duration_s) {
  return minimumSnapStopBezierControlPoints(
      minimumSnapStopPiece(initial_state, duration_s));
}

inline BackupBrakingSeed makeBackupBrakingSeed(
    double switch_time_s, const navigation_math::StatePVAJ &switch_state,
    double max_velocity_mps, double max_acc_mps2, double max_jerk_mps3,
    double sample_traj_dt_s, double feasibility_margin) {
  BackupBrakingSeed result;
  result.switch_time_s = switch_time_s;
  if (!std::isfinite(switch_time_s) || !switch_state.allFinite() ||
      !std::isfinite(max_velocity_mps) || max_velocity_mps <= 0.0 ||
      !std::isfinite(max_acc_mps2) || max_acc_mps2 <= 0.0 ||
      !std::isfinite(max_jerk_mps3) || max_jerk_mps3 <= 0.0 ||
      !std::isfinite(sample_traj_dt_s) || sample_traj_dt_s <= 0.0 ||
      !std::isfinite(feasibility_margin) || feasibility_margin < 0.0) {
    return result;
  }

  const double speed_mps = switch_state.col(1).norm();
  result.initial_velocity_mps = speed_mps;
  result.initial_overspeed = speed_mps > max_velocity_mps;
  // An observed overspeed cannot be removed instantaneously. Certify that the
  // braking polynomial never accelerates beyond the physically unavoidable
  // initial speed while retaining the normal mission cap for nominal states.
  result.allowed_peak_velocity_mps = std::max(speed_mps, max_velocity_mps);
  // Keep speed authorization and BACKUP construction on the same full-state
  // PVAJ envelope. The polynomial certificate below remains authoritative for
  // the exact vector extrema.
  const auto envelope = stopKinematicEnvelope(
      switch_state, max_acc_mps2, max_jerk_mps3);
  double duration_s = envelope.stopping_time_s;
  duration_s = std::max(4.0 * sample_traj_dt_s, 1.15 * duration_s);

  const double gate = 1.0 + feasibility_margin;
  // With non-zero boundary acceleration/jerk, polynomial extrema are not
  // monotone in duration. Search a bounded interval around the jerk-limited
  // estimate instead of repeatedly stretching a candidate that may acquire a
  // larger velocity overshoot.
  duration_s = std::max(4.0 * sample_traj_dt_s, duration_s * 0.5);
  for (int attempt = 0; attempt < 24; ++attempt) {
    const auto piece = minimumSnapStopPiece(switch_state, duration_s);
    result.duration_s = duration_s;
    result.endpoint = piece.getPos(duration_s);
    result.maximum_velocity_mps = piece.getMaxVelRate();
    result.maximum_acceleration_mps2 = piece.getMaxAccRate();
    result.maximum_jerk_mps3 = piece.getMaxJerRate();
    result.feasible = result.endpoint.allFinite() &&
        std::isfinite(result.maximum_velocity_mps) &&
        std::isfinite(result.maximum_acceleration_mps2) &&
        std::isfinite(result.maximum_jerk_mps3) &&
        navigation_planning::withinNumericalDynamicLimit(
            result.maximum_velocity_mps, gate * result.allowed_peak_velocity_mps) &&
        navigation_planning::withinNumericalDynamicLimit(
            result.maximum_acceleration_mps2, gate * max_acc_mps2) &&
        navigation_planning::withinNumericalDynamicLimit(
            result.maximum_jerk_mps3, gate * max_jerk_mps3);
    if (result.feasible) {
      return result;
    }
    duration_s *= 1.15;
  }
  return result;
}

inline BackupBrakingSeed makeBackupBrakingSeedWithTerminalAltitude(
    double switch_time_s, const navigation_math::StatePVAJ &switch_state,
    double max_velocity_mps, double max_acc_mps2, double max_jerk_mps3,
    double sample_traj_dt_s, double feasibility_margin,
    double terminal_altitude_m) {
  BackupBrakingSeed result = makeBackupBrakingSeed(
      switch_time_s, switch_state, max_velocity_mps, max_acc_mps2,
      max_jerk_mps3, sample_traj_dt_s, feasibility_margin);
  if (!result.feasible || !std::isfinite(terminal_altitude_m)) {
    return result;
  }

  const double gate = 1.0 + feasibility_margin;
  double duration_s = result.duration_s;
  // A vertical recovery target can need a little more time than the free-end
  // stop. Search only a bounded extension of the already certified seed; the
  // exact V/A/J limits remain unchanged and a failure retains the free-end
  // certificate returned above.
  for (int attempt = 0; attempt < 12; ++attempt) {
    const auto altitude_piece = minimumSnapStopPieceWithTerminalAltitude(
        switch_state, duration_s, terminal_altitude_m);
    const double maximum_velocity_mps = altitude_piece.getMaxVelRate();
    const double maximum_acceleration_mps2 = altitude_piece.getMaxAccRate();
    const double maximum_jerk_mps3 = altitude_piece.getMaxJerRate();
    if (altitude_piece.getPos(duration_s).allFinite() &&
        std::isfinite(maximum_velocity_mps) &&
        std::isfinite(maximum_acceleration_mps2) &&
        std::isfinite(maximum_jerk_mps3) &&
        navigation_planning::withinNumericalDynamicLimit(
            maximum_velocity_mps, gate * result.allowed_peak_velocity_mps) &&
        navigation_planning::withinNumericalDynamicLimit(
            maximum_acceleration_mps2, gate * max_acc_mps2) &&
        navigation_planning::withinNumericalDynamicLimit(
            maximum_jerk_mps3, gate * max_jerk_mps3)) {
      result.duration_s = duration_s;
      result.endpoint = altitude_piece.getPos(duration_s);
      result.maximum_velocity_mps = maximum_velocity_mps;
      result.maximum_acceleration_mps2 = maximum_acceleration_mps2;
      result.maximum_jerk_mps3 = maximum_jerk_mps3;
      result.terminal_altitude_preserved = true;
      return result;
    }
    duration_s *= 1.15;
    if (!std::isfinite(duration_s) || duration_s <= 0.0) {
      break;
    }
  }
  return result;
}

// A non-terminal pass-through BACKUP may stop before its active waypoint, but
// it must not enter that waypoint's measured acceptance ball and then stop
// beyond it. The latter looks collision-safe locally while leaving the
// mission with no forward route handoff; a later correction would be a
// forbidden route regression. This is a geometric disposition check only:
// the caller still owns the exact measured acceptance and all world/dynamic
// certificates.
inline bool backupEntersAcceptanceAndEndsOutside(
    const geometry_utils::Piece &piece, const Eigen::Vector3d &waypoint,
    const double acceptance_radius_m, const double sample_dt_s) noexcept {
  const double duration_s = piece.getDuration();
  if (!std::isfinite(duration_s) || duration_s <= 0.0 ||
      !waypoint.allFinite() || !std::isfinite(acceptance_radius_m) ||
      acceptance_radius_m <= 0.0 || !std::isfinite(sample_dt_s) ||
      sample_dt_s <= 0.0) {
    return false;
  }
  const auto endpoint = piece.getPos(duration_s);
  if (!endpoint.allFinite() ||
      (endpoint.cast<double>() - waypoint).norm() <= acceptance_radius_m + 1.0e-6) {
    return false;
  }
  const auto sample_count = static_cast<std::size_t>(std::ceil(
      duration_s / sample_dt_s));
  if (sample_count > 10000000U) return false;
  for (std::size_t sample = 0U; sample <= sample_count; ++sample) {
    const double time_s = std::min(
        duration_s, static_cast<double>(sample) * sample_dt_s);
    const auto position = piece.getPos(time_s);
    if (position.allFinite() &&
        (position.cast<double>() - waypoint).norm() <= acceptance_radius_m + 1.0e-6) {
      return true;
    }
  }
  return false;
}

}  // namespace navigation_planning_backend
