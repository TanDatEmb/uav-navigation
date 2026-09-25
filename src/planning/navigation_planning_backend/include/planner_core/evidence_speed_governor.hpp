#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include <planner_core/backup_braking.hpp>

namespace navigation_planning_backend {

enum class EvidenceSpeedFailure : std::uint8_t {
  kNone,
  kInvalidInput,
  kOutsideRecoveryEnvelope,
  kInsufficientSupport,
  kStopSynthesisFailed,
  kBudgetExhausted,
};

struct EvidenceSpeedLimit final {
  double speed_mps{0.0};
  // Scalar polynomial-hull support proposal only. It does not prove the
  // continuation lies inside a world/corridor; the planner runs those
  // independent candidate validators before staging.
  double support_m{0.0};
  // Means a concrete stop polynomial fits this scalar support and caller V/A/J
  // limits, not that a command has execution or world-certificate authority.
  bool sufficient{false};
  EvidenceSpeedFailure failure{EvidenceSpeedFailure::kInvalidInput};
};

namespace detail {

inline EvidenceSpeedFailure stopFailureToSpeedFailure(
    const StopFailureReason failure) noexcept {
  switch (failure) {
    case StopFailureReason::kNone:
      return EvidenceSpeedFailure::kNone;
    case StopFailureReason::kInvalidState:
    case StopFailureReason::kInvalidDynamics:
      return EvidenceSpeedFailure::kInvalidInput;
    case StopFailureReason::kOutsideRecoveryEnvelope:
      return EvidenceSpeedFailure::kOutsideRecoveryEnvelope;
    case StopFailureReason::kInsufficientSupport:
      return EvidenceSpeedFailure::kInsufficientSupport;
    case StopFailureReason::kSynthesisFailed:
      return EvidenceSpeedFailure::kStopSynthesisFailed;
    case StopFailureReason::kBudgetExhausted:
      return EvidenceSpeedFailure::kBudgetExhausted;
  }
  return EvidenceSpeedFailure::kInvalidInput;
}

inline double minimumSnapSteadyCruiseSpeedCap(
    const double requested_speed_mps, const double maximum_velocity_mps,
    const double maximum_acceleration_mps2,
    const double maximum_jerk_mps3, const double support_m,
    const double sample_traj_dt_s) noexcept {
  if (!std::isfinite(requested_speed_mps) || requested_speed_mps <= 0.0 ||
      !std::isfinite(maximum_velocity_mps) || maximum_velocity_mps <= 0.0 ||
      !std::isfinite(maximum_acceleration_mps2) || maximum_acceleration_mps2 <= 0.0 ||
      !std::isfinite(maximum_jerk_mps3) || maximum_jerk_mps3 <= 0.0 ||
      !std::isfinite(support_m) || support_m <= 0.0 ||
      !std::isfinite(sample_traj_dt_s) || sample_traj_dt_s <= 0.0) {
    return 0.0;
  }
  const long double support = static_cast<long double>(support_m);
  const long double floor = 4.0L * static_cast<long double>(sample_traj_dt_s);
  const long double acceleration =
      static_cast<long double>(maximum_acceleration_mps2);
  const long double jerk = static_cast<long double>(maximum_jerk_mps3);
  const long double cap = std::min({
      static_cast<long double>(requested_speed_mps),
      static_cast<long double>(maximum_velocity_mps),
      2.0L * support / floor,
      std::sqrt(16.0L * acceleration * support / 15.0L),
      std::cbrt(2.0L * std::sqrt(3.0L) * jerk * support * support / 5.0L)});
  const double result = static_cast<double>(cap);
  return std::isfinite(result) && result > 0.0 ? result : 0.0;
}

template <typename ShouldAbort>
[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimitImpl(
    const navigation_math::StatePVAJ& state,
    const navigation_planning::DynamicLimits& dynamics,
    const std::array<double, 4>& support_m,
    const double sample_traj_dt_s,
    ShouldAbort&& should_abort) noexcept {
  EvidenceSpeedLimit result;
  if (!state.allFinite() || !dynamics.valid() ||
      !std::isfinite(sample_traj_dt_s) || sample_traj_dt_s <= 0.0) {
    return result;
  }
  result.support_m = std::numeric_limits<double>::infinity();
  for (const double support : support_m) {
    if (!std::isfinite(support) || support <= 0.0) {
      result.support_m = 0.0;
      result.failure = EvidenceSpeedFailure::kInsufficientSupport;
      return result;
    }
    result.support_m = std::min(result.support_m, support);
  }

  const auto abort_requested = [&]() noexcept {
    try {
      return static_cast<bool>(should_abort());
    } catch (...) {
      return true;
    }
  };
  if (abort_requested()) {
    result.failure = EvidenceSpeedFailure::kBudgetExhausted;
    return result;
  }

  // First certify the actual PVAJ boundary under the caller's physical
  // recovery model. Reducing desired cruise speed does not reduce current
  // measured velocity, acceleration, or jerk.
  const auto current_stop = evaluateStopReachabilityWithAbort(
      state, dynamics, result.support_m, sample_traj_dt_s, 0.0,
      abort_requested);
  if (!current_stop.feasible) {
    result.failure = stopFailureToSpeedFailure(current_stop.failure);
    return result;
  }

  const Eigen::Vector3d velocity = state.col(1);
  const Eigen::Vector3d direction = velocity.norm() > 1.0e-9
      ? velocity.normalized() : Eigen::Vector3d::UnitX();
  const double upper = dynamics.intent.requested_cruise_speed_mps;
  double candidate_speed = minimumSnapSteadyCruiseSpeedCap(
      upper, dynamics.vehicle.maximum_velocity_mps,
      dynamics.vehicle.maximum_acceleration_mps2,
      dynamics.vehicle.maximum_jerk_mps3, result.support_m,
      sample_traj_dt_s);
  if (candidate_speed <= 0.0) {
    result.failure = EvidenceSpeedFailure::kInsufficientSupport;
    return result;
  }

  // The proposal is restricted to steady prospective cruise (a=j=0). Its
  // closed-form cap is checked by the same production stop polynomial and
  // continuous extrema used by BACKUP. Correct only bounded floating-point
  // boundary discrepancies; this is not a speed search grid.
  // The analytic cap can land on a Bezier-hull support boundary. A few
  // representable downward steps may be needed for the concrete stop support
  // bound to fit; this is bounded numerical correction, not a speed-policy
  // margin.
  constexpr std::size_t kMaximumRoundoffCorrections = 8U;
  for (std::size_t correction = 0U;
       correction <= kMaximumRoundoffCorrections; ++correction) {
    if (abort_requested()) {
      result.failure = EvidenceSpeedFailure::kBudgetExhausted;
      return result;
    }
    navigation_math::StatePVAJ candidate_state;
    candidate_state.setZero();
    candidate_state.col(0) = state.col(0);
    candidate_state.col(1) = direction * candidate_speed;
    const auto candidate_stop = evaluateStopReachabilityWithAbort(
        candidate_state, dynamics, result.support_m, sample_traj_dt_s, 0.0,
        abort_requested);
    if (candidate_stop.feasible) {
      if (abort_requested()) {
        result.failure = EvidenceSpeedFailure::kBudgetExhausted;
        return result;
      }
      result.speed_mps = candidate_speed;
      result.sufficient = true;
      result.failure = EvidenceSpeedFailure::kNone;
      return result;
    }
    if (candidate_stop.failure == StopFailureReason::kBudgetExhausted) {
      result.failure = EvidenceSpeedFailure::kBudgetExhausted;
      return result;
    }
    if (candidate_stop.failure == StopFailureReason::kOutsideRecoveryEnvelope) {
      result.failure = EvidenceSpeedFailure::kOutsideRecoveryEnvelope;
      return result;
    } else if (candidate_stop.failure == StopFailureReason::kSynthesisFailed) {
      result.failure = EvidenceSpeedFailure::kStopSynthesisFailed;
      return result;
    } else if (candidate_stop.failure == StopFailureReason::kInsufficientSupport) {
      candidate_speed = std::nextafter(candidate_speed, 0.0);
      if (!std::isfinite(candidate_speed) || candidate_speed <= 0.0) {
        result.failure = EvidenceSpeedFailure::kInsufficientSupport;
        return result;
      }
      continue;
    } else {
      result.failure = stopFailureToSpeedFailure(candidate_stop.failure);
      return result;
    }
  }
  result.failure = EvidenceSpeedFailure::kInsufficientSupport;
  return result;
}

}  // namespace detail

template <typename ShouldAbort>
[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimit(
    const navigation_math::StatePVAJ& state,
    const navigation_planning::DynamicLimits& dynamics,
    const std::array<double, 4>& support_m,
    const double sample_traj_dt_s,
    ShouldAbort&& should_abort) noexcept {
  return detail::evidenceAwareSpeedLimitImpl(
      state, dynamics, support_m, sample_traj_dt_s,
      std::forward<ShouldAbort>(should_abort));
}

[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimit(
    const navigation_math::StatePVAJ& state,
    const navigation_planning::DynamicLimits& dynamics,
    const std::array<double, 4>& support_m) noexcept {
  return detail::evidenceAwareSpeedLimitImpl(
      state, dynamics, support_m, 0.05, [] { return false; });
}

[[nodiscard]] inline EvidenceSpeedLimit evidenceAwareSpeedLimit(
    const double configured_speed_mps, const double max_acceleration_mps2,
    const double max_jerk_mps3, const std::array<double, 4>& support_m) noexcept {
  EvidenceSpeedLimit result;
  if (!std::isfinite(configured_speed_mps) || configured_speed_mps <= 0.0 ||
      !std::isfinite(max_acceleration_mps2) || max_acceleration_mps2 <= 0.0 ||
      !std::isfinite(max_jerk_mps3) || max_jerk_mps3 <= 0.0) {
    return result;
  }
  // This convenience form has no measured boundary state; interpret the
  // configured value only as a prospective steady cruise speed.
  navigation_math::StatePVAJ state = navigation_math::StatePVAJ::Zero();
  navigation_planning::DynamicLimits dynamics;
  dynamics.vehicle.maximum_velocity_mps = configured_speed_mps;
  dynamics.vehicle.maximum_acceleration_mps2 = max_acceleration_mps2;
  dynamics.vehicle.maximum_jerk_mps3 = max_jerk_mps3;
  dynamics.intent.requested_cruise_speed_mps = configured_speed_mps;
  return evidenceAwareSpeedLimit(state, dynamics, support_m);
}

}  // namespace navigation_planning_backend
