#include "px4_odometry_bridge/geometric_jump_continuity.hpp"

#include <cmath>

#include <navigation_common/time.hpp>

namespace px4_odometry_bridge {

namespace {

GeometricJumpContinuityObservation reseed(
    const ExternalOdometryFrame& current, const bool generation_valid,
    const std::uint64_t generation,
    const GeometricJumpContinuityReason reason,
    const bool continuity_trusted,
    GeometricJumpContinuityState& state) {
  GeometricJumpContinuityObservation observation;
  observation.baseline_reseeded = true;
  observation.reason = reason;
  state.last_received = current;
  state.last_generation = generation;
  state.last_generation_valid = generation_valid;
  state.continuity_trusted = continuity_trusted;
  return observation;
}

}  // namespace

GeometricJumpContinuityObservation observe_geometric_jump_continuity(
    const ExternalOdometryFrame& current, const bool source_valid,
    const bool generation_valid, const std::uint64_t generation,
    const GeometricJumpContinuityConfig& config,
    GeometricJumpContinuityState& state) {
  const auto config_valid = [&]() {
    return std::isfinite(config.position_jump_margin_m) &&
           config.position_jump_margin_m >= 0.0 &&
           std::isfinite(config.orientation_jump_margin_rad) &&
           config.orientation_jump_margin_rad >= 0.0 &&
           std::isfinite(config.maximum_expected_speed_mps) &&
           config.maximum_expected_speed_mps >= 0.0 &&
           std::isfinite(config.maximum_expected_angular_rate_rad_s) &&
           config.maximum_expected_angular_rate_rad_s >= 0.0 &&
           std::isfinite(config.minimum_continuity_dt_s) &&
           config.minimum_continuity_dt_s > 0.0 &&
           std::isfinite(config.maximum_continuity_dt_s) &&
           config.maximum_continuity_dt_s >= config.minimum_continuity_dt_s;
  };
  const auto frame_valid = [](const ExternalOdometryFrame& frame) {
    const double quaternion_squared_norm =
        frame.orientation_ned.squaredNorm();
    return frame.timestamp_ns > 0 && frame.position_ned.allFinite() &&
           frame.velocity_ned.allFinite() &&
           frame.angular_velocity_body_frd.allFinite() &&
           frame.orientation_ned.coeffs().allFinite() &&
           std::isfinite(quaternion_squared_norm) &&
           std::abs(quaternion_squared_norm - 1.0) <= 1.0e-6;
  };
  if (!config_valid() || !frame_valid(current)) {
    return reseed(current, false, generation,
                  GeometricJumpContinuityReason::kCurrentFrameInvalid,
                  false, state);
  }
  if (!source_valid) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kSourceInvalid,
                  false, state);
  }
  if (!generation_valid || generation == 0U) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kGenerationInvalid,
                  false, state);
  }
  if (!current.frame_valid) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kCurrentFrameInvalid,
                  false, state);
  }
  if (!state.last_received.has_value() ||
      !state.last_generation_valid ||
      !state.continuity_trusted) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kNoBaseline,
                  true, state);
  }
  if (!state.last_received->frame_valid || !frame_valid(*state.last_received)) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kPreviousFrameInvalid,
                  true, state);
  }
  if (state.last_generation != generation) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kGenerationChanged,
                  true, state);
  }

  const auto dt = navigation_common::checkedDifference(
      current.timestamp_ns, state.last_received->timestamp_ns);
  if (!dt || *dt <= 0) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kTimestampNotIncreasing,
                  true, state);
  }

  GeometricJumpContinuityObservation observation;
  observation.dt_s = static_cast<double>(*dt) * 1e-9;
  if (observation.dt_s < config.minimum_continuity_dt_s) {
    observation = reseed(current, generation_valid, generation,
                         GeometricJumpContinuityReason::kDtTooSmall,
                         true, state);
    observation.dt_s = static_cast<double>(*dt) * 1e-9;
    return observation;
  }
  if (observation.dt_s > config.maximum_continuity_dt_s) {
    observation = reseed(current, generation_valid, generation,
                         GeometricJumpContinuityReason::kDtTooLarge,
                         true, state);
    observation.dt_s = static_cast<double>(*dt) * 1e-9;
    return observation;
  }

  observation.evaluated = true;
  observation.position_delta_m =
      (current.position_ned - state.last_received->position_ned).norm();
  observation.orientation_delta_rad =
      state.last_received->orientation_ned.angularDistance(current.orientation_ned);
  observation.allowed_position_delta_m =
      config.position_jump_margin_m + config.maximum_expected_speed_mps * observation.dt_s;
  observation.allowed_orientation_delta_rad =
      config.orientation_jump_margin_rad +
      config.maximum_expected_angular_rate_rad_s * observation.dt_s;
  observation.jumped =
      observation.position_delta_m > observation.allowed_position_delta_m ||
      observation.orientation_delta_rad > observation.allowed_orientation_delta_rad;
  observation.reason = observation.jumped
                           ? GeometricJumpContinuityReason::kJumpDetected
                           : GeometricJumpContinuityReason::kWithinThreshold;

  state.last_received = current;
  state.last_generation = generation;
  state.last_generation_valid = generation_valid;
  state.continuity_trusted = true;
  return observation;
}

const char* to_string(const GeometricJumpContinuityReason reason) noexcept {
  switch (reason) {
    case GeometricJumpContinuityReason::kNoBaseline:
      return "NO_BASELINE";
    case GeometricJumpContinuityReason::kSourceInvalid:
      return "SOURCE_INVALID";
    case GeometricJumpContinuityReason::kGenerationInvalid:
      return "GENERATION_INVALID";
    case GeometricJumpContinuityReason::kGenerationChanged:
      return "GENERATION_CHANGED";
    case GeometricJumpContinuityReason::kPreviousFrameInvalid:
      return "PREVIOUS_FRAME_INVALID";
    case GeometricJumpContinuityReason::kCurrentFrameInvalid:
      return "CURRENT_FRAME_INVALID";
    case GeometricJumpContinuityReason::kTimestampNotIncreasing:
      return "TIMESTAMP_NOT_INCREASING";
    case GeometricJumpContinuityReason::kDtTooSmall:
      return "DT_TOO_SMALL";
    case GeometricJumpContinuityReason::kDtTooLarge:
      return "DT_TOO_LARGE";
    case GeometricJumpContinuityReason::kWithinThreshold:
      return "WITHIN_THRESHOLD";
    case GeometricJumpContinuityReason::kJumpDetected:
      return "JUMP_DETECTED";
  }
  return "UNKNOWN";
}

}  // namespace px4_odometry_bridge
