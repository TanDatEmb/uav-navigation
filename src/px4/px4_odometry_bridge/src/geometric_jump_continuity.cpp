#include "px4_odometry_bridge/geometric_jump_continuity.hpp"

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
  if (!state.last_received->frame_valid) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kPreviousFrameInvalid,
                  true, state);
  }
  if (state.last_generation != generation) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kGenerationChanged,
                  true, state);
  }

  const std::int64_t dt_ns = current.timestamp_ns - state.last_received->timestamp_ns;
  if (dt_ns <= 0) {
    return reseed(current, generation_valid, generation,
                  GeometricJumpContinuityReason::kTimestampNotIncreasing,
                  true, state);
  }

  GeometricJumpContinuityObservation observation;
  observation.dt_s = static_cast<double>(dt_ns) * 1e-9;
  if (observation.dt_s < config.minimum_continuity_dt_s) {
    observation = reseed(current, generation_valid, generation,
                         GeometricJumpContinuityReason::kDtTooSmall,
                         true, state);
    observation.dt_s = static_cast<double>(dt_ns) * 1e-9;
    return observation;
  }
  if (observation.dt_s > config.maximum_continuity_dt_s) {
    observation = reseed(current, generation_valid, generation,
                         GeometricJumpContinuityReason::kDtTooLarge,
                         true, state);
    observation.dt_s = static_cast<double>(dt_ns) * 1e-9;
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
