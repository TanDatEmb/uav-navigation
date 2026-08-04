#include "px4_odometry_bridge/external_odometry_gate.hpp"

namespace px4_odometry_bridge {

ExternalOdometryGateResult evaluate_external_odometry_gate(
    const ExternalOdometryGateInput& input) {
  ExternalOdometryGateResult result;
  result.node_ready = input.node_ready;
  result.transport_ready = input.transport_ready;
  result.timestamp_ready = input.timestamp_ready;
  result.covariance_ready = input.covariance_ready;
  result.supervisor_authorized = input.supervisor_authorized;
  result.public_frame_generation_valid = input.public_frame_generation_valid;
  result.corrected_propagated_fresh = input.corrected_propagated_fresh;
  result.supervisor_fresh = input.supervisor_fresh;
  result.frame_valid = input.frame_valid;
  result.geometric_jump_latched = input.geometric_jump_latched;
  result.publisher_ready = result.node_ready && result.transport_ready;
  result.publication_ready =
      result.publisher_ready && result.timestamp_ready &&
      result.covariance_ready && result.supervisor_authorized &&
      result.public_frame_generation_valid && result.corrected_propagated_fresh &&
      result.supervisor_fresh && result.frame_valid &&
      !result.geometric_jump_latched;
  if (result.publication_ready) {
    result.reason = "READY";
  } else if (result.geometric_jump_latched) {
    result.reason = "GEOMETRIC_JUMP_LATCHED";
  } else if (!result.supervisor_authorized) {
    result.reason = "SUPERVISOR_UNAUTHORIZED";
  } else if (!result.public_frame_generation_valid) {
    result.reason = "PUBLIC_FRAME_GENERATION_INVALID";
  } else if (!result.timestamp_ready) {
    result.reason = "TIMESTAMP_INVALID";
  } else if (!result.covariance_ready) {
    result.reason = "COVARIANCE_INVALID";
  } else if (!result.corrected_propagated_fresh) {
    result.reason = "LIO_ODOMETRY_STALE";
  } else if (!result.supervisor_fresh) {
    result.reason = "SUPERVISOR_STALE";
  } else if (!result.frame_valid) {
    result.reason = "FRAME_INVALID";
  } else if (!result.transport_ready) {
    result.reason = "PX4_INPUT_TRANSPORT_NOT_READY";
  } else {
    result.reason = "NODE_NOT_READY";
  }
  return result;
}

}  // namespace px4_odometry_bridge
