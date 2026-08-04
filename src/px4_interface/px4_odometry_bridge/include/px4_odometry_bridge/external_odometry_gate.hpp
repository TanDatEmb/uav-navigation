#pragma once

#include <string>

namespace px4_odometry_bridge {

struct ExternalOdometryGateInput {
  bool node_ready{false};
  bool transport_ready{false};
  bool timestamp_ready{false};
  bool covariance_ready{false};
  bool supervisor_authorized{false};
  bool public_frame_generation_valid{false};
  bool corrected_propagated_fresh{false};
  bool supervisor_fresh{false};
  bool frame_valid{false};
  bool geometric_jump_latched{false};
};

struct ExternalOdometryGateResult {
  bool node_ready{false};
  bool transport_ready{false};
  bool timestamp_ready{false};
  bool covariance_ready{false};
  bool supervisor_authorized{false};
  bool public_frame_generation_valid{false};
  bool corrected_propagated_fresh{false};
  bool supervisor_fresh{false};
  bool frame_valid{false};
  bool geometric_jump_latched{false};
  bool publisher_ready{false};
  bool publication_ready{false};
  std::string reason{"GATE_CLOSED"};
};

[[nodiscard]] ExternalOdometryGateResult evaluate_external_odometry_gate(
    const ExternalOdometryGateInput& input);

}  // namespace px4_odometry_bridge
