#pragma once

#include <cstdint>
#include <optional>

#include "px4_odometry_bridge/external_odometry_conversion.hpp"

namespace px4_odometry_bridge {

struct GeometricJumpContinuityConfig {
  double position_jump_margin_m{0.75};
  double orientation_jump_margin_rad{0.35};
  double maximum_expected_speed_mps{10.0};
  double maximum_expected_angular_rate_rad_s{6.0};
  double minimum_continuity_dt_s{1e-4};
  double maximum_continuity_dt_s{0.5};
};

enum class GeometricJumpContinuityReason {
  kNoBaseline,
  kSourceInvalid,
  kGenerationInvalid,
  kGenerationChanged,
  kPreviousFrameInvalid,
  kCurrentFrameInvalid,
  kTimestampNotIncreasing,
  kDtTooSmall,
  kDtTooLarge,
  kWithinThreshold,
  kJumpDetected,
};

struct GeometricJumpContinuityObservation {
  bool evaluated{false};
  bool jumped{false};
  bool baseline_reseeded{false};
  GeometricJumpContinuityReason reason{GeometricJumpContinuityReason::kNoBaseline};
  double dt_s{0.0};
  double position_delta_m{0.0};
  double allowed_position_delta_m{0.0};
  double orientation_delta_rad{0.0};
  double allowed_orientation_delta_rad{0.0};
};

struct GeometricJumpContinuityState {
  std::optional<ExternalOdometryFrame> last_received;
  std::uint64_t last_generation{0};
  bool last_generation_valid{false};
  bool continuity_trusted{false};
};

[[nodiscard]] GeometricJumpContinuityObservation observe_geometric_jump_continuity(
    const ExternalOdometryFrame& current, bool source_valid,
    bool generation_valid, std::uint64_t generation,
    const GeometricJumpContinuityConfig& config,
    GeometricJumpContinuityState& state);

[[nodiscard]] const char* to_string(
    GeometricJumpContinuityReason reason) noexcept;

}  // namespace px4_odometry_bridge
