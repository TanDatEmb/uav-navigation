#pragma once

#include <cstdint>
#include <optional>

#include <navigation_common/time.hpp>
#include <navigation_contracts/msg/estimator_health.hpp>

namespace px4_odometry_bridge {

struct ExternalLioHealthSnapshot {
  std::int64_t source_stamp_ns{0};
  std::uint64_t public_frame_generation{0};
  bool public_frame_generation_valid{false};
  bool lio_valid{false};
  bool covariance_valid{false};
};

// Source time is a monotonic high-water mark for the typed health stream.
// Rejected samples must not overwrite the last accepted state or its
// freshness timestamp; a newer invalid sample remains an accepted fail-closed
// state.
[[nodiscard]] inline bool externalLioHealthIsNewer(
    const std::optional<ExternalLioHealthSnapshot>& health,
    std::int64_t last_source_stamp_ns) noexcept {
  return health.has_value() &&
         health->source_stamp_ns > last_source_stamp_ns;
}

// The typed health contract is the sole authority for the LIO public-frame
// generation and the gate state used by the external-vision bridge. The
// DiagnosticArray topic remains observational and must not be joined with a
// propagated sample to manufacture a reset counter.
[[nodiscard]] inline std::optional<ExternalLioHealthSnapshot>
externalLioHealthSnapshot(
    const navigation_contracts::msg::EstimatorHealth& message) {
  const auto source_stamp_ns = navigation_common::rosTimeToNanoseconds(
      message.header.stamp);
  if (!source_stamp_ns || *source_stamp_ns <= 0 || message.localization_epoch == 0U) {
    return std::nullopt;
  }

  ExternalLioHealthSnapshot snapshot;
  snapshot.source_stamp_ns = *source_stamp_ns;
  snapshot.public_frame_generation = message.localization_epoch;
  snapshot.public_frame_generation_valid = true;
  snapshot.lio_valid =
      message.state == navigation_contracts::msg::EstimatorHealth::TRACKING &&
      message.navigation_valid && message.observability_valid &&
      message.correction_fresh && message.propagation_valid;
  snapshot.covariance_valid = message.covariance_valid;
  return snapshot;
}

}  // namespace px4_odometry_bridge
