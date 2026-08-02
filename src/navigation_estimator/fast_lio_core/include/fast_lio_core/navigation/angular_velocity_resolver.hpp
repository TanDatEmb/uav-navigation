#pragma once

#include <cstddef>
#include <span>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/navigation/kinematic_state_estimate.hpp"
#include "fast_lio_core/sensor/imu_sample.hpp"

namespace uav::nav::lio {

struct AngularVelocityDiagnostics {
  bool angular_velocity_available{false};
  std::size_t exact_sample_count{0U};
  std::size_t interpolated_count{0U};
  std::size_t missing_bracket_count{0U};
  std::size_t timestamp_mismatch_count{0U};
  std::size_t nonfinite_reject_count{0U};
};

class AngularVelocityResolver {
 public:
  [[nodiscard]] static Result<KinematicStateEstimate> resolve(
      const StateEstimate& estimate, std::span<const ImuSample> samples,
      AngularVelocityDiagnostics* diagnostics = nullptr);

  [[nodiscard]] static Result<KinematicStateEstimate> resolveExact(
      const StateEstimate& estimate, const ImuSample& sample,
      AngularVelocityDiagnostics* diagnostics = nullptr);
};

}  // namespace uav::nav::lio
