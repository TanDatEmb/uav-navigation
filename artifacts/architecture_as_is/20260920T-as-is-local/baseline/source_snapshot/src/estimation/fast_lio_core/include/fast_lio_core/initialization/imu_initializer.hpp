#pragma once

#include <cstddef>
#include <deque>
#include <optional>

#include "fast_lio_core/common/constants.hpp"
#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/initialization/initialization_result.hpp"
#include "fast_lio_core/sensor/imu_sample.hpp"
#include "fast_lio_core/time/timestamp_validator.hpp"

namespace uav::nav::lio {

struct ImuInitializerConfig {
  std::size_t minimum_imu_samples{200};
  std::size_t maximum_imu_samples{1000};
  bool require_stationary{true};
  double gravity_magnitude_m_s2{kStandardGravityMps2};
  double maximum_gyro_mean_norm_rad_s{0.1};
  double maximum_gyro_variance_rad2_s2{1e-3};
  double maximum_accel_variance_m2_s4{0.1};
  double maximum_gravity_norm_error_m_s2{1.0};
};

class ImuInitializer {
 public:
  explicit ImuInitializer(ImuInitializerConfig config = {});

  [[nodiscard]] Status addSample(const ImuSample& sample);
  [[nodiscard]] Result<InitializationResult> tryInitialize() const;
  [[nodiscard]] InitializationQuality quality() const;
  [[nodiscard]] std::size_t sampleCount() const noexcept;
  [[nodiscard]] bool hasEnoughSamples() const noexcept;
  void reset();

 private:
  ImuInitializerConfig config_;
  std::deque<ImuSample> samples_;
  TimestampValidator timestamp_validator_;
};

}  // namespace uav::nav::lio
