#pragma once

#include <cstdint>
#include <vector>

#include "fast_lio_core/sensor/imu_sample.hpp"
#include "fast_lio_core/sensor/lidar_scan.hpp"

namespace uav::nav::lio {

struct MeasurementGroup {
  LidarScan scan;
  std::vector<ImuSample> imu_samples;
  // The estimator advances from this time through scan.end_time. It is scan
  // start for the first group and the prior synchronized scan end afterwards,
  // so inter-scan motion is never skipped.
  Timestamp propagation_start_time;
  bool has_start_bracket{false};
  bool has_end_bracket{false};
  std::int64_t max_imu_gap_ns{0};

  [[nodiscard]] bool fullyBracketed() const noexcept {
    return has_start_bracket && has_end_bracket;
  }
};

}  // namespace uav::nav::lio
