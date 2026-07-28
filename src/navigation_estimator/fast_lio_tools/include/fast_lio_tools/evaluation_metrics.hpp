#pragma once

#include <cstddef>
#include <limits>

namespace uav::nav::lio {

struct EvaluationMetrics {
  std::size_t imu_sample_count{};
  std::size_t lidar_scan_count{};
  std::size_t successful_correction_count{};
  std::size_t rejected_scan_count{};
  double maximum_imu_gap_ms{};
  double residual_rms{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace uav::nav::lio
