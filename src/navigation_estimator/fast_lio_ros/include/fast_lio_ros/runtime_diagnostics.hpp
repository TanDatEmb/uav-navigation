#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace uav::nav::lio {

struct RuntimeDiagnostics {
  std::size_t current_input_queue_depth{0};
  std::size_t current_imu_queue_depth{0};
  std::size_t current_lidar_queue_depth{0};
  std::size_t maximum_queue_depth{0};
  std::size_t maximum_imu_queue_depth{0};
  std::size_t maximum_lidar_queue_depth{0};
  std::size_t imu_queue_capacity{0};
  std::size_t lidar_queue_capacity{0};
  std::size_t received_imu_count{0};
  std::size_t received_lidar_count{0};
  std::size_t processed_imu_count{0};
  std::size_t processed_lidar_count{0};
  std::size_t imu_drop_count{0};
  std::size_t lidar_drop_count{0};
  std::size_t reject_reason_imu_buffer_full{0};
  std::size_t reject_reason_lidar_buffer_full{0};
  std::size_t reject_reason_nonfinite_xyz{0};
  std::size_t reject_reason_invalid_point_time{0};
  std::size_t reject_reason_too_few_points{0};
  std::size_t reject_reason_imu_gap{0};
  std::size_t reject_reason_timestamp_regression{0};
  std::size_t reject_reason_not_initialized{0};
  std::size_t reject_reason_correction_failed{0};
  std::int64_t latest_received_time_ns{0};
  std::int64_t latest_processed_time_ns{0};
  std::int64_t processing_lag_ns{0};
  std::size_t worker_heartbeat{0};
  std::int64_t worker_last_progress_wall_time_ns{0};
  std::int64_t last_scan_processing_us{0};
  double mean_scan_processing_us{0.0};
  std::int64_t p95_scan_processing_us{0};
  std::int64_t p99_scan_processing_us{0};
  double worker_busy_ratio{0.0};
  bool overflow_detected{false};
  bool processing_lag_exceeded{false};
};

class RuntimeStatistics {
 public:
  explicit RuntimeStatistics(
      std::chrono::steady_clock::time_point started_at = std::chrono::steady_clock::now())
      : started_at_(started_at) {}

  void recordScan(std::int64_t elapsed_us) {
    elapsed_us = std::max<std::int64_t>(0, elapsed_us);
    last_scan_processing_us_ = elapsed_us;
    scan_processing_sum_us_ += static_cast<double>(elapsed_us);
    ++scan_processing_count_;
    samples_us_.push_back(elapsed_us);
    if (samples_us_.size() > kMaximumSamples) {
      samples_us_.pop_front();
    }
  }

  void recordBusy(std::chrono::steady_clock::duration elapsed) { busy_duration_ += elapsed; }

  void populate(RuntimeDiagnostics& diagnostics, std::chrono::steady_clock::time_point now =
                                                     std::chrono::steady_clock::now()) const {
    diagnostics.last_scan_processing_us = last_scan_processing_us_;
    diagnostics.mean_scan_processing_us =
        scan_processing_count_ == 0
            ? 0.0
            : scan_processing_sum_us_ / static_cast<double>(scan_processing_count_);
    diagnostics.p95_scan_processing_us = percentile(0.95);
    diagnostics.p99_scan_processing_us = percentile(0.99);
    const auto wall = now - started_at_;
    diagnostics.worker_busy_ratio =
        wall <= std::chrono::steady_clock::duration::zero()
            ? 0.0
            : std::clamp(
                  static_cast<double>(
                      std::chrono::duration_cast<std::chrono::nanoseconds>(busy_duration_)
                          .count()) /
                      static_cast<double>(
                          std::chrono::duration_cast<std::chrono::nanoseconds>(wall).count()),
                  0.0, 1.0);
  }

 private:
  [[nodiscard]] std::int64_t percentile(double quantile) const {
    if (samples_us_.empty()) {
      return 0;
    }
    std::vector<std::int64_t> sorted(samples_us_.begin(), samples_us_.end());
    std::sort(sorted.begin(), sorted.end());
    const auto rank = static_cast<std::size_t>(quantile * static_cast<double>(sorted.size() - 1));
    return sorted[rank];
  }

  static constexpr std::size_t kMaximumSamples = 1024;
  std::chrono::steady_clock::time_point started_at_;
  std::chrono::steady_clock::duration busy_duration_{};
  std::deque<std::int64_t> samples_us_;
  std::int64_t last_scan_processing_us_{0};
  double scan_processing_sum_us_{0.0};
  std::size_t scan_processing_count_{0};
};

}  // namespace uav::nav::lio
