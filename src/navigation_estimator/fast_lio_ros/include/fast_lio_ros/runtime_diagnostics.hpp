#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "fast_lio_core/navigation/base_link_covariance_projector.hpp"

namespace uav::nav::lio {

struct CovarianceProjectionRuntimeSnapshot {
  bool pose_covariance_available{false};
  bool twist_covariance_available{false};
  std::size_t projection_success_count{0};
  std::size_t projection_failure_count{0};
  std::size_t source_nonfinite_count{0};
  std::size_t source_asymmetry_count{0};
  std::size_t source_non_psd_count{0};
  std::size_t source_zero_count{0};
  std::size_t output_pose_nonfinite_count{0};
  std::size_t output_twist_nonfinite_count{0};
  std::size_t output_pose_non_psd_count{0};
  std::size_t output_twist_non_psd_count{0};
  std::size_t roundoff_repair_count{0};
  double pose_covariance_trace{0.0};
  double twist_covariance_trace{0.0};
  double pose_covariance_minimum_eigenvalue{0.0};
  double twist_covariance_minimum_eigenvalue{0.0};
  std::int64_t covariance_projection_us{0};
  std::int64_t maximum_covariance_projection_us{0};
  double mean_covariance_projection_us{0.0};
};

class CovarianceProjectionRuntime {
 public:
  void record(const BaseLinkCovarianceProjectionDiagnostics& projection,
              std::int64_t elapsed_us) noexcept {
    elapsed_us = std::max<std::int64_t>(0, elapsed_us);
    projection_time_sum_us_.fetch_add(static_cast<std::size_t>(elapsed_us));
    projection_count_.fetch_add(1U);
    covariance_projection_us_.store(elapsed_us);
    auto maximum = maximum_covariance_projection_us_.load();
    while (maximum < elapsed_us &&
           !maximum_covariance_projection_us_.compare_exchange_weak(
               maximum, elapsed_us)) {
    }
    if (projection.success) {
      projection_success_count_.fetch_add(1U);
    } else {
      projection_failure_count_.fetch_add(1U);
    }
    source_nonfinite_count_.fetch_add(projection.source_nonfinite ? 1U : 0U);
    source_asymmetry_count_.fetch_add(projection.source_asymmetry ? 1U : 0U);
    source_non_psd_count_.fetch_add(projection.source_non_psd ? 1U : 0U);
    source_zero_count_.fetch_add(projection.source_zero ? 1U : 0U);
    output_pose_nonfinite_count_.fetch_add(
        projection.output_pose_nonfinite ? 1U : 0U);
    output_twist_nonfinite_count_.fetch_add(
        projection.output_twist_nonfinite ? 1U : 0U);
    output_pose_non_psd_count_.fetch_add(
        projection.output_pose_non_psd ? 1U : 0U);
    output_twist_non_psd_count_.fetch_add(
        projection.output_twist_non_psd ? 1U : 0U);
    roundoff_repair_count_.fetch_add(projection.roundoff_repair ? 1U : 0U);
    pose_covariance_available_.store(projection.success);
    twist_covariance_available_.store(projection.success);
    pose_covariance_trace_.store(projection.pose_covariance_trace);
    twist_covariance_trace_.store(projection.twist_covariance_trace);
    pose_covariance_minimum_eigenvalue_.store(
        projection.pose_covariance_minimum_eigenvalue);
    twist_covariance_minimum_eigenvalue_.store(
        projection.twist_covariance_minimum_eigenvalue);
  }

  [[nodiscard]] CovarianceProjectionRuntimeSnapshot snapshot() const noexcept {
    const auto count = projection_count_.load();
    return CovarianceProjectionRuntimeSnapshot{
        pose_covariance_available_.load(),
        twist_covariance_available_.load(),
        projection_success_count_.load(),
        projection_failure_count_.load(),
        source_nonfinite_count_.load(),
        source_asymmetry_count_.load(),
        source_non_psd_count_.load(),
        source_zero_count_.load(),
        output_pose_nonfinite_count_.load(),
        output_twist_nonfinite_count_.load(),
        output_pose_non_psd_count_.load(),
        output_twist_non_psd_count_.load(),
        roundoff_repair_count_.load(),
        pose_covariance_trace_.load(),
        twist_covariance_trace_.load(),
        pose_covariance_minimum_eigenvalue_.load(),
        twist_covariance_minimum_eigenvalue_.load(),
        covariance_projection_us_.load(),
        maximum_covariance_projection_us_.load(),
        count == 0U
            ? 0.0
            : static_cast<double>(projection_time_sum_us_.load()) /
                  static_cast<double>(count)};
  }

 private:
  std::atomic<std::size_t> projection_count_{0};
  std::atomic<std::size_t> projection_time_sum_us_{0};
  std::atomic<std::int64_t> covariance_projection_us_{0};
  std::atomic<std::int64_t> maximum_covariance_projection_us_{0};
  std::atomic<std::size_t> projection_success_count_{0};
  std::atomic<std::size_t> projection_failure_count_{0};
  std::atomic<std::size_t> source_nonfinite_count_{0};
  std::atomic<std::size_t> source_asymmetry_count_{0};
  std::atomic<std::size_t> source_non_psd_count_{0};
  std::atomic<std::size_t> source_zero_count_{0};
  std::atomic<std::size_t> output_pose_nonfinite_count_{0};
  std::atomic<std::size_t> output_twist_nonfinite_count_{0};
  std::atomic<std::size_t> output_pose_non_psd_count_{0};
  std::atomic<std::size_t> output_twist_non_psd_count_{0};
  std::atomic<std::size_t> roundoff_repair_count_{0};
  std::atomic<bool> pose_covariance_available_{false};
  std::atomic<bool> twist_covariance_available_{false};
  std::atomic<double> pose_covariance_trace_{0.0};
  std::atomic<double> twist_covariance_trace_{0.0};
  std::atomic<double> pose_covariance_minimum_eigenvalue_{0.0};
  std::atomic<double> twist_covariance_minimum_eigenvalue_{0.0};
};

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
  bool static_geometry_ready{false};
  std::string static_geometry_source;
  std::string dynamic_tf_owner;
  std::size_t dynamic_tf_publication_count{0};
  std::size_t dynamic_tf_timestamp_suppressed_count{0};
  std::size_t dynamic_tf_conversion_failure_count{0};
  CovarianceProjectionRuntimeSnapshot covariance_projection;
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
