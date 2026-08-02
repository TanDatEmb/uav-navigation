#include <gtest/gtest.h>

#include <chrono>

#include "fast_lio_ros/runtime_diagnostics.hpp"

namespace uav::nav::lio {
namespace {

TEST(RuntimeStatisticsTest, ReportsScanPercentilesMeanAndBusyRatio) {
  const auto start = std::chrono::steady_clock::time_point{};
  RuntimeStatistics statistics(start);
  for (std::int64_t sample = 1; sample <= 100; ++sample) {
    statistics.recordScan(sample);
  }
  statistics.recordBusy(std::chrono::milliseconds(250));

  RuntimeDiagnostics diagnostics;
  statistics.populate(diagnostics, start + std::chrono::seconds(1));

  EXPECT_EQ(diagnostics.last_scan_processing_us, 100);
  EXPECT_DOUBLE_EQ(diagnostics.mean_scan_processing_us, 50.5);
  EXPECT_EQ(diagnostics.p95_scan_processing_us, 95);
  EXPECT_EQ(diagnostics.p99_scan_processing_us, 99);
  EXPECT_DOUBLE_EQ(diagnostics.worker_busy_ratio, 0.25);
}

TEST(RuntimeStatisticsTest, EmptyStatisticsAreZero) {
  const auto start = std::chrono::steady_clock::time_point{};
  RuntimeStatistics statistics(start);
  RuntimeDiagnostics diagnostics;
  statistics.populate(diagnostics, start);
  EXPECT_EQ(diagnostics.p95_scan_processing_us, 0);
  EXPECT_EQ(diagnostics.p99_scan_processing_us, 0);
  EXPECT_DOUBLE_EQ(diagnostics.worker_busy_ratio, 0.0);
}

TEST(CovarianceProjectionRuntimeTest, RecordsAvailabilityFailuresAndTiming) {
  CovarianceProjectionRuntime runtime;
  BaseLinkCovarianceProjectionDiagnostics success;
  success.success = true;
  success.pose_covariance_trace = 12.0;
  success.twist_covariance_trace = 8.0;
  success.pose_covariance_minimum_eigenvalue = 0.2;
  success.twist_covariance_minimum_eigenvalue = 0.3;
  runtime.record(success, 17);

  BaseLinkCovarianceProjectionDiagnostics failure;
  failure.source_non_psd = true;
  failure.source_zero = true;
  failure.output_pose_non_psd = true;
  runtime.record(failure, 23);

  const auto snapshot = runtime.snapshot();
  EXPECT_FALSE(snapshot.pose_covariance_available);
  EXPECT_FALSE(snapshot.twist_covariance_available);
  EXPECT_EQ(snapshot.projection_success_count, 1U);
  EXPECT_EQ(snapshot.projection_failure_count, 1U);
  EXPECT_EQ(snapshot.source_non_psd_count, 1U);
  EXPECT_EQ(snapshot.source_zero_count, 1U);
  EXPECT_EQ(snapshot.output_pose_non_psd_count, 1U);
  EXPECT_EQ(snapshot.covariance_projection_us, 23);
  EXPECT_EQ(snapshot.maximum_covariance_projection_us, 23);
  EXPECT_DOUBLE_EQ(snapshot.mean_covariance_projection_us, 20.0);
  EXPECT_DOUBLE_EQ(snapshot.pose_covariance_trace, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.twist_covariance_trace, 0.0);
}

}  // namespace
}  // namespace uav::nav::lio
