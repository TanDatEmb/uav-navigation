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
  EXPECT_EQ(diagnostics.scan_processing_count, 100U);
  EXPECT_EQ(diagnostics.p50_scan_processing_us, 50);
  EXPECT_EQ(diagnostics.p95_scan_processing_us, 95);
  EXPECT_EQ(diagnostics.p99_scan_processing_us, 99);
  EXPECT_EQ(diagnostics.maximum_scan_processing_us, 100);
  EXPECT_DOUBLE_EQ(diagnostics.worker_busy_ratio, 0.25);
}

TEST(RuntimeStatisticsTest, EmptyStatisticsAreZero) {
  const auto start = std::chrono::steady_clock::time_point{};
  RuntimeStatistics statistics(start);
  RuntimeDiagnostics diagnostics;
  statistics.populate(diagnostics, start);
  EXPECT_EQ(diagnostics.p95_scan_processing_us, 0);
  EXPECT_EQ(diagnostics.p99_scan_processing_us, 0);
  EXPECT_EQ(diagnostics.scan_processing_count, 0U);
  EXPECT_DOUBLE_EQ(diagnostics.worker_busy_ratio, 0.0);
}

TEST(RuntimeStatisticsTest, OneSampleIsUsedForEveryPercentile) {
  RuntimeStatistics statistics;
  statistics.recordScan(42);
  RuntimeDiagnostics diagnostics;
  statistics.populate(diagnostics);
  EXPECT_EQ(diagnostics.p50_scan_processing_us, 42);
  EXPECT_EQ(diagnostics.p95_scan_processing_us, 42);
  EXPECT_EQ(diagnostics.p99_scan_processing_us, 42);
  EXPECT_EQ(diagnostics.maximum_scan_processing_us, 42);
}

TEST(RuntimeStatisticsTest, PercentilesAreIndependentOfInputOrder) {
  RuntimeStatistics ordered;
  RuntimeStatistics reversed;
  for (std::int64_t value = 1; value <= 100; ++value) {
    ordered.recordScan(value);
    reversed.recordScan(101 - value);
  }
  RuntimeDiagnostics first;
  RuntimeDiagnostics second;
  ordered.populate(first);
  reversed.populate(second);
  EXPECT_EQ(first.p50_scan_processing_us, second.p50_scan_processing_us);
  EXPECT_EQ(first.p95_scan_processing_us, second.p95_scan_processing_us);
  EXPECT_EQ(first.p99_scan_processing_us, second.p99_scan_processing_us);
  EXPECT_DOUBLE_EQ(first.mean_scan_processing_us, second.mean_scan_processing_us);
}

TEST(RuntimeStatisticsTest, FullRunPopulationDoesNotDropSamplesAfter1024) {
  RuntimeStatistics statistics;
  for (std::int64_t value = 1; value <= 2'000; ++value) {
    statistics.recordScan(value);
  }
  RuntimeDiagnostics diagnostics;
  statistics.populate(diagnostics);
  EXPECT_EQ(diagnostics.scan_processing_count, 2'000U);
  EXPECT_DOUBLE_EQ(diagnostics.mean_scan_processing_us, 1'000.5);
  EXPECT_EQ(diagnostics.p95_scan_processing_us, 1'900);
  EXPECT_EQ(diagnostics.p99_scan_processing_us, 1'980);
  EXPECT_EQ(diagnostics.maximum_scan_processing_us, 2'000);
}

TEST(RuntimeStatisticsTest, OutlierAffectsP99AndMaximum) {
  RuntimeStatistics statistics;
  for (int index = 0; index < 98; ++index) statistics.recordScan(10);
  statistics.recordScan(10'000);
  statistics.recordScan(10'000);
  RuntimeDiagnostics diagnostics;
  statistics.populate(diagnostics);
  EXPECT_EQ(diagnostics.p99_scan_processing_us, 10'000);
  EXPECT_EQ(diagnostics.maximum_scan_processing_us, 10'000);
}

TEST(RuntimeStatisticsTest, SnapshotsDoNotResetTheDistribution) {
  RuntimeStatistics statistics;
  statistics.recordScan(10);
  RuntimeDiagnostics first;
  statistics.populate(first);
  statistics.recordScan(20);
  RuntimeDiagnostics second;
  statistics.populate(second);
  EXPECT_EQ(first.scan_processing_count, 1U);
  EXPECT_EQ(second.scan_processing_count, 2U);
  EXPECT_DOUBLE_EQ(second.mean_scan_processing_us, 15.0);
}

TEST(RuntimeStatisticsTest, TracksSeparatedLatencyPopulations) {
  RuntimeStatistics statistics;
  statistics.recordPipelinePushLidar(7);
  statistics.recordPipelinePushLidar(9);
  statistics.recordResultProcessing(100);
  statistics.recordCorrectedScanEndToEnd(500);
  statistics.recordRegistrationUpdate(80);
  RuntimeDiagnostics diagnostics;
  statistics.populate(diagnostics);
  EXPECT_EQ(diagnostics.pipeline_push_lidar_count, 2U);
  EXPECT_DOUBLE_EQ(diagnostics.mean_pipeline_push_lidar_us, 8.0);
  EXPECT_EQ(diagnostics.result_processing_count, 1U);
  EXPECT_EQ(diagnostics.p95_corrected_scan_end_to_end_us, 500);
  EXPECT_EQ(diagnostics.registration_update_count, 1U);
  EXPECT_EQ(diagnostics.p99_registration_update_us, 80);
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
