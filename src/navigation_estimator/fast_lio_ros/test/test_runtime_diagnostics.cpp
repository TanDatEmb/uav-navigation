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

}  // namespace
}  // namespace uav::nav::lio
