#include <gtest/gtest.h>

#include "fast_lio_core/common/constants.hpp"
#include "fast_lio_core/synchronization/measurement_synchronizer.hpp"

namespace uav::nav::lio {
namespace {

LidarScan syncScan(std::int64_t start_ns, std::int64_t end_ns,
                   ClockDomain domain = ClockDomain::kSensorTime) {
  LidarScan scan;
  scan.start_time = Timestamp(start_ns, domain);
  scan.end_time = Timestamp(end_ns, domain);
  scan.points.push_back(LidarPoint{Eigen::Vector3f(1.0F, 0.0F, 0.0F), 0, 0, 0, 0});
  return scan;
}

ImuSample syncImu(std::int64_t time_ns, ClockDomain domain = ClockDomain::kSensorTime) {
  ImuSample sample;
  sample.time = Timestamp(time_ns, domain);
  sample.linear_acceleration_imu_m_s2 = Eigen::Vector3d(0.0, 0.0, kStandardGravityMps2);
  return sample;
}

TEST(MeasurementSynchronizerTest, IncludesBothImuBrackets) {
  MeasurementBuffer buffer;
  ASSERT_TRUE(buffer.pushLidar(syncScan(100, 200)).ok());
  for (const auto time_ns : {90, 110, 150, 210, 250}) {
    ASSERT_TRUE(buffer.pushImu(syncImu(time_ns)).ok());
  }
  MeasurementSynchronizerConfig config;
  config.maximum_imu_gap_ns = 100;
  MeasurementSynchronizer synchronizer(config);
  const auto result = synchronizer.synchronizeNext(buffer);
  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.value().has_value());
  const MeasurementGroup& group = *result.value();
  ASSERT_TRUE(group.fullyBracketed());
  ASSERT_EQ(group.imu_samples.size(), 4U);
  EXPECT_EQ(group.imu_samples.front().time.nanoseconds(), 90);
  EXPECT_EQ(group.imu_samples.back().time.nanoseconds(), 210);
  EXPECT_EQ(group.propagation_start_time.nanoseconds(), 100);
  EXPECT_EQ(buffer.imuSize(), 3U);
}

TEST(MeasurementSynchronizerTest, IncludesInterScanPropagationInterval) {
  MeasurementBuffer buffer;
  ASSERT_TRUE(buffer.pushLidar(syncScan(100, 200)).ok());
  ASSERT_TRUE(buffer.pushLidar(syncScan(300, 400)).ok());
  for (const auto time_ns : {90, 150, 210, 250, 310, 410, 450}) {
    ASSERT_TRUE(buffer.pushImu(syncImu(time_ns)).ok());
  }
  MeasurementSynchronizerConfig config;
  config.maximum_imu_gap_ns = 100;
  MeasurementSynchronizer synchronizer(config);
  const auto first = synchronizer.synchronizeNext(buffer);
  ASSERT_TRUE(first.ok());
  ASSERT_TRUE(first.value().has_value());
  const auto second = synchronizer.synchronizeNext(buffer);
  ASSERT_TRUE(second.ok());
  ASSERT_TRUE(second.value().has_value());
  EXPECT_EQ(second.value()->propagation_start_time.nanoseconds(), 200);
  EXPECT_LE(second.value()->imu_samples.front().time.nanoseconds(), 200);
  EXPECT_GE(second.value()->imu_samples.back().time.nanoseconds(), 400);
  bool contains_gap_sample = false;
  for (const auto& sample : second.value()->imu_samples) {
    contains_gap_sample |= sample.time.nanoseconds() == 250;
  }
  EXPECT_TRUE(contains_gap_sample);
}

TEST(MeasurementSynchronizerTest, WaitsForEndBracket) {
  MeasurementBuffer buffer;
  ASSERT_TRUE(buffer.pushLidar(syncScan(100, 200)).ok());
  ASSERT_TRUE(buffer.pushImu(syncImu(90)).ok());
  ASSERT_TRUE(buffer.pushImu(syncImu(150)).ok());
  MeasurementSynchronizer synchronizer;
  const auto result = synchronizer.synchronizeNext(buffer);
  ASSERT_TRUE(result.ok());
  EXPECT_FALSE(result.value().has_value());
  EXPECT_EQ(buffer.lidarSize(), 1U);
}

TEST(MeasurementSynchronizerTest, RejectsPermanentlyMissingStartBracket) {
  MeasurementBuffer buffer;
  ASSERT_TRUE(buffer.pushLidar(syncScan(100, 200)).ok());
  ASSERT_TRUE(buffer.pushImu(syncImu(110)).ok());
  ASSERT_TRUE(buffer.pushImu(syncImu(210)).ok());
  MeasurementSynchronizer synchronizer;
  const auto result = synchronizer.synchronizeNext(buffer);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kMissingStartBracket);
  EXPECT_EQ(buffer.lidarSize(), 0U);
}

TEST(MeasurementSynchronizerTest, RejectsExcessiveImuGap) {
  MeasurementBuffer buffer;
  ASSERT_TRUE(buffer.pushLidar(syncScan(100, 200)).ok());
  ASSERT_TRUE(buffer.pushImu(syncImu(90)).ok());
  ASSERT_TRUE(buffer.pushImu(syncImu(210)).ok());
  MeasurementSynchronizerConfig config;
  config.maximum_imu_gap_ns = 50;
  MeasurementSynchronizer synchronizer(config);
  const auto result = synchronizer.synchronizeNext(buffer);
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(synchronizer.stats().rejected_imu_gap, 1U);
  EXPECT_EQ(result.status().code(), StatusCode::kImuGap);
}

TEST(MeasurementSynchronizerTest, Mid360ConsecutiveScansRemainBelowGapLimit) {
  MeasurementBuffer buffer;
  constexpr std::int64_t kImuPeriod = 5'780'000;
  ASSERT_TRUE(buffer.pushLidar(syncScan(40'000'000, 72'635'000)).ok());
  ASSERT_TRUE(buffer.pushLidar(syncScan(74'000'000, 106'635'000)).ok());
  for (std::int64_t time = 35'000'000; time <= 112'000'000;
       time += kImuPeriod) {
    ASSERT_TRUE(buffer.pushImu(syncImu(time)).ok());
  }
  MeasurementSynchronizer synchronizer({20'000'000});
  ASSERT_TRUE(synchronizer.synchronizeNext(buffer).value().has_value());
  const auto second = synchronizer.synchronizeNext(buffer);
  ASSERT_TRUE(second.ok());
  ASSERT_TRUE(second.value().has_value());
  EXPECT_LT(second.value()->max_imu_gap_ns, 20'000'000);
}

TEST(MeasurementSynchronizerTest, LightOverlapIsDistinctAndNextScanStillSyncs) {
  MeasurementBuffer buffer;
  ASSERT_TRUE(buffer.pushLidar(syncScan(40'000'000, 72'635'000)).ok());
  ASSERT_TRUE(buffer.pushLidar(syncScan(72'500'000, 105'135'000)).ok());
  ASSERT_TRUE(buffer.pushLidar(syncScan(106'000'000, 138'635'000)).ok());
  for (std::int64_t time = 35'000'000; time <= 145'000'000;
       time += 5'780'000) {
    ASSERT_TRUE(buffer.pushImu(syncImu(time)).ok());
  }
  MeasurementSynchronizer synchronizer({20'000'000});
  ASSERT_TRUE(synchronizer.synchronizeNext(buffer).value().has_value());
  const auto overlap = synchronizer.synchronizeNext(buffer);
  ASSERT_FALSE(overlap.ok());
  EXPECT_EQ(overlap.status().code(), StatusCode::kScanOverlap);
  const auto following = synchronizer.synchronizeNext(buffer);
  ASSERT_TRUE(following.ok());
  ASSERT_TRUE(following.value().has_value());
  EXPECT_EQ(synchronizer.stats().rejected_imu_gap, 0U);
}

TEST(MeasurementSynchronizerTest, DelayedProcessingWithCompleteQueueDoesNotLoseImu) {
  MeasurementBuffer buffer;
  for (std::int64_t time = 0; time <= 250'000'000; time += 5'780'000) {
    ASSERT_TRUE(buffer.pushImu(syncImu(time)).ok());
  }
  ASSERT_TRUE(buffer.pushLidar(syncScan(100'000'000, 132'635'000)).ok());
  MeasurementSynchronizer synchronizer({20'000'000});
  const auto result = synchronizer.synchronizeNext(buffer);
  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.value().has_value());
  EXPECT_LT(result.value()->max_imu_gap_ns, 20'000'000);
}

TEST(MeasurementSynchronizerTest, PruningKeepsPropagationBoundaryBrackets) {
  MeasurementBuffer buffer;
  ASSERT_TRUE(buffer.pushLidar(syncScan(100, 200)).ok());
  ASSERT_TRUE(buffer.pushLidar(syncScan(230, 300)).ok());
  for (const auto time : {90, 150, 210, 250, 310}) {
    ASSERT_TRUE(buffer.pushImu(syncImu(time)).ok());
  }
  MeasurementSynchronizer synchronizer({100});
  ASSERT_TRUE(synchronizer.synchronizeNext(buffer).value().has_value());
  const auto second = synchronizer.synchronizeNext(buffer);
  ASSERT_TRUE(second.ok());
  ASSERT_TRUE(second.value().has_value());
  EXPECT_LE(second.value()->imu_samples.front().time.nanoseconds(), 200);
  EXPECT_GE(second.value()->imu_samples.back().time.nanoseconds(), 300);
}

TEST(MeasurementSynchronizerTest, ResetClearsPriorScanEpoch) {
  MeasurementBuffer first_buffer;
  ASSERT_TRUE(first_buffer.pushLidar(syncScan(100, 200)).ok());
  ASSERT_TRUE(first_buffer.pushImu(syncImu(90)).ok());
  ASSERT_TRUE(first_buffer.pushImu(syncImu(210)).ok());
  MeasurementSynchronizerConfig config;
  config.maximum_imu_gap_ns = 200;
  MeasurementSynchronizer synchronizer(config);
  ASSERT_TRUE(synchronizer.synchronizeNext(first_buffer).value().has_value());

  synchronizer.reset();
  MeasurementBuffer reset_buffer;
  ASSERT_TRUE(reset_buffer.pushLidar(syncScan(10, 20)).ok());
  ASSERT_TRUE(reset_buffer.pushImu(syncImu(9)).ok());
  ASSERT_TRUE(reset_buffer.pushImu(syncImu(21)).ok());
  const auto after_reset = synchronizer.synchronizeNext(reset_buffer);
  ASSERT_TRUE(after_reset.ok());
  ASSERT_TRUE(after_reset.value().has_value());
  EXPECT_EQ(after_reset.value()->propagation_start_time.nanoseconds(), 10);
}

}  // namespace
}  // namespace uav::nav::lio
