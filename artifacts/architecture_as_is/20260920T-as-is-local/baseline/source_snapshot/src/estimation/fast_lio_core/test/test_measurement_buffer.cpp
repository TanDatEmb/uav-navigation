#include <gtest/gtest.h>

#include "fast_lio_core/common/constants.hpp"
#include "fast_lio_core/synchronization/measurement_buffer.hpp"
#include "fast_lio_core/synchronization/measurement_synchronizer.hpp"

namespace uav::nav::lio {
namespace {

LidarScan makeScan(std::int64_t start_ns, std::int64_t end_ns,
                   ClockDomain domain = ClockDomain::kSensorTime) {
  LidarScan scan;
  scan.start_time = Timestamp(start_ns, domain);
  scan.end_time = Timestamp(end_ns, domain);
  scan.points.push_back(LidarPoint{Eigen::Vector3f(1.0F, 0.0F, 0.0F), 0, 0, 0, 0});
  return scan;
}

ImuSample makeImu(std::int64_t time_ns, ClockDomain domain = ClockDomain::kSensorTime) {
  ImuSample sample;
  sample.time = Timestamp(time_ns, domain);
  sample.linear_acceleration_imu_m_s2 = Eigen::Vector3d(0.0, 0.0, kStandardGravityMps2);
  return sample;
}

TEST(MeasurementBufferTest, AcceptsOrderedStreams) {
  MeasurementBuffer buffer;
  EXPECT_TRUE(buffer.pushLidar(makeScan(10, 20)).ok());
  EXPECT_TRUE(buffer.pushImu(makeImu(9)).ok());
  EXPECT_EQ(buffer.lidarSize(), 1U);
  EXPECT_EQ(buffer.imuSize(), 1U);
  EXPECT_EQ(buffer.stats().accepted_lidar_scans, 1U);
  EXPECT_EQ(buffer.stats().accepted_imu_samples, 1U);
}

TEST(MeasurementBufferTest, AcceptsZeroDurationSimultaneousScan) {
  MeasurementBuffer buffer;
  const auto status = buffer.pushLidar(makeScan(10, 10));
  EXPECT_TRUE(status.ok()) << status.message();
  EXPECT_EQ(buffer.lidarSize(), 1U);
}

TEST(MeasurementBufferTest, RejectsInvalidCapacityAndEpoch) {
  MeasurementBufferConfig config;
  config.maximum_lidar_scans = 0;
  EXPECT_THROW(
      { MeasurementBuffer buffer(config); }, std::invalid_argument);

  MeasurementBuffer buffer;
  EXPECT_EQ(buffer.pushImu(makeImu(0)).code(), StatusCode::kInvalidArgument);
  EXPECT_EQ(buffer.pushLidar(makeScan(0, 20)).code(), StatusCode::kInvalidArgument);
}

TEST(MeasurementBufferTest, RejectsTimestampRegression) {
  MeasurementBuffer buffer;
  ASSERT_TRUE(buffer.pushImu(makeImu(100)).ok());
  const Status status = buffer.pushImu(makeImu(99));
  EXPECT_EQ(status.code(), StatusCode::kTimestampRegression);
  EXPECT_EQ(buffer.imuSize(), 1U);
  EXPECT_EQ(buffer.stats().timestamp_regressions, 1U);
}

TEST(MeasurementBufferTest, RejectsClockDomainChangeWithinStream) {
  MeasurementBuffer buffer;
  ASSERT_TRUE(buffer.pushImu(makeImu(100)).ok());
  const Status status = buffer.pushImu(makeImu(101, ClockDomain::kRosTime));
  EXPECT_EQ(status.code(), StatusCode::kClockDomainMismatch);
  EXPECT_EQ(buffer.stats().clock_domain_mismatches, 1U);
}

TEST(MeasurementBufferTest, RejectsWhenCapacityIsReached) {
  MeasurementBufferConfig config;
  config.maximum_lidar_scans = 1;
  MeasurementBuffer buffer(config);
  ASSERT_TRUE(buffer.pushLidar(makeScan(10, 20)).ok());
  const Status status = buffer.pushLidar(makeScan(30, 40));
  EXPECT_EQ(status.code(), StatusCode::kBufferFull);
  EXPECT_EQ(buffer.stats().buffer_full_rejections, 1U);
}

TEST(MeasurementBufferTest, PermittedRegressionKeepsQueueOrderedForSync) {
  MeasurementBufferConfig buffer_config;
  buffer_config.reject_timestamp_regression = false;
  MeasurementBuffer buffer(buffer_config);
  ASSERT_TRUE(buffer.pushLidar(makeScan(100, 200)).ok());
  ASSERT_TRUE(buffer.pushImu(makeImu(210)).ok());
  ASSERT_TRUE(buffer.pushImu(makeImu(90)).ok());
  ASSERT_TRUE(buffer.pushImu(makeImu(150)).ok());
  MeasurementSynchronizerConfig sync_config;
  sync_config.maximum_imu_gap_ns = 100;
  MeasurementSynchronizer synchronizer(sync_config);
  const auto result = synchronizer.synchronizeNext(buffer);
  ASSERT_TRUE(result.ok());
  ASSERT_TRUE(result.value().has_value());
  ASSERT_EQ(result.value()->imu_samples.size(), 3U);
  EXPECT_EQ(result.value()->imu_samples[0].time.nanoseconds(), 90);
  EXPECT_EQ(result.value()->imu_samples[1].time.nanoseconds(), 150);
  EXPECT_EQ(result.value()->imu_samples[2].time.nanoseconds(), 210);
  EXPECT_EQ(buffer.stats().timestamp_regressions, 2U);
}

}  // namespace
}  // namespace uav::nav::lio
