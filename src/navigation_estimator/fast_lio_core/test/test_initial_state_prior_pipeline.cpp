#include <gtest/gtest.h>

#include "fast_lio_core/pipeline/fast_lio_pipeline.hpp"

namespace uav::nav::lio {
namespace {

ImuSample imu(std::int64_t time_ns) {
  ImuSample sample;
  sample.time = Timestamp(time_ns);
  sample.linear_acceleration_imu_m_s2 = Eigen::Vector3d(0.0, 0.0, 9.80665);
  sample.angular_velocity_imu_rad_s.setZero();
  return sample;
}

LidarScan scan(std::int64_t time_ns) {
  LidarScan value;
  value.start_time = Timestamp(time_ns);
  value.end_time = Timestamp(time_ns);
  value.points.push_back({Eigen::Vector3f(1.0F, 0.0F, 0.0F), 0, 0, 0, 0});
  return value;
}

InitialStatePrior topicPrior(std::int64_t time_ns) {
  InitialStatePrior prior;
  prior.sample_time = Timestamp(time_ns);
  prior.reference_frame = odomFrame();
  prior.body_frame = baseFrame();
  prior.source = InitialStatePriorSource::kTopic;
  prior.context = InitialStatePriorContext::kGroundStartup;
  prior.mask = {true, false, PriorAttitudeMode::kNone};
  prior.position_odom_base_m = Eigen::Vector3d(3.0, -2.0, 1.0);
  prior.provenance = "test";
  return prior;
}

EstimatorConfig topicConfig() {
  EstimatorConfig config;
  config.initialization.minimum_imu_samples = 3;
  config.initialization.maximum_imu_samples = 20;
  config.initialization.require_stationary = true;
  config.deskew.mode = DeskewMode::kSimultaneousScan;
  config.initial_prior.source = InitialStatePriorSource::kTopic;
  config.initial_prior.context = InitialStatePriorContext::kGroundStartup;
  config.initial_prior.mask = {true, false, PriorAttitudeMode::kNone};
  config.initial_prior.topic_wait_timeout_ns = 1'000'000'000;
  config.initial_prior.maximum_topic_prior_age_ns = 500'000'000;
  config.initial_prior.ground_fallback = InitialPriorFallback::kReject;
  config.insertion_policy.minimum_point_count = 1;
  return config;
}

TEST(InitialStatePriorPipelineTest, PendingTopicPriorDoesNotConsumeQueuedLidar) {
  FastLioPipeline pipeline(topicConfig());
  ASSERT_TRUE(pipeline.pushImu(imu(100'000'000)).ok());
  ASSERT_TRUE(pipeline.pushImu(imu(200'000'000)).ok());
  ASSERT_TRUE(pipeline.pushImu(imu(300'000'000)).ok());
  ASSERT_TRUE(pipeline.pushLidar(scan(400'000'000)).ok());
  ASSERT_EQ(pipeline.status(), EstimatorStatus::kInitializingImu);
  const std::size_t queued_before = pipeline.pendingLidarCount();
  EXPECT_FALSE(pipeline.processNext().has_value());
  EXPECT_EQ(pipeline.pendingLidarCount(), queued_before);
  EXPECT_EQ(pipeline.diagnostics().initial_prior.status, InitialPriorStatus::kWaiting);
}

TEST(InitialStatePriorPipelineTest, ValidTopicPriorAppliesOnceBeforeMapBootstrap) {
  FastLioPipeline pipeline(topicConfig());
  MeasurementGroup group;
  group.scan.start_time = Timestamp(1'000'000'000);
  group.scan.end_time = Timestamp(1'000'000'000);
  group.scan.points.push_back({Eigen::Vector3f(1.0F, 0.0F, 0.0F), 0, 0, 0, 0});
  group.imu_samples = {imu(800'000'000), imu(900'000'000), imu(1'000'000'000)};
  group.propagation_start_time = Timestamp(800'000'000);
  group.has_start_bracket = true;
  group.has_end_bracket = true;
  group.max_imu_gap_ns = 100'000'000;

  const ProcessResult pending = pipeline.process(group);
  EXPECT_EQ(pending.rejection_reason, "INITIAL_STATE_PRIOR_PENDING");
  EXPECT_FALSE(pipeline.diagnostics().initial_prior.applied);
  ASSERT_TRUE(pipeline.submitInitialStatePrior(topicPrior(1'000'000'000)).ok());
  const ProcessResult applied = pipeline.process(group);
  ASSERT_EQ(applied.status_after, EstimatorStatus::kInitializingMap)
      << applied.rejection_reason << " / " << applied.diagnostics.reason;
  EXPECT_EQ(applied.status_after, EstimatorStatus::kInitializingMap);
  EXPECT_EQ(applied.diagnostics.initial_prior.status, InitialPriorStatus::kClosed)
      << applied.diagnostics.initial_prior.reason;
  EXPECT_TRUE(applied.diagnostics.initial_prior.applied);
  EXPECT_TRUE(applied.diagnostics.initial_prior.bootstrap_map_after_prior)
      << applied.rejection_reason << " / " << applied.diagnostics.reason;
  EXPECT_TRUE(pipeline.state().position_odom_imu_m().isApprox(
      topicPrior(1'000'000'000).position_odom_base_m));

  const Status late = pipeline.submitInitialStatePrior(topicPrior(1'100'000'000));
  EXPECT_EQ(late.code(), StatusCode::kNotReady);
  EXPECT_EQ(pipeline.diagnostics().initial_prior.late_rejected_count, 1U);
}

}  // namespace
}  // namespace uav::nav::lio
