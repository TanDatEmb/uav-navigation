#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "fast_lio_core/pipeline/fast_lio_pipeline.hpp"

namespace uav::nav::lio {
namespace {

constexpr std::int64_t kMillisecondNs = 1'000'000;

LidarScan makePlanarScan(std::int64_t time_ns, double z_m = 0.0) {
  LidarScan scan;
  scan.start_time = Timestamp(time_ns);
  scan.end_time = Timestamp(time_ns);
  scan.has_per_point_time = false;
  for (int x = -3; x <= 3; ++x) {
    for (int y = -3; y <= 3; ++y) {
      LidarPoint point;
      point.position_lidar_m = Eigen::Vector3f(
          0.25F * static_cast<float>(x), 0.25F * static_cast<float>(y), static_cast<float>(z_m));
      scan.points.push_back(point);
    }
  }
  return scan;
}

ImuSample stationaryImu(std::int64_t time_ns) {
  ImuSample sample;
  sample.time = Timestamp(time_ns);
  sample.angular_velocity_imu_rad_s.setZero();
  sample.linear_acceleration_imu_m_s2 = Eigen::Vector3d(0.0, 0.0, 9.80665);
  return sample;
}

MeasurementGroup makeGroup(LidarScan scan, std::int64_t propagation_start_ns,
                           std::vector<ImuSample> imu_samples) {
  MeasurementGroup group;
  group.scan = std::move(scan);
  group.imu_samples = std::move(imu_samples);
  group.propagation_start_time = Timestamp(propagation_start_ns);
  group.has_start_bracket = true;
  group.has_end_bracket = true;
  group.max_imu_gap_ns = 50 * kMillisecondNs;
  return group;
}

EstimatorConfig testConfig() {
  EstimatorConfig config;
  config.initialization.minimum_imu_samples = 3;
  config.initialization.maximum_imu_samples = 20;
  config.initialization.require_stationary = true;
  config.propagation.maximum_integration_step_ns = 100 * kMillisecondNs;
  config.deskew.mode = DeskewMode::kSimultaneousScan;
  config.preprocessing.point_filter.minimum_range_m = 0.01;
  config.preprocessing.point_filter.maximum_range_m = 10.0;
  config.preprocessing.enable_voxel_filter = false;
  config.residual_builder.correspondence_search.neighbor_count = 5;
  config.residual_builder.correspondence_search.maximum_neighbor_distance_m = 1.5;
  config.iterated_filter.convergence.maximum_iterations = 4;
  config.iterated_filter.convergence.minimum_accepted_residuals = 9;
  config.insertion_policy.minimum_point_count = 9;
  config.registration_map.voxel_size_m = 0.02;
  config.local_map.half_extent_m = {20.0, 20.0, 20.0};
  config.local_map.crop_trigger_distance_m = 1.0;
  config.lifecycle.degraded_after_registration_failures = 1;
  config.lifecycle.lost_after_registration_failures = 3;
  config.lifecycle.local_map_snapshot_period_scans = 1;
  return config;
}

TEST(FastLioPipelineTest, PublishesOnlyAfterCorrectionAndInsertsCorrectedOdomPoints) {
  FastLioPipeline pipeline(testConfig());
  const MeasurementGroup reference_group = makeGroup(
      makePlanarScan(0), 0,
      {stationaryImu(-20 * kMillisecondNs), stationaryImu(-10 * kMillisecondNs), stationaryImu(0)});

  const ProcessResult reference = pipeline.process(reference_group);

  EXPECT_EQ(reference.status_after, EstimatorStatus::kInitializingMap);
  EXPECT_FALSE(reference.has_corrected_odometry);
  EXPECT_FALSE(reference.lidar_correction_successful);
  EXPECT_TRUE(pipeline.registrationMapSnapshot().empty());
  EXPECT_EQ(reference.rejection_reason, "INITIAL_MAP_REFERENCE_CAPTURED");

  const MeasurementGroup tracking_group = makeGroup(
      makePlanarScan(100 * kMillisecondNs), 0,
      {stationaryImu(0), stationaryImu(50 * kMillisecondNs), stationaryImu(100 * kMillisecondNs)});
  const ProcessResult tracked = pipeline.process(tracking_group);

  EXPECT_EQ(tracked.status_after, EstimatorStatus::kTracking);
  EXPECT_TRUE(tracked.has_corrected_odometry);
  EXPECT_TRUE(tracked.lidar_correction_successful);
  EXPECT_TRUE(tracked.diagnostics.registration.converged);
  EXPECT_FALSE(tracked.registered_points_odom_m.empty());
  EXPECT_FALSE(pipeline.registrationMapSnapshot().empty());
  EXPECT_EQ(tracked.diagnostics.map.map_point_count, pipeline.registrationMapSnapshot().size());
  for (const Eigen::Vector3d& point : tracked.registered_points_odom_m) {
    EXPECT_NEAR(point.z(), 0.0, 1e-7);
  }
}

TEST(FastLioPipelineTest, FailedRegistrationTransitionsToDegradedWithoutMapInsertion) {
  FastLioPipeline pipeline(testConfig());
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs), stationaryImu(0)})));
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(100 * kMillisecondNs), 0,
                                 {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
                                  stationaryImu(100 * kMillisecondNs)})));
  const std::size_t map_size_before = pipeline.registrationMapSnapshot().size();

  const ProcessResult failed = pipeline.process(
      makeGroup(makePlanarScan(200 * kMillisecondNs, 1.0), 100 * kMillisecondNs,
                {stationaryImu(100 * kMillisecondNs), stationaryImu(150 * kMillisecondNs),
                 stationaryImu(200 * kMillisecondNs)}));

  EXPECT_EQ(failed.status_after, EstimatorStatus::kDegraded);
  EXPECT_FALSE(failed.has_corrected_odometry);
  EXPECT_FALSE(failed.lidar_correction_successful);
  EXPECT_EQ(pipeline.registrationMapSnapshot().size(), map_size_before);
  EXPECT_EQ(failed.diagnostics.map.inserted_point_count, 0U);
}

TEST(FastLioPipelineTest, ResetStopsPublishingAndClearsRegistrationMap) {
  FastLioPipeline pipeline(testConfig());
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs), stationaryImu(0)})));
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(100 * kMillisecondNs), 0,
                                 {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
                                  stationaryImu(100 * kMillisecondNs)})));
  ASSERT_FALSE(pipeline.registrationMapSnapshot().empty());

  pipeline.reset();

  EXPECT_EQ(pipeline.status(), EstimatorStatus::kWaitingForSensors);
  EXPECT_TRUE(pipeline.registrationMapSnapshot().empty());
  EXPECT_EQ(pipeline.diagnostics().reason, "RESET_COMPLETE");
}

TEST(FastLioPipelineTest, QueuedRuntimePathDoesNotAddInitializationImuTwice) {
  FastLioPipeline pipeline(testConfig());
  ASSERT_TRUE(pipeline.pushImu(stationaryImu(-20 * kMillisecondNs)).ok());
  ASSERT_TRUE(pipeline.pushImu(stationaryImu(-10 * kMillisecondNs)).ok());
  ASSERT_TRUE(pipeline.pushImu(stationaryImu(0)).ok());
  ASSERT_TRUE(pipeline.pushLidar(makePlanarScan(0)).ok());

  const auto result = pipeline.processNext();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status_after, EstimatorStatus::kInitializingMap);
  EXPECT_EQ(result->rejection_reason, "INITIAL_MAP_REFERENCE_CAPTURED");
  EXPECT_NE(result->rejection_reason, "timestamp must be strictly increasing");
}

}  // namespace
}  // namespace uav::nav::lio
