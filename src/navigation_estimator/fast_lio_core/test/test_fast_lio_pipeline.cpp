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

LidarScan makeCornerScan(std::int64_t time_ns) {
  auto scan = makePlanarScan(time_ns);
  for (int a = -3; a <= 3; ++a) {
    for (int b = -3; b <= 3; ++b) {
      LidarPoint x_plane;
      x_plane.position_lidar_m =
          Eigen::Vector3f(1.0F, 0.25F * static_cast<float>(a),
                          0.25F * static_cast<float>(b));
      scan.points.push_back(x_plane);
      LidarPoint y_plane;
      y_plane.position_lidar_m =
          Eigen::Vector3f(0.25F * static_cast<float>(a), 1.0F,
                          0.25F * static_cast<float>(b));
      scan.points.push_back(y_plane);
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
  config.ikfom.maximum_integration_step_ns = 100 * kMillisecondNs;
  config.deskew.mode = DeskewMode::kSimultaneousScan;
  config.preprocessing.point_filter.minimum_range_m = 0.01;
  config.preprocessing.point_filter.maximum_range_m = 10.0;
  config.preprocessing.enable_voxel_filter = false;
  config.residual_builder.correspondence_search.neighbor_count = 5;
  config.residual_builder.correspondence_search.maximum_neighbor_distance_m = 1.5;
  config.ikfom.maximum_iterations = 4;
  config.ikfom.minimum_accepted_residuals = 9;
  config.insertion_policy.minimum_point_count = 9;
  config.registration_map.voxel_size_m = 0.02;
  config.local_map.half_extent_m = {20.0, 20.0, 20.0};
  config.local_map.crop_trigger_distance_m = 1.0;
  config.lifecycle.degraded_after_registration_failures = 1;
  config.lifecycle.lost_after_registration_failures = 3;
  return config;
}

void expectTransientStagesCleared(const ProcessResult& result) {
  EXPECT_FALSE(result.diagnostics.deskew.deskew_applied);
  EXPECT_FALSE(result.diagnostics.registration.correction_attempted);
  EXPECT_EQ(result.diagnostics.registration.residual_rms_m, 0.0);
  EXPECT_EQ(result.diagnostics.registration.iteration_count, 0U);
  EXPECT_EQ(result.diagnostics.timing.imu_prediction_us, 0);
  EXPECT_EQ(result.diagnostics.timing.deskew_us, 0);
  EXPECT_EQ(result.diagnostics.timing.preprocessing_us, 0);
  EXPECT_EQ(result.diagnostics.timing.residual_build_us, 0);
  EXPECT_EQ(result.diagnostics.timing.ikfom_update_us, 0);
  EXPECT_EQ(result.diagnostics.timing.map_insert_crop_us, 0);
  EXPECT_EQ(result.diagnostics.timing.map_maintenance_us, 0);
  EXPECT_EQ(result.diagnostics.timing.snapshot_us, 0);
  EXPECT_EQ(result.diagnostics.timing.total_processing_us, 0);
  EXPECT_FALSE(result.diagnostics.map.map_update_performed);
  EXPECT_EQ(result.diagnostics.map.map_candidate_count, 0U);
  EXPECT_EQ(result.diagnostics.map.map_inserted_count, 0U);
  EXPECT_EQ(result.diagnostics.map.crop_removed_count, 0U);
  EXPECT_EQ(result.diagnostics.map.snapshot_point_count, 0U);
}

TEST(FastLioPipelineTest, PublishesOnlyAfterCorrectionAndInsertsCorrectedOdomPoints) {
  FastLioPipeline pipeline(testConfig());
  const MeasurementGroup reference_group = makeGroup(
      makePlanarScan(0), 0,
      {stationaryImu(-20 * kMillisecondNs), stationaryImu(-10 * kMillisecondNs), stationaryImu(0)});

  const ProcessResult reference = pipeline.process(reference_group);

  EXPECT_EQ(reference.status_after, EstimatorStatus::kInitializingMap);
  EXPECT_EQ(reference.estimate_validity, EstimateValidity::kPredictedOnly);
  EXPECT_EQ(reference.lidar_update_status, LidarUpdateStatus::kNotAttempted);
  EXPECT_TRUE(reference.hasPredictedOutput());
  EXPECT_FALSE(reference.hasCorrectedOutput());
  EXPECT_FALSE(reference.corrected_estimate.has_value());
  EXPECT_TRUE(reference.registered_points_odom_m.empty());
  EXPECT_TRUE(reference.diagnostics.output.predicted_estimate_valid);
  EXPECT_FALSE(reference.diagnostics.output.corrected_estimate_valid);
  EXPECT_TRUE(pipeline.registrationMapSnapshot().empty());
  EXPECT_EQ(reference.rejection_reason, "INITIAL_MAP_REFERENCE_CAPTURED");

  const MeasurementGroup tracking_group = makeGroup(
      makePlanarScan(100 * kMillisecondNs), 0,
      {stationaryImu(0), stationaryImu(50 * kMillisecondNs), stationaryImu(100 * kMillisecondNs)});
  const ProcessResult tracked = pipeline.process(tracking_group);

  EXPECT_EQ(tracked.status_after, EstimatorStatus::kTracking);
  EXPECT_EQ(tracked.estimate_validity, EstimateValidity::kCorrected);
  EXPECT_EQ(tracked.lidar_update_status, LidarUpdateStatus::kSucceeded);
  EXPECT_TRUE(tracked.hasPredictedOutput());
  EXPECT_TRUE(tracked.hasCorrectedOutput());
  ASSERT_TRUE(tracked.corrected_estimate.has_value());
  EXPECT_EQ(tracked.corrected_estimate->time, tracking_group.scan.end_time);
  EXPECT_TRUE(tracked.diagnostics.output.corrected_estimate_valid);
  EXPECT_TRUE(tracked.diagnostics.output.registered_scan_valid);
  EXPECT_EQ(tracked.diagnostics.output.last_lidar_correction_time_ns,
            tracking_group.scan.end_time.nanoseconds());
  EXPECT_TRUE(tracked.diagnostics.registration.converged);
  EXPECT_FALSE(tracked.registered_points_odom_m.empty());
  EXPECT_FALSE(pipeline.registrationMapSnapshot().empty());
  EXPECT_EQ(tracked.diagnostics.map.map_point_count, pipeline.registrationMapSnapshot().size());
  for (const Eigen::Vector3d& point : tracked.registered_points_odom_m) {
    EXPECT_NEAR(point.z(), 0.0, 2e-7);
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
  EXPECT_EQ(failed.estimate_validity, EstimateValidity::kPredictedOnly);
  EXPECT_EQ(failed.lidar_update_status, LidarUpdateStatus::kRejected);
  EXPECT_TRUE(failed.hasPredictedOutput());
  EXPECT_FALSE(failed.hasCorrectedOutput());
  EXPECT_FALSE(failed.corrected_estimate.has_value());
  EXPECT_TRUE(failed.registered_points_odom_m.empty());
  EXPECT_TRUE(failed.diagnostics.output.predicted_estimate_valid);
  EXPECT_FALSE(failed.diagnostics.output.corrected_estimate_valid);
  EXPECT_FALSE(failed.diagnostics.output.registered_scan_valid);
  EXPECT_EQ(pipeline.registrationMapSnapshot().size(), map_size_before);
  EXPECT_EQ(failed.diagnostics.map.inserted_point_count, 0U);
}

TEST(FastLioPipelineTest,
     ImuDiscontinuityRebasesBothEpochsAndDoesNotRepeatGap) {
  auto config = testConfig();
  config.synchronization.maximum_imu_gap_ns = 20 * kMillisecondNs;
  config.tracking.maximum_recoverable_imu_gap_ns = 100 * kMillisecondNs;
  FastLioPipeline pipeline(config);
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs),
                                  stationaryImu(0)})));
  static_cast<void>(pipeline.process(
      makeGroup(makePlanarScan(100 * kMillisecondNs), 0,
                {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
                 stationaryImu(100 * kMillisecondNs)})));
  ASSERT_EQ(pipeline.status(), EstimatorStatus::kTracking);
  const auto covariance_before = pipeline.covariance();

  for (const auto time_ns :
       {100 * kMillisecondNs, 110 * kMillisecondNs,
        190 * kMillisecondNs, 200 * kMillisecondNs,
        210 * kMillisecondNs, 220 * kMillisecondNs}) {
    ASSERT_TRUE(pipeline.pushImu(stationaryImu(time_ns)).ok());
  }
  LidarScan crossing = makePlanarScan(105 * kMillisecondNs);
  crossing.end_time = Timestamp(200 * kMillisecondNs);
  ASSERT_TRUE(pipeline.pushLidar(std::move(crossing)).ok());

  const auto discontinuity = pipeline.processNext();
  ASSERT_TRUE(discontinuity.has_value());
  EXPECT_EQ(discontinuity->status_after, EstimatorStatus::kDegraded);
  EXPECT_EQ(discontinuity->rejection_reason,
            "IMU_PROPAGATION_DISCONTINUITY");
  EXPECT_EQ(discontinuity->diagnostics.propagation_discontinuity_count, 1U);
  EXPECT_EQ(discontinuity->diagnostics.last_propagation_gap_ns,
            80 * kMillisecondNs);
  ASSERT_TRUE(pipeline.stateTime().has_value());
  ASSERT_TRUE(pipeline.synchronizationEpoch().has_value());
  EXPECT_EQ(*pipeline.stateTime(), *pipeline.synchronizationEpoch());
  EXPECT_EQ(pipeline.stateTime()->nanoseconds(), 190 * kMillisecondNs);
  EXPECT_GT(pipeline.covariance().trace(), covariance_before.trace());

  ASSERT_TRUE(
      pipeline.pushLidar(makePlanarScan(220 * kMillisecondNs)).ok());
  const auto following = pipeline.processNext();
  ASSERT_TRUE(following.has_value());
  EXPECT_TRUE(following->diagnostics.registration.correction_attempted);
  EXPECT_EQ(following->diagnostics.propagation_discontinuity_count, 1U);
}

TEST(FastLioPipelineTest,
     RecoveryRequiresThreeCorrectionsAndQuarantinesMapUntilConfirmed) {
  auto config = testConfig();
  config.tracking.recovery_confirmation_updates = 3;
  FastLioPipeline pipeline(config);
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs),
                                  stationaryImu(0)})));
  static_cast<void>(pipeline.process(
      makeGroup(makePlanarScan(100 * kMillisecondNs), 0,
                {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
                 stationaryImu(100 * kMillisecondNs)})));
  const std::size_t confirmed_map_size =
      pipeline.registrationMapSnapshot().size();

  const ProcessResult failed = pipeline.process(
      makeGroup(makePlanarScan(200 * kMillisecondNs, 1.0),
                100 * kMillisecondNs,
                {stationaryImu(100 * kMillisecondNs),
                 stationaryImu(150 * kMillisecondNs),
                 stationaryImu(200 * kMillisecondNs)}));
  EXPECT_EQ(failed.status_after, EstimatorStatus::kDegraded);
  EXPECT_EQ(failed.diagnostics.consecutive_uncorrected_lidar_updates, 1U);
  EXPECT_TRUE(failed.diagnostics.map_insertion_frozen);
  EXPECT_FALSE(failed.diagnostics.navigation_valid);
  EXPECT_EQ(pipeline.registrationMapSnapshot().size(), confirmed_map_size);

  for (std::size_t recovery_index = 1; recovery_index <= 3;
       ++recovery_index) {
    const std::int64_t start_ns =
        (1 + static_cast<std::int64_t>(recovery_index)) *
        100 * kMillisecondNs;
    const std::int64_t end_ns = start_ns + 100 * kMillisecondNs;
    const ProcessResult recovered = pipeline.process(
        makeGroup(makePlanarScan(end_ns), start_ns,
                  {stationaryImu(start_ns),
                   stationaryImu(start_ns + 50 * kMillisecondNs),
                   stationaryImu(end_ns)}));
    ASSERT_EQ(recovered.lidar_update_status, LidarUpdateStatus::kSucceeded);
    if (recovery_index < 3) {
      EXPECT_EQ(recovered.status_after, EstimatorStatus::kDegraded);
      EXPECT_EQ(recovered.diagnostics.consecutive_recovery_successes,
                recovery_index);
      EXPECT_TRUE(recovered.diagnostics.map_insertion_frozen);
      EXPECT_EQ(pipeline.registrationMapSnapshot().size(),
                confirmed_map_size);
    } else {
      EXPECT_EQ(recovered.status_after, EstimatorStatus::kTracking);
      EXPECT_TRUE(recovered.diagnostics.navigation_valid);
      EXPECT_FALSE(recovered.diagnostics.map_insertion_frozen);
      EXPECT_TRUE(recovered.diagnostics.map.map_update_performed);
    }
  }
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

TEST(FastLioPipelineTest, OverlapRejectionDoesNotRepeatCorrectedScanDiagnostics) {
  FastLioPipeline pipeline(testConfig());
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs),
                                  stationaryImu(0)})));
  const auto direct_corrected = pipeline.process(
      makeGroup(makePlanarScan(90 * kMillisecondNs), 0,
                {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
                 stationaryImu(90 * kMillisecondNs)}));
  ASSERT_TRUE(direct_corrected.hasCorrectedOutput());
  for (const std::int64_t time_ns :
       {90 * kMillisecondNs, 100 * kMillisecondNs,
        110 * kMillisecondNs}) {
    ASSERT_TRUE(pipeline.pushImu(stationaryImu(time_ns)).ok());
  }
  auto corrected_scan = makePlanarScan(90 * kMillisecondNs);
  corrected_scan.end_time = Timestamp(100 * kMillisecondNs);
  ASSERT_TRUE(pipeline.pushLidar(std::move(corrected_scan)).ok());
  const auto corrected = pipeline.processNext();
  ASSERT_TRUE(corrected.has_value());
  ASSERT_TRUE(corrected->hasCorrectedOutput());
  ASSERT_TRUE(corrected->diagnostics.registration.correction_attempted);

  auto overlap = makePlanarScan(99 * kMillisecondNs);
  overlap.end_time = Timestamp(110 * kMillisecondNs);
  ASSERT_TRUE(pipeline.pushLidar(std::move(overlap)).ok());
  const auto rejected = pipeline.processNext();
  ASSERT_TRUE(rejected.has_value());
  EXPECT_NE(rejected->rejection_reason.find("OVERLAPPING_LIDAR_INTERVAL"),
            std::string::npos);
  EXPECT_FALSE(rejected->diagnostics.synchronization.synchronized);
  expectTransientStagesCleared(*rejected);
}

TEST(FastLioPipelineTest, MissingBracketDoesNotRepeatCorrectedScanDiagnostics) {
  FastLioPipeline pipeline(testConfig());
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs),
                                  stationaryImu(0)})));
  const auto corrected = pipeline.process(
      makeGroup(makePlanarScan(100 * kMillisecondNs), 0,
                {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
                 stationaryImu(100 * kMillisecondNs)}));
  ASSERT_TRUE(corrected.hasCorrectedOutput());

  auto missing = makeGroup(
      makePlanarScan(200 * kMillisecondNs), 100 * kMillisecondNs,
      {stationaryImu(150 * kMillisecondNs),
       stationaryImu(200 * kMillisecondNs)});
  missing.has_start_bracket = false;
  const auto rejected = pipeline.process(missing);
  EXPECT_EQ(rejected.rejection_reason, "MEASUREMENT_GROUP_NOT_FULLY_BRACKETED");
  expectTransientStagesCleared(rejected);
}

TEST(FastLioPipelineTest, CorrectedResultsOwnIndependentTimingSnapshots) {
  FastLioPipeline pipeline(testConfig());
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs),
                                  stationaryImu(0)})));
  const ProcessResult first = pipeline.process(
      makeGroup(makePlanarScan(100 * kMillisecondNs), 0,
                {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
                 stationaryImu(100 * kMillisecondNs)}));
  ASSERT_TRUE(first.hasCorrectedOutput());
  const auto first_timing = first.diagnostics.timing;
  const ProcessResult second = pipeline.process(
      makeGroup(makePlanarScan(200 * kMillisecondNs), 100 * kMillisecondNs,
                {stationaryImu(100 * kMillisecondNs),
                 stationaryImu(150 * kMillisecondNs),
                 stationaryImu(200 * kMillisecondNs)}));
  ASSERT_TRUE(second.hasCorrectedOutput());
  EXPECT_EQ(first.diagnostics.timing.total_processing_us,
            first_timing.total_processing_us);
  EXPECT_TRUE(first.diagnostics.registration.correction_succeeded);
  EXPECT_TRUE(second.diagnostics.registration.correction_succeeded);
  EXPECT_TRUE(first.diagnostics.map.map_update_performed);
  EXPECT_TRUE(second.diagnostics.map.map_update_performed);
}

TEST(FastLioPipelineTest,
     PeriodicLocalMapSnapshotUsesSuccessfulCorrectionCadence) {
  auto config = testConfig();
  config.lifecycle.local_map_snapshot_period_scans = 10;
  config.lifecycle.lost_after_registration_failures = 20;
  FastLioPipeline pipeline(config);
  static_cast<void>(pipeline.process(makeGroup(
      makeCornerScan(0), 0,
      {stationaryImu(-20 * kMillisecondNs),
       stationaryImu(-10 * kMillisecondNs), stationaryImu(0)})));

  for (int correction = 1; correction <= 10; ++correction) {
    SCOPED_TRACE(correction);
    const auto time_ns = correction * 100 * kMillisecondNs;
    auto propagation_start_ns =
        (correction - 1) * 100 * kMillisecondNs;
    if (correction == 5) {
      auto invalid_group = makeGroup(
          makeCornerScan(time_ns), propagation_start_ns,
          {stationaryImu(propagation_start_ns),
           stationaryImu(time_ns)});
      invalid_group.has_start_bracket = false;
      const auto failed = pipeline.process(invalid_group);
      ASSERT_FALSE(failed.hasCorrectedOutput());
      EXPECT_TRUE(failed.local_map_points_odom_m.empty());
      EXPECT_EQ(failed.diagnostics.timing.snapshot_us, 0);
    }
    std::vector<ImuSample> correction_imu{
        stationaryImu(propagation_start_ns)};
    if (time_ns - 50 * kMillisecondNs > propagation_start_ns) {
      correction_imu.push_back(
          stationaryImu(time_ns - 50 * kMillisecondNs));
    }
    correction_imu.push_back(stationaryImu(time_ns));
    const auto corrected = pipeline.process(
        makeGroup(makeCornerScan(time_ns), propagation_start_ns,
                  std::move(correction_imu)));
    ASSERT_TRUE(corrected.hasCorrectedOutput());
    if (correction == 1 || correction == 10) {
      EXPECT_FALSE(corrected.local_map_points_odom_m.empty());
      EXPECT_GT(corrected.diagnostics.map.snapshot_point_count, 0U);
    } else {
      EXPECT_TRUE(corrected.local_map_points_odom_m.empty());
      EXPECT_EQ(corrected.diagnostics.timing.snapshot_us, 0);
    }
  }
}

TEST(FastLioPipelineTest,
     DisabledPeriodicSnapshotSkipsOutputButFinalMapRemainsAvailable) {
  auto config = testConfig();
  config.lifecycle.enable_periodic_local_map_snapshot = false;
  FastLioPipeline pipeline(config);
  static_cast<void>(pipeline.process(makeGroup(
      makePlanarScan(0), 0,
      {stationaryImu(-20 * kMillisecondNs),
       stationaryImu(-10 * kMillisecondNs), stationaryImu(0)})));
  const auto corrected = pipeline.process(makeGroup(
      makePlanarScan(100 * kMillisecondNs), 0,
      {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
       stationaryImu(100 * kMillisecondNs)}));

  ASSERT_TRUE(corrected.hasCorrectedOutput());
  EXPECT_TRUE(corrected.local_map_points_odom_m.empty());
  EXPECT_EQ(corrected.diagnostics.map.snapshot_point_count, 0U);
  EXPECT_EQ(corrected.diagnostics.timing.snapshot_us, 0);
  EXPECT_FALSE(pipeline.registrationMapSnapshot().empty());
  EXPECT_EQ(corrected.diagnostics.map.map_point_count,
            pipeline.registrationMapSnapshot().size());
}

}  // namespace
}  // namespace uav::nav::lio
