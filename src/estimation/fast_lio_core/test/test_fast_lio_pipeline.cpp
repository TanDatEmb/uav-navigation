#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cstdint>
#include <vector>

#include "fast_lio_core/geometry/frame_ids.hpp"
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
  config.residual_builder.correspondence_search.maximum_neighbor_distance_m = 1.5;
  config.residual_builder.minimum_translation_observability_ratio = 0.0;
  config.ikfom.maximum_iterations = 4;
  config.ikfom.minimum_accepted_residuals = 9;
  config.insertion_policy.minimum_point_count = 9;
  config.registration_map.voxel_size_m = 0.02;
  config.local_map.half_extent_m = {20.0, 20.0, 20.0};
  config.local_map.crop_trigger_distance_m = 1.0;
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
  EXPECT_EQ(result.diagnostics.timing.total_processing_us, 0);
  EXPECT_FALSE(result.diagnostics.map.map_update_performed);
  EXPECT_EQ(result.diagnostics.map.map_candidate_count, 0U);
  EXPECT_EQ(result.diagnostics.map.map_inserted_count, 0U);
  EXPECT_EQ(result.diagnostics.map.crop_removed_count, 0U);
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
  EXPECT_EQ(pipeline.registrationMapSize(), 0U);
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
  ASSERT_TRUE(tracked.corrected_kinematic_estimate.has_value());
  EXPECT_EQ(tracked.corrected_kinematic_estimate->estimate.time,
            tracking_group.scan.end_time);
  EXPECT_TRUE(tracked.corrected_kinematic_estimate->angular_velocity_imu_rad_s
                  .allFinite());
  EXPECT_EQ(tracked.diagnostics.corrected_angular_velocity.exact_sample_count,
            1U);
  EXPECT_TRUE(tracked.diagnostics.output.corrected_estimate_valid);
  EXPECT_TRUE(tracked.diagnostics.output.registered_scan_valid);
  EXPECT_EQ(tracked.diagnostics.output.last_lidar_correction_time_ns,
            tracking_group.scan.end_time.nanoseconds());
  EXPECT_TRUE(tracked.diagnostics.registration.converged);
  EXPECT_FALSE(tracked.registered_points_odom_m.empty());
  EXPECT_GT(pipeline.registrationMapSize(), 0U);
  EXPECT_EQ(tracked.diagnostics.map.map_point_count, pipeline.registrationMapSize());
  for (const Eigen::Vector3d& point : tracked.registered_points_odom_m) {
    EXPECT_NEAR(point.z(), 0.0, 2e-7);
  }
}

// P1 mandatory correctness test A (docs/architecture/navigation_layers.md):
// the mapping observation epoch, the corrected sensor pose epoch, and the
// scan reference time must be exactly the same value, and the mapping
// observation must only be available once a correction has succeeded.
TEST(FastLioPipelineTest, MappingObservationEpochMatchesCorrectedSensorPoseEpoch) {
  auto config = testConfig();
  config.preprocessing.retain_mapping_candidate = true;
  FastLioPipeline pipeline(config);
  const MeasurementGroup reference_group = makeGroup(
      makePlanarScan(0), 0,
      {stationaryImu(-20 * kMillisecondNs), stationaryImu(-10 * kMillisecondNs), stationaryImu(0)});
  const ProcessResult reference = pipeline.process(reference_group);
  // Before any correction succeeds, a mapping observation must not be
  // reported as available, even though preprocessing already ran.
  EXPECT_FALSE(reference.hasMappingObservationOutput());
  EXPECT_FALSE(reference.sensor_pose_odom_lidar.has_value());

  const MeasurementGroup tracking_group = makeGroup(
      makePlanarScan(100 * kMillisecondNs), 0,
      {stationaryImu(0), stationaryImu(50 * kMillisecondNs), stationaryImu(100 * kMillisecondNs)});
  const ProcessResult tracked = pipeline.process(tracking_group);

  ASSERT_TRUE(tracked.hasCorrectedOutput());
  ASSERT_TRUE(tracked.hasMappingObservationOutput());
  ASSERT_TRUE(tracked.scan_time.has_value());
  ASSERT_TRUE(tracked.corrected_estimate.has_value());
  ASSERT_TRUE(tracked.sensor_pose_odom_lidar.has_value());

  // The epoch contract: scan reference time == corrected estimate time ==
  // the epoch the corrected sensor pose was derived at (scan.end_time, which
  // is FAST-LIO's configured deskew reference).
  EXPECT_EQ(tracked.scan_time->nanoseconds(), tracking_group.scan.end_time.nanoseconds());
  EXPECT_EQ(tracked.corrected_estimate->time.nanoseconds(),
            tracking_group.scan.end_time.nanoseconds());

  // The sensor pose must come from the same corrected state, not from a
  // separately propagated/predicted state.
  EXPECT_EQ(tracked.sensor_pose_odom_lidar->targetFrame(), lioOdomFrame());
  EXPECT_EQ(tracked.sensor_pose_odom_lidar->sourceFrame(), lidarFrame());
  EXPECT_TRUE(tracked.sensor_pose_odom_lidar->translation().allFinite());
  EXPECT_TRUE(tracked.sensor_pose_odom_lidar->rotation().coeffs().allFinite());

  // The retained mapping candidate must be non-empty and expressed at the
  // same reference epoch (it is never separately timestamped; its epoch is
  // scan_time by construction).
  EXPECT_FALSE(tracked.mapping_candidate_points_lidar_m.empty());
}

TEST(FastLioPipelineTest, MappingCandidateNotRetainedUnlessConfigured) {
  FastLioPipeline pipeline(testConfig());  // retain_mapping_candidate defaults false
  const MeasurementGroup reference_group = makeGroup(
      makePlanarScan(0), 0,
      {stationaryImu(-20 * kMillisecondNs), stationaryImu(-10 * kMillisecondNs), stationaryImu(0)});
  static_cast<void>(pipeline.process(reference_group));
  const MeasurementGroup tracking_group = makeGroup(
      makePlanarScan(100 * kMillisecondNs), 0,
      {stationaryImu(0), stationaryImu(50 * kMillisecondNs), stationaryImu(100 * kMillisecondNs)});
  const ProcessResult tracked = pipeline.process(tracking_group);
  ASSERT_TRUE(tracked.hasCorrectedOutput());
  // Mapping candidate retention must be opt-in and must not change estimator
  // behavior (the correction path above must succeed identically either way).
  EXPECT_TRUE(tracked.mapping_candidate_points_lidar_m.empty());
}

TEST(FastLioPipelineTest, AcceptedTerminalIterateAlsoUpdatesRegistrationMap) {
  auto config = testConfig();
  config.ikfom.maximum_iterations = 1U;
  config.ikfom.convergence_limit = 1e-12;
  FastLioPipeline pipeline(config);
  static_cast<void>(pipeline.process(makeGroup(
      makePlanarScan(0), 0,
      {stationaryImu(-20 * kMillisecondNs), stationaryImu(-10 * kMillisecondNs),
       stationaryImu(0)})));

  const ProcessResult accepted = pipeline.process(makeGroup(
      makePlanarScan(100 * kMillisecondNs, 0.05), 0,
      {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
       stationaryImu(100 * kMillisecondNs)}));

  ASSERT_EQ(accepted.lidar_update_status, LidarUpdateStatus::kSucceeded)
      << accepted.rejection_reason;
  EXPECT_FALSE(accepted.diagnostics.registration.converged);
  EXPECT_TRUE(accepted.diagnostics.map.map_update_performed);
  EXPECT_GT(accepted.diagnostics.map.map_inserted_count, 0U);
  EXPECT_GT(pipeline.registrationMapSize(), 0U);
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
  const std::size_t map_size_before = pipeline.registrationMapSize();

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
  EXPECT_EQ(pipeline.registrationMapSize(), map_size_before);
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
            "IMU_DISCONTINUITY_RECOVERY_REBASE");
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
      pipeline.registrationMapSize();

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
  EXPECT_EQ(pipeline.registrationMapSize(), confirmed_map_size);

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
      EXPECT_EQ(pipeline.registrationMapSize(),
                confirmed_map_size);
    } else {
      EXPECT_EQ(recovered.status_after, EstimatorStatus::kTracking);
      EXPECT_TRUE(recovered.diagnostics.navigation_valid);
      EXPECT_FALSE(recovered.diagnostics.map_insertion_frozen);
      EXPECT_TRUE(recovered.diagnostics.map.map_update_performed);
    }
  }
}

TEST(FastLioPipelineTest,
     AbsoluteMapGuardFailureRequiresResetBeforeTrackingRecovery) {
  auto config = testConfig();
  config.tracking.recovery_confirmation_updates = 3;
  config.local_map.absolute_map_point_guard = 1U;
  FastLioPipeline pipeline(config);

  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs),
                                  stationaryImu(0)})));
  const ProcessResult guard_failure = pipeline.process(
      makeGroup(makePlanarScan(100 * kMillisecondNs), 0,
                {stationaryImu(0), stationaryImu(50 * kMillisecondNs),
                 stationaryImu(100 * kMillisecondNs)}));

  ASSERT_TRUE(guard_failure.diagnostics.map.absolute_guard_recovery_failed);
  EXPECT_TRUE(guard_failure.diagnostics.map.map_insertion_frozen);
  EXPECT_EQ(guard_failure.status_after, EstimatorStatus::kDegraded);
  EXPECT_FALSE(guard_failure.diagnostics.navigation_valid);
  EXPECT_EQ(guard_failure.diagnostics.reason, "LOCAL_MAP_RECOVERY_REQUIRED");
  const std::size_t frozen_map_size = pipeline.registrationMapSize();

  for (std::size_t recovery_index = 1; recovery_index <= 3;
       ++recovery_index) {
    const std::int64_t start_ns =
        static_cast<std::int64_t>(recovery_index) * 100 * kMillisecondNs;
    const std::int64_t end_ns = start_ns + 100 * kMillisecondNs;
    const ProcessResult recovered = pipeline.process(
        makeGroup(makePlanarScan(end_ns), start_ns,
                  {stationaryImu(start_ns),
                   stationaryImu(start_ns + 50 * kMillisecondNs),
                   stationaryImu(end_ns)}));
    ASSERT_EQ(recovered.lidar_update_status, LidarUpdateStatus::kSucceeded);
    EXPECT_EQ(recovered.status_after, EstimatorStatus::kDegraded);
    EXPECT_FALSE(recovered.diagnostics.navigation_valid);
    EXPECT_TRUE(recovered.diagnostics.map.map_insertion_frozen);
    EXPECT_EQ(recovered.diagnostics.reason, "LOCAL_MAP_RECOVERY_REQUIRED");
    EXPECT_EQ(pipeline.registrationMapSize(), frozen_map_size);
  }

  pipeline.reset();
  EXPECT_EQ(pipeline.status(), EstimatorStatus::kWaitingForSensors);
}

TEST(FastLioPipelineTest, ShortGapAtFailureThresholdStaysLost) {
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
  for (std::int64_t end_ns : {200 * kMillisecondNs,
                              300 * kMillisecondNs}) {
    const std::int64_t start_ns = end_ns - 100 * kMillisecondNs;
    static_cast<void>(pipeline.process(
        makeGroup(makePlanarScan(end_ns, 1.0), start_ns,
                  {stationaryImu(start_ns),
                   stationaryImu(start_ns + 50 * kMillisecondNs),
                   stationaryImu(end_ns)})));
  }
  ASSERT_EQ(pipeline.status(), EstimatorStatus::kDegraded);
  const std::size_t map_size = pipeline.registrationMapSize();
  for (const auto time_ns :
       {300 * kMillisecondNs, 310 * kMillisecondNs,
        390 * kMillisecondNs, 400 * kMillisecondNs}) {
    ASSERT_TRUE(pipeline.pushImu(stationaryImu(time_ns)).ok());
  }
  LidarScan crossing = makePlanarScan(305 * kMillisecondNs);
  crossing.end_time = Timestamp(400 * kMillisecondNs);
  ASSERT_TRUE(pipeline.pushLidar(std::move(crossing)).ok());

  const auto result = pipeline.processNext();

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status_after, EstimatorStatus::kLost);
  EXPECT_EQ(result->diagnostics.consecutive_uncorrected_lidar_updates, 3U);
  EXPECT_FALSE(result->diagnostics.navigation_valid);
  EXPECT_TRUE(result->diagnostics.map_insertion_frozen);
  EXPECT_EQ(pipeline.registrationMapSize(), map_size);

  for (const auto time_ns :
       {410 * kMillisecondNs, 490 * kMillisecondNs,
        500 * kMillisecondNs}) {
    ASSERT_TRUE(pipeline.pushImu(stationaryImu(time_ns)).ok());
  }
  LidarScan while_lost = makePlanarScan(405 * kMillisecondNs);
  while_lost.end_time = Timestamp(500 * kMillisecondNs);
  ASSERT_TRUE(pipeline.pushLidar(std::move(while_lost)).ok());
  const auto still_lost = pipeline.processNext();
  ASSERT_TRUE(still_lost.has_value());
  EXPECT_EQ(still_lost->status_after, EstimatorStatus::kLost);
  EXPECT_EQ(still_lost->diagnostics.consecutive_uncorrected_lidar_updates,
            4U);
}

TEST(FastLioPipelineTest, LongGapLostRecoveryRequiresConfirmation) {
  auto config = testConfig();
  config.synchronization.maximum_imu_gap_ns = 20 * kMillisecondNs;
  config.tracking.maximum_recoverable_imu_gap_ns = 50 * kMillisecondNs;
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
  const std::size_t map_size = pipeline.registrationMapSize();
  for (const auto time_ns :
       {100 * kMillisecondNs, 110 * kMillisecondNs,
        200 * kMillisecondNs, 210 * kMillisecondNs}) {
    ASSERT_TRUE(pipeline.pushImu(stationaryImu(time_ns)).ok());
  }
  LidarScan crossing = makePlanarScan(105 * kMillisecondNs);
  crossing.end_time = Timestamp(210 * kMillisecondNs);
  ASSERT_TRUE(pipeline.pushLidar(std::move(crossing)).ok());
  const auto lost = pipeline.processNext();
  ASSERT_TRUE(lost.has_value());
  EXPECT_EQ(lost->status_after, EstimatorStatus::kLost);
  EXPECT_EQ(pipeline.registrationMapSize(), map_size);
  ASSERT_TRUE(pipeline.stateTime().has_value());
  ASSERT_TRUE(pipeline.synchronizationEpoch().has_value());
  EXPECT_EQ(*pipeline.stateTime(), *pipeline.synchronizationEpoch());

  std::int64_t start_ns = 200 * kMillisecondNs;
  const auto process_scan = [&](double z_m) {
    const std::int64_t end_ns = start_ns + 100 * kMillisecondNs;
    ProcessResult result = pipeline.process(
        makeGroup(makePlanarScan(end_ns, z_m), start_ns,
                  {stationaryImu(start_ns),
                   stationaryImu(start_ns + 50 * kMillisecondNs),
                   stationaryImu(end_ns)}));
    start_ns = end_ns;
    return result;
  };
  for (std::size_t recovery_index = 1; recovery_index <= 2;
       ++recovery_index) {
    const ProcessResult recovered = process_scan(0.0);
    ASSERT_EQ(recovered.lidar_update_status, LidarUpdateStatus::kSucceeded);
    EXPECT_EQ(recovered.status_after, EstimatorStatus::kDegraded);
    EXPECT_EQ(recovered.diagnostics.consecutive_recovery_successes,
              recovery_index);
    EXPECT_EQ(pipeline.registrationMapSize(), map_size);
  }
  const ProcessResult interrupted = process_scan(1.0);
  EXPECT_EQ(interrupted.lidar_update_status, LidarUpdateStatus::kRejected);
  EXPECT_EQ(interrupted.status_after, EstimatorStatus::kDegraded);
  EXPECT_EQ(interrupted.diagnostics.consecutive_recovery_successes, 0U);

  for (std::size_t recovery_index = 1; recovery_index <= 3;
       ++recovery_index) {
    const ProcessResult recovered = process_scan(0.0);
    ASSERT_EQ(recovered.lidar_update_status, LidarUpdateStatus::kSucceeded);
    EXPECT_EQ(recovered.status_after,
              recovery_index < 3 ? EstimatorStatus::kDegraded
                                 : EstimatorStatus::kTracking);
    EXPECT_EQ(recovered.diagnostics.consecutive_recovery_successes,
              recovery_index < 3 ? recovery_index : 0U);
    EXPECT_EQ(recovered.diagnostics.map.map_update_performed,
              recovery_index == 3);
  }
}

TEST(FastLioPipelineTest, RepeatedDiscontinuityInflationRemainsValid) {
  auto config = testConfig();
  config.synchronization.maximum_imu_gap_ns = 20 * kMillisecondNs;
  config.tracking.maximum_recoverable_imu_gap_ns = 50 * kMillisecondNs;
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

  std::int64_t epoch_ns = 100 * kMillisecondNs;
  // This exceeds the number of recovery inflations in the five-minute SITL
  // reproduction. Without bounded eigenvalues, 10^N growth eventually makes
  // IKFoM correction and then prediction non-finite.
  for (std::size_t index = 0; index < 40; ++index) {
    const std::int64_t before_gap_ns = epoch_ns + 10 * kMillisecondNs;
    const std::int64_t resume_ns = epoch_ns + 40 * kMillisecondNs;
    const std::int64_t end_ns = epoch_ns + 50 * kMillisecondNs;
    if (index == 0U) {
      ASSERT_TRUE(pipeline.pushImu(stationaryImu(epoch_ns)).ok());
      ASSERT_TRUE(pipeline.pushImu(stationaryImu(before_gap_ns)).ok());
    }
    for (const auto time_ns : {resume_ns, end_ns}) {
      ASSERT_TRUE(pipeline.pushImu(stationaryImu(time_ns)).ok());
    }
    LidarScan crossing = makePlanarScan(epoch_ns + 5 * kMillisecondNs);
    crossing.end_time = Timestamp(end_ns);
    ASSERT_TRUE(pipeline.pushLidar(std::move(crossing)).ok());
    ASSERT_TRUE(pipeline.processNext().has_value());
    const auto covariance = pipeline.covariance();
    EXPECT_TRUE(covariance.allFinite());
    EXPECT_LT((covariance - covariance.transpose()).cwiseAbs().maxCoeff(),
              1e-8);
    Eigen::SelfAdjointEigenSolver<ManifoldState::Covariance> solver(
        0.5 * (covariance + covariance.transpose()),
        Eigen::EigenvaluesOnly);
    ASSERT_EQ(solver.info(), Eigen::Success);
    EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
    EXPECT_LE(solver.eigenvalues().maxCoeff(), 100.0 + 1e-9);
    epoch_ns = resume_ns;
  }

  for (std::size_t recovery_index = 0; recovery_index < 3; ++recovery_index) {
    const std::int64_t end_ns = epoch_ns + 100 * kMillisecondNs;
    const ProcessResult recovered = pipeline.process(makeGroup(
        makePlanarScan(end_ns), epoch_ns,
        {stationaryImu(epoch_ns),
         stationaryImu(epoch_ns + 50 * kMillisecondNs),
         stationaryImu(end_ns)}));
    EXPECT_EQ(recovered.lidar_update_status, LidarUpdateStatus::kSucceeded)
        << recovered.rejection_reason;
    EXPECT_TRUE(pipeline.state().allFinite());
    EXPECT_TRUE(pipeline.covariance().allFinite());
    epoch_ns = end_ns;
  }
  const EstimatorDiagnostics diagnostics = pipeline.diagnostics();
  EXPECT_GT(diagnostics.recovery_covariance_clamp_count, 0U);
  EXPECT_GT(
      diagnostics.recovery_covariance_maximum_eigenvalue_before_clamp,
      100.0);
  EXPECT_LE(diagnostics.recovery_covariance_maximum_eigenvalue_after_clamp,
            100.0 + 1e-9);
}

TEST(FastLioPipelineTest, InitialMapShortGapKeepsBootstrapForCorrection) {
  auto config = testConfig();
  config.synchronization.maximum_imu_gap_ns = 20 * kMillisecondNs;
  config.tracking.maximum_recoverable_imu_gap_ns = 50 * kMillisecondNs;
  FastLioPipeline pipeline(config);
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs),
                                  stationaryImu(0)})));
  for (const auto time_ns :
       {std::int64_t{0}, 10 * kMillisecondNs, 40 * kMillisecondNs,
        50 * kMillisecondNs, 60 * kMillisecondNs}) {
    ASSERT_TRUE(pipeline.pushImu(stationaryImu(time_ns)).ok());
  }
  LidarScan crossing = makePlanarScan(5 * kMillisecondNs);
  crossing.end_time = Timestamp(45 * kMillisecondNs);
  ASSERT_TRUE(pipeline.pushLidar(std::move(crossing)).ok());
  ASSERT_TRUE(pipeline.processNext().has_value());
  ASSERT_EQ(pipeline.status(), EstimatorStatus::kInitializingMap);
  ASSERT_TRUE(
      pipeline.pushLidar(makePlanarScan(60 * kMillisecondNs)).ok());

  const auto following = pipeline.processNext();

  ASSERT_TRUE(following.has_value());
  EXPECT_TRUE(following->diagnostics.registration.correction_attempted);
}

TEST(FastLioPipelineTest, InitialMapLongGapRestartsBootstrapAcquisition) {
  auto config = testConfig();
  config.synchronization.maximum_imu_gap_ns = 20 * kMillisecondNs;
  config.tracking.maximum_recoverable_imu_gap_ns = 50 * kMillisecondNs;
  FastLioPipeline pipeline(config);
  static_cast<void>(
      pipeline.process(makeGroup(makePlanarScan(0), 0,
                                 {stationaryImu(-20 * kMillisecondNs),
                                  stationaryImu(-10 * kMillisecondNs),
                                  stationaryImu(0)})));
  for (const auto time_ns :
       {std::int64_t{0}, 10 * kMillisecondNs, 100 * kMillisecondNs,
        110 * kMillisecondNs, 120 * kMillisecondNs}) {
    ASSERT_TRUE(pipeline.pushImu(stationaryImu(time_ns)).ok());
  }
  LidarScan crossing = makePlanarScan(5 * kMillisecondNs);
  crossing.end_time = Timestamp(105 * kMillisecondNs);
  ASSERT_TRUE(pipeline.pushLidar(std::move(crossing)).ok());
  const auto discontinuity = pipeline.processNext();
  ASSERT_TRUE(discontinuity.has_value());
  EXPECT_EQ(discontinuity->status_after, EstimatorStatus::kInitializingMap);
  EXPECT_EQ(discontinuity->rejection_reason,
            "INITIAL_MAP_RESTART_AFTER_LONG_IMU_DISCONTINUITY");
  ASSERT_TRUE(
      pipeline.pushLidar(makePlanarScan(120 * kMillisecondNs)).ok());

  const auto reference = pipeline.processNext();

  ASSERT_TRUE(reference.has_value());
  EXPECT_EQ(reference->rejection_reason, "INITIAL_MAP_REFERENCE_CAPTURED");
  EXPECT_FALSE(reference->diagnostics.registration.correction_attempted);
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
  ASSERT_GT(pipeline.registrationMapSize(), 0U);

  pipeline.reset();

  EXPECT_EQ(pipeline.status(), EstimatorStatus::kWaitingForSensors);
  EXPECT_EQ(pipeline.registrationMapSize(), 0U);
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

}  // namespace
}  // namespace uav::nav::lio
