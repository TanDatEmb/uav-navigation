#include <gtest/gtest.h>

#include <vector>

#include "fast_lio_core/propagation/imu_state_propagator.hpp"

namespace uav::nav::lio {
namespace {

ImuSample sample(std::int64_t time_ns) {
  return ImuSample{Timestamp(time_ns), {0.01, -0.02, 0.03},
                   {0.1, -0.1, 9.80665}};
}

StateEstimate correction(std::int64_t time_ns) {
  StateEstimate result;
  result.time = Timestamp(time_ns);
  result.state.set_position_odom_imu_m({1.0, 2.0, 3.0});
  result.covariance = 0.001 * ManifoldState::Covariance::Identity();
  return result;
}

TEST(ImuStatePropagatorTest, ReanchorsBetweenSamplesAndPropagatesForward) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  for (std::int64_t time_ns = 0; time_ns <= 100'000'000;
       time_ns += 10'000'000) {
    ASSERT_TRUE(propagator.acceptImu(sample(time_ns)).ok());
  }
  ASSERT_TRUE(propagator.reanchorAndReplay(correction(55'000'000)).ok());
  ASSERT_TRUE(propagator.valid());
  ASSERT_TRUE(propagator.estimate().has_value());
  EXPECT_EQ(propagator.estimate()->time.nanoseconds(), 100'000'000);
  EXPECT_EQ(propagator.diagnostics().anchor_time->nanoseconds(), 55'000'000);
  EXPECT_EQ(propagator.diagnostics().last_replay_sample_count, 6U);

  ASSERT_TRUE(propagator.acceptImu(sample(110'000'000)).ok());
  ASSERT_TRUE(propagator.flushPendingPrediction().ok());
  EXPECT_EQ(propagator.estimate()->time.nanoseconds(), 110'000'000);
}

TEST(ImuStatePropagatorTest, RejectsTimestampRegressionAndInvalidatesOutput) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  ASSERT_TRUE(propagator.acceptImu(sample(0)).ok());
  ASSERT_TRUE(propagator.acceptImu(sample(10'000'000)).ok());
  ASSERT_TRUE(propagator.reanchorAndReplay(correction(0)).ok());
  ASSERT_TRUE(propagator.valid());

  const auto rejected = propagator.acceptImu(sample(10'000'000));
  EXPECT_EQ(rejected.code(), StatusCode::kTimestampRegression);
  EXPECT_FALSE(propagator.valid());
  EXPECT_TRUE(propagator.diagnostics().requires_reanchor);
  EXPECT_FALSE(propagator.estimate().has_value());
  EXPECT_EQ(propagator.diagnostics().timestamp_regression_count, 1U);
}

TEST(ImuStatePropagatorTest, RecordsReplayImuWithoutPrediction) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  ASSERT_TRUE(propagator.recordImuForReplay(sample(0)).ok());
  ASSERT_TRUE(propagator.recordImuForReplay(sample(10'000'000)).ok());
  EXPECT_FALSE(propagator.valid());
  EXPECT_FALSE(propagator.estimate().has_value());
  ASSERT_TRUE(propagator.diagnostics().latest_imu_time.has_value());
  EXPECT_EQ(propagator.diagnostics().latest_imu_time->nanoseconds(),
            10'000'000);
  EXPECT_FALSE(propagator.diagnostics().propagated_time.has_value());
  EXPECT_EQ(propagator.diagnostics().current_imu_history_size, 2U);
}

TEST(ImuStatePropagatorTest, MissingBracketInvalidatesWithoutChangingState) {
  ImuStatePropagatorConfig config;
  config.imu_history_duration_ns = 30'000'000;
  ImuStatePropagator propagator(config);
  for (std::int64_t time_ns = 0; time_ns <= 100'000'000;
       time_ns += 10'000'000) {
    ASSERT_TRUE(propagator.acceptImu(sample(time_ns)).ok());
  }
  const Status failure = propagator.reanchorAndReplay(correction(20'000'000));
  EXPECT_EQ(failure.code(), StatusCode::kMissingStartBracket);
  EXPECT_FALSE(propagator.valid());
  EXPECT_EQ(propagator.diagnostics().missing_bracket_count, 1U);
}

TEST(ImuStatePropagatorTest, ReplayGapRollsBackStateAndCovariance) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  for (std::int64_t time_ns = 0; time_ns <= 40'000'000;
       time_ns += 10'000'000) {
    ASSERT_TRUE(propagator.acceptImu(sample(time_ns)).ok());
  }
  ASSERT_TRUE(propagator.reanchorAndReplay(correction(0)).ok());
  const auto before = propagator.estimate();
  ASSERT_TRUE(before.has_value());
  ASSERT_TRUE(propagator.acceptImu(sample(50'000'000)).ok());

  StateEstimate newer = correction(5'000'000);
  newer.state.set_position_odom_imu_m({9.0, 8.0, 7.0});
  const Status failure = propagator.reanchorAndReplay(newer);
  ASSERT_TRUE(failure.ok()) << failure.message();
  ASSERT_TRUE(propagator.estimate().has_value());

  // A later oversized interval starts a new continuity epoch and is retained.
  const auto state_before_gap = propagator.estimate();
  const auto gap = propagator.acceptImu(sample(80'000'001));
  EXPECT_EQ(gap.code(), StatusCode::kImuGap);
  EXPECT_EQ(gap.disposition, ImuRecordDisposition::kContinuityRestarted);
  EXPECT_FALSE(propagator.valid());
  EXPECT_FALSE(propagator.estimate().has_value());
  EXPECT_FALSE(propagator.diagnostics().propagated_time.has_value());
  EXPECT_NE(propagator.diagnostics().latest_imu_time->nanoseconds(),
            state_before_gap->time.nanoseconds());
  EXPECT_EQ(propagator.diagnostics().current_imu_history_size, 1U);
}

TEST(ImuStatePropagatorTest, ForwardGapStartsNewContinuityEpoch) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  ASSERT_TRUE(propagator.acceptImu(sample(0)).ok());
  ASSERT_TRUE(propagator.acceptImu(sample(10'000'000)).ok());
  ASSERT_TRUE(propagator.reanchorAndReplay(correction(0)).ok());
  ASSERT_TRUE(propagator.valid());

  const auto restart = propagator.acceptImu(sample(40'000'000));
  EXPECT_EQ(restart.disposition, ImuRecordDisposition::kContinuityRestarted);
  EXPECT_EQ(restart.code(), StatusCode::kImuGap);
  EXPECT_FALSE(propagator.valid());
  EXPECT_TRUE(propagator.diagnostics().requires_reanchor);
  EXPECT_EQ(propagator.diagnostics().continuity_epoch, 1U);
  EXPECT_EQ(propagator.diagnostics().continuity_reset_count, 1U);
  ASSERT_TRUE(propagator.diagnostics().last_continuity_reset_time.has_value());
  EXPECT_EQ(propagator.diagnostics().last_continuity_reset_time->nanoseconds(),
            40'000'000);
  EXPECT_EQ(propagator.diagnostics().latest_imu_time->nanoseconds(),
            40'000'000);
  EXPECT_EQ(propagator.diagnostics().current_imu_history_size, 1U);
}

TEST(ImuStatePropagatorTest, SampleAfterGapIsRecordedAsNewEpochStart) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  ASSERT_TRUE(propagator.acceptImu(sample(0)).ok());
  ASSERT_TRUE(propagator.acceptImu(sample(40'000'000)).disposition ==
              ImuRecordDisposition::kContinuityRestarted);
  ASSERT_TRUE(propagator.acceptImu(sample(50'000'000)).ok());
  ASSERT_TRUE(propagator.recordImuForReplay(sample(60'000'000)).ok());
  EXPECT_EQ(propagator.diagnostics().continuity_epoch, 1U);
  EXPECT_EQ(propagator.diagnostics().latest_imu_time->nanoseconds(),
            60'000'000);
  EXPECT_EQ(propagator.diagnostics().current_imu_history_size, 3U);
  EXPECT_FALSE(propagator.diagnostics().propagated_time.has_value());
}

TEST(ImuStatePropagatorTest, SamplesContinueAdvancingAfterGap) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  ASSERT_TRUE(propagator.acceptImu(sample(0)).ok());
  ASSERT_EQ(propagator.acceptImu(sample(40'000'000)).disposition,
             ImuRecordDisposition::kContinuityRestarted);
  for (const std::int64_t time_ns : {50'000'000LL, 60'000'000LL}) {
    EXPECT_EQ(propagator.acceptImu(sample(time_ns)).disposition,
              ImuRecordDisposition::kRecorded);
  }
  EXPECT_EQ(propagator.diagnostics().latest_imu_time->nanoseconds(),
            60'000'000);
}

TEST(ImuStatePropagatorTest, GapRequiresNewCorrectionBeforePrediction) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  ASSERT_TRUE(propagator.acceptImu(sample(0)).ok());
  ASSERT_TRUE(propagator.acceptImu(sample(10'000'000)).ok());
  ASSERT_TRUE(propagator.reanchorAndReplay(correction(0)).ok());
  ASSERT_TRUE(propagator.acceptImu(sample(40'000'000)).disposition ==
              ImuRecordDisposition::kContinuityRestarted);
  ASSERT_TRUE(propagator.acceptImu(sample(50'000'000)).ok());
  EXPECT_TRUE(propagator.flushPendingPrediction().ok());
  EXPECT_FALSE(propagator.valid());
  EXPECT_FALSE(propagator.estimate().has_value());
}

TEST(ImuStatePropagatorTest, SuccessfulCorrectionClearsRequiresReanchor) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  ASSERT_TRUE(propagator.acceptImu(sample(0)).ok());
  ASSERT_TRUE(propagator.acceptImu(sample(10'000'000)).ok());
  ASSERT_TRUE(propagator.reanchorAndReplay(correction(0)).ok());
  ASSERT_TRUE(propagator.acceptImu(sample(40'000'000)).disposition ==
              ImuRecordDisposition::kContinuityRestarted);
  ASSERT_TRUE(propagator.acceptImu(sample(50'000'000)).ok());
  ASSERT_TRUE(propagator.reanchorAndReplay(correction(40'000'000)).ok());
  EXPECT_FALSE(propagator.diagnostics().requires_reanchor);
  EXPECT_TRUE(propagator.valid());
  EXPECT_EQ(propagator.estimate()->time.nanoseconds(), 50'000'000);
}

TEST(ImuStatePropagatorTest, FlushCannotRestoreValidityWithoutCorrection) {
  ImuStatePropagator propagator(ImuStatePropagatorConfig{});
  ASSERT_TRUE(propagator.acceptImu(sample(0)).ok());
  ASSERT_EQ(propagator.acceptImu(sample(40'000'000)).disposition,
             ImuRecordDisposition::kContinuityRestarted);
  ASSERT_TRUE(propagator.acceptImu(sample(50'000'000)).ok());
  EXPECT_TRUE(propagator.flushPendingPrediction().ok());
  EXPECT_TRUE(propagator.diagnostics().requires_reanchor);
  EXPECT_FALSE(propagator.valid());
}

TEST(ImuStatePropagatorTest, PruningRetainsSampleBeforeCutoff) {
  ImuStatePropagatorConfig config;
  config.imu_history_duration_ns = 25'000'000;
  ImuStatePropagator propagator(config);
  for (std::int64_t time_ns = 0; time_ns <= 100'000'000;
       time_ns += 10'000'000) {
    ASSERT_TRUE(propagator.acceptImu(sample(time_ns)).ok());
  }
  EXPECT_EQ(propagator.diagnostics().current_imu_history_size, 4U);
  EXPECT_TRUE(propagator.reanchorAndReplay(correction(75'000'000)).ok());
}

}  // namespace
}  // namespace uav::nav::lio
