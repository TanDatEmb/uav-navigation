#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "px4_navigation_external_mode/px4_tracking_adapter.hpp"

namespace {

using namespace px4_navigation_external_mode::tracking_adapter;

Reference makeReference() {
  Reference reference;
  reference.identity.localization_epoch = 4U;
  reference.identity.mission_id = "mission";
  reference.identity.goal_epoch = 7U;
  reference.identity.request_id = 11U;
  reference.identity.bundle_generation = 13U;
  reference.identity.sample_id = 17U;
  reference.identity.reference_sample_time_ns = 2'000'000'000LL;
  reference.identity.lease_valid_until_ns = 2'100'000'000LL;
  reference.position_enu = Eigen::Vector3d{11.0, 22.0, 33.0};
  reference.velocity_enu = Eigen::Vector3d{1.0, 2.0, 3.0};
  reference.acceleration_enu = Eigen::Vector3d{4.0, 5.0, 6.0};
  reference.yaw_enu = 0.4;
  reference.yaw_rate_enu_rad_s = 0.7;
  return reference;
}

LioState makeLio() {
  LioState lio;
  lio.position_enu = Eigen::Vector3d{10.0, 20.0, 30.0};
  lio.velocity_enu = Eigen::Vector3d{0.5, 0.5, 0.5};
  lio.yaw_enu = 0.1;
  lio.position_valid = true;
  lio.velocity_valid = true;
  lio.orientation_valid = true;
  lio.navigation_valid = true;
  lio.covariance_valid = true;
  lio.observability_valid = true;
  lio.correction_fresh = true;
  lio.propagation_valid = true;
  lio.relative_heading_valid = true;
  lio.tilt_valid = true;
  lio.extrinsic_valid = true;
  lio.localization_epoch = 4U;
  lio.sequence = 19U;
  lio.source_stamp_ns = 990'000'000LL;
  lio.receive_steady_ns = 2'000'000'000LL;
  return lio;
}

RawPx4State makeRawPx4() {
  RawPx4State raw;
  raw.position_ned = Eigen::Vector3d{100.0, 200.0, -300.0};
  raw.velocity_ned = Eigen::Vector3d{8.0, 9.0, 10.0};
  raw.yaw_ned = 0.2;
  raw.position_valid = {true, true, true};
  raw.velocity_valid = {true, true, true};
  raw.heading_valid = true;
  raw.heading_good_for_control = true;
  raw.heading_variance_rad2 = 0.01;
  raw.timestamp_us = 3'000'000U;
  raw.timestamp_sample_us = 2'999'000U;
  raw.receive_steady_ns = 2'000'000'000LL;
  raw.reset_counters = ResetCounters{1U, 2U, 3U, 4U, 5U};
  return raw;
}

TimingWitness makeTiming() {
  TimingWitness timing;
  timing.reference_sample_id = 17U;
  timing.lio_localization_epoch = 4U;
  timing.lio_sequence = 19U;
  timing.px4_timestamp_us = 3'000'000U;
  timing.px4_timestamp_sample_us = 2'999'000U;
  timing.clock_mapping_generation = 2U;
  timing.conservative_bound_model = TimingWitness::kConservativeBoundModelV1;
  timing.common_time_contract_valid = true;
  timing.clock_mapping_uncertainty_s = 0.001;
  timing.expected_reference_use_time_ns = 2'010'000'000LL;
  timing.reference_age_s = 0.010;
  timing.pair_skew_s = 0.002;
  timing.lio_source_age_s = 0.010;
  timing.lio_receive_age_s = 0.008;
  timing.px4_source_age_s = 0.012;
  timing.px4_receive_age_s = 0.006;
  timing.predicted_anchor_age_s = 0.014;
  timing.output_transport_age_s = 0.004;
  timing.px4_consume_age_s = 0.005;
  timing.total_bound_s = 0.024;
  return timing;
}

Policy makePolicy() {
  Policy policy;
  policy.mode = Mode::kShadow;
  policy.experiment_id = "px4-level-a-shadow-v1";
  policy.maximum_timing_bound_s = 0.025;
  policy.maximum_reference_age_s = 0.025;
  policy.expected_px4_reset_counters = ResetCounters{1U, 2U, 3U, 4U, 5U};
  policy.expected_lio_localization_epoch = 4U;
  return policy;
}

}  // namespace

TEST(Px4TrackingAdapter, AppliesRelativeFrameAndCarriesImmutableWitness) {
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), makeTiming(), makePolicy());
  ASSERT_TRUE(result.success());
  ASSERT_TRUE(result.output.has_value());
  const auto& output = *result.output;
  const Eigen::Vector3d expected_error{1.0, 2.0, 3.0};
  EXPECT_TRUE(output.witness.error_lio_enu.isApprox(expected_error));
  EXPECT_TRUE(output.witness.error_adapter_ned.isApprox(
      output.position_ned - makeRawPx4().position_ned));
  EXPECT_TRUE(output.witness.rotation_lio_enu_to_px4_ned.allFinite());
  EXPECT_EQ(output.witness.identity.sample_id, 17U);
  EXPECT_EQ(output.witness.experiment_id, "px4-level-a-shadow-v1");
  EXPECT_EQ(output.witness.px4_reset_counters, (ResetCounters{1U, 2U, 3U, 4U, 5U}));
  EXPECT_EQ(output.witness.mode, Mode::kShadow);
  EXPECT_DOUBLE_EQ(output.yaw_rate_ned_rad_s, -0.7);
}

TEST(Px4TrackingAdapter, BasisIdentityMapsEastNorthUpToEastNorthDown) {
  auto reference = makeReference();
  auto lio = makeLio();
  auto raw = makeRawPx4();
  reference.position_enu = lio.position_enu + Eigen::Vector3d{1.0, 0.0, 0.0};
  reference.velocity_enu = Eigen::Vector3d{1.0, 0.0, 0.0};
  reference.acceleration_enu = Eigen::Vector3d{0.0, 0.0, 1.0};
  lio.yaw_enu = std::numbers::pi_v<double> / 2.0 - raw.yaw_ned;
  const auto result = adapt(reference, lio, raw, makeTiming(), makePolicy());
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.output->witness.error_adapter_ned.isApprox(Eigen::Vector3d{0.0, 1.0, 0.0}));
  EXPECT_TRUE(result.output->velocity_ned.isApprox(Eigen::Vector3d{0.0, 1.0, 0.0}));
  EXPECT_TRUE(result.output->acceleration_ned.isApprox(Eigen::Vector3d{0.0, 0.0, -1.0}));
}

TEST(Px4TrackingAdapter, AppliesNinetyDegreeRelativeHeadingWithoutChangingBasisContract) {
  auto reference = makeReference();
  auto lio = makeLio();
  auto raw = makeRawPx4();
  reference.position_enu = lio.position_enu + Eigen::Vector3d{1.0, 0.0, 0.0};
  reference.velocity_enu = Eigen::Vector3d{1.0, 0.0, 0.0};
  reference.acceleration_enu = Eigen::Vector3d{0.0, 0.0, 0.0};
  lio.yaw_enu = 0.0;
  raw.yaw_ned = 0.0;
  const auto result = adapt(reference, lio, raw, makeTiming(), makePolicy());
  ASSERT_TRUE(result.success());
  EXPECT_TRUE(result.output->witness.error_adapter_ned.isApprox(Eigen::Vector3d{1.0, 0.0, 0.0}));
  EXPECT_TRUE(result.output->velocity_ned.isApprox(Eigen::Vector3d{1.0, 0.0, 0.0}));
}

TEST(Px4TrackingAdapter, VelocityOnlyUsesLioPathErrorAndLeavesPositionAccelerationUnset) {
  auto reference = makeReference();
  auto lio = makeLio();
  auto raw = makeRawPx4();
  auto policy = makePolicy();
  policy.boundary = SetpointBoundary::kVelocityOnly;
  policy.lio_position_feedback_gain_s_inv = 0.5;
  policy.maximum_velocity_mps = 3.0;
  reference.velocity_enu = Eigen::Vector3d::Zero();
  lio.yaw_enu = std::numbers::pi_v<double> / 2.0 - raw.yaw_ned;

  const auto result = adapt(reference, lio, raw, makeTiming(), policy);
  ASSERT_TRUE(result.success());
  ASSERT_TRUE(result.output.has_value());
  const auto& output = *result.output;
  const Eigen::Vector3d expected_lio_velocity{0.5, 1.0, 1.5};
  EXPECT_TRUE(output.witness.velocity_command_lio_enu.isApprox(expected_lio_velocity));
  EXPECT_TRUE(output.velocity_ned.isApprox(Eigen::Vector3d{1.0, 0.5, -1.5}));
  EXPECT_TRUE(output.position_ned.array().isNaN().all());
  EXPECT_TRUE(output.acceleration_ned.array().isNaN().all());
  EXPECT_TRUE(output.witness.error_adapter_ned.array().isNaN().all());
  EXPECT_FALSE(output.witness.velocity_limited);
  EXPECT_EQ(output.witness.boundary, SetpointBoundary::kVelocityOnly);
}

TEST(Px4TrackingAdapter, VelocityOnlyBoundsLioOwnedCommandWithoutPx4PositionFeedback) {
  auto reference = makeReference();
  auto lio = makeLio();
  auto raw = makeRawPx4();
  raw.position_ned = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  raw.position_valid = {false, false, false};
  auto policy = makePolicy();
  policy.boundary = SetpointBoundary::kVelocityOnly;
  policy.lio_position_feedback_gain_s_inv = 1.0;
  policy.maximum_velocity_mps = 2.0;
  reference.velocity_enu = Eigen::Vector3d{4.0, 0.0, 0.0};
  reference.position_enu = lio.position_enu + Eigen::Vector3d{3.0, 0.0, 0.0};

  const auto result = adapt(reference, lio, raw, makeTiming(), policy);
  ASSERT_TRUE(result.success());
  ASSERT_TRUE(result.output.has_value());
  EXPECT_NEAR(result.output->witness.velocity_command_lio_enu.norm(), 2.0, 1.0e-12);
  EXPECT_TRUE(result.output->velocity_ned.allFinite());
  EXPECT_TRUE(result.output->position_ned.array().isNaN().all());
  EXPECT_TRUE(result.output->acceleration_ned.array().isNaN().all());
  EXPECT_TRUE(result.output->witness.velocity_limited);
}

TEST(Px4TrackingAdapter, RejectsDisabledPolicyWithoutReturningSetpoint) {
  auto policy = makePolicy();
  policy.mode = Mode::kOff;
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), makeTiming(), policy);
  EXPECT_FALSE(result.success());
  EXPECT_FALSE(result.output.has_value());
  EXPECT_EQ(result.failure, FailureReason::kPolicyDisabled);
}

TEST(Px4TrackingAdapter, RejectsNonZeroVelocityLambda) {
  auto policy = makePolicy();
  policy.velocity_lambda = 0.01;
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), makeTiming(), policy);
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kInvalidPolicy);
}

TEST(Px4TrackingAdapter, RejectsVelocityOnlyPolicyWithoutExplicitBoundedGain) {
  auto policy = makePolicy();
  policy.boundary = SetpointBoundary::kVelocityOnly;
  policy.maximum_velocity_mps = 2.0;
  EXPECT_EQ(adapt(makeReference(), makeLio(), makeRawPx4(), makeTiming(), policy).failure,
            FailureReason::kInvalidPolicy);

  policy.lio_position_feedback_gain_s_inv = 0.5;
  policy.maximum_velocity_mps.reset();
  EXPECT_EQ(adapt(makeReference(), makeLio(), makeRawPx4(), makeTiming(), policy).failure,
            FailureReason::kInvalidPolicy);

  policy.maximum_velocity_mps = 0.0;
  EXPECT_EQ(adapt(makeReference(), makeLio(), makeRawPx4(), makeTiming(), policy).failure,
            FailureReason::kInvalidPolicy);
}

TEST(Px4TrackingAdapter, RejectsHeadingWhenTiltOrExtrinsicIsInvalid) {
  auto lio = makeLio();
  lio.tilt_valid = false;
  const auto result = adapt(makeReference(), lio, makeRawPx4(), makeTiming(), makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kRelativeHeadingInvalid);
}

TEST(Px4TrackingAdapter, RejectsResetMismatchAndDeadReckoning) {
  auto raw = makeRawPx4();
  raw.reset_counters.xy++;
  auto result = adapt(makeReference(), makeLio(), raw, makeTiming(), makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kResetMismatch);

  raw = makeRawPx4();
  raw.dead_reckoning = true;
  result = adapt(makeReference(), makeLio(), raw, makeTiming(), makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kDeadReckoning);
}

TEST(Px4TrackingAdapter, RejectsStaleTimingBeforeProducingOutput) {
  auto timing = makeTiming();
  timing.total_bound_s = 0.030;
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), timing, makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_FALSE(result.output.has_value());
  EXPECT_EQ(result.failure, FailureReason::kInvalidTiming);
}

TEST(Px4TrackingAdapter, RejectsTimingBoundThatDoesNotCoverSnapshotAges) {
  auto timing = makeTiming();
  timing.predicted_anchor_age_s = 1.0;
  timing.total_bound_s = 0.0;
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), timing, makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kInvalidTiming);
}

TEST(Px4TrackingAdapter, RejectsTimingWitnessForAnotherSnapshot) {
  auto timing = makeTiming();
  timing.lio_sequence++;
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), timing, makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kInvalidTiming);
}

TEST(Px4TrackingAdapter, RejectsStaleReferenceInsideAValidLease) {
  auto timing = makeTiming();
  timing.expected_reference_use_time_ns = 2'050'000'000LL;
  timing.reference_age_s = 0.050;
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), timing, makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kInvalidTiming);
}

TEST(Px4TrackingAdapter, RejectsReferenceSampleFromTheFutureOfEvaluationTime) {
  auto timing = makeTiming();
  timing.expected_reference_use_time_ns = 1'990'000'000LL;
  timing.reference_age_s = 0.0;
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), timing, makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kInvalidTiming);
}

TEST(Px4TrackingAdapter, LevelARequiresExplicitTimingReferenceAgeAndResetPolicy) {
  auto policy = makePolicy();
  policy.mode = Mode::kLevelA;
  policy.maximum_timing_bound_s.reset();
  EXPECT_EQ(adapt(makeReference(), makeLio(), makeRawPx4(), makeTiming(), policy).failure,
            FailureReason::kInvalidPolicy);

  policy = makePolicy();
  policy.mode = Mode::kLevelA;
  policy.maximum_reference_age_s.reset();
  EXPECT_EQ(adapt(makeReference(), makeLio(), makeRawPx4(), makeTiming(), policy).failure,
            FailureReason::kInvalidPolicy);

  policy = makePolicy();
  policy.mode = Mode::kLevelA;
  policy.expected_px4_reset_counters.reset();
  EXPECT_EQ(adapt(makeReference(), makeLio(), makeRawPx4(), makeTiming(), policy).failure,
            FailureReason::kInvalidPolicy);
}

TEST(Px4TrackingAdapter, RejectsUseAfterReferenceLease) {
  auto timing = makeTiming();
  timing.expected_reference_use_time_ns = 2'100'000'001LL;
  timing.reference_age_s = 0.100000001;
  timing.total_bound_s = 0.110000001;
  auto policy = makePolicy();
  policy.maximum_reference_age_s.reset();
  policy.maximum_timing_bound_s.reset();
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), timing, policy);
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kInvalidReference);
}

TEST(Px4TrackingAdapter, RejectsInvalidPacketWithoutRefreshingIt) {
  auto raw = makeRawPx4();
  raw.position_valid[2] = false;
  const auto result = adapt(makeReference(), makeLio(), raw, makeTiming(), makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.failure, FailureReason::kInvalidPx4State);

  auto lio = makeLio();
  lio.source_stamp_ns = 0;
  const auto lio_result = adapt(makeReference(), lio, makeRawPx4(), makeTiming(), makePolicy());
  EXPECT_FALSE(lio_result.success());
  EXPECT_EQ(lio_result.failure, FailureReason::kInvalidLioState);
}

TEST(Px4TrackingAdapter, RejectsNonFiniteInputsWithoutDefaultOutput) {
  auto reference = makeReference();
  reference.position_enu.x() = std::numeric_limits<double>::quiet_NaN();
  const auto result = adapt(reference, makeLio(), makeRawPx4(), makeTiming(), makePolicy());
  EXPECT_FALSE(result.success());
  EXPECT_FALSE(result.output.has_value());
  EXPECT_EQ(result.failure, FailureReason::kInvalidReference);
}
