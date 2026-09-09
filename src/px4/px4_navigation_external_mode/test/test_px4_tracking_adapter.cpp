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
  reference.identity.reference_sample_time_ns = 1'000'000'000LL;
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
  timing.clock_mapping_generation = 2U;
  timing.clock_mapping_uncertainty_s = 0.001;
  timing.expected_reference_use_time_ns = 2'010'000'000LL;
  timing.pair_skew_s = 0.002;
  timing.lio_source_age_s = 0.010;
  timing.lio_receive_age_s = 0.008;
  timing.px4_source_age_s = 0.012;
  timing.px4_receive_age_s = 0.006;
  timing.predicted_anchor_age_s = 0.014;
  timing.output_transport_age_s = 0.004;
  timing.px4_consume_age_s = 0.005;
  timing.total_bound_s = 0.020;
  return timing;
}

Policy makePolicy() {
  Policy policy;
  policy.mode = Mode::kShadow;
  policy.experiment_id = "px4-level-a-shadow-v1";
  policy.maximum_timing_bound_s = 0.025;
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

TEST(Px4TrackingAdapter, RejectsUseAfterReferenceLease) {
  auto timing = makeTiming();
  timing.expected_reference_use_time_ns = 2'100'000'001LL;
  const auto result = adapt(makeReference(), makeLio(), makeRawPx4(), timing, makePolicy());
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
