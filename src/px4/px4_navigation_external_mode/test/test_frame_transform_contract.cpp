#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "px4_navigation_external_mode/frame_transform_contract.hpp"

namespace px4_navigation_external_mode {
namespace {

FrameTransformObservation observation(const std::int64_t stamp_ns,
                                      const Eigen::Vector3d& bias,
                                      const Eigen::Vector3d& variance =
                                          Eigen::Vector3d::Constant(0.25),
                                      const Eigen::Vector3d& test_ratio =
                                          Eigen::Vector3d::Constant(0.1),
                                      const bool yaw_align = true,
                                      const std::uint8_t heading_reset_counter = 0U) {
  FrameTransformObservation result;
  result.source_stamp_ns = stamp_ns;
  result.bias_m = bias;
  result.bias_var_m2 = variance;
  result.innov_test_ratio = test_ratio;
  result.yaw_align = yaw_align;
  result.heading_reset_counter = heading_reset_counter;
  return result;
}

TEST(FrameTransformContract, ConfigRequiresSlowFilter) {
  FrameTransformConfig config;
  config.tau_s = 9.999;
  EXPECT_FALSE(config.valid());
}

TEST(FrameTransformContract, FirstSampleInitializesAppliedBias) {
  FrameTransformContract contract;
  const auto result = contract.observe(observation(1'000'000'000, {1.0, 2.0, 3.0}));

  EXPECT_EQ(result.decision, FrameTransformDecision::kApplied);
  ASSERT_TRUE(result.applied_bias_m.has_value());
  EXPECT_TRUE(result.applied_bias_m->isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_EQ(result.version, 1U);
}

TEST(FrameTransformContract, SlowFilterLimitsAppliedMotion) {
  FrameTransformContract contract;
  ASSERT_EQ(contract.observe(observation(1'000'000'000, Eigen::Vector3d::Zero())).decision,
            FrameTransformDecision::kApplied);

  const auto result = contract.observe(observation(2'000'000'000,
                                                   Eigen::Vector3d{0.1, 0.0, 0.0}));
  EXPECT_EQ(result.decision, FrameTransformDecision::kApplied);
  ASSERT_TRUE(result.applied_bias_m.has_value());
  EXPECT_NEAR(result.applied_bias_m->x(), 0.1 / 11.0, 1.0e-12);
  EXPECT_NEAR(result.raw_applied_pressure_m, 0.1, 1.0e-12);
}

TEST(FrameTransformContract, InvalidOrNonMonotonicSampleCannotApply) {
  FrameTransformContract contract;
  ASSERT_EQ(contract.observe(observation(1'000'000'000, Eigen::Vector3d::Zero())).decision,
            FrameTransformDecision::kApplied);
  const auto before = contract.appliedBias();

  auto invalid = observation(2'000'000'000, Eigen::Vector3d::Zero());
  invalid.bias_m.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(contract.observe(invalid).decision, FrameTransformDecision::kUnavailable);
  EXPECT_TRUE(contract.appliedBias()->isApprox(*before));

  EXPECT_EQ(contract.observe(observation(1'500'000'000, Eigen::Vector3d::Zero())).decision,
            FrameTransformDecision::kUnavailable);
  EXPECT_TRUE(contract.appliedBias()->isApprox(*before));
}

TEST(FrameTransformContract, InnovationRejectionRequiresRecertification) {
  FrameTransformContract contract;
  ASSERT_EQ(contract.observe(observation(1'000'000'000, Eigen::Vector3d::Zero())).decision,
            FrameTransformDecision::kApplied);

  const auto result = contract.observe(observation(
      2'000'000'000, Eigen::Vector3d{0.05, 0.0, 0.0},
      Eigen::Vector3d::Constant(0.25), Eigen::Vector3d{1.0, 0.1, 0.1}));
  EXPECT_EQ(result.decision, FrameTransformDecision::kRecertifyRequired);
  EXPECT_TRUE(contract.appliedBias()->isApprox(Eigen::Vector3d::Zero()));
}

TEST(FrameTransformContract, BiasPressureRequiresRecertification) {
  FrameTransformContract contract;
  ASSERT_EQ(contract.observe(observation(1'000'000'000, Eigen::Vector3d::Zero())).decision,
            FrameTransformDecision::kApplied);

  const auto result = contract.observe(observation(
      2'000'000'000, Eigen::Vector3d{0.3, 0.0, 0.0}));
  EXPECT_EQ(result.decision, FrameTransformDecision::kRecertifyRequired);
  EXPECT_GT(result.raw_applied_pressure_m, contract.config().frame_budget_m);
  EXPECT_TRUE(contract.appliedBias()->isApprox(Eigen::Vector3d::Zero()));
}

TEST(FrameTransformContract, VarianceCollapseRequiresRecertification) {
  FrameTransformContract contract;
  ASSERT_EQ(contract.observe(observation(1'000'000'000, Eigen::Vector3d::Zero())).decision,
            FrameTransformDecision::kApplied);

  const auto result = contract.observe(observation(
      2'000'000'000, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
  EXPECT_EQ(result.decision, FrameTransformDecision::kRecertifyRequired);
}

TEST(FrameTransformContract, YawLossDegradesToVelocityOnly) {
  FrameTransformContract contract;
  ASSERT_EQ(contract.observe(observation(1'000'000'000, Eigen::Vector3d::Zero())).decision,
            FrameTransformDecision::kApplied);

  const auto result = contract.observe(observation(
      2'000'000'000, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.25),
      Eigen::Vector3d::Constant(0.1), false));
  EXPECT_EQ(result.decision, FrameTransformDecision::kDegradeToL1);
  EXPECT_TRUE(contract.appliedBias()->isApprox(Eigen::Vector3d::Zero()));

  EXPECT_EQ(contract.observe(observation(
                3'000'000'000, Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Constant(0.25), Eigen::Vector3d::Constant(0.1), true))
                .decision,
            FrameTransformDecision::kDegradeToL1);
}

TEST(FrameTransformContract, HeadingResetDegradesToVelocityOnly) {
  FrameTransformContract contract;
  ASSERT_EQ(contract.observe(observation(1'000'000'000, Eigen::Vector3d::Zero())).decision,
            FrameTransformDecision::kApplied);

  const auto result = contract.observe(observation(
      2'000'000'000, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.25),
      Eigen::Vector3d::Constant(0.1), true, 1U));
  EXPECT_EQ(result.decision, FrameTransformDecision::kDegradeToL1);
}

}  // namespace
}  // namespace px4_navigation_external_mode
