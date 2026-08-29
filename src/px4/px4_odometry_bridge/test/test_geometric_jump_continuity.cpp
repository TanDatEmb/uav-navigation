#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "px4_odometry_bridge/geometric_jump_continuity.hpp"
#include "px4_odometry_bridge/geometric_jump_latch.hpp"

namespace px4_odometry_bridge {
namespace {

ExternalOdometryFrame frame(std::int64_t timestamp_ns, double x_m,
                            double yaw_rad = 0.0) {
  ExternalOdometryFrame sample;
  sample.timestamp_ns = timestamp_ns;
  sample.position_ned = Eigen::Vector3d(x_m, 0.0, 0.0);
  sample.orientation_ned = Eigen::Quaterniond(
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
  sample.frame_valid = true;
  return sample;
}

TEST(GeometricJumpContinuityTest, HighSpeedValidInputDoesNotLatchAtTenMetersPerSecond) {
  GeometricJumpContinuityConfig config;
  GeometricJumpContinuityState state;
  GeometricJumpLatch latch;

  const auto first = observe_geometric_jump_continuity(
      frame(1'000'000'000LL, 0.0), true, true, 1U, config, state);
  EXPECT_FALSE(first.evaluated);

  const auto second = observe_geometric_jump_continuity(
      frame(1'100'000'000LL, 1.0), true, true, 1U, config, state);
  EXPECT_TRUE(second.evaluated);
  EXPECT_FALSE(second.jumped);
  EXPECT_FALSE(latch.observeGeometricJump(second.jumped));
  EXPECT_FALSE(latch.latched());
}

TEST(GeometricJumpContinuityTest, PublicationGapDoesNotBreakSourceContinuityBaseline) {
  GeometricJumpContinuityConfig config;
  GeometricJumpContinuityState state;
  GeometricJumpLatch latch;

  (void)observe_geometric_jump_continuity(
      frame(1'000'000'000LL, 0.0), true, true, 1U, config, state);
  const auto a = observe_geometric_jump_continuity(
      frame(1'100'000'000LL, 1.0), true, true, 1U, config, state);
  const auto b = observe_geometric_jump_continuity(
      frame(1'200'000'000LL, 2.0), true, true, 1U, config, state);
  const auto c = observe_geometric_jump_continuity(
      frame(1'300'000'000LL, 3.0), true, true, 1U, config, state);

  EXPECT_TRUE(a.evaluated);
  EXPECT_TRUE(b.evaluated);
  EXPECT_TRUE(c.evaluated);
  EXPECT_FALSE(a.jumped);
  EXPECT_FALSE(b.jumped);
  EXPECT_FALSE(c.jumped);
  EXPECT_FALSE(latch.observeGeometricJump(a.jumped));
  EXPECT_FALSE(latch.observeGeometricJump(b.jumped));
  EXPECT_FALSE(latch.observeGeometricJump(c.jumped));
  EXPECT_FALSE(latch.latched());
}

TEST(GeometricJumpContinuityTest, RealDiscontinuityLatchesForSameGeneration) {
  GeometricJumpContinuityConfig config;
  GeometricJumpContinuityState state;
  GeometricJumpLatch latch;

  (void)observe_geometric_jump_continuity(
      frame(1'000'000'000LL, 0.0), true, true, 7U, config, state);
  const auto jump = observe_geometric_jump_continuity(
      frame(1'020'000'000LL, 2.0), true, true, 7U, config, state);

  EXPECT_TRUE(jump.evaluated);
  EXPECT_TRUE(jump.jumped);
  EXPECT_TRUE(latch.observeGeometricJump(jump.jumped));
  EXPECT_TRUE(latch.latched());
}

TEST(GeometricJumpContinuityTest, GenerationChangeReseedsAndClearsLatch) {
  GeometricJumpContinuityConfig config;
  GeometricJumpContinuityState state;
  GeometricJumpLatch latch;

  latch.observePublicFrameGeneration(true, 1U);
  (void)observe_geometric_jump_continuity(
      frame(1'000'000'000LL, 0.0), true, true, 1U, config, state);
  const auto jump = observe_geometric_jump_continuity(
      frame(1'020'000'000LL, 2.0), true, true, 1U, config, state);
  ASSERT_TRUE(jump.jumped);
  ASSERT_TRUE(latch.observeGeometricJump(jump.jumped));
  ASSERT_TRUE(latch.latched());

  const auto reseed = observe_geometric_jump_continuity(
      frame(1'040'000'000LL, 2.1), true, true, 2U, config, state);
  EXPECT_FALSE(reseed.evaluated);
  EXPECT_FALSE(reseed.jumped);
  EXPECT_EQ(reseed.reason, GeometricJumpContinuityReason::kGenerationChanged);

  EXPECT_TRUE(latch.observePublicFrameGeneration(true, 2U));
  EXPECT_FALSE(latch.latched());

  const std::uint8_t before = public_frame_generation_to_reset_counter(1U);
  const std::uint8_t after = public_frame_generation_to_reset_counter(2U);
  EXPECT_EQ(after, static_cast<std::uint8_t>(before + 1U));
}

TEST(GeometricJumpContinuityTest, TrackingRecoveryWithContinuousFrameDoesNotRelatch) {
  GeometricJumpContinuityConfig config;
  GeometricJumpContinuityState state;
  GeometricJumpLatch latch;

  (void)observe_geometric_jump_continuity(
      frame(1'000'000'000LL, 0.0), true, true, 9U, config, state);
  const auto degraded = observe_geometric_jump_continuity(
      frame(1'100'000'000LL, 1.0), false, true, 9U, config, state);
  EXPECT_FALSE(degraded.evaluated);
  EXPECT_EQ(degraded.reason, GeometricJumpContinuityReason::kSourceInvalid);

  const auto recover_seed = observe_geometric_jump_continuity(
      frame(1'200'000'000LL, 2.0), true, true, 9U, config, state);
  EXPECT_FALSE(recover_seed.evaluated);
  EXPECT_EQ(recover_seed.reason, GeometricJumpContinuityReason::kNoBaseline);

  const auto stable = observe_geometric_jump_continuity(
      frame(1'300'000'000LL, 3.0), true, true, 9U, config, state);
  EXPECT_TRUE(stable.evaluated);
  EXPECT_FALSE(stable.jumped);
  EXPECT_FALSE(latch.observeGeometricJump(stable.jumped));
  EXPECT_FALSE(latch.latched());

  const std::uint8_t before = public_frame_generation_to_reset_counter(9U);
  const std::uint8_t after = public_frame_generation_to_reset_counter(9U);
  EXPECT_EQ(after, before);
}

TEST(GeometricJumpContinuityTest, RejectsInvalidConfigAndExtremeTimestampDelta) {
  GeometricJumpContinuityConfig config;
  GeometricJumpContinuityState state;
  auto first = frame(1, 0.0);
  auto second = frame(std::numeric_limits<std::int64_t>::max(), 0.0);
  (void)observe_geometric_jump_continuity(first, true, true, 1U, config, state);
  const auto result = observe_geometric_jump_continuity(
      second, true, true, 1U, config, state);
  EXPECT_FALSE(result.evaluated);
  EXPECT_EQ(result.reason, GeometricJumpContinuityReason::kDtTooLarge);

  config.maximum_expected_speed_mps = -1.0;
  const auto invalid = observe_geometric_jump_continuity(
      frame(2'000'000'000LL, 0.0), true, true, 1U, config, state);
  EXPECT_FALSE(invalid.evaluated);
  EXPECT_EQ(invalid.reason, GeometricJumpContinuityReason::kCurrentFrameInvalid);
}

}  // namespace
}  // namespace px4_odometry_bridge
