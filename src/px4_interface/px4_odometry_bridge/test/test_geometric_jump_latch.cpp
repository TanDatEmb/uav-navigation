#include <gtest/gtest.h>

#include "px4_odometry_bridge/geometric_jump_latch.hpp"

namespace px4_odometry_bridge {
namespace {

TEST(GeometricJumpLatchTest, LatchesOnceAndRepeatedJumpsDoNotRecount) {
  GeometricJumpLatch latch;
  EXPECT_FALSE(latch.observeGeometricJump(false));
  EXPECT_TRUE(latch.observeGeometricJump(true));
  EXPECT_FALSE(latch.observeGeometricJump(true));
  EXPECT_EQ(latch.count(), 1U);
  EXPECT_TRUE(latch.latched());
}

TEST(GeometricJumpLatchTest, GateAndPx4EventsCannotClearLatch) {
  GeometricJumpLatch latch;
  latch.observePublicFrameGeneration(true, 1);
  latch.observeGeometricJump(true);
  // There is deliberately no reset path for gate close/reopen or PX4 frame
  // generation changes in this producer-owned latch.
  EXPECT_TRUE(latch.latched());
  EXPECT_EQ(latch.count(), 1U);
}

TEST(GeometricJumpLatchTest, NewPublicGenerationOrOperatorEventRecovers) {
  GeometricJumpLatch latch;
  latch.observePublicFrameGeneration(true, 1);
  latch.observeGeometricJump(true);
  EXPECT_TRUE(latch.observePublicFrameGeneration(true, 2));
  EXPECT_FALSE(latch.latched());
  latch.observeGeometricJump(true);
  EXPECT_TRUE(latch.observeOperatorReset(true, 3));
  EXPECT_FALSE(latch.latched());
}

TEST(GeometricJumpLatchTest, ResetCounterUsesExplicitUint8Modulo) {
  EXPECT_EQ(public_frame_generation_to_reset_counter(1), 1U);
  EXPECT_EQ(public_frame_generation_to_reset_counter(255), 255U);
  EXPECT_EQ(public_frame_generation_to_reset_counter(256), 0U);
  EXPECT_EQ(public_frame_generation_to_reset_counter(257), 1U);
}

}  // namespace
}  // namespace px4_odometry_bridge
