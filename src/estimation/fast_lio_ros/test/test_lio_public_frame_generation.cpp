#include <gtest/gtest.h>

#include "fast_lio_ros/lio_public_frame_generation.hpp"

namespace uav::nav::lio {
namespace {

TEST(LioPublicFrameGenerationTest, IgnoresNonPublicGenerationEvents) {
  LioPublicFrameGeneration generation;
  generation.observe(PublicFrameEvent::kInternalLioGenerationChange);
  generation.observe(PublicFrameEvent::kCorrectedPropagatedHandoff);
  generation.observe(PublicFrameEvent::kPx4Reset);
  generation.observe(PublicFrameEvent::kPublicFrameDiscontinuity);
  EXPECT_EQ(generation.snapshot().generation, 1U);
  EXPECT_EQ(generation.snapshot().discontinuity_count, 0U);
}

TEST(LioPublicFrameGenerationTest, CountsOneGenerationPerPublicDiscontinuity) {
  LioPublicFrameGeneration generation;
  generation.observe(PublicFrameEvent::kPublicFrameDiscontinuity, 17,
                     "PUBLIC_REANCHOR");
  generation.observe(PublicFrameEvent::kPublicFrameDiscontinuity, 17,
                     "DUPLICATE_EVENT");
  generation.observe(PublicFrameEvent::kPublicFrameDiscontinuity, 18,
                     "PUBLIC_RESTART");
  EXPECT_EQ(generation.snapshot().generation, 3U);
  EXPECT_EQ(generation.snapshot().discontinuity_count, 2U);
  EXPECT_EQ(generation.snapshot().last_event, "PUBLIC_RESTART");
}

}  // namespace
}  // namespace uav::nav::lio
