#include <gtest/gtest.h>

#include <limits>

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

TEST(LioPublicFrameGenerationTest, FailsClosedBeforeGenerationWrap) {
  LioPublicFrameGeneration generation(std::numeric_limits<std::uint64_t>::max());
  generation.observe(PublicFrameEvent::kPublicFrameDiscontinuity, 1,
                     "PUBLIC_RESTART");
  const auto snapshot = generation.snapshot();
  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(snapshot.generation, std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(snapshot.last_event, "PUBLIC_FRAME_GENERATION_EXHAUSTED");
}

}  // namespace
}  // namespace uav::nav::lio
