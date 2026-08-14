#include <gtest/gtest.h>

#include "navigation_mapping/generation_tracker.hpp"

namespace navigation_mapping {
namespace {

TEST(GenerationTrackerTest, FirstObservationIsAlwaysAdopted) {
  GenerationTracker tracker;
  EXPECT_FALSE(tracker.hasGeneration());
  EXPECT_EQ(tracker.decide(1), GenerationDecision::kResetAndAdoptNewGeneration);
}

TEST(GenerationTrackerTest, SameGenerationIsAcceptedNormally) {
  GenerationTracker tracker;
  tracker.adopt(1);
  EXPECT_EQ(tracker.decide(1), GenerationDecision::kAcceptCurrentGeneration);
}

TEST(GenerationTrackerTest, HigherGenerationTriggersResetAndAdopt) {
  GenerationTracker tracker;
  tracker.adopt(1);
  EXPECT_EQ(tracker.decide(2), GenerationDecision::kResetAndAdoptNewGeneration);
  tracker.adopt(2);
  EXPECT_EQ(tracker.currentGeneration(), 2U);
  EXPECT_EQ(tracker.resetCount(), 2U);  // initial adopt(1) + adopt(2)
}

TEST(GenerationTrackerTest, LowerGenerationIsRejectedAsStale) {
  GenerationTracker tracker;
  tracker.adopt(5);
  EXPECT_EQ(tracker.decide(4), GenerationDecision::kRejectStaleGeneration);
  EXPECT_EQ(tracker.currentGeneration(), 5U);  // unchanged
}

TEST(GenerationTrackerTest, StaleObservationNeverMutatesCurrentGeneration) {
  GenerationTracker tracker;
  tracker.adopt(10);
  for (std::uint64_t stale = 0; stale < 10; ++stale) {
    EXPECT_EQ(tracker.decide(stale), GenerationDecision::kRejectStaleGeneration);
    tracker.recordStaleRejection();
  }
  EXPECT_EQ(tracker.currentGeneration(), 10U);
  EXPECT_EQ(tracker.staleRejectedCount(), 10U);
}

}  // namespace
}  // namespace navigation_mapping
