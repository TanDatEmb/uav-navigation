#include <gtest/gtest.h>

#include <navigation_runtime/execution_episode.hpp>

namespace {

navigation_planning::CandidateBundle bundle(
    navigation_planning::CandidateBundleKind kind,
    std::uint64_t generation) {
  navigation_planning::CandidateBundle value;
  value.kind = kind;
  value.bundle_generation = generation;
  return value;
}

TEST(ExecutionEpisode, KeepsOneAuthoritativeLifecycleSnapshot) {
  navigation_runtime::ExecutionEpisode episode;
  episode.reset(4U);
  episode.beginGoal(4U, 7U, 11U, false);
  auto initial = episode.snapshot();
  EXPECT_EQ(initial.localization_epoch, 4U);
  EXPECT_EQ(initial.goal_epoch, 7U);
  EXPECT_EQ(initial.request_id, 11U);
  EXPECT_EQ(initial.phase, navigation_runtime::ExecutionEpisodePhase::kInitialHold);
  EXPECT_FALSE(initial.command_available);

  episode.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 20U));
  auto tracking = episode.snapshot();
  EXPECT_EQ(tracking.phase, navigation_runtime::ExecutionEpisodePhase::kTrackingMain);
  EXPECT_TRUE(tracking.command_available);
  EXPECT_EQ(tracking.active_generation, 20U);

  episode.roleObserved(navigation_planning::CandidateRole::kBackup, 20U);
  EXPECT_EQ(episode.snapshot().phase,
            navigation_runtime::ExecutionEpisodePhase::kTrackingBackup);
  episode.stoppedHold(20U);
  EXPECT_EQ(episode.snapshot().phase,
            navigation_runtime::ExecutionEpisodePhase::kStoppedHold);
}

TEST(ExecutionEpisode, FailClosedClearsCommandExposure) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, true);
  episode.failClosed();
  const auto state = episode.snapshot();
  EXPECT_EQ(state.phase, navigation_runtime::ExecutionEpisodePhase::kPx4Hold);
  EXPECT_FALSE(state.command_available);
  EXPECT_TRUE(state.failure_latched);
}

}  // namespace
