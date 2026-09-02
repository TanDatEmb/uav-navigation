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

TEST(ExecutionEpisode, StoppedHoldPreservesMeasuredRestartRequest) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(1U, 2U, 3U, true);
  episode.requestRestartFromRest();

  episode.stoppedHold(4U);
  const auto state = episode.snapshot();
  EXPECT_EQ(state.phase, navigation_runtime::ExecutionEpisodePhase::kStoppedHold);
  EXPECT_TRUE(state.command_available);
  EXPECT_TRUE(state.restart_from_rest);
}

TEST(ExecutionEpisode, SuspendAndClearDoNotRetainCommandIdentity) {
  navigation_runtime::ExecutionEpisode episode;
  episode.beginGoal(8U, 9U, 10U, true);
  episode.commandCommitted(bundle(
      navigation_planning::CandidateBundleKind::kMainWithBackup, 31U));

  episode.suspendCommand();
  auto suspended = episode.snapshot();
  EXPECT_FALSE(suspended.command_available);
  EXPECT_EQ(suspended.active_generation, 31U);
  EXPECT_EQ(suspended.goal_epoch, 9U);

  episode.clearGoal(12U);
  const auto cleared = episode.snapshot();
  EXPECT_EQ(cleared.localization_epoch, 12U);
  EXPECT_EQ(cleared.goal_epoch, 0U);
  EXPECT_EQ(cleared.request_id, 0U);
  EXPECT_EQ(cleared.active_generation, 0U);
  EXPECT_FALSE(cleared.command_available);
  EXPECT_FALSE(cleared.failure_latched);
  EXPECT_EQ(cleared.phase,
            navigation_runtime::ExecutionEpisodePhase::kInitialHold);
}

}  // namespace
