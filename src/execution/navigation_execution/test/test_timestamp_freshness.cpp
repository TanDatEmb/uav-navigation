#include "navigation_execution/timestamp_freshness.hpp"
#include "navigation_execution/execution_state_gate.hpp"
#include <navigation_contracts/execution_state_freshness.hpp>

#include <gtest/gtest.h>

#include <thread>
#include <vector>

namespace navigation_execution {

TEST(TimestampFreshness, DistinguishesPastFutureAndInvalidTime) {
  constexpr std::int64_t now = 10'000;
  constexpr std::int64_t limit = 500;
  EXPECT_EQ(classifyTimestampFreshness(now, 0, limit), TimestampFreshness::INVALID);
  EXPECT_EQ(classifyTimestampFreshness(now, now - limit - 1, limit),
            TimestampFreshness::STALE);
  EXPECT_EQ(classifyTimestampFreshness(now, now + limit + 1, limit),
            TimestampFreshness::FUTURE);
}

TEST(TimestampFreshness, BoundaryIsAcceptedWithoutAbsoluteTimeComparison) {
  constexpr std::int64_t now = 10'000;
  constexpr std::int64_t limit = 500;
  EXPECT_EQ(classifyTimestampFreshness(now, now - limit, limit),
            TimestampFreshness::VALID);
  EXPECT_EQ(classifyTimestampFreshness(now, now + limit, limit),
            TimestampFreshness::VALID);
}

TEST(ExecutionStateFreshness, RequiresBothRosSourceAndSteadyReceiptToBeFresh) {
  constexpr std::int64_t now_ros = 10'000'000'000LL;
  constexpr std::int64_t now_steady = 20'000'000'000LL;
  const auto valid = navigation_contracts::evaluateExecutionStateFreshness(
      now_ros, now_ros - 500'000'000LL, now_steady,
      now_steady - 500'000'000LL, 0.5);
  EXPECT_TRUE(valid.valid());
  EXPECT_DOUBLE_EQ(valid.source_age_ms, 500.0);
  EXPECT_DOUBLE_EQ(valid.receive_age_ms, 500.0);

  const auto frozen_ros = navigation_contracts::evaluateExecutionStateFreshness(
      now_ros, now_ros, now_steady, now_steady - 500'000'001LL, 0.5);
  EXPECT_EQ(frozen_ros.reason,
            navigation_contracts::ExecutionStateFreshnessReason::kReceiveStale);

  const auto stale_source = navigation_contracts::evaluateExecutionStateFreshness(
      now_ros, now_ros - 500'000'001LL, now_steady, now_steady, 0.5);
  EXPECT_EQ(stale_source.reason,
            navigation_contracts::ExecutionStateFreshnessReason::kSourceStale);
}

TEST(ExecutionStateFreshness, FailsClosedForMissingFutureAndInvalidInputs) {
  using Reason = navigation_contracts::ExecutionStateFreshnessReason;
  constexpr std::int64_t now_ros = 10'000'000'000LL;
  constexpr std::int64_t now_steady = 20'000'000'000LL;
  EXPECT_EQ(navigation_contracts::evaluateExecutionStateFreshness(
                now_ros, 0, now_steady, now_steady, 0.5).reason,
            Reason::kMissingSourceStamp);
  EXPECT_EQ(navigation_contracts::evaluateExecutionStateFreshness(
                now_ros, now_ros, now_steady, 0, 0.5).reason,
            Reason::kMissingReceiveStamp);
  EXPECT_EQ(navigation_contracts::evaluateExecutionStateFreshness(
                now_ros, now_ros + 500'000'001LL, now_steady, now_steady, 0.5).reason,
            Reason::kSourceFuture);
  EXPECT_EQ(navigation_contracts::evaluateExecutionStateFreshness(
                now_ros, now_ros, now_steady, now_steady + 1, 0.5).reason,
            Reason::kReceiveFuture);
  EXPECT_EQ(navigation_contracts::evaluateExecutionStateFreshness(
                now_ros, now_ros, now_steady, now_steady, 0.0).reason,
            Reason::kInvalidLimit);
}

TEST(ExecutionStateFailureLatch, TransitionsExactlyOnceUntilExplicitGoalReset) {
  ExecutionStateFailureLatch latch;
  std::atomic_int transitions{0};
  std::vector<std::thread> contenders;
  for (int i = 0; i < 16; ++i) {
    contenders.emplace_back([&] {
      if (latch.tryLatch()) ++transitions;
    });
  }
  for (auto& contender : contenders) contender.join();
  EXPECT_TRUE(latch.latched());
  EXPECT_FALSE(latch.allowsCommandExposure());
  EXPECT_EQ(transitions.load(), 1);
  EXPECT_FALSE(latch.tryLatch());

  latch.resetForNewGoal();
  EXPECT_FALSE(latch.latched());
  EXPECT_TRUE(latch.allowsCommandExposure());
  EXPECT_TRUE(latch.tryLatch());
}

TEST(ExecutionStateFailureLatch, SerializesSolveExposureWithStaleInvalidation) {
  ExecutionStateFailureLatch latch;
  bool command_available = false;

  {
    std::lock_guard<std::mutex> solve_transition(latch.transitionMutex());
    ASSERT_TRUE(latch.allowsCommandExposure());
    command_available = true;
  }
  {
    std::lock_guard<std::mutex> stale_transition(latch.transitionMutex());
    EXPECT_TRUE(latch.tryLatch());
    command_available = false;
  }
  EXPECT_FALSE(command_available);

  latch.resetForNewGoal();
  {
    std::lock_guard<std::mutex> stale_transition(latch.transitionMutex());
    ASSERT_TRUE(latch.tryLatch());
    command_available = false;
  }
  {
    std::lock_guard<std::mutex> solve_transition(latch.transitionMutex());
    if (latch.allowsCommandExposure()) command_available = true;
  }
  EXPECT_FALSE(command_available);
}

TEST(ExecutionStateFailureLatch, NewGoalResetAndOldCommandClearShareOneTransition) {
  ExecutionStateFailureLatch latch;
  bool command_available = true;
  ASSERT_TRUE(latch.tryLatch());

  {
    std::lock_guard<std::mutex> goal_transition(latch.transitionMutex());
    latch.resetForNewGoalWithinTransition();
    command_available = false;
  }
  {
    std::lock_guard<std::mutex> delayed_publish(latch.transitionMutex());
    EXPECT_TRUE(latch.allowsCommandExposure());
    EXPECT_FALSE(command_available);
  }
}

TEST(ExecutionStateFailureLatch, HotFailureAndWatchdogCannotExposeACommand) {
  ExecutionStateFailureLatch latch;
  bool command_available = true;
  bool planner_failed = false;

  {
    std::lock_guard<std::mutex> hot_failure(latch.transitionMutex());
    const bool usable_safety_suffix = false;
    planner_failed = !usable_safety_suffix;
    if (!usable_safety_suffix) command_available = false;
  }
  {
    std::lock_guard<std::mutex> publish_decision(latch.transitionMutex());
    EXPECT_FALSE(command_available);
    EXPECT_TRUE(planner_failed);
  }

  latch.resetForNewGoal();
  command_available = false;
  planner_failed = false;
  std::atomic_uint64_t timed_out_generation{42U};
  {
    std::lock_guard<std::mutex> solve_exposure(latch.transitionMutex());
    const std::uint64_t solve_generation = 42U;
    if (timed_out_generation.load() == solve_generation) {
      command_available = false;
      planner_failed = true;
    } else if (latch.allowsCommandExposure()) {
      command_available = true;
    }
  }
  EXPECT_FALSE(command_available);
  EXPECT_TRUE(planner_failed);
}

TEST(ExecutionStateFailureLatch, OldSolveCannotExposeAcrossGoalEpoch) {
  ExecutionStateFailureLatch latch;
  std::atomic_uint64_t goal_epoch{7U};
  std::atomic_uint64_t command_goal_epoch{7U};
  const std::uint64_t old_solve_goal_epoch = goal_epoch.load();
  bool command_available = false;

  {
    std::lock_guard<std::mutex> new_goal_transition(latch.transitionMutex());
    ++goal_epoch;
    latch.resetForNewGoalWithinTransition();
    command_available = false;
    command_goal_epoch.store(0U);
  }
  {
    std::lock_guard<std::mutex> old_solve_exposure(latch.transitionMutex());
    if (goal_epoch.load() == old_solve_goal_epoch &&
        latch.allowsCommandExposure()) {
      command_available = true;
    }
  }
  EXPECT_FALSE(command_available);
  EXPECT_NE(command_goal_epoch.load(), goal_epoch.load());

  // PASS_THROUGH may deliberately transfer the current bundle to the newer
  // goal. A late old solve must be discard-only and preserve that ownership.
  command_available = true;
  command_goal_epoch.store(goal_epoch.load());
  const auto transferred_epoch = command_goal_epoch.load();
  {
    std::lock_guard<std::mutex> old_solve_fallback(latch.transitionMutex());
    if (goal_epoch.load() == old_solve_goal_epoch) {
      command_available = false;
      command_goal_epoch.store(0U);
    }
  }
  EXPECT_TRUE(command_available);
  EXPECT_EQ(command_goal_epoch.load(), transferred_epoch);
}

}  // namespace navigation_execution
