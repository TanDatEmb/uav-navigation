#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "navigation_runtime/planning_supervisor.hpp"
#include "navigation_runtime/planning_worker.hpp"
#include "navigation_runtime/heading_rebind_worker.hpp"

namespace navigation_runtime {
namespace {

using namespace std::chrono_literals;

struct FakePlanner {
};

PlanningKey makeKey(std::uint64_t request_id = 1U) {
  PlanningKey key;
  key.localization_epoch = 1U;
  key.goal_epoch = 2U;
  key.request_id = request_id;
  key.route_revision = 3U;
  key.committed_bundle_generation = 4U;
  key.pinned_world_generation = 5U;
  key.pinned_world_revision = 6U;
  key.start_mode = PlanningStartMode::kCommittedFutureState;
  key.anchor_stamp_ns = 7;
  key.dynamics_hash = 8U;
  return key;
}

class JobGate {
 public:
  void started() {
    std::lock_guard lock(mutex_);
    started_ = true;
    cv_.notify_all();
  }
  bool waitUntilStarted() {
    std::unique_lock lock(mutex_);
    return cv_.wait_for(lock, 2s, [this] { return started_; });
  }
  void release() {
    std::lock_guard lock(mutex_);
    released_ = true;
    cv_.notify_all();
  }
  void waitUntilReleased(std::stop_token stop) {
    std::unique_lock lock(mutex_);
    cv_.wait(lock, stop, [this] { return released_; });
  }
  void waitUntilReleasedIgnoringStop() {
    std::unique_lock lock(mutex_);
    cv_.wait(lock, [this] { return released_; });
  }
 private:
  std::mutex mutex_;
  std::condition_variable_any cv_;
  bool started_{false};
  bool released_{false};
};

TEST(PlanningSupervisor, MapRevisionAndAnchorDoNotSupersedeActiveSolve) {
  const auto active = makeKey();
  auto newer = active;
  ++newer.pinned_world_revision;
  ++newer.anchor_stamp_ns;
  EXPECT_TRUE(samePlanningCancellationIdentity(active, newer));
  EXPECT_TRUE(PlanningSupervisor::resultStillCurrent(active, newer));
}

TEST(PlanningSupervisor, OwnershipIdentityChangesInvalidateResult) {
  const auto request = makeKey();
  auto current = request;
  ++current.route_revision;
  EXPECT_FALSE(PlanningSupervisor::resultStillCurrent(request, current));
  current = request;
  ++current.committed_bundle_generation;
  EXPECT_FALSE(PlanningSupervisor::resultStillCurrent(request, current));
  current = request;
  ++current.pinned_world_generation;
  EXPECT_FALSE(PlanningSupervisor::resultStillCurrent(request, current));
}

TEST(PlanningWorker, RunsOnlyOneJobAndKeepsLatestEqualPriorityPending) {
  auto planner = std::make_unique<FakePlanner>();
  PlanningWorker<FakePlanner> worker(std::move(planner));
  worker.start();
  JobGate gate;
  std::atomic_int first_running{0};
  std::atomic_int maximum_running{0};
  std::atomic_int completed_marker{0};
  ASSERT_EQ(worker.submit(
      makeKey(1U), PlanningPriority::kNormalRenewal,
      [&](FakePlanner&, std::stop_token stop) {
        const int running = ++first_running;
        maximum_running.store(std::max(maximum_running.load(), running));
        gate.started();
        gate.waitUntilReleased(stop);
        --first_running;
      }), PlanningSubmitDisposition::kAccepted);
  ASSERT_TRUE(gate.waitUntilStarted());

  auto second = makeKey(1U);
  ++second.anchor_stamp_ns;
  EXPECT_EQ(worker.submit(second, PlanningPriority::kNormalRenewal,
                          [&](FakePlanner&, std::stop_token) { completed_marker = 2; }),
            PlanningSubmitDisposition::kAccepted);
  auto third = second;
  ++third.anchor_stamp_ns;
  EXPECT_EQ(worker.submit(third, PlanningPriority::kNormalRenewal,
                          [&](FakePlanner&, std::stop_token) { completed_marker = 3; }),
            PlanningSubmitDisposition::kReplacedPending);
  gate.release();
  for (int attempt = 0; attempt < 200 && completed_marker.load() != 3; ++attempt) {
    std::this_thread::sleep_for(5ms);
  }
  worker.shutdown();
  EXPECT_EQ(completed_marker.load(), 3);
  EXPECT_EQ(maximum_running.load(), 1);
  EXPECT_EQ(worker.snapshot().replaced_pending, 1U);
  EXPECT_GE(worker.snapshot().last_enqueue_wait_us, 0);
  EXPECT_GE(worker.snapshot().last_worker_runtime_us, 0);
}

TEST(PlanningWorker, DropsExactDuplicateWithoutCancellingActiveSolve) {
  PlanningWorker<FakePlanner> worker(std::make_unique<FakePlanner>());
  worker.start();
  JobGate gate;
  const auto key = makeKey();
  ASSERT_EQ(worker.submit(key, PlanningPriority::kNormalRenewal,
                          [&](FakePlanner&, std::stop_token stop) {
                            gate.started();
                            gate.waitUntilReleased(stop);
                          }),
            PlanningSubmitDisposition::kAccepted);
  ASSERT_TRUE(gate.waitUntilStarted());
  EXPECT_EQ(worker.submit(key, PlanningPriority::kNormalRenewal,
                          [](FakePlanner&, std::stop_token) {}),
            PlanningSubmitDisposition::kExactDuplicate);
  gate.release();
  worker.shutdown();
  EXPECT_EQ(worker.snapshot().exact_duplicates, 1U);
}

TEST(PlanningWorker, AllowsSameLogicalKeyRetryAfterPriorJobCompletes) {
  auto planner = std::make_unique<FakePlanner>();
  PlanningWorker<FakePlanner> worker(std::move(planner));
  worker.start();
  std::atomic_int running{0};
  std::atomic_int maximum_running{0};
  std::atomic_int completed_jobs{0};
  const auto key = makeKey();
  const auto job = [&](FakePlanner&, std::stop_token) {
    const int current = ++running;
    maximum_running.store(std::max(maximum_running.load(), current));
    ++completed_jobs;
    --running;
  };

  ASSERT_EQ(worker.submit(key, PlanningPriority::kNormalRenewal, job),
            PlanningSubmitDisposition::kAccepted);
  for (int attempt = 0; attempt < 200 && worker.snapshot().completed < 1U; ++attempt) {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(worker.snapshot().completed, 1U);

  // A retry after the prior job has left active_ must be accepted even though
  // it carries the same logical PlanningKey. While active, the same key is
  // still rejected as an exact duplicate by the production worker contract.
  EXPECT_EQ(worker.submit(key, PlanningPriority::kNormalRenewal, job),
            PlanningSubmitDisposition::kAccepted);
  for (int attempt = 0; attempt < 200 && worker.snapshot().completed < 2U; ++attempt) {
    std::this_thread::sleep_for(5ms);
  }
  worker.shutdown();
  EXPECT_EQ(completed_jobs.load(), 2);
  EXPECT_EQ(maximum_running.load(), 1);
  EXPECT_EQ(worker.snapshot().completed, 2U);
}

TEST(PlanningWorker, MapRevisionQueuesWithoutCancellingActiveSolve) {
  PlanningWorker<FakePlanner> worker(std::make_unique<FakePlanner>());
  worker.start();
  JobGate gate;
  const auto key = makeKey();
  ASSERT_EQ(worker.submit(key, PlanningPriority::kNormalRenewal,
                          [&](FakePlanner&, std::stop_token stop) {
                            gate.started();
                            gate.waitUntilReleased(stop);
                          }),
            PlanningSubmitDisposition::kAccepted);
  ASSERT_TRUE(gate.waitUntilStarted());
  auto newer_map = key;
  ++newer_map.pinned_world_revision;
  EXPECT_EQ(worker.submit(newer_map, PlanningPriority::kNormalRenewal,
                          [](FakePlanner&, std::stop_token) {}),
            PlanningSubmitDisposition::kAccepted);
  gate.release();
  worker.shutdown();
}

TEST(HeadingRebindWorker, RunsBeforeBlockedPositionSolveAndStaleSolveCannotRollback) {
  auto planner = std::make_unique<FakePlanner>();
  PlanningWorker<FakePlanner> position_worker(std::move(planner));
  position_worker.start();
  JobGate position_gate;
  std::atomic_uint64_t execution_generation{1U};
  ASSERT_EQ(position_worker.submit(
                makeKey(1U), PlanningPriority::kGoalTransition,
                [&](FakePlanner&, std::stop_token stop) {
                  position_gate.started();
                  position_gate.waitUntilReleased(stop);
                  // This models a stale nominal completion. It may not
                  // overwrite the out-of-band execution owner installed by
                  // the command boundary while this solve was blocked.
                  std::uint64_t expected = 1U;
                  execution_generation.compare_exchange_strong(expected, 3U);
                }),
            PlanningSubmitDisposition::kAccepted);
  ASSERT_TRUE(position_gate.waitUntilStarted());

  HeadingRebindWorker heading_worker;
  heading_worker.start();
  std::atomic_bool heading_finished{false};
  ASSERT_TRUE(heading_worker.submit([&](std::stop_token stop) {
    if (stop.stop_requested()) return;
    execution_generation.store(2U);
    heading_finished.store(true);
  }));
  for (int attempt = 0; attempt < 200 && !heading_finished.load(); ++attempt) {
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_TRUE(heading_finished.load());
  EXPECT_EQ(execution_generation.load(), 2U);

  position_gate.release();
  position_worker.shutdown();
  heading_worker.shutdown();
  EXPECT_EQ(execution_generation.load(), 2U);
  EXPECT_GE(heading_worker.snapshot().completed, 1U);
}

TEST(PlanningWorker, GoalIdentityChangeCancelsInflightAndKeepsReplacement) {
  PlanningWorker<FakePlanner> worker(std::make_unique<FakePlanner>());
  worker.start();
  JobGate gate;
  std::atomic_bool replacement_ran{false};
  ASSERT_EQ(worker.submit(makeKey(1U), PlanningPriority::kNormalRenewal,
                          [&](FakePlanner&, std::stop_token stop) {
                            gate.started();
                            gate.waitUntilReleased(stop);
                          }),
            PlanningSubmitDisposition::kAccepted);
  ASSERT_TRUE(gate.waitUntilStarted());
  EXPECT_EQ(worker.submit(makeKey(2U), PlanningPriority::kGoalTransition,
                          [&](FakePlanner&, std::stop_token) { replacement_ran = true; }),
            PlanningSubmitDisposition::kAccepted);
  for (int attempt = 0; attempt < 200 && !replacement_ran.load(); ++attempt) {
    std::this_thread::sleep_for(5ms);
  }
  worker.shutdown();
  EXPECT_TRUE(replacement_ran.load());
  EXPECT_GE(worker.snapshot().cancelled, 1U);
}

TEST(PlanningWorker, ReplacementWaitsForCancelledJobToDrainWithoutGlobalInterrupt) {
  PlanningWorker<FakePlanner> worker(std::make_unique<FakePlanner>());
  worker.start();

  JobGate gate;
  std::atomic_bool first_finished{false};
  std::atomic_bool replacement_started{false};

  ASSERT_EQ(worker.submit(
      makeKey(1U), PlanningPriority::kNormalRenewal,
      [&](FakePlanner&, std::stop_token stop) {
        gate.started();
        // Model an uninterruptible native operation. Cancellation is recorded
        // on this job's token, but the worker must drain it before starting B.
        gate.waitUntilReleasedIgnoringStop();
        EXPECT_TRUE(stop.stop_requested());
        first_finished.store(true, std::memory_order_release);
      }),
      PlanningSubmitDisposition::kAccepted);
  ASSERT_TRUE(gate.waitUntilStarted());

  ASSERT_EQ(worker.submit(makeKey(2U), PlanningPriority::kGoalTransition,
                          [&](FakePlanner&, std::stop_token) {
                            replacement_started.store(true, std::memory_order_release);
                          }), PlanningSubmitDisposition::kAccepted);
  EXPECT_FALSE(first_finished.load(std::memory_order_acquire));
  EXPECT_FALSE(replacement_started.load(std::memory_order_acquire));
  EXPECT_EQ(worker.snapshot().cancelled, 1U);
  gate.release();
  for (int attempt = 0; attempt < 200 &&
       !replacement_started.load(std::memory_order_acquire); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  worker.shutdown();
  EXPECT_TRUE(first_finished.load(std::memory_order_acquire));
  EXPECT_TRUE(replacement_started.load(std::memory_order_acquire));
}

TEST(PlanningWorker, TerminalCancellationIsScopedToExecutionIdentity) {
  PlanningWorker<FakePlanner> worker(std::make_unique<FakePlanner>());
  worker.start();
  JobGate gate;
  std::atomic_bool old_finished{false};
  std::atomic_bool replacement_started{false};
  std::atomic_bool cancel_result{false};
  std::atomic_bool observed_stop{false};
  const auto old_key = makeKey(1U);
  ASSERT_EQ(worker.submit(old_key, PlanningPriority::kNormalRenewal,
                          [&](FakePlanner&, std::stop_token stop) {
                            gate.started();
                            gate.waitUntilReleased(stop);
                            observed_stop.store(stop.stop_requested(),
                                                std::memory_order_release);
                            old_finished.store(true, std::memory_order_release);
                          }),
            PlanningSubmitDisposition::kAccepted);
  ASSERT_TRUE(gate.waitUntilStarted());

  EXPECT_FALSE(worker.cancelActiveIfExecutionIdentity(
      old_key.localization_epoch, old_key.goal_epoch + 1U, old_key.request_id,
      old_key.committed_bundle_generation));
  EXPECT_FALSE(observed_stop.load(std::memory_order_acquire));
  cancel_result.store(worker.cancelActiveIfExecutionIdentity(
      old_key.localization_epoch, old_key.goal_epoch, old_key.request_id,
      old_key.committed_bundle_generation), std::memory_order_release);
  EXPECT_TRUE(cancel_result.load(std::memory_order_acquire));
  ASSERT_TRUE(gate.waitUntilStarted());
  gate.release();
  ASSERT_TRUE(worker.submit(makeKey(2U), PlanningPriority::kGoalTransition,
                            [&](FakePlanner&, std::stop_token) {
                              replacement_started.store(true, std::memory_order_release);
                            }) == PlanningSubmitDisposition::kAccepted);
  for (int attempt = 0; attempt < 200 &&
       !replacement_started.load(std::memory_order_acquire); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  worker.shutdown();
  EXPECT_TRUE(cancel_result.load(std::memory_order_acquire));
  EXPECT_TRUE(replacement_started.load(std::memory_order_acquire));
  EXPECT_TRUE(observed_stop.load(std::memory_order_acquire));
}

TEST(PlanningWorker, RejectsLowerPriorityWhileHigherPriorityIsInflight) {
  auto planner = std::make_unique<FakePlanner>();
  PlanningWorker<FakePlanner> worker(std::move(planner));
  worker.start();
  JobGate gate;
  ASSERT_EQ(worker.submit(makeKey(), PlanningPriority::kEmergency,
                          [&](FakePlanner&, std::stop_token stop) {
                            gate.started();
                            gate.waitUntilReleased(stop);
                          }),
            PlanningSubmitDisposition::kAccepted);
  ASSERT_TRUE(gate.waitUntilStarted());
  auto newer_anchor = makeKey();
  ++newer_anchor.anchor_stamp_ns;
  EXPECT_EQ(worker.submit(newer_anchor, PlanningPriority::kNormalRenewal,
                          [](FakePlanner&, std::stop_token) {}),
            PlanningSubmitDisposition::kRejectedLowerPriority);
  gate.release();
  worker.shutdown();
  EXPECT_EQ(worker.snapshot().rejected_lower_priority, 1U);
}

TEST(PlanningWorker, ConcurrentSubmittersKeepBoundedOwnership) {
  auto planner = std::make_unique<FakePlanner>();
  PlanningWorker<FakePlanner> worker(std::move(planner));
  worker.start();
  constexpr int kSubmitterCount = 4;
  constexpr int kSubmitsPerThread = 32;
  std::vector<std::thread> submitters;
  submitters.reserve(kSubmitterCount);
  for (int thread_index = 0; thread_index < kSubmitterCount; ++thread_index) {
    submitters.emplace_back([&worker, thread_index] {
      for (int index = 0; index < kSubmitsPerThread; ++index) {
        const auto request_id = static_cast<std::uint64_t>(
            100 + thread_index * kSubmitsPerThread + index);
        (void)worker.submit(
            makeKey(request_id), PlanningPriority::kNormalRenewal,
            [](FakePlanner&, std::stop_token) {});
      }
    });
  }
  for (auto& submitter : submitters) submitter.join();
  worker.shutdown();
  const auto snapshot = worker.snapshot();
  EXPECT_FALSE(snapshot.fatal);
  EXPECT_FALSE(snapshot.in_flight);
  EXPECT_FALSE(snapshot.pending);
  EXPECT_GT(snapshot.started, 0U);
}

}  // namespace
}  // namespace navigation_runtime
