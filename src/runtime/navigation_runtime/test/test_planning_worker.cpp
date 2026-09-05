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

namespace navigation_runtime {
namespace {

using namespace std::chrono_literals;

struct FakePlanner {
  void cancelActiveSolve() noexcept {
    ++cancel_calls;
    if (cancel_hook) cancel_hook();
  }
  std::atomic_uint64_t cancel_calls{0};
  std::function<void()> cancel_hook;
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
}

TEST(PlanningWorker, DropsExactDuplicateWithoutCancellingActiveSolve) {
  auto planner = std::make_unique<FakePlanner>();
  auto* planner_view = planner.get();
  PlanningWorker<FakePlanner> worker(std::move(planner));
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
  EXPECT_EQ(planner_view->cancel_calls.load(), 0U);
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
  auto planner = std::make_unique<FakePlanner>();
  auto* planner_view = planner.get();
  PlanningWorker<FakePlanner> worker(std::move(planner));
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
  EXPECT_EQ(planner_view->cancel_calls.load(), 0U);
  gate.release();
  worker.shutdown();
}

TEST(PlanningWorker, GoalIdentityChangeCancelsInflightAndKeepsReplacement) {
  auto planner = std::make_unique<FakePlanner>();
  auto* planner_view = planner.get();
  PlanningWorker<FakePlanner> worker(std::move(planner));
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
  EXPECT_GE(planner_view->cancel_calls.load(), 1U);
  EXPECT_GE(worker.snapshot().cancelled, 1U);
}

TEST(PlanningWorker, TerminalCancelKeepsWorkerIdentityThroughBackendInterrupt) {
  auto planner = std::make_unique<FakePlanner>();
  auto* planner_view = planner.get();
  PlanningWorker<FakePlanner> worker(std::move(planner));
  worker.start();
  JobGate gate;
  std::atomic_bool old_finished{false};
  std::atomic_bool cancel_entered{false};
  std::atomic_bool release_cancel{false};
  std::atomic_bool replacement_started{false};
  std::atomic_bool cancel_result{false};
  planner_view->cancel_hook = [&] {
    cancel_entered.store(true, std::memory_order_release);
    while (!old_finished.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    while (!release_cancel.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  };
  const auto old_key = makeKey(1U);
  ASSERT_EQ(worker.submit(old_key, PlanningPriority::kNormalRenewal,
                          [&](FakePlanner&, std::stop_token stop) {
                            gate.started();
                            gate.waitUntilReleased(stop);
                            old_finished.store(true, std::memory_order_release);
                          }),
            PlanningSubmitDisposition::kAccepted);
  ASSERT_TRUE(gate.waitUntilStarted());

  std::thread terminal_cancel([&] {
    cancel_result.store(worker.cancelActiveIfExecutionIdentity(
        old_key.localization_epoch, old_key.goal_epoch, old_key.request_id,
        old_key.committed_bundle_generation), std::memory_order_release);
  });
  for (int attempt = 0; attempt < 200 &&
       !cancel_entered.load(std::memory_order_acquire); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  if (!cancel_entered.load(std::memory_order_acquire)) {
    release_cancel.store(true, std::memory_order_release);
    gate.release();
    terminal_cancel.join();
    worker.shutdown();
    ADD_FAILURE() << "identity-scoped terminal cancellation did not enter backend";
    return;
  }
  for (int attempt = 0; attempt < 200 &&
       !old_finished.load(std::memory_order_acquire); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  if (!old_finished.load(std::memory_order_acquire)) {
    release_cancel.store(true, std::memory_order_release);
    gate.release();
    terminal_cancel.join();
    worker.shutdown();
    ADD_FAILURE() << "old worker job did not finish after stop request";
    return;
  }

  std::thread replacement_submit([&] {
    (void)worker.submit(makeKey(2U), PlanningPriority::kGoalTransition,
                        [&](FakePlanner&, std::stop_token) {
                          replacement_started.store(true, std::memory_order_release);
                        });
  });
  std::this_thread::sleep_for(10ms);
  EXPECT_FALSE(replacement_started.load(std::memory_order_acquire));
  release_cancel.store(true, std::memory_order_release);
  terminal_cancel.join();
  replacement_submit.join();
  for (int attempt = 0; attempt < 200 &&
       !replacement_started.load(std::memory_order_acquire); ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  worker.shutdown();
  EXPECT_TRUE(cancel_result.load(std::memory_order_acquire));
  EXPECT_TRUE(replacement_started.load(std::memory_order_acquire));
  EXPECT_GE(planner_view->cancel_calls.load(), 1U);
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
