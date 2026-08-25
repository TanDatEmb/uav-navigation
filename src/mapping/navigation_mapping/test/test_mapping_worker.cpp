#include "navigation_mapping/observation_accounting.hpp"
#include "navigation_mapping/mapping_worker.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>
#include <future>
#include <latch>
#include <mutex>
#include <vector>

namespace navigation_mapping {
namespace {

TEST(ObservationAccounting, AccountsPublishedObservationExactlyOnce) {
  ObservationAccounting accounting;
  EXPECT_FALSE(accounting.recordAcceptedToInbox());
  accounting.waitingSubmitted(false);
  accounting.mappingStarted();
  accounting.mappingPublished();

  const auto state = accounting.snapshot();
  EXPECT_EQ(state.received, 1U);
  EXPECT_EQ(state.mapping_published, 1U);
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(ObservationAccounting, AccountsReplacementAndPendingAtStop) {
  ObservationAccounting accounting;
  EXPECT_FALSE(accounting.recordAcceptedToInbox());
  EXPECT_TRUE(accounting.recordAcceptedToInbox());

  const auto state = accounting.snapshot();
  EXPECT_EQ(state.replaced_pending, 1U);
  EXPECT_EQ(state.replaced_waiting, 1U);
  EXPECT_EQ(state.pending, 1U);
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(MappingWorker, ProcessesOneReadyObservationExactlyOnce) {
  ObservationAccounting accounting;
  std::promise<int> processed;
  MappingWorker<int> worker(accounting, [&](int&& value) {
    processed.set_value(value);
  });
  worker.start();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(7));
  EXPECT_EQ(processed.get_future().get(), 7);
  worker.shutdown();
  const auto state = accounting.snapshot();
  EXPECT_EQ(state.mapping_published, 1U);
  EXPECT_EQ(state.ready, 0U);
  EXPECT_EQ(state.in_flight, 0U);
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(MappingWorker, EmitsOnePostTerminalNotificationPerPublishedObservation) {
  ObservationAccounting accounting;
  std::promise<void> published;
  std::atomic_uint32_t notifications{0};
  MappingWorker<int> worker(
      accounting, [](int&&) {}, {}, {}, [&] {
        ++notifications;
        EXPECT_EQ(accounting.snapshot().mapping_published, 1U);
        published.set_value();
      });
  worker.start();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(1));
  published.get_future().wait();
  worker.shutdown();
  EXPECT_EQ(notifications.load(), 1U);
  EXPECT_TRUE(accounting.snapshot().allInvariantsHold());
}

TEST(MappingWorker, NotificationFailureDoesNotRelabelPublishedMapAsFailed) {
  ObservationAccounting accounting;
  std::promise<void> fatal;
  MappingWorker<int> worker(
      accounting, [](int&&) {},
      [&](std::exception_ptr failure) {
        EXPECT_NE(failure, nullptr);
        fatal.set_value();
      }, {}, [] { throw std::runtime_error("diagnostics publish failed"); });
  worker.start();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(1));
  fatal.get_future().wait();
  worker.shutdown();
  const auto state = accounting.snapshot();
  EXPECT_EQ(state.mapping_published, 1U);
  EXPECT_EQ(state.mapping_failed, 0U);
  EXPECT_TRUE(worker.fatal());
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(MappingWorker, ReplacesOnlyReadyAndNeverInflight) {
  ObservationAccounting accounting;
  std::promise<void> first_started;
  std::promise<void> release_first;
  std::promise<void> third_processed;
  auto release = release_first.get_future().share();
  std::vector<int> processed;
  std::mutex processed_mutex;
  MappingWorker<int> worker(accounting, [&](int&& value) {
    if (value == 1) {
      first_started.set_value();
      release.wait();
    }
    std::lock_guard lock(processed_mutex);
    processed.push_back(value);
    if (value == 3) third_processed.set_value();
  });
  worker.start();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(1));
  first_started.get_future().wait();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(2));
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(3));
  release_first.set_value();
  third_processed.get_future().wait();
  worker.shutdown();
  const auto state = accounting.snapshot();
  EXPECT_EQ(state.replaced_ready, 1U);
  EXPECT_EQ(state.mapping_published, 2U);
  EXPECT_TRUE(state.allInvariantsHold());
  std::lock_guard lock(processed_mutex);
  ASSERT_EQ(processed.size(), 2U);
  EXPECT_EQ(processed[0], 1);
  EXPECT_EQ(processed[1], 3);
}

TEST(MappingWorker, ResetDiscardsReadyAndRestartsOrderAfterInflightCompletes) {
  using namespace std::chrono_literals;
  ObservationAccounting accounting;
  std::promise<void> first_started;
  std::promise<void> release_first;
  std::promise<void> reset_finished;
  std::promise<void> new_epoch_processed;
  auto released = release_first.get_future().share();
  std::vector<int> processed;
  std::mutex processed_mutex;
  MappingWorker<int> worker(accounting, [&](int&& value) {
    if (value == 10) {
      first_started.set_value();
      released.wait();
    }
    {
      std::lock_guard lock(processed_mutex);
      processed.push_back(value);
    }
    if (value == 1) new_epoch_processed.set_value();
  });
  worker.setStrictlyIncreasingOrderKey(
      [](const int& value) { return static_cast<std::int64_t>(value); });
  worker.start();

  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(10));
  ASSERT_EQ(first_started.get_future().wait_for(2s), std::future_status::ready);
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(20));

  auto reset = std::async(std::launch::async, [&] {
    worker.reset();
    reset_finished.set_value();
  });
  while (!worker.resetting()) std::this_thread::yield();

  accounting.recordAcceptedToInbox();
  EXPECT_FALSE(worker.submitFromWaiting(30));
  release_first.set_value();
  ASSERT_EQ(reset_finished.get_future().wait_for(2s), std::future_status::ready);
  reset.get();

  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(1));
  ASSERT_EQ(new_epoch_processed.get_future().wait_for(2s), std::future_status::ready);
  worker.shutdown();

  const auto state = accounting.snapshot();
  EXPECT_EQ(state.mapping_published, 2U);
  EXPECT_EQ(state.discarded_ready, 1U);
  EXPECT_EQ(state.discarded_pending, 2U);
  EXPECT_TRUE(state.allInvariantsHold());
  std::lock_guard lock(processed_mutex);
  ASSERT_EQ(processed.size(), 2U);
  EXPECT_EQ(processed[0], 10);
  EXPECT_EQ(processed[1], 1);
}

TEST(MappingWorker, FailureStopsFutureProcessingAndAccountsOnce) {
  ObservationAccounting accounting;
  std::promise<void> fatal_called;
  MappingWorker<int> worker(accounting, [](int&&) {
    throw std::runtime_error("injected mapping failure");
  }, [&](std::exception_ptr) { fatal_called.set_value(); });
  worker.start();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(1));
  fatal_called.get_future().wait();
  ASSERT_TRUE(worker.fatal());
  accounting.recordAcceptedToInbox();
  EXPECT_FALSE(worker.submitFromWaiting(2));
  worker.shutdown();
  const auto state = accounting.snapshot();
  EXPECT_EQ(state.mapping_failed, 1U);
  EXPECT_EQ(state.discarded_waiting, 1U);
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(MappingWorker, ShutdownFinishesInflightAndDiscardsReady) {
  ObservationAccounting accounting;
  std::promise<void> started;
  std::promise<void> release;
  auto released = release.get_future().share();
  MappingWorker<int> worker(accounting, [&](int&& value) {
    if (value == 1) {
      started.set_value();
      released.wait();
    }
  });
  worker.start();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(1));
  started.get_future().wait();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(2));
  auto stopping = std::async(std::launch::async, [&] { worker.shutdown(); });
  while (!worker.stopping()) std::this_thread::yield();
  release.set_value();
  stopping.get();
  worker.shutdown();
  const auto state = accounting.snapshot();
  EXPECT_EQ(state.mapping_published, 1U);
  EXPECT_EQ(state.discarded_shutdown_ready, 1U);
  EXPECT_EQ(state.ready, 0U);
  EXPECT_EQ(state.in_flight, 0U);
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(MappingWorker, ConcurrentProducersAndShutdownPreserveExactLifecycle) {
  using namespace std::chrono_literals;
  ObservationAccounting accounting;
  std::promise<void> first_started;
  std::promise<void> release_first;
  auto released = release_first.get_future().share();
  std::mutex admission_mutex;
  std::mutex processed_mutex;
  std::vector<int> processed;
  std::atomic_int next_sequence{1};
  std::atomic_uint32_t admitted_before_stop{0};
  std::atomic_uint32_t rejected_after_stop{0};
  std::latch producers_admitted{2};
  std::latch release_producers{1};

  MappingWorker<int> worker(accounting, [&](int&& value) {
    if (value == 1) {
      first_started.set_value();
      released.wait();
    }
    std::lock_guard lock(processed_mutex);
    processed.push_back(value);
  });
  worker.setStrictlyIncreasingOrderKey(
      [](const int& value) { return static_cast<std::int64_t>(value); });
  worker.start();

  {
    std::lock_guard lock(admission_mutex);
    accounting.recordAcceptedToInbox();
    ASSERT_TRUE(worker.submitFromWaiting(next_sequence.fetch_add(1)));
  }
  auto first_started_future = first_started.get_future();
  if (first_started_future.wait_for(2s) != std::future_status::ready) {
    release_first.set_value();
    worker.shutdown();
    FAIL() << "worker did not enter deterministic IN_FLIGHT state";
  }

  // Guarantee an IN_FLIGHT item plus a replaced READY item before amplifying
  // the producer/shutdown race.
  for (int index = 0; index < 2; ++index) {
    std::lock_guard lock(admission_mutex);
    accounting.recordAcceptedToInbox();
    if (!worker.submitFromWaiting(next_sequence.fetch_add(1))) {
      release_first.set_value();
      worker.shutdown();
      FAIL() << "worker rejected deterministic READY setup";
    }
  }
  ASSERT_GT(accounting.snapshot().replaced_ready, 0U);

  auto producer = [&] {
    {
      std::lock_guard lock(admission_mutex);
      accounting.recordAcceptedToInbox();
      if (worker.submitFromWaiting(next_sequence.fetch_add(1))) {
        admitted_before_stop.fetch_add(1);
      }
    }
    producers_admitted.count_down();
    release_producers.wait();
    for (int index = 0; index < 999; ++index) {
      std::lock_guard lock(admission_mutex);
      accounting.recordAcceptedToInbox();
      if (!worker.submitFromWaiting(next_sequence.fetch_add(1))) {
        rejected_after_stop.fetch_add(1);
      }
    }
  };
  std::thread producer_a(producer);
  std::thread producer_b(producer);
  producers_admitted.wait();
  if (admitted_before_stop.load() != 2U) {
    release_producers.count_down();
    release_first.set_value();
    worker.shutdown();
    producer_a.join();
    producer_b.join();
    FAIL() << "both producers must overlap the accepting worker";
  }

  auto shutdown_a = std::async(std::launch::async, [&] { worker.shutdown(); });
  auto shutdown_b = std::async(std::launch::async, [&] { worker.shutdown(); });
  while (!worker.stopping()) std::this_thread::yield();
  release_producers.count_down();
  release_first.set_value();
  producer_a.join();
  producer_b.join();
  ASSERT_EQ(shutdown_a.wait_for(2s), std::future_status::ready);
  ASSERT_EQ(shutdown_b.wait_for(2s), std::future_status::ready);
  shutdown_a.get();
  shutdown_b.get();

  const auto state = accounting.snapshot();
  EXPECT_EQ(state.waiting, 0U);
  EXPECT_EQ(state.ready, 0U);
  EXPECT_EQ(state.in_flight, 0U);
  EXPECT_EQ(state.mapping_started, state.mapping_published);
  EXPECT_EQ(state.mapping_failed, 0U);
  EXPECT_GT(state.replaced_ready, 0U);
  EXPECT_LE(state.discarded_shutdown_ready, 1U);
  EXPECT_EQ(admitted_before_stop.load(), 2U);
  EXPECT_GT(rejected_after_stop.load(), 0U);
  EXPECT_TRUE(state.allInvariantsHold());

  std::lock_guard lock(processed_mutex);
  EXPECT_FALSE(processed.empty());
  EXPECT_TRUE(std::is_sorted(processed.begin(), processed.end()));
  EXPECT_EQ(std::adjacent_find(processed.begin(), processed.end()), processed.end());
}

TEST(MappingWorker, ValidationDiscardDoesNotStartMapping) {
  ObservationAccounting accounting;
  std::atomic_bool processed{false};
  std::atomic_int validation_calls{0};
  std::promise<void> validated;
  MappingWorker<int> worker(
      accounting, [&](int&&) { processed.store(true); }, {},
      [&](const int& value) {
        const int call = ++validation_calls;
        if (call == 2) validated.set_value();
        return value > 0 && call == 1;
      });
  worker.start();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(1));
  validated.get_future().wait();
  worker.shutdown();
  EXPECT_FALSE(processed.load());
  const auto state = accounting.snapshot();
  EXPECT_EQ(state.discarded_ready, 1U);
  EXPECT_EQ(state.mapping_started, 0U);
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(MappingWorker, RejectsDuplicateAndRegressedOrderBeforeProcessing) {
  ObservationAccounting accounting;
  std::promise<void> processed;
  MappingWorker<int> worker(accounting, [&](int&&) { processed.set_value(); });
  worker.setStrictlyIncreasingOrderKey([](const int& value) {
    return static_cast<std::int64_t>(value);
  });
  worker.start();
  accounting.recordAcceptedToInbox();
  ASSERT_TRUE(worker.submitFromWaiting(10));
  processed.get_future().wait();
  accounting.recordAcceptedToInbox();
  EXPECT_FALSE(worker.submitFromWaiting(10));
  accounting.recordAcceptedToInbox();
  EXPECT_FALSE(worker.submitFromWaiting(9));
  worker.shutdown();
  const auto state = accounting.snapshot();
  EXPECT_EQ(state.mapping_published, 1U);
  EXPECT_EQ(state.discarded_nonmonotonic, 2U);
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(MappingWorker, ValidationExceptionIsFatalAndTerminallyAccountsWaiting) {
  ObservationAccounting accounting;
  std::promise<void> fatal_called;
  MappingWorker<int> worker(
      accounting, [](int&&) {},
      [&](std::exception_ptr failure) {
        EXPECT_NE(failure, nullptr);
        fatal_called.set_value();
      },
      [](const int&) -> bool { throw std::runtime_error("validation failed"); });
  worker.start();
  accounting.recordAcceptedToInbox();
  EXPECT_FALSE(worker.submitFromWaiting(1));
  fatal_called.get_future().wait();
  worker.shutdown();
  const auto state = accounting.snapshot();
  EXPECT_EQ(state.discarded_waiting, 1U);
  EXPECT_EQ(state.mapping_started, 0U);
  EXPECT_EQ(state.mapping_failed, 0U);
  EXPECT_TRUE(worker.fatal());
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(ObservationAccounting, AccountsRejectedDiscardedAndFailedPaths) {
  ObservationAccounting accounting;
  accounting.recordRejectedBeforeInbox();

  accounting.recordAcceptedToInbox();
  accounting.discardedPending();

  accounting.recordAcceptedToInbox();
  accounting.waitingSubmitted(false);
  accounting.mappingStarted();
  accounting.mappingFailed();

  const auto state = accounting.snapshot();
  EXPECT_EQ(state.rejected_before_inbox, 1U);
  EXPECT_EQ(state.discarded_pending, 1U);
  EXPECT_EQ(state.mapping_failed, 1U);
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(ObservationAccounting, SnapshotIsInternallyConsistentUnderConcurrentReplacement) {
  ObservationAccounting accounting;
  for (int i = 0; i < 100; ++i) {
    accounting.recordAcceptedToInbox();
    EXPECT_TRUE(accounting.snapshot().allInvariantsHold());
  }
}

TEST(ObservationAccounting, IllegalDuplicateTerminalLatchesViolation) {
  ObservationAccounting accounting;
  accounting.recordAcceptedToInbox();
  accounting.waitingSubmitted(false);
  accounting.mappingStarted();
  accounting.mappingPublished();
  accounting.mappingPublished();

  const auto state = accounting.snapshot();
  EXPECT_EQ(state.violation_count, 1U);
  EXPECT_FALSE(state.allInvariantsHold());
}

TEST(ObservationAccounting, ConcurrentSnapshotsRemainCoherent) {
  ObservationAccounting accounting;
  std::atomic_bool finished{false};
  std::atomic_bool coherent{true};
  std::thread reader([&] {
    while (!finished.load()) {
      if (!accounting.snapshot().allInvariantsHold()) coherent.store(false);
    }
  });
  for (int i = 0; i < 10000; ++i) {
    accounting.recordAcceptedToInbox();
    accounting.waitingSubmitted(false);
    accounting.mappingStarted();
    accounting.mappingPublished();
  }
  finished.store(true);
  reader.join();

  EXPECT_TRUE(coherent.load());
  EXPECT_TRUE(accounting.snapshot().allInvariantsHold());
}

}  // namespace
}  // namespace navigation_mapping
