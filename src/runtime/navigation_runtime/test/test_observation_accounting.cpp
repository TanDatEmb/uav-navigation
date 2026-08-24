#include "navigation_runtime/observation_accounting.hpp"
#include "navigation_runtime/mapping_worker.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <future>

namespace navigation_runtime {
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
}  // namespace navigation_runtime
