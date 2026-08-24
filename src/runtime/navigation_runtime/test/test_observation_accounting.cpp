#include "navigation_runtime/observation_accounting.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

namespace navigation_runtime {
namespace {

TEST(ObservationAccounting, AccountsPublishedObservationExactlyOnce) {
  ObservationAccounting accounting;
  EXPECT_FALSE(accounting.recordAcceptedToInbox());
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
  EXPECT_EQ(state.pending, 1U);
  EXPECT_TRUE(state.allInvariantsHold());
}

TEST(ObservationAccounting, AccountsRejectedDiscardedAndFailedPaths) {
  ObservationAccounting accounting;
  accounting.recordRejectedBeforeInbox();

  accounting.recordAcceptedToInbox();
  accounting.discardedPending();

  accounting.recordAcceptedToInbox();
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
