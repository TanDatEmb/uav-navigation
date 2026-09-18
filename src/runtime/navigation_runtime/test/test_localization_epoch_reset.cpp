#include "navigation_runtime/localization_epoch_reset.hpp"

#include <atomic>
#include <future>
#include <latch>
#include <mutex>
#include <stdexcept>
#include <unistd.h>

#include <gtest/gtest.h>
#include <navigation_mapping/mapping_worker.hpp>
#include <navigation_mapping/observation_accounting.hpp>

namespace navigation_runtime {
namespace {

TEST(LocalizationEpochReset, RealMappingPublicationCanDrainWithEmptyTimeline) {
  // A child watchdog bounds a genuinely blocked legacy topology. The event
  // order is controlled by a latch, not by sleeps or repeated lucky runs.
  ASSERT_EXIT({
    alarm(3);
    std::mutex lifecycle_mutex;
    std::unique_lock lifecycle_lock(lifecycle_mutex);
    navigation_mapping::ObservationAccounting accounting;
    std::latch callback_started{1};
    std::latch publication_allowed{1};
    std::atomic_bool publication_finished{false};
    navigation_mapping::MappingWorker<int> worker(accounting, [&](int&&) {
      callback_started.count_down();
      publication_allowed.wait();
      std::lock_guard lock(lifecycle_mutex);
      publication_finished.store(true);
    });
    worker.start();
    accounting.recordAcceptedToInbox();
    if (!worker.submitFromWaiting(1)) _exit(10);
    callback_started.wait();
    drainMappingForLocalizationReset(lifecycle_lock, [&] {
      publication_allowed.count_down();
      worker.reset();
    });
    if (!lifecycle_lock.owns_lock() || !publication_finished.load()) _exit(11);
    worker.shutdown();
    if (!accounting.snapshot().allInvariantsHold()) _exit(12);
    alarm(0);
    _exit(0);
  }, testing::ExitedWithCode(0), "");
}

TEST(LocalizationEpochReset, RelocksOwnerAfterSuccessfulDrain) {
  std::mutex mutex;
  std::unique_lock owner(mutex);
  bool drain_observed_unlocked_owner = false;
  drainMappingForLocalizationReset(owner, [&] {
    drain_observed_unlocked_owner = std::async(std::launch::async, [&] {
      const bool unlocked = mutex.try_lock();
      if (unlocked) mutex.unlock();
      return unlocked;
    }).get();
  });
  EXPECT_TRUE(drain_observed_unlocked_owner);
  EXPECT_TRUE(owner.owns_lock());
}

TEST(LocalizationEpochReset, RelocksOwnerWhenDrainThrows) {
  std::mutex mutex;
  std::unique_lock owner(mutex);
  bool drain_observed_unlocked_owner = false;
  EXPECT_THROW(drainMappingForLocalizationReset(owner, [&] {
    drain_observed_unlocked_owner = std::async(std::launch::async, [&] {
      const bool unlocked = mutex.try_lock();
      if (unlocked) mutex.unlock();
      return unlocked;
    }).get();
    throw std::runtime_error("drain failed");
  }), std::runtime_error);
  EXPECT_TRUE(drain_observed_unlocked_owner);
  EXPECT_TRUE(owner.owns_lock());
}

TEST(LocalizationEpochReset, RejectsMissingOwnerLockBeforeDrain) {
  std::mutex mutex;
  std::unique_lock owner(mutex, std::defer_lock);
  bool called = false;
  EXPECT_THROW(drainMappingForLocalizationReset(owner, [&] { called = true; }),
               std::invalid_argument);
  EXPECT_FALSE(called);
}

}  // namespace
}  // namespace navigation_runtime
