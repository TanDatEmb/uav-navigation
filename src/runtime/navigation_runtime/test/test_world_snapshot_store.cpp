#include <navigation_mapping/world_snapshot_store.hpp>

#include <atomic>
#include <future>
#include <latch>
#include <mutex>
#include <optional>
#include <thread>
#include <type_traits>

#include <gtest/gtest.h>

namespace {

class IdentityOnlyWorld final : public navigation_world_model::WorldModelView {
 public:
  explicit IdentityOnlyWorld(navigation_world_model::WorldSnapshotIdentity identity)
      : identity_(identity) {}

  navigation_world_model::WorldGeometry geometry() const noexcept override {
    navigation_world_model::WorldGeometry result;
    result.inflated_resolution_m = 0.2;
    return result;
  }
  navigation_world_model::WorldSnapshotIdentity identity() const noexcept override {
    return identity_;
  }
  navigation_world_model::CellState classify(
      const navigation_world_model::Point3&, navigation_world_model::GridLayer) const noexcept override {
    return navigation_world_model::CellState::kUnknown;
  }
  bool contains(const navigation_world_model::Point3&) const noexcept override { return true; }
  navigation_world_model::GridIndex3 positionToIndex(
      const navigation_world_model::Point3&, navigation_world_model::GridLayer) const noexcept override {
    return navigation_world_model::GridIndex3::Zero();
  }
  navigation_world_model::Point3 indexToPosition(
      const navigation_world_model::GridIndex3&, navigation_world_model::GridLayer) const noexcept override {
    return navigation_world_model::Point3::Zero();
  }
  std::optional<navigation_world_model::Point3> nearestNotOccupied(
      const navigation_world_model::Point3& start, navigation_world_model::GridLayer,
      double) const override {
    return start;
  }
  bool isSegmentTraversable(
      const navigation_world_model::Point3&, const navigation_world_model::Point3&,
      navigation_world_model::GridLayer,
      navigation_world_model::UnknownPolicy) const noexcept override {
    return true;
  }
  navigation_world_model::AxisAlignedBox clampToLocalBounds(
      const navigation_world_model::AxisAlignedBox& box) const noexcept override {
    return box;
  }
  navigation_world_model::PointVector observedOccupiedPoints(
      const navigation_world_model::AxisAlignedBox&) const override {
    return {};
  }

 private:
  navigation_world_model::WorldSnapshotIdentity identity_;
};

navigation_world_model::WorldModelViewPtr world(
    std::uint64_t generation, std::uint64_t revision, std::int64_t stamp,
    std::uint64_t localization_epoch = 1U) {
  return std::make_shared<IdentityOnlyWorld>(
      navigation_world_model::WorldSnapshotIdentity{
          localization_epoch, generation, revision, stamp});
}

// This is deliberately a product-level test command. It models only the
// atomic state that the execution boundary needs; it does not pull planner
// implementation headers into the runtime package.
class TestCommand final {
 public:
  struct Snapshot {
    std::uint64_t generation{0};
    navigation_world_model::WorldSnapshotIdentity pinned_world{};
    navigation_world_model::WorldSnapshotIdentity validated_world{};
    std::uint64_t position_revision{0};
    bool backup{false};
  };

  bool commit(const navigation_world_model::WorldSnapshotIdentity& identity) {
    std::lock_guard<std::mutex> guard(mutex_);
    pinned_world_ = identity;
    validated_world_ = identity;
    position_revision_ = identity.revision;
    backup_ = true;
    ++generation_;
    return true;
  }

  Snapshot snapshot() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return {generation_, pinned_world_, validated_world_, position_revision_, backup_};
  }

 private:
  mutable std::mutex mutex_;
  std::uint64_t generation_{0};
  navigation_world_model::WorldSnapshotIdentity pinned_world_{};
  navigation_world_model::WorldSnapshotIdentity validated_world_{};
  std::uint64_t position_revision_{0};
  bool backup_{false};
};

static_assert(!std::is_copy_constructible_v<navigation_mapping::WorldSnapshotStore>);

TEST(WorldSnapshotStore, PublishesAndPinsOneCoherentImmutableIdentity) {
  navigation_mapping::WorldSnapshotStore store;
  EXPECT_FALSE(store.load());
  auto first = world(1, 0, 0);
  store.publish(first);
  const auto pinned = store.load();
  ASSERT_TRUE(pinned);
  EXPECT_EQ(pinned.view.get(), first.get());
  EXPECT_TRUE(navigation_mapping::WorldSnapshotStore::sameIdentity(
      pinned.identity, {1, 1, 0, 0}));

  store.publish(world(1, 1, 100));
  EXPECT_EQ(pinned.view.get(), first.get());
  EXPECT_TRUE(navigation_mapping::WorldSnapshotStore::sameIdentity(
      pinned.identity, {1, 1, 0, 0}));
  EXPECT_TRUE(navigation_mapping::WorldSnapshotStore::sameIdentity(
      store.load().identity, {1, 1, 1, 100}));
}

TEST(WorldSnapshotStore, RejectsNullInvalidAndNonMonotonicPublication) {
  navigation_mapping::WorldSnapshotStore store;
  EXPECT_THROW(store.publish(nullptr), std::invalid_argument);
  EXPECT_THROW(store.publish(world(0, 0, 0)), std::invalid_argument);
  store.publish(world(1, 3, 300));
  EXPECT_THROW(store.publish(world(1, 3, 301)), std::logic_error);
  EXPECT_THROW(store.publish(world(1, 4, 299)), std::logic_error);
  EXPECT_THROW(store.publish(world(0, 5, 500)), std::invalid_argument);
  EXPECT_NO_THROW(store.publish(world(2, 0, 0)));
}

TEST(WorldSnapshotStore, LocalizationEpochIsPartOfPublicationIdentity) {
  navigation_mapping::WorldSnapshotStore store;
  store.publish(world(7, 1, 100, 1));

  EXPECT_EQ(store.authorize({2, 7, 1, 100}, [] { return true; }),
            navigation_world_model::WorldCommitDecision::kWorldAdvanced);
  EXPECT_NO_THROW(store.publish(world(1, 0, 0, 2)));
  EXPECT_EQ(store.load().identity.localization_epoch, 2U);
  EXPECT_EQ(store.load().identity.generation, 1U);
}

TEST(WorldSnapshotStore, AuthorizationRejectsStaleIdentityWithoutInvokingCommit) {
  navigation_mapping::WorldSnapshotStore store;
  store.publish(world(1, 1, 100));
  bool invoked = false;
  EXPECT_EQ(store.authorize({1, 1, 0, 0}, [&] {
              invoked = true;
              return true;
            }),
            navigation_world_model::WorldCommitDecision::kWorldAdvanced);
  EXPECT_FALSE(invoked);
  EXPECT_EQ(store.authorize({1, 1, 1, 100}, [&] {
              invoked = true;
              return true;
            }),
            navigation_world_model::WorldCommitDecision::kCommitted);
  EXPECT_TRUE(invoked);
}

TEST(WorldSnapshotStore, PublicationCannotInterleaveAnAuthorizedCommit) {
  navigation_mapping::WorldSnapshotStore store;
  store.publish(world(1, 1, 100));
  std::promise<void> commit_entered;
  std::promise<void> release_commit;
  auto release = release_commit.get_future().share();
  auto authorization = std::async(std::launch::async, [&] {
    return store.authorize({1, 1, 1, 100}, [&] {
      commit_entered.set_value();
      release.wait();
      return true;
    });
  });
  commit_entered.get_future().wait();
  std::atomic_bool publication_finished{false};
  std::promise<void> publication_started;
  auto publication = std::async(std::launch::async, [&] {
    publication_started.set_value();
    store.publish(world(1, 2, 200));
    publication_finished.store(true);
  });
  publication_started.get_future().wait();
  EXPECT_FALSE(publication_finished.load());
  release_commit.set_value();
  EXPECT_EQ(authorization.get(), navigation_world_model::WorldCommitDecision::kCommitted);
  publication.get();
  EXPECT_TRUE(publication_finished.load());
  EXPECT_EQ(store.load().identity.revision, 2U);
}

TEST(WorldSnapshotStore, WorldAdvanceAfterCandidateValidationCannotCommitCommand) {
  navigation_mapping::WorldSnapshotStore store;
  store.publish(world(1, 1, 100));
  const auto lease = store.latest();
  ASSERT_TRUE(lease);
  store.publish(world(1, 2, 200));

  TestCommand command;
  bool commit_invoked = false;
  EXPECT_EQ(store.commitIfCurrent(lease.identity, [&] {
              commit_invoked = true;
              return command.commit(lease.identity);
            }),
            navigation_world_model::WorldCommitDecision::kWorldAdvanced);
  EXPECT_FALSE(commit_invoked);
  EXPECT_EQ(command.snapshot().generation, 0U);
}

TEST(WorldSnapshotStore, ConcurrentPublishAuthorizeAndCommandSampleStayCoherent) {
  constexpr std::uint64_t kIterations = 100;
  navigation_mapping::WorldSnapshotStore store;
  store.publish(world(1, 1, 100));
  TestCommand command;
  ASSERT_TRUE(command.commit(store.latest().identity));

  std::atomic_bool publisher_done{false};
  std::atomic_bool authorizer_done{false};
  std::atomic_bool publisher_active{false};
  std::atomic_bool authorizer_active{false};
  std::atomic_bool incoherent{false};
  std::atomic_uint64_t committed{0};
  std::atomic_uint64_t world_advanced{0};
  std::atomic_uint64_t overlap_samples{0};
  std::atomic_uint64_t observed_generation_changes{0};
  std::atomic_bool first_authorization_committed{false};
  std::latch sampler_ready{1};
  std::latch writers_ready{2};
  std::latch start_writers{1};
  std::latch first_authorization_done{1};
  std::latch sampler_observed_change{1};

  std::thread publisher([&] {
    publisher_active.store(true, std::memory_order_release);
    writers_ready.count_down();
    start_writers.wait();
    sampler_observed_change.wait();
    for (std::uint64_t revision = 2; revision <= kIterations; ++revision) {
      store.publish(world(1, revision, static_cast<std::int64_t>(revision * 100)));
      if ((revision & 31U) == 0U) std::this_thread::yield();
    }
    publisher_active.store(false, std::memory_order_release);
    publisher_done.store(true, std::memory_order_release);
  });
  std::thread authorizer([&] {
    authorizer_active.store(true, std::memory_order_release);
    writers_ready.count_down();
    start_writers.wait();
    const auto authorize_once = [&] {
      const auto lease = store.latest();
      const auto decision = store.commitIfCurrent(lease.identity, [&] {
        return command.commit(lease.identity);
      });
      if (decision == navigation_world_model::WorldCommitDecision::kCommitted) {
        committed.fetch_add(1);
        return true;
      }
      if (decision == navigation_world_model::WorldCommitDecision::kWorldAdvanced) {
        world_advanced.fetch_add(1);
      } else {
        incoherent.store(true);
      }
      return false;
    };
    first_authorization_committed.store(
        authorize_once(), std::memory_order_release);
    first_authorization_done.count_down();
    sampler_observed_change.wait();
    for (std::uint64_t index = 1; index < kIterations; ++index) {
      authorize_once();
      if ((index & 31U) == 0U) std::this_thread::yield();
    }
    authorizer_active.store(false, std::memory_order_release);
    authorizer_done.store(true, std::memory_order_release);
  });
  std::thread sampler([&] {
    auto previous = command.snapshot();
    bool change_signaled = false;
    sampler_ready.count_down();
    start_writers.wait();
    first_authorization_done.wait();
    if (!first_authorization_committed.load(std::memory_order_acquire)) {
      incoherent.store(true);
    } else {
      observed_generation_changes.fetch_add(1);
    }
    sampler_observed_change.count_down();
    change_signaled = true;
    do {
      const auto current = command.snapshot();
      if (publisher_active.load(std::memory_order_acquire) ||
          authorizer_active.load(std::memory_order_acquire)) {
        overlap_samples.fetch_add(1);
      }
      if (current.generation != previous.generation) {
        observed_generation_changes.fetch_add(1);
        if (!change_signaled) {
          sampler_observed_change.count_down();
          change_signaled = true;
        }
      }
      if (current.generation < previous.generation || !current.backup ||
          !navigation_mapping::WorldSnapshotStore::sameIdentity(
              current.pinned_world, current.validated_world) ||
          current.position_revision != current.validated_world.revision) {
        incoherent.store(true);
      }
      previous = current;
    } while (!publisher_done.load(std::memory_order_acquire) ||
             !authorizer_done.load(std::memory_order_acquire));
  });

  sampler_ready.wait();
  writers_ready.wait();
  start_writers.count_down();
  publisher.join();
  authorizer.join();
  sampler.join();

  EXPECT_FALSE(incoherent.load());
  EXPECT_GT(committed.load(), 0U);
  EXPECT_GT(world_advanced.load(), 0U);
  EXPECT_GT(overlap_samples.load(), 0U);
  EXPECT_GT(observed_generation_changes.load(), 0U);
  EXPECT_EQ(store.load().identity.revision, kIterations);
}

}  // namespace
