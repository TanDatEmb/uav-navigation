#include <navigation_runtime/world_snapshot_store.hpp>
#include <super_core/super_planner.h>
#include <super_core/trajectory_world_validator.hpp>

#include <atomic>
#include <cmath>
#include <future>
#include <latch>
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

navigation_world_model::WorldModelViewPtr world(std::uint64_t generation,
                                                 std::uint64_t revision,
                                                 std::int64_t stamp) {
  return std::make_shared<IdentityOnlyWorld>(
      navigation_world_model::WorldSnapshotIdentity{generation, revision, stamp});
}

std::optional<super_planner::CandidateCommandBundle> commandForRevision(
    std::uint64_t revision) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 7) = static_cast<double>(revision);
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({0.1}, {coefficients});
  geometry_utils::Trajectory yaw({0.1}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 10.0;
  return super_planner::CmdTraj::buildEmergencyCandidate(position, yaw);
}

static_assert(!std::is_copy_constructible_v<navigation_runtime::WorldSnapshotStore>);

TEST(WorldSnapshotStore, PublishesAndPinsOneCoherentImmutableIdentity) {
  navigation_runtime::WorldSnapshotStore store;
  EXPECT_FALSE(store.load());
  auto first = world(1, 0, 0);
  store.publish(first);
  const auto pinned = store.load();
  ASSERT_TRUE(pinned);
  EXPECT_EQ(pinned.view.get(), first.get());
  EXPECT_TRUE(navigation_runtime::WorldSnapshotStore::sameIdentity(
      pinned.identity, {1, 0, 0}));

  store.publish(world(1, 1, 100));
  EXPECT_EQ(pinned.view.get(), first.get());
  EXPECT_TRUE(navigation_runtime::WorldSnapshotStore::sameIdentity(
      pinned.identity, {1, 0, 0}));
  EXPECT_TRUE(navigation_runtime::WorldSnapshotStore::sameIdentity(
      store.load().identity, {1, 1, 100}));
}

TEST(WorldSnapshotStore, RejectsNullInvalidAndNonMonotonicPublication) {
  navigation_runtime::WorldSnapshotStore store;
  EXPECT_THROW(store.publish(nullptr), std::invalid_argument);
  EXPECT_THROW(store.publish(world(0, 0, 0)), std::invalid_argument);
  store.publish(world(1, 3, 300));
  EXPECT_THROW(store.publish(world(1, 3, 301)), std::logic_error);
  EXPECT_THROW(store.publish(world(1, 4, 299)), std::logic_error);
  EXPECT_THROW(store.publish(world(0, 5, 500)), std::invalid_argument);
  EXPECT_NO_THROW(store.publish(world(2, 0, 0)));
}

TEST(WorldSnapshotStore, AuthorizationRejectsStaleIdentityWithoutInvokingCommit) {
  navigation_runtime::WorldSnapshotStore store;
  store.publish(world(1, 1, 100));
  bool invoked = false;
  EXPECT_EQ(store.authorize({1, 0, 0}, [&] {
              invoked = true;
              return true;
            }),
            navigation_runtime::WorldCommitDecision::kWorldAdvanced);
  EXPECT_FALSE(invoked);
  EXPECT_EQ(store.authorize({1, 1, 100}, [&] {
              invoked = true;
              return true;
            }),
            navigation_runtime::WorldCommitDecision::kCommitted);
  EXPECT_TRUE(invoked);
}

TEST(WorldSnapshotStore, PublicationCannotInterleaveAnAuthorizedCommit) {
  navigation_runtime::WorldSnapshotStore store;
  store.publish(world(1, 1, 100));
  std::promise<void> commit_entered;
  std::promise<void> release_commit;
  auto release = release_commit.get_future().share();
  auto authorization = std::async(std::launch::async, [&] {
    return store.authorize({1, 1, 100}, [&] {
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
  EXPECT_EQ(authorization.get(), navigation_runtime::WorldCommitDecision::kCommitted);
  publication.get();
  EXPECT_TRUE(publication_finished.load());
  EXPECT_EQ(store.load().identity.revision, 2U);
}

TEST(WorldSnapshotStore, WorldAdvanceAfterCandidateValidationCannotCommitBundle) {
  navigation_runtime::WorldSnapshotStore store;
  store.publish(world(1, 1, 100));
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 8);
  coefficients(0, 6) = 1.0;
  coefficients(2, 7) = 3.0;
  geometry_utils::Trajectory position({1.0}, {coefficients});
  geometry_utils::Trajectory yaw({1.0}, {Eigen::MatrixXd::Zero(3, 8)});
  position.start_WT = yaw.start_WT = 10.0;
  auto candidate = super_planner::CmdTraj::buildEmergencyCandidate(position, yaw);
  ASSERT_TRUE(candidate);
  const auto lease = store.latest();
  ASSERT_TRUE(super_planner::validateExecutableCandidate(
      *lease.view, *candidate, 10.0).valid);

  store.publish(world(1, 2, 200));
  super_planner::CmdTraj command;
  bool commit_invoked = false;
  EXPECT_EQ(store.commitIfCurrent(lease.identity, [&] {
              commit_invoked = true;
              return command.commitCandidate(std::move(*candidate), {});
            }), navigation_world_model::WorldCommitDecision::kWorldAdvanced);
  EXPECT_FALSE(commit_invoked);
  EXPECT_TRUE(command.empty());
  EXPECT_EQ(command.generation(), 0U);
}

TEST(WorldSnapshotStore, ConcurrentPublishAuthorizeAndCommandSampleStayCoherent) {
  constexpr std::uint64_t kIterations = 10000;
  navigation_runtime::WorldSnapshotStore store;
  store.publish(world(1, 1, 100));
  super_planner::CmdTraj command;
  auto initial = commandForRevision(1);
  ASSERT_TRUE(initial);
  ASSERT_TRUE(command.commitCandidate(
      std::move(*initial), {{1, 1, 100}, {1, 1, 100}, 0.0}));

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
    auto authorize_once = [&] {
      const auto lease = store.latest();
      auto candidate = commandForRevision(lease.identity.revision);
      if (!candidate) {
        incoherent.store(true);
        return false;
      }
      const super_planner::CommandCertificate certificate{
          lease.identity, lease.identity, 0.0};
      const auto decision = store.commitIfCurrent(lease.identity, [&] {
        return command.commitCandidate(std::move(*candidate), certificate);
      });
      if (decision == navigation_world_model::WorldCommitDecision::kCommitted) {
        committed.fetch_add(1);
        return true;
      } else if (decision ==
                 navigation_world_model::WorldCommitDecision::kWorldAdvanced) {
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
    std::uint64_t previous_generation = command.generation();
    bool change_signaled = false;
    sampler_ready.count_down();
    start_writers.wait();
    first_authorization_done.wait();
    if (!first_authorization_committed.load(std::memory_order_acquire)) {
      incoherent.store(true);
      sampler_observed_change.count_down();
      change_signaled = true;
    }
    do {
      std::uint64_t generation = 0;
      navigation_world_model::WorldSnapshotIdentity pinned{};
      navigation_world_model::WorldSnapshotIdentity validated{};
      super_utils::Vec3f position = super_utils::Vec3f::Zero();
      super_utils::Vec3f yaw = super_utils::Vec3f::Zero();
      bool backup = false;
      command.lock();
      generation = command.generation();
      pinned = command.certificate().pinned_world;
      validated = command.certificate().validated_world;
      position = command.getPos(0.0);
      yaw = command.getYaw(0.0);
      backup = command.isTTOnBackupTraj(0.0);
      command.unlock();

      if (publisher_active.load(std::memory_order_acquire) ||
          authorizer_active.load(std::memory_order_acquire)) {
        overlap_samples.fetch_add(1);
      }
      if (generation != previous_generation) {
        observed_generation_changes.fetch_add(1);
        if (!change_signaled) {
          sampler_observed_change.count_down();
          change_signaled = true;
        }
      }
      if (generation < previous_generation || !position.allFinite() ||
          !yaw.allFinite() || !backup ||
          !navigation_runtime::WorldSnapshotStore::sameIdentity(pinned, validated) ||
          position.x() != static_cast<double>(validated.revision)) {
        incoherent.store(true);
      }
      previous_generation = generation;
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
  EXPECT_GT(overlap_samples.load(), 0U);
  EXPECT_GT(observed_generation_changes.load(), 0U);
  EXPECT_EQ(store.load().identity.revision, kIterations);
}

}  // namespace
