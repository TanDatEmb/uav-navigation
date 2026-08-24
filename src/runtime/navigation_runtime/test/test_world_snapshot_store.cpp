#include <navigation_runtime/world_snapshot_store.hpp>
#include <super_core/super_planner.h>
#include <super_core/trajectory_world_validator.hpp>

#include <atomic>
#include <future>
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

}  // namespace
