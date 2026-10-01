#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <navigation_world_model/world_model_view.hpp>
#include <navigation_world_model/world_commit_authorizer.hpp>

namespace navigation_mapping {

using WorldCommitDecision = navigation_world_model::WorldCommitDecision;

struct PinnedWorldSnapshot {
  navigation_world_model::WorldModelViewPtr view;
  navigation_world_model::WorldSnapshotIdentity identity;

  [[nodiscard]] explicit operator bool() const noexcept {
    return static_cast<bool>(view);
  }
};

// Product-owned publication boundary between the sole mutable mapping owner
// and planners that pin immutable views. Snapshot construction and candidate
// validation happen outside publication_gate_; only publication and the final
// authorized commit are linearized here.
class WorldSnapshotStore final
    : public navigation_world_model::WorldCommitAuthorizer {
 public:
  WorldSnapshotStore() = default;
  WorldSnapshotStore(const WorldSnapshotStore&) = delete;
  WorldSnapshotStore& operator=(const WorldSnapshotStore&) = delete;

  [[nodiscard]] PinnedWorldSnapshot load() const noexcept {
    auto view = latest_.load(std::memory_order_acquire);
    return {view, view ? view->identity()
                       : navigation_world_model::WorldSnapshotIdentity{}};
  }

  [[nodiscard]] navigation_world_model::WorldValidationLease latest()
      const noexcept override {
    const auto pinned = load();
    return {pinned.view, pinned.identity};
  }

  void publish(navigation_world_model::WorldModelViewPtr next) {
    if (!next) throw std::invalid_argument("cannot publish a null WorldModel snapshot");
    const auto next_identity = next->identity();
    if (next_identity.localization_epoch == 0U || next_identity.generation == 0U ||
        next_identity.observation_stamp_ns < 0) {
      throw std::invalid_argument("cannot publish an invalid WorldModel identity");
    }
    std::lock_guard<std::mutex> guard(publication_gate_);
    const auto current = latest_.load(std::memory_order_relaxed);
    if (current && !strictlyAdvances(current->identity(), next_identity)) {
      throw std::logic_error("WorldModel publication identity is not monotonic");
    }
    latest_.store(std::move(next), std::memory_order_release);
  }

  // Publish the immutable view and run a dependent certificate transition
  // under one publication gate.  A planner may authorize a candidate only
  // after this operation returns, so the world identity and the execution
  // certificate cannot be observed in opposite orders.
  template <typename FinalizeFunction>
  bool publishAndFinalize(navigation_world_model::WorldModelViewPtr next,
                          FinalizeFunction&& finalize) {
    return publishAndFinalizeDecision(std::move(next),
                                      std::forward<FinalizeFunction>(finalize)) ==
        navigation_world_model::WorldCommitDecision::kCommitted;
  }

  // Typed form of publishAndFinalize.  A stale dependent transaction is a
  // normal optimistic-concurrency outcome, not a finalization failure: leave
  // the old world visible and let the newer callback retry from a fresh
  // execution snapshot.
  template <typename FinalizeFunction>
  navigation_world_model::WorldCommitDecision publishAndFinalizeDecision(
      navigation_world_model::WorldModelViewPtr next,
      FinalizeFunction&& finalize) {
    if (!next) throw std::invalid_argument("cannot publish a null WorldModel snapshot");
    const auto next_identity = next->identity();
    if (next_identity.localization_epoch == 0U || next_identity.generation == 0U ||
        next_identity.observation_stamp_ns < 0) {
      throw std::invalid_argument("cannot publish an invalid WorldModel identity");
    }
    std::lock_guard<std::mutex> guard(publication_gate_);
    const auto current = latest_.load(std::memory_order_relaxed);
    if (current && !strictlyAdvances(current->identity(), next_identity)) {
      throw std::logic_error("WorldModel publication identity is not monotonic");
    }
    try {
      using FinalizeResult = std::decay_t<std::invoke_result_t<FinalizeFunction>>;
      if constexpr (std::is_same_v<FinalizeResult,
                                   navigation_world_model::WorldCommitDecision>) {
        const auto decision = std::invoke(std::forward<FinalizeFunction>(finalize));
        if (decision != navigation_world_model::WorldCommitDecision::kCommitted) {
          return decision;
        }
      } else if (!std::invoke(std::forward<FinalizeFunction>(finalize))) {
        return navigation_world_model::WorldCommitDecision::kCancelled;
      }
    } catch (...) {
      return navigation_world_model::WorldCommitDecision::kCancelled;
    }
    // Keep the old world visible until all dependent execution state has been
    // invalidated or recertified.  Readers therefore cannot observe a new
    // world while an old-world command is still the only available bundle.
    latest_.store(std::move(next), std::memory_order_release);
    return navigation_world_model::WorldCommitDecision::kCommitted;
  }

  template <typename CommitFunction>
  navigation_world_model::WorldCommitDecision authorize(
      const navigation_world_model::WorldSnapshotIdentity& validated_identity,
      CommitFunction&& commit) {
    std::lock_guard<std::mutex> guard(publication_gate_);
    const auto latest = latest_.load(std::memory_order_acquire);
    if (!latest) return navigation_world_model::WorldCommitDecision::kNoPublishedWorld;
    if (!sameIdentity(latest->identity(), validated_identity)) {
      return navigation_world_model::WorldCommitDecision::kWorldAdvanced;
    }
    return std::invoke(std::forward<CommitFunction>(commit))
        ? navigation_world_model::WorldCommitDecision::kCommitted
        : navigation_world_model::WorldCommitDecision::kCancelled;
  }

  navigation_world_model::WorldCommitDecision commitIfCurrent(
      const navigation_world_model::WorldSnapshotIdentity& validated_identity,
      const std::function<bool()>& final_commit) override {
    return authorize(validated_identity, final_commit);
  }

  navigation_world_model::WorldCommitDecision commitIfCurrentOrUnaffected(
      const navigation_world_model::WorldSnapshotIdentity& validated_identity,
      const navigation_world_model::AxisAlignedBox& protected_region,
      const std::function<bool(const navigation_world_model::WorldValidationLease&)>&
          final_commit) override {
    std::lock_guard<std::mutex> guard(publication_gate_);
    const auto latest = latest_.load(std::memory_order_acquire);
    if (!latest) return navigation_world_model::WorldCommitDecision::kNoPublishedWorld;
    const auto latest_identity = latest->identity();
    if (!sameIdentity(latest_identity, validated_identity) &&
        latest->changedRegionIntersectsSince(validated_identity, protected_region)) {
      return navigation_world_model::WorldCommitDecision::kWorldAdvanced;
    }
    const navigation_world_model::WorldValidationLease lease{latest, latest_identity};
    return std::invoke(final_commit, lease)
        ? navigation_world_model::WorldCommitDecision::kCommitted
        : navigation_world_model::WorldCommitDecision::kCancelled;
  }

  [[nodiscard]] static bool sameIdentity(
      const navigation_world_model::WorldSnapshotIdentity& lhs,
      const navigation_world_model::WorldSnapshotIdentity& rhs) noexcept {
    return navigation_world_model::sameWorldSnapshotIdentity(lhs, rhs);
  }

 private:
  [[nodiscard]] static bool strictlyAdvances(
      const navigation_world_model::WorldSnapshotIdentity& current,
      const navigation_world_model::WorldSnapshotIdentity& next) noexcept {
    if (next.localization_epoch != current.localization_epoch) {
      return next.localization_epoch > current.localization_epoch;
    }
    if (next.generation != current.generation) {
      return next.generation > current.generation;
    }
    return next.revision > current.revision &&
           next.observation_stamp_ns >= current.observation_stamp_ns;
  }

  mutable std::mutex publication_gate_;
  std::atomic<navigation_world_model::WorldModelViewPtr> latest_{};
};

}  // namespace navigation_mapping
