#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_runtime {

struct PinnedWorldSnapshot {
  navigation_world_model::WorldModelViewPtr view;
  navigation_world_model::WorldSnapshotIdentity identity;

  [[nodiscard]] explicit operator bool() const noexcept {
    return static_cast<bool>(view);
  }
};

enum class WorldCommitDecision {
  kCommitted,
  kNoPublishedWorld,
  kWorldAdvanced,
  kCommitRejected,
};

// Product-owned publication boundary between the sole mutable mapping owner
// and planners that pin immutable views. Snapshot construction and candidate
// validation happen outside publication_gate_; only publication and the final
// authorized commit are linearized here.
class WorldSnapshotStore {
 public:
  WorldSnapshotStore() = default;
  WorldSnapshotStore(const WorldSnapshotStore&) = delete;
  WorldSnapshotStore& operator=(const WorldSnapshotStore&) = delete;

  [[nodiscard]] PinnedWorldSnapshot load() const noexcept {
    auto view = latest_.load(std::memory_order_acquire);
    return {view, view ? view->identity()
                       : navigation_world_model::WorldSnapshotIdentity{}};
  }

  void publish(navigation_world_model::WorldModelViewPtr next) {
    if (!next) throw std::invalid_argument("cannot publish a null WorldModel snapshot");
    const auto next_identity = next->identity();
    if (next_identity.generation == 0U || next_identity.observation_stamp_ns < 0) {
      throw std::invalid_argument("cannot publish an invalid WorldModel identity");
    }
    std::lock_guard<std::mutex> guard(publication_gate_);
    const auto current = latest_.load(std::memory_order_relaxed);
    if (current && !strictlyAdvances(current->identity(), next_identity)) {
      throw std::logic_error("WorldModel publication identity is not monotonic");
    }
    latest_.store(std::move(next), std::memory_order_release);
  }

  template <typename CommitFunction>
  WorldCommitDecision authorize(
      const navigation_world_model::WorldSnapshotIdentity& validated_identity,
      CommitFunction&& commit) {
    std::lock_guard<std::mutex> guard(publication_gate_);
    const auto latest = latest_.load(std::memory_order_acquire);
    if (!latest) return WorldCommitDecision::kNoPublishedWorld;
    if (!sameIdentity(latest->identity(), validated_identity)) {
      return WorldCommitDecision::kWorldAdvanced;
    }
    return std::invoke(std::forward<CommitFunction>(commit))
        ? WorldCommitDecision::kCommitted
        : WorldCommitDecision::kCommitRejected;
  }

  [[nodiscard]] static bool sameIdentity(
      const navigation_world_model::WorldSnapshotIdentity& lhs,
      const navigation_world_model::WorldSnapshotIdentity& rhs) noexcept {
    return lhs.generation == rhs.generation && lhs.revision == rhs.revision &&
           lhs.observation_stamp_ns == rhs.observation_stamp_ns;
  }

 private:
  [[nodiscard]] static bool strictlyAdvances(
      const navigation_world_model::WorldSnapshotIdentity& current,
      const navigation_world_model::WorldSnapshotIdentity& next) noexcept {
    if (next.generation != current.generation) {
      return next.generation > current.generation;
    }
    return next.revision > current.revision &&
           next.observation_stamp_ns >= current.observation_stamp_ns;
  }

  mutable std::mutex publication_gate_;
  std::atomic<navigation_world_model::WorldModelViewPtr> latest_{};
};

}  // namespace navigation_runtime
