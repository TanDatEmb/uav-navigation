#pragma once

#include <cstdint>
#include <mutex>

namespace navigation_runtime {

// Exact lifecycle accounting for the single-latest mapping inbox. All state
// transitions and snapshots are serialized so diagnostics never observe a
// torn collection of counters while ROS callbacks replace a pending cloud.
class ObservationAccounting {
 public:
  struct Snapshot {
    std::uint64_t received{0};
    std::uint64_t rejected_before_inbox{0};
    std::uint64_t accepted_to_inbox{0};
    std::uint64_t replaced_pending{0};
    std::uint64_t discarded_pending{0};
    std::uint64_t mapping_started{0};
    std::uint64_t mapping_published{0};
    std::uint64_t mapping_failed{0};
    std::uint64_t pending{0};
    std::uint64_t in_flight{0};
    std::uint64_t violation_count{0};

    [[nodiscard]] bool inputInvariantHolds() const noexcept {
      return received == rejected_before_inbox + accepted_to_inbox;
    }

    [[nodiscard]] bool inboxInvariantHolds() const noexcept {
      return accepted_to_inbox ==
             replaced_pending + discarded_pending + mapping_started + pending;
    }

    [[nodiscard]] bool mappingInvariantHolds() const noexcept {
      return mapping_started == mapping_published + mapping_failed + in_flight;
    }

    [[nodiscard]] bool allInvariantsHold() const noexcept {
      return inputInvariantHolds() && inboxInvariantHolds() && mappingInvariantHolds() &&
             violation_count == 0U;
    }
  };

  void recordRejectedBeforeInbox() {
    std::lock_guard lock(mutex_);
    ++state_.received;
    ++state_.rejected_before_inbox;
  }

  // Returns true when accepting this observation replaced a pending one.
  bool recordAcceptedToInbox() {
    std::lock_guard lock(mutex_);
    ++state_.received;
    ++state_.accepted_to_inbox;
    const bool replaced = state_.pending != 0U;
    if (replaced) {
      ++state_.replaced_pending;
    } else {
      state_.pending = 1U;
    }
    return replaced;
  }

  void discardedPending() {
    std::lock_guard lock(mutex_);
    if (state_.pending == 0U) {
      ++state_.violation_count;
      return;
    }
    state_.pending = 0U;
    ++state_.discarded_pending;
  }

  void mappingStarted() {
    std::lock_guard lock(mutex_);
    if (state_.pending == 0U) {
      ++state_.violation_count;
      return;
    }
    state_.pending = 0U;
    ++state_.mapping_started;
    ++state_.in_flight;
  }

  void mappingPublished() {
    std::lock_guard lock(mutex_);
    if (state_.in_flight == 0U) {
      ++state_.violation_count;
      return;
    }
    --state_.in_flight;
    ++state_.mapping_published;
  }

  void mappingFailed() {
    std::lock_guard lock(mutex_);
    if (state_.in_flight == 0U) {
      ++state_.violation_count;
      return;
    }
    --state_.in_flight;
    ++state_.mapping_failed;
  }

  [[nodiscard]] Snapshot snapshot() const {
    std::lock_guard lock(mutex_);
    return state_;
  }

 private:
  mutable std::mutex mutex_;
  Snapshot state_;
};

}  // namespace navigation_runtime
