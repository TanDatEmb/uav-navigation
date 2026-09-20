#pragma once

#include <cstdint>
#include <mutex>

namespace navigation_mapping {

// Exact lifecycle accounting for the single-latest mapping inbox. All state
// transitions and snapshots are serialized so diagnostics never observe a
// torn collection of counters while ROS callbacks replace a pending cloud.
class ObservationAccounting {
 public:
  struct Snapshot {
    std::uint64_t received{0};
    std::uint64_t rejected_before_inbox{0};
    std::uint64_t accepted_to_inbox{0};
    std::uint64_t replaced_waiting{0};
    std::uint64_t discarded_waiting{0};
    std::uint64_t discarded_nonmonotonic{0};
    std::uint64_t ready_submitted{0};
    std::uint64_t waiting{0};
    std::uint64_t replaced_ready{0};
    std::uint64_t discarded_ready{0};
    std::uint64_t discarded_shutdown_ready{0};
    std::uint64_t ready{0};
    // Compatibility totals retained for existing report consumers during the
    // transition to the product-owned mapping package.
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
             replaced_waiting + discarded_waiting + ready_submitted + waiting;
    }

    [[nodiscard]] bool readyInvariantHolds() const noexcept {
      return ready_submitted == replaced_ready + discarded_ready + discarded_shutdown_ready +
                                mapping_started + ready;
    }

    [[nodiscard]] bool mappingInvariantHolds() const noexcept {
      return mapping_started == mapping_published + mapping_failed + in_flight;
    }

    [[nodiscard]] bool allInvariantsHold() const noexcept {
      return inputInvariantHolds() && inboxInvariantHolds() && readyInvariantHolds() &&
             mappingInvariantHolds() &&
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
    const bool replaced = state_.waiting != 0U;
    if (replaced) {
      ++state_.replaced_waiting;
      ++state_.replaced_pending;
    } else {
      state_.waiting = 1U;
      state_.pending = 1U;
    }
    return replaced;
  }

  void discardedPending() {
    std::lock_guard lock(mutex_);
    if (state_.waiting == 0U) {
      ++state_.violation_count;
      return;
    }
    state_.waiting = 0U;
    state_.pending = state_.ready;
    ++state_.discarded_waiting;
    ++state_.discarded_pending;
  }

  void discardedNonmonotonicWaiting() {
    std::lock_guard lock(mutex_);
    if (state_.waiting == 0U) {
      ++state_.violation_count;
      return;
    }
    state_.waiting = 0U;
    state_.pending = state_.ready;
    ++state_.discarded_waiting;
    ++state_.discarded_nonmonotonic;
    ++state_.discarded_pending;
  }

  // Moves the assembler's WAITING_PAIR observation into the worker's READY
  // slot. `replaced_ready` is decided while holding the worker inbox lock.
  void waitingSubmitted(bool replaced_ready) {
    std::lock_guard lock(mutex_);
    if (state_.waiting == 0U) {
      ++state_.violation_count;
      return;
    }
    state_.waiting = 0U;
    ++state_.ready_submitted;
    if (replaced_ready) {
      if (state_.ready == 0U) {
        ++state_.violation_count;
        return;
      }
      ++state_.replaced_ready;
      ++state_.replaced_pending;
    } else {
      if (state_.ready != 0U) {
        ++state_.violation_count;
        return;
      }
      state_.ready = 1U;
    }
    state_.pending = state_.waiting + state_.ready;
  }

  void discardedShutdownReady() {
    std::lock_guard lock(mutex_);
    if (state_.ready == 0U) {
      ++state_.violation_count;
      return;
    }
    state_.ready = 0U;
    state_.pending = state_.waiting;
    ++state_.discarded_shutdown_ready;
    ++state_.discarded_pending;
  }

  void discardedReady() {
    std::lock_guard lock(mutex_);
    if (state_.ready == 0U) {
      ++state_.violation_count;
      return;
    }
    state_.ready = 0U;
    state_.pending = state_.waiting;
    ++state_.discarded_ready;
    ++state_.discarded_pending;
  }

  void mappingStarted() {
    std::lock_guard lock(mutex_);
    if (state_.ready == 0U) {
      ++state_.violation_count;
      return;
    }
    state_.ready = 0U;
    state_.pending = state_.waiting;
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

}  // namespace navigation_mapping
