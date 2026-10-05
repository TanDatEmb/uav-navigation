#pragma once

#include <memory>
#include <mutex>

#include "navigation_execution/execution_state_lease.hpp"

namespace navigation_execution {

// The latest propagated state is one immutable, epoch-tagged lease. A state
// from an older localization epoch can never replace or survive a reset.
class ExecutionStateStore final {
 public:
  ExecutionStateStore() = default;
  ExecutionStateStore(const ExecutionStateStore&) = delete;
  ExecutionStateStore& operator=(const ExecutionStateStore&) = delete;

  bool publish(navigation_planning::KinematicState state) noexcept {
    if (!state.finite()) return false;
    std::lock_guard lock(mutex_);
    if (active_epoch_ != 0 && state.localization_epoch != active_epoch_) return false;
    if (state_ && state.source_stamp_ns <= state_->state.source_stamp_ns) return false;
    active_epoch_ = state.localization_epoch;
    const auto ingress_sequence = ++next_ingress_sequence_;
    state_ = std::make_shared<const ExecutionStateLease>(
        ExecutionStateLease{std::move(state), ingress_sequence});
    return true;
  }

  bool resetForLocalizationEpoch(std::uint64_t localization_epoch) noexcept {
    if (localization_epoch == 0U) return false;
    std::lock_guard lock(mutex_);
    active_epoch_ = localization_epoch;
    state_.reset();
    return true;
  }

  [[nodiscard]] std::shared_ptr<const ExecutionStateLease> load() const noexcept {
    std::lock_guard lock(mutex_);
    return state_;
  }

 private:
  mutable std::mutex mutex_;
  std::uint64_t active_epoch_{0};
  std::uint64_t next_ingress_sequence_{0};
  std::shared_ptr<const ExecutionStateLease> state_;
};

}  // namespace navigation_execution
