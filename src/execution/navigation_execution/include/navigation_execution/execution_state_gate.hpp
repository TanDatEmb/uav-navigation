#pragma once

#include <atomic>
#include <mutex>

namespace navigation_execution {

class ExecutionStateFailureLatch final {
 public:
  bool tryLatch() noexcept {
    bool expected = false;
    return latched_.compare_exchange_strong(expected, true);
  }

  void resetForNewGoal() noexcept {
    std::lock_guard<std::mutex> lock(transition_mutex_);
    resetForNewGoalWithinTransition();
  }
  void resetForNewGoalWithinTransition() noexcept { latched_.store(false); }
  bool latched() const noexcept { return latched_.load(); }
  bool allowsCommandExposure() const noexcept { return !latched(); }
  std::mutex& transitionMutex() noexcept { return transition_mutex_; }

 private:
  std::atomic_bool latched_{false};
  std::mutex transition_mutex_;
};

}  // namespace navigation_execution
