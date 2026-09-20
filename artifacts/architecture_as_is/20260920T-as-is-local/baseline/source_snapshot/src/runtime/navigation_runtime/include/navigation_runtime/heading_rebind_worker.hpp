#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>

namespace navigation_runtime {

struct HeadingRebindWorkerSnapshot {
  std::uint64_t submitted{0};
  std::uint64_t started{0};
  std::uint64_t completed{0};
  std::uint64_t replaced_pending{0};
  bool in_flight{false};
  bool pending{false};
};

// The heading path is intentionally a separate, latest-only worker. Its job
// may read immutable execution/planner snapshots, but it must never call a
// mutable optimizer operation or mutate planner warm-start history. The
// resulting immutable candidate is handed back to the command-clock boundary.
class HeadingRebindWorker final {
 public:
  using Job = std::function<void(std::stop_token)>;

  HeadingRebindWorker() = default;
  HeadingRebindWorker(const HeadingRebindWorker&) = delete;
  HeadingRebindWorker& operator=(const HeadingRebindWorker&) = delete;
  ~HeadingRebindWorker() { shutdown(); }

  void start() {
    std::lock_guard lock(mutex_);
    if (started_ || stopping_) return;
    started_ = true;
    worker_ = std::jthread([this](std::stop_token stop) { run(stop); });
  }

  bool submit(Job job) {
    if (!job) return false;
    {
      std::lock_guard lock(mutex_);
      if (!started_ || stopping_) return false;
      if (pending_) ++snapshot_.replaced_pending;
      pending_ = std::move(job);
      ++snapshot_.submitted;
      snapshot_.pending = true;
    }
    condition_.notify_one();
    return true;
  }

  void shutdown() noexcept {
    {
      std::unique_lock lock(mutex_);
      if (stopping_) {
        shutdown_condition_.wait(lock, [this] { return joined_; });
        return;
      }
      stopping_ = true;
      pending_.reset();
      snapshot_.pending = false;
    }
    worker_.request_stop();
    condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    {
      std::lock_guard lock(mutex_);
      joined_ = true;
    }
    shutdown_condition_.notify_all();
  }

  [[nodiscard]] HeadingRebindWorkerSnapshot snapshot() const noexcept {
    std::lock_guard lock(mutex_);
    auto result = snapshot_;
    result.pending = pending_.has_value();
    return result;
  }

 private:
  void run(std::stop_token stop) noexcept {
    for (;;) {
      std::optional<Job> job;
      {
        std::unique_lock lock(mutex_);
        condition_.wait(lock, stop, [this] { return pending_.has_value(); });
        if (stop.stop_requested()) return;
        job = std::move(pending_);
        pending_.reset();
        snapshot_.pending = false;
        snapshot_.in_flight = true;
        ++snapshot_.started;
      }
      try {
        (*job)(stop);
      } catch (...) {
        // A failed out-of-band heading attempt never owns a fallback. The
        // active execution bundle remains authoritative and the next command
        // tick may submit a fresh immutable request.
      }
      {
        std::lock_guard lock(mutex_);
        snapshot_.in_flight = false;
        ++snapshot_.completed;
      }
    }
  }

  mutable std::mutex mutex_;
  std::condition_variable_any condition_;
  std::condition_variable shutdown_condition_;
  std::optional<Job> pending_;
  HeadingRebindWorkerSnapshot snapshot_{};
  bool started_{false};
  bool stopping_{false};
  bool joined_{false};
  std::jthread worker_;
};

}  // namespace navigation_runtime
