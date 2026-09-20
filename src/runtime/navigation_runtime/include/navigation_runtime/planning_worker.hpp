#pragma once

#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <utility>

#include "navigation_runtime/planning_supervisor.hpp"

namespace navigation_runtime {

enum class PlanningSubmitDisposition : std::uint8_t {
  kAccepted,
  kReplacedPending,
  kExactDuplicate,
  kRejectedLowerPriority,
  kRejectedInvalid,
  kRejectedStopped,
};

struct PlanningWorkerSnapshot {
  std::uint64_t submitted{0};
  std::uint64_t started{0};
  std::uint64_t completed{0};
  std::uint64_t cancelled{0};
  std::uint64_t exact_duplicates{0};
  std::uint64_t replaced_pending{0};
  std::uint64_t rejected_lower_priority{0};
  std::uint64_t accepted{0};
  std::uint64_t rejected_invalid{0};
  std::uint64_t rejected_stopped{0};
  // Monotonic timing witnesses for the latest worker transaction. These are
  // diagnostic only and do not participate in queue admission or cancellation.
  std::int64_t last_enqueue_wait_us{0};
  std::int64_t last_worker_runtime_us{0};
  std::int64_t last_enqueue_time_steady_ns{0};
  std::int64_t last_worker_start_steady_ns{0};
  std::int64_t last_backend_entry_steady_ns{0};
  std::int64_t last_backend_exit_steady_ns{0};
  // Identity of the latest worker transaction. Timing fields above are
  // attributable to a planner cycle only when this tuple matches its key.
  std::uint64_t last_transaction_localization_epoch{0};
  std::uint64_t last_transaction_goal_epoch{0};
  std::uint64_t last_transaction_request_id{0};
  std::uint64_t last_transaction_route_revision{0};
  std::uint64_t last_transaction_committed_bundle_generation{0};
  std::uint64_t last_transaction_pinned_world_generation{0};
  std::uint64_t last_transaction_pinned_world_revision{0};
  std::uint64_t last_transaction_anchor_stamp_ns{0};
  std::uint64_t last_transaction_dynamics_hash{0};
  std::uint64_t last_submit_disposition{0};
  bool in_flight{false};
  bool pending{false};
  bool fatal{false};
};

// A bounded, single-owner planning executor. There is exactly one mutable
// planner instance, one active job, and one latest pending job. Planner is a
// template parameter so concurrency semantics can be tested without loading
// the production backend.
template <typename Planner>
class PlanningWorker {
 public:
  using Job = std::function<void(Planner&, std::stop_token)>;
  using FatalHandler = std::function<void(std::exception_ptr)>;

  PlanningWorker(std::unique_ptr<Planner> planner, FatalHandler fatal_handler = {})
      : planner_(std::move(planner)), fatal_handler_(std::move(fatal_handler)) {
    if (!planner_) throw std::invalid_argument("PlanningWorker planner must not be null");
  }

  PlanningWorker(const PlanningWorker&) = delete;
  PlanningWorker& operator=(const PlanningWorker&) = delete;
  ~PlanningWorker() { shutdown(); }

  void start() {
    std::lock_guard lock(mutex_);
    if (started_ || shutdown_started_) {
      throw std::logic_error("PlanningWorker start called in invalid lifecycle state");
    }
    started_ = true;
    worker_ = std::jthread([this](std::stop_token stop) { run(stop); });
  }

  [[nodiscard]] PlanningSubmitDisposition submit(
      PlanningKey key, PlanningPriority priority, Job job) {
    if (!key.valid() || !planningPriorityKnown(priority) || !job) {
      std::lock_guard lock(mutex_);
      ++snapshot_.rejected_invalid;
      snapshot_.last_submit_disposition = static_cast<std::uint64_t>(
          PlanningSubmitDisposition::kRejectedInvalid);
      return PlanningSubmitDisposition::kRejectedInvalid;
    }

    PlanningSubmitDisposition disposition = PlanningSubmitDisposition::kAccepted;
    {
      std::lock_guard lock(mutex_);
      if (!accepting_ || fatal_ || shutdown_started_) {
        ++snapshot_.rejected_stopped;
        snapshot_.last_submit_disposition = static_cast<std::uint64_t>(
            PlanningSubmitDisposition::kRejectedStopped);
        return PlanningSubmitDisposition::kRejectedStopped;
      }
      if ((active_ && active_->key == key) || (pending_ && pending_->key == key)) {
        ++snapshot_.exact_duplicates;
        snapshot_.last_submit_disposition = static_cast<std::uint64_t>(
            PlanningSubmitDisposition::kExactDuplicate);
        return PlanningSubmitDisposition::kExactDuplicate;
      }

      const bool supersedes_active = active_ &&
          (!samePlanningCancellationIdentity(active_->key, key) ||
           higherPriority(priority, active_->priority));
      if (supersedes_active) {
        if (active_->stop_source.request_stop()) {
          ++snapshot_.cancelled;
        }
      }

      const auto incumbent_priority = pending_
          ? pending_->priority
          : (active_ ? active_->priority : PlanningPriority::kQualityRefinement);
      if (!supersedes_active && (pending_ || active_) &&
          higherPriority(incumbent_priority, priority)) {
        ++snapshot_.rejected_lower_priority;
        snapshot_.last_submit_disposition = static_cast<std::uint64_t>(
            PlanningSubmitDisposition::kRejectedLowerPriority);
        return PlanningSubmitDisposition::kRejectedLowerPriority;
      }

      if (pending_) {
        ++snapshot_.replaced_pending;
        disposition = PlanningSubmitDisposition::kReplacedPending;
      }
      pending_ = WorkItem{std::move(key), priority, std::move(job),
                          std::chrono::steady_clock::now()};
      ++snapshot_.submitted;
      ++snapshot_.accepted;
      snapshot_.last_submit_disposition = static_cast<std::uint64_t>(disposition);
      snapshot_.last_enqueue_time_steady_ns = std::chrono::duration_cast<
          std::chrono::nanoseconds>(pending_->enqueued_at.time_since_epoch()).count();
      snapshot_.last_transaction_localization_epoch =
          pending_->key.localization_epoch;
      snapshot_.last_transaction_goal_epoch = pending_->key.goal_epoch;
      snapshot_.last_transaction_request_id = pending_->key.request_id;
      snapshot_.last_transaction_route_revision = pending_->key.route_revision;
      snapshot_.last_transaction_committed_bundle_generation =
          pending_->key.committed_bundle_generation;
      snapshot_.last_transaction_pinned_world_generation =
          pending_->key.pinned_world_generation;
      snapshot_.last_transaction_pinned_world_revision =
          pending_->key.pinned_world_revision;
      snapshot_.last_transaction_anchor_stamp_ns =
          static_cast<std::uint64_t>(pending_->key.anchor_stamp_ns);
      snapshot_.last_transaction_dynamics_hash = pending_->key.dynamics_hash;
      snapshot_.pending = true;
    }
    cv_.notify_one();
    return disposition;
  }

  // Cancellation is scoped to this worker item and observed through its stop
  // token. The planner never receives an ambient cross-job interrupt.
  void cancelActive() noexcept {
    std::lock_guard lock(mutex_);
    if (active_) {
      if (active_->stop_source.request_stop()) ++snapshot_.cancelled;
    }
  }

  // A command-sampler callback may observe the terminal sample of an older
  // execution bundle while a newer desired goal is already being solved.  A
  // bare cancelActive() would then interrupt that newer solve.  Match the
  // execution ownership tuple while the caller holds its lifecycle
  // transaction; only the worker item carrying the completed command's
  // identity may have its scoped stop token requested.
  bool cancelActiveIfExecutionIdentity(
      const std::uint64_t localization_epoch,
      const std::uint64_t goal_epoch,
      const std::uint64_t request_id,
      const std::uint64_t committed_bundle_generation) noexcept {
    if (localization_epoch == 0U || goal_epoch == 0U || request_id == 0U ||
        committed_bundle_generation == 0U) {
      return false;
    }
    std::lock_guard lock(mutex_);
    if (!active_ || active_->key.localization_epoch != localization_epoch ||
        active_->key.goal_epoch != goal_epoch ||
        active_->key.request_id != request_id ||
        active_->key.committed_bundle_generation !=
            committed_bundle_generation) {
      return false;
    }
    if (active_->stop_source.request_stop()) ++snapshot_.cancelled;
    return true;
  }

  void shutdown() noexcept {
    {
      std::unique_lock lock(mutex_);
      if (shutdown_started_) {
        shutdown_cv_.wait(lock, [this] { return joined_; });
        return;
      }
      shutdown_started_ = true;
      accepting_ = false;
      pending_.reset();
      snapshot_.pending = false;
      if (active_) {
        if (active_->stop_source.request_stop()) ++snapshot_.cancelled;
      }
    }
    worker_.request_stop();
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    {
      std::lock_guard lock(mutex_);
      joined_ = true;
    }
    shutdown_cv_.notify_all();
  }

  [[nodiscard]] PlanningWorkerSnapshot snapshot() const noexcept {
    std::lock_guard lock(mutex_);
    auto result = snapshot_;
    result.in_flight = active_.has_value();
    result.pending = pending_.has_value();
    result.fatal = fatal_;
    return result;
  }

 private:
  struct WorkItem {
    PlanningKey key;
    PlanningPriority priority{PlanningPriority::kNormalRenewal};
    Job job;
    std::chrono::steady_clock::time_point enqueued_at;
  };
  struct ActiveItem {
    PlanningKey key;
    PlanningPriority priority{PlanningPriority::kNormalRenewal};
    std::stop_source stop_source;
  };

  void run(std::stop_token worker_stop) noexcept {
    for (;;) {
      std::optional<WorkItem> work;
      std::stop_token job_stop;
      std::chrono::steady_clock::time_point worker_started_at;
      {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, worker_stop, [this] { return pending_.has_value(); });
        if (worker_stop.stop_requested()) return;
        work = std::move(pending_);
        pending_.reset();
        worker_started_at = std::chrono::steady_clock::now();
        snapshot_.last_worker_start_steady_ns = std::chrono::duration_cast<
            std::chrono::nanoseconds>(worker_started_at.time_since_epoch()).count();
        snapshot_.last_enqueue_wait_us = std::chrono::duration_cast<
            std::chrono::microseconds>(worker_started_at - work->enqueued_at).count();
        active_.emplace();
        active_->key = work->key;
        active_->priority = work->priority;
        job_stop = active_->stop_source.get_token();
        snapshot_.pending = false;
        snapshot_.in_flight = true;
        ++snapshot_.started;
      }

      try {
        {
          std::lock_guard lock(mutex_);
          snapshot_.last_backend_entry_steady_ns = std::chrono::duration_cast<
              std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }
        std::lock_guard<std::recursive_mutex> backend_lock(backend_access_mutex_);
        work->job(*planner_, job_stop);
        {
          std::lock_guard lock(mutex_);
          snapshot_.last_backend_exit_steady_ns = std::chrono::duration_cast<
              std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }
      } catch (...) {
        const auto failure = std::current_exception();
        {
          std::lock_guard lock(mutex_);
          failure_ = failure;
          fatal_ = true;
          accepting_ = false;
          active_.reset();
          pending_.reset();
          snapshot_.in_flight = false;
          snapshot_.pending = false;
        }
        cv_.notify_all();
        if (fatal_handler_) fatal_handler_(failure);
        return;
      }

      {
        std::lock_guard lock(mutex_);
        active_.reset();
        snapshot_.last_worker_runtime_us = std::chrono::duration_cast<
            std::chrono::microseconds>(std::chrono::steady_clock::now() -
                                       worker_started_at).count();
        snapshot_.in_flight = false;
        ++snapshot_.completed;
      }
      cv_.notify_all();
    }
  }

  std::unique_ptr<Planner> planner_;
  FatalHandler fatal_handler_;
  mutable std::mutex mutex_;
  std::recursive_mutex backend_access_mutex_;
  std::condition_variable_any cv_;
  std::condition_variable shutdown_cv_;
  std::optional<WorkItem> pending_;
  std::optional<ActiveItem> active_;
  PlanningWorkerSnapshot snapshot_;
  bool accepting_{true};
  bool started_{false};
  bool fatal_{false};
  bool shutdown_started_{false};
  bool joined_{false};
  std::exception_ptr failure_;
  std::jthread worker_;
};

}  // namespace navigation_runtime
