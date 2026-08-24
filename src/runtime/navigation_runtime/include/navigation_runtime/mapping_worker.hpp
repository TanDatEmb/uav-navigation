#pragma once

#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <utility>

#include "navigation_runtime/observation_accounting.hpp"

namespace navigation_runtime {

// Bounded single-owner execution primitive for WM-3. The process callback is
// invoked only by worker_, so moving the mutable ROG owner into that callback
// gives it one mutation thread. The inbox is latest-only by design; every
// replacement and shutdown disposition is terminally accounted.
template <typename Observation>
class MappingWorker {
 public:
  using Process = std::function<void(Observation&&)>;
  using Validate = std::function<bool(const Observation&)>;
  using FatalHandler = std::function<void(std::exception_ptr)>;
  using OrderKey = std::function<std::int64_t(const Observation&)>;
  using PublishedHandler = std::function<void()>;

  MappingWorker(ObservationAccounting& accounting, Process process,
                FatalHandler fatal_handler = {}, Validate validate = {},
                PublishedHandler published_handler = {})
      : accounting_(accounting), process_(std::move(process)),
        fatal_handler_(std::move(fatal_handler)), validate_(std::move(validate)),
        published_handler_(std::move(published_handler)) {
    if (!process_) throw std::invalid_argument("MappingWorker process must not be empty");
  }

  void setStrictlyIncreasingOrderKey(OrderKey order_key,
                                     std::int64_t initial_key = 0) {
    std::lock_guard lock(mutex_);
    if (started_ || ready_) {
      throw std::logic_error("MappingWorker order key must be configured before start");
    }
    order_key_ = std::move(order_key);
    highest_admitted_key_ = initial_key;
  }

  MappingWorker(const MappingWorker&) = delete;
  MappingWorker& operator=(const MappingWorker&) = delete;
  ~MappingWorker() { shutdown(); }

  void start() {
    std::lock_guard lock(mutex_);
    if (started_ || shutdown_started_) {
      throw std::logic_error("MappingWorker start called in invalid lifecycle state");
    }
    started_ = true;
    worker_ = std::jthread([this](std::stop_token stop) { run(stop); });
  }

  // The caller owns one WAITING_PAIR lifecycle item. Returns false when
  // shutdown/fatal state discarded it instead of accepting it into READY.
  bool submitFromWaiting(Observation observation) {
    std::unique_lock lock(mutex_);
    if (!accepting_ || fatal_) {
      accounting_.discardedPending();
      return false;
    }
    if (validate_) {
      bool valid = false;
      try {
        valid = validate_(observation);
      } catch (...) {
        const auto failure = std::current_exception();
        accounting_.discardedPending();
        fatal_ = true;
        accepting_ = false;
        failure_ = failure;
        lock.unlock();
        if (fatal_handler_) fatal_handler_(failure);
        return false;
      }
      if (!valid) {
        accounting_.discardedPending();
        return false;
      }
    }
    if (order_key_) {
      std::int64_t key = 0;
      try {
        key = order_key_(observation);
      } catch (...) {
        const auto failure = std::current_exception();
        accounting_.discardedPending();
        fatal_ = true;
        accepting_ = false;
        failure_ = failure;
        lock.unlock();
        if (fatal_handler_) fatal_handler_(failure);
        return false;
      }
      if (key <= highest_admitted_key_) {
        accounting_.discardedNonmonotonicWaiting();
        return false;
      }
      highest_admitted_key_ = key;
    }
    const bool replaced = ready_.has_value();
    accounting_.waitingSubmitted(replaced);
    ready_ = std::move(observation);
    cv_.notify_one();
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
      if (!started_ && ready_) {
        ready_.reset();
        accounting_.discardedShutdownReady();
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

  [[nodiscard]] bool fatal() const noexcept {
    std::lock_guard lock(mutex_);
    return fatal_;
  }

  [[nodiscard]] bool stopping() const noexcept {
    std::lock_guard lock(mutex_);
    return shutdown_started_;
  }

  [[nodiscard]] std::exception_ptr failure() const noexcept {
    std::lock_guard lock(mutex_);
    return failure_;
  }

 private:
  void run(std::stop_token stop) noexcept {
    for (;;) {
      std::optional<Observation> observation;
      {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, stop, [this] { return ready_.has_value(); });
        if (stop.stop_requested()) {
          if (ready_) {
            ready_.reset();
            accounting_.discardedShutdownReady();
          }
          return;
        }
        if (validate_) {
          bool valid = false;
          try {
            valid = validate_(*ready_);
          } catch (...) {
            const auto failure = std::current_exception();
            ready_.reset();
            accounting_.discardedReady();
            fatal_ = true;
            accepting_ = false;
            failure_ = failure;
            lock.unlock();
            if (fatal_handler_) fatal_handler_(failure);
            return;
          }
          if (!valid) {
            ready_.reset();
            accounting_.discardedReady();
            continue;
          }
        }
        observation = std::move(ready_);
        ready_.reset();
        accounting_.mappingStarted();
      }
      try {
        process_(std::move(*observation));
        accounting_.mappingPublished();
      } catch (...) {
        accounting_.mappingFailed();
        std::exception_ptr failure = std::current_exception();
        {
          std::lock_guard lock(mutex_);
          fatal_ = true;
          accepting_ = false;
          failure_ = failure;
          if (ready_) {
            ready_.reset();
            accounting_.discardedShutdownReady();
          }
        }
        if (fatal_handler_) fatal_handler_(failure);
        return;
      }
      if (published_handler_) {
        try {
          published_handler_();
        } catch (...) {
          const auto failure = std::current_exception();
          {
            std::lock_guard lock(mutex_);
            fatal_ = true;
            accepting_ = false;
            failure_ = failure;
            if (ready_) {
              ready_.reset();
              accounting_.discardedShutdownReady();
            }
          }
          // The world revision is already published and remains terminally
          // accounted as PUBLISHED. A diagnostics failure must never relabel
          // that observation as a mutable-map failure.
          if (fatal_handler_) fatal_handler_(failure);
          return;
        }
      }
    }
  }

  ObservationAccounting& accounting_;
  Process process_;
  FatalHandler fatal_handler_;
  Validate validate_;
  PublishedHandler published_handler_;
  OrderKey order_key_;
  mutable std::mutex mutex_;
  std::condition_variable_any cv_;
  std::condition_variable shutdown_cv_;
  std::optional<Observation> ready_;
  bool accepting_{true};
  bool started_{false};
  bool fatal_{false};
  bool shutdown_started_{false};
  bool joined_{false};
  std::exception_ptr failure_;
  std::int64_t highest_admitted_key_{0};
  std::jthread worker_;
};

}  // namespace navigation_runtime
