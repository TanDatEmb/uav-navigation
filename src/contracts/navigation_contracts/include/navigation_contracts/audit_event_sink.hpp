#pragma once

// Removable experiment transport. This header is included only by ON builds.
// Producers never publish, allocate, wait for the consumer or throw.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string_view>
#include <unistd.h>

#include <navigation_contracts/msg/audit_event.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navigation_contracts::audit {

using Event = navigation_contracts::msg::AuditEvent;

inline std::uint64_t missionHash(std::string_view id) noexcept {
  std::uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char c : id) {
    hash = (hash ^ c) * 1099511628211ULL;
  }
  return hash;
}

inline std::int64_t steadyNowNs() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

template <class F>
class ScopeExit final {
 public:
  explicit ScopeExit(F f) noexcept : f_(f) {}
  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;
  ~ScopeExit() noexcept { try { f_(); } catch (...) {} }
 private:
  F f_;
};

class Sink final {
 public:
  static constexpr std::size_t kCapacity = 1024;
  static constexpr std::size_t kDrainLimit = 64;

  Sink(rclcpp::Node& node, std::uint8_t producer_id)
      : node_(node), producer_id_(producer_id),
        incarnation_(static_cast<std::uint64_t>(steadyNowNs()) ^
                     (static_cast<std::uint64_t>(::getpid()) << 32)),
        records_(std::make_unique<std::array<Event, kCapacity>>()) {
    publisher_ = node_.create_publisher<Event>(
        "/navigation/audit_event", rclcpp::QoS{rclcpp::KeepLast{64}}.best_effort());
    timer_ = node_.create_wall_timer(std::chrono::milliseconds{20},
                                     [this]() { drain(); });
  }

  Sink(const Sink&) = delete;
  Sink& operator=(const Sink&) = delete;

  ~Sink() noexcept {
    try {
      if (timer_) timer_->cancel();
      for (std::size_t i = 0; i < (kCapacity / kDrainLimit) + 1; ++i) drain();
    } catch (...) {}
  }

  struct Stats {
    std::uint64_t enqueued;
    std::uint64_t dropped;
    std::uint64_t published;
    std::uint32_t max_occupancy;
  };

  [[nodiscard]] Stats stats() const noexcept {
    return {enqueued_.load(std::memory_order_relaxed),
            dropped_.load(std::memory_order_relaxed),
            published_.load(std::memory_order_relaxed),
            max_occupancy_.load(std::memory_order_relaxed)};
  }

  void emit(Event event) noexcept {
    const auto start = steadyNowNs();
    event.producer_id = producer_id_;
    event.process_incarnation = incarnation_;
    event.diagnostic_sequence = sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    if (event.steady_ns == 0) event.steady_ns = start;
    if (!mutex_.try_lock()) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (size_ == kCapacity) {
      mutex_.unlock();
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    (*records_)[tail_] = event;
    (*records_)[tail_].producer_enqueue_ns = steadyNowNs() - start;
    tail_ = (tail_ + 1) % kCapacity;
    ++size_;
    const auto previous = max_occupancy_.load(std::memory_order_relaxed);
    if (size_ > previous) max_occupancy_.store(size_, std::memory_order_relaxed);
    mutex_.unlock();
    enqueued_.fetch_add(1, std::memory_order_relaxed);
  }

  void drain() noexcept {
    std::array<Event, kDrainLimit> batch;
    std::size_t count = 0;
    if (mutex_.try_lock()) {
      while (count < batch.size() && size_ > 0) {
        batch[count++] = (*records_)[head_];
        head_ = (head_ + 1) % kCapacity;
        --size_;
      }
      mutex_.unlock();
    }
    for (std::size_t i = 0; i < count; ++i) {
      try {
        const auto start = steadyNowNs();
        publisher_->publish(batch[i]);
        const auto cost = steadyNowNs() - start;
        last_consumer_publish_ns_.store(cost, std::memory_order_relaxed);
        publish_costs_[publish_cost_next_ % publish_costs_.size()] = cost;
        ++publish_cost_next_;
        published_.fetch_add(1, std::memory_order_relaxed);
      } catch (...) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
      }
    }
    const auto now = steadyNowNs();
    if (now - last_accounting_ns_ >= 1'000'000'000LL) {
      last_accounting_ns_ = now;
      Event accounting{};
      accounting.event_type = Event::QUEUE_ACCOUNTING;
      accounting.producer_id = producer_id_;
      accounting.process_incarnation = incarnation_;
      accounting.diagnostic_sequence = sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
      accounting.steady_ns = now;
      accounting.enqueued_count = enqueued_.load(std::memory_order_relaxed);
      accounting.dropped_count = dropped_.load(std::memory_order_relaxed);
      accounting.published_count = published_.load(std::memory_order_relaxed);
      accounting.max_queue_occupancy = max_occupancy_.load(std::memory_order_relaxed);
      accounting.consumer_publish_ns = last_consumer_publish_ns_.load(std::memory_order_relaxed);
      const auto sample_count = std::min(publish_cost_next_, publish_costs_.size());
      accounting.consumer_publish_samples = static_cast<std::uint32_t>(sample_count);
      if (sample_count > 0) {
        auto sorted_costs = publish_costs_;
        std::sort(sorted_costs.begin(), sorted_costs.begin() + sample_count);
        const auto percentile = [&](const std::size_t numerator) {
          const auto index = (sample_count * numerator + 99) / 100 - 1;
          return sorted_costs[index];
        };
        accounting.consumer_publish_p50_ns = percentile(50);
        accounting.consumer_publish_p95_ns = percentile(95);
        accounting.consumer_publish_p99_ns = percentile(99);
        accounting.consumer_publish_max_ns = sorted_costs[sample_count - 1];
      }
      publish_cost_next_ = 0;
      try {
        publisher_->publish(accounting);
        published_.fetch_add(1, std::memory_order_relaxed);
      } catch (...) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

 private:
  rclcpp::Node& node_;
  const std::uint8_t producer_id_;
  const std::uint64_t incarnation_;
  std::unique_ptr<std::array<Event, kCapacity>> records_;
  std::mutex mutex_;
  std::size_t head_{0};
  std::size_t tail_{0};
  std::size_t size_{0};
  std::atomic<std::uint64_t> sequence_{0};
  std::atomic<std::uint64_t> enqueued_{0};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<std::uint64_t> published_{0};
  std::atomic<std::uint32_t> max_occupancy_{0};
  std::atomic<std::int64_t> last_consumer_publish_ns_{0};
  std::array<std::int64_t, 4096> publish_costs_{};
  std::size_t publish_cost_next_{0};
  std::int64_t last_accounting_ns_{0};
  rclcpp::Publisher<Event>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigation_contracts::audit
