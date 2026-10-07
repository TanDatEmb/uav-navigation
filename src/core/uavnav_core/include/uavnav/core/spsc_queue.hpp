#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <new>
#include <optional>
#include <stop_token>
#include <string_view>
#include <type_traits>
#include <utility>

#include "uavnav/core/result.hpp"

namespace uavnav::concurrency {

/// Why `SpscQueue::try_push` refused an item.
enum class QueueError : std::uint8_t { kFull };

constexpr std::string_view to_string(QueueError e) {
  switch (e) {
    case QueueError::kFull:
      return "FULL";
  }
  return "UNKNOWN";
}

/// Counters of a queue. `pushed + rejected` is every `try_push` call; `pushed - popped` is the
/// number of queued items. A snapshot read from any thread satisfies `popped <= pushed`.
struct QueueStats {
  std::uint64_t pushed;
  std::uint64_t popped;
  std::uint64_t rejected;
};

/// Bounded, lock-free single-producer/single-consumer FIFO (SYSTEM_DESIGN §3.5, D30/O14).
///
/// - A fixed ring of exactly `Capacity` usable slots inside the object: nothing is allocated after
///   construction.
/// - `try_push` (producer thread only) never blocks, never allocates and never throws. On a full
///   ring it returns `QueueError::kFull`, leaves the item unmoved and counts it in `rejected`.
///   The queue emits no event; the pushing side does, with its own reason.
/// - `try_pop` (consumer thread only) never blocks. `wait_nonempty` (consumer thread only)
///   blocks until the queue is non-empty or the stop token is requested.
/// - `stats()` may be called from any thread.
///
/// Memory ordering. `tail_` (written only by the producer) and `head_` (written only by the
/// consumer) are monotonically increasing item counts; slot = count % Capacity.
/// - Producer: constructs the item in slot `tail % Capacity`, then `tail_.store(release)`. The
///   consumer's `tail_.load(acquire)` therefore sees the fully constructed item.
/// - Consumer: moves the item out and destroys the slot, then `head_.store(release)`. The
///   producer's `head_.load(acquire)` therefore never reuses a slot the consumer still reads.
/// - Each side caches the other side's index and reloads it only when the cache says full/empty.
/// - `tail_` and `head_` (with their caches) live on separate cache lines, as do the wake counter
///   and the slots, so the two threads do not false-share.
///
/// Wakeups (`wait_nonempty`). `wake_seq_` is an event counter. A push does
/// `tail_.store(release)`, then `wake_seq_.fetch_add(release)`, then `wake_seq_.notify_one()`
/// (a futex wake when someone waits, nothing otherwise; never blocks). The waiter loops on:
/// `s = wake_seq_.load(acquire)`; return if non-empty or stop requested; `wake_seq_.wait(s)`.
/// No lost wakeup:
/// - If the waiter read `s` from the push's increment (or a later one), the acquire/release pair
///   makes the push's `tail_` store visible, so the emptiness check sees the item.
/// - Otherwise the increment follows `s` in `wake_seq_`'s modification order, so `wait(s)` either
///   sees a different value and returns, or is blocked and is unblocked by the following notify
///   ([atomics.wait]: X = the value `s`, Y = the increment, Y happens before the notify).
/// A `std::stop_callback` registered for the whole wait does the same increment + notify after
/// the stop state is set, so a stop request is covered by the same argument (`stop_requested()`
/// is read after `s`). If stop was already requested, the callback runs inline and the first
/// check returns.
template <class T, std::size_t Capacity>
class SpscQueue {
  static_assert(Capacity >= 1, "SpscQueue needs at least one slot");
  static_assert(std::is_nothrow_move_constructible_v<T>, "SpscQueue items must be nothrow-movable");
  static_assert(std::is_nothrow_destructible_v<T>, "SpscQueue items must be nothrow-destructible");

 public:
  SpscQueue() noexcept = default;
  SpscQueue(const SpscQueue&) = delete;
  SpscQueue& operator=(const SpscQueue&) = delete;
  SpscQueue(SpscQueue&&) = delete;
  SpscQueue& operator=(SpscQueue&&) = delete;

  /// Destroys the items still queued. No thread may use the queue any more.
  ~SpscQueue() {
    const std::uint64_t tail = tail_.load(std::memory_order_acquire);
    for (std::uint64_t i = head_.load(std::memory_order_acquire); i != tail; ++i) {
      std::destroy_at(slot(i));
    }
  }

  /// Producer thread only. Never blocks or allocates. On `kFull`, `item` is not moved from.
  Result<void, QueueError> try_push(T&& item) noexcept {
    const std::uint64_t tail = tail_.load(std::memory_order_relaxed);  // own index
    if (tail - head_cache_ >= Capacity) {
      head_cache_ = head_.load(std::memory_order_acquire);  // pairs with the consumer's release
      if (tail - head_cache_ >= Capacity) {
        rejected_.store(rejected_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
        return std::unexpected(QueueError::kFull);
      }
    }
    ::new (static_cast<void*>(slots_[tail % Capacity].bytes)) T(std::move(item));
    tail_.store(tail + 1, std::memory_order_release);  // publishes the slot
    wake_seq_.fetch_add(1, std::memory_order_release);
    wake_seq_.notify_one();
    return {};
  }

  /// Consumer thread only. Never blocks. Returns the oldest item, or nullopt when empty.
  std::optional<T> try_pop() noexcept {
    const std::uint64_t head = head_.load(std::memory_order_relaxed);  // own index
    if (head == tail_cache_) {
      tail_cache_ = tail_.load(std::memory_order_acquire);  // pairs with the producer's release
      if (head == tail_cache_) {
        return std::nullopt;
      }
    }
    T* const item = slot(head);
    std::optional<T> out(std::in_place, std::move(*item));
    std::destroy_at(item);
    head_.store(head + 1, std::memory_order_release);  // hands the slot back to the producer
    return out;
  }

  /// Consumer thread only. Returns once the queue is non-empty or `stop` is requested.
  void wait_nonempty(const std::stop_token& stop) const noexcept {
    const std::stop_callback wake_on_stop(stop, [this]() noexcept { bump_and_notify(); });
    for (;;) {
      const std::uint32_t seen = wake_seq_.load(std::memory_order_acquire);
      if (tail_.load(std::memory_order_acquire) != head_.load(std::memory_order_relaxed) ||
          stop.stop_requested()) {
        return;
      }
      wake_seq_.wait(seen, std::memory_order_acquire);
    }
  }

  /// Any thread. `head_` is read first with acquire, so the `tail_` read after it is at least
  /// the value the consumer had seen: `popped <= pushed` always holds in a snapshot.
  QueueStats stats() const noexcept {
    const std::uint64_t popped = head_.load(std::memory_order_acquire);
    const std::uint64_t pushed = tail_.load(std::memory_order_acquire);
    return QueueStats{pushed, popped, rejected_.load(std::memory_order_relaxed)};
  }

  static constexpr std::size_t capacity() noexcept { return Capacity; }

 private:
  // Fixed value instead of std::hardware_destructive_interference_size, whose use in a header
  // is an ABI warning on GCC (-Winterference-size). 64 bytes covers x86-64 and the Jetson ARM cores.
  static constexpr std::size_t kCacheLine = 64;

  struct alignas(T) Slot {
    std::byte bytes[sizeof(T)];
  };

  // Only for a slot that holds a live item (constructed by try_push, not yet destroyed).
  T* slot(std::uint64_t index) noexcept {
    return std::launder(reinterpret_cast<T*>(slots_[index % Capacity].bytes));
  }

  void bump_and_notify() const noexcept {
    wake_seq_.fetch_add(1, std::memory_order_release);
    wake_seq_.notify_one();
  }

  // Producer cache line: own index, cached consumer index, rejected count.
  alignas(kCacheLine) std::atomic<std::uint64_t> tail_{0};
  std::uint64_t head_cache_{0};
  std::atomic<std::uint64_t> rejected_{0};
  // Consumer cache line: own index, cached producer index.
  alignas(kCacheLine) std::atomic<std::uint64_t> head_{0};
  std::uint64_t tail_cache_{0};
  // Wake counter (32-bit: a native futex word on Linux). Mutable: the const wait registers a
  // stop callback that bumps it.
  alignas(kCacheLine) mutable std::atomic<std::uint32_t> wake_seq_{0};
  alignas(kCacheLine) Slot slots_[Capacity];
};

}  // namespace uavnav::concurrency
