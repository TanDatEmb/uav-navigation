#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace navigation_common {

// A fixed-capacity single-producer/single-consumer queue for diagnostic
// handoff.  The producer never waits for the recorder: a full queue is an
// explicit evidence loss that the owner must account for.  The queue has no
// ROS or control ownership and is safe only for one producer and one consumer.
template <typename T, std::size_t Capacity>
class BoundedSpscQueue final {
  static_assert(Capacity > 0U, "BoundedSpscQueue capacity must be positive");
  static_assert(std::is_default_constructible_v<T>,
                "BoundedSpscQueue values must be default constructible");
  static_assert(std::is_copy_assignable_v<T>,
                "BoundedSpscQueue values must be copy assignable");
  static_assert(std::is_nothrow_copy_assignable_v<T>,
                "BoundedSpscQueue values must be nothrow copy assignable");

 public:
  [[nodiscard]] bool tryPush(const T& value) noexcept {
    const auto write = write_index_.load(std::memory_order_relaxed);
    const auto read = read_index_.load(std::memory_order_acquire);
    if (write - read >= Capacity) return false;
    slots_[write % Capacity] = value;
    write_index_.store(write + 1U, std::memory_order_release);
    return true;
  }

  [[nodiscard]] bool tryPop(T& value) noexcept {
    const auto read = read_index_.load(std::memory_order_relaxed);
    const auto write = write_index_.load(std::memory_order_acquire);
    if (read == write) return false;
    value = slots_[read % Capacity];
    read_index_.store(read + 1U, std::memory_order_release);
    return true;
  }

  [[nodiscard]] std::size_t sizeApprox() const noexcept {
    const auto write = write_index_.load(std::memory_order_acquire);
    const auto read = read_index_.load(std::memory_order_acquire);
    const auto size = write - read;
    return size > Capacity ? Capacity : static_cast<std::size_t>(size);
  }

 private:
  std::array<T, Capacity> slots_{};
  alignas(64) std::atomic<std::uint64_t> write_index_{0U};
  alignas(64) std::atomic<std::uint64_t> read_index_{0U};
};

}  // namespace navigation_common
