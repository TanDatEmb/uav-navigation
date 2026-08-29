#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace navigation_runtime {

inline std::optional<std::uint64_t> advanceMonotonicId(std::atomic_uint64_t& value) noexcept {
  auto current = value.load(std::memory_order_acquire);
  while (current != std::numeric_limits<std::uint64_t>::max()) {
    const auto next = current + 1U;
    if (value.compare_exchange_weak(current, next, std::memory_order_acq_rel,
                                    std::memory_order_acquire)) {
      return next;
    }
  }
  return std::nullopt;
}

inline std::optional<std::chrono::nanoseconds> ratePeriodNanoseconds(
    const double rate_hz) noexcept {
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) return std::nullopt;
  const long double period_ns = 1.0e9L / static_cast<long double>(rate_hz);
  if (!std::isfinite(period_ns) || period_ns < 1.0L ||
      period_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  return std::chrono::nanoseconds{static_cast<std::int64_t>(period_ns)};
}

}  // namespace navigation_runtime
