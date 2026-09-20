#pragma once

#include <compare>
#include <cstdint>

#include "fast_lio_core/common/constants.hpp"
#include "fast_lio_core/common/result.hpp"

namespace uav::nav::lio {

class Duration {
 public:
  constexpr Duration() noexcept = default;
  explicit constexpr Duration(std::int64_t nanoseconds) noexcept : nanoseconds_(nanoseconds) {}

  [[nodiscard]] constexpr std::int64_t nanoseconds() const noexcept { return nanoseconds_; }

  [[nodiscard]] constexpr double seconds() const noexcept {
    return static_cast<double>(nanoseconds_) / static_cast<double>(kNanosecondsPerSecond);
  }

  auto operator<=>(const Duration&) const = default;

 private:
  std::int64_t nanoseconds_{0};
};

[[nodiscard]] Result<Duration> checkedAdd(Duration lhs, Duration rhs);
[[nodiscard]] Result<Duration> checkedSubtract(Duration lhs, Duration rhs);

}  // namespace uav::nav::lio
