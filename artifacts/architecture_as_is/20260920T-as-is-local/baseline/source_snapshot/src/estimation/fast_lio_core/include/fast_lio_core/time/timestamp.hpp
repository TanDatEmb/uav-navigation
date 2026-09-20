#pragma once

#include <cstdint>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/time/clock_domain.hpp"
#include "fast_lio_core/time/duration.hpp"

namespace uav::nav::lio {

class Timestamp {
 public:
  constexpr Timestamp() noexcept = default;
  explicit constexpr Timestamp(std::int64_t nanoseconds,
                               ClockDomain clock_domain = ClockDomain::kRosTime) noexcept
      : nanoseconds_(nanoseconds), clock_domain_(clock_domain) {}

  [[nodiscard]] constexpr std::int64_t nanoseconds() const noexcept { return nanoseconds_; }

  [[nodiscard]] constexpr ClockDomain clock_domain() const noexcept { return clock_domain_; }

  [[nodiscard]] constexpr bool sameClockDomain(const Timestamp& other) const noexcept {
    return clock_domain_ == other.clock_domain_;
  }

  friend constexpr bool operator==(const Timestamp& lhs, const Timestamp& rhs) noexcept {
    return lhs.clock_domain_ == rhs.clock_domain_ && lhs.nanoseconds_ == rhs.nanoseconds_;
  }

 private:
  std::int64_t nanoseconds_{0};
  ClockDomain clock_domain_{ClockDomain::kRosTime};
};

[[nodiscard]] Result<Duration> checkedDifference(Timestamp lhs, Timestamp rhs);
[[nodiscard]] Result<Timestamp> checkedAdd(Timestamp timestamp, Duration duration);
[[nodiscard]] Result<Timestamp> checkedSubtract(Timestamp timestamp, Duration duration);
[[nodiscard]] Result<int> checkedCompare(Timestamp lhs, Timestamp rhs);

}  // namespace uav::nav::lio
