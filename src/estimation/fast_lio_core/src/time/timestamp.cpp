#include "fast_lio_core/time/timestamp.hpp"

#include <limits>
#include <string>

namespace uav::nav::lio {
namespace {

[[nodiscard]] Status clockMismatch(Timestamp lhs, Timestamp rhs) {
  return Status(StatusCode::kClockDomainMismatch,
                "Cannot combine timestamps from " + std::string(toString(lhs.clock_domain())) +
                    " and " + std::string(toString(rhs.clock_domain())));
}

}  // namespace

Result<Duration> checkedDifference(Timestamp lhs, Timestamp rhs) {
  if (!lhs.sameClockDomain(rhs)) {
    return clockMismatch(lhs, rhs);
  }
  const auto lhs_ns = lhs.nanoseconds();
  const auto rhs_ns = rhs.nanoseconds();
  if ((rhs_ns < 0 && lhs_ns > std::numeric_limits<std::int64_t>::max() + rhs_ns) ||
      (rhs_ns > 0 && lhs_ns < std::numeric_limits<std::int64_t>::min() + rhs_ns)) {
    return Status(StatusCode::kOutOfRange, "Timestamp difference overflows int64");
  }
  return Duration(lhs_ns - rhs_ns);
}

Result<Timestamp> checkedAdd(Timestamp timestamp, Duration duration) {
  const auto sum = checkedAdd(Duration(timestamp.nanoseconds()), duration);
  if (!sum.ok()) {
    return sum.status();
  }
  return Timestamp(sum.value().nanoseconds(), timestamp.clock_domain());
}

Result<Timestamp> checkedSubtract(Timestamp timestamp, Duration duration) {
  const auto difference = checkedSubtract(Duration(timestamp.nanoseconds()), duration);
  if (!difference.ok()) {
    return difference.status();
  }
  return Timestamp(difference.value().nanoseconds(), timestamp.clock_domain());
}

Result<int> checkedCompare(Timestamp lhs, Timestamp rhs) {
  if (!lhs.sameClockDomain(rhs)) {
    return clockMismatch(lhs, rhs);
  }
  if (lhs.nanoseconds() < rhs.nanoseconds()) {
    return -1;
  }
  if (lhs.nanoseconds() > rhs.nanoseconds()) {
    return 1;
  }
  return 0;
}

}  // namespace uav::nav::lio
