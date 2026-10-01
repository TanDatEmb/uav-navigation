#include "fast_lio_core/time/duration.hpp"

#include <limits>

namespace uav::nav::lio {
namespace {

[[nodiscard]] bool additionOverflows(std::int64_t lhs, std::int64_t rhs) noexcept {
  return (rhs > 0 && lhs > std::numeric_limits<std::int64_t>::max() - rhs) ||
         (rhs < 0 && lhs < std::numeric_limits<std::int64_t>::min() - rhs);
}

}  // namespace

Result<Duration> checkedAdd(Duration lhs, Duration rhs) {
  if (additionOverflows(lhs.nanoseconds(), rhs.nanoseconds())) {
    return Status(StatusCode::kOutOfRange, "Duration addition overflows int64");
  }
  return Duration(lhs.nanoseconds() + rhs.nanoseconds());
}

Result<Duration> checkedSubtract(Duration lhs, Duration rhs) {
  const auto lhs_ns = lhs.nanoseconds();
  const auto rhs_ns = rhs.nanoseconds();
  if ((rhs_ns > 0 && lhs_ns < std::numeric_limits<std::int64_t>::min() + rhs_ns) ||
      (rhs_ns < 0 && lhs_ns > std::numeric_limits<std::int64_t>::max() + rhs_ns)) {
    return Status(StatusCode::kOutOfRange, "Duration subtraction overflows int64");
  }
  return Duration(lhs_ns - rhs_ns);
}

}  // namespace uav::nav::lio
