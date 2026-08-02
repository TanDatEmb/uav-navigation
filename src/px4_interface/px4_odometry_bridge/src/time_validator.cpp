#include "px4_odometry_bridge/time_validator.hpp"

#include <limits>

namespace px4_odometry_bridge {

std::optional<std::int64_t> checked_microseconds_to_nanoseconds(std::uint64_t timestamp_us) {
  constexpr auto max = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
  if (timestamp_us > max / 1000U) return std::nullopt;
  const auto ns = timestamp_us * 1000U;
  if (ns == 0 || ns > max) return std::nullopt;
  return static_cast<std::int64_t>(ns);
}

TimeValidationResult TimestampValidator::observe(std::uint64_t timestamp_us,
                                                 std::int64_t now_ns) {
  const auto timestamp_ns = checked_microseconds_to_nanoseconds(timestamp_us);
  if (!timestamp_ns.has_value()) return {false, generation_, "timestamp conversion overflow/zero"};
  if (last_timestamp_ns_.has_value() && *timestamp_ns <= *last_timestamp_ns_) {
    return {false, generation_, "duplicate or regressing PX4 sample timestamp"};
  }
  if (now_ns - *timestamp_ns > config_.max_stale_ns) {
    return {false, generation_, "stale PX4 sample timestamp"};
  }
  if (*timestamp_ns - now_ns > config_.max_future_ns) {
    return {false, generation_, "future PX4 sample timestamp"};
  }
  last_timestamp_ns_ = *timestamp_ns;
  return {true, generation_, {}};
}

void TimestampValidator::clear() {
  last_timestamp_ns_.reset();
  ++generation_;
}

}  // namespace px4_odometry_bridge
