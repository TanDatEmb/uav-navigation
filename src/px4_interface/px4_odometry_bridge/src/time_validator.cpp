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

std::optional<std::int64_t> checked_ros_time_to_nanoseconds(
    std::int32_t sec, std::uint32_t nanosec) {
  if (sec < 0 || nanosec >= 1'000'000'000U) return std::nullopt;
  constexpr auto max = std::numeric_limits<std::int64_t>::max();
  const auto seconds = static_cast<std::int64_t>(sec);
  if (seconds > (max - static_cast<std::int64_t>(nanosec)) / 1'000'000'000LL) {
    return std::nullopt;
  }
  const auto result = seconds * 1'000'000'000LL + static_cast<std::int64_t>(nanosec);
  return result > 0 ? std::optional<std::int64_t>(result) : std::nullopt;
}

TimeValidationResult TimestampValidator::observe(std::uint64_t timestamp_us,
                                                 std::int64_t now_ns) {
  const auto timestamp_ns = checked_microseconds_to_nanoseconds(timestamp_us);
  if (!timestamp_ns.has_value()) {
    return {false, generation_, "timestamp conversion overflow/zero", TimestampEvent::kInvalid};
  }
  if (last_timestamp_ns_.has_value() && *timestamp_ns <= *last_timestamp_ns_) {
    if (*timestamp_ns == *last_timestamp_ns_) {
      return {false, generation_, "duplicate PX4 sample timestamp", TimestampEvent::kDuplicate};
    }
    const auto regression_ns = *last_timestamp_ns_ - *timestamp_ns;
    if (regression_ns >= config_.probable_restart_regression_ns &&
        *last_timestamp_ns_ > config_.restart_low_epoch_max_ns &&
        *timestamp_ns <= config_.restart_low_epoch_max_ns) {
      ++generation_;
      last_timestamp_ns_ = *timestamp_ns;
      return {true, generation_, "probable PX4 source restart", TimestampEvent::kProbableSourceRestart};
    }
    return {false, generation_, "small PX4 timestamp regression", TimestampEvent::kSmallRegression};
  }
  if (now_ns - *timestamp_ns > config_.max_stale_ns) {
    return {false, generation_, "stale PX4 sample timestamp", TimestampEvent::kStale};
  }
  if (*timestamp_ns - now_ns > config_.max_future_ns) {
    return {false, generation_, "future PX4 sample timestamp", TimestampEvent::kFuture};
  }
  last_timestamp_ns_ = *timestamp_ns;
  return {true, generation_, {}, TimestampEvent::kAccepted};
}

void TimestampValidator::clear() {
  last_timestamp_ns_.reset();
  ++generation_;
}

}  // namespace px4_odometry_bridge
