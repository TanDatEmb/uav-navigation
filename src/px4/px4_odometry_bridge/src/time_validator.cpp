#include "px4_odometry_bridge/time_validator.hpp"

#include <limits>

#include <navigation_common/time.hpp>

namespace px4_odometry_bridge {

std::optional<std::int64_t> checked_microseconds_to_nanoseconds(std::uint64_t timestamp_us) {
  return navigation_common::microsecondsToNanoseconds(timestamp_us);
}

std::optional<std::int64_t> checked_ros_time_to_nanoseconds(
    std::int32_t sec, std::uint32_t nanosec) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  const auto result = navigation_common::rosTimeToNanoseconds(stamp);
  return result && *result > 0 ? result : std::nullopt;
}

TimeValidationResult TimestampValidator::observe(std::uint64_t timestamp_us,
                                                 std::int64_t now_ns) {
  if (now_ns <= 0) {
    return {false, generation_, "invalid current time", TimestampEvent::kInvalid};
  }
  const auto timestamp_ns = checked_microseconds_to_nanoseconds(timestamp_us);
  if (!timestamp_ns.has_value()) {
    return {false, generation_, "timestamp conversion overflow/zero", TimestampEvent::kInvalid};
  }
  if (last_timestamp_ns_.has_value() && *timestamp_ns <= *last_timestamp_ns_) {
    if (*timestamp_ns == *last_timestamp_ns_) {
      return {false, generation_, "duplicate PX4 sample timestamp", TimestampEvent::kDuplicate};
    }
    const auto regression = navigation_common::checkedDifference(
        *last_timestamp_ns_, *timestamp_ns);
    if (!regression) {
      return {false, generation_, "timestamp delta is not representable",
              TimestampEvent::kInvalid};
    }
    const auto regression_ns = *regression;
    if (regression_ns >= config_.probable_restart_regression_ns &&
        *last_timestamp_ns_ > config_.restart_low_epoch_max_ns &&
        *timestamp_ns <= config_.restart_low_epoch_max_ns) {
      if (generation_ == std::numeric_limits<std::uint64_t>::max()) {
        return {false, generation_, "timestamp generation exhausted", TimestampEvent::kInvalid};
      }
      ++generation_;
      last_timestamp_ns_ = *timestamp_ns;
      return {true, generation_, "probable PX4 source restart", TimestampEvent::kProbableSourceRestart};
    }
    return {false, generation_, "small PX4 timestamp regression", TimestampEvent::kSmallRegression};
  }
  const auto age = navigation_common::checkedDifference(now_ns, *timestamp_ns);
  if (!age) {
    return {false, generation_, "timestamp delta is not representable",
            TimestampEvent::kInvalid};
  }
  if (*age > config_.max_stale_ns) {
    return {false, generation_, "stale PX4 sample timestamp", TimestampEvent::kStale};
  }
  const auto future = navigation_common::checkedDifference(*timestamp_ns, now_ns);
  if (!future) {
    return {false, generation_, "timestamp delta is not representable",
            TimestampEvent::kInvalid};
  }
  if (*future > config_.max_future_ns) {
    return {false, generation_, "future PX4 sample timestamp", TimestampEvent::kFuture};
  }
  last_timestamp_ns_ = *timestamp_ns;
  return {true, generation_, {}, TimestampEvent::kAccepted};
}

void TimestampValidator::clear() {
  last_timestamp_ns_.reset();
  if (generation_ != std::numeric_limits<std::uint64_t>::max()) ++generation_;
}

}  // namespace px4_odometry_bridge
