#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <stdexcept>

namespace px4_odometry_bridge {

std::optional<std::int64_t> checked_microseconds_to_nanoseconds(std::uint64_t timestamp_us);
std::optional<std::int64_t> checked_ros_time_to_nanoseconds(
    std::int32_t sec, std::uint32_t nanosec);

struct TimeValidationConfig {
  std::int64_t max_stale_ns{200'000'000};
  std::int64_t max_future_ns{200'000'000};
  std::int64_t probable_restart_regression_ns{1'000'000'000};
  std::int64_t restart_low_epoch_max_ns{10'000'000'000};
};

enum class TimestampEvent : std::uint8_t {
  kAccepted,
  kDuplicate,
  kSmallRegression,
  kProbableSourceRestart,
  kStale,
  kFuture,
  kInvalid,
};

struct TimeValidationResult {
  bool accepted{false};
  std::uint64_t generation{0};
  std::string reason;
  TimestampEvent event{TimestampEvent::kInvalid};
};

class TimestampValidator {
 public:
  explicit TimestampValidator(TimeValidationConfig config = {}) : config_(config) {
    if (config_.max_stale_ns <= 0 || config_.max_future_ns <= 0 ||
        config_.probable_restart_regression_ns <= 0 ||
        config_.restart_low_epoch_max_ns <= 0) {
      throw std::invalid_argument("invalid PX4 timestamp validation configuration");
    }
  }
  TimeValidationResult observe(std::uint64_t timestamp_us, std::int64_t now_ns);
  void clear();

 private:
  TimeValidationConfig config_;
  std::optional<std::int64_t> last_timestamp_ns_;
  std::uint64_t generation_{0};
};

}  // namespace px4_odometry_bridge
