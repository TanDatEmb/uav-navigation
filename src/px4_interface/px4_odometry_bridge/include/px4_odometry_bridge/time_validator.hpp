#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace px4_odometry_bridge {

std::optional<std::int64_t> checked_microseconds_to_nanoseconds(std::uint64_t timestamp_us);
std::optional<std::int64_t> checked_ros_time_to_nanoseconds(
    std::int32_t sec, std::uint32_t nanosec);

struct TimeValidationConfig {
  std::int64_t max_stale_ns{200'000'000};
  std::int64_t max_future_ns{200'000'000};
};

struct TimeValidationResult {
  bool accepted{false};
  std::uint64_t generation{0};
  std::string reason;
};

class TimestampValidator {
 public:
  explicit TimestampValidator(TimeValidationConfig config = {}) : config_(config) {}
  TimeValidationResult observe(std::uint64_t timestamp_us, std::int64_t now_ns);
  void clear();

 private:
  TimeValidationConfig config_;
  std::optional<std::int64_t> last_timestamp_ns_;
  std::uint64_t generation_{0};
};

}  // namespace px4_odometry_bridge
