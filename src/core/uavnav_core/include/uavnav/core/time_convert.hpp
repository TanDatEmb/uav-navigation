#pragma once

#include <cstdint>
#include <string_view>

#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"

// Range-checked construction of TimePoint / Duration from external data (ROS stamps,
// config seconds). time.hpp does not check for overflow (it is undefined behaviour), so
// every raw value from outside must come through here before it becomes a time value.
namespace uavnav::time {

enum class TimeError : std::uint8_t {
  kNegativeSeconds,         ///< stamp seconds < 0
  kNanosecondsOutOfRange,   ///< stamp nanoseconds >= 1e9
  kNotFinite,               ///< NaN or +-inf seconds
  kOutOfRange               ///< stamp seconds > 9e9, or |duration seconds| > 1e6
};

constexpr std::string_view to_string(TimeError e) {
  switch (e) {
    case TimeError::kNegativeSeconds: return "NEGATIVE_SECONDS";
    case TimeError::kNanosecondsOutOfRange: return "NANOSECONDS_OUT_OF_RANGE";
    case TimeError::kNotFinite: return "NOT_FINITE";
    case TimeError::kOutOfRange: return "OUT_OF_RANGE";
  }
  return "";
}
static_assert(ReasonEnum<TimeError>);

/// Largest accepted stamp seconds (about 285 years): 9e9 * 1e9 + 999'999'999 < INT64_MAX.
inline constexpr std::int64_t kMaxStampSeconds = 9'000'000'000;
/// Largest accepted |seconds| for duration_from_seconds.
inline constexpr double kMaxDurationSeconds = 1e6;

/// Builds an instant from a (sec, nanosec) stamp. sec in [0, 9'000'000'000], nanosec < 1e9.
/// Checks run in the order kNegativeSeconds, kNanosecondsOutOfRange, kOutOfRange, all
/// before the multiply, so no signed overflow can occur for any input.
template <class Tag>
constexpr Result<TimePoint<Tag>, TimeError> from_stamp(std::int64_t sec, std::uint32_t nanosec) {
  if (sec < 0) return std::unexpected(TimeError::kNegativeSeconds);
  if (nanosec >= 1'000'000'000U) return std::unexpected(TimeError::kNanosecondsOutOfRange);
  if (sec > kMaxStampSeconds) return std::unexpected(TimeError::kOutOfRange);
  return TimePoint<Tag>{sec * 1'000'000'000 + static_cast<std::int64_t>(nanosec)};
}

/// Converts seconds to a Duration, rounding to the nearest nanosecond.
/// kNotFinite is checked first (NaN, +-inf), then |s| <= 1e6, else kOutOfRange.
Result<Duration, TimeError> duration_from_seconds(double s) noexcept;

}  // namespace uavnav::time
