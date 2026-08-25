#pragma once

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace navigation_common {

using TimestampNs = std::int64_t;

inline constexpr TimestampNs kNanosecondsPerSecond = 1'000'000'000LL;
inline constexpr TimestampNs kNanosecondsPerMicrosecond = 1'000LL;

// Steady time is only for local receive, timeout, and latency measurements.
// It must not be compared with ROS, simulation, or sensor source timestamps.
[[nodiscard]] inline TimestampNs steadyClockNowNanoseconds() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

[[nodiscard]] inline std::optional<TimestampNs> microsecondsToNanoseconds(
    const std::uint64_t microseconds) noexcept {
  constexpr auto max = static_cast<std::uint64_t>(
      std::numeric_limits<TimestampNs>::max());
  if (microseconds > max / static_cast<std::uint64_t>(kNanosecondsPerMicrosecond)) {
    return std::nullopt;
  }
  const auto nanoseconds =
      microseconds * static_cast<std::uint64_t>(kNanosecondsPerMicrosecond);
  if (nanoseconds == 0U) {
    return std::nullopt;
  }
  return static_cast<TimestampNs>(nanoseconds);
}

[[nodiscard]] inline std::optional<std::uint64_t> nanosecondsToMicroseconds(
    const TimestampNs nanoseconds) noexcept {
  if (nanoseconds <= 0) {
    return std::nullopt;
  }
  const auto microseconds = static_cast<std::uint64_t>(
      nanoseconds / kNanosecondsPerMicrosecond);
  if (microseconds == 0U) {
    return std::nullopt;
  }
  return microseconds;
}

[[nodiscard]] inline std::optional<TimestampNs> rosTimeToNanoseconds(
    const builtin_interfaces::msg::Time& stamp) noexcept {
  if (stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond)) {
    return std::nullopt;
  }
  const auto seconds = static_cast<__int128>(stamp.sec);
  const auto nanoseconds = seconds * kNanosecondsPerSecond + stamp.nanosec;
  if (nanoseconds < std::numeric_limits<TimestampNs>::min() ||
      nanoseconds > std::numeric_limits<TimestampNs>::max()) {
    return std::nullopt;
  }
  return static_cast<TimestampNs>(nanoseconds);
}

[[nodiscard]] inline std::optional<builtin_interfaces::msg::Time> nanosecondsToRosTime(
    const TimestampNs nanoseconds) noexcept {
  if (nanoseconds < 0) {
    return std::nullopt;
  }
  const auto seconds = nanoseconds / kNanosecondsPerSecond;
  if (seconds > std::numeric_limits<std::int32_t>::max()) {
    return std::nullopt;
  }
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(seconds);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
  return stamp;
}

[[nodiscard]] inline std::optional<builtin_interfaces::msg::Time> secondsToRosTime(
    const double seconds) noexcept {
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return std::nullopt;
  }
  const auto whole_seconds = static_cast<std::int64_t>(seconds);
  const auto fraction = seconds - static_cast<double>(whole_seconds);
  if (whole_seconds > std::numeric_limits<std::int32_t>::max() || fraction < 0.0) {
    return std::nullopt;
  }
  const auto nanoseconds = static_cast<std::int64_t>(fraction * kNanosecondsPerSecond);
  if (nanoseconds < 0 || nanoseconds >= kNanosecondsPerSecond) {
    return std::nullopt;
  }
  return nanosecondsToRosTime(whole_seconds * kNanosecondsPerSecond + nanoseconds);
}

}  // namespace navigation_common
