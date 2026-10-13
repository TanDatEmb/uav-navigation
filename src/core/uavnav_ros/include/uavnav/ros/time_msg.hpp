#pragma once

#include <cstdint>
#include <limits>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/clock.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/core/time_convert.hpp"

// Conversion between builtin_interfaces/msg/Time and the clock-domain time types (SYSTEM_DESIGN §6.3,
// D25). A stamp from outside the process is range-checked before it becomes a TimePoint.
namespace uavnav::rosutil {

/// A message stamp as an instant in the domain `Tag`, via time::from_stamp. `sec` is int32, so it may be
/// negative: kNegativeSeconds; nanosec >= 1e9: kNanosecondsOutOfRange. kOutOfRange (sec > 9e9) cannot
/// happen for an int32 sec but comes from the same call.
template <class Tag>
Result<time::TimePoint<Tag>, time::TimeError> from_msg(const builtin_interfaces::msg::Time& stamp) {
  return time::from_stamp<Tag>(stamp.sec, stamp.nanosec);
}

/// An instant as a message stamp. Errors, never a wrapped or clamped stamp:
/// - t.ns < 0: kNegativeSeconds (the same reason from_msg gives a negative stamp);
/// - t.ns / 1e9 > INT32_MAX (year 2038 and later): kOutOfRange. builtin_interfaces/Time.sec is an int32, so
///   such a stamp would wrap to a negative one in the message. kOutOfRange is the reason from_stamp already
///   uses for "seconds above the representable maximum"; a new TimeError would change the core enum.
inline Result<builtin_interfaces::msg::Time, time::TimeError> to_msg(time::SensorTime t) {
  if (t.ns < 0) return std::unexpected(time::TimeError::kNegativeSeconds);
  const std::int64_t sec = t.ns / 1'000'000'000;
  if (sec > std::numeric_limits<std::int32_t>::max()) return std::unexpected(time::TimeError::kOutOfRange);
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<std::int32_t>(sec);
  msg.nanosec = static_cast<std::uint32_t>(t.ns % 1'000'000'000);
  return msg;
}

/// One snapshot of the two clocks: steady_now() and `clock.now()` (the node's ROS clock, which follows
/// /clock in simulation). ros is 0 when the clock time is not convertible (negative or above the
/// time_convert range, or the clock threw), so a bad clock never makes a garbage RosTime. Never throws.
inline time::TimeSnapshot snapshot(const rclcpp::Clock& clock) noexcept {
  time::TimeSnapshot s{time::steady_now(), time::RosTime{0}};
  try {
    const std::int64_t ns = clock.now().nanoseconds();
    if (ns >= 0) {
      const auto ros = time::from_stamp<time::RosTag>(ns / 1'000'000'000, static_cast<std::uint32_t>(ns % 1'000'000'000));
      if (ros.has_value()) s.ros = *ros;
    }
  } catch (...) {
    // Leave ros at 0.
  }
  return s;
}

}  // namespace uavnav::rosutil
