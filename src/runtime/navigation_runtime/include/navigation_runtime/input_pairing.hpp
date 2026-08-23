#pragma once

#include <cstdlib>
#include <cstdint>
#include <deque>
#include <optional>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace navigation_runtime::input_pairing {

inline std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

inline std::optional<std::size_t> nearestOdometryIndex(
    const std::deque<nav_msgs::msg::Odometry>& history,
    std::int64_t cloud_stamp_ns,
    std::int64_t max_skew_ns) {
  if (history.empty() || cloud_stamp_ns <= 0 || max_skew_ns < 0) return std::nullopt;
  std::optional<std::size_t> best_index;
  std::int64_t best_delta = max_skew_ns + 1;
  for (std::size_t index = 0; index < history.size(); ++index) {
    const auto odometry_stamp_ns = stampNanoseconds(history[index].header.stamp);
    if (odometry_stamp_ns <= 0) continue;
    const auto delta = std::llabs(odometry_stamp_ns - cloud_stamp_ns);
    if (delta < best_delta) {
      best_delta = delta;
      best_index = index;
    }
  }
  return best_index;
}

}  // namespace navigation_runtime::input_pairing
