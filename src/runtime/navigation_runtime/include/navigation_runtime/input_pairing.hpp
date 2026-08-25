#pragma once

#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <navigation_common/time.hpp>

namespace navigation_runtime::input_pairing {

template <typename Payload>
struct StampedObservation {
  Payload payload;
  std::int64_t stamp_ns{0};
};

template <typename Payload>
struct CoherentPair {
  Payload payload;
  nav_msgs::msg::Odometry corrected_odometry;
  std::int64_t stamp_ns{0};
};

inline std::optional<std::size_t> nearestOdometryIndex(
    const std::deque<nav_msgs::msg::Odometry>& history,
    std::int64_t cloud_stamp_ns,
    std::int64_t max_skew_ns) {
  if (history.empty() || cloud_stamp_ns <= 0 || max_skew_ns < 0) return std::nullopt;
  std::optional<std::size_t> best_index;
  std::int64_t best_delta = max_skew_ns + 1;
  for (std::size_t index = 0; index < history.size(); ++index) {
    const auto odometry_stamp_ns =
        navigation_common::rosTimeToNanoseconds(history[index].header.stamp).value_or(0);
    if (odometry_stamp_ns <= 0) continue;
    const auto delta = std::llabs(odometry_stamp_ns - cloud_stamp_ns);
    if (delta < best_delta) {
      best_delta = delta;
      best_index = index;
    }
  }
  return best_index;
}

inline std::optional<std::size_t> exactOdometryIndex(
    const std::deque<nav_msgs::msg::Odometry>& history,
    std::int64_t observation_stamp_ns) {
  if (history.empty() || observation_stamp_ns <= 0) return std::nullopt;
  for (std::size_t index = 0; index < history.size(); ++index) {
    if (navigation_common::rosTimeToNanoseconds(history[index].header.stamp).value_or(0) ==
        observation_stamp_ns) {
      return index;
    }
  }
  return std::nullopt;
}

template <typename Payload>
std::optional<CoherentPair<Payload>> tryTakeExactPair(
    std::optional<StampedObservation<Payload>>& pending,
    std::deque<nav_msgs::msg::Odometry>& corrected_history) {
  if (!pending) return std::nullopt;
  const auto index = exactOdometryIndex(corrected_history, pending->stamp_ns);
  if (!index) return std::nullopt;
  CoherentPair<Payload> result{
      std::move(pending->payload), corrected_history.at(*index), pending->stamp_ns};
  pending.reset();
  corrected_history.erase(
      corrected_history.begin(),
      corrected_history.begin() + static_cast<std::ptrdiff_t>(*index + 1U));
  return result;
}

}  // namespace navigation_runtime::input_pairing
