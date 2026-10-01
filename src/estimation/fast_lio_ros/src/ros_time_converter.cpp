#include "fast_lio_ros/ros_time_converter.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>

#include <navigation_common/time.hpp>

namespace uav::nav::lio {

Timestamp RosTimeConverter::fromRos(
    const builtin_interfaces::msg::Time& stamp,
    ClockDomain clock_domain) {
  const auto nanoseconds = navigation_common::rosTimeToNanoseconds(stamp);
  if (!nanoseconds) {
    throw std::invalid_argument("ROS timestamp is malformed or out of range");
  }
  return Timestamp{*nanoseconds, clock_domain};
}

builtin_interfaces::msg::Time RosTimeConverter::toRos(Timestamp stamp) {
  const auto result = navigation_common::nanosecondsToRosTime(stamp.nanoseconds());
  if (!result) {
    throw std::invalid_argument("timestamp cannot be represented as ROS Time");
  }
  return *result;
}

}  // namespace uav::nav::lio
