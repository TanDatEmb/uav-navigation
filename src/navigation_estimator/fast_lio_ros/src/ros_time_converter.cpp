#include "fast_lio_ros/ros_time_converter.hpp"

#include <cstdint>
#include <stdexcept>

namespace uav::nav::lio {

Timestamp RosTimeConverter::fromRos(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.nanosec >= 1'000'000'000U) {
    throw std::invalid_argument("ROS timestamp nanosec is out of range");
  }
  constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
  return Timestamp{static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
                       static_cast<std::int64_t>(stamp.nanosec),
                   ClockDomain::kRosTime};
}

builtin_interfaces::msg::Time RosTimeConverter::toRos(Timestamp stamp) {
  constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
  if (stamp.nanoseconds() < 0) {
    throw std::invalid_argument("negative ROS timestamps are unsupported");
  }
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(stamp.nanoseconds() / kNanosecondsPerSecond);
  result.nanosec = static_cast<std::uint32_t>(stamp.nanoseconds() % kNanosecondsPerSecond);
  return result;
}

}  // namespace uav::nav::lio
