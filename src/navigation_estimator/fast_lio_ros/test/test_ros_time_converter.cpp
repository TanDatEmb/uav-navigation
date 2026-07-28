#include <gtest/gtest.h>

#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

TEST(RosTimeConverterTest, PreservesIntegerNanoseconds) {
  builtin_interfaces::msg::Time input;
  input.sec = 12;
  input.nanosec = 345U;
  const auto core = RosTimeConverter::fromRos(input);
  EXPECT_EQ(core.nanoseconds(), 12'000'000'345LL);
  EXPECT_EQ(core.clock_domain(), ClockDomain::kRosTime);
  const auto output = RosTimeConverter::toRos(core);
  EXPECT_EQ(output.sec, input.sec);
  EXPECT_EQ(output.nanosec, input.nanosec);
}

TEST(RosTimeConverterTest, RejectsInvalidNanosecondField) {
  builtin_interfaces::msg::Time input;
  input.nanosec = 1'000'000'000U;
  EXPECT_THROW(static_cast<void>(RosTimeConverter::fromRos(input)), std::invalid_argument);
}

}  // namespace uav::nav::lio
