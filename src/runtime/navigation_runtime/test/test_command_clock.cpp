#include <ros_interface/ros_interface.hpp>

#include <gtest/gtest.h>

namespace navigation_runtime {
namespace {

TEST(CommandClock, ReadsTheCurrentRuntimeClockOnEverySample) {
  double now_s = 10.0;
  ros_interface::RosInterface clock{[&now_s]() { return now_s; }};

  EXPECT_DOUBLE_EQ(clock.getSimTime(), 10.0);
  now_s = 10.02;
  EXPECT_DOUBLE_EQ(clock.getSimTime(), 10.02);
}

}  // namespace
}  // namespace navigation_runtime
