#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/core/time_convert.hpp"
#include "uavnav/ros/time_msg.hpp"

using namespace uavnav;
using namespace uavnav::time;

namespace {

builtin_interfaces::msg::Time stamp(std::int32_t sec, std::uint32_t nanosec) {
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  t.nanosec = nanosec;
  return t;
}

}  // namespace

TEST(RosTime, FromMsgIsRangeChecked) {
  const auto negative = rosutil::from_msg<RosTag>(stamp(-1, 0));
  ASSERT_FALSE(negative.has_value());
  EXPECT_EQ(negative.error(), TimeError::kNegativeSeconds);

  const auto bad_nanos = rosutil::from_msg<RosTag>(stamp(1, 1'000'000'000U));
  ASSERT_FALSE(bad_nanos.has_value());
  EXPECT_EQ(bad_nanos.error(), TimeError::kNanosecondsOutOfRange);

  const auto huge_nanos = rosutil::from_msg<SensorTag>(stamp(0, std::numeric_limits<std::uint32_t>::max()));
  ASSERT_FALSE(huge_nanos.has_value());
  EXPECT_EQ(huge_nanos.error(), TimeError::kNanosecondsOutOfRange);

  const auto min_sec = rosutil::from_msg<SensorTag>(stamp(std::numeric_limits<std::int32_t>::min(), 0));
  ASSERT_FALSE(min_sec.has_value());
  EXPECT_EQ(min_sec.error(), TimeError::kNegativeSeconds);

  const auto zero = rosutil::from_msg<RosTag>(stamp(0, 0));
  ASSERT_TRUE(zero.has_value());
  EXPECT_EQ(zero->ns, 0);

  const auto ok = rosutil::from_msg<SensorTag>(stamp(2, 500));
  ASSERT_TRUE(ok.has_value());
  EXPECT_EQ(ok->ns, 2'000'000'500);

  const auto max = rosutil::from_msg<RosTag>(stamp(std::numeric_limits<std::int32_t>::max(), 999'999'999U));
  ASSERT_TRUE(max.has_value());
  EXPECT_EQ(max->ns, std::int64_t{2'147'483'647} * 1'000'000'000 + 999'999'999);
}

TEST(RosTime, ToMsgRoundTrip) {
  for (const std::int64_t ns : {std::int64_t{0}, std::int64_t{1}, std::int64_t{999'999'999}, std::int64_t{1'000'000'000},
                                std::int64_t{1'700'000'000'123'456'789},  // a realistic epoch stamp (Nov 2023)
                                std::int64_t{1'700'000'000'123'456'789} / 1000,
                                std::int64_t{2'147'483'647} * 1'000'000'000 + 999'999'999}) {
    const auto msg = rosutil::to_msg(SensorTime{ns});
    ASSERT_TRUE(msg.has_value()) << ns;
    EXPECT_EQ(msg->sec, ns / 1'000'000'000);
    EXPECT_EQ(msg->nanosec, static_cast<std::uint32_t>(ns % 1'000'000'000));
    const auto back = rosutil::from_msg<SensorTag>(*msg);
    ASSERT_TRUE(back.has_value()) << ns;
    EXPECT_EQ(back->ns, ns);
  }
}

TEST(RosTime, ToMsgIsAvailableForEveryDomain) {
  // NavCommand.stamp is a RosTime, LioState.stamp a SensorTime: same conversion, same checks.
  const auto ros = rosutil::to_msg(RosTime{1'700'000'000'123'456'789});
  ASSERT_TRUE(ros.has_value());
  EXPECT_EQ(ros->sec, 1'700'000'000);
  EXPECT_EQ(ros->nanosec, 123'456'789U);
  const auto back = rosutil::from_msg<RosTag>(*ros);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(back->ns, 1'700'000'000'123'456'789);

  const auto negative = rosutil::to_msg(RosTime{-1});
  ASSERT_FALSE(negative.has_value());
  EXPECT_EQ(negative.error(), TimeError::kNegativeSeconds);
  const auto over = rosutil::to_msg(RosTime{(std::int64_t{1} << 31) * 1'000'000'000});
  ASSERT_FALSE(over.has_value());
  EXPECT_EQ(over.error(), TimeError::kOutOfRange);
  const auto steady = rosutil::to_msg(SteadyTime{5});
  ASSERT_TRUE(steady.has_value());
  EXPECT_EQ(steady->sec, 0);
}

TEST(RosTime, ToMsgRefusesWhatTheMessageCannotHold) {
  const auto negative = rosutil::to_msg(SensorTime{-1});
  ASSERT_FALSE(negative.has_value());
  EXPECT_EQ(negative.error(), TimeError::kNegativeSeconds);
  const auto min = rosutil::to_msg(SensorTime{std::numeric_limits<std::int64_t>::min()});
  ASSERT_FALSE(min.has_value());
  EXPECT_EQ(min.error(), TimeError::kNegativeSeconds);

  // builtin_interfaces/Time.sec is int32: 2^31 s would wrap to a negative stamp, so it is an error.
  const std::int64_t first_bad = (std::int64_t{1} << 31) * 1'000'000'000;
  const auto over = rosutil::to_msg(SensorTime{first_bad});
  ASSERT_FALSE(over.has_value());
  EXPECT_EQ(over.error(), TimeError::kOutOfRange);
  const auto max = rosutil::to_msg(SensorTime{std::numeric_limits<std::int64_t>::max()});
  ASSERT_FALSE(max.has_value());
  EXPECT_EQ(max.error(), TimeError::kOutOfRange);
  const auto last_ok = rosutil::to_msg(SensorTime{first_bad - 1});
  ASSERT_TRUE(last_ok.has_value());
  EXPECT_EQ(last_ok->sec, std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(last_ok->nanosec, 999'999'999U);
}

namespace {

// A ROS-time clock whose time is overridden to `ns` (as a /clock publisher would do).
void set_ros_time(rclcpp::Clock& clock, std::int64_t ns) {
  rcl_clock_t* handle = clock.get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(handle), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(handle, ns), RCL_RET_OK);
}

std::int64_t ros_ns_of(std::int64_t clock_ns) {
  rclcpp::Clock clock(RCL_ROS_TIME);
  set_ros_time(clock, clock_ns);
  return rosutil::snapshot(clock).ros.ns;
}

}  // namespace

TEST(RosTime, SnapshotPairsSteadyAndRosClocks) {
  const SteadyTime before = steady_now();
  rclcpp::Clock ros(RCL_ROS_TIME);  // no sim time: follows the system clock
  const TimeSnapshot s = rosutil::snapshot(ros);
  const SteadyTime after = steady_now();
  EXPECT_GE(s.steady.ns, before.ns);
  EXPECT_LE(s.steady.ns, after.ns);
  EXPECT_GT(s.ros.ns, 0);  // wall time since 1970

  EXPECT_EQ(ros_ns_of(12'345'678'901LL), 12'345'678'901LL);
  EXPECT_EQ(ros_ns_of(0), 0);
}

TEST(RosTime, SnapshotRosIsZeroForAClockThatIsNotRosTime) {
  rclcpp::Clock system(RCL_SYSTEM_TIME);
  rclcpp::Clock steady(RCL_STEADY_TIME);
  EXPECT_EQ(rosutil::snapshot(system).ros.ns, 0);
  EXPECT_EQ(rosutil::snapshot(steady).ros.ns, 0);
  EXPECT_GT(rosutil::snapshot(system).steady.ns, 0);
}

TEST(RosTime, SnapshotRosIsZeroWhenTheClockTimeIsUnconvertible) {
  EXPECT_EQ(ros_ns_of(-5), 0);
  EXPECT_EQ(ros_ns_of(std::numeric_limits<std::int64_t>::min()), 0);
  EXPECT_EQ(ros_ns_of(std::numeric_limits<std::int64_t>::max()), 0);
  EXPECT_EQ(ros_ns_of((kMaxStampSeconds + 1) * 1'000'000'000LL), 0);
  EXPECT_EQ(ros_ns_of(kMaxStampSeconds * 1'000'000'000LL + 999'999'999), kMaxStampSeconds * 1'000'000'000LL + 999'999'999);

  rclcpp::Clock clock(RCL_ROS_TIME);
  set_ros_time(clock, -5);
  EXPECT_GT(rosutil::snapshot(clock).steady.ns, 0);  // the steady half is still read
}
