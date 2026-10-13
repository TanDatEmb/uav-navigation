#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "uavnav/ros/qos.hpp"

namespace qos = uavnav::rosutil::qos;

namespace {

struct Expected {
  rmw_qos_reliability_policy_t reliability;
  rmw_qos_durability_policy_t durability;
  std::size_t depth;
};

void expect_profile(const rclcpp::QoS& q, const Expected& e) {
  const rmw_qos_profile_t p = q.get_rmw_qos_profile();
  EXPECT_EQ(p.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(p.depth, e.depth);
  EXPECT_EQ(p.reliability, e.reliability);
  EXPECT_EQ(p.durability, e.durability);
}

}  // namespace

// SYSTEM_DESIGN §6.5, D33/O21: sensor inputs SensorDataQoS 5; /lio/state best effort 10; /lio/odometry,
// /lio/health, /alignment reliable 10; /events reliable 1000; PX4 best effort 5.
TEST(RosQos, ProductProfilesMatchTheDesign) {
  expect_profile(qos::sensor_input(), {RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE, 5});
  expect_profile(qos::state_output(), {RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE, 10});
  expect_profile(qos::reliable_output(), {RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE, 10});
  expect_profile(qos::events(), {RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_VOLATILE, 1000});
  expect_profile(qos::px4(), {RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, RMW_QOS_POLICY_DURABILITY_VOLATILE, 5});
}

TEST(RosQos, SensorInputIsTheSensorDataProfile) {
  // SensorDataQoS is best effort with volatile durability; depth 5 is the design value, not the rclcpp default.
  const rmw_qos_profile_t sensor = rclcpp::SensorDataQoS().keep_last(5).get_rmw_qos_profile();
  const rmw_qos_profile_t mine = qos::sensor_input().get_rmw_qos_profile();
  EXPECT_EQ(mine.reliability, sensor.reliability);
  EXPECT_EQ(mine.durability, sensor.durability);
  EXPECT_EQ(mine.history, sensor.history);
  EXPECT_EQ(mine.depth, sensor.depth);
}
