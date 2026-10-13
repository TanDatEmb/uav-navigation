#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdlib>

#include "rclcpp/rclcpp.hpp"

// One main for every rclcpp test of this package: initialise rclcpp once per process.
//
// Isolation: the DDS domain id is derived from the pid (215 + pid % 6 = 215..220; lower domains map onto UDP ports
// inside the Linux ephemeral range) and discovery is limited to the localhost. Each package owns a disjoint range
// (uavnav_ros 215..220, uavnav_lio 221..226, uavnav_px4_bridge 227..232), because colcon tests packages in
// parallel and RESOURCE_LOCK only serialises inside one package's ctest: tests of two packages, or other ROS
// processes on the machine, neither see nor disturb each other's topics (/events, /lio/*).
// CMake also serialises the in-process ROS tests with the RESOURCE_LOCK `ros_dds`.
int main(int argc, char** argv) {
  ::setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(static_cast<std::size_t>(215 + (::getpid() % 6)));
  rclcpp::init(argc, argv, init_options);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
