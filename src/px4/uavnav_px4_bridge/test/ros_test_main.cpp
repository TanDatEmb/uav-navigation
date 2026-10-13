#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdlib>
#include <string>

#include "rclcpp/rclcpp.hpp"

// One main for every in-process rclcpp test of this package (the scheme of uavnav_ros/test/ros_test_main.cpp; a
// package does not include another package's test sources): initialise rclcpp once per process.
//
// Isolation: the DDS domain id is derived from the pid (227 + pid % 6 = 227..232; lower domains map onto UDP ports
// inside the Linux ephemeral range) and discovery is limited to the localhost. Each package owns a disjoint range
// (uavnav_ros 215..220, uavnav_lio 221..226, uavnav_px4_bridge 227..232), because colcon tests packages in
// parallel and RESOURCE_LOCK only serialises inside one package's ctest. ROS_DOMAIN_ID is also exported, so the
// child process of the main() test (the real px4_bridge_node executable) joins the same domain.
// CMake also serialises the in-process ROS tests with the RESOURCE_LOCK `ros_dds` (the name uavnav_ros uses).
int main(int argc, char** argv) {
  const std::size_t domain = static_cast<std::size_t>(227 + (::getpid() % 6));
  ::setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  ::setenv("ROS_DOMAIN_ID", std::to_string(domain).c_str(), 1);
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(domain);
  rclcpp::init(argc, argv, init_options);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
