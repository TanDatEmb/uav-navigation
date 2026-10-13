#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdlib>

#include "rclcpp/rclcpp.hpp"

// One main for every rclcpp test of this package: initialise rclcpp once per process.
//
// Isolation: the DDS domain id is derived from the pid (100..199) and discovery is limited to the
// localhost, so tests running in parallel under ctest/colcon, or other ROS processes on the machine,
// neither see nor disturb each other's /events topic.
int main(int argc, char** argv) {
  ::setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(static_cast<std::size_t>(100 + (::getpid() % 100)));
  rclcpp::init(argc, argv, init_options);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
