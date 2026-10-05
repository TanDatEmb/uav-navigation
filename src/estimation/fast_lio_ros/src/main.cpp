#include <exception>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "fast_lio_ros/fast_lio_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::NodeOptions options;
    options.arguments(
        std::vector<std::string>(argv + 1, argv + argc));
    rclcpp::spin(
        std::make_shared<uav::nav::lio::FastLioNode>(options));
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("fast_lio"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
