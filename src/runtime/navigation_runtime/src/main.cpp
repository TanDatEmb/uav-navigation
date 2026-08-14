#include <rclcpp/rclcpp.hpp>

#include "navigation_runtime/navigation_runtime_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<navigation_runtime::NavigationRuntimeNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
