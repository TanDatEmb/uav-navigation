#include <rclcpp/rclcpp.hpp>

#include "navigation_mapping/navigation_mapping_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<navigation_mapping::NavigationMappingNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
