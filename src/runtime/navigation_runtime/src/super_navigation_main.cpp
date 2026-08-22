#include "navigation_runtime/super_navigation_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_runtime::SuperNavigationNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
