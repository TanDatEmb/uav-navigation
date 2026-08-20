#include <rclcpp/rclcpp.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "navigation_runtime/navigation_runtime_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<navigation_runtime::NavigationRuntimeNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
