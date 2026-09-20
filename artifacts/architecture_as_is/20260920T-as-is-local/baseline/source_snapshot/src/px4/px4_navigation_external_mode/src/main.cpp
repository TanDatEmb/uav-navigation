#include "px4_navigation_external_mode/navigation_mode.hpp"
#include "px4_navigation_external_mode/paired_node_lifetime.hpp"

#include <exception>
#include <mutex>
#include <thread>

#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  using Node = px4_ros2::NodeWithModeExecutor<
      px4_navigation_external_mode::NavigationModeExecutor,
      px4_navigation_external_mode::NavigationMode>;
  int exit_code = 0;
  std::thread state_input_thread;
  std::exception_ptr state_input_exception;
  std::mutex state_input_exception_mutex;
  const auto logException = [](const char* component,
                               const std::exception_ptr& exception) noexcept {
    try {
      if (exception) std::rethrow_exception(exception);
    } catch (const std::exception& error) {
      RCLCPP_FATAL(rclcpp::get_logger("px4_navigation_external_mode"),
                   "%s terminated with exception: %s", component, error.what());
    } catch (...) {
      RCLCPP_FATAL(rclcpp::get_logger("px4_navigation_external_mode"),
                   "%s terminated with a non-standard exception", component);
    }
  };

  px4_navigation_external_mode::PairedNodeLifetime<Node, rclcpp::Node> node_lifetime;
  try {
    node_lifetime.mode =
        std::make_shared<Node>("px4_navigation_external_mode", true);
    node_lifetime.state_input =
        std::make_shared<rclcpp::Node>("px4_navigation_external_mode_state_input");
    node_lifetime.mode->getMode().attachStateInputNode(*node_lifetime.state_input);
    state_input_thread = std::thread([
        input_node = node_lifetime.state_input,
        &state_input_exception, &state_input_exception_mutex]() {
      try {
        rclcpp::spin(input_node);
      } catch (...) {
        {
          std::lock_guard<std::mutex> lock(state_input_exception_mutex);
          state_input_exception = std::current_exception();
        }
        // A receiver failure must terminate the paired mode executor too;
        // otherwise the main thread can continue publishing against a dead
        // FMU/state-input path until the process is forcibly aborted.
        if (rclcpp::ok()) rclcpp::shutdown();
      }
    });
    try {
      rclcpp::spin(node_lifetime.mode);
    } catch (...) {
      const auto exception = std::current_exception();
      logException("mode executor", exception);
      exit_code = 1;
      // px4_ros2 may throw when the FMU disappears during shutdown. Convert
      // that lifecycle failure into a logged nonzero exit and let the paired
      // receiver thread observe shutdown and join cleanly.
      if (rclcpp::ok()) rclcpp::shutdown();
    }
  } catch (...) {
    const auto exception = std::current_exception();
    logException("node setup", exception);
    exit_code = 1;
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  if (rclcpp::ok()) rclcpp::shutdown();
  node_lifetime.joinReceiver(state_input_thread);
  std::exception_ptr captured_state_input_exception;
  {
    std::lock_guard<std::mutex> lock(state_input_exception_mutex);
    captured_state_input_exception = state_input_exception;
  }
  if (captured_state_input_exception) {
    logException("state input executor", captured_state_input_exception);
    exit_code = 1;
  }
  return exit_code;
}
