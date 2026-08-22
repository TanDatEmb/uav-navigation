#pragma once

#include <atomic>
#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <navigation_interfaces/msg/navigation_goal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mars_quadrotor_msgs/msg/polynomial_trajectory.hpp>

#include <ros_interface/ros_interface.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <super_core/super_planner.h>

namespace navigation_runtime {

// Product ROS boundary for the imported SUPER core. The subscriptions only
// retain the newest frame; map mutation and planning are serialized in the
// timer callback so SUPER always observes one coherent ROG-Map state.
class SuperNavigationNode final : public rclcpp::Node {
 public:
  explicit SuperNavigationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message);
  void onGoal(const navigation_interfaces::msg::NavigationGoal::ConstSharedPtr& message);
  void runCycle();
  void publishHeartbeat();

  static bool decodeCloud(const sensor_msgs::msg::PointCloud2& message,
                          rog_map::PointCloud& output);
  static builtin_interfaces::msg::Time rosTimeFromSeconds(double seconds);
  mars_quadrotor_msgs::msg::PolynomialTrajectory serializeTrajectory(
      const geometry_utils::Trajectory& position,
      const geometry_utils::Trajectory& yaw,
      double now_seconds,
      bool append_terminal_hold);

  std::string cloud_topic_;
  std::string odometry_topic_;
  std::string goal_topic_;
  std::string trajectory_topic_;
  std::string super_config_path_;
  std::string planning_frame_;
  double planner_rate_hz_{10.0};
  double heartbeat_rate_hz_{50.0};
  double heartbeat_horizon_s_{3.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<navigation_interfaces::msg::NavigationGoal>::SharedPtr goal_subscription_;
  rclcpp::Publisher<mars_quadrotor_msgs::msg::PolynomialTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr heartbeat_callback_group_;

  std::mutex input_mutex_;
  std::shared_ptr<rog_map::PointCloud> latest_cloud_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<navigation_interfaces::msg::NavigationGoal> active_goal_;
  bool new_goal_{false};
  std::uint64_t dropped_cloud_count_{0};
  std::uint64_t received_cloud_count_{0};
  std::uint64_t accepted_cloud_count_{0};
  std::uint64_t map_update_exception_count_{0};
  std::int64_t last_input_conversion_us_{0};
  std::atomic_uint32_t trajectory_id_{0};
  std::uint64_t cycle_count_{0};
  std::atomic_uint64_t cycle_success_count_{0};
  std::atomic_uint64_t heartbeat_publish_count_{0};
  std::int64_t last_map_update_us_{0};
  std::int64_t last_planner_us_{0};
  std::atomic_int64_t last_publish_us_{0};
  std::int64_t last_input_lock_wait_us_{0};
  std::chrono::steady_clock::time_point metrics_log_time_{std::chrono::steady_clock::now()};
  std::vector<double> end_to_end_samples_ms_;

  ros_interface::RosInterface::Ptr ros_interface_;
  rog_map::ROGMapROS::Ptr map_;
  super_planner::SuperPlanner::Ptr planner_;
};

}  // namespace navigation_runtime
