#pragma once

#include <atomic>
#include <cstdint>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <navigation_interfaces/msg/navigation_goal.hpp>
#include <navigation_interfaces/msg/navigation_mode_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>

#include "navigation_runtime/input_pairing.hpp"
#include "navigation_runtime/planner_fsm.hpp"
#include <ros_interface/ros_interface.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <super_core/super_planner.h>

namespace navigation_runtime {

// Product ROS boundary for the imported SUPER core. The cloud is retained as
// one pending observation and odometry is retained as a short timestamped
// history; planning consumes only a compatible cloud/odometry pair.
class SuperNavigationNode final : public rclcpp::Node {
 public:
  explicit SuperNavigationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

 private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message);
  void onGoal(const navigation_interfaces::msg::NavigationGoal::ConstSharedPtr& message);
  void onModeStatus(
      const navigation_interfaces::msg::NavigationModeStatus::ConstSharedPtr& message);
  void runCycle();
  void publishCommand();

  static bool decodeCloud(const sensor_msgs::msg::PointCloud2& message,
                          rog_map::PointCloud& output);
  static builtin_interfaces::msg::Time rosTimeFromSeconds(double seconds);
  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time& stamp);
  std::string cloud_topic_;
  std::string odometry_topic_;
  std::string goal_topic_;
  std::string status_topic_;
  std::string command_topic_;
  std::string super_config_path_;
  std::string planning_frame_;
  double planner_rate_hz_{10.0};
  double command_rate_hz_{50.0};
  double input_pair_max_skew_s_{0.1};
  double input_max_age_s_{0.5};
  double max_safety_suffix_anchor_error_m_{0.75};
  double planner_solve_timeout_s_{1.0};
  double plan_from_rest_failure_confirmation_s_{0.5};
  std::uint32_t max_plan_from_rest_failures_{3U};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<navigation_interfaces::msg::NavigationGoal>::SharedPtr goal_subscription_;
  rclcpp::Subscription<navigation_interfaces::msg::NavigationModeStatus>::SharedPtr
      status_subscription_;
  rclcpp::Publisher<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;

  std::mutex input_mutex_;
  struct StampedCloud {
    std::shared_ptr<rog_map::PointCloud> cloud;
    std::int64_t stamp_ns{0};
  };
  std::optional<StampedCloud> latest_cloud_;
  std::deque<nav_msgs::msg::Odometry> odometry_history_;
  std::optional<navigation_interfaces::msg::NavigationGoal> active_goal_;
  bool new_goal_{false};
  // PASS_THROUGH waypoint transitions retarget SUPER through ReplanOnce so the
  // committed polynomial supplies the future PVA initial state.
  bool hot_goal_transition_{false};
  bool restart_from_rest_{false};
  // SUPER's native FSM skips one replan timer callback immediately after a
  // successful PlanFromRest.  Keep that state at the ROS adapter boundary so
  // the first hot replan is not run against a trajectory that has just been
  // committed.
  bool skip_replan_once_{false};
  ConsecutiveFailureBudget plan_from_rest_failure_budget_{3U};
  std::int64_t plan_from_rest_first_failure_steady_ns_{0};
  std::uint64_t dropped_cloud_count_{0};
  std::uint64_t received_cloud_count_{0};
  std::uint64_t accepted_cloud_count_{0};
  std::uint64_t stale_input_count_{0};
  std::uint64_t map_update_exception_count_{0};
  std::int64_t last_input_conversion_us_{0};
  std::atomic_uint32_t command_id_{0};
  std::atomic_bool planner_command_available_{false};
  std::atomic_bool planner_failure_latched_{false};
  std::atomic_bool safety_suffix_active_{false};
  std::atomic_uint64_t planner_solve_generation_{0U};
  std::atomic_uint64_t active_planner_solve_generation_{0U};
  std::atomic_uint64_t timed_out_planner_solve_generation_{0U};
  std::atomic_int64_t planner_solve_started_steady_ns_{0};
  // A local SUPER trajectory can end at a known-free frontier before the
  // mission goal.  The FSM must call PlanFromRest again after that trajectory
  // finishes instead of holding the completed old trajectory forever.
  std::atomic_bool trajectory_finished_{false};
  std::atomic_bool trajectory_reaches_goal_{false};
  std::uint64_t cycle_count_{0};
  std::atomic_uint64_t cycle_success_count_{0};
  std::atomic_uint64_t command_publish_count_{0};
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
