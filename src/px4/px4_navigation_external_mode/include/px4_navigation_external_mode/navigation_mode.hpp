#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <navigation_contracts/msg/navigation_goal.hpp>
#include <navigation_contracts/msg/navigation_mode_status.hpp>
#include <navigation_contracts/msg/propagated_odometry.hpp>
#include <navigation_contracts/msg/estimator_health.hpp>
#include <navigation_contracts/msg/navigation_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Core>

#include "px4_navigation_external_mode/mission.hpp"
#include "px4_navigation_external_mode/mission_controller.hpp"

namespace px4_navigation_external_mode {

class NavigationMode final : public px4_ros2::ModeBase {
 public:
  explicit NavigationMode(rclcpp::Node& node);

  void setPx4HoldHandover(std::function<void()> callback);

  void onActivate() override;
  void onDeactivate() override;
  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter) override;
  void updateSetpoint(float dt_s) override;

 private:
  rclcpp::Node& node_;
  void onNavigationCommand(
      const navigation_contracts::msg::NavigationCommand::ConstSharedPtr& message);
  void onOdometry(
      const navigation_contracts::msg::PropagatedOdometry::ConstSharedPtr& message);
  void onEstimatorHealth(
      const navigation_contracts::msg::EstimatorHealth::ConstSharedPtr& message);
  void updateMission();
  void handleMissionEvent(const MissionControllerEvent& event, double now_s);
  void safetyStopNavigation(const char* reason);
  void failNavigation(const char* reason);
  void logRuntimeMetrics(const rclcpp::Time& now);
  void publishStatus(std::uint8_t state, std::uint8_t reason,
                     const MissionControllerEvent* event = nullptr);

  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
  rclcpp::Subscription<navigation_contracts::msg::PropagatedOdometry>::SharedPtr
      odometry_subscription_;
  rclcpp::Subscription<navigation_contracts::msg::NavigationCommand>::SharedPtr
      navigation_command_subscription_;
  rclcpp::Subscription<navigation_contracts::msg::EstimatorHealth>::SharedPtr
      estimator_health_subscription_;
  rclcpp::Publisher<navigation_contracts::msg::NavigationGoal>::SharedPtr goal_publisher_;
  rclcpp::Publisher<navigation_contracts::msg::NavigationModeStatus>::SharedPtr
      status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_publisher_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  std::mutex trajectory_mutex_;
  rclcpp::Time activation_time_;
  rclcpp::Time last_setpoint_time_;
  std::string navigation_command_topic_;
  std::string goal_topic_;
  std::string planning_frame_;
  std::string body_frame_;
  double stale_after_s_{0.5};
  double state_stale_after_s_{0.5};
  double trajectory_wait_timeout_s_{2.0};
  double planner_recovery_wait_timeout_s_{0.5};
  std::int64_t stale_after_ns_{0};
  std::int64_t state_stale_after_ns_{0};
  std::int64_t planner_recovery_wait_timeout_ns_{0};
  std::optional<nav_msgs::msg::Odometry> odometry_;
  std::optional<navigation_contracts::msg::NavigationCommand> navigation_command_;
  bool lio_health_valid_{false};
  bool typed_health_seen_{false};
  std::uint8_t last_health_state_{0U};
  bool last_health_navigation_valid_{false};
  bool last_health_covariance_valid_{false};
  bool last_health_observability_valid_{false};
  bool last_health_correction_fresh_{false};
  bool last_health_propagation_valid_{false};
  std::uint64_t lio_localization_epoch_{0U};
  std::int64_t last_propagated_state_stamp_ns_{0};
  std::uint64_t last_propagated_state_sequence_{0U};
  std::int64_t last_lio_diagnostics_ns_{0};
  std::int64_t last_health_source_stamp_ns_{0};
  std::optional<Mission> mission_;
  std::unique_ptr<MissionController> mission_controller_;
  std::function<void()> px4_hold_handover_;
  bool failure_reported_{false};
  bool mode_active_{false};
  bool mission_terminal_{false};
  bool handover_requested_{false};
  bool planner_recovery_pending_{false};
  std::int64_t planner_recovery_deadline_ns_{0};
  std::uint32_t last_completed_waypoint_index_{0U};
  std::uint64_t last_completed_request_id_{0U};
  std::optional<Eigen::Vector3d> completion_position_;
  std::optional<Eigen::Vector3d> safety_hold_position_;
  bool mission_complete_published_{false};
  std::uint8_t last_status_state_{navigation_contracts::msg::NavigationModeStatus::PAUSED};
  std::uint64_t odometry_callback_count_{0U};
  std::uint64_t trajectory_received_count_{0U};
  std::uint64_t trajectory_accepted_count_{0U};
  std::uint64_t trajectory_rejected_count_{0U};
  std::uint64_t waypoint_handoff_retained_command_count_{0U};
  std::uint64_t setpoint_update_count_{0U};
  std::uint64_t stale_state_failure_count_{0U};
  std::int64_t last_odometry_receive_ns_{0};
  std::int64_t last_odometry_receive_steady_ns_{0};
  std::int64_t last_goal_publish_ns_{0};
  std::int64_t last_command_receive_ns_{0};
  std::int64_t maximum_odometry_callback_gap_us_{0};
  std::int64_t last_setpoint_update_ns_{0};
  std::int64_t maximum_setpoint_callback_gap_us_{0};
  std::int64_t last_metrics_log_ns_{0};
  std::int64_t last_planner_debug_log_ns_{0};
  double last_state_age_s_{-1.0};
  Eigen::Vector3d last_velocity_command_enu_{Eigen::Vector3d::Zero()};
  std::uint64_t last_forward_guard_count_{0U};
};

class NavigationModeExecutor final : public px4_ros2::ModeExecutorBase {
 public:
  explicit NavigationModeExecutor(px4_ros2::ModeBase& owned_mode);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;
  void onFailsafeDeferred() override;

 private:
  void onOwnedModeCompleted(px4_ros2::Result result);
  void schedulePx4Hold(bool complete_navigation_failure);
  void onPx4HoldHandoverCompleted(px4_ros2::Result result,
                                  bool complete_navigation_failure);

  rclcpp::Node& node_;
  NavigationMode& navigation_mode_;
};

}  // namespace px4_navigation_external_mode
