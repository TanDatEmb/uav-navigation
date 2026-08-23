#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <navigation_interfaces/msg/navigation_goal.hpp>
#include <navigation_interfaces/msg/navigation_mode_status.hpp>
#include <mars_quadrotor_msgs/msg/position_command.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
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

  void setPositionControlHandover(std::function<void()> callback);
  std::optional<Eigen::Vector3d> handoverPosition();

  enum class OutputMode { PositionVelocityAcceleration, PositionVelocity, Velocity };

  void onActivate() override;
  void onDeactivate() override;
  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter) override;
  void updateSetpoint(float dt_s) override;

 private:
  rclcpp::Node& node_;
  void onSuperCommand(
      const mars_quadrotor_msgs::msg::PositionCommand::ConstSharedPtr& message);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message);
  void onLioDiagnostics(
      const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& message);
  void updateMission();
  void handleMissionEvent(const MissionControllerEvent& event, double now_s);
  void safetyStopNavigation(const char* reason);
  void failNavigation(const char* reason);
  void logRuntimeMetrics(const rclcpp::Time& now);
  void publishStatus(std::uint8_t state, std::uint8_t reason,
                     const MissionControllerEvent* event = nullptr);

  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr
      super_command_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      lio_diagnostics_subscription_;
  rclcpp::Publisher<navigation_interfaces::msg::NavigationGoal>::SharedPtr goal_publisher_;
  rclcpp::Publisher<navigation_interfaces::msg::NavigationModeStatus>::SharedPtr
      status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_publisher_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  std::mutex trajectory_mutex_;
  rclcpp::Time activation_time_;
  rclcpp::Time last_setpoint_time_;
  std::string super_command_topic_;
  std::string goal_topic_;
  std::string planning_frame_;
  double stale_after_s_{0.5};
  double command_anchor_max_error_m_{2.0};
  double state_stale_after_s_{0.5};
  double trajectory_wait_timeout_s_{2.0};
  bool prefer_velocity_output_{true};
  // FAST-LIO can report one or two unhealthy registration updates while a
  // scan is being re-anchored.  Keep the last valid control stream through a
  // short, bounded burst; a sustained invalid state still fails closed.
  double lio_health_grace_s_{1.0};
  std::optional<nav_msgs::msg::Odometry> odometry_;
  std::optional<mars_quadrotor_msgs::msg::PositionCommand> super_command_;
  bool lio_health_valid_{false};
  std::int64_t last_lio_diagnostics_ns_{0};
  std::int64_t lio_unhealthy_since_ns_{0};
  std::optional<Mission> mission_;
  std::unique_ptr<MissionController> mission_controller_;
  OutputMode output_mode_{OutputMode::PositionVelocityAcceleration};
  std::function<void()> position_control_handover_;
  bool failure_reported_{false};
  bool mode_active_{false};
  bool mission_terminal_{false};
  bool handover_requested_{false};
  std::optional<Eigen::Vector3d> completion_position_;
  std::optional<Eigen::Vector3d> safety_hold_position_;
  bool mission_complete_published_{false};
  std::uint8_t last_status_state_{navigation_interfaces::msg::NavigationModeStatus::PAUSED};
  std::uint64_t odometry_callback_count_{0U};
  std::uint64_t trajectory_received_count_{0U};
  std::uint64_t trajectory_accepted_count_{0U};
  std::uint64_t trajectory_rejected_count_{0U};
  std::uint64_t setpoint_update_count_{0U};
  std::uint64_t stale_state_failure_count_{0U};
  std::int64_t last_odometry_receive_ns_{0};
  std::int64_t last_goal_publish_ns_{0};
  std::int64_t last_command_receive_ns_{0};
  std::int64_t maximum_odometry_callback_gap_us_{0};
  std::int64_t last_setpoint_update_ns_{0};
  std::int64_t maximum_setpoint_callback_gap_us_{0};
  std::int64_t last_metrics_log_ns_{0};
  std::int64_t last_super_debug_log_ns_{0};
  double last_state_age_s_{-1.0};
  Eigen::Vector3d last_velocity_command_enu_{Eigen::Vector3d::Zero()};
  std::uint64_t last_forward_guard_count_{0U};
};

class NavigationHoldMode final : public px4_ros2::ModeBase {
 public:
  explicit NavigationHoldMode(rclcpp::Node& node);
  void setHoldPosition(const Eigen::Vector3d& position_enu);
  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;

 private:
  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
  std::mutex target_mutex_;
  std::optional<Eigen::Vector3d> target_enu_;
};

class NavigationModeExecutor final : public px4_ros2::ModeExecutorBase {
 public:
  NavigationModeExecutor(px4_ros2::ModeBase& owned_mode, NavigationHoldMode& hold_mode);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;
  void onFailsafeDeferred() override;

 private:
  void onOwnedModeCompleted(px4_ros2::Result result);
  void onPositionControlHandoverCompleted(px4_ros2::Result result,
                                          bool complete_navigation_failure);
  void onHoldHandoverCompleted(px4_ros2::Result result,
                               bool complete_navigation_failure);
  void scheduleExternalHold(bool complete_navigation_failure);

  rclcpp::Node& node_;
  NavigationMode& navigation_mode_;
  NavigationHoldMode& hold_mode_;
};

}  // namespace px4_navigation_external_mode
