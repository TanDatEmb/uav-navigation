#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <navigation_interfaces/msg/navigation_goal.hpp>
#include <navigation_interfaces/msg/navigation_mode_status.hpp>
#include <navigation_interfaces/msg/planned_trajectory.hpp>
#include <navigation_interfaces/msg/planned_trajectory_bundle.hpp>
#include <navigation_interfaces/msg/trajectory_bundle.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include <Eigen/Core>

#include "px4_navigation_external_mode/trajectory_contract.hpp"
#include "px4_navigation_external_mode/mission.hpp"
#include "px4_navigation_external_mode/mission_controller.hpp"
#include "px4_navigation_external_mode/velocity_tracker.hpp"

namespace px4_navigation_external_mode {

class NavigationMode final : public px4_ros2::ModeBase {
 public:
  explicit NavigationMode(rclcpp::Node& node);

  void setPositionControlHandover(std::function<void()> callback);

  enum class OutputMode { PositionVelocityAcceleration, PositionVelocity, Velocity };

  void onActivate() override;
  void onDeactivate() override;
  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter) override;
  void updateSetpoint(float dt_s) override;

 private:
  void onTrajectory(
      const navigation_interfaces::msg::PlannedTrajectory::ConstSharedPtr& message);
  void onTrajectoryBundle(
      const navigation_interfaces::msg::PlannedTrajectoryBundle::ConstSharedPtr& message);
  void onTrajectoryBundleV2(
      const navigation_interfaces::msg::TrajectoryBundle::ConstSharedPtr& message);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message);
  void onLioDiagnostics(
      const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& message);
  void onPlannerHeartbeat(const std_msgs::msg::Empty::ConstSharedPtr& message);
  void updateMission();
  void handleMissionEvent(const MissionControllerEvent& event, double now_s);
  void failNavigation(const char* reason);
  void logRuntimeMetrics(const rclcpp::Time& now);
  void publishStatus(std::uint8_t state, std::uint8_t reason,
                     const MissionControllerEvent* event = nullptr);

  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
  rclcpp::Subscription<navigation_interfaces::msg::PlannedTrajectoryBundle>::SharedPtr
      trajectory_bundle_subscription_;
  rclcpp::Subscription<navigation_interfaces::msg::TrajectoryBundle>::SharedPtr
      trajectory_bundle_v2_subscription_;
  rclcpp::Subscription<navigation_interfaces::msg::PlannedTrajectory>::SharedPtr
      trajectory_subscription_;
  rclcpp::Subscription<navigation_interfaces::msg::PlannedTrajectory>::SharedPtr
      trajectory_failure_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      lio_diagnostics_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr planner_heartbeat_subscription_;
  rclcpp::Publisher<navigation_interfaces::msg::NavigationGoal>::SharedPtr goal_publisher_;
  rclcpp::Publisher<navigation_interfaces::msg::NavigationModeStatus>::SharedPtr
      status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_publisher_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  std::mutex trajectory_mutex_;
  std::optional<navigation_interfaces::msg::PlannedTrajectory> trajectory_;
  std::optional<navigation_interfaces::msg::PlannedTrajectory> pending_trajectory_;
  std::optional<navigation_interfaces::msg::PlannedTrajectory> safety_backup_trajectory_;
  std::optional<navigation_interfaces::msg::PlannedTrajectory> pending_safety_backup_trajectory_;
  rclcpp::Time trajectory_start_time_;
  rclcpp::Time activation_time_;
  rclcpp::Time last_setpoint_time_;
  std::string trajectory_topic_;
  std::string trajectory_failure_topic_;
  std::string trajectory_bundle_topic_;
  std::string trajectory_bundle_v2_topic_;
  std::string goal_topic_;
  std::string planning_frame_;
  double stale_after_s_{0.5};
  double state_stale_after_s_{0.5};
  double trajectory_wait_timeout_s_{2.0};
  double trajectory_preview_s_{0.15};
  double velocity_only_fallback_position_error_m_{1.5};
  bool prefer_velocity_output_{true};
  // FAST-LIO can report one or two unhealthy registration updates while a
  // scan is being re-anchored.  Keep the last valid control stream through a
  // short, bounded burst; a sustained invalid state still fails closed.
  double lio_health_grace_s_{1.0};
  std::optional<nav_msgs::msg::Odometry> odometry_;
  bool lio_health_valid_{false};
  std::int64_t last_lio_diagnostics_ns_{0};
  std::int64_t lio_unhealthy_since_ns_{0};
  std::optional<Mission> mission_;
  std::unique_ptr<MissionController> mission_controller_;
  VelocityTracker velocity_tracker_;
  OutputMode output_mode_{OutputMode::PositionVelocityAcceleration};
  bool velocity_only_fallback_active_{false};
  std::function<void()> position_control_handover_;
  bool failure_reported_{false};
  bool mode_active_{false};
  bool mission_terminal_{false};
  bool handover_requested_{false};
  std::optional<Eigen::Vector3d> completion_position_;
  bool mission_complete_published_{false};
  std::uint8_t last_status_state_{navigation_interfaces::msg::NavigationModeStatus::PAUSED};
  bool accepted_world_identity_valid_{false};
  std::uint64_t accepted_world_generation_{0U};
  std::uint64_t accepted_world_revision_{0U};
  std::uint64_t odometry_callback_count_{0U};
  std::uint64_t trajectory_received_count_{0U};
  std::uint64_t trajectory_accepted_count_{0U};
  std::uint64_t trajectory_rejected_count_{0U};
  std::uint64_t setpoint_update_count_{0U};
  std::uint64_t stale_state_failure_count_{0U};
  std::int64_t last_odometry_receive_ns_{0};
  std::int64_t last_trajectory_receive_ns_{0};
  std::int64_t last_planner_heartbeat_ns_{0};
  std::int64_t maximum_odometry_callback_gap_us_{0};
  std::int64_t last_setpoint_update_ns_{0};
  std::int64_t maximum_setpoint_callback_gap_us_{0};
  std::int64_t last_metrics_log_ns_{0};
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

  rclcpp::Node& node_;
};

}  // namespace px4_navigation_external_mode
