#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <navigation_interfaces/msg/planned_trajectory.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "px4_navigation_external_mode/trajectory_contract.hpp"

namespace px4_navigation_external_mode {

class NavigationMode final : public px4_ros2::ModeBase {
 public:
  explicit NavigationMode(rclcpp::Node& node);

  void onActivate() override;
  void onDeactivate() override;
  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter& reporter) override;
  void updateSetpoint(float dt_s) override;

 private:
  void onTrajectory(
      const navigation_interfaces::msg::PlannedTrajectory::ConstSharedPtr& message);
  void failNavigation(const char* reason);

  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
  rclcpp::Subscription<navigation_interfaces::msg::PlannedTrajectory>::SharedPtr
      trajectory_subscription_;
  std::mutex trajectory_mutex_;
  std::optional<navigation_interfaces::msg::PlannedTrajectory> trajectory_;
  rclcpp::Time trajectory_start_time_;
  rclcpp::Time activation_time_;
  std::string trajectory_topic_;
  std::string planning_frame_;
  double stale_after_s_{0.5};
  bool failure_reported_{false};
};

class NavigationModeExecutor final : public px4_ros2::ModeExecutorBase {
 public:
  explicit NavigationModeExecutor(px4_ros2::ModeBase& owned_mode);

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;
  void onFailsafeDeferred() override;

 private:
  void scheduleOwnedMode(px4_ros2::Result previous_result);

  rclcpp::Node& node_;
};

}  // namespace px4_navigation_external_mode
