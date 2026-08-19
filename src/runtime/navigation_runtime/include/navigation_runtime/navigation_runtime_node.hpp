#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navigation_interfaces/msg/lidar_mapping_observation.hpp>
#include <navigation_interfaces/msg/navigation_goal.hpp>
#include <navigation_interfaces/msg/planned_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>

#include "navigation_mapping/mapping_pipeline.hpp"
#include "navigation_planning/planner.hpp"
#include "navigation_planning/trajectory_verifier.hpp"
#include "navigation_runtime/local_goal_selector.hpp"

namespace navigation_runtime {

// Single ROS composition boundary. It owns the only MappingPipeline/WorldModel
// instance and is the only subscription path into the product map.
class NavigationRuntimeNode : public rclcpp::Node {
 public:
  explicit NavigationRuntimeNode(const rclcpp::NodeOptions& options = {});

 private:
  void onObservation(
      const navigation_interfaces::msg::LidarMappingObservation::ConstSharedPtr& message);
  void publishDiagnostics();
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message);
  void onGoal(const navigation_interfaces::msg::NavigationGoal::ConstSharedPtr& message);
  void planActiveGoal();
  [[nodiscard]] const char* localGoalStatusName(LocalGoalSelectionStatus status) const noexcept;
  void publishPlanningDiagnostics(const navigation_planning::PlanResult& result);
  navigation_interfaces::msg::PlannedTrajectory makeTrajectoryMessage(
      const navigation_planning::PlanResult& result,
      const navigation_interfaces::msg::NavigationGoal& goal) const;
  nav_msgs::msg::Path makePathMessage(const navigation_planning::PlanResult& result,
                                      const std_msgs::msg::Header& header) const;
  void publishMapVisualization();
  sensor_msgs::msg::PointCloud2 makePointCloud(
      const rog_map::vec_E<rog_map::Vec3f>& points,
      const builtin_interfaces::msg::Time& stamp) const;

  std::unique_ptr<navigation_mapping::MappingPipeline> pipeline_;
  rclcpp::Subscription<navigation_interfaces::msg::LidarMappingObservation>::SharedPtr
      observation_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr planning_diagnostics_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr planner_heartbeat_publisher_;
  rclcpp::Publisher<navigation_interfaces::msg::PlannedTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<navigation_interfaces::msg::NavigationGoal>::SharedPtr goal_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_occupied_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::CallbackGroup::SharedPtr mapping_callback_group_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<navigation_interfaces::msg::NavigationGoal> active_goal_;
  double state_max_age_s_{0.5};
  std::string state_topic_;
  std::string planning_frame_id_;
  double replan_rate_hz_{5.0};
  double replan_tracking_error_m_{0.5};
  double local_goal_boundary_margin_m_{1.0};
  double local_goal_max_distance_m_{5.0};
  double local_goal_switch_distance_m_{0.8};
  double local_goal_continuation_speed_fraction_{0.35};
  bool local_subgoal_enabled_{true};
  navigation_planning::PlannerConfig planner_config_{};
  navigation_planning::Planner planner_;
  navigation_planning::TrajectoryVerifier trajectory_verifier_;
  std::uint64_t plan_count_{0};
  std::uint64_t plan_skip_count_{0};
  std::uint64_t plan_success_count_{0};
  std::uint64_t safety_fallback_count_{0};
  std::uint64_t safety_route_plan_count_{0};
  std::uint64_t safety_route_verified_count_{0};
  std::uint64_t safety_route_selected_count_{0};
  std::uint64_t safety_stop_selected_count_{0};
  std::uint64_t nominal_plan_count_{0};
  std::uint64_t nominal_selected_count_{0};
  std::uint64_t dual_verification_failure_count_{0};
  std::uint64_t plan_failure_count_{0};
  std::uint64_t verification_failure_count_{0};
  std::uint64_t local_subgoal_selected_count_{0};
  std::uint64_t local_subgoal_failure_count_{0};
  std::uint64_t trajectory_revalidation_count_{0};
  std::uint64_t trajectory_revalidation_failure_count_{0};
  std::uint64_t trajectory_reuse_count_{0};
  std::uint64_t full_replan_count_{0};
  bool last_local_subgoal_selected_{false};
  bool last_goal_terminal_{true};
  navigation_mapping::Vec3 last_terminal_velocity_{navigation_mapping::Vec3::Zero()};
  LocalGoalSelectionStatus last_local_goal_status_{LocalGoalSelectionStatus::InvalidState};
  navigation_mapping::Vec3 last_effective_goal_{navigation_mapping::Vec3::Zero()};
  std::optional<navigation_mapping::Vec3> committed_local_goal_;
  std::int64_t last_verification_time_us_{0};
  navigation_planning::PlanFailureCode last_failure_code_{navigation_planning::PlanFailureCode::None};
  navigation_planning::PlanFailureCode last_nominal_failure_code_{
      navigation_planning::PlanFailureCode::None};
  navigation_planning::PlanFailureCode last_safety_failure_code_{
      navigation_planning::PlanFailureCode::None};
  navigation_planning::VerificationFailureCode last_nominal_verification_failure_{
      navigation_planning::VerificationFailureCode::None};
  navigation_planning::VerificationFailureCode last_safety_verification_failure_{
      navigation_planning::VerificationFailureCode::None};
  navigation_planning::VerificationFailureCode last_dual_verification_failure_{
      navigation_planning::VerificationFailureCode::None};
  bool last_plan_identity_valid_{false};
  std::string last_planned_mission_id_;
  std::uint32_t last_planned_waypoint_index_{0};
  std::uint64_t last_planned_request_id_{0};
  std::uint64_t last_planned_world_generation_{0};
  std::uint64_t last_planned_world_revision_{0};
  bool last_plan_success_{false};
  // A verified braking stop remains latched until a newer goal/request is
  // accepted.  This survives transient failed publication/revalidation ticks
  // and prevents the planner from falling back to nominal execution after a
  // safety handover has already started.
  bool braking_stop_latched_{false};
  std::int64_t last_plan_time_ns_{0};
  double last_planned_duration_s_{0.0};
  std::optional<navigation_planning::TimeParameterizedTrajectory> last_planned_trajectory_;
  navigation_planning::PlanRole last_planned_role_{navigation_planning::PlanRole::Committed};
  navigation_planning::SafetyPlanKind last_planned_safety_kind_{
      navigation_planning::SafetyPlanKind::None};
  std::string last_replan_reason_{"initial"};
  std::uint64_t invalid_cloud_count_{0};
  bool visualization_enabled_{false};
  bool publish_unknown_{false};
  bool publish_frontier_{false};
  double visualization_range_x_m_{15.0};
  double visualization_range_y_m_{15.0};
  double visualization_range_z_m_{6.0};
  std::size_t visualization_max_points_{150000};
  std::string visualization_frame_id_;
  std::uint64_t last_visualization_update_count_{0};
};

}  // namespace navigation_runtime
