#pragma once

#include <cstdint>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navigation_interfaces/msg/lidar_mapping_observation.hpp>
#include <navigation_interfaces/msg/navigation_goal.hpp>
#include <navigation_interfaces/msg/planned_trajectory.hpp>
#include <navigation_interfaces/msg/planned_trajectory_bundle.hpp>
#include <navigation_interfaces/msg/trajectory_bundle.hpp>
#include <navigation_interfaces/msg/planner_cycle_trace.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "navigation_mapping/mapping_pipeline.hpp"
#include "navigation_planning/planner.hpp"
#include "navigation_planning/horizon_policy.hpp"
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
      const navigation_interfaces::msg::NavigationGoal& goal);
  navigation_interfaces::msg::TrajectoryCandidate makeTrajectoryCandidateMessage(
      const navigation_planning::PlanResult& result,
      const navigation_interfaces::msg::NavigationGoal& goal,
      const builtin_interfaces::msg::Time& valid_from,
      std::uint64_t trajectory_id,
      std::uint64_t parent_trajectory_id) const;
  navigation_interfaces::msg::PlannedTrajectoryBundle makeTrajectoryBundleMessage(
      const navigation_planning::PlanResult& selected,
      const navigation_planning::PlanResult& nominal,
      const navigation_planning::PlanResult& safety,
      const navigation_interfaces::msg::NavigationGoal& goal,
      const builtin_interfaces::msg::Time& valid_from,
      std::uint64_t bundle_id,
      std::uint64_t parent_bundle_id) const;
  navigation_interfaces::msg::TrajectoryBundle makeTrajectoryBundleV2Message(
      const navigation_planning::PlanResult& selected,
      const navigation_planning::PlanResult& nominal,
      const navigation_planning::PlanResult& safety,
      const navigation_interfaces::msg::NavigationGoal& goal,
      const builtin_interfaces::msg::Time& valid_from,
      std::uint64_t bundle_id,
      std::uint64_t parent_bundle_id) const;
  nav_msgs::msg::Path makePathMessage(const navigation_planning::PlanResult& result,
                                      const std_msgs::msg::Header& header) const;
  void publishPlanningPaths(
      const navigation_planning::PlanResult& selected,
      const navigation_planning::PlanResult& nominal,
      const navigation_planning::PlanResult& safety,
      const std_msgs::msg::Header& header,
      const std::optional<navigation_planning::TimeParameterizedTrajectory>&
          full_nominal_trajectory);
  void publishMapVisualization();
  void publishNavigationVisualization(
      const navigation_planning::PlanResult& result,
      const navigation_interfaces::msg::NavigationGoal& goal);
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
  rclcpp::Publisher<navigation_interfaces::msg::PlannedTrajectoryBundle>::SharedPtr
      trajectory_bundle_publisher_;
  rclcpp::Publisher<navigation_interfaces::msg::TrajectoryBundle>::SharedPtr
      trajectory_bundle_v2_publisher_;
  rclcpp::Publisher<navigation_interfaces::msg::PlannerCycleTrace>::SharedPtr
      planner_trace_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_nominal_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_safety_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_selected_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr navigation_marker_publisher_;
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
  rclcpp::CallbackGroup::SharedPtr ingress_callback_group_;
  mutable std::mutex input_mutex_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<navigation_mapping::Vec3> latest_acceleration_;
  std::optional<navigation_mapping::Vec3> previous_velocity_;
  std::int64_t previous_odometry_stamp_ns_{0};
  std::optional<navigation_interfaces::msg::NavigationGoal> active_goal_;
  double state_max_age_s_{0.5};
  std::string state_topic_;
  std::string planning_frame_id_;
  double replan_rate_hz_{5.0};
  double replan_tracking_error_m_{0.5};
  double switch_delay_s_{0.12};
  double safety_latency_s_{0.08};
  double safety_stop_margin_m_{0.25};
  double safety_visibility_horizon_m_{15.0};
  double local_goal_boundary_margin_m_{1.0};
  // Use the observable forward map depth as the default rolling horizon.  A
  // short fixed value turns every visible obstacle into an artificial goal.
  double local_goal_max_distance_m_{15.0};
  double local_goal_switch_distance_m_{0.8};
  double local_goal_continuation_speed_fraction_{0.35};
  double planning_horizon_min_distance_m_{10.0};
  double planning_horizon_max_distance_m_{30.0};
  double planning_horizon_preview_time_s_{5.0};
  double planning_horizon_boundary_margin_m_{2.0};
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
  std::uint64_t horizon_endpoint_change_count_{0};
  std::uint64_t horizon_endpoint_repeat_count_{0};
  std::uint64_t horizon_backward_rejection_count_{0};
  std::uint64_t trajectory_revalidation_count_{0};
  std::uint64_t trajectory_revalidation_failure_count_{0};
  std::uint64_t trajectory_reuse_count_{0};
  std::uint64_t full_replan_count_{0};
  bool last_local_subgoal_selected_{false};
  bool last_goal_terminal_{true};
  navigation_mapping::Vec3 last_terminal_velocity_{navigation_mapping::Vec3::Zero()};
  LocalGoalSelectionStatus last_local_goal_status_{LocalGoalSelectionStatus::InvalidState};
  navigation_mapping::Vec3 last_effective_goal_{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 last_horizon_tangent_{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 last_planning_state_position_{navigation_mapping::Vec3::Zero()};
  navigation_mapping::Vec3 last_planning_state_velocity_{navigation_mapping::Vec3::Zero()};
  double last_horizon_forward_projection_m_{0.0};
  double last_planning_horizon_distance_m_{0.0};
  double last_horizon_progress_m_{0.0};
  bool last_horizon_ray_occupied_{false};
  double last_adaptive_velocity_cap_mps_{0.0};
  double last_known_free_horizon_m_{0.0};
  double last_splice_position_residual_m_{0.0};
  double last_splice_velocity_residual_mps_{0.0};
  double last_splice_acceleration_residual_mps2_{0.0};
  double last_splice_jerk_residual_mps3_{0.0};
  double last_splice_snap_residual_mps4_{0.0};
  std::optional<navigation_mapping::Vec3> committed_local_goal_;
  // Keep the receding-horizon leg on one altitude.  A voxel selector may
  // otherwise alternate between adjacent known-free z cells as the rolling
  // map is updated, which turns harmless grid quantisation into a real
  // altitude oscillation on a long mission.
  std::optional<double> committed_local_altitude_m_;
  std::int64_t last_verification_time_us_{0};
  navigation_planning::PlanFailureCode last_failure_code_{navigation_planning::PlanFailureCode::None};
  navigation_planning::PlanFailureCode last_nominal_failure_code_{
      navigation_planning::PlanFailureCode::None};
  navigation_planning::PlanFailureCode last_safety_failure_code_{
      navigation_planning::PlanFailureCode::None};
  std::uint64_t last_nominal_raw_path_node_count_{0};
  std::uint64_t last_nominal_corridor_checked_count_{0};
  std::uint64_t last_nominal_corridor_blocked_count_{0};
  bool last_nominal_collision_free_{false};
  bool last_nominal_dynamic_limits_satisfied_{false};
  double last_nominal_duration_s_{0.0};
  double last_nominal_max_velocity_mps_{0.0};
  double last_nominal_max_acceleration_mps2_{0.0};
  double last_nominal_max_deceleration_mps2_{0.0};
  double last_nominal_max_jerk_mps3_{0.0};
  double last_nominal_geometric_path_length_m_{0.0};
  double last_nominal_trajectory_length_m_{0.0};
  double last_nominal_objective_cost_{0.0};
  std::uint64_t last_nominal_collision_check_failure_count_{0};
  navigation_mapping::Vec3 last_nominal_first_collision_position_{
      navigation_mapping::Vec3::Zero()};
  navigation_planning::PlanFailureCode last_safety_route_failure_code_{
      navigation_planning::PlanFailureCode::None};
  navigation_mapping::Vec3 last_safety_goal_position_{navigation_mapping::Vec3::Zero()};
  bool last_safety_goal_known_free_{false};
  std::uint64_t last_safety_raw_path_node_count_{0};
  std::uint64_t last_safety_corridor_checked_count_{0};
  std::uint64_t last_safety_corridor_blocked_count_{0};
  bool last_safety_collision_free_{false};
  bool last_safety_dynamic_limits_satisfied_{false};
  double last_safety_duration_s_{0.0};
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
  std::atomic_bool braking_stop_latched_{false};
  std::int64_t last_plan_time_ns_{0};
  double plan_valid_from_delay_s_{0.0};
  std::uint64_t next_trajectory_id_{1U};
  std::uint64_t last_trajectory_id_{0U};
  std::uint64_t next_bundle_id_{1U};
  std::uint64_t last_bundle_id_{0U};
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
