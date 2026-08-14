#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navigation_interfaces/msg/lidar_mapping_observation.hpp>
#include <navigation_interfaces/msg/planned_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "navigation_mapping/mapping_pipeline.hpp"
#include "navigation_planning/planner.hpp"

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
  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& message);
  void publishPlanningDiagnostics(const navigation_planning::PlanResult& result);
  navigation_interfaces::msg::PlannedTrajectory makeTrajectoryMessage(
      const navigation_planning::PlanResult& result,
      const std_msgs::msg::Header& header) const;
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
  rclcpp::Publisher<navigation_interfaces::msg::PlannedTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_occupied_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::CallbackGroup::SharedPtr mapping_callback_group_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  double state_max_age_s_{0.5};
  std::string state_topic_;
  std::string planning_frame_id_;
  navigation_planning::Planner planner_;
  std::uint64_t plan_count_{0};
  std::uint64_t plan_success_count_{0};
  std::uint64_t plan_failure_count_{0};
  navigation_planning::PlanFailureCode last_failure_code_{navigation_planning::PlanFailureCode::None};
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
