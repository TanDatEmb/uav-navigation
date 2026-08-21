#include "navigation_runtime/navigation_runtime_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "navigation_mapping/world_model.hpp"
#include "navigation_planning/planner.hpp"

namespace navigation_runtime {
namespace {

diagnostic_msgs::msg::KeyValue keyValue(std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

std::int64_t timeNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

const char* failureName(navigation_planning::PlanFailureCode code) {
  using navigation_planning::PlanFailureCode;
  switch (code) {
    case PlanFailureCode::None: return "none";
    case PlanFailureCode::WorldModelUnavailable: return "world_model_unavailable";
    case PlanFailureCode::ClearanceUnavailable: return "clearance_unavailable";
    case PlanFailureCode::InvalidState: return "invalid_state";
    case PlanFailureCode::StartOutsideBounds: return "start_outside_bounds";
    case PlanFailureCode::GoalOutsideBounds: return "goal_outside_bounds";
    case PlanFailureCode::StartOccupied: return "start_occupied";
    case PlanFailureCode::GoalOccupied: return "goal_occupied";
    case PlanFailureCode::NoPath: return "no_path";
    case PlanFailureCode::CorridorInfeasible: return "corridor_infeasible";
    case PlanFailureCode::DynamicLimitsInfeasible: return "dynamic_limits_infeasible";
    case PlanFailureCode::TrajectoryInvalid: return "trajectory_invalid";
    case PlanFailureCode::WorldChanged: return "world_changed";
    case PlanFailureCode::SafetyStopUnavailable: return "safety_stop_unavailable";
  }
  return "unknown";
}

const char* roleName(navigation_planning::PlanRole role) {
  switch (role) {
    case navigation_planning::PlanRole::Nominal: return "nominal";
    case navigation_planning::PlanRole::Safety: return "safety";
    case navigation_planning::PlanRole::Committed: return "committed";
  }
  return "unknown";
}

const char* safetyKindName(navigation_planning::SafetyPlanKind kind) {
  using navigation_planning::SafetyPlanKind;
  switch (kind) {
    case SafetyPlanKind::None: return "none";
    case SafetyPlanKind::Route: return "route";
    case SafetyPlanKind::BrakingStop: return "braking_stop";
  }
  return "unknown";
}

const char* localGoalStatusName(navigation_runtime::LocalGoalSelectionStatus status) {
  switch (status) {
    case navigation_runtime::LocalGoalSelectionStatus::Direct: return "direct";
    case navigation_runtime::LocalGoalSelectionStatus::LocalSubGoal: return "local_subgoal";
    case navigation_runtime::LocalGoalSelectionStatus::InvalidState: return "invalid_state";
    case navigation_runtime::LocalGoalSelectionStatus::StartOutsideBounds:
      return "start_outside_bounds";
    case navigation_runtime::LocalGoalSelectionStatus::NoUsableSubGoal:
      return "no_usable_subgoal";
  }
  return "unknown";
}

const char* verificationFailureName(navigation_planning::VerificationFailureCode code) {
  using navigation_planning::VerificationFailureCode;
  switch (code) {
    case VerificationFailureCode::None: return "none";
    case VerificationFailureCode::WorldModelUnavailable: return "world_model_unavailable";
    case VerificationFailureCode::InvalidTrajectory: return "invalid_trajectory";
    case VerificationFailureCode::StartDiscontinuity: return "start_discontinuity";
    case VerificationFailureCode::OutsideBounds: return "outside_bounds";
    case VerificationFailureCode::OccupiedSample: return "occupied_sample";
    case VerificationFailureCode::UnknownBeyondCommitment: return "unknown_beyond_commitment";
    case VerificationFailureCode::SafetyUnknownSample: return "safety_unknown_sample";
    case VerificationFailureCode::DynamicLimitsExceeded: return "dynamic_limits_exceeded";
    case VerificationFailureCode::WorldChanged: return "world_changed";
  }
  return "unknown";
}

navigation_planning::TimeParameterizedTrajectory committedPrefix(
    const navigation_planning::TimeParameterizedTrajectory& trajectory, double horizon_s) {
  if (!std::isfinite(horizon_s) || horizon_s <= 0.0 ||
      horizon_s >= trajectory.duration_s || trajectory.points.size() < 2U) {
    return trajectory;
  }
  navigation_planning::TimeParameterizedTrajectory prefix;
  prefix.points.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points) {
    if (point.time_from_start_s < horizon_s) prefix.points.push_back(point);
  }
  const auto upper = std::lower_bound(
      trajectory.points.begin(), trajectory.points.end(), horizon_s,
      [](const auto& point, double time) { return point.time_from_start_s < time; });
  if (upper == trajectory.points.begin()) {
    prefix.points.push_back(trajectory.points.front());
  } else if (upper == trajectory.points.end()) {
    prefix.points.push_back(trajectory.points.back());
  } else {
    const auto& after = *upper;
    const auto& before = *(upper - 1);
    const double span = after.time_from_start_s - before.time_from_start_s;
    const double alpha = span > 0.0 ? (horizon_s - before.time_from_start_s) / span : 0.0;
    navigation_planning::TrajectoryPoint point;
    point.time_from_start_s = horizon_s;
    point.position = before.position + alpha * (after.position - before.position);
    point.velocity = before.velocity + alpha * (after.velocity - before.velocity);
    point.acceleration = before.acceleration + alpha * (after.acceleration - before.acceleration);
    point.jerk = before.jerk + alpha * (after.jerk - before.jerk);
    point.snap = before.snap + alpha * (after.snap - before.snap);
    prefix.points.push_back(point);
  }
  prefix.duration_s = prefix.points.empty() ? 0.0 : prefix.points.back().time_from_start_s;
  return prefix;
}

navigation_planning::TrajectoryPoint interpolateTrajectory(
    const navigation_planning::TimeParameterizedTrajectory& trajectory, double time_s) {
  if (trajectory.points.size() == 1U) return trajectory.points.front();
  const auto upper = std::lower_bound(
      trajectory.points.begin(), trajectory.points.end(), time_s,
      [](const auto& point, double time) { return point.time_from_start_s < time; });
  if (upper == trajectory.points.begin()) return *upper;
  if (upper == trajectory.points.end()) return trajectory.points.back();
  const auto& before = *(upper - 1);
  const auto& after = *upper;
  const double span = after.time_from_start_s - before.time_from_start_s;
  const double alpha = span > 0.0 ? (time_s - before.time_from_start_s) / span : 0.0;
  navigation_planning::TrajectoryPoint result;
  result.time_from_start_s = time_s;
  result.position = before.position + alpha * (after.position - before.position);
  result.velocity = before.velocity + alpha * (after.velocity - before.velocity);
  result.acceleration = before.acceleration + alpha * (after.acceleration - before.acceleration);
  result.jerk = before.jerk + alpha * (after.jerk - before.jerk);
  result.snap = before.snap + alpha * (after.snap - before.snap);
  return result;
}

navigation_planning::TimeParameterizedTrajectory remainingTrajectory(
    const navigation_planning::TimeParameterizedTrajectory& trajectory, double elapsed_s) {
  navigation_planning::TimeParameterizedTrajectory remaining;
  if (!trajectory.finiteAndMonotonic() || trajectory.points.empty() ||
      !std::isfinite(elapsed_s) || elapsed_s < 0.0 || elapsed_s >= trajectory.duration_s) {
    return remaining;
  }
  const auto start = interpolateTrajectory(trajectory, elapsed_s);
  auto first = start;
  first.time_from_start_s = 0.0;
  // Keep position, velocity, and acceleration sampled from the committed
  // trajectory. Splicing the measured state into the first point (especially
  // with an invented zero acceleration) creates a discontinuity at every map
  // revision and invalidates an otherwise safe rolling trajectory.
  remaining.points.push_back(first);
  for (const auto& point : trajectory.points) {
    if (point.time_from_start_s <= elapsed_s) continue;
    auto shifted = point;
    shifted.time_from_start_s -= elapsed_s;
    remaining.points.push_back(shifted);
  }
  remaining.duration_s = remaining.points.back().time_from_start_s;
  return remaining;
}

navigation_interfaces::msg::TrajectorySegment makeTrajectorySegment(
    const navigation_planning::TimeParameterizedTrajectory& trajectory) {
  navigation_interfaces::msg::TrajectorySegment segment;
  segment.duration_s = trajectory.duration_s;
  segment.time_from_start.reserve(trajectory.points.size());
  segment.position.reserve(trajectory.points.size());
  segment.velocity.reserve(trajectory.points.size());
  segment.acceleration.reserve(trajectory.points.size());
  for (const auto& point : trajectory.points) {
    segment.time_from_start.push_back(point.time_from_start_s);
    geometry_msgs::msg::Point position;
    position.x = point.position.x();
    position.y = point.position.y();
    position.z = point.position.z();
    segment.position.push_back(position);
    geometry_msgs::msg::Vector3 velocity;
    velocity.x = point.velocity.x();
    velocity.y = point.velocity.y();
    velocity.z = point.velocity.z();
    segment.velocity.push_back(velocity);
    geometry_msgs::msg::Vector3 acceleration;
    acceleration.x = point.acceleration.x();
    acceleration.y = point.acceleration.y();
    acceleration.z = point.acceleration.z();
    segment.acceleration.push_back(acceleration);
  }
  return segment;
}

navigation_planning::PlanFailureCode mapVerificationFailure(
    navigation_planning::VerificationFailureCode failure) {
  using navigation_planning::PlanFailureCode;
  using navigation_planning::VerificationFailureCode;
  switch (failure) {
    case VerificationFailureCode::WorldChanged:
      return PlanFailureCode::WorldChanged;
    case VerificationFailureCode::DynamicLimitsExceeded:
      return PlanFailureCode::DynamicLimitsInfeasible;
    case VerificationFailureCode::None:
      return PlanFailureCode::None;
    case VerificationFailureCode::WorldModelUnavailable:
    case VerificationFailureCode::InvalidTrajectory:
    case VerificationFailureCode::StartDiscontinuity:
    case VerificationFailureCode::OutsideBounds:
    case VerificationFailureCode::OccupiedSample:
    case VerificationFailureCode::UnknownBeyondCommitment:
    case VerificationFailureCode::SafetyUnknownSample:
    default:
      return PlanFailureCode::TrajectoryInvalid;
  }
}

builtin_interfaces::msg::Time rosTimeFromNanoseconds(std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time stamp;
  const std::int64_t seconds = nanoseconds / 1'000'000'000;
  const std::int64_t remainder = nanoseconds % 1'000'000'000;
  stamp.sec = static_cast<std::int32_t>(seconds);
  stamp.nanosec = static_cast<std::uint32_t>(remainder);
  if (remainder < 0) {
    --stamp.sec;
    stamp.nanosec = static_cast<std::uint32_t>(remainder + 1'000'000'000);
  }
  return stamp;
}

bool cloudHasXyzFloatFields(const sensor_msgs::msg::PointCloud2& cloud) {
  const std::uint64_t minimum_row_bytes =
      static_cast<std::uint64_t>(cloud.point_step) * cloud.width;
  if (cloud.point_step == 0 || cloud.row_step < minimum_row_bytes) {
    return false;
  }
  const std::uint64_t required_bytes = static_cast<std::uint64_t>(cloud.row_step) * cloud.height;
  if (required_bytes > cloud.data.size()) {
    return false;
  }
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto& field : cloud.fields) {
    const bool valid_xyz_field = field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                                 field.count >= 1 &&
                                 static_cast<std::uint64_t>(field.offset) + sizeof(float) <=
                                     cloud.point_step;
    if (!valid_xyz_field) continue;
    if (field.name == "x") has_x = true;
    if (field.name == "y") has_y = true;
    if (field.name == "z") has_z = true;
  }
  return has_x && has_y && has_z;
}

template <typename Model>
double forwardMapDistance(const Model& world,
                          const navigation_mapping::Vec3& start,
                          const navigation_mapping::Vec3& direction,
                          double boundary_margin_m) {
  if (!start.allFinite() || !direction.allFinite() || direction.norm() <= 1e-9 ||
      !std::isfinite(boundary_margin_m) || boundary_margin_m < 0.0) {
    return 0.0;
  }
  const auto bounds = world.bounds(navigation_mapping::WorldLayer::Inflated);
  const auto lower_grid = world.gridToWorld(navigation_mapping::WorldLayer::Inflated, bounds.min);
  const auto upper_grid = world.gridToWorld(navigation_mapping::WorldLayer::Inflated, bounds.max);
  const auto lower = lower_grid.cwiseMin(upper_grid);
  const auto upper = lower_grid.cwiseMax(upper_grid);
  if (!lower.allFinite() || !upper.allFinite() ||
      (upper.array() <= lower.array()).any()) {
    return 0.0;
  }
  const auto unit = direction.normalized();
  double distance = std::numeric_limits<double>::infinity();
  for (int axis = 0; axis < 3; ++axis) {
    if (unit[axis] > 1e-9) {
      distance = std::min(distance, (upper[axis] - start[axis]) / unit[axis]);
    } else if (unit[axis] < -1e-9) {
      distance = std::min(distance, (lower[axis] - start[axis]) / unit[axis]);
    }
  }
  if (!std::isfinite(distance)) return distance;
  return std::max(0.0, distance - boundary_margin_m);
}

}  // namespace

NavigationRuntimeNode::NavigationRuntimeNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("navigation_runtime", options),
      planner_config_([this]() {
        navigation_planning::PlannerConfig config;
        config.limits.max_velocity_mps =
            declare_parameter("navigation.planner.max_velocity_mps", 2.0);
        config.limits.max_acceleration_mps2 =
            declare_parameter("navigation.planner.max_acceleration_mps2", 3.0);
        config.limits.max_deceleration_mps2 =
            declare_parameter("navigation.planner.max_deceleration_mps2", 3.0);
        config.limits.max_jerk_mps3 =
            declare_parameter("navigation.planner.max_jerk_mps3", 6.0);
        config.trajectory_sample_dt_s =
            declare_parameter("navigation.planner.trajectory_sample_dt_s", 0.05);
        const auto generator = declare_parameter(
            "navigation.planner.trajectory_generator", std::string("bspline"));
        if (generator == "bspline") {
          config.trajectory_generator = navigation_planning::TrajectoryGeneratorKind::Bspline;
        } else if (generator == "legacy_quintic") {
          config.trajectory_generator =
              navigation_planning::TrajectoryGeneratorKind::QuinticLegacy;
        } else {
          throw std::invalid_argument(
              "navigation.planner.trajectory_generator must be bspline or legacy_quintic");
        }
        config.bspline_smoothing_iterations = static_cast<int>(declare_parameter<std::int64_t>(
            "navigation.planner.bspline.smoothing_iterations", 12));
        config.bspline_smoothing_step = declare_parameter(
            "navigation.planner.bspline.smoothing_step", 0.04);
        config.bspline_snap_weight = declare_parameter(
            "navigation.planner.bspline.snap_weight", 1.0);
        config.bspline_path_length_weight = declare_parameter(
            "navigation.planner.bspline.path_length_weight", 0.10);
        config.bspline_reference_weight = declare_parameter(
            "navigation.planner.bspline.reference_weight", 0.35);
        config.bspline_time_scale = declare_parameter(
            "navigation.planner.bspline.time_scale", 1.10);
        config.bspline_time_margin_s = declare_parameter(
            "navigation.planner.bspline.time_margin_s", 0.25);
        config.bspline_optimization_sample_dt_s = declare_parameter(
            "navigation.planner.bspline.optimization_sample_dt_s", 0.04);
        config.corridor_sample_spacing_m =
            declare_parameter("navigation.planner.corridor_sample_spacing_m", 0.0);
        config.maximum_time_scaling_iterations = static_cast<int>(declare_parameter<std::int64_t>(
            "navigation.planner.maximum_time_scaling_iterations", 5));
        config.allow_unknown_start = declare_parameter(
            "navigation.planner.allow_unknown_start", false);
        config.unknown_start_radius_m = declare_parameter(
            "navigation.planner.unknown_start_radius_m", 0.0);
        config.allow_nominal_unknown = declare_parameter(
            "navigation.planner.allow_nominal_unknown", true);
        config.nominal_commitment_horizon_s = declare_parameter(
            "navigation.planner.nominal_commitment_horizon_s", 1.0);
        return config;
      }()),
      planner_(planner_config_),
      trajectory_verifier_([this]() {
        navigation_planning::TrajectoryVerificationConfig config;
        config.sample_dt_s = planner_config_.trajectory_sample_dt_s;
        config.limits = planner_config_.limits;
        config.allow_unknown_start = planner_config_.allow_unknown_start;
        config.unknown_start_radius_m = planner_config_.unknown_start_radius_m;
        config.commitment_horizon_s = planner_config_.nominal_commitment_horizon_s;
        config.allow_nominal_unknown = planner_config_.allow_nominal_unknown;
        return config;
      }()) {
  const bool mapping_enabled = declare_parameter("mapping.enabled", true);

  navigation_mapping::MappingPipelineConfig pipeline_config;
  pipeline_config.contract.odom_frame_id = declare_parameter("mapping.frames.odom", std::string("lio_odom"));
  pipeline_config.contract.lidar_frame_id =
      declare_parameter("mapping.frames.lidar", std::string("livox_frame"));

  pipeline_config.point_filter.voxel_size_m =
      declare_parameter("mapping.input.voxel_size_m", 0.20);
  pipeline_config.point_filter.minimum_range_m =
      declare_parameter("mapping.input.min_range_m", 0.0);
  pipeline_config.point_filter.maximum_range_m =
      declare_parameter("mapping.input.max_range_m", 0.0);

  pipeline_config.rog.resolution_m = declare_parameter("mapping.map.resolution_m", 0.20);
  pipeline_config.rog.inflation_resolution_m =
      declare_parameter("mapping.map.inflation_resolution_m", 0.20);
  const auto local_map_size = declare_parameter<std::vector<double>>(
      "mapping.map.local_size_m", std::vector<double>{30.0, 30.0, 12.0});
  if (local_map_size.size() == 3) {
    pipeline_config.rog.local_map_size_m = {local_map_size[0], local_map_size[1],
                                            local_map_size[2]};
  } else {
    RCLCPP_WARN(get_logger(),
                "mapping.map.local_size_m must have exactly 3 elements; using default");
  }
  pipeline_config.rog.ray_range_min_m = declare_parameter("mapping.raycast.min_range_m", 0.3);
  pipeline_config.rog.ray_range_max_m = declare_parameter("mapping.raycast.max_range_m", 15.0);
  pipeline_config.collision.vehicle_radius_m = declare_parameter(
      "navigation.collision.vehicle_radius_m", std::numeric_limits<double>::quiet_NaN());
  pipeline_config.collision.safety_margin_m = declare_parameter(
      "navigation.collision.safety_margin_m", std::numeric_limits<double>::quiet_NaN());

  visualization_enabled_ = declare_parameter("mapping.visualization.enabled", false);
  publish_unknown_ = declare_parameter("mapping.visualization.publish_unknown", false);
  publish_frontier_ = declare_parameter("mapping.visualization.publish_frontier", false);
  pipeline_config.rog.frontier_debug = publish_frontier_;
  const auto visualization_range = declare_parameter<std::vector<double>>(
      "mapping.visualization.range_m", std::vector<double>{15.0, 15.0, 6.0});
  if (visualization_range.size() == 3 &&
      std::all_of(visualization_range.begin(), visualization_range.end(),
                  [](double value) { return std::isfinite(value) && value > 0.0; })) {
    visualization_range_x_m_ = visualization_range[0];
    visualization_range_y_m_ = visualization_range[1];
    visualization_range_z_m_ = visualization_range[2];
  } else {
    RCLCPP_WARN(get_logger(),
                "mapping.visualization.range_m must contain three positive finite values; "
                "using [15, 15, 6] m");
  }
  const auto visualization_max_points = declare_parameter<std::int64_t>(
      "mapping.visualization.max_points", 150000);
  visualization_max_points_ = visualization_max_points > 0
                                  ? static_cast<std::size_t>(visualization_max_points)
                                  : 150000U;
  visualization_frame_id_ = pipeline_config.contract.odom_frame_id;
  const double visualization_rate_hz = declare_parameter(
      "mapping.visualization.publish_rate_hz", 2.0);
  const std::string qos_reliability =
      declare_parameter("mapping.input_qos.reliability", std::string("best_effort"));
  state_topic_ = declare_parameter("navigation.state.topic", std::string("/lio/odometry_propagated"));
  state_max_age_s_ = declare_parameter("navigation.state.max_age_s", 0.5);
  trajectory_state_max_age_s_ = declare_parameter(
      "navigation.safety.trajectory_state_max_age_s", 0.35);
  trajectory_start_position_error_m_ = declare_parameter(
      "navigation.safety.trajectory_start_position_error_m", 0.35);
  planning_frame_id_ = pipeline_config.contract.odom_frame_id;
  replan_rate_hz_ = declare_parameter("navigation.replan_rate_hz", 5.0);
  replan_tracking_error_m_ = declare_parameter("navigation.replan_tracking_error_m", 0.5);
  switch_delay_s_ = declare_parameter("navigation.planner.switch_delay_s", 0.12);
  safety_latency_s_ = declare_parameter("navigation.safety.latency_s", 0.08);
  safety_stop_margin_m_ = declare_parameter("navigation.safety.stop_margin_m", 0.25);
  safety_stop_hold_duration_s_ = declare_parameter(
      "navigation.safety.stop_hold_duration_s", 5.0);
  safety_visibility_horizon_m_ = declare_parameter(
      "navigation.safety.visibility_horizon_m", 15.0);
  local_subgoal_enabled_ = declare_parameter("navigation.local_subgoal.enabled", true);
  execute_nominal_unknown_ = declare_parameter(
      "navigation.planner.execute_nominal_unknown", false);
  fail_closed_on_unknown_mission_goal_ = declare_parameter(
      "navigation.planner.fail_closed_on_unknown_mission_goal", false);
  local_goal_boundary_margin_m_ = declare_parameter(
      "navigation.local_subgoal.boundary_margin_m", 1.0);
  local_goal_max_distance_m_ = declare_parameter(
      "navigation.local_subgoal.max_distance_m", 15.0);
  local_goal_switch_distance_m_ = declare_parameter(
      "navigation.local_subgoal.switch_distance_m", 0.8);
  local_goal_continuation_speed_fraction_ = declare_parameter(
      "navigation.local_subgoal.continuation_speed_fraction", 0.35);
  planning_horizon_min_distance_m_ = declare_parameter(
      "navigation.planning_horizon.minimum_distance_m", 10.0);
  planning_horizon_max_distance_m_ = declare_parameter(
      "navigation.planning_horizon.maximum_distance_m", 30.0);
  planning_horizon_preview_time_s_ = declare_parameter(
      "navigation.planning_horizon.preview_time_s", 5.0);
  planning_horizon_boundary_margin_m_ = declare_parameter(
      "navigation.planning_horizon.boundary_margin_m", 2.0);
  if (!std::isfinite(replan_rate_hz_) || replan_rate_hz_ <= 0.0) {
    throw std::invalid_argument("navigation.replan_rate_hz must be positive and finite");
  }
  if (!std::isfinite(replan_tracking_error_m_) || replan_tracking_error_m_ <= 0.0) {
    throw std::invalid_argument(
        "navigation.replan_tracking_error_m must be finite and positive");
  }
  if (!std::isfinite(switch_delay_s_) || switch_delay_s_ < 0.0 || switch_delay_s_ > 1.0) {
    throw std::invalid_argument(
        "navigation.planner.switch_delay_s must be finite and in [0, 1]");
  }
  if (!std::isfinite(safety_latency_s_) || safety_latency_s_ < 0.0 ||
      !std::isfinite(trajectory_state_max_age_s_) || trajectory_state_max_age_s_ <= 0.0 ||
      !std::isfinite(trajectory_start_position_error_m_) ||
      trajectory_start_position_error_m_ <= 0.0 ||
      !std::isfinite(safety_stop_margin_m_) || safety_stop_margin_m_ < 0.0 ||
      !std::isfinite(safety_stop_hold_duration_s_) || safety_stop_hold_duration_s_ < 0.0 ||
      !std::isfinite(safety_visibility_horizon_m_) || safety_visibility_horizon_m_ <= 0.0) {
    throw std::invalid_argument("navigation.safety parameters are invalid");
  }
  if (!std::isfinite(local_goal_boundary_margin_m_) || local_goal_boundary_margin_m_ < 0.0) {
    throw std::invalid_argument(
        "navigation.local_subgoal.boundary_margin_m must be finite and non-negative");
  }
  if (!std::isfinite(local_goal_max_distance_m_) || local_goal_max_distance_m_ <= 0.0) {
    throw std::invalid_argument(
        "navigation.local_subgoal.max_distance_m must be finite and positive");
  }
  if (!std::isfinite(local_goal_switch_distance_m_) || local_goal_switch_distance_m_ <= 0.0) {
    throw std::invalid_argument(
        "navigation.local_subgoal.switch_distance_m must be finite and positive");
  }
  if (!std::isfinite(local_goal_continuation_speed_fraction_) ||
      local_goal_continuation_speed_fraction_ < 0.0 ||
      local_goal_continuation_speed_fraction_ > 1.0) {
    throw std::invalid_argument(
        "navigation.local_subgoal.continuation_speed_fraction must be in [0, 1]");
  }
  if (!std::isfinite(planning_horizon_min_distance_m_) ||
      planning_horizon_min_distance_m_ <= 0.0 ||
      !std::isfinite(planning_horizon_max_distance_m_) ||
      planning_horizon_max_distance_m_ < planning_horizon_min_distance_m_ ||
      !std::isfinite(planning_horizon_preview_time_s_) ||
      planning_horizon_preview_time_s_ < 0.0 ||
      !std::isfinite(planning_horizon_boundary_margin_m_) ||
      planning_horizon_boundary_margin_m_ < 0.0) {
    throw std::invalid_argument("navigation.planning_horizon parameters are invalid");
  }
  if (!std::isfinite(planner_config_.nominal_commitment_horizon_s) ||
      planner_config_.nominal_commitment_horizon_s < 0.0) {
    throw std::invalid_argument(
        "navigation.planner.nominal_commitment_horizon_s must be finite and non-negative");
  }
  if (planner_config_.bspline_smoothing_iterations < 0 ||
      !std::isfinite(planner_config_.bspline_smoothing_step) ||
      planner_config_.bspline_smoothing_step < 0.0 ||
      !std::isfinite(planner_config_.bspline_snap_weight) ||
      planner_config_.bspline_snap_weight < 0.0 ||
      !std::isfinite(planner_config_.bspline_path_length_weight) ||
      planner_config_.bspline_path_length_weight < 0.0 ||
      !std::isfinite(planner_config_.bspline_reference_weight) ||
      planner_config_.bspline_reference_weight < 0.0 ||
      !std::isfinite(planner_config_.bspline_time_scale) ||
      planner_config_.bspline_time_scale <= 0.0 ||
      !std::isfinite(planner_config_.bspline_time_margin_s) ||
      planner_config_.bspline_time_margin_s < 0.0 ||
      !std::isfinite(planner_config_.bspline_optimization_sample_dt_s) ||
      planner_config_.bspline_optimization_sample_dt_s <= 0.0) {
    throw std::invalid_argument("navigation.planner.bspline parameters are invalid");
  }

  const std::string generated_config_directory =
      declare_parameter("mapping.generated_config_directory",
                        std::string("/tmp/navigation_mapping"));
  std::filesystem::create_directories(generated_config_directory);

  pipeline_ = std::make_unique<navigation_mapping::MappingPipeline>(
      pipeline_config, [this]() { return get_clock()->now().seconds(); },
      generated_config_directory);

  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "navigation_mapping/diagnostics", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable());
  planning_diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "navigation_planning/diagnostics", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable());
  planner_heartbeat_publisher_ = create_publisher<std_msgs::msg::Empty>(
      "navigation/planner_heartbeat", rclcpp::QoS{rclcpp::KeepLast{1}}.reliable());
  trajectory_publisher_ = create_publisher<navigation_interfaces::msg::PlannedTrajectory>(
      "navigation/trajectory", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable());
  trajectory_bundle_publisher_ =
      create_publisher<navigation_interfaces::msg::PlannedTrajectoryBundle>(
          "navigation/trajectory_bundle", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable());
  trajectory_bundle_v2_publisher_ =
      create_publisher<navigation_interfaces::msg::TrajectoryBundle>(
          "navigation/trajectory_bundle_v2", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable());
  planner_trace_publisher_ =
      create_publisher<navigation_interfaces::msg::PlannerCycleTrace>(
          "navigation/planner_trace", rclcpp::QoS{rclcpp::KeepLast{50}}.reliable());
  planned_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "navigation/visualization/planned_path", rclcpp::QoS{rclcpp::KeepLast{1}}.reliable());
  planned_path_nominal_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "navigation/planned_path_nominal", rclcpp::QoS{rclcpp::KeepLast{1}}.reliable());
  planned_path_safety_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "navigation/planned_path_safety", rclcpp::QoS{rclcpp::KeepLast{1}}.reliable());
  planned_path_selected_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "navigation/planned_path_selected", rclcpp::QoS{rclcpp::KeepLast{1}}.reliable());
  navigation_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "navigation/visualization/markers",
      rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local());

  // Diagnostics are a periodic snapshot, not part of the observation hot
  // path. Create this in the same group as map mutation so the plain
  // MappingDiagnostics struct is never read concurrently with an update.
  mapping_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  ingress_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds{500}, [this]() { publishDiagnostics(); },
      mapping_callback_group_);

  if (!mapping_enabled) {
    RCLCPP_INFO(get_logger(),
                "mapping.enabled is false; navigation_runtime running idle "
                "(no subscription, no map mutation)");
    return;
  }

  // All map-touching callbacks are serialized in one
  // MutuallyExclusiveCallbackGroup. There is exactly one such callback today
  // (onObservation), but this makes the serialization requirement explicit
  // and future-proof rather than implicit in "there happens to be one
  // subscription".
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = mapping_callback_group_;

  auto qos = rclcpp::QoS{rclcpp::KeepLast{1}}.durability_volatile();
  if (qos_reliability == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  observation_subscription_ =
      create_subscription<navigation_interfaces::msg::LidarMappingObservation>(
          "lio/mapping_observation", qos,
          [this](const navigation_interfaces::msg::LidarMappingObservation::ConstSharedPtr&
                     message) { onObservation(message); },
          subscription_options);

  odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      // State is sampled by the planner; replaying a queue of old states is
      // worse than dropping an intermediate sample because it makes the
      // planner act on a stale pose after a callback burst.
      state_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) { onOdometry(message); },
      [&]() {
        auto options = subscription_options;
        options.callback_group = ingress_callback_group_;
        return options;
      }());
  goal_subscription_ = create_subscription<navigation_interfaces::msg::NavigationGoal>(
      "navigation/goal", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable(),
      [this](const navigation_interfaces::msg::NavigationGoal::ConstSharedPtr message) {
        onGoal(message);
      },
      [&]() {
        auto options = subscription_options;
        options.callback_group = ingress_callback_group_;
        return options;
      }());
  planning_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(1.0 / replan_rate_hz_)),
      [this]() { planActiveGoal(); }, mapping_callback_group_);

  if (visualization_enabled_) {
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    occupied_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "navigation_mapping/visualization/occupied", qos);
    inflated_occupied_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "navigation_mapping/visualization/inflated_occupied", qos);
    if (publish_unknown_) {
      unknown_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "navigation_mapping/visualization/unknown", qos);
    }
    if (publish_frontier_) {
      frontier_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "navigation_mapping/visualization/frontier", qos);
    }
    const double period_s = std::isfinite(visualization_rate_hz) && visualization_rate_hz > 0.0
                                ? 1.0 / visualization_rate_hz
                                : 0.5;
    visualization_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(period_s)),
        [this]() { publishMapVisualization(); }, mapping_callback_group_);
    RCLCPP_INFO(get_logger(),
                "navigation_mapping visualization enabled under navigation_mapping/visualization "
                "unk=%s frontier=%s rate=%.2f Hz max_points=%zu",
                publish_unknown_ ? "on" : "off", publish_frontier_ ? "on" : "off",
                1.0 / period_s, visualization_max_points_);
  }
}

const char* NavigationRuntimeNode::localGoalStatusName(
    LocalGoalSelectionStatus status) const noexcept {
  return ::navigation_runtime::localGoalStatusName(status);
}

void NavigationRuntimeNode::onObservation(
    const navigation_interfaces::msg::LidarMappingObservation::ConstSharedPtr& message) {
  const auto callback_started = std::chrono::steady_clock::now();
  const auto callback_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  auto& diagnostics = pipeline_->diagnostics();
  if (diagnostics.first_callback_wall_ns == 0) {
    diagnostics.first_callback_wall_ns = callback_wall_ns;
  }
  diagnostics.last_callback_wall_ns = callback_wall_ns;
  ++diagnostics.mapping_observation_receive_count;
  const std::uint64_t stream_id = message->observation_stream_id;
  const std::uint64_t sequence = message->observation_sequence;
  if (diagnostics.last_received_observation_stream_id == 0) {
    diagnostics.last_received_observation_stream_id = stream_id;
    diagnostics.last_received_observation_sequence = sequence;
  } else if (stream_id != diagnostics.last_received_observation_stream_id) {
    ++diagnostics.observation_sequence_stream_switch_count;
    diagnostics.last_received_observation_stream_id = stream_id;
    diagnostics.last_received_observation_sequence = sequence;
  } else if (diagnostics.last_received_observation_sequence == 0) {
    diagnostics.last_received_observation_sequence = sequence;
  } else if (sequence > diagnostics.last_received_observation_sequence) {
    const std::uint64_t missing = sequence - diagnostics.last_received_observation_sequence - 1U;
    diagnostics.observation_sequence_missing_count += missing;
    diagnostics.observation_sequence_max_consecutive_missing = std::max(
        diagnostics.observation_sequence_max_consecutive_missing, missing);
    diagnostics.last_received_observation_sequence = sequence;
  } else if (sequence == diagnostics.last_received_observation_sequence) {
    ++diagnostics.observation_sequence_duplicate_count;
  } else {
    ++diagnostics.observation_sequence_regression_count;
  }
  if (!cloudHasXyzFloatFields(message->points)) {
    ++invalid_cloud_count_;
    ++diagnostics.invalid_cloud_count;
    ++diagnostics.mapping_observation_rejection_count;
    return;
  }

  navigation_mapping::ObservationInput input;
  input.header_frame_id = message->header.frame_id;
  input.header_stamp = message->header.stamp;
  input.points_frame_id = message->points.header.frame_id;
  input.points_stamp = message->points.header.stamp;
  input.sensor_pose = message->sensor_pose;
  input.public_frame_generation = message->public_frame_generation;

  const auto decode_started = std::chrono::steady_clock::now();
  const std::size_t point_count =
      static_cast<std::size_t>(message->points.width) * message->points.height;
  input.points_lidar_m.reserve(point_count);
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(message->points, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(message->points, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(message->points, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      input.points_lidar_m.emplace_back(*x, *y, *z);
    }
  } catch (const std::exception& error) {
    ++invalid_cloud_count_;
    ++diagnostics.invalid_cloud_count;
    ++diagnostics.processing_exception_count;
    ++diagnostics.mapping_observation_rejection_count;
    diagnostics.ros_pointcloud_decode_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - decode_started).count();
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Malformed mapping PointCloud2; keeping node alive: %s", error.what());
    return;
  }
  diagnostics.ros_pointcloud_decode_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - decode_started).count();

  const auto accepted_before = diagnostics.accepted_observation_count;
  bool processing_failed = false;
  try {
    pipeline_->process(input);
  } catch (const std::exception& error) {
    processing_failed = true;
    ++diagnostics.processing_exception_count;
    ++diagnostics.mapping_observation_rejection_count;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "ROG-Map update failed; keeping node alive: %s", error.what());
  }
  if (diagnostics.accepted_observation_count == accepted_before &&
      !processing_failed) {
    ++diagnostics.mapping_observation_rejection_count;
  }
  diagnostics.mapping_callback_total_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - callback_started).count();
}

void NavigationRuntimeNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message) {
  const auto& position = message->pose.pose.position;
  const auto& velocity = message->twist.twist.linear;
  if (message->header.frame_id != planning_frame_id_ || timeNanoseconds(message->header.stamp) <= 0 ||
      !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Rejecting navigation state with invalid frame, epoch, or values");
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  const auto stamp_ns = timeNanoseconds(message->header.stamp);
  const navigation_mapping::Vec3 current_velocity{velocity.x, velocity.y, velocity.z};
  if (previous_velocity_.has_value() && previous_odometry_stamp_ns_ > 0 &&
      stamp_ns > previous_odometry_stamp_ns_) {
    const double dt_s = static_cast<double>(stamp_ns - previous_odometry_stamp_ns_) / 1e9;
    if (std::isfinite(dt_s) && dt_s >= 0.005 && dt_s <= 0.2) {
      const auto raw_acceleration = (current_velocity - *previous_velocity_) / dt_s;
      if (raw_acceleration.allFinite()) {
        const auto previous = latest_acceleration_.value_or(navigation_mapping::Vec3::Zero());
        // Odometry has no authoritative acceleration field. A short low-pass
        // estimate is still better than injecting an acceleration discontinuity
        // of exactly zero at every replan. Keep it conservative and bounded.
        latest_acceleration_ = 0.25 * raw_acceleration + 0.75 * previous;
      }
    }
  }
  previous_velocity_ = current_velocity;
  previous_odometry_stamp_ns_ = stamp_ns;
  latest_odometry_ = *message;
}

void NavigationRuntimeNode::onGoal(
    const navigation_interfaces::msg::NavigationGoal::ConstSharedPtr& message) {
  if (message->mission_id.empty() || message->request_id == 0U ||
      !std::isfinite(message->acceptance_radius_m) || message->acceptance_radius_m <= 0.0) {
    RCLCPP_WARN(get_logger(), "Rejecting navigation goal with invalid identity or acceptance radius");
    return;
  }
  if (message->behavior > navigation_interfaces::msg::NavigationGoal::BEHAVIOR_STOP ||
      (message->has_next_target &&
       (!std::isfinite(message->next_target.x) || !std::isfinite(message->next_target.y) ||
        !std::isfinite(message->next_target.z)))) {
    RCLCPP_WARN(get_logger(), "Rejecting navigation goal with invalid continuation behavior");
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (active_goal_.has_value()) {
    const auto& active = *active_goal_;
    if (message->mission_id == active.mission_id &&
        message->waypoint_index == active.waypoint_index &&
        message->request_id <= active.request_id) {
      return;
    }
    if (message->mission_id == active.mission_id &&
        message->waypoint_index < active.waypoint_index) {
      return;
    }
  }
  const bool goal_changed = !active_goal_.has_value() ||
                            message->mission_id != active_goal_->mission_id ||
                            message->waypoint_index != active_goal_->waypoint_index ||
                            message->request_id != active_goal_->request_id;
  if (goal_changed) braking_stop_latched_ = false;
  active_goal_ = *message;
}

void NavigationRuntimeNode::planActiveGoal() {
  std::optional<navigation_interfaces::msg::NavigationGoal> active_goal;
  std::optional<nav_msgs::msg::Odometry> latest_odometry;
  std::optional<navigation_mapping::Vec3> latest_acceleration;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    active_goal = active_goal_;
    latest_odometry = latest_odometry_;
    latest_acceleration = latest_acceleration_;
  }
  if (!active_goal.has_value()) return;
  if (planner_heartbeat_publisher_) {
    planner_heartbeat_publisher_->publish(std_msgs::msg::Empty{});
  }
  const auto goal_message = *active_goal;
  // The planner and PX4 mode run on separate ROS/DDS callbacks. A small
  // future timestamp is normal transport skew; larger skew is rejected.
  constexpr std::int64_t kMaximumFutureStateSkewNs = 50'000'000LL;
  navigation_mapping::WorldModel world(pipeline_->adapter());
  const auto current_position = [&]() -> std::optional<navigation_mapping::Vec3> {
    if (!latest_odometry.has_value()) return std::nullopt;
    const auto& odometry = *latest_odometry;
    const auto& position = odometry.pose.pose.position;
    const auto value = navigation_mapping::Vec3{position.x, position.y, position.z};
    return value.allFinite() ? std::optional<navigation_mapping::Vec3>{value} : std::nullopt;
  }();
  const bool goal_unchanged =
      last_plan_identity_valid_ &&
      goal_message.mission_id == last_planned_mission_id_ &&
      goal_message.waypoint_index == last_planned_waypoint_index_ &&
      goal_message.request_id == last_planned_request_id_;
  bool braking_stop_latched = goal_unchanged && braking_stop_latched_;
  if (!goal_unchanged) {
    committed_local_goal_.reset();
    last_detour_obstacle_anchor_.reset();
    last_detour_lateral_direction_.reset();
    // Preserve a level mission altitude across waypoint changes.  Reset only
    // for a genuine vertical mission transition; otherwise each new request
    // would reintroduce a fresh z-cell choice at the turn.
    if (!committed_local_altitude_m_.has_value() ||
        !std::isfinite(*committed_local_altitude_m_) ||
        std::abs(*committed_local_altitude_m_ - goal_message.target.z) > 0.6) {
      committed_local_altitude_m_ = goal_message.target.z;
    }
  }
  const bool tracking_error_exceeded = [&]() {
    if (!current_position.has_value() || !last_planned_trajectory_.has_value() ||
        last_plan_time_ns_ <= 0) {
      return true;
    }
    const double elapsed_s =
        static_cast<double>(get_clock()->now().nanoseconds() - last_plan_time_ns_) / 1e9;
    if (!std::isfinite(elapsed_s) || elapsed_s >= last_planned_trajectory_->duration_s) {
      return true;
    }
    const auto expected = interpolateTrajectory(
        *last_planned_trajectory_, std::max(0.0, elapsed_s));
    const double tracking_error = (*current_position - expected.position).norm();
    return !std::isfinite(tracking_error) || tracking_error >= replan_tracking_error_m_;
  }();
  const bool current_state_fresh = [&]() {
    if (!latest_odometry.has_value()) return false;
    const auto& odometry = *latest_odometry;
    const auto age_ns = get_clock()->now().nanoseconds() - timeNanoseconds(odometry.header.stamp);
    return odometry.header.frame_id == planning_frame_id_ &&
           age_ns >= -kMaximumFutureStateSkewNs &&
           std::isfinite(state_max_age_s_) && state_max_age_s_ > 0.0 &&
           age_ns <= static_cast<std::int64_t>(state_max_age_s_ * 1e9);
  }();
  // A terminal STOP waypoint is the one intentional exception to rolling
  // replacement. Once the vehicle is inside the mission acceptance radius
  // and has settled, publishing another spline from each voxel-map revision
  // can briefly expose a boundary/unknown-state failure and make PX4 resume
  // motion after it has already arrived. Keep the last verified terminal
  // trajectory until MissionController observes the hold/complete event.
  // Pass-through waypoints never enter this hold path.
  if (goal_message.behavior == navigation_interfaces::msg::NavigationGoal::BEHAVIOR_STOP &&
      goal_unchanged && last_plan_success_ && last_goal_terminal_ && current_state_fresh &&
      latest_odometry.has_value() && last_planned_trajectory_.has_value()) {
    const auto& position = latest_odometry->pose.pose.position;
    const auto& velocity = latest_odometry->twist.twist.linear;
    const navigation_mapping::Vec3 terminal_position{position.x, position.y, position.z};
    const navigation_mapping::Vec3 terminal_velocity{velocity.x, velocity.y, velocity.z};
    const navigation_mapping::Vec3 mission_terminal{goal_message.target.x, goal_message.target.y,
                                                    goal_message.target.z};
    const double terminal_distance = (terminal_position - mission_terminal).norm();
    if (terminal_position.allFinite() && terminal_velocity.allFinite() &&
        mission_terminal.allFinite() && std::isfinite(terminal_distance) &&
        terminal_distance <= goal_message.acceptance_radius_m &&
        terminal_velocity.norm() <= 0.20) {
      ++plan_skip_count_;
      ++trajectory_reuse_count_;
      last_replan_reason_ = "reuse_terminal_hold";
      // Refresh the already-settled terminal state as an atomic bundle. PX4
      // expires a trajectory after duration + stale_after; simply returning
      // here would let a correct terminal hold time out and hand control back
      // while the mission controller is still confirming acceptance. This is
      // a one-point safety hold, not a new corridor/subgoal plan.
      navigation_planning::TimeParameterizedTrajectory hold_trajectory;
      navigation_planning::TrajectoryPoint hold_start;
      hold_start.position = terminal_position;
      hold_start.time_from_start_s = 0.0;
      navigation_planning::TrajectoryPoint hold_end = hold_start;
      hold_end.time_from_start_s = 1.0;
      hold_trajectory.points = {hold_start, hold_end};
      hold_trajectory.duration_s = 1.0;
      navigation_planning::VehicleState hold_state;
      hold_state.position = terminal_position;
      hold_state.velocity = navigation_mapping::Vec3::Zero();
      hold_state.acceleration = navigation_mapping::Vec3::Zero();
      const auto safety_verification = trajectory_verifier_.verify(
          hold_trajectory, hold_state, navigation_planning::PlanRole::Safety, world);
      if (safety_verification.success) {
        navigation_planning::PlanResult selected_hold;
        selected_hold.success = true;
        selected_hold.role = navigation_planning::PlanRole::Committed;
        selected_hold.safety_kind = navigation_planning::SafetyPlanKind::None;
        selected_hold.failure_code = navigation_planning::PlanFailureCode::None;
        selected_hold.world_generation = world.generation();
        selected_hold.world_revision = world.revision();
        selected_hold.trajectory = hold_trajectory;
        auto safety_hold = selected_hold;
        safety_hold.role = navigation_planning::PlanRole::Safety;
        // This artifact is a stationary terminal hold, not a collision-aware
        // route. Keep the safety kind truthful so the bundle/PX4 adapter and
        // HTML report do not label a zero-motion hold as a detour route.
        safety_hold.safety_kind = navigation_planning::SafetyPlanKind::BrakingStop;
        const auto now_stamp = get_clock()->now();
        const auto valid_from = rosTimeFromNanoseconds(now_stamp.nanoseconds());
        trajectory_publisher_->publish(makeTrajectoryMessage(selected_hold, goal_message));
        trajectory_bundle_publisher_->publish(makeTrajectoryBundleMessage(
            selected_hold, selected_hold, safety_hold, goal_message, valid_from,
            next_bundle_id_, last_bundle_id_));
        trajectory_bundle_v2_publisher_->publish(makeTrajectoryBundleV2Message(
            selected_hold, selected_hold, safety_hold, goal_message, valid_from,
            next_bundle_id_, last_bundle_id_));
        last_bundle_id_ = next_bundle_id_++;
        last_plan_time_ns_ = now_stamp.nanoseconds();
        last_planned_duration_s_ = hold_trajectory.duration_s;
        last_planned_trajectory_ = hold_trajectory;
        last_planned_role_ = selected_hold.role;
        last_planned_safety_kind_ = selected_hold.safety_kind;
        last_planned_world_generation_ = world.generation();
        last_planned_world_revision_ = world.revision();
        publishPlanningPaths(
            selected_hold, selected_hold, safety_hold, goal_message.header, std::nullopt);
        publishNavigationVisualization(selected_hold, goal_message);
        publishPlanningDiagnostics(selected_hold);
      } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Terminal hold safety verification failed: %s",
            verificationFailureName(safety_verification.failure_code));
      }
      return;
    }
  }
  const bool trajectory_near_stale = [&]() {
    if (last_plan_time_ns_ <= 0 || !std::isfinite(last_planned_duration_s_) ||
        last_planned_duration_s_ <= 0.0) {
      return true;
    }
    const auto now_ns = get_clock()->now().nanoseconds();
    if (now_ns < last_plan_time_ns_) return false;
    constexpr double kReplanBeforeStaleS = 0.25;
    const double replan_deadline_s =
        std::max(0.0, last_planned_duration_s_ - kReplanBeforeStaleS);
    return static_cast<double>(now_ns - last_plan_time_ns_) / 1e9 >= replan_deadline_s;
  }();
  const bool world_unchanged =
      world.generation() == last_planned_world_generation_ &&
      world.revision() == last_planned_world_revision_;
  // A local endpoint is a rolling corridor anchor, but the emitted trajectory
  // is still a committed control suffix. Replanning on every map revision
  // makes independent A*/spline solutions compete at the PX4 boundary even
  // when the old suffix remains verified. Decide whether a fresh plan is
  // actually needed first; an unchanged map and a changed-but-still-valid map
  // both reuse the committed suffix below.
  const bool committed_path_requires_refresh =
      !goal_unchanged || !last_plan_success_ || !last_planned_trajectory_.has_value() ||
      !current_state_fresh || tracking_error_exceeded || trajectory_near_stale;
  const bool rolling_horizon_replan = !braking_stop_latched && committed_path_requires_refresh;

  // A verified braking stop is a committed terminal trajectory for this
  // waypoint/request.  Once its stop phase has completed, a rolling planning
  // timer must not turn the same goal into a stream of freshly generated
  // zero-duration stops.  Revalidate a still-running stop only when the map
  // changed; after the stop has elapsed, the measured vehicle state is the
  // authority and a replacement is needed only if it is no longer stopped.
  if (braking_stop_latched && current_state_fresh && latest_odometry.has_value() &&
      last_planned_trajectory_.has_value() && last_plan_time_ns_ > 0) {
    const auto& odometry = *latest_odometry;
    const navigation_planning::VehicleState state{
        navigation_mapping::Vec3{odometry.pose.pose.position.x,
                                 odometry.pose.pose.position.y,
                                 odometry.pose.pose.position.z},
        navigation_mapping::Vec3{odometry.twist.twist.linear.x,
                                 odometry.twist.twist.linear.y,
                                 odometry.twist.twist.linear.z},
        navigation_mapping::Vec3::Zero()};
    const double elapsed_s =
        static_cast<double>(get_clock()->now().nanoseconds() - last_plan_time_ns_) / 1e9;
    bool held_stop_safe = false;
    bool release_stop_for_route_retry = false;
    if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
        elapsed_s >= last_planned_trajectory_->duration_s &&
        state.velocity.norm() <= 0.15 + 1e-6) {
      // A non-terminal braking stop is a recovery state, not a reason to
      // regenerate the same zero-motion trajectory at every map revision.
      // Once settled, retry the route only on a wall-clock cooldown.  The
      // retry must release the latch so the normal route branch is reached;
      // calling planSafetyStop() again here can never discover a detour.
      constexpr double kSafetyRouteRetryIntervalS = 2.0;
      const auto now_ns = get_clock()->now().nanoseconds();
      const auto retry_anchor_ns = last_safety_retry_time_ns_ > 0
                                       ? last_safety_retry_time_ns_
                                       : last_plan_time_ns_;
      const double retry_elapsed_s =
          static_cast<double>(now_ns - retry_anchor_ns) / 1e9;
      if (last_goal_terminal_ || !std::isfinite(retry_elapsed_s) ||
          retry_elapsed_s < kSafetyRouteRetryIntervalS) {
        held_stop_safe = true;
      } else {
        release_stop_for_route_retry = true;
        braking_stop_latched_ = false;
        ++safety_stop_retry_count_;
        last_safety_retry_time_ns_ = now_ns;
        last_replan_reason_ = "safety_stop_route_retry";
      }
    } else if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
               elapsed_s < last_planned_trajectory_->duration_s &&
               last_planned_trajectory_->finiteAndMonotonic()) {
      // A braking trajectory is already the fail-closed command. Do not
      // invalidate it just because a lidar revision arrived while the vehicle
      // is still decelerating; that produced a map-revision/chattering loop.
      held_stop_safe = true;
    }
    if (held_stop_safe) {
      ++plan_skip_count_;
      ++trajectory_reuse_count_;
      last_planned_world_generation_ = world.generation();
      last_planned_world_revision_ = world.revision();
      last_replan_reason_ = "reuse_latched_braking_stop";
      return;
    }
    if (release_stop_for_route_retry) {
      // Continue into the ordinary rolling planner below.  `braking_stop_latched`
      // is recomputed from this local value after this block, so a recovered
      // map can produce a SafetyRoute instead of another braking stop.
      braking_stop_latched = false;
    } else {
      last_replan_reason_ = "safety_stop_latched_invalidated";
    }
  }

  // A committed trajectory is reusable across map revisions only after the
  // remaining corridor has been checked against the new world. This avoids
  // turning every incoming scan into a full A* invocation while retaining a
  // fail-closed response to a newly occupied cell.
  if (!rolling_horizon_replan && goal_unchanged && last_plan_success_ &&
      current_state_fresh && !tracking_error_exceeded && !trajectory_near_stale) {
    if (world_unchanged) {
      ++plan_skip_count_;
      ++trajectory_reuse_count_;
      last_replan_reason_ = "reuse_unchanged_world";
      return;
    }
    if (world.generation() == last_planned_world_generation_ &&
        last_planned_trajectory_.has_value() && last_plan_time_ns_ > 0) {
      const auto& odometry = *latest_odometry;
      navigation_planning::VehicleState state;
      state.position = navigation_mapping::Vec3{odometry.pose.pose.position.x,
                                                odometry.pose.pose.position.y,
                                                odometry.pose.pose.position.z};
      state.velocity = navigation_mapping::Vec3{odometry.twist.twist.linear.x,
                                                odometry.twist.twist.linear.y,
                                                odometry.twist.twist.linear.z};
      state.acceleration = latest_acceleration.value_or(navigation_mapping::Vec3::Zero());
      const double elapsed_s =
          static_cast<double>(get_clock()->now().nanoseconds() - last_plan_time_ns_) / 1e9;
      const auto expected = interpolateTrajectory(
          *last_planned_trajectory_, std::max(0.0, elapsed_s));
      const double tracking_error = (state.position - expected.position).norm();
      ++trajectory_revalidation_count_;
      if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
          std::isfinite(tracking_error) && tracking_error < replan_tracking_error_m_) {
        const auto remaining = remainingTrajectory(*last_planned_trajectory_, elapsed_s);
        navigation_planning::VehicleState verification_state;
        verification_state.position = remaining.points.front().position;
        verification_state.velocity = remaining.points.front().velocity;
        verification_state.acceleration = remaining.points.front().acceleration;
        const auto verification = trajectory_verifier_.verify(
            remaining, verification_state, last_planned_role_, world);
        if (verification.success) {
          ++plan_skip_count_;
          ++trajectory_reuse_count_;
          last_planned_world_generation_ = world.generation();
          last_planned_world_revision_ = world.revision();
          last_replan_reason_ = "reuse_revalidated_trajectory";
          return;
        }
      }
      ++trajectory_revalidation_failure_count_;
      last_replan_reason_ = "trajectory_revalidation_failed";
    } else {
      last_replan_reason_ = "trajectory_cache_unavailable";
    }
  } else if (rolling_horizon_replan) {
    last_replan_reason_ = "rolling_horizon_refresh";
  } else if (!goal_unchanged) {
    last_replan_reason_ = "goal_changed";
  } else if (tracking_error_exceeded) {
    last_replan_reason_ = "tracking_error_exceeded";
  } else if (trajectory_near_stale) {
    last_replan_reason_ = "trajectory_near_stale";
  } else if (braking_stop_latched) {
    last_replan_reason_ = "safety_stop_latched_replacement";
  } else {
    last_replan_reason_ = "state_or_policy_invalid";
  }

  ++full_replan_count_;
  plan_valid_from_delay_s_ = 0.0;

  navigation_planning::PlanResult result;
  navigation_planning::PlanResult nominal_candidate;
  navigation_planning::PlanResult safety_candidate;
  // Keep the complete planning artifact for RViz/diagnostics.  The selected
  // result may intentionally be reduced to the commitment prefix before it
  // is handed to PX4, but that safety handover must not make the planned path
  // look like a sequence of one-second subgoals.
  std::optional<navigation_planning::TimeParameterizedTrajectory>
      full_nominal_visualization_trajectory;
  if (goal_message.header.frame_id != planning_frame_id_ ||
      timeNanoseconds(goal_message.header.stamp) <= 0 ||
      !std::isfinite(goal_message.target.x) || !std::isfinite(goal_message.target.y) ||
      !std::isfinite(goal_message.target.z)) {
    result.failure_code = navigation_planning::PlanFailureCode::InvalidState;
  } else if (!latest_odometry.has_value()) {
    result.failure_code = navigation_planning::PlanFailureCode::InvalidState;
  } else {
    const auto& odometry = *latest_odometry;
    const auto age_ns = get_clock()->now().nanoseconds() - timeNanoseconds(odometry.header.stamp);
    last_planning_state_age_s_ = static_cast<double>(age_ns) / 1e9;
    const auto& position = odometry.pose.pose.position;
    const auto& velocity = odometry.twist.twist.linear;
    if (age_ns < -kMaximumFutureStateSkewNs || !std::isfinite(state_max_age_s_) ||
        state_max_age_s_ <= 0.0 || !std::isfinite(trajectory_state_max_age_s_) ||
        trajectory_state_max_age_s_ <= 0.0 ||
        age_ns > static_cast<std::int64_t>(std::min(
            state_max_age_s_, trajectory_state_max_age_s_) * 1e9)) {
      result.failure_code = navigation_planning::PlanFailureCode::InvalidState;
    } else {
      const navigation_planning::VehicleState measured_state{
          navigation_mapping::Vec3{position.x, position.y, position.z},
          navigation_mapping::Vec3{velocity.x, velocity.y, velocity.z},
          latest_acceleration.value_or(navigation_mapping::Vec3::Zero())};
      navigation_planning::VehicleState state = measured_state;
      // The trajectory optimizer reserves a small design margin below the
      // hard acceleration limit. Clamp the measured initial acceleration to
      // that same margin; otherwise a single noisy odometry sample exactly
      // at the hard limit makes every nominal replan report
      // DynamicLimitsInfeasible and fall back to a braking stop.
      const double initial_acceleration_limit =
          0.995 * planner_config_.limits.max_acceleration_mps2;
      if (state.acceleration.norm() > initial_acceleration_limit) {
        state.acceleration *= initial_acceleration_limit /
                              std::max(state.acceleration.norm(), 1e-9);
      }
      // Do not replace the active trajectory at the instant a map update
      // arrives. Plan from a future state on the committed trajectory and
      // publish a valid_from timestamp; PX4 keeps the old trajectory until
      // that point. This is the first layer of cross-replan continuity.
      plan_valid_from_delay_s_ = 0.0;
      if (!braking_stop_latched && goal_unchanged && !tracking_error_exceeded &&
          current_state_fresh && latest_odometry.has_value() &&
          last_planned_trajectory_.has_value() &&
          last_plan_time_ns_ > 0 && last_planned_trajectory_->finiteAndMonotonic()) {
        const double elapsed_s = std::max(
            0.0, static_cast<double>(get_clock()->now().nanoseconds() - last_plan_time_ns_) /
                     1e9);
        const double switch_time_s = elapsed_s + switch_delay_s_;
        if (std::isfinite(elapsed_s) && switch_time_s < last_planned_trajectory_->duration_s &&
            switch_time_s >= 0.0) {
          const auto switch_state = interpolateTrajectory(*last_planned_trajectory_, switch_time_s);
          state.position = switch_state.position;
          state.velocity = switch_state.velocity;
          state.acceleration = switch_state.acceleration;
          plan_valid_from_delay_s_ = switch_delay_s_;
        }
      }
      const navigation_mapping::Vec3 requested_goal{goal_message.target.x, goal_message.target.y,
                                                    goal_message.target.z};
      // Keep a fixed geometric direction for the active mission leg.  The
      // measured velocity is still useful for the stopping-distance horizon,
      // but it is not a route intent: after a lateral obstacle escape it can
      // point along a wall and make the next rolling selector advance farther
      // away from the waypoint on every tick.
      if (!goal_unchanged || !mission_leg_initialized_ ||
          !mission_leg_unit_.allFinite() || mission_leg_unit_.norm() <= 1e-6) {
        const auto mission_delta = requested_goal - state.position;
        const double mission_distance = mission_delta.norm();
        if (std::isfinite(mission_distance) && mission_distance > 1e-6) {
          mission_leg_start_position_ = state.position;
          mission_leg_unit_ = (mission_delta / mission_distance).eval();
          committed_mission_progress_m_ = 0.0;
          mission_leg_initialized_ = true;
        } else {
          mission_leg_initialized_ = false;
          mission_leg_start_position_ = state.position;
          mission_leg_unit_ = navigation_mapping::Vec3::Zero();
          committed_mission_progress_m_ = 0.0;
        }
      }
      const auto mission_route_tangent = [&]() {
        navigation_mapping::Vec3 tangent = navigation_mapping::Vec3::Zero();
        if (mission_leg_initialized_ && mission_leg_unit_.allFinite() &&
            mission_leg_unit_.norm() > 1e-6) {
          tangent = mission_leg_unit_.normalized();
        } else {
          const auto mission_delta = requested_goal - state.position;
          if (mission_delta.norm() > 1e-6) tangent = mission_delta.normalized();
        }
        return tangent;
      }();
      if (mission_leg_initialized_ && mission_route_tangent.norm() > 1e-6) {
        const double observed_progress =
            (state.position - mission_leg_start_position_).dot(mission_route_tangent);
        if (std::isfinite(observed_progress)) {
          committed_mission_progress_m_ = std::max(committed_mission_progress_m_, observed_progress);
        }
      }
      // The horizon is generated in the direction the vehicle is actually
      // travelling at the splice, because the safety governor and local
      // trajectory splice must follow the current stopping direction. The
      // mission_route_tangent is kept separately for the monotonic progress
      // guard; safety lookahead, splice continuity, and route intent are
      // deliberately separate contracts.
      const auto planning_tangent = [&]() -> navigation_mapping::Vec3 {
        const auto mission_delta = requested_goal - state.position;
        const double mission_distance = mission_delta.norm();
        // A new mission request is a deliberate leg transition.  The measured
        // velocity still belongs to the previous leg (for example northbound
        // at [48, 5] while the next target is westbound at [41, 5]); using it
        // as the safety horizon direction sends the fallback past the new
        // waypoint and can create an unbounded reverse detour.  Keep measured
        // velocity for rolling replans of the same request, but seed a new
        // request from its actual goal direction.
        if (!goal_unchanged && std::isfinite(mission_distance) && mission_distance > 1e-6) {
          return (mission_delta / mission_distance).eval();
        }
        if (state.velocity.allFinite() && state.velocity.norm() > 0.1) {
          return state.velocity.normalized().eval();
        }
        if (std::isfinite(mission_distance) && mission_distance > 1e-6) {
          return (mission_delta / mission_distance).eval();
        }
        return navigation_mapping::Vec3::Zero();
      }();
      navigation_mapping::Vec3 planning_projection_tangent =
          navigation_mapping::Vec3::UnitX();
      if (planning_tangent.norm() > 1e-6) {
        planning_projection_tangent = planning_tangent.normalized();
      }
      const double map_forward_distance = forwardMapDistance(
          world, state.position, planning_tangent, planning_horizon_boundary_margin_m_);
      navigation_planning::HorizonPolicyConfig horizon_config;
      horizon_config.minimum_distance_m = planning_horizon_min_distance_m_;
      horizon_config.maximum_distance_m = planning_horizon_max_distance_m_;
      horizon_config.preview_time_s = planning_horizon_preview_time_s_;
      horizon_config.map_boundary_margin_m = 0.0;
      navigation_planning::HorizonRequest horizon_request;
      horizon_request.route.projected_arc_length_m = 0.0;
      const double bounded_map_forward_distance = std::isfinite(map_forward_distance)
                                                      ? map_forward_distance
                                                      : planning_horizon_max_distance_m_;
      horizon_request.route.route_length_m = bounded_map_forward_distance;
      horizon_request.route.usable_forward_distance_m = bounded_map_forward_distance;
      horizon_request.speed_mps = state.velocity.norm();
      horizon_request.max_deceleration_mps2 = planner_config_.limits.max_deceleration_mps2;
      horizon_request.pipeline_latency_s = switch_delay_s_ + safety_latency_s_;
      horizon_request.stop_margin_m = safety_stop_margin_m_;
      const auto planning_horizon = navigation_planning::HorizonPolicy{horizon_config}.compute(
          horizon_request);
      const double horizon_distance = planning_horizon.success
                                          ? planning_horizon.forward_distance_m
                                          : 0.0;
      last_planning_horizon_distance_m_ = horizon_distance;
      // The selector's cheap endpoint test is not a reachability proof. A
      // point can be KnownFree while the straight corridor from the current
      // splice pose crosses an Unknown pocket or a newly occupied voxel. Do
      // not reject that endpoint here solely because the cheap interpolated
      // ray is Unknown: in dual-planning mode the nominal branch is explicitly
      // allowed to commit a short unknown prefix, while A* and the dense
      // trajectory verifier remain authoritative and the safety branch still
      // fails closed. Rejecting the endpoint at this layer forced the local
      // selector to choose whichever side happened to be observed first,
      // producing the long upper detour seen in long_three_pillars.
      const double selector_resolution = world.resolution(
          navigation_mapping::WorldLayer::Inflated);
      const double selector_step = std::isfinite(selector_resolution) && selector_resolution > 0.0
                                       ? std::max(0.10, 0.5 * selector_resolution)
                                       : 0.10;
      const auto selector_start_index = world.worldToGrid(
          navigation_mapping::WorldLayer::Inflated, state.position);
      const auto selector_bounds = world.bounds(navigation_mapping::WorldLayer::Inflated);
      const double selector_start_radius =
          planner_config_.allow_unknown_start &&
                  std::isfinite(planner_config_.unknown_start_radius_m) &&
                  planner_config_.unknown_start_radius_m >= 0.0 &&
                  std::isfinite(selector_resolution) && selector_resolution > 0.0
              ? planner_config_.unknown_start_radius_m +
                    0.5 * selector_resolution * std::sqrt(3.0)
              : 0.0;
      const auto observed_corridor = [&](const navigation_mapping::Vec3& candidate) {
        if (!candidate.allFinite()) return false;
        const auto delta = candidate - state.position;
        const double distance = delta.norm();
        if (!std::isfinite(distance)) return false;
        const int samples = std::max(1, static_cast<int>(std::ceil(distance / selector_step)));
        for (int sample = 1; sample <= samples; ++sample) {
          const double alpha = static_cast<double>(sample) / samples;
          const auto point = state.position + alpha * delta;
          const auto index = world.worldToGrid(
              navigation_mapping::WorldLayer::Inflated, point);
          if (!selector_bounds.contains(index)) return false;
          const bool trusted_start = planner_config_.allow_unknown_start &&
                                     (index == selector_start_index ||
                                      (point - state.position).norm() <=
                                          selector_start_radius + 1e-9);
          if (!trusted_start && world.cellState(
                  navigation_mapping::WorldLayer::Inflated, index) !=
                  navigation_mapping::CellState::KnownFree) {
            return false;
          }
        }
        return true;
      };
      bool mission_progress_rejected_this_cycle = false;
      // Keep a previous local anchor as a soft monotonicity constraint while
      // its complete observed corridor remains valid. This is intentionally
      // weaker than the mission progress watermark: a newly occupied or
      // unknown corridor may invalidate the anchor and permit a safe retreat,
      // but a harmless map revision must not move the endpoint metres backward
      // and make the vehicle decelerate/re-accelerate on every replan.
      const std::optional<navigation_mapping::Vec3> previous_horizon_goal =
          goal_unchanged && last_local_subgoal_selected_ && last_effective_goal_.allFinite() &&
                  last_nominal_failure_code_ != navigation_planning::PlanFailureCode::NoPath &&
                  last_nominal_failure_code_ !=
                      navigation_planning::PlanFailureCode::CorridorInfeasible &&
                  last_nominal_failure_code_ !=
                      navigation_planning::PlanFailureCode::GoalOutsideBounds &&
                  last_nominal_failure_code_ != navigation_planning::PlanFailureCode::WorldChanged
              ? std::optional<navigation_mapping::Vec3>{last_effective_goal_}
              : std::nullopt;
      const bool previous_horizon_corridor_valid =
          previous_horizon_goal.has_value() && observed_corridor(*previous_horizon_goal);
      const auto previous_detour_lateral_direction =
          goal_unchanged ? last_detour_lateral_direction_ :
                           std::optional<navigation_mapping::Vec3>{};
      const auto previous_detour_obstacle_anchor =
          goal_unchanged ? last_detour_obstacle_anchor_ :
                           std::optional<navigation_mapping::Vec3>{};
      // The local selector should search along the mission leg by default.
      // A measured velocity tangent is retained only while a same-obstacle
      // detour is active; otherwise a one-cycle lateral avoidance tangent can
      // become the next cycle's global search direction and accumulate into
      // wall-following drift.
      const auto selector_forward_direction =
          previous_detour_lateral_direction.has_value() &&
                  previous_detour_obstacle_anchor.has_value()
              ? planning_tangent
              : mission_route_tangent;
      // A safety/recovery trajectory may temporarily rotate the measured
      // velocity away from the mission leg.  Do not let that measured
      // velocity redefine the rolling horizon and turn a transient braking
      // failure into a valid route behind the active waypoint.  A small
      // backward allowance is useful for a lateral obstacle escape, but a
      // multi-metre reverse route is a mission-progress violation and must
      // fail closed into the braking-stop path instead.
      const auto mission_progress_guard = [&](const navigation_mapping::Vec3& candidate) {
        if (!candidate.allFinite()) return false;
        const auto mission_delta = requested_goal - state.position;
        const double mission_distance = mission_delta.norm();
        if (!std::isfinite(mission_distance) || mission_distance <= 1e-6) return true;
        const auto mission_unit = mission_route_tangent.norm() > 1e-6
                                      ? mission_route_tangent
                                      : (mission_delta / mission_distance).eval();
        const double backward_allowance_m = std::max(0.5, 2.0 * selector_resolution);
        const double minimum_mission_progress = -backward_allowance_m;
        if ((candidate - state.position).dot(mission_unit) < minimum_mission_progress) {
          return false;
        }
        // The per-tick check above is insufficient when the vehicle has
        // already drifted laterally: a sequence of individually small
        // backward steps can still move the endpoint metres away from the
        // mission leg.  Compare against the greatest committed arc progress
        // in a fixed leg frame and permit only one map-cell of numerical /
        // obstacle-escape tolerance.
        if (mission_leg_initialized_ && mission_route_tangent.norm() > 1e-6) {
          const double candidate_progress =
              (candidate - mission_leg_start_position_).dot(mission_route_tangent);
          const double persistent_allowance = std::max(0.5, 2.0 * selector_resolution);
          if (!std::isfinite(candidate_progress) ||
              candidate_progress < committed_mission_progress_m_ - persistent_allowance) {
            mission_progress_rejected_this_cycle = true;
            return false;
          }
        }
        if (previous_horizon_corridor_valid) {
          const double previous_progress =
              (*previous_horizon_goal - mission_leg_start_position_).dot(mission_route_tangent);
          const double candidate_progress =
              (candidate - mission_leg_start_position_).dot(mission_route_tangent);
          const double anchor_retraction_allowance_m =
              std::max(1.0, 3.0 * selector_resolution);
          if (!std::isfinite(previous_progress) || !std::isfinite(candidate_progress) ||
              candidate_progress < previous_progress - anchor_retraction_allowance_m) {
            mission_progress_rejected_this_cycle = true;
            return false;
          }
        }
        return true;
      };
      // Keep the previous horizon anchor only as a soft continuity prior. It
      // is revalidated by the selector on this very planning tick and is
      // discarded as soon as it is behind the predicted splice state, no
      // longer KnownFree, or outside the current search window. This prevents
      // freshly revealed left/right detours from alternating every map update
      // without reintroducing a "reach the subgoal first" gate.
      // The mission waypoint may be outside the current sliding map. Keep the
      // bounded rolling horizon as the execution endpoint in that case; the
      // mission controller still evaluates acceptance against requested_goal.
      // A separate global reachability probe below prevents this local endpoint
      // from becoming an endless wall-following substitute when the actual
      // waypoint is already blocked in the observed map.
      LocalGoalSelection local_goal;
      if (!local_subgoal_enabled_) {
        // The compatibility parameter is a real policy switch.  With it off,
        // submit the mission waypoint directly; if it is outside the known
        // map the normal safety planner must fail closed rather than silently
        // inventing a rolling endpoint.
        local_goal.status = LocalGoalSelectionStatus::Direct;
        local_goal.goal = requested_goal;
        local_goal.tangent = mission_route_tangent;
        local_goal.forward_projection_m =
            (requested_goal - state.position).dot(mission_route_tangent);
      } else {
        local_goal = selectPlanningHorizon(
            world, state.position, requested_goal, local_goal_boundary_margin_m_, horizon_distance,
            previous_horizon_goal, selector_forward_direction, mission_progress_guard,
            previous_detour_lateral_direction, previous_detour_obstacle_anchor);
      }
      // The selector and A* read the rolling map through separate calls. A
      // lidar update can invalidate a just-selected direct waypoint between
      // those calls (the diagnostics then show Direct followed by an UNKNOWN
      // goal cell). Re-check that boundary and immediately fall back to the
      // bounded receding-horizon target; otherwise a harmless map revision
      // becomes a braking stop at every orthogonal waypoint.
      if (local_subgoal_enabled_ && local_goal.status == LocalGoalSelectionStatus::Direct) {
        const auto direct_index = world.worldToGrid(
            navigation_mapping::WorldLayer::Inflated, requested_goal);
        const auto direct_bounds = world.bounds(navigation_mapping::WorldLayer::Inflated);
        bool direct_horizon_known_free = direct_bounds.contains(direct_index) &&
                                         world.cellState(
                                             navigation_mapping::WorldLayer::Inflated,
                                             direct_index) ==
                                             navigation_mapping::CellState::KnownFree;
        if (!direct_horizon_known_free) {
          local_goal = selectPlanningHorizon(world, state.position, requested_goal,
                                             local_goal_boundary_margin_m_, horizon_distance,
                                             previous_horizon_goal, selector_forward_direction,
                                             mission_progress_guard,
                                             previous_detour_lateral_direction,
                                             previous_detour_obstacle_anchor);
        }
      }
      if (local_goal.usesSubGoal() && committed_local_altitude_m_.has_value() &&
          std::isfinite(*committed_local_altitude_m_) &&
          std::isfinite(selector_resolution) && selector_resolution > 0.0) {
        const double altitude_tolerance = std::max(0.35, 2.0 * selector_resolution);
        const double altitude_error =
            std::abs(local_goal.goal.z() - *committed_local_altitude_m_);
        if (std::isfinite(altitude_error) && altitude_error > altitude_tolerance) {
          if (planner_config_.allow_nominal_unknown) {
            // In dual-planning simulation, the nominal branch may carry a
            // short Unknown prefix, but it must keep the mission altitude.
            // Safety planning below derives a separate KnownFree endpoint.
            local_goal.goal.z() = *committed_local_altitude_m_;
            const auto corrected_delta = local_goal.goal - state.position;
            if (corrected_delta.norm() > 1e-6) {
              local_goal.tangent = corrected_delta.normalized();
              local_goal.forward_projection_m =
                  corrected_delta.dot(planning_projection_tangent);
            }
          } else {
            // Real/default operation has no permission to use Unknown space
            // as a level-flight shortcut. Do not silently descend to a
            // ground-adjacent KnownFree cell; fail closed and wait for map
            // evidence at the configured mission altitude.
            local_goal.status = LocalGoalSelectionStatus::NoUsableSubGoal;
          }
        }
      }
      if (mission_progress_rejected_this_cycle) ++mission_progress_rejection_count_;
      if (local_goal.occupied_on_forward_ray && local_goal.detour_obstacle_anchor.has_value() &&
          local_goal.detour_obstacle_anchor->allFinite()) {
        const auto detour_delta = local_goal.goal - state.position;
        const auto detour_lateral = detour_delta -
                                    detour_delta.dot(planning_tangent) * planning_tangent;
        if (detour_lateral.allFinite() && detour_lateral.norm() > 0.5 * selector_resolution) {
          last_detour_obstacle_anchor_ = local_goal.detour_obstacle_anchor;
          last_detour_lateral_direction_ = detour_lateral.normalized();
        }
      } else {
        // A clear forward ray means the current detour tangent is no longer
        // needed. Release it immediately so a side choice cannot remain a
        // mission-wide heading prior. Endpoint continuity is still provided
        // by previous_horizon_goal and the candidate continuity cost.
        last_detour_obstacle_anchor_.reset();
        last_detour_lateral_direction_.reset();
      }
      if (local_goal.usesSubGoal()) {
        if (!committed_local_altitude_m_.has_value() ||
            !std::isfinite(*committed_local_altitude_m_)) {
          committed_local_altitude_m_ = local_goal.goal.z();
        }
        // The selector has already proved the x/y detour cell known-free. If
        // the same cell is also known-free at the committed altitude, snap to
        // it before A*. This preserves a single smooth level through rolling
        // replans while retaining fail-closed behaviour when the altitude
        // projection is genuinely not observed yet.
        const auto altitude_goal = navigation_mapping::Vec3{
            local_goal.goal.x(), local_goal.goal.y(), *committed_local_altitude_m_};
        const auto altitude_index = world.worldToGrid(
            navigation_mapping::WorldLayer::Inflated, altitude_goal);
        const auto altitude_bounds = world.bounds(navigation_mapping::WorldLayer::Inflated);
        if (altitude_bounds.contains(altitude_index) &&
            world.cellState(navigation_mapping::WorldLayer::Inflated, altitude_index) ==
                navigation_mapping::CellState::KnownFree) {
          local_goal.goal = altitude_goal;
        }
        // This point is a rolling corridor anchor, not a completion target.
        // Do not retain it as hysteresis state: every full replan must be
        // allowed to choose a new anchor when the obstacle boundary or the
        // observed corridor changes.
        committed_local_goal_.reset();
      } else if (local_goal.status == LocalGoalSelectionStatus::Direct ||
                 !local_goal.success()) {
        committed_local_goal_.reset();
      }
      const auto previous_effective_goal = last_effective_goal_;
      const bool previous_was_subgoal = last_local_subgoal_selected_;
      last_local_goal_status_ = local_goal.status;
      last_local_subgoal_selected_ = local_goal.usesSubGoal();
      last_effective_goal_ = local_goal.goal;
      last_horizon_tangent_ = local_goal.tangent.allFinite() && local_goal.tangent.norm() > 1e-6
                                  ? local_goal.tangent.normalized()
                                  : planning_tangent;
      last_planning_state_position_ = state.position;
      last_planning_state_velocity_ = state.velocity;
      last_horizon_forward_projection_m_ = local_goal.forward_projection_m;
      last_horizon_progress_m_ = (local_goal.goal - state.position).dot(
          planning_projection_tangent);
      last_horizon_ray_occupied_ = local_goal.occupied_on_forward_ray;
      if (local_goal.success()) {
        const bool same_endpoint = previous_was_subgoal &&
                                    (local_goal.goal - previous_effective_goal).norm() <=
                                        std::max(0.05, 0.5 * world.resolution(
                                                          navigation_mapping::WorldLayer::Inflated));
        if (same_endpoint) {
          ++horizon_endpoint_repeat_count_;
        } else {
          ++horizon_endpoint_change_count_;
        }
      }
      if (local_goal.usesSubGoal()) {
        ++local_subgoal_selected_count_;
      } else if (!local_goal.success()) {
        ++local_subgoal_failure_count_;
      }
      const auto motion_direction = [&]() {
        if (local_goal.tangent.allFinite() && local_goal.tangent.norm() > 0.1) {
          return local_goal.tangent.normalized();
        }
        if (state.velocity.norm() > 0.1) return state.velocity.normalized();
        const auto delta = (local_goal.success() ? local_goal.goal : requested_goal) - state.position;
        return delta.norm() > 1e-6 ? delta.normalized() : navigation_mapping::Vec3::Zero();
      }();
      const double map_resolution = world.resolution(navigation_mapping::WorldLayer::Inflated);
      const double step = std::isfinite(map_resolution) && map_resolution > 0.0
                              ? std::max(0.05, 0.5 * map_resolution)
                              : 0.1;
      double known_free_horizon = 0.0;
      const auto bounds = world.bounds(navigation_mapping::WorldLayer::Inflated);
      const auto start_index = world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                                 state.position);
      const double start_cell_radius =
          planner_config_.allow_unknown_start &&
                  std::isfinite(planner_config_.unknown_start_radius_m) &&
                  planner_config_.unknown_start_radius_m >= 0.0
              ? planner_config_.unknown_start_radius_m + 0.5 * map_resolution * std::sqrt(3.0)
              : 0.0;
      for (double distance = 0.0; distance <= safety_visibility_horizon_m_; distance += step) {
        const auto sample_position = state.position + distance * motion_direction;
        const auto index = world.worldToGrid(navigation_mapping::WorldLayer::Inflated,
                                             sample_position);
        if (!bounds.contains(index)) break;
        const auto cell = world.cellState(navigation_mapping::WorldLayer::Inflated, index);
        // FAST-LIO can leave the vehicle voxel (and one adjacent inflated
        // voxel) UNKNOWN even while the sensor has a valid local map.  Use
        // the same bounded unknown-start radius as A* instead of requiring an
        // exact grid index match; this prevents a zero-speed stop at mission
        // activation without making unknown space generally traversable.
        const bool trusted_start = planner_config_.allow_unknown_start &&
                                   (index == start_index ||
                                    (sample_position - state.position).norm() <=
                                        start_cell_radius + 1e-9);
        if (cell != navigation_mapping::CellState::KnownFree && !trusted_start) break;
        known_free_horizon = distance;
      }
      const double deceleration = planner_config_.limits.max_deceleration_mps2;
      double adaptive_velocity_cap = planner_config_.limits.max_velocity_mps;
      if (motion_direction.norm() > 0.0 && known_free_horizon > 0.0 && deceleration > 0.0) {
        const auto safeAt = [&](double speed) {
          return speed * speed / (2.0 * deceleration) + speed * safety_latency_s_ +
                 safety_stop_margin_m_ <= known_free_horizon;
        };
        double low = 0.0;
        double high = planner_config_.limits.max_velocity_mps;
        for (int iteration = 0; iteration < 32; ++iteration) {
          const double middle = 0.5 * (low + high);
          if (safeAt(middle)) low = middle;
          else high = middle;
        }
        adaptive_velocity_cap = low;
      } else if (known_free_horizon <= safety_stop_margin_m_) {
        adaptive_velocity_cap = 0.0;
      }
      last_known_free_horizon_m_ = known_free_horizon;
      last_adaptive_velocity_cap_mps_ = adaptive_velocity_cap;
      navigation_planning::Goal goal;
      goal.position = local_goal.success() ? local_goal.goal : requested_goal;
      goal.velocity_limit_mps = adaptive_velocity_cap > 1e-6
                                    ? adaptive_velocity_cap
                                    : 0.05;
      const bool mission_pass_through =
          goal_message.behavior == navigation_interfaces::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH &&
          goal_message.has_next_target;
      goal.terminal = !local_goal.usesSubGoal() && !mission_pass_through;
      if (!goal.terminal) {
        // Carry motion through a receding-horizon local goal. Prefer the
        // current measured velocity when it already points toward the local
        // target; otherwise seed a conservative continuation tangent. The
        // planner applies its design-limit clamp and the verifier remains the
        // final authority.
        // A local sub-goal is a point on the currently observed corridor. Its
        // continuation tangent must follow that corridor, not jump toward a
        // later mission waypoint that may be orthogonal to the local route
        // (for example while skirting a pillar).  Passing the future mission
        // target here injects a sharp endpoint direction into every quintic
        // replan and forces the time scaler to stretch otherwise short arcs.
        // Only the final, direct pass-through segment inherits the next
        // mission direction so the vehicle can turn before reaching the
        // waypoint without stopping there.
        const navigation_mapping::Vec3 direction_to_goal = goal.position - state.position;
        // Do not impose the next-leg tangent on a very short remaining
        // pass-through segment. Near a corner, that tangent can be almost
        // opposite to the current displacement (for example the final metre
        // of a 90-degree turn), so a high-order spline has no room to rotate
        // without leaving the verified corridor. Aim at the current waypoint
        // until it is accepted; the next mission event then supplies the next
        // leg tangent while preserving a non-zero continuation speed.
        // Blend the next-leg tangent early enough for the measured velocity
        // to rotate before a sharp pass-through corner. The acceptance radius
        // alone is too late at flight speed: the vehicle can enter the radius
        // while still moving almost entirely along the previous leg, then the
        // next goal requires an infeasible reverse turn. This is only a
        // trajectory-direction prior; MissionController still requires the
        // measured position to enter the waypoint radius for acceptance.
        const double measured_corner_speed = state.velocity.allFinite()
                                                 ? state.velocity.norm()
                                                 : 0.0;
        const double corner_deceleration = std::max(
            0.5, planner_config_.limits.max_deceleration_mps2);
        const double corner_lookahead_m = std::clamp(
            local_goal_switch_distance_m_ +
                measured_corner_speed * (switch_delay_s_ + safety_latency_s_) +
                measured_corner_speed * measured_corner_speed /
                    (2.0 * corner_deceleration),
            std::max(1.0, 1.5 * goal_message.acceptance_radius_m), 5.0);
        // A local horizon endpoint is not the mission corner.  Use the
        // distance to the requested waypoint so the next-leg tangent cannot
        // be activated repeatedly at every rolling anchor along a long leg.
        const double distance_to_mission_goal = (requested_goal - state.position).norm();
        const bool near_pass_through_corner =
            mission_pass_through && std::isfinite(distance_to_mission_goal) &&
            distance_to_mission_goal <= corner_lookahead_m;
        const navigation_mapping::Vec3 next_leg_direction = navigation_mapping::Vec3{
            goal_message.next_target.x, goal_message.next_target.y,
            goal_message.next_target.z} - requested_goal;
        navigation_mapping::Vec3 direction = [&]() {
          if (mission_pass_through && near_pass_through_corner &&
              next_leg_direction.norm() > 1e-6) {
            // This is deliberately gated by the actual mission waypoint,
            // not the local anchor. It gives the planner a bounded corner
            // tangent before acceptance while preserving the local goal's
            // geometry on the long incoming leg.
            return next_leg_direction.normalized();
          }
          if (local_goal.usesSubGoal() && local_goal.tangent.allFinite() &&
              local_goal.tangent.norm() > 1e-6) {
            return local_goal.tangent.normalized();
          }
          if (!goal_unchanged && direction_to_goal.norm() > 1e-6) {
            // The request changed at a pass-through corner.  Turn toward the
            // current waypoint immediately; retaining the old velocity here
            // would make the endpoint velocity and safety horizon follow the
            // previous leg after the mission controller has already advanced.
            return direction_to_goal;
          }
          // Until a pass-through waypoint is actually inside its acceptance
          // radius, retain the incoming tangent.  Applying the next-leg
          // direction several metres early was the source of the observed
          // negative-x command while approaching the north corner.
          if (state.velocity.allFinite() && state.velocity.norm() > 0.1) {
            return state.velocity;
          }
          return direction_to_goal;
        }();
        const bool local_goal_requires_reversal =
            state.velocity.allFinite() && state.velocity.norm() > 0.1 &&
            direction_to_goal.allFinite() && direction_to_goal.norm() > 1e-6 &&
            direction_to_goal.normalized().dot(state.velocity.normalized()) < -0.25;
        if (state.velocity.allFinite() && state.velocity.norm() > 0.1 &&
            direction.norm() > 1e-6 && !near_pass_through_corner &&
            !local_goal_requires_reversal) {
          const auto incoming = state.velocity.normalized();
          if (direction.normalized().dot(incoming) < -0.25) {
            ++horizon_backward_rejection_count_;
            direction = incoming;
          }
        }
        const double direction_norm = direction.norm();
        // A fixed endpoint-speed floor is safe on a straight corridor but is
        // pathological at a visibility-limited corner: the old rule carried
        // the full scalar speed through a 60--90 degree heading change and
        // made PX4 chase an infeasible lateral impulse.  Keep the high speed
        // when the measured velocity is aligned with the new corridor, and
        // smoothly lower only the tangential speed at a corner.  This is a
        // velocity-space equivalent of curvature-aware time allocation and
        // preserves continuity without inserting a stop at every sub-goal.
        double continuation_speed = 0.0;
        if (direction_norm > 1e-6 && std::isfinite(goal.velocity_limit_mps) &&
            goal.velocity_limit_mps > 1e-6) {
          const auto direction_unit = direction / direction_norm;
          const double measured_speed = state.velocity.norm();
          const double alignment = measured_speed > 1e-3
                                       ? std::clamp(state.velocity.normalized().dot(direction_unit),
                                                    0.0, 1.0)
                                       : 0.0;
          // Keep one continuation policy for open and detour corridors.
          // Raising the terminal floor only in open space increases the
          // endpoint velocity exactly when a map revision can change the
          // local tangent. The B-spline then stretches the rolling segment
          // and PX4 receives a slower prefix. Continuity is governed by the
          // measured splice velocity; this fraction is only the bounded
          // minimum hand-over speed.
          const double nominal_fraction = local_goal_continuation_speed_fraction_;
          // The corner floor is deliberately below the nominal floor, but
          // never zero.  It gives the vehicle a continuous tangent through a
          // sharp detour while leaving acceleration/deceleration limits to
          // the planner and verifier.
          const double corner_fraction = std::min(0.45, 0.5 * nominal_fraction);
          const double speed_floor_fraction =
              corner_fraction + (nominal_fraction - corner_fraction) * alignment;
          const double speed_floor = speed_floor_fraction * goal.velocity_limit_mps;
          const double projected_speed = std::max(0.0, state.velocity.dot(direction_unit));
          continuation_speed = std::clamp(
              std::max(projected_speed, speed_floor), speed_floor,
              0.995 * goal.velocity_limit_mps);
        }
        if (direction_norm > 1e-6 && std::isfinite(continuation_speed)) {
          goal.terminal_velocity = (continuation_speed / direction_norm) * direction;
        } else {
          goal.terminal_velocity = navigation_mapping::Vec3::Zero();
        }
      }
      // A bounded local endpoint is useful when the mission waypoint is
      // outside the sliding map, but it must not become an unbounded
      // wall-following substitute for a waypoint that is already inside the
      // observed map. When the selector reports an occupied forward ray,
      // probe the actual mission goal with the conservative known-free
      // planner. A topological failure forces the normal safety-stop fallback;
      // long routes remain eligible for rolling execution because an
      // out-of-bounds mission goal is not probed until it enters the map.
      bool mission_goal_unreachable = false;
      if (local_goal.usesSubGoal() && local_goal.occupied_on_forward_ray) {
        const auto mission_goal_index = world.worldToGrid(
            navigation_mapping::WorldLayer::Inflated, requested_goal);
        const auto mission_goal_bounds = world.bounds(navigation_mapping::WorldLayer::Inflated);
        const bool mission_goal_known_free =
            mission_goal_bounds.contains(mission_goal_index) &&
            world.cellState(navigation_mapping::WorldLayer::Inflated, mission_goal_index) ==
                navigation_mapping::CellState::KnownFree;
        if (mission_goal_known_free ||
            (fail_closed_on_unknown_mission_goal_ &&
             mission_goal_bounds.contains(mission_goal_index))) {
          auto mission_goal_probe = goal;
          mission_goal_probe.position = requested_goal;
          mission_goal_probe.terminal = true;
          mission_goal_probe.terminal_velocity = navigation_mapping::Vec3::Zero();
          const auto probe = planner_.plan(state, mission_goal_probe, world);
          mission_goal_unreachable =
              !probe.success &&
              (probe.failure_code == navigation_planning::PlanFailureCode::NoPath ||
               probe.failure_code == navigation_planning::PlanFailureCode::GoalOccupied ||
               probe.failure_code == navigation_planning::PlanFailureCode::CorridorInfeasible);
        }
      }
      last_goal_terminal_ = goal.terminal;
      last_terminal_velocity_ = goal.terminal_velocity;
      // Nominal planning may intentionally terminate in a freshly observed
      // Unknown cell.  The safety candidate must not share that endpoint:
      // A* correctly rejects Unknown for safety, which previously converted
      // every rolling cycle into a braking stop when the selector and map
      // update raced.  Keep nominal on the requested rolling endpoint, but
      // derive a separately verified KnownFree endpoint for the backup.
      navigation_planning::Goal safety_goal = goal;
      const auto isKnownFreePlanningCell = [&](const navigation_mapping::Vec3& point) {
        if (!point.allFinite()) return false;
        const auto safety_bounds = world.bounds(navigation_mapping::WorldLayer::Inflated);
        const auto safety_index = world.worldToGrid(
            navigation_mapping::WorldLayer::Inflated, point);
        return safety_bounds.contains(safety_index) &&
               world.cellState(navigation_mapping::WorldLayer::Inflated, safety_index) ==
                   navigation_mapping::CellState::KnownFree;
      };
      if (planner_config_.allow_nominal_unknown && local_subgoal_enabled_ &&
          !isKnownFreePlanningCell(safety_goal.position)) {
        const double map_resolution = world.resolution(navigation_mapping::WorldLayer::Inflated);
        const double retry_step = std::isfinite(map_resolution) && map_resolution > 0.0
                                      ? std::max(0.4, 2.0 * map_resolution)
                                      : 0.4;
        const double initial_distance = std::isfinite(last_planning_horizon_distance_m_) &&
                                                last_planning_horizon_distance_m_ > 0.0
                                            ? last_planning_horizon_distance_m_
                                            : planning_horizon_max_distance_m_;
        for (int retry = 0; retry < 8; ++retry) {
          const double retry_distance = std::max(
              retry_step, initial_distance - static_cast<double>(retry) * retry_step);
          const auto safety_horizon = selectPlanningHorizon(
              world, state.position, requested_goal, local_goal_boundary_margin_m_,
              retry_distance, std::nullopt, planning_tangent, mission_progress_guard,
              std::nullopt, std::nullopt);
          if (!safety_horizon.success() ||
              !isKnownFreePlanningCell(safety_horizon.goal)) {
            continue;
          }
          safety_goal.position = safety_horizon.goal;
          safety_goal.terminal = false;
          safety_goal.terminal_velocity = navigation_mapping::Vec3::Zero();
          break;
        }
      }
      if (local_goal.occupied_on_forward_ray) {
        // A safety route that ends at the visible edge of a blocked ray must
        // not carry continuation velocity into the next Unknown/occupied
        // region. The nominal branch may keep its rolling tangent for A/B
        // diagnostics, but the executable safety branch must stop at the
        // verified endpoint and hold while the map reveals a detour.
        safety_goal.terminal = true;
        safety_goal.terminal_velocity = navigation_mapping::Vec3::Zero();
      }
      last_safety_goal_position_ = safety_goal.position;
      last_safety_goal_known_free_ = isKnownFreePlanningCell(safety_goal.position);
      const auto validate = [&](navigation_planning::PlanResult candidate) {
        if (!candidate.success) return candidate;
        const auto verification = trajectory_verifier_.verify(
            candidate.trajectory, state, candidate.role, world);
        last_verification_time_us_ = verification.statistics.verification_time_us;
        if (candidate.role == navigation_planning::PlanRole::Safety) {
          last_safety_verification_failure_ = verification.failure_code;
        } else {
          // Committed is the known-free baseline used by the normal runtime;
          // retain it with nominal diagnostics so planner and verifier causes
          // remain distinguishable when a safety fallback replaces it.
          last_nominal_verification_failure_ = verification.failure_code;
        }
        if (!verification.success) {
          ++verification_failure_count_;
          candidate.success = false;
          candidate.trajectory.points.clear();
          candidate.trajectory.duration_s = 0.0;
          candidate.failure_code = mapVerificationFailure(verification.failure_code);
        }
        return candidate;
      };
      const auto rejectNominalTrajectoryThatBacktracks =
          [&](navigation_planning::PlanResult candidate) {
            if (!candidate.success || candidate.role == navigation_planning::PlanRole::Safety ||
                !mission_leg_initialized_ || mission_route_tangent.norm() <= 1e-6 ||
                candidate.trajectory.points.size() < 2U) {
              return candidate;
            }
            // The local selector constrains the endpoint, but the trajectory
            // generator/A* path can still take a short reverse manoeuvre before
            // reaching that endpoint. On a rolling mission this is not a
            // harmless geometric detail: the next 5 Hz splice starts from the
            // backward prefix, then the following plan points forward again,
            // which produces the observed crawl/chattering loop. Nominal
            // execution must preserve mission-leg progress over its committed
            // prefix. A safety route remains allowed to retreat because it is
            // fail-closed recovery and is not selected for nominal flight.
            const auto tangent = mission_route_tangent.normalized();
            const double first_progress =
                (candidate.trajectory.points.front().position - mission_leg_start_position_)
                    .dot(tangent);
            const double backward_allowance = std::max(0.5, 2.0 * selector_resolution);
            if (!std::isfinite(first_progress) || !std::isfinite(backward_allowance)) {
              candidate.success = false;
              candidate.failure_code = navigation_planning::PlanFailureCode::TrajectoryInvalid;
              ++trajectory_progress_rejection_count_;
              return candidate;
            }
            for (const auto& point : candidate.trajectory.points) {
              const double progress = (point.position - mission_leg_start_position_).dot(tangent);
              if (!std::isfinite(progress) || progress < first_progress - backward_allowance) {
                candidate.success = false;
                candidate.failure_code =
                    navigation_planning::PlanFailureCode::TrajectoryInvalid;
                ++trajectory_progress_rejection_count_;
                return candidate;
              }
            }
            return candidate;
          };
      const auto rejectCandidate = [](navigation_planning::PlanResult candidate,
                                       navigation_planning::PlanFailureCode failure) {
        candidate.success = false;
        candidate.trajectory.points.clear();
        candidate.trajectory.duration_s = 0.0;
        candidate.failure_code = failure;
        return candidate;
      };
      const auto rejectSafetyMotionPastBlockedRay =
          [&](navigation_planning::PlanResult candidate) {
            if (!candidate.success || candidate.role != navigation_planning::PlanRole::Safety ||
                !safety_goal.terminal || candidate.trajectory.points.empty()) {
              return candidate;
            }
            const auto& endpoint = candidate.trajectory.points.back();
            const auto forward = planning_projection_tangent.allFinite() &&
                                         planning_projection_tangent.norm() > 1e-6
                                     ? planning_projection_tangent.normalized()
                                     : navigation_mapping::Vec3::Zero();
            const double forward_velocity = endpoint.velocity.allFinite() && forward.norm() > 0.5
                                                ? endpoint.velocity.dot(forward)
                                                : std::numeric_limits<double>::infinity();
            if (!endpoint.velocity.allFinite() || !std::isfinite(forward_velocity) ||
                forward_velocity > 0.05) {
              // Every safety fallback shares this boundary. If the visible
              // forward ray is blocked, forward motion into that frontier is
              // not a safe route. A verified lateral detour may continue with
              // a non-zero tangential velocity; rejecting that motion would
              // turn a recoverable obstacle into an unnecessary mission
              // failure. Do not let prefix or preserved-trajectory fallbacks
              // bypass this forward-motion invariant.
              return rejectCandidate(std::move(candidate),
                                     navigation_planning::PlanFailureCode::TrajectoryInvalid);
            }
            return candidate;
          };
      const auto stabilizeStationarySafetyStop =
          [&](navigation_planning::PlanResult candidate) {
            if (!candidate.success || candidate.role != navigation_planning::PlanRole::Safety ||
                candidate.safety_kind != navigation_planning::SafetyPlanKind::BrakingStop ||
                !std::isfinite(candidate.trajectory.duration_s) ||
                candidate.trajectory.duration_s > 1e-6 ||
                candidate.trajectory.points.empty()) {
              return candidate;
            }
            // Planner-level safety-stop semantics intentionally use a single
            // point when the measured speed is already below the settling
            // deadband. That is correct for the pure planner API, but a
            // pass-through runtime goal may see map revisions while this stop
            // is latched. Republishing a zero-duration trajectory then makes
            // PX4 expire the command every tick and creates a safety-stop
            // chattering loop. Convert it into a verified one-second hold at
            // the current state; the next planning tick can still replace it
            // with a newly verified route when the map reveals one.
            navigation_planning::TimeParameterizedTrajectory hold;
            navigation_planning::TrajectoryPoint start;
            start.position = state.position;
            start.velocity = navigation_mapping::Vec3::Zero();
            start.acceleration = navigation_mapping::Vec3::Zero();
            start.time_from_start_s = 0.0;
            auto end = start;
            end.time_from_start_s = 1.0;
            hold.points = {start, end};
            hold.duration_s = 1.0;
            navigation_planning::VehicleState hold_state;
            hold_state.position = state.position;
            hold_state.velocity = navigation_mapping::Vec3::Zero();
            hold_state.acceleration = navigation_mapping::Vec3::Zero();
            const auto verification = trajectory_verifier_.verify(
                hold, hold_state, navigation_planning::PlanRole::Safety, world);
            if (!verification.success) {
              ++verification_failure_count_;
              last_safety_verification_failure_ = verification.failure_code;
              candidate.success = false;
              candidate.trajectory.points.clear();
              candidate.trajectory.duration_s = 0.0;
              candidate.failure_code = mapVerificationFailure(verification.failure_code);
              return candidate;
            }
            candidate.trajectory = hold;
            candidate.statistics.corridor.segment_count = 1U;
            candidate.statistics.corridor.checked_sample_count = 2U;
            candidate.statistics.trajectory_optimization.sampled_point_count = 2U;
            candidate.statistics.trajectory_optimization.maximum_velocity_mps = 0.0;
            candidate.statistics.trajectory_optimization.maximum_acceleration_mps2 = 0.0;
            candidate.statistics.trajectory_optimization.maximum_deceleration_mps2 = 0.0;
            candidate.statistics.trajectory_optimization.maximum_jerk_mps3 = 0.0;
            candidate.statistics.trajectory_optimization.trajectory_length_m = 0.0;
            candidate.statistics.trajectory_optimization.duration_s = 1.0;
            last_safety_verification_failure_ =
                navigation_planning::VerificationFailureCode::None;
            return candidate;
          };
      const auto extendMovingSafetyStopFrom =
          [&](navigation_planning::PlanResult candidate,
              const navigation_planning::VehicleState& verification_state) {
            if (!candidate.success || candidate.role != navigation_planning::PlanRole::Safety ||
                candidate.safety_kind != navigation_planning::SafetyPlanKind::BrakingStop ||
                safety_stop_hold_duration_s_ <= 0.0 ||
                candidate.trajectory.points.size() < 2U ||
                !std::isfinite(candidate.trajectory.duration_s) ||
                candidate.trajectory.duration_s <= 1e-6) {
              return candidate;
            }
            const auto& stop = candidate.trajectory.points.back();
            if (!stop.position.allFinite() || !stop.velocity.allFinite() ||
                !stop.acceleration.allFinite() || stop.velocity.norm() > 1e-3 ||
                stop.acceleration.norm() > 1e-3) {
              return candidate;
            }
            auto extended = candidate;
            auto hold = stop;
            hold.time_from_start_s = candidate.trajectory.duration_s +
                                     safety_stop_hold_duration_s_;
            hold.velocity = navigation_mapping::Vec3::Zero();
            hold.acceleration = navigation_mapping::Vec3::Zero();
            hold.jerk = navigation_mapping::Vec3::Zero();
            hold.snap = navigation_mapping::Vec3::Zero();
            extended.trajectory.points.push_back(hold);
            extended.trajectory.duration_s = hold.time_from_start_s;
            extended.statistics.trajectory_optimization.duration_s = hold.time_from_start_s;

            // The original stop was already verified. Re-verify the appended
            // stationary suffix so the hold remains subject to the same map,
            // bounds, and dynamic contracts as the braking phase.
            const auto verification = trajectory_verifier_.verify(
                extended.trajectory, verification_state,
                navigation_planning::PlanRole::Safety, world);
            if (!verification.success) {
              ++verification_failure_count_;
              last_safety_verification_failure_ = verification.failure_code;
              extended.success = false;
              extended.trajectory.points.clear();
              extended.trajectory.duration_s = 0.0;
              extended.failure_code = mapVerificationFailure(verification.failure_code);
              return extended;
            }
            return extended;
          };
      const auto extendMovingSafetyStop =
          [&](navigation_planning::PlanResult candidate) {
            return extendMovingSafetyStopFrom(std::move(candidate), state);
          };
      bool immediate_start_guard_recovery_selected = false;
      const auto enforceImmediateTrajectoryStart =
          [&](navigation_planning::PlanResult candidate) {
            if (!candidate.success || plan_valid_from_delay_s_ > 1e-9 ||
                candidate.trajectory.points.empty()) {
              return candidate;
            }
            const auto& first = candidate.trajectory.points.front();
            const double residual =
                (first.position - measured_state.position).norm();
            last_trajectory_start_position_residual_m_ = residual;
            if (std::isfinite(residual) &&
                residual <= trajectory_start_position_error_m_) {
              return candidate;
            }

            ++trajectory_start_guard_rejection_count_;
            RCLCPP_WARN(
                get_logger(),
                "Rejecting immediate trajectory start: residual=%.3f m limit=%.3f m "
                "measured=(%.3f,%.3f,%.3f) planned=(%.3f,%.3f,%.3f); "
                "recovering from measured state",
                residual, trajectory_start_position_error_m_,
                measured_state.position.x(), measured_state.position.y(),
                measured_state.position.z(), first.position.x(), first.position.y(),
                first.position.z());

            auto safety_state = measured_state;
            safety_state.acceleration = navigation_mapping::Vec3::Zero();
            auto stop_candidate = extendMovingSafetyStopFrom(
                stabilizeStationarySafetyStop(validate(
                    planner_.planSafetyStop(safety_state, world))),
                measured_state);
            if (stop_candidate.success) {
              stop_candidate.role = navigation_planning::PlanRole::Safety;
              stop_candidate.safety_kind =
                  navigation_planning::SafetyPlanKind::BrakingStop;
              stop_candidate.world_generation = world.generation();
              stop_candidate.world_revision = world.revision();
              immediate_start_guard_recovery_selected = true;
              return stop_candidate;
            }

            return rejectCandidate(
                std::move(candidate),
                navigation_planning::PlanFailureCode::TrajectoryInvalid);
          };
      if (!local_goal.success() || mission_goal_unreachable) {
        if (mission_goal_unreachable) {
          result.failure_code = navigation_planning::PlanFailureCode::NoPath;
        } else {
          result.failure_code =
              local_goal.status == LocalGoalSelectionStatus::StartOutsideBounds
                  ? navigation_planning::PlanFailureCode::StartOutsideBounds
                  : navigation_planning::PlanFailureCode::GoalOutsideBounds;
        }
      } else if (braking_stop_latched) {
        // Once a verified braking stop has been selected, map revisions and
        // rolling timer ticks may only replace it with another verified
        // braking stop. Never restart nominal execution at the same waypoint.
        auto safety_state = state;
        safety_state.acceleration = navigation_mapping::Vec3::Zero();
        result = extendMovingSafetyStop(stabilizeStationarySafetyStop(
            validate(planner_.planSafetyStop(safety_state, world))));
        safety_candidate = result;
        last_safety_failure_code_ = result.failure_code;
      } else {
        // Always produce both candidates for the active local horizon. In the
        // normal runtime the nominal candidate is the conservative known-free
        // plan; the optional simulation flag only changes its unknown-space
        // policy. The safety route/stop is verified independently so reports
        // show which branch was selected instead of hiding safety planning
        // behind a planner failure.
        ++nominal_plan_count_;
        auto nominal_result = planner_config_.allow_nominal_unknown
                                  ? planner_.planNominal(state, goal, world)
                                  : planner_.plan(state, goal, world);
        nominal_result = rejectNominalTrajectoryThatBacktracks(std::move(nominal_result));
        if (nominal_result.success) {
          full_nominal_visualization_trajectory = nominal_result.trajectory;
        }
        nominal_candidate = nominal_result;
        ++safety_route_plan_count_;
        auto safety_result = planner_.planSafetyRoute(state, safety_goal, world);
        last_safety_route_failure_code_ = safety_result.failure_code;
        safety_result = rejectSafetyMotionPastBlockedRay(validate(safety_result));
        if (!safety_result.success) {
          last_safety_route_failure_code_ = safety_result.failure_code;
          // A KnownFree endpoint is not sufficient for a safety route: the
          // lateral detour selected for nominal planning may still be
          // separated from the vehicle by Unknown cells.  Before falling
          // back to a one-point brake, search the already observed forward
          // prefix and use the farthest KnownFree endpoint that A* can
          // actually connect to.  This preserves fail-closed behaviour while
          // preventing an avoidable braking-stop loop in front of a newly
          // revealed obstacle.
          const double prefix_step = std::isfinite(map_resolution) && map_resolution > 0.0
                                         ? std::max(0.5, 3.0 * map_resolution)
                                         : 0.6;
          const double prefix_limit = std::min(
              std::max(prefix_step, last_known_free_horizon_m_ - safety_stop_margin_m_),
              std::max(prefix_step, last_planning_horizon_distance_m_));
          for (int retry = 0; retry < 8 && !safety_result.success; ++retry) {
            const double prefix_distance = prefix_limit -
                                           static_cast<double>(retry) * prefix_step;
            if (!std::isfinite(prefix_distance) || prefix_distance <= 0.5) break;
            // The measured velocity is allowed to point backwards while a
            // braking stop or a rejected handover is settling. It is not a
            // valid direction for a KnownFree prefix: using it here makes a
            // safety retry walk the endpoint behind the active waypoint and
            // can also inherit a vertical settling component. Keep the
            // prefix on the committed mission leg; an active obstacle detour
            // is selected by planSafetyRoute/A* rather than by this fallback
            // ray.
            const auto safety_prefix_tangent = mission_route_tangent.norm() > 1e-6
                                                   ? mission_route_tangent
                                                   : planning_projection_tangent;
            const auto prefix_position = state.position + prefix_distance * safety_prefix_tangent;
            if (!isKnownFreePlanningCell(prefix_position)) continue;
            auto prefix_goal = safety_goal;
            prefix_goal.position = prefix_position;
            prefix_goal.terminal = true;
            prefix_goal.terminal_velocity = navigation_mapping::Vec3::Zero();
            auto prefix_result = planner_.planSafetyRoute(state, prefix_goal, world);
            last_safety_route_failure_code_ = prefix_result.failure_code;
            prefix_result = rejectSafetyMotionPastBlockedRay(validate(prefix_result));
            if (prefix_result.success) {
              safety_goal = prefix_goal;
              last_safety_goal_position_ = safety_goal.position;
              last_safety_goal_known_free_ = true;
              safety_result = std::move(prefix_result);
            }
          }
        }
        if (safety_result.success) ++safety_route_verified_count_;
        if (!safety_result.success && goal_unchanged && last_plan_success_ &&
            last_planned_trajectory_.has_value() && last_plan_time_ns_ > 0 &&
            current_state_fresh && latest_odometry.has_value()) {
          // A map update can invalidate the newly selected safety endpoint
          // while the already committed prefix remains a valid known-free
          // handover. Reuse that prefix only after checking actual tracking
          // error and verifying it as a Safety trajectory; never reuse the
          // optimistic nominal suffix or bypass the fail-closed contract.
          const double elapsed_s = static_cast<double>(
              get_clock()->now().nanoseconds() - last_plan_time_ns_) / 1e9;
          const auto expected = interpolateTrajectory(
              *last_planned_trajectory_, std::max(0.0, elapsed_s));
          const auto current = navigation_mapping::Vec3{
              latest_odometry->pose.pose.position.x, latest_odometry->pose.pose.position.y,
              latest_odometry->pose.pose.position.z};
          const double tracking_error = (current - expected.position).norm();
          if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
              std::isfinite(tracking_error) && tracking_error < replan_tracking_error_m_) {
            const auto remaining = remainingTrajectory(*last_planned_trajectory_, elapsed_s);
            if (remaining.points.size() >= 2U && remaining.duration_s > 0.05) {
              navigation_planning::VehicleState verification_state;
              verification_state.position = remaining.points.front().position;
              verification_state.velocity = remaining.points.front().velocity;
              verification_state.acceleration = remaining.points.front().acceleration;
              const auto verification = trajectory_verifier_.verify(
                  remaining, verification_state, navigation_planning::PlanRole::Safety, world);
              if (verification.success) {
                navigation_planning::PlanResult preserved;
                preserved.success = true;
                preserved.role = navigation_planning::PlanRole::Safety;
                preserved.safety_kind = navigation_planning::SafetyPlanKind::Route;
                preserved.failure_code = navigation_planning::PlanFailureCode::None;
                preserved.world_generation = world.generation();
                preserved.world_revision = world.revision();
                preserved.trajectory = remaining;
                safety_result = rejectSafetyMotionPastBlockedRay(std::move(preserved));
                last_safety_verification_failure_ =
                    navigation_planning::VerificationFailureCode::None;
              }
            }
          }
        }
        if (!safety_result.success) {
          auto safety_state = state;
          safety_state.acceleration = navigation_mapping::Vec3::Zero();
          safety_result = extendMovingSafetyStop(stabilizeStationarySafetyStop(
              validate(planner_.planSafetyStop(safety_state, world))));
        }
        safety_candidate = safety_result;
        const auto recordCandidateTelemetry = [&](const navigation_planning::PlanResult& candidate,
                                                  bool nominal) {
          const auto& optimization = candidate.statistics.trajectory_optimization;
          if (nominal) {
            last_nominal_raw_path_node_count_ = optimization.raw_path_node_count;
            last_nominal_corridor_checked_count_ =
                candidate.statistics.corridor.checked_sample_count;
            last_nominal_corridor_blocked_count_ =
                candidate.statistics.corridor.blocked_sample_count;
            last_nominal_collision_free_ = optimization.collision_free;
            last_nominal_dynamic_limits_satisfied_ = optimization.dynamic_limits_satisfied;
            last_nominal_duration_s_ = optimization.duration_s;
            last_nominal_max_velocity_mps_ = optimization.maximum_velocity_mps;
            last_nominal_max_acceleration_mps2_ = optimization.maximum_acceleration_mps2;
            last_nominal_max_deceleration_mps2_ = optimization.maximum_deceleration_mps2;
            last_nominal_max_jerk_mps3_ = optimization.maximum_jerk_mps3;
            last_nominal_geometric_path_length_m_ =
                optimization.geometric_path_length_m;
            last_nominal_trajectory_length_m_ = optimization.trajectory_length_m;
            last_nominal_objective_cost_ = optimization.objective_cost;
            last_nominal_collision_check_failure_count_ =
                optimization.collision_check_failure_count;
            last_nominal_first_collision_position_ =
                optimization.first_collision_position;
          } else {
            last_safety_raw_path_node_count_ = optimization.raw_path_node_count;
            last_safety_corridor_checked_count_ =
                candidate.statistics.corridor.checked_sample_count;
            last_safety_corridor_blocked_count_ =
                candidate.statistics.corridor.blocked_sample_count;
            last_safety_collision_free_ = optimization.collision_free;
            last_safety_dynamic_limits_satisfied_ = optimization.dynamic_limits_satisfied;
            last_safety_duration_s_ = optimization.duration_s;
          }
        };
        recordCandidateTelemetry(nominal_result, true);
        recordCandidateTelemetry(safety_result, false);
        last_nominal_failure_code_ = nominal_result.failure_code;
        last_safety_failure_code_ = safety_result.failure_code;
        if (nominal_result.success && safety_result.success) {
          auto committed_nominal = nominal_result.trajectory;
          if (planner_config_.allow_nominal_unknown) {
            committed_nominal = committedPrefix(
                committed_nominal, planner_config_.nominal_commitment_horizon_s);
          }
          nominal_candidate.trajectory = committed_nominal;
          nominal_candidate.trajectory.duration_s = committed_nominal.duration_s;
          const auto dual = trajectory_verifier_.verifyDual(
              committed_nominal, safety_result.trajectory, state, world);
          last_nominal_verification_failure_ = dual.nominal.failure_code;
          last_safety_verification_failure_ = dual.safety.failure_code;
          last_verification_time_us_ =
              dual.nominal.statistics.verification_time_us +
              dual.safety.statistics.verification_time_us;
          if (!dual.success) {
            last_dual_verification_failure_ = dual.failure_code;
            ++dual_verification_failure_count_;
            ++verification_failure_count_;
            result = rejectCandidate(safety_result,
                                     mapVerificationFailure(dual.failure_code));
          } else if (dual.nominal_selected &&
                     (execute_nominal_unknown_ || !planner_config_.allow_nominal_unknown)) {
            result = nominal_result;
            result.trajectory = std::move(committed_nominal);
            result.role = planner_config_.allow_nominal_unknown
                              ? navigation_planning::PlanRole::Nominal
                              : navigation_planning::PlanRole::Committed;
            result.safety_kind = navigation_planning::SafetyPlanKind::None;
            ++nominal_selected_count_;
          } else {
            // The nominal branch is intentionally retained in the bundle and
            // diagnostics for A/B comparison, but an optimistic Unknown-space
            // path is not a legal control command by default. Selecting the
            // independently verified safety route prevents the dual-planning
            // experiment from driving through a narrow, partially observed
            // gap merely because the nominal verifier accepted its short
            // commitment prefix.
            result = safety_result;
            result.role = navigation_planning::PlanRole::Safety;
            if (dual.nominal_selected) ++safety_fallback_count_;
          }
        } else if (!nominal_result.success && safety_result.success) {
          result = safety_result;
          if (result.success) ++safety_fallback_count_;
        } else if (nominal_result.success) {
          // An optimistic candidate without a valid known-free stop is never
          // publishable, even if its own verifier would pass.
          ++dual_verification_failure_count_;
          // Preserve the selected-role contract: a rejected nominal must not
          // be serialized as SELECTED_NOMINAL with nominal_available=false.
          // Publish a safety-role failure instead so PX4 can fail closed and
          // hand over, rather than repeatedly rejecting an internally
          // contradictory bundle while holding a stale trajectory.
          result = rejectCandidate(safety_result, safety_result.failure_code);
          result.role = navigation_planning::PlanRole::Safety;
        } else {
          result = safety_result;
        }
      }

      // A failed trajectory is a state invalidation event. Preserve the
      // current map provenance so the PX4 adapter can process the failure
      // before its accepted-revision filter instead of keeping an old path.
        if (!result.success) {
          RCLCPP_WARN(
              get_logger(),
              "No verified trajectory candidate: cycle=%lu safety_goal=(%.3f,%.3f,%.3f) "
              "terminal=%s known_free=%s ray=%s route_failure=%s final_failure=%s",
              static_cast<unsigned long>(plan_count_), last_safety_goal_position_.x(),
              last_safety_goal_position_.y(), last_safety_goal_position_.z(),
              safety_goal.terminal ? "true" : "false",
              last_safety_goal_known_free_ ? "true" : "false",
              last_horizon_ray_occupied_ ? "true" : "false",
              failureName(last_safety_route_failure_code_), failureName(result.failure_code));
          result.world_generation = world.generation();
        result.world_revision = world.revision();
        // A failed dual-verification decision may still have a valid safety
        // candidate. Promote it instead of serializing a Safety selection with
        // safety_available=false. If both route candidates failed, create and
        // verify an explicit braking stop as the only legal fail-closed
        // publication. The PX4 adapter must receive either a valid safety
        // trajectory or no bundle at all; a contradictory legacy bundle makes
        // the adapter reject the handover on every planning tick.
        if (safety_candidate.success) {
          result = safety_candidate;
          result.role = navigation_planning::PlanRole::Safety;
        } else {
          auto safety_state = state;
          safety_state.acceleration = navigation_mapping::Vec3::Zero();
          auto stop_candidate = extendMovingSafetyStop(stabilizeStationarySafetyStop(
              validate(planner_.planSafetyStop(safety_state, world))));
          if (stop_candidate.success) {
            safety_candidate = stop_candidate;
            result = stop_candidate;
            result.role = navigation_planning::PlanRole::Safety;
          } else {
            // A failed bundle must never advertise a stale candidate as a
            // backup. The invalid single-trajectory publication below is
            // retained for watchdog/diagnostic consumers, but no legacy or V2
            // bundle is emitted without a valid safety branch.
            nominal_candidate.success = false;
            safety_candidate.success = false;
            nominal_candidate.trajectory.points.clear();
            nominal_candidate.trajectory.duration_s = 0.0;
            safety_candidate.trajectory.points.clear();
            safety_candidate.trajectory.duration_s = 0.0;
          }
        }
      }

      // The planner may use a short future splice to preserve continuity, but
      // an immediate message is executed against the live PX4 state. Keep the
      // External Mode guard as a final safety net and make the planner enforce
      // the same contract first. If a predicted splice is already too far from
      // measured odometry, publish a measured-state braking stop instead of
      // handing PX4 a trajectory that it must chase backwards/sideways.
      result = enforceImmediateTrajectoryStart(std::move(result));
      if (immediate_start_guard_recovery_selected) {
        safety_candidate = result;
        nominal_candidate.success = false;
        nominal_candidate.trajectory.points.clear();
        nominal_candidate.trajectory.duration_s = 0.0;
      }

      if (result.success && result.safety_kind == navigation_planning::SafetyPlanKind::Route) {
        ++safety_route_selected_count_;
      } else if (result.success &&
                 result.safety_kind == navigation_planning::SafetyPlanKind::BrakingStop) {
        ++safety_stop_selected_count_;
      }

      // A map revision can invalidate the nominal suffix while the old
      // prefix is still committed. A verified safety candidate is then the
      // earliest legal replacement; do not add another nominal switch delay.
      if (result.success && result.role == navigation_planning::PlanRole::Safety &&
          !world_unchanged && !braking_stop_latched) {
        plan_valid_from_delay_s_ = 0.0;
      }

      if (result.success && plan_valid_from_delay_s_ > 0.0 &&
          !result.trajectory.points.empty()) {
        // The first point is the state handed to Planner. These residuals are
        // the direct continuity KPI for a rolling replan; non-zero values
        // indicate a planner/serialization contract violation rather than a
        // vehicle tracking error.
        const auto& first = result.trajectory.points.front();
        last_splice_position_residual_m_ = (first.position - state.position).norm();
        last_splice_velocity_residual_mps_ = (first.velocity - state.velocity).norm();
        last_splice_acceleration_residual_mps2_ =
            (first.acceleration - state.acceleration).norm();
        last_splice_jerk_residual_mps3_ = first.jerk.norm();
        last_splice_snap_residual_mps4_ = first.snap.norm();
      } else {
        last_splice_position_residual_m_ = 0.0;
        last_splice_velocity_residual_mps_ = 0.0;
        last_splice_acceleration_residual_mps2_ = 0.0;
        last_splice_jerk_residual_mps3_ = 0.0;
        last_splice_snap_residual_mps4_ = 0.0;
      }

      // Only measured/splice-state progress is committed above.  A local
      // endpoint is deliberately several metres ahead of the vehicle; using
      // that planned lookahead as a hard progress watermark would reject a
      // valid safety detour after a map revision and latch a braking stop.
      last_mission_progress_m_ = committed_mission_progress_m_;

      last_plan_identity_valid_ = true;
      last_planned_mission_id_ = goal_message.mission_id;
      last_planned_waypoint_index_ = goal_message.waypoint_index;
      last_planned_request_id_ = goal_message.request_id;
      last_planned_world_generation_ = world.generation();
      last_planned_world_revision_ = world.revision();
      if (result.success) {
        last_plan_time_ns_ = get_clock()->now().nanoseconds() +
                             static_cast<std::int64_t>(plan_valid_from_delay_s_ * 1e9);
        last_planned_duration_s_ = result.trajectory.duration_s;
        last_planned_trajectory_ = result.trajectory;
        last_planned_role_ = result.role;
        last_planned_safety_kind_ = result.safety_kind;
        // A braking stop is a committed recovery action for this goal. For a
        // pass-through waypoint it is released only by a newer map revision;
        // this prevents an endless stop/replan loop while preserving a path
        // to recovery when mapping reveals a new corridor.
        braking_stop_latched_ =
            result.role == navigation_planning::PlanRole::Safety &&
            result.safety_kind == navigation_planning::SafetyPlanKind::BrakingStop;
        if (braking_stop_latched_) {
          // A newly selected stop starts a fresh route-retry cooldown. Without
          // resetting this anchor, a failed retry could immediately produce
          // another route attempt on the next map tick.
          last_safety_retry_time_ns_ = 0;
        }
      } else {
        plan_valid_from_delay_s_ = 0.0;
        last_planned_trajectory_.reset();
        last_planned_duration_s_ = 0.0;
        last_planned_safety_kind_ = navigation_planning::SafetyPlanKind::None;
      }
    }
  }

  ++plan_count_;
  if (result.success) {
    ++plan_success_count_;
  } else {
    ++plan_failure_count_;
  }
  last_failure_code_ = result.failure_code;
  const bool had_valid_plan_before_cycle = last_plan_success_;
  last_plan_success_ = result.success;
  // Do not send a contradictory bundle during startup when the map has not
  // produced a valid KnownFree safety candidate yet.  The PX4 adapter is
  // intentionally strict and must reject such a bundle, but publishing it
  // repeatedly makes a healthy startup look like a controller failure.  Keep
  // the mode waiting for the next planning tick; once a valid bundle has been
  // accepted, a later safety failure is still published and fails closed.
  const bool defer_initial_invalid_bundle = !result.success &&
                                            !had_valid_plan_before_cycle &&
                                            !safety_candidate.success;
  if (defer_initial_invalid_bundle) {
    publishPlanningPaths(result, nominal_candidate, safety_candidate, goal_message.header,
                         full_nominal_visualization_trajectory);
    publishNavigationVisualization(result, goal_message);
    publishPlanningDiagnostics(result);
    return;
  }
  trajectory_publisher_->publish(makeTrajectoryMessage(result, goal_message));
  if (result.success) {
    const auto now_stamp = get_clock()->now();
    const auto valid_from = rosTimeFromNanoseconds(
        now_stamp.nanoseconds() + static_cast<std::int64_t>(plan_valid_from_delay_s_ * 1e9));
    trajectory_bundle_publisher_->publish(makeTrajectoryBundleMessage(
        result, nominal_candidate, safety_candidate, goal_message, valid_from, next_bundle_id_,
        last_bundle_id_));
    if (safety_candidate.success) {
      trajectory_bundle_v2_publisher_->publish(makeTrajectoryBundleV2Message(
          result, nominal_candidate, safety_candidate, goal_message, valid_from,
          next_bundle_id_, last_bundle_id_));
    }
    last_bundle_id_ = next_bundle_id_++;
  } else {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No verified trajectory candidate for mission=%s waypoint=%u; suppressing invalid bundle",
        goal_message.mission_id.c_str(), goal_message.waypoint_index);
  }
  publishPlanningPaths(result, nominal_candidate, safety_candidate, goal_message.header,
                       full_nominal_visualization_trajectory);
  publishNavigationVisualization(result, goal_message);
  publishPlanningDiagnostics(result);
}

navigation_interfaces::msg::PlannedTrajectory NavigationRuntimeNode::makeTrajectoryMessage(
    const navigation_planning::PlanResult& result,
    const navigation_interfaces::msg::NavigationGoal& goal) {
  navigation_interfaces::msg::PlannedTrajectory message;
  // The trajectory timestamp is the execution time origin. The adapter uses
  // it to account for transport latency rather than inheriting phase from a
  // previous rolling trajectory.
  message.header = goal.header;
  message.header.stamp = get_clock()->now();
  message.valid_from = rosTimeFromNanoseconds(
      message.header.stamp.sec * 1'000'000'000LL + message.header.stamp.nanosec +
      static_cast<std::int64_t>(plan_valid_from_delay_s_ * 1e9));
  message.trajectory_id = next_trajectory_id_++;
  message.parent_trajectory_id = last_trajectory_id_;
  message.commitment_horizon_s = planner_config_.nominal_commitment_horizon_s;
  last_trajectory_id_ = message.trajectory_id;
  message.mission_id = goal.mission_id;
  message.waypoint_index = goal.waypoint_index;
  message.request_id = goal.request_id;
  message.success = result.success;
  message.failure_code = static_cast<std::uint8_t>(result.failure_code);
  message.trajectory_role = static_cast<std::uint8_t>(result.role);
  message.safety_plan_kind = static_cast<std::uint8_t>(result.safety_kind);
  message.world_generation = result.world_generation;
  message.world_revision = result.world_revision;
  message.duration_s = result.trajectory.duration_s;
  for (const auto& point : result.trajectory.points) {
    message.time_from_start.push_back(point.time_from_start_s);
    geometry_msgs::msg::Point position;
    position.x = point.position.x();
    position.y = point.position.y();
    position.z = point.position.z();
    message.position.push_back(position);
    geometry_msgs::msg::Vector3 velocity;
    velocity.x = point.velocity.x();
    velocity.y = point.velocity.y();
    velocity.z = point.velocity.z();
    message.velocity.push_back(velocity);
    geometry_msgs::msg::Vector3 acceleration;
    acceleration.x = point.acceleration.x();
    acceleration.y = point.acceleration.y();
    acceleration.z = point.acceleration.z();
    message.acceleration.push_back(acceleration);
  }
  return message;
}

navigation_interfaces::msg::TrajectoryCandidate
NavigationRuntimeNode::makeTrajectoryCandidateMessage(
    const navigation_planning::PlanResult& result,
    const navigation_interfaces::msg::NavigationGoal& goal,
    const builtin_interfaces::msg::Time& valid_from,
    std::uint64_t trajectory_id,
    std::uint64_t parent_trajectory_id) const {
  navigation_interfaces::msg::TrajectoryCandidate message;
  message.header = goal.header;
  message.header.stamp = get_clock()->now();
  message.valid_from = valid_from;
  message.trajectory_id = trajectory_id;
  message.parent_trajectory_id = parent_trajectory_id;
  message.commitment_horizon_s = planner_config_.nominal_commitment_horizon_s;
  message.mission_id = goal.mission_id;
  message.waypoint_index = goal.waypoint_index;
  message.request_id = goal.request_id;
  message.success = result.success;
  message.failure_code = static_cast<std::uint8_t>(result.failure_code);
  message.trajectory_role = result.role == navigation_planning::PlanRole::Safety
                                ? navigation_interfaces::msg::TrajectoryCandidate::ROLE_SAFETY
                                : navigation_interfaces::msg::TrajectoryCandidate::ROLE_NOMINAL;
  message.safety_plan_kind = result.role == navigation_planning::PlanRole::Safety
                                 ? static_cast<std::uint8_t>(result.safety_kind)
                                 : navigation_interfaces::msg::TrajectoryCandidate::SAFETY_KIND_NONE;
  message.world_generation = result.world_generation;
  message.world_revision = result.world_revision;
  message.known_free_only = result.role == navigation_planning::PlanRole::Safety;
  message.current_pose_unknown_overlay =
      message.known_free_only && planner_config_.allow_unknown_start;
  message.duration_s = result.trajectory.duration_s;
  message.time_from_start.reserve(result.trajectory.points.size());
  message.position.reserve(result.trajectory.points.size());
  message.velocity.reserve(result.trajectory.points.size());
  message.acceleration.reserve(result.trajectory.points.size());
  for (const auto& point : result.trajectory.points) {
    message.time_from_start.push_back(point.time_from_start_s);
    geometry_msgs::msg::Point position;
    position.x = point.position.x();
    position.y = point.position.y();
    position.z = point.position.z();
    message.position.push_back(position);
    geometry_msgs::msg::Vector3 velocity;
    velocity.x = point.velocity.x();
    velocity.y = point.velocity.y();
    velocity.z = point.velocity.z();
    message.velocity.push_back(velocity);
    geometry_msgs::msg::Vector3 acceleration;
    acceleration.x = point.acceleration.x();
    acceleration.y = point.acceleration.y();
    acceleration.z = point.acceleration.z();
    message.acceleration.push_back(acceleration);
  }
  return message;
}

navigation_interfaces::msg::PlannedTrajectoryBundle
NavigationRuntimeNode::makeTrajectoryBundleMessage(
    const navigation_planning::PlanResult& selected,
    const navigation_planning::PlanResult& nominal,
    const navigation_planning::PlanResult& safety,
    const navigation_interfaces::msg::NavigationGoal& goal,
    const builtin_interfaces::msg::Time& valid_from,
    std::uint64_t bundle_id,
    std::uint64_t parent_bundle_id) const {
  navigation_interfaces::msg::PlannedTrajectoryBundle message;
  message.header = goal.header;
  message.header.stamp = get_clock()->now();
  message.valid_from = valid_from;
  message.bundle_id = bundle_id;
  message.parent_bundle_id = parent_bundle_id;
  message.mission_id = goal.mission_id;
  message.waypoint_index = goal.waypoint_index;
  message.request_id = goal.request_id;
  message.selected_candidate = selected.role == navigation_planning::PlanRole::Safety
                                   ? navigation_interfaces::msg::PlannedTrajectoryBundle::SELECTED_SAFETY
                                   : navigation_interfaces::msg::PlannedTrajectoryBundle::SELECTED_NOMINAL;
  message.world_generation = selected.world_generation;
  message.world_revision = selected.world_revision;
  // The nominal branch is publishable only when the same planning cycle also
  // produced a valid KnownFree safety candidate. A nominal-only message is
  // therefore never accepted by the PX4 adapter.
  message.nominal_available = nominal.success && safety.success && selected.success;
  message.safety_available = safety.success;
  message.nominal = makeTrajectoryCandidateMessage(
      nominal, goal, valid_from, bundle_id * 2U, parent_bundle_id * 2U);
  message.safety = makeTrajectoryCandidateMessage(
      safety, goal, valid_from, bundle_id * 2U + 1U, parent_bundle_id * 2U + 1U);
  if (!message.nominal_available) message.nominal.success = false;
  return message;
}

navigation_interfaces::msg::TrajectoryBundle
NavigationRuntimeNode::makeTrajectoryBundleV2Message(
    const navigation_planning::PlanResult& selected,
    const navigation_planning::PlanResult& nominal,
    const navigation_planning::PlanResult& safety,
    const navigation_interfaces::msg::NavigationGoal& goal,
    const builtin_interfaces::msg::Time& valid_from,
    std::uint64_t bundle_id,
    std::uint64_t parent_bundle_id) const {
  navigation_interfaces::msg::TrajectoryBundle message;
  message.header = goal.header;
  message.header.stamp = get_clock()->now();
  message.valid_from = valid_from;
  // The migration publisher currently sends complete candidate suffixes. The
  // next splice-aware planner will populate common_prefix and move branch_time
  // after the preserved prefix; leaving it empty here is explicit rather than
  // fabricating continuity from two independently optimized candidates.
  message.branch_time = valid_from;
  message.decision_deadline = valid_from;
  message.bundle_id = bundle_id;
  message.parent_bundle_id = parent_bundle_id;
  message.mission_id = goal.mission_id;
  message.waypoint_index = goal.waypoint_index;
  message.request_id = goal.request_id;
  message.world_generation = selected.world_generation;
  message.world_revision = selected.world_revision;
  message.route_id = bundle_id;
  message.nominal_valid = nominal.success && safety.success;
  if (nominal.success) message.nominal_suffix = makeTrajectorySegment(nominal.trajectory);
  if (safety.success) message.safety_suffix = makeTrajectorySegment(safety.trajectory);
  message.safety_kind = safety.safety_kind == navigation_planning::SafetyPlanKind::BrakingStop
                            ? navigation_interfaces::msg::TrajectoryBundle::SAFETY_STOP
                            : navigation_interfaces::msg::TrajectoryBundle::SAFETY_ROUTE;
  message.selected_branch = selected.role == navigation_planning::PlanRole::Safety
                                ? navigation_interfaces::msg::TrajectoryBundle::BRANCH_SAFETY
                                : navigation_interfaces::msg::TrajectoryBundle::BRANCH_NOMINAL;
  message.requested_branch = message.selected_branch;
  return message;
}

nav_msgs::msg::Path NavigationRuntimeNode::makePathMessage(
    const navigation_planning::PlanResult& result, const std_msgs::msg::Header& header) const {
  nav_msgs::msg::Path path;
  path.header = header;
  path.header.frame_id = planning_frame_id_;
  path.header.stamp = get_clock()->now();
  path.poses.reserve(result.trajectory.points.size());
  for (const auto& point : result.trajectory.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = point.position.x();
    pose.pose.position.y = point.position.y();
    pose.pose.position.z = point.position.z();
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

void NavigationRuntimeNode::publishPlanningPaths(
    const navigation_planning::PlanResult& selected,
    const navigation_planning::PlanResult& nominal,
    const navigation_planning::PlanResult& safety,
    const std_msgs::msg::Header& header,
    const std::optional<navigation_planning::TimeParameterizedTrajectory>&
        full_nominal_trajectory) {
  const auto makeEmptyPath = [&]() {
    nav_msgs::msg::Path path;
    path.header = header;
    path.header.frame_id = planning_frame_id_;
    path.header.stamp = get_clock()->now();
    return path;
  };
  const auto makeCandidatePath = [&](const navigation_planning::PlanResult& candidate,
                                     const std::optional<
                                         navigation_planning::TimeParameterizedTrajectory>&
                                         full_trajectory) {
    if (!candidate.success) return makeEmptyPath();
    if (full_trajectory.has_value()) {
      auto full_candidate = candidate;
      full_candidate.trajectory = *full_trajectory;
      return makePathMessage(full_candidate, header);
    }
    return makePathMessage(candidate, header);
  };

  // The nominal candidate is the only trajectory currently retained in both
  // full and PX4 commitment-prefix forms. Safety and selected use their full
  // runtime trajectory when valid; invalid candidates are deliberately
  // represented by an empty Path instead of stale visualization data.
  const auto nominal_path = makeCandidatePath(nominal, full_nominal_trajectory);
  const auto safety_path = makeCandidatePath(safety, std::nullopt);
  const auto selected_full_trajectory =
      selected.role == navigation_planning::PlanRole::Safety
          ? std::optional<navigation_planning::TimeParameterizedTrajectory>{}
          : full_nominal_trajectory;
  const auto selected_path = makeCandidatePath(selected, selected_full_trajectory);

  planned_path_nominal_publisher_->publish(nominal_path);
  planned_path_safety_publisher_->publish(safety_path);
  planned_path_selected_publisher_->publish(selected_path);
  // Keep the migration topic and its existing selected/full-nominal behavior.
  planned_path_publisher_->publish(selected_path);
}

void NavigationRuntimeNode::publishNavigationVisualization(
    const navigation_planning::PlanResult& result,
    const navigation_interfaces::msg::NavigationGoal& goal) {
  if (!navigation_marker_publisher_) return;
  visualization_msgs::msg::MarkerArray array;
  const auto stamp = get_clock()->now();
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = planning_frame_id_;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  const auto addPoint = [&](int id, const navigation_mapping::Vec3& point,
                            float r, float g, float b, const std::string& ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.95F;
    array.markers.push_back(marker);
  };
  addPoint(1, {goal.target.x, goal.target.y, goal.target.z}, 1.0F, 0.1F, 0.1F,
            "mission_goal");
  addPoint(2, last_effective_goal_, 1.0F, 0.75F, 0.05F, "effective_local_goal");
  if (goal.has_next_target) {
    addPoint(5, {goal.next_target.x, goal.next_target.y, goal.next_target.z}, 0.1F, 0.45F,
             1.0F, "next_mission_goal");
  }

  visualization_msgs::msg::Marker status;
  status.header.frame_id = planning_frame_id_;
  status.header.stamp = stamp;
  status.ns = "planner_status";
  status.id = 3;
  status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  status.action = visualization_msgs::msg::Marker::ADD;
  status.pose.position.x = goal.target.x;
  status.pose.position.y = goal.target.y;
  status.pose.position.z = goal.target.z + 0.6;
  status.pose.orientation.w = 1.0;
  status.scale.z = 0.28;
  status.color.r = 1.0F;
  status.color.g = 1.0F;
  status.color.b = 1.0F;
  status.color.a = 1.0F;
  status.text = result.success ?
      std::string("goal ") + std::to_string(goal.waypoint_index) +
          (goal.behavior == navigation_interfaces::msg::NavigationGoal::BEHAVIOR_STOP
               ? " STOP"
               : " PASS") + " / " + roleName(result.role) :
      std::string("PLAN FAILED / ") + failureName(result.failure_code);
  array.markers.push_back(status);
  navigation_marker_publisher_->publish(array);
}

void NavigationRuntimeNode::publishPlanningDiagnostics(
    const navigation_planning::PlanResult& result) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_planning/planner";
  status.hardware_id = "navigation_runtime";
  status.level = result.success ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message = result.success ? "trajectory generated" : failureName(result.failure_code);
  const auto& statistics = result.statistics;
  const auto cellStateName = [](const navigation_mapping::CellState state) {
    switch (state) {
      case navigation_mapping::CellState::KnownFree:
        return "KNOWN_FREE";
      case navigation_mapping::CellState::Occupied:
        return "OCCUPIED";
      case navigation_mapping::CellState::Unknown:
      default:
        return "UNKNOWN";
    }
  };
  const auto& trajectory_points = result.trajectory.points;
  const auto trajectoryValue = [&](double value) {
    return std::to_string(std::isfinite(value) ? value : 0.0);
  };
  const auto trajectoryStart = [&]() -> navigation_planning::TrajectoryPoint {
    return trajectory_points.empty() ? navigation_planning::TrajectoryPoint{}
                                      : trajectory_points.front();
  };
  const auto trajectoryEnd = [&]() -> navigation_planning::TrajectoryPoint {
    return trajectory_points.empty() ? navigation_planning::TrajectoryPoint{}
                                      : trajectory_points.back();
  };
  const auto start_point = trajectoryStart();
  const auto end_point = trajectoryEnd();
  status.values = {
      keyValue("plan_count", std::to_string(plan_count_)),
      keyValue("plan_skip_count", std::to_string(plan_skip_count_)),
      keyValue("success_count", std::to_string(plan_success_count_)),
      keyValue("safety_fallback_count", std::to_string(safety_fallback_count_)),
      keyValue("safety_route_plan_count", std::to_string(safety_route_plan_count_)),
      keyValue("safety_route_verified_count", std::to_string(safety_route_verified_count_)),
      keyValue("safety_route_selected_count", std::to_string(safety_route_selected_count_)),
      keyValue("safety_stop_selected_count", std::to_string(safety_stop_selected_count_)),
      keyValue("safety_stop_retry_count", std::to_string(safety_stop_retry_count_)),
      keyValue("nominal_plan_count", std::to_string(nominal_plan_count_)),
      keyValue("nominal_selected_count", std::to_string(nominal_selected_count_)),
      keyValue("trajectory_bundle_id", std::to_string(last_bundle_id_)),
      keyValue("trajectory_bundle_parent_id", std::to_string(last_bundle_id_ > 0U
                                                                  ? last_bundle_id_ - 1U
                                                                  : 0U)),
      keyValue("dual_verification_failure_count",
               std::to_string(dual_verification_failure_count_)),
      keyValue("verification_failure_count", std::to_string(verification_failure_count_)),
      keyValue("local_subgoal_enabled", local_subgoal_enabled_ ? "true" : "false"),
      keyValue("fail_closed_on_unknown_mission_goal",
               fail_closed_on_unknown_mission_goal_ ? "true" : "false"),
      keyValue("local_subgoal_selected_count", std::to_string(local_subgoal_selected_count_)),
      keyValue("local_subgoal_failure_count", std::to_string(local_subgoal_failure_count_)),
      keyValue("horizon_endpoint_change_count", std::to_string(horizon_endpoint_change_count_)),
      keyValue("horizon_endpoint_repeat_count", std::to_string(horizon_endpoint_repeat_count_)),
      keyValue("horizon_backward_rejection_count",
               std::to_string(horizon_backward_rejection_count_)),
      keyValue("mission_progress_rejection_count",
               std::to_string(mission_progress_rejection_count_)),
      keyValue("trajectory_progress_rejection_count",
               std::to_string(trajectory_progress_rejection_count_)),
      keyValue("trajectory_revalidation_count", std::to_string(trajectory_revalidation_count_)),
      keyValue("trajectory_revalidation_failure_count",
               std::to_string(trajectory_revalidation_failure_count_)),
      keyValue("trajectory_reuse_count", std::to_string(trajectory_reuse_count_)),
      keyValue("full_replan_count", std::to_string(full_replan_count_)),
      keyValue("trajectory_start_guard_rejection_count",
               std::to_string(trajectory_start_guard_rejection_count_)),
      keyValue("trajectory_start_position_error_m",
               std::to_string(trajectory_start_position_error_m_)),
      keyValue("trajectory_start_position_residual_m",
               std::to_string(last_trajectory_start_position_residual_m_)),
      keyValue("replan_reason", last_replan_reason_),
      keyValue("local_subgoal_selected", last_local_subgoal_selected_ ? "true" : "false"),
      keyValue("goal_terminal", last_goal_terminal_ ? "true" : "false"),
      keyValue("terminal_velocity_x", std::to_string(last_terminal_velocity_.x())),
      keyValue("terminal_velocity_y", std::to_string(last_terminal_velocity_.y())),
      keyValue("terminal_velocity_z", std::to_string(last_terminal_velocity_.z())),
      keyValue("continuation_speed_fraction",
               std::to_string(local_goal_continuation_speed_fraction_)),
      keyValue("local_goal_status", localGoalStatusName(last_local_goal_status_)),
      keyValue("effective_goal_x", std::to_string(last_effective_goal_.x())),
      keyValue("effective_goal_y", std::to_string(last_effective_goal_.y())),
      keyValue("effective_goal_z", std::to_string(last_effective_goal_.z())),
      keyValue("safety_goal_x", std::to_string(last_safety_goal_position_.x())),
      keyValue("safety_goal_y", std::to_string(last_safety_goal_position_.y())),
      keyValue("safety_goal_z", std::to_string(last_safety_goal_position_.z())),
      keyValue("safety_goal_terminal", last_horizon_ray_occupied_ ? "true" : "false"),
      keyValue("safety_goal_known_free", last_safety_goal_known_free_ ? "true" : "false"),
      keyValue("safety_route_failure", failureName(last_safety_route_failure_code_)),
      keyValue("safety_failure", failureName(last_safety_failure_code_)),
      keyValue("horizon_tangent_x", std::to_string(last_horizon_tangent_.x())),
      keyValue("horizon_tangent_y", std::to_string(last_horizon_tangent_.y())),
      keyValue("horizon_tangent_z", std::to_string(last_horizon_tangent_.z())),
      keyValue("planning_state_x", std::to_string(last_planning_state_position_.x())),
      keyValue("planning_state_y", std::to_string(last_planning_state_position_.y())),
      keyValue("planning_state_z", std::to_string(last_planning_state_position_.z())),
      keyValue("planning_velocity_x", std::to_string(last_planning_state_velocity_.x())),
      keyValue("planning_velocity_y", std::to_string(last_planning_state_velocity_.y())),
      keyValue("planning_velocity_z", std::to_string(last_planning_state_velocity_.z())),
      keyValue("planning_state_age_s", std::to_string(last_planning_state_age_s_)),
      keyValue("horizon_forward_projection_m",
               std::to_string(last_horizon_forward_projection_m_)),
      keyValue("planning_horizon_distance_m",
               std::to_string(last_planning_horizon_distance_m_)),
      keyValue("horizon_progress_m", std::to_string(last_horizon_progress_m_)),
      keyValue("mission_progress_m", std::to_string(last_mission_progress_m_)),
      keyValue("horizon_ray_occupied", last_horizon_ray_occupied_ ? "true" : "false"),
      keyValue("detour_obstacle_anchor_active",
               last_detour_obstacle_anchor_.has_value() ? "true" : "false"),
      keyValue("detour_obstacle_anchor_x",
               last_detour_obstacle_anchor_.has_value()
                   ? std::to_string(last_detour_obstacle_anchor_->x())
                   : "0.0"),
      keyValue("detour_obstacle_anchor_y",
               last_detour_obstacle_anchor_.has_value()
                   ? std::to_string(last_detour_obstacle_anchor_->y())
                   : "0.0"),
      keyValue("detour_obstacle_anchor_z",
               last_detour_obstacle_anchor_.has_value()
                   ? std::to_string(last_detour_obstacle_anchor_->z())
                   : "0.0"),
      keyValue("detour_lateral_direction_x",
               last_detour_lateral_direction_.has_value()
                   ? std::to_string(last_detour_lateral_direction_->x())
                   : "0.0"),
      keyValue("detour_lateral_direction_y",
               last_detour_lateral_direction_.has_value()
                   ? std::to_string(last_detour_lateral_direction_->y())
                   : "0.0"),
      keyValue("detour_lateral_direction_z",
               last_detour_lateral_direction_.has_value()
                   ? std::to_string(last_detour_lateral_direction_->z())
                   : "0.0"),
      keyValue("adaptive_velocity_cap_mps", std::to_string(last_adaptive_velocity_cap_mps_)),
      keyValue("known_free_horizon_m", std::to_string(last_known_free_horizon_m_)),
      keyValue("splice_position_residual_m", std::to_string(last_splice_position_residual_m_)),
      keyValue("splice_velocity_residual_mps",
               std::to_string(last_splice_velocity_residual_mps_)),
      keyValue("splice_acceleration_residual_mps2",
               std::to_string(last_splice_acceleration_residual_mps2_)),
      keyValue("splice_jerk_residual_mps3",
               std::to_string(last_splice_jerk_residual_mps3_)),
      keyValue("splice_snap_residual_mps4",
               std::to_string(last_splice_snap_residual_mps4_)),
      keyValue("trajectory_start_x", trajectoryValue(start_point.position.x())),
      keyValue("trajectory_start_y", trajectoryValue(start_point.position.y())),
      keyValue("trajectory_start_z", trajectoryValue(start_point.position.z())),
      keyValue("trajectory_end_x", trajectoryValue(end_point.position.x())),
      keyValue("trajectory_end_y", trajectoryValue(end_point.position.y())),
      keyValue("trajectory_end_z", trajectoryValue(end_point.position.z())),
      keyValue("trajectory_initial_velocity_x", trajectoryValue(start_point.velocity.x())),
      keyValue("trajectory_initial_velocity_y", trajectoryValue(start_point.velocity.y())),
      keyValue("trajectory_initial_velocity_z", trajectoryValue(start_point.velocity.z())),
      keyValue("failure_count", std::to_string(plan_failure_count_)),
      keyValue("plan_role", roleName(result.role)),
      keyValue("safety_plan_kind", safetyKindName(result.safety_kind)),
      keyValue("last_failure_code", failureName(last_failure_code_)),
      keyValue("last_nominal_failure_code", failureName(last_nominal_failure_code_)),
      keyValue("last_safety_failure_code", failureName(last_safety_failure_code_)),
      keyValue("last_safety_route_failure_code",
               failureName(last_safety_route_failure_code_)),
      keyValue("safety_goal_x", std::to_string(last_safety_goal_position_.x())),
      keyValue("safety_goal_y", std::to_string(last_safety_goal_position_.y())),
      keyValue("safety_goal_z", std::to_string(last_safety_goal_position_.z())),
      keyValue("safety_goal_known_free", last_safety_goal_known_free_ ? "true" : "false"),
      keyValue("nominal_raw_path_node_count",
               std::to_string(last_nominal_raw_path_node_count_)),
      keyValue("nominal_corridor_checked_count",
               std::to_string(last_nominal_corridor_checked_count_)),
      keyValue("nominal_corridor_blocked_count",
               std::to_string(last_nominal_corridor_blocked_count_)),
      keyValue("nominal_collision_free", last_nominal_collision_free_ ? "true" : "false"),
      keyValue("nominal_dynamic_limits_satisfied",
               last_nominal_dynamic_limits_satisfied_ ? "true" : "false"),
      keyValue("nominal_duration_s", std::to_string(last_nominal_duration_s_)),
      keyValue("nominal_max_velocity_mps",
               std::to_string(last_nominal_max_velocity_mps_)),
      keyValue("nominal_max_acceleration_mps2",
               std::to_string(last_nominal_max_acceleration_mps2_)),
      keyValue("nominal_max_deceleration_mps2",
               std::to_string(last_nominal_max_deceleration_mps2_)),
      keyValue("nominal_max_jerk_mps3",
               std::to_string(last_nominal_max_jerk_mps3_)),
      keyValue("nominal_geometric_path_length_m",
               std::to_string(last_nominal_geometric_path_length_m_)),
      keyValue("nominal_trajectory_length_m",
               std::to_string(last_nominal_trajectory_length_m_)),
      keyValue("nominal_objective_cost",
               std::to_string(last_nominal_objective_cost_)),
      keyValue("nominal_collision_check_failure_count",
               std::to_string(last_nominal_collision_check_failure_count_)),
      keyValue("nominal_first_collision_x",
               std::to_string(last_nominal_first_collision_position_.x())),
      keyValue("nominal_first_collision_y",
               std::to_string(last_nominal_first_collision_position_.y())),
      keyValue("nominal_first_collision_z",
               std::to_string(last_nominal_first_collision_position_.z())),
      keyValue("safety_raw_path_node_count",
               std::to_string(last_safety_raw_path_node_count_)),
      keyValue("safety_corridor_checked_count",
               std::to_string(last_safety_corridor_checked_count_)),
      keyValue("safety_corridor_blocked_count",
               std::to_string(last_safety_corridor_blocked_count_)),
      keyValue("safety_collision_free", last_safety_collision_free_ ? "true" : "false"),
      keyValue("safety_dynamic_limits_satisfied",
               last_safety_dynamic_limits_satisfied_ ? "true" : "false"),
      keyValue("safety_duration_s", std::to_string(last_safety_duration_s_)),
      keyValue("last_nominal_verification_failure",
               verificationFailureName(last_nominal_verification_failure_)),
      keyValue("last_safety_verification_failure",
               verificationFailureName(last_safety_verification_failure_)),
      keyValue("last_dual_verification_failure",
               verificationFailureName(last_dual_verification_failure_)),
      keyValue("world_generation", std::to_string(result.world_generation)),
      keyValue("world_revision", std::to_string(result.world_revision)),
      keyValue("planning_path_search_us", std::to_string(statistics.search.search_time_us)),
      keyValue("planning_corridor_us", std::to_string(statistics.corridor.corridor_time_us)),
      keyValue("planning_trajectory_optimization_us",
               std::to_string(statistics.trajectory_optimization.optimization_time_us)),
      keyValue("planning_total_us", std::to_string(statistics.total_planning_time_us)),
      keyValue("trajectory_verification_us", std::to_string(last_verification_time_us_)),
      keyValue("maximum_velocity_mps",
               std::to_string(statistics.trajectory_optimization.maximum_velocity_mps)),
      keyValue("maximum_acceleration_mps2",
               std::to_string(statistics.trajectory_optimization.maximum_acceleration_mps2)),
      keyValue("maximum_deceleration_mps2",
               std::to_string(statistics.trajectory_optimization.maximum_deceleration_mps2)),
      keyValue("maximum_jerk_mps3",
               std::to_string(statistics.trajectory_optimization.maximum_jerk_mps3)),
      keyValue("integrated_squared_jerk",
               std::to_string(statistics.trajectory_optimization.integrated_squared_jerk)),
      keyValue("integrated_squared_snap",
               std::to_string(statistics.trajectory_optimization.integrated_squared_snap)),
      keyValue("trajectory_objective_cost",
               std::to_string(statistics.trajectory_optimization.objective_cost)),
      keyValue("c2_continuity_residual",
               std::to_string(statistics.trajectory_optimization.c2_continuity_residual)),
      keyValue("geometric_path_length_m",
               std::to_string(statistics.trajectory_optimization.geometric_path_length_m)),
      keyValue("trajectory_length_m",
               std::to_string(statistics.trajectory_optimization.trajectory_length_m)),
      keyValue("duration_s",
               std::to_string(statistics.trajectory_optimization.duration_s)),
      keyValue("trajectory_duration_s",
               std::to_string(statistics.trajectory_optimization.duration_s)),
      keyValue("kinematic_lower_bound_s",
               std::to_string(statistics.trajectory_optimization.geometric_path_length_m /
                              std::max(1e-9, planner_config_.limits.max_velocity_mps))),
      keyValue("minimum_clearance_m",
               std::to_string(statistics.corridor.minimum_clearance_radius_m)),
      keyValue("raw_path_node_count",
               std::to_string(statistics.trajectory_optimization.raw_path_node_count)),
      keyValue("simplified_path_node_count",
               std::to_string(statistics.trajectory_optimization.simplified_path_node_count)),
      keyValue("shortcut_count",
               std::to_string(statistics.trajectory_optimization.shortcut_count)),
      keyValue("trajectory_collision_free",
               statistics.trajectory_optimization.collision_free ? "true" : "false"),
      keyValue("trajectory_dynamic_limits_satisfied",
               statistics.trajectory_optimization.dynamic_limits_satisfied ? "true" : "false"),
      keyValue("expanded_search_nodes", std::to_string(statistics.search.expanded_nodes)),
      keyValue("planning_start_cell_state",
               cellStateName(statistics.search.start_cell_state)),
      keyValue("planning_goal_cell_state",
               cellStateName(statistics.search.goal_cell_state)),
      keyValue("corridor_checked_samples",
               std::to_string(statistics.corridor.checked_sample_count)),
      keyValue("trajectory_samples",
               std::to_string(statistics.trajectory_optimization.sampled_point_count)),
      keyValue("trajectory_duration_s",
               std::to_string(statistics.trajectory_optimization.duration_s)),
  };
  array.status.push_back(status);
  planning_diagnostics_publisher_->publish(array);

  navigation_interfaces::msg::PlannerCycleTrace trace;
  trace.header.stamp = get_clock()->now();
  trace.header.frame_id = planning_frame_id_;
  trace.cycle_id = plan_count_;
  trace.bundle_id = result.success ? last_bundle_id_ : 0U;
  trace.request_id = last_planned_request_id_;
  trace.world_generation = result.world_generation;
  trace.world_revision = result.world_revision;
  // Route geometry is not exposed by the legacy A* result yet. Keep these
  // fields zero until RouteManager supplies a real route id/arc projection;
  // the report marks the resulting trace as partial instead of inventing it.
  trace.route_id = 0U;
  trace.route_candidate_count = 0U;
  trace.corridor_region_count = static_cast<std::uint32_t>(
      std::min<std::uint64_t>(statistics.corridor.segment_count,
                              std::numeric_limits<std::uint32_t>::max()));
  trace.horizon_start_arc_m = 0.0;
  trace.horizon_end_arc_m = 0.0;
  trace.horizon_endpoint.x = end_point.position.x();
  trace.horizon_endpoint.y = end_point.position.y();
  trace.horizon_endpoint.z = end_point.position.z();
  trace.planning_state_position.x = last_planning_state_position_.x();
  trace.planning_state_position.y = last_planning_state_position_.y();
  trace.planning_state_position.z = last_planning_state_position_.z();
  trace.planning_state_velocity.x = last_planning_state_velocity_.x();
  trace.planning_state_velocity.y = last_planning_state_velocity_.y();
  trace.planning_state_velocity.z = last_planning_state_velocity_.z();
  trace.planning_state_age_s = last_planning_state_age_s_;
  trace.planning_horizon_distance_m = last_planning_horizon_distance_m_;
  trace.horizon_forward_projection_m = last_horizon_forward_projection_m_;
  trace.horizon_progress_m = last_horizon_progress_m_;
  trace.known_free_horizon_m = last_known_free_horizon_m_;
  trace.horizon_ray_occupied = last_horizon_ray_occupied_;
  trace.planning_latency_ms = static_cast<double>(statistics.total_planning_time_us) / 1000.0;
  trace.optimizer_latency_ms = static_cast<double>(
      statistics.trajectory_optimization.optimization_time_us) / 1000.0;
  trace.splice_position_residual_m = last_splice_position_residual_m_;
  trace.splice_velocity_residual_mps = last_splice_velocity_residual_mps_;
  trace.splice_acceleration_residual_mps2 = last_splice_acceleration_residual_mps2_;
  trace.maximum_velocity_mps = statistics.trajectory_optimization.maximum_velocity_mps;
  trace.maximum_acceleration_mps2 = statistics.trajectory_optimization.maximum_acceleration_mps2;
  trace.maximum_jerk_mps3 = statistics.trajectory_optimization.maximum_jerk_mps3;
  trace.time_cost = statistics.trajectory_optimization.duration_s;
  trace.snap_cost = statistics.trajectory_optimization.integrated_squared_snap;
  trace.clearance_cost = statistics.corridor.minimum_clearance_radius_m;
  trace.unknown_cost = 0.0;
  trace.previous_trajectory_cost = 0.0;
  trace.selected_branch = result.role == navigation_planning::PlanRole::Safety
                              ? navigation_interfaces::msg::PlannerCycleTrace::BRANCH_SAFETY
                              : navigation_interfaces::msg::PlannerCycleTrace::BRANCH_NOMINAL;
  trace.status = result.success
                     ? navigation_interfaces::msg::PlannerCycleTrace::STATUS_SUCCESS
                     : navigation_interfaces::msg::PlannerCycleTrace::STATUS_FAILED;
  trace.failure_code = failureName(result.failure_code);
  planner_trace_publisher_->publish(trace);
}

sensor_msgs::msg::PointCloud2 NavigationRuntimeNode::makePointCloud(
    const rog_map::vec_E<rog_map::Vec3f>& points,
    const builtin_interfaces::msg::Time& stamp) const {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = visualization_frame_id_;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  const std::size_t stride = points.size() > visualization_max_points_
                                 ? (points.size() + visualization_max_points_ - 1U) /
                                       visualization_max_points_
                                 : 1U;
  const std::size_t output_count =
      points.empty() ? 0U : (points.size() + stride - 1U) / stride;
  cloud.width = static_cast<std::uint32_t>(std::min<std::size_t>(
      output_count, std::numeric_limits<std::uint32_t>::max()));
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(cloud.width);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (std::size_t index = 0; index < points.size() && x != x.end(); index += stride) {
    *x = static_cast<float>(points[index].x());
    *y = static_cast<float>(points[index].y());
    *z = static_cast<float>(points[index].z());
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

void NavigationRuntimeNode::publishMapVisualization() {
  if (!visualization_enabled_ || !pipeline_->adapter().isInitialized()) {
    return;
  }

  const bool have_occupied_subscriber = occupied_publisher_ &&
      occupied_publisher_->get_subscription_count() > 0;
  const bool have_inflated_subscriber = inflated_occupied_publisher_ &&
      inflated_occupied_publisher_->get_subscription_count() > 0;
  const bool have_unknown_subscriber = unknown_publisher_ &&
      unknown_publisher_->get_subscription_count() > 0;
  const bool have_frontier_subscriber = frontier_publisher_ &&
      frontier_publisher_->get_subscription_count() > 0;
  pipeline_->diagnostics().visualization_subscriber_count =
      static_cast<std::uint64_t>(occupied_publisher_->get_subscription_count() +
                                 inflated_occupied_publisher_->get_subscription_count() +
                                 (unknown_publisher_ ? unknown_publisher_->get_subscription_count() : 0) +
                                 (frontier_publisher_ ? frontier_publisher_->get_subscription_count() : 0));
  if (!have_occupied_subscriber && !have_inflated_subscriber &&
      !have_unknown_subscriber && !have_frontier_subscriber) {
    return;
  }

  try {
    const auto visualization_started = std::chrono::steady_clock::now();
    auto& map = pipeline_->adapter().map();
    const auto center = map.getLocalMapOrigin();
    const rog_map::Vec3f half_range(
        visualization_range_x_m_ * 0.5, visualization_range_y_m_ * 0.5,
        visualization_range_z_m_ * 0.5);
    const rog_map::Vec3f box_min = center - half_range;
    const rog_map::Vec3f box_max = center + half_range;
    rog_map::vec_E<rog_map::Vec3f> occupied;
    rog_map::vec_E<rog_map::Vec3f> inflated_occupied;
    rog_map::vec_E<rog_map::Vec3f> unknown;
    rog_map::vec_E<rog_map::Vec3f> frontier;
    const auto occ_started = std::chrono::steady_clock::now();
    if (have_occupied_subscriber) {
      map.boxSearch(box_min, box_max, super_utils::OCCUPIED, occupied);
    }
    pipeline_->diagnostics().visualization_occ_query_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - occ_started).count();
    const auto inf_started = std::chrono::steady_clock::now();
    if (have_inflated_subscriber) {
      map.boxSearchInflate(box_min, box_max, super_utils::OCCUPIED, inflated_occupied);
    }
    pipeline_->diagnostics().visualization_inf_occ_query_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - inf_started).count();
    const auto unknown_started = std::chrono::steady_clock::now();
    if (have_unknown_subscriber && publish_unknown_) {
      map.boxSearch(box_min, box_max, super_utils::UNKNOWN, unknown);
    }
    pipeline_->diagnostics().visualization_unknown_query_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - unknown_started).count();
    const auto frontier_started = std::chrono::steady_clock::now();
    if (have_frontier_subscriber && publish_frontier_) {
      map.boxSearch(box_min, box_max, super_utils::FRONTIER, frontier);
    }
    pipeline_->diagnostics().visualization_frontier_query_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - frontier_started).count();

    const auto& diagnostics = pipeline_->diagnostics();
    const auto stamp = diagnostics.last_successful_update_stamp_ns > 0
                           ? rosTimeFromNanoseconds(diagnostics.last_successful_update_stamp_ns)
                           : rosTimeFromNanoseconds(get_clock()->now().nanoseconds());
    const auto build_started = std::chrono::steady_clock::now();
    auto occupied_cloud = have_occupied_subscriber ? makePointCloud(occupied, stamp)
                                                   : sensor_msgs::msg::PointCloud2{};
    auto inflated_cloud = have_inflated_subscriber ? makePointCloud(inflated_occupied, stamp)
                                                   : sensor_msgs::msg::PointCloud2{};
    auto unknown_cloud = have_unknown_subscriber && publish_unknown_
                             ? makePointCloud(unknown, stamp)
                             : sensor_msgs::msg::PointCloud2{};
    auto frontier_cloud = have_frontier_subscriber && publish_frontier_
                              ? makePointCloud(frontier, stamp)
                              : sensor_msgs::msg::PointCloud2{};
    pipeline_->diagnostics().visualization_pointcloud_build_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - build_started).count();
    const auto publish_started = std::chrono::steady_clock::now();
    if (have_occupied_subscriber) occupied_publisher_->publish(occupied_cloud);
    if (have_inflated_subscriber) inflated_occupied_publisher_->publish(inflated_cloud);
    if (have_unknown_subscriber && publish_unknown_) {
      unknown_publisher_->publish(unknown_cloud);
    }
    if (have_frontier_subscriber && publish_frontier_) {
      frontier_publisher_->publish(frontier_cloud);
    }
    pipeline_->diagnostics().visualization_publish_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - publish_started).count();
    auto& mutable_diagnostics = pipeline_->diagnostics();
    ++mutable_diagnostics.visualization_publish_count;
    mutable_diagnostics.visualization_occupied_point_count = occupied.size();
    mutable_diagnostics.visualization_inflated_occupied_point_count = inflated_occupied.size();
    mutable_diagnostics.visualization_unknown_point_count = unknown.size();
    mutable_diagnostics.visualization_frontier_point_count = frontier.size();
    mutable_diagnostics.map_updates_since_last_visualization =
        diagnostics.accepted_observation_count - last_visualization_update_count_;
    last_visualization_update_count_ = diagnostics.accepted_observation_count;
    mutable_diagnostics.visualization_source_age_ms =
        std::max<std::int64_t>(0, get_clock()->now().nanoseconds() -
          diagnostics.last_successful_update_stamp_ns) / 1'000'000;
    mutable_diagnostics.visualization_total_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - visualization_started).count();
  } catch (const std::exception& error) {
    ++pipeline_->diagnostics().visualization_exception_count;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "ROG-Map visualization failed; keeping node alive: %s", error.what());
  }
}

void NavigationRuntimeNode::publishDiagnostics() {
  const auto& diagnostics = pipeline_->diagnostics();
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_mapping/world_model";
  status.hardware_id = "rog_map";
  status.level = diagnostics.processing_exception_count > 0 ||
                         diagnostics.visualization_exception_count > 0
                     ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                     : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = pipeline_->adapter().isInitialized() ? "map initialized" : "waiting for first generation";
  status.values = {
      keyValue("received_observation_count", std::to_string(diagnostics.received_observation_count)),
      keyValue("accepted_observation_count", std::to_string(diagnostics.accepted_observation_count)),
      keyValue("generation", std::to_string(diagnostics.generation)),
      keyValue("revision", std::to_string(diagnostics.revision)),
      keyValue("generation_reset_count", std::to_string(diagnostics.generation_reset_count)),
      keyValue("old_generation_drop_count", std::to_string(diagnostics.old_generation_drop_count)),
      keyValue("invalid_stamp_count", std::to_string(diagnostics.invalid_stamp_count)),
      keyValue("invalid_frame_count", std::to_string(diagnostics.invalid_frame_count)),
      keyValue("invalid_pose_count", std::to_string(diagnostics.invalid_pose_count)),
      keyValue("nonfinite_point_count", std::to_string(diagnostics.nonfinite_point_count)),
      keyValue("post_filter_nonfinite_point_count",
               std::to_string(diagnostics.post_filter_nonfinite_point_count)),
      keyValue("transform_nonfinite_point_count",
               std::to_string(diagnostics.transform_nonfinite_point_count)),
      keyValue("invalid_cloud_count", std::to_string(invalid_cloud_count_)),
      keyValue("input_point_count", std::to_string(diagnostics.input_point_count)),
      keyValue("range_filtered_point_count", std::to_string(diagnostics.range_filtered_point_count)),
      keyValue("filtered_point_count", std::to_string(diagnostics.filtered_point_count)),
      keyValue("mapping_point_count", std::to_string(diagnostics.mapping_point_count)),
      keyValue("last_input_stamp_ns", std::to_string(diagnostics.last_input_stamp_ns)),
      keyValue("first_input_stamp_ns", std::to_string(diagnostics.first_input_stamp_ns)),
      keyValue("first_callback_wall_ns", std::to_string(diagnostics.first_callback_wall_ns)),
      keyValue("last_callback_wall_ns", std::to_string(diagnostics.last_callback_wall_ns)),
      keyValue("last_successful_update_stamp_ns",
               std::to_string(diagnostics.last_successful_update_stamp_ns)),
      keyValue("map_update_us", std::to_string(diagnostics.map_update_us)),
      keyValue("mapping_callback_total_us", std::to_string(diagnostics.mapping_callback_total_us)),
      keyValue("ros_pointcloud_decode_us", std::to_string(diagnostics.ros_pointcloud_decode_us)),
      keyValue("mapping_filter_us", std::to_string(diagnostics.mapping_filter_us)),
      keyValue("transform_to_odom_us", std::to_string(diagnostics.transform_to_odom_us)),
      keyValue("rog_update_us", std::to_string(diagnostics.rog_update_us)),
      keyValue("rog_total_update_us", std::to_string(diagnostics.rog_total_update_us)),
      keyValue("rog_raycast_us", std::to_string(diagnostics.rog_raycast_us)),
      keyValue("rog_probability_update_us", std::to_string(diagnostics.rog_probability_update_us)),
      keyValue("rog_inflation_us", std::to_string(diagnostics.rog_inflation_us)),
      keyValue("rog_slide_us", std::to_string(diagnostics.rog_slide_us)),
      keyValue("mapping_filter_input_point_count", std::to_string(diagnostics.mapping_filter_input_point_count)),
      keyValue("mapping_filter_output_point_count", std::to_string(diagnostics.mapping_filter_output_point_count)),
      keyValue("rog_endpoint_count", std::to_string(diagnostics.rog_endpoint_count)),
      keyValue("rog_ray_attempt_count", std::to_string(diagnostics.rog_ray_attempt_count)),
      keyValue("rog_ray_processed_count", std::to_string(diagnostics.rog_ray_processed_count)),
      keyValue("rog_ray_clipped_count", std::to_string(diagnostics.rog_ray_clipped_count)),
      keyValue("rog_ray_skipped_count", std::to_string(diagnostics.rog_ray_skipped_count)),
      keyValue("rog_skip_nonfinite", std::to_string(diagnostics.rog_skip_nonfinite)),
      keyValue("rog_skip_intensity", std::to_string(diagnostics.rog_skip_intensity)),
      keyValue("rog_skip_point_filter", std::to_string(diagnostics.rog_skip_point_filter)),
      keyValue("rog_skip_below_raycast_min_range", std::to_string(diagnostics.rog_skip_below_raycast_min_range)),
      keyValue("rog_skip_endpoint_outside_local_map", std::to_string(diagnostics.rog_skip_endpoint_outside_local_map)),
      keyValue("rog_clipped_virtual_ground_or_ceiling", std::to_string(diagnostics.rog_clipped_virtual_ground_or_ceiling)),
      keyValue("rog_clipped_raycast_max_range", std::to_string(diagnostics.rog_clipped_raycast_max_range)),
      keyValue("rog_clipped_local_update_box", std::to_string(diagnostics.rog_clipped_local_update_box)),
      keyValue("rog_ray_outside_local_map_step", std::to_string(diagnostics.rog_ray_outside_local_map_step)),
      keyValue("rog_voxel_traversal_count", std::to_string(diagnostics.rog_voxel_traversal_count)),
      keyValue("rog_hit_candidate_count", std::to_string(diagnostics.rog_hit_candidate_count)),
      keyValue("rog_miss_candidate_count", std::to_string(diagnostics.rog_miss_candidate_count)),
      keyValue("rog_update_cache_entry_count", std::to_string(diagnostics.rog_update_cache_entry_count)),
      keyValue("map_slide_count", std::to_string(diagnostics.map_slide_count)),
      keyValue("map_slide_cells_cleared", std::to_string(diagnostics.map_slide_cells_cleared)),
      keyValue("rog_allocated_voxel_count", std::to_string(diagnostics.rog_allocated_voxel_count)),
      keyValue("sensor_origin_grid_type", diagnostics.sensor_origin_grid_type),
      keyValue("mapping_observation_receive_count", std::to_string(diagnostics.mapping_observation_receive_count)),
      keyValue("mapping_observation_rejection_count", std::to_string(diagnostics.mapping_observation_rejection_count)),
      keyValue("last_received_observation_sequence",
               std::to_string(diagnostics.last_received_observation_sequence)),
      keyValue("last_received_observation_stream_id",
               std::to_string(diagnostics.last_received_observation_stream_id)),
      keyValue("observation_sequence_stream_switch_count",
               std::to_string(diagnostics.observation_sequence_stream_switch_count)),
      keyValue("observation_sequence_missing_count",
               std::to_string(diagnostics.observation_sequence_missing_count)),
      keyValue("observation_sequence_duplicate_count",
               std::to_string(diagnostics.observation_sequence_duplicate_count)),
      keyValue("observation_sequence_regression_count",
               std::to_string(diagnostics.observation_sequence_regression_count)),
      keyValue("observation_sequence_max_consecutive_missing",
               std::to_string(diagnostics.observation_sequence_max_consecutive_missing)),
      keyValue("processing_exception_count",
               std::to_string(diagnostics.processing_exception_count)),
      keyValue("visualization_publish_count",
               std::to_string(diagnostics.visualization_publish_count)),
      keyValue("visualization_subscriber_count",
               std::to_string(diagnostics.visualization_subscriber_count)),
      keyValue("visualization_exception_count",
               std::to_string(diagnostics.visualization_exception_count)),
      keyValue("visualization_occupied_point_count",
               std::to_string(diagnostics.visualization_occupied_point_count)),
      keyValue("visualization_inflated_occupied_point_count",
               std::to_string(diagnostics.visualization_inflated_occupied_point_count)),
      keyValue("visualization_unknown_point_count",
               std::to_string(diagnostics.visualization_unknown_point_count)),
      keyValue("visualization_frontier_point_count",
               std::to_string(diagnostics.visualization_frontier_point_count)),
      keyValue("visualization_total_us", std::to_string(diagnostics.visualization_total_us)),
      keyValue("visualization_source_age_ms", std::to_string(diagnostics.visualization_source_age_ms)),
      keyValue("map_updates_since_last_visualization",
               std::to_string(diagnostics.map_updates_since_last_visualization)),
  };
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

}  // namespace navigation_runtime
