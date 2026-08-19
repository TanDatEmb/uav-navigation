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
        config.corridor_sample_spacing_m =
            declare_parameter("navigation.planner.corridor_sample_spacing_m", 0.0);
        config.maximum_time_scaling_iterations = static_cast<int>(declare_parameter<std::int64_t>(
            "navigation.planner.maximum_time_scaling_iterations", 8));
        config.allow_unknown_start = declare_parameter(
            "navigation.planner.allow_unknown_start", false);
        config.unknown_start_radius_m = declare_parameter(
            "navigation.planner.unknown_start_radius_m", 0.0);
        config.allow_nominal_unknown = declare_parameter(
            "navigation.planner.allow_nominal_unknown", false);
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
  planning_frame_id_ = pipeline_config.contract.odom_frame_id;
  replan_rate_hz_ = declare_parameter("navigation.replan_rate_hz", 5.0);
  replan_tracking_error_m_ = declare_parameter("navigation.replan_tracking_error_m", 0.5);
  local_subgoal_enabled_ = declare_parameter("navigation.local_subgoal.enabled", true);
  local_goal_boundary_margin_m_ = declare_parameter(
      "navigation.local_subgoal.boundary_margin_m", 1.0);
  local_goal_max_distance_m_ = declare_parameter(
      "navigation.local_subgoal.max_distance_m", 5.0);
  local_goal_switch_distance_m_ = declare_parameter(
      "navigation.local_subgoal.switch_distance_m", 0.8);
  local_goal_continuation_speed_fraction_ = declare_parameter(
      "navigation.local_subgoal.continuation_speed_fraction", 0.35);
  if (!std::isfinite(replan_rate_hz_) || replan_rate_hz_ <= 0.0) {
    throw std::invalid_argument("navigation.replan_rate_hz must be positive and finite");
  }
  if (!std::isfinite(replan_tracking_error_m_) || replan_tracking_error_m_ <= 0.0) {
    throw std::invalid_argument(
        "navigation.replan_tracking_error_m must be finite and positive");
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
  if (!std::isfinite(planner_config_.nominal_commitment_horizon_s) ||
      planner_config_.nominal_commitment_horizon_s < 0.0) {
    throw std::invalid_argument(
        "navigation.planner.nominal_commitment_horizon_s must be finite and non-negative");
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
  planned_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "navigation/visualization/planned_path", rclcpp::QoS{rclcpp::KeepLast{1}}.reliable());

  // Diagnostics are a periodic snapshot, not part of the observation hot
  // path. Create this in the same group as map mutation so the plain
  // MappingDiagnostics struct is never read concurrently with an update.
  mapping_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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
      state_topic_, rclcpp::QoS{rclcpp::KeepLast{10}}.reliable(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) { onOdometry(message); },
      subscription_options);
  goal_subscription_ = create_subscription<navigation_interfaces::msg::NavigationGoal>(
      "navigation/goal", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable(),
      [this](const navigation_interfaces::msg::NavigationGoal::ConstSharedPtr message) {
        onGoal(message);
      },
      subscription_options);
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
  latest_odometry_ = *message;
}

void NavigationRuntimeNode::onGoal(
    const navigation_interfaces::msg::NavigationGoal::ConstSharedPtr& message) {
  if (message->mission_id.empty() || message->request_id == 0U ||
      !std::isfinite(message->acceptance_radius_m) || message->acceptance_radius_m <= 0.0) {
    RCLCPP_WARN(get_logger(), "Rejecting navigation goal with invalid identity or acceptance radius");
    return;
  }
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
  planActiveGoal();
}

void NavigationRuntimeNode::planActiveGoal() {
  if (!active_goal_.has_value()) return;
  if (planner_heartbeat_publisher_) {
    planner_heartbeat_publisher_->publish(std_msgs::msg::Empty{});
  }
  const auto goal_message = *active_goal_;
  // The planner and PX4 mode run on separate ROS/DDS callbacks. A small
  // future timestamp is normal transport skew; larger skew is rejected.
  constexpr std::int64_t kMaximumFutureStateSkewNs = 50'000'000LL;
  navigation_mapping::WorldModel world(pipeline_->adapter());
  const auto current_position = [&]() -> std::optional<navigation_mapping::Vec3> {
    if (!latest_odometry_.has_value()) return std::nullopt;
    const auto& odometry = *latest_odometry_;
    const auto& position = odometry.pose.pose.position;
    const auto value = navigation_mapping::Vec3{position.x, position.y, position.z};
    return value.allFinite() ? std::optional<navigation_mapping::Vec3>{value} : std::nullopt;
  }();
  const bool goal_unchanged =
      last_plan_identity_valid_ &&
      goal_message.mission_id == last_planned_mission_id_ &&
      goal_message.waypoint_index == last_planned_waypoint_index_ &&
      goal_message.request_id == last_planned_request_id_;
  const bool braking_stop_latched = goal_unchanged && braking_stop_latched_;
  if (!goal_unchanged) {
    committed_local_goal_.reset();
  } else if (committed_local_goal_.has_value() && current_position.has_value() &&
             (*committed_local_goal_ - *current_position).norm() <=
                 local_goal_switch_distance_m_) {
    // The current receding-horizon leg is complete. Allow the selector to
    // choose the next long leg toward the mission waypoint.
    committed_local_goal_.reset();
  }
  const bool tracking_error_exceeded = [&]() {
    if (!current_position.has_value() || !last_planned_trajectory_.has_value() ||
        last_plan_time_ns_ <= 0) {
      return true;
    }
    const double elapsed_s =
        static_cast<double>(get_clock()->now().nanoseconds() - last_plan_time_ns_) / 1e9;
    if (!std::isfinite(elapsed_s) || elapsed_s < 0.0 ||
        elapsed_s >= last_planned_trajectory_->duration_s) {
      return true;
    }
    const auto expected = interpolateTrajectory(*last_planned_trajectory_, elapsed_s);
    const double tracking_error = (*current_position - expected.position).norm();
    return !std::isfinite(tracking_error) || tracking_error >= replan_tracking_error_m_;
  }();
  const bool current_state_fresh = [&]() {
    if (!latest_odometry_.has_value()) return false;
    const auto& odometry = *latest_odometry_;
    const auto age_ns = get_clock()->now().nanoseconds() - timeNanoseconds(odometry.header.stamp);
    return odometry.header.frame_id == planning_frame_id_ &&
           age_ns >= -kMaximumFutureStateSkewNs &&
           std::isfinite(state_max_age_s_) && state_max_age_s_ > 0.0 &&
           age_ns <= static_cast<std::int64_t>(state_max_age_s_ * 1e9);
  }();
  const bool trajectory_near_stale = [&]() {
    if (last_plan_time_ns_ <= 0 || !std::isfinite(last_planned_duration_s_) ||
        last_planned_duration_s_ <= 0.0) {
      return true;
    }
    const auto now_ns = get_clock()->now().nanoseconds();
    if (now_ns < last_plan_time_ns_) return true;
    constexpr double kReplanBeforeStaleS = 0.25;
    const double replan_deadline_s =
        std::max(0.0, last_planned_duration_s_ - kReplanBeforeStaleS);
    return static_cast<double>(now_ns - last_plan_time_ns_) / 1e9 >= replan_deadline_s;
  }();
  const bool world_unchanged =
      world.generation() == last_planned_world_generation_ &&
      world.revision() == last_planned_world_revision_;

  // A verified braking stop is a committed terminal trajectory for this
  // waypoint/request.  Once its stop phase has completed, a rolling planning
  // timer must not turn the same goal into a stream of freshly generated
  // zero-duration stops.  Revalidate a still-running stop only when the map
  // changed; after the stop has elapsed, the measured vehicle state is the
  // authority and a replacement is needed only if it is no longer stopped.
  if (braking_stop_latched && current_state_fresh && latest_odometry_.has_value() &&
      last_planned_trajectory_.has_value() && last_plan_time_ns_ > 0) {
    const auto& odometry = *latest_odometry_;
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
    if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
        elapsed_s >= last_planned_trajectory_->duration_s &&
        state.velocity.norm() <= 0.15 + 1e-6) {
      held_stop_safe = true;
    } else if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
               elapsed_s < last_planned_trajectory_->duration_s && world_unchanged) {
      held_stop_safe = true;
    } else if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
               elapsed_s < last_planned_trajectory_->duration_s &&
               last_planned_trajectory_->finiteAndMonotonic()) {
      const auto expected = interpolateTrajectory(*last_planned_trajectory_, elapsed_s);
      const double tracking_error = (state.position - expected.position).norm();
      if (std::isfinite(tracking_error) && tracking_error < replan_tracking_error_m_) {
        const auto remaining = remainingTrajectory(*last_planned_trajectory_, elapsed_s);
        navigation_planning::VehicleState verification_state;
        verification_state.position = remaining.points.front().position;
        verification_state.velocity = remaining.points.front().velocity;
        verification_state.acceleration = remaining.points.front().acceleration;
        const auto verification = trajectory_verifier_.verify(
            remaining, verification_state, navigation_planning::PlanRole::Safety, world);
        held_stop_safe = verification.success;
      }
    }
    if (held_stop_safe) {
      ++plan_skip_count_;
      ++trajectory_reuse_count_;
      last_planned_world_generation_ = world.generation();
      last_planned_world_revision_ = world.revision();
      last_replan_reason_ = "reuse_latched_braking_stop";
      return;
    }
    last_replan_reason_ = "safety_stop_latched_invalidated";
  }

  // A committed trajectory is reusable across map revisions only after the
  // remaining corridor has been checked against the new world. This avoids
  // turning every incoming scan into a full A* invocation while retaining a
  // fail-closed response to a newly occupied cell.
  if (!planner_config_.allow_nominal_unknown && goal_unchanged && last_plan_success_ &&
      current_state_fresh && !tracking_error_exceeded &&
      (braking_stop_latched || !trajectory_near_stale)) {
    if (world_unchanged) {
      ++plan_skip_count_;
      ++trajectory_reuse_count_;
      last_replan_reason_ = "reuse_unchanged_world";
      return;
    }
    if (world.generation() == last_planned_world_generation_ &&
        last_planned_trajectory_.has_value() && last_plan_time_ns_ > 0) {
      const auto& odometry = *latest_odometry_;
      navigation_planning::VehicleState state;
      state.position = navigation_mapping::Vec3{odometry.pose.pose.position.x,
                                                odometry.pose.pose.position.y,
                                                odometry.pose.pose.position.z};
      state.velocity = navigation_mapping::Vec3{odometry.twist.twist.linear.x,
                                                odometry.twist.twist.linear.y,
                                                odometry.twist.twist.linear.z};
      state.acceleration = navigation_mapping::Vec3::Zero();
      const double elapsed_s =
          static_cast<double>(get_clock()->now().nanoseconds() - last_plan_time_ns_) / 1e9;
      const auto expected = interpolateTrajectory(*last_planned_trajectory_, elapsed_s);
      const double tracking_error = (state.position - expected.position).norm();
      ++trajectory_revalidation_count_;
      if (std::isfinite(tracking_error) && tracking_error < replan_tracking_error_m_) {
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

  navigation_planning::PlanResult result;
  if (goal_message.header.frame_id != planning_frame_id_ ||
      timeNanoseconds(goal_message.header.stamp) <= 0 ||
      !std::isfinite(goal_message.target.x) || !std::isfinite(goal_message.target.y) ||
      !std::isfinite(goal_message.target.z)) {
    result.failure_code = navigation_planning::PlanFailureCode::InvalidState;
  } else if (!latest_odometry_.has_value()) {
    result.failure_code = navigation_planning::PlanFailureCode::InvalidState;
  } else {
    const auto& odometry = *latest_odometry_;
    const auto age_ns = get_clock()->now().nanoseconds() - timeNanoseconds(odometry.header.stamp);
    const auto& position = odometry.pose.pose.position;
    const auto& velocity = odometry.twist.twist.linear;
    if (age_ns < -kMaximumFutureStateSkewNs || !std::isfinite(state_max_age_s_) ||
        state_max_age_s_ <= 0.0 ||
        age_ns > static_cast<std::int64_t>(state_max_age_s_ * 1e9)) {
      result.failure_code = navigation_planning::PlanFailureCode::InvalidState;
    } else {
      navigation_planning::VehicleState state;
      state.position = navigation_mapping::Vec3{position.x, position.y, position.z};
      state.velocity = navigation_mapping::Vec3{velocity.x, velocity.y, velocity.z};
      // Odometry has no authoritative linear acceleration field; zero is the
      // only safe value accepted by the current planner state contract.
      state.acceleration = navigation_mapping::Vec3::Zero();
      const navigation_mapping::Vec3 requested_goal{goal_message.target.x, goal_message.target.y,
                                                    goal_message.target.z};
      const auto local_goal = local_subgoal_enabled_
                                  ? selectLocalGoal(world, state.position, requested_goal,
                                                    local_goal_boundary_margin_m_,
                                                    local_goal_max_distance_m_,
                                                    committed_local_goal_)
                                  : LocalGoalSelection{LocalGoalSelectionStatus::Direct,
                                                        requested_goal};
      if (local_goal.usesSubGoal()) {
        committed_local_goal_ = local_goal.goal;
      } else if (local_goal.status == LocalGoalSelectionStatus::Direct ||
                 !local_goal.success()) {
        committed_local_goal_.reset();
      }
      last_local_goal_status_ = local_goal.status;
      last_local_subgoal_selected_ = local_goal.usesSubGoal();
      last_effective_goal_ = local_goal.goal;
      if (local_goal.usesSubGoal()) {
        ++local_subgoal_selected_count_;
      } else if (!local_goal.success()) {
        ++local_subgoal_failure_count_;
      }
      navigation_planning::Goal goal;
      goal.position = local_goal.success() ? local_goal.goal : requested_goal;
      goal.terminal = !local_goal.usesSubGoal();
      if (!goal.terminal) {
        // Carry motion through a receding-horizon local goal. Prefer the
        // current measured velocity when it already points toward the local
        // target; otherwise seed a conservative continuation tangent. The
        // planner applies its design-limit clamp and the verifier remains the
        // final authority.
        const auto direction = goal.position - state.position;
        const double direction_norm = direction.norm();
        const double continuation_speed = std::clamp(
            state.velocity.norm(),
            local_goal_continuation_speed_fraction_ * planner_config_.limits.max_velocity_mps,
            designVelocityLimit(planner_config_.limits));
        if (direction_norm > 1e-6 && std::isfinite(continuation_speed)) {
          goal.terminal_velocity = (continuation_speed / direction_norm) * direction;
        } else {
          goal.terminal_velocity = navigation_mapping::Vec3::Zero();
        }
      }
      last_goal_terminal_ = goal.terminal;
      last_terminal_velocity_ = goal.terminal_velocity;
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
      const auto rejectCandidate = [](navigation_planning::PlanResult candidate,
                                       navigation_planning::PlanFailureCode failure) {
        candidate.success = false;
        candidate.trajectory.points.clear();
        candidate.trajectory.duration_s = 0.0;
        candidate.failure_code = failure;
        return candidate;
      };

      if (!local_goal.success()) {
        result.failure_code = local_goal.status == LocalGoalSelectionStatus::StartOutsideBounds
                                  ? navigation_planning::PlanFailureCode::StartOutsideBounds
                                  : navigation_planning::PlanFailureCode::GoalOutsideBounds;
      } else if (braking_stop_latched) {
        // Once a verified braking stop has been selected, map revisions and
        // rolling timer ticks may only replace it with another verified
        // braking stop. Never restart nominal execution at the same waypoint.
        result = validate(planner_.planSafetyStop(state, world));
        last_safety_failure_code_ = result.failure_code;
      } else {
        // Always produce both candidates for the active local horizon. In the
        // normal runtime the nominal candidate is the conservative known-free
        // plan; the optional simulation flag only changes its unknown-space
        // policy. The safety route/stop is verified independently so reports
        // show which branch was selected instead of hiding safety planning
        // behind a planner failure.
        ++nominal_plan_count_;
        const auto nominal_result = planner_config_.allow_nominal_unknown
                                         ? planner_.planNominal(state, goal, world)
                                         : planner_.plan(state, goal, world);
        ++safety_route_plan_count_;
        auto safety_result = planner_.planSafetyRoute(state, goal, world);
        safety_result = validate(safety_result);
        if (safety_result.success) ++safety_route_verified_count_;
        if (!safety_result.success) {
          safety_result = validate(planner_.planSafetyStop(state, world));
        }
        last_nominal_failure_code_ = nominal_result.failure_code;
        last_safety_failure_code_ = safety_result.failure_code;
        if (nominal_result.success && safety_result.success) {
          auto committed_nominal = nominal_result.trajectory;
          if (planner_config_.allow_nominal_unknown) {
            committed_nominal = committedPrefix(
                committed_nominal, planner_config_.nominal_commitment_horizon_s);
          }
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
          } else if (dual.nominal_selected) {
            result = nominal_result;
            result.trajectory = std::move(committed_nominal);
            result.role = planner_config_.allow_nominal_unknown
                              ? navigation_planning::PlanRole::Nominal
                              : navigation_planning::PlanRole::Committed;
            result.safety_kind = navigation_planning::SafetyPlanKind::None;
            ++nominal_selected_count_;
          } else {
            result = safety_result;
            result.role = navigation_planning::PlanRole::Safety;
            ++safety_fallback_count_;
          }
        } else if (!nominal_result.success && safety_result.success) {
          result = safety_result;
          if (result.success) ++safety_fallback_count_;
        } else if (nominal_result.success) {
          // An optimistic candidate without a valid known-free stop is never
          // publishable, even if its own verifier would pass.
          ++dual_verification_failure_count_;
          result = rejectCandidate(nominal_result, safety_result.failure_code);
        } else {
          result = safety_result;
        }
      }

      // A failed trajectory is a state invalidation event. Preserve the
      // current map provenance so the PX4 adapter can process the failure
      // before its accepted-revision filter instead of keeping an old path.
      if (!result.success) {
        result.world_generation = world.generation();
        result.world_revision = world.revision();
      }

      if (result.success && result.safety_kind == navigation_planning::SafetyPlanKind::Route) {
        ++safety_route_selected_count_;
      } else if (result.success &&
                 result.safety_kind == navigation_planning::SafetyPlanKind::BrakingStop) {
        ++safety_stop_selected_count_;
      }

      last_plan_identity_valid_ = true;
      last_planned_mission_id_ = goal_message.mission_id;
      last_planned_waypoint_index_ = goal_message.waypoint_index;
      last_planned_request_id_ = goal_message.request_id;
      last_planned_world_generation_ = world.generation();
      last_planned_world_revision_ = world.revision();
      if (result.success) {
        last_plan_time_ns_ = get_clock()->now().nanoseconds();
        last_planned_duration_s_ = result.trajectory.duration_s;
        last_planned_trajectory_ = result.trajectory;
        last_planned_role_ = result.role;
        last_planned_safety_kind_ = result.safety_kind;
        braking_stop_latched_ =
            result.role == navigation_planning::PlanRole::Safety &&
            result.safety_kind == navigation_planning::SafetyPlanKind::BrakingStop;
      } else {
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
  last_plan_success_ = result.success;
  trajectory_publisher_->publish(makeTrajectoryMessage(result, goal_message));
  if (result.success) {
    planned_path_publisher_->publish(makePathMessage(result, goal_message.header));
  }
  publishPlanningDiagnostics(result);
}

navigation_interfaces::msg::PlannedTrajectory NavigationRuntimeNode::makeTrajectoryMessage(
    const navigation_planning::PlanResult& result,
    const navigation_interfaces::msg::NavigationGoal& goal) const {
  navigation_interfaces::msg::PlannedTrajectory message;
  // The trajectory timestamp is the execution time origin. The adapter uses
  // it to account for transport latency rather than inheriting phase from a
  // previous rolling trajectory.
  message.header = goal.header;
  message.header.stamp = get_clock()->now();
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

nav_msgs::msg::Path NavigationRuntimeNode::makePathMessage(
    const navigation_planning::PlanResult& result, const std_msgs::msg::Header& header) const {
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(result.trajectory.points.size());
  for (const auto& point : result.trajectory.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.header.stamp = header.stamp;
    pose.pose.position.x = point.position.x();
    pose.pose.position.y = point.position.y();
    pose.pose.position.z = point.position.z();
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
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
      keyValue("nominal_plan_count", std::to_string(nominal_plan_count_)),
      keyValue("nominal_selected_count", std::to_string(nominal_selected_count_)),
      keyValue("dual_verification_failure_count",
               std::to_string(dual_verification_failure_count_)),
      keyValue("verification_failure_count", std::to_string(verification_failure_count_)),
      keyValue("local_subgoal_enabled", local_subgoal_enabled_ ? "true" : "false"),
      keyValue("local_subgoal_selected_count", std::to_string(local_subgoal_selected_count_)),
      keyValue("local_subgoal_failure_count", std::to_string(local_subgoal_failure_count_)),
      keyValue("trajectory_revalidation_count", std::to_string(trajectory_revalidation_count_)),
      keyValue("trajectory_revalidation_failure_count",
               std::to_string(trajectory_revalidation_failure_count_)),
      keyValue("trajectory_reuse_count", std::to_string(trajectory_reuse_count_)),
      keyValue("full_replan_count", std::to_string(full_replan_count_)),
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
