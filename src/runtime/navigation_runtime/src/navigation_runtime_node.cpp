#include "navigation_runtime/navigation_runtime_node.hpp"
#include "navigation_runtime/mission_dynamics.hpp"
#include "navigation_runtime/mapping_fail_stop.hpp"
#include "navigation_runtime/commit_trace.hpp"

#include <navigation_mapping/mapping_actor.hpp>

#include "navigation_runtime/planner_fsm.hpp"
#include "navigation_runtime/planning_supervisor.hpp"
#include "navigation_runtime/planning_worker.hpp"
#include "navigation_runtime/runtime_boundaries.hpp"
#include <navigation_execution/timestamp_freshness.hpp>
#include <navigation_contracts/command_safety_contract.hpp>
#include <navigation_contracts/execution_state_freshness.hpp>
#include <navigation_common/time.hpp>
#include <navigation_world_model/goal_contract.hpp>
#include <navigation_planning_backend/planner_facade.hpp>
#include <navigation_planning/candidate_admission.hpp>
#include <navigation_planning/planning_timing.hpp>
#include <navigation_mission/route_progress.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace navigation_runtime {
namespace {

bool finiteNonzeroQuaternion(const Eigen::Quaterniond& quaternion) {
  const double scale = quaternion.coeffs().cwiseAbs().maxCoeff();
  return quaternion.coeffs().allFinite() && std::isfinite(scale) && scale > 1.0e-9;
}

bool propagatedOdometryFinite(const nav_msgs::msg::Odometry& odometry) {
  const auto& position = odometry.pose.pose.position;
  const auto& orientation = odometry.pose.pose.orientation;
  const auto& velocity = odometry.twist.twist.linear;
  const Eigen::Quaterniond quaternion{
      orientation.w, orientation.x, orientation.y, orientation.z};
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z) && std::isfinite(velocity.x) &&
         std::isfinite(velocity.y) && std::isfinite(velocity.z) &&
         finiteNonzeroQuaternion(quaternion);
}

bool hasFloatField(const sensor_msgs::msg::PointCloud2& message, const std::string& name) {
  return std::any_of(message.fields.begin(), message.fields.end(), [&](const auto& field) {
    return field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
           field.count >= 1 && static_cast<std::uint64_t>(field.offset) + sizeof(float) <=
                                    message.point_step;
  });
}

double pointFromMessage(const geometry_msgs::msg::Point& point, int axis) {
  if (axis == 0) return point.x;
  if (axis == 1) return point.y;
  return point.z;
}

const geometry_msgs::msg::Point& plannerTarget(
    const navigation_contracts::msg::NavigationGoal& goal) {
  // The current checkpoint remains the geometric endpoint. For pass-through
  // continuity, the runtime separately forwards next_target as an outgoing
  // tangent; it must never replace this waypoint in the geometric problem.
  return goal.target;
}

double goalCompletionTolerance(
    const navigation_contracts::msg::NavigationGoal& goal) {
  // MissionController owns the waypoint acceptance contract.  The planner's
  // smaller geometric connection tolerance is intentionally not reused for
  // the runtime terminal decision: a certified local endpoint inside the
  // requested acceptance ball is a valid waypoint hold, including a safety
  // suffix that stopped short of the exact coordinate.
  if (std::isfinite(goal.acceptance_radius_m) && goal.acceptance_radius_m > 0.0) {
    return std::max(navigation_world_model::kGoalCompletionToleranceM,
                    goal.acceptance_radius_m);
  }
  return navigation_world_model::kGoalCompletionToleranceM;
}

std::optional<navigation_mission::ImmutableRouteSnapshot> decodeRouteSnapshot(
    const navigation_contracts::msg::NavigationGoal& goal) noexcept {
  try {
  const auto& source = goal.route;
  const std::size_t count = source.waypoint_positions.size();
  if (count == 0U || count > navigation_mission::kMaximumMissionWaypoints ||
      source.mission_id.size() > navigation_mission::kMaximumMissionIdLength ||
      source.frame_id.size() > navigation_mission::kMaximumMissionFrameLength ||
      source.waypoint_ids.size() != count ||
      source.waypoint_acceptance_radii_m.size() != count ||
      source.waypoint_behaviors.size() != count ||
      source.mission_id != goal.mission_id || source.frame_id != goal.header.frame_id ||
      source.request_id != goal.request_id ||
      source.active_waypoint_index != goal.waypoint_index ||
      source.active_waypoint_index >= count || !source.measured_progress_valid) {
    return std::nullopt;
  }
  navigation_mission::Mission mission;
  mission.id = source.mission_id;
  mission.frame = source.frame_id;
  mission.waypoints.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const auto& point = source.waypoint_positions[index];
    const double radius = source.waypoint_acceptance_radii_m[index];
    const std::uint8_t behavior = source.waypoint_behaviors[index];
    if (source.waypoint_ids[index].empty() ||
        source.waypoint_ids[index].size() > navigation_mission::kMaximumWaypointIdLength ||
        !std::isfinite(point.x) ||
        !std::isfinite(point.y) || !std::isfinite(point.z) ||
        !std::isfinite(radius) || radius <= 0.0 ||
        (behavior != navigation_contracts::msg::RouteSnapshot::BEHAVIOR_PASS_THROUGH &&
         behavior != navigation_contracts::msg::RouteSnapshot::BEHAVIOR_STOP)) {
      return std::nullopt;
    }
    navigation_mission::MissionWaypoint waypoint;
    waypoint.id = source.waypoint_ids[index];
    waypoint.position_enu = Eigen::Vector3d{point.x, point.y, point.z};
    waypoint.acceptance_radius_m = radius;
    waypoint.behavior =
        behavior == navigation_contracts::msg::RouteSnapshot::BEHAVIOR_STOP
            ? navigation_mission::MissionWaypoint::Behavior::Stop
            : navigation_mission::MissionWaypoint::Behavior::PassThrough;
    mission.waypoints.push_back(std::move(waypoint));
  }
  const bool active_pass_through =
      mission.waypoints[source.active_waypoint_index].behavior ==
      navigation_mission::MissionWaypoint::Behavior::PassThrough;
  if (!waypointBehaviorContractValid(
          active_pass_through, source.active_waypoint_index + 1U < count)) {
    return std::nullopt;
  }

  navigation_mission::RouteProgress route(mission);
  navigation_mission::ImmutableRouteSnapshot snapshot;
  snapshot.mission_id = source.mission_id;
  snapshot.frame = source.frame_id;
  snapshot.route_revision = source.route_revision;
  snapshot.request_id = source.request_id;
  snapshot.active_waypoint_index = source.active_waypoint_index;
  snapshot.waypoints = mission.waypoints;
  snapshot.segments = route.segments();
  snapshot.waypoint_arc_lengths_m.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    snapshot.waypoint_arc_lengths_m.push_back(route.waypointArcLengthM(index));
  }
  snapshot.total_length_m = route.totalLengthM();
  snapshot.measured_progress.valid = true;
  snapshot.measured_progress.progress_arc_m = source.measured_progress_arc_m;
  snapshot.measured_progress.projection.valid = true;
  snapshot.measured_progress.projection.segment_index = snapshot.segments.empty()
      ? std::numeric_limits<std::size_t>::max()
      : source.measured_segment_index;
  snapshot.measured_progress.projection.arc_length_m =
      source.measured_projection_arc_m;
  snapshot.measured_progress.projection.lateral_error_m =
      source.measured_lateral_error_m;
  if ((!snapshot.segments.empty() &&
       source.measured_segment_index >= snapshot.segments.size()) ||
      (snapshot.segments.empty() && source.measured_segment_index != 0U) ||
      !std::isfinite(source.measured_progress_arc_m) ||
      !std::isfinite(source.measured_projection_arc_m) ||
      !std::isfinite(source.measured_lateral_error_m)) {
    return std::nullopt;
  }
  if (snapshot.segments.empty()) {
    if (std::abs(source.measured_projection_arc_m) > 1.0e-6) {
      return std::nullopt;
    }
    snapshot.measured_progress.projection.segment_fraction = 0.0;
  } else {
    const auto& measured_segment =
        snapshot.segments[source.measured_segment_index];
    const double fraction =
        (source.measured_projection_arc_m - measured_segment.start_arc_m) /
        measured_segment.length_m;
    if (!std::isfinite(fraction) || fraction < -1.0e-6 || fraction > 1.0 + 1.0e-6) {
      return std::nullopt;
    }
    snapshot.measured_progress.projection.segment_fraction =
        std::clamp(fraction, 0.0, 1.0);
  }
  const auto progress_point = route.pointAtArc(source.measured_projection_arc_m);
  if (!progress_point.has_value()) return std::nullopt;
  snapshot.measured_progress.projection.point = *progress_point;
  snapshot.measured_progress.projection.tangent = snapshot.segments.empty()
      ? Eigen::Vector3d::Zero()
      : snapshot.segments[snapshot.measured_progress.projection.segment_index].tangent;
    return snapshot.valid()
             ? std::optional<navigation_mission::ImmutableRouteSnapshot>{
                   std::move(snapshot)}
             : std::nullopt;
  } catch (const std::exception&) {
    return std::nullopt;
  } catch (...) {
    return std::nullopt;
  }
}

bool routeSnapshotMatchesGoalMirrors(
    const navigation_mission::ImmutableRouteSnapshot& route,
    const navigation_contracts::msg::NavigationGoal& goal) noexcept {
  if (!route.valid() || route.active_waypoint_index >= route.waypoints.size()) {
    return false;
  }
  const auto point_matches = [](const geometry_msgs::msg::Point& message,
                                const Eigen::Vector3d& expected) {
    return std::isfinite(message.x) && std::isfinite(message.y) &&
           std::isfinite(message.z) &&
           (Eigen::Vector3d{message.x, message.y, message.z} - expected).norm() <=
               1.0e-6;
  };
  const auto& active = route.waypoints[route.active_waypoint_index];
  const std::uint8_t expected_behavior =
      active.behavior == navigation_mission::MissionWaypoint::Behavior::Stop
          ? navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP
          : navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH;
  if (!point_matches(goal.target, active.position_enu) ||
      !std::isfinite(goal.acceptance_radius_m) ||
      std::abs(goal.acceptance_radius_m - active.acceptance_radius_m) > 1.0e-9 ||
      goal.behavior != expected_behavior) {
    return false;
  }
  const std::size_t next_index = route.active_waypoint_index + 1U;
  const bool expected_next = next_index < route.waypoints.size();
  return goal.has_next_target == expected_next &&
         (!expected_next ||
          point_matches(goal.next_target, route.waypoints[next_index].position_enu));
}

void addObservationAccountingValues(
    diagnostic_msgs::msg::DiagnosticStatus& status,
    const navigation_mapping::ObservationAccounting::Snapshot& accounting) {
  const auto add_value = [&status](const std::string& key, std::uint64_t value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  };
  // These fields are one canonical snapshot. Report consumers must not
  // reconstruct lifecycle state by combining independent diagnostic events.
  add_value("received_observation_count", accounting.received);
  add_value("observation_rejected_before_inbox_count", accounting.rejected_before_inbox);
  add_value("accepted_observation_count", accounting.accepted_to_inbox);
  add_value("observation_accepted_to_inbox_count", accounting.accepted_to_inbox);
  add_value("observation_replaced_pending_count", accounting.replaced_pending);
  add_value("observation_replaced_waiting_count", accounting.replaced_waiting);
  add_value("observation_replaced_ready_count", accounting.replaced_ready);
  add_value("dropped_cloud_count", accounting.replaced_waiting + accounting.replaced_ready);
  add_value("observation_discarded_pending_count", accounting.discarded_pending);
  add_value("observation_discarded_waiting_count", accounting.discarded_waiting);
  add_value("observation_discarded_ready_count", accounting.discarded_ready);
  add_value("observation_discarded_shutdown_ready_count", accounting.discarded_shutdown_ready);
  add_value("observation_discarded_nonmonotonic_count", accounting.discarded_nonmonotonic);
  add_value("observation_ready_submitted_count", accounting.ready_submitted);
  add_value("observation_waiting_count", accounting.waiting);
  add_value("observation_ready_count", accounting.ready);
  add_value("mapping_started_count", accounting.mapping_started);
  add_value("mapping_published_count", accounting.mapping_published);
  add_value("mapping_failed_count", accounting.mapping_failed);
  add_value("mapping_pending_count", accounting.pending);
  add_value("mapping_in_flight_count", accounting.in_flight);
  add_value("observation_accounting_valid", accounting.allInvariantsHold() ? 1U : 0U);
  add_value("observation_accounting_violation_count", accounting.violation_count);
}

void appendJsonString(std::ostringstream& output, const std::string_view value) {
  output << '"';
  for (const char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      default:
        if (static_cast<unsigned char>(character) < 0x20U) {
          constexpr char hex[] = "0123456789abcdef";
          const auto value_byte = static_cast<unsigned char>(character);
          output << "\\u00" << hex[value_byte >> 4U] << hex[value_byte & 0x0fU];
        } else {
          output << character;
        }
        break;
    }
  }
  output << '"';
}

void appendJsonNumber(std::ostringstream& output, const double value) {
  if (std::isfinite(value)) {
    output << std::setprecision(17) << value;
  } else {
    output << "null";
  }
}

void appendJsonVector(std::ostringstream& output, const Eigen::Vector3d& value) {
  output << '[';
  appendJsonNumber(output, value.x());
  output << ',';
  appendJsonNumber(output, value.y());
  output << ',';
  appendJsonNumber(output, value.z());
  output << ']';
}

void appendJsonPath(std::ostringstream& output,
                    const std::vector<Eigen::Vector3d>& points) {
  output << '[';
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (index != 0U) output << ',';
    appendJsonVector(output, points[index]);
  }
  output << ']';
}

std::string plannerPathSnapshotJson(
    const navigation_planning::CommittedTrajectorySnapshot& snapshot,
    const std::uint64_t planning_cycle_id,
    const std::uint64_t solve_generation,
    const int planner_result,
    const int replan_code,
    const int commit_decision,
    const std::string_view solve_stage_name,
    const Eigen::Vector3d& target,
    const Eigen::Vector3d& planning_target,
    const double goal_acceptance_radius_m,
    const bool goal_endpoint_adjusted,
    const navigation_world_model::CellState robot_grid,
    const navigation_world_model::CellState robot_inflated_grid,
    const navigation_world_model::CellState target_grid,
    const navigation_world_model::CellState target_inflated_grid,
    const bool safety_suffix_active) {
  const auto& trajectory = snapshot.position;
  const bool has_committed_bundle = snapshot.generation > 0U && !trajectory.empty();
  std::vector<Eigen::Vector3d> nominal_path;
  std::vector<Eigen::Vector3d> backup_path;
  std::vector<Eigen::Vector3d> emergency_path;
  if (has_committed_bundle && std::isfinite(trajectory.duration_s) &&
      trajectory.duration_s > 0.0) {
    constexpr std::size_t kMaximumPathPoints = 64U;
    const auto point_count = boundedTrajectorySampleCount(
        trajectory.duration_s, 0.08, kMaximumPathPoints);
    if (point_count) {
      nominal_path.reserve(*point_count);
      backup_path.reserve(*point_count);
      emergency_path.reserve(*point_count);
    }
    for (std::size_t index = 0; point_count && index < *point_count; ++index) {
      const double trajectory_time = trajectory.duration_s * static_cast<double>(index) /
                                     static_cast<double>(*point_count - 1U);
      navigation_planning::TrajectoryPoint point;
      if (!trajectory.sample(trajectory_time, point)) continue;
      switch (point.role) {
        case navigation_planning::CandidateRole::kMain:
          nominal_path.push_back(point.position_world);
          break;
        case navigation_planning::CandidateRole::kBackup:
          backup_path.push_back(point.position_world);
          break;
        case navigation_planning::CandidateRole::kEmergency:
          emergency_path.push_back(point.position_world);
          break;
      }
    }
  }

  std::ostringstream output;
  output << '{';
  output << "\"schema_version\":1";
  output << ",\"source\":";
  appendJsonString(output, has_committed_bundle ? "committed_bundle" : "none");
  output << ",\"path_available\":" << (has_committed_bundle ? "true" : "false");
  output << ",\"absence_reason\":";
  appendJsonString(output, has_committed_bundle ? "" : "no_committed_executable_candidate");
  output << ",\"terminal_command_semantics\":";
  appendJsonString(output, has_committed_bundle ? "trajectory_sample" : "no_path_origin_hold");
  output << ",\"planning_cycle_id\":" << planning_cycle_id;
  output << ",\"solve_generation\":" << solve_generation;
  output << ",\"bundle_generation\":" << snapshot.generation;
  output << ",\"planner_result\":" << planner_result;
  output << ",\"replan_code\":" << replan_code;
  output << ",\"commit_decision\":" << commit_decision;
  output << ",\"solve_stage_name\":";
  appendJsonString(output, solve_stage_name);
  output << ",\"selected_role\":";
  appendJsonString(output, safety_suffix_active ? "backup" : "nominal");
  output << ",\"target\" :";
  appendJsonVector(output, target);
  output << ",\"planning_target\" :";
  appendJsonVector(output, planning_target);
  output << ",\"goal_acceptance_radius_m\":";
  appendJsonNumber(output, goal_acceptance_radius_m);
  output << ",\"goal_endpoint_adjusted\":"
         << (goal_endpoint_adjusted ? "true" : "false");
  output << ",\"robot_grid_state\":" << static_cast<int>(robot_grid);
  output << ",\"robot_inflated_grid_state\":" << static_cast<int>(robot_inflated_grid);
  output << ",\"target_grid_state\":" << static_cast<int>(target_grid);
  output << ",\"target_inflated_grid_state\":" << static_cast<int>(target_inflated_grid);
  output << ",\"certificate_world_generation\":" << snapshot.certificate.validated_world.generation;
  output << ",\"certificate_world_revision\":" << snapshot.certificate.validated_world.revision;
  output << ",\"certificate_world_stamp_ns\":" << snapshot.certificate.validated_world.observation_stamp_ns;
  output << ",\"duration_s\":";
  appendJsonNumber(output, trajectory.duration_s);
  output << ",\"backup_available\":" << (snapshot.backup_available ? "true" : "false");
  output << ",\"backup_start_time_s\":";
  appendJsonNumber(output, snapshot.backup_start_time_s);
  output << ",\"nominal_path\":";
  appendJsonPath(output, nominal_path);
  output << ",\"backup_path\":";
  appendJsonPath(output, backup_path);
  output << ",\"emergency_path\":";
  appendJsonPath(output, emergency_path);
  output << '}';
  return output.str();
}

}  // namespace

NavigationRuntimeNode::NavigationRuntimeNode(const rclcpp::NodeOptions& options)
    : NavigationRuntimeNode(options, NavigationRuntimeDependencies{}) {}

NavigationRuntimeNode::NavigationRuntimeNode(
    const rclcpp::NodeOptions& options, NavigationRuntimeDependencies dependencies)
    : rclcpp::Node("navigation_runtime_node", options),
      command_sampler_(command_bundle_store_),
      mapping_lifecycle_observer_(std::move(dependencies.lifecycle_observer)) {
  registered_scan_topic_ = declare_parameter(
      "navigation_runtime.registered_scan_topic", std::string("/lio/mapping_observation"));
  propagated_odometry_topic_ = declare_parameter(
      "navigation_runtime.propagated_odometry_topic", std::string("/lio/odometry_propagated"));
  goal_topic_ = declare_parameter("navigation_runtime.goal_topic", std::string("/navigation/goal"));
  status_topic_ = declare_parameter(
      "navigation_runtime.status_topic", std::string("/navigation/mode_status"));
  command_topic_ = declare_parameter(
      "navigation_runtime.command_topic", std::string("/navigation/navigation_command"));
  planning_frame_ = declare_parameter("navigation_runtime.planning_frame", std::string("lio_odom"));
  body_frame_id_ = declare_parameter("navigation_runtime.body_frame_id", std::string("base_link"));
  deployment_profile_ = declare_parameter(
      "navigation_runtime.deployment_profile", std::string("sitl"));
  planner_rate_hz_ = declare_parameter("navigation_runtime.planner_rate_hz", 5.0);
  command_rate_hz_ = declare_parameter("navigation_runtime.command_rate_hz", 50.0);
  mapping_snapshot_publication_period_s_ = declare_parameter(
      "navigation_runtime.mapping_snapshot_publication_period_s", 0.10);
  data_freshness_window_s_ = declare_parameter(
      "navigation_runtime.data_freshness_window_s", 0.5);
  command_stream_timeout_s_ = declare_parameter(
      "navigation_runtime.command_stream_timeout_s", 0.10);
  planner_watchdog_timeout_s_ = declare_parameter(
      "navigation_runtime.planner_watchdog_timeout_s", 1.0);
  stopped_recovery_timeout_s_ = declare_parameter(
      "navigation_runtime.stopped_recovery_timeout_s", 5.0);
  const auto max_plan_from_rest_failures = declare_parameter(
      "navigation_runtime.max_plan_from_rest_failures", std::int64_t{3});
  planner_config_path_ = declare_parameter("navigation_runtime.config_path", std::string{});
  const auto mission_file =
      declare_parameter("navigation_runtime.mission_file", std::string{});
  if (planner_config_path_.empty()) {
    planner_config_path_ = ament_index_cpp::get_package_share_directory("navigation_runtime") +
                         "/config/planner.yaml";
  }

  if (!std::filesystem::exists(planner_config_path_)) {
    throw std::runtime_error("planner backend config does not exist: " + planner_config_path_);
  }
  if (deployment_profile_ != "sitl" && deployment_profile_ != "hardware") {
    throw std::invalid_argument(
        "navigation_runtime.deployment_profile must be 'sitl' or 'hardware'");
  }
  if (deployment_profile_ == "hardware") {
    throw std::invalid_argument(
        "hardware planner backend runtime is blocked until an immutable sensor-visibility "
        "certificate and its runtime verifier are implemented");
  }
  if (!std::isfinite(planner_rate_hz_) ||
      std::abs(planner_rate_hz_ - 5.0) > 1.0e-9) {
    throw std::invalid_argument("navigation_runtime.planner_rate_hz must equal 5 Hz");
  }
  const auto planning_period = ratePeriodNanoseconds(planner_rate_hz_);
  const auto command_period = ratePeriodNanoseconds(command_rate_hz_);
  if (!planning_period || !command_period ||
      std::chrono::duration_cast<std::chrono::microseconds>(*planning_period).count() <= 0) {
    throw std::invalid_argument(
        "navigation_runtime planner/command rates must produce positive representable periods");
  }
  if (!std::isfinite(command_rate_hz_) || command_rate_hz_ <= 0.0 ||
      !std::isfinite(mapping_snapshot_publication_period_s_) ||
      mapping_snapshot_publication_period_s_ <= 0.0 ||
      !std::isfinite(data_freshness_window_s_) || data_freshness_window_s_ <= 0.0 ||
      !std::isfinite(command_stream_timeout_s_) ||
      std::abs(command_stream_timeout_s_ -
               navigation_planning::PlanningTimingContract::kCommandStreamTimeoutS) > 1.0e-9 ||
      !std::isfinite(planner_watchdog_timeout_s_) || planner_watchdog_timeout_s_ <= 0.0 ||
      !std::isfinite(stopped_recovery_timeout_s_) ||
      std::abs(stopped_recovery_timeout_s_ -
               navigation_planning::PlanningTimingContract::kStoppedRecoveryTimeoutS) > 1.0e-9 ||
      std::abs(command_rate_hz_ - 50.0) > 1.0e-9 ||
      std::abs(mapping_snapshot_publication_period_s_ -
               navigation_planning::PlanningTimingContract::kSnapshotPeriodS) > 1.0e-9) {
    throw std::invalid_argument(
        "planner backend timing and safety parameters must be positive");
  }
  if (mapping_snapshot_publication_period_s_ > data_freshness_window_s_ ||
      mapping_snapshot_publication_period_s_ > 1.0 / planner_rate_hz_) {
    throw std::invalid_argument(
        "navigation_runtime.mapping_snapshot_publication_period_s must not exceed "
        "the world freshness window or planner period");
  }
  const auto data_freshness_window_ns =
      navigation_common::secondsToNanoseconds(data_freshness_window_s_);
  const auto planner_watchdog_timeout_ns =
      navigation_common::secondsToNanoseconds(planner_watchdog_timeout_s_);
  const auto command_stream_timeout_ns =
      navigation_common::secondsToNanoseconds(command_stream_timeout_s_);
  if (!data_freshness_window_ns || *data_freshness_window_ns <= 0 ||
      !planner_watchdog_timeout_ns || *planner_watchdog_timeout_ns <= 0 ||
      !command_stream_timeout_ns || *command_stream_timeout_ns <= 0) {
    throw std::invalid_argument(
        "navigation_runtime timing values are too small or large for nanosecond precision");
  }
  data_freshness_window_ns_ = *data_freshness_window_ns;
  command_stream_timeout_ns_ = *command_stream_timeout_ns;
  planner_watchdog_timeout_ns_ = *planner_watchdog_timeout_ns;
  if (body_frame_id_.empty()) {
    throw std::invalid_argument("navigation_runtime.body_frame_id must not be empty");
  }
  if (max_plan_from_rest_failures <= 0 ||
      max_plan_from_rest_failures >
          static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) {
    throw std::invalid_argument(
        "navigation_runtime.max_plan_from_rest_failures must fit uint32 and be positive");
  }
  max_plan_from_rest_failures_ =
      static_cast<std::uint32_t>(max_plan_from_rest_failures);
  plan_from_rest_failure_budget_ =
      ConsecutiveFailureBudget(max_plan_from_rest_failures_);

  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/navigation/diagnostics", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
  std::optional<navigation_planning::DynamicLimits> mission_limits;
  if (!mission_file.empty()) {
    mission_limits = loadMissionDynamicLimits(mission_file, planning_frame_);
    RCLCPP_INFO(
        get_logger(),
        "Applying mission dynamics to planner backend before optimizer construction: "
        "velocity=%.3f acceleration=%.3f jerk=%.3f",
        mission_limits->max_velocity_mps,
        mission_limits->max_acceleration_mps2,
        mission_limits->max_jerk_mps3);
  }
  // Dynamics are immutable after construction. This process-local identity is
  // sufficient for request supersession and is never used as a certificate.
  dynamics_hash_ = static_cast<std::uint64_t>(std::hash<std::string>{}(
      planner_config_path_ + "\n" + mission_file));
  if (dynamics_hash_ == 0U) dynamics_hash_ = 1U;
  const auto ros_clock = get_clock();
  auto mapping_actor = std::make_shared<navigation_mapping::MappingActor>(
      planner_config_path_, [ros_clock] { return ros_clock->now().seconds(); },
      navigation_mapping::MappingFrameContract{planning_frame_, body_frame_id_},
      mapping_snapshot_publication_period_s_);
  const auto mapping_configuration = mapping_actor->configuration();
  if (mapping_configuration.callbacks_enabled ||
      mapping_configuration.raycasting_batch_update_size != 1) {
    throw std::invalid_argument(
        "navigation_runtime owns mapping callbacks and requires one mapping "
        "observation per raycast update");
  }
  if (planning_frame_ == "lio_odom" &&
      mapping_configuration.virtual_ground_ceiling_enabled) {
    throw std::invalid_argument(
        "absolute mapping virtual ground/ceiling planes are invalid in lio_odom; "
        "disable the virtual ground/ceiling planes in the mapping configuration");
  }
  const auto initial_snapshot = mapping_actor->initialSnapshot();
  world_snapshot_store_.publish(initial_snapshot.view);
  mapping_telemetry_ = std::make_shared<MappingTelemetry>();
  MappingTelemetrySnapshot initial_telemetry;
  initial_telemetry.snapshot_bytes = initial_snapshot.metrics.bytes;
  initial_telemetry.snapshot_owned_bytes = initial_snapshot.metrics.owned_bytes;
  initial_telemetry.snapshot_shared_metadata_bytes = initial_snapshot.metrics.shared_metadata_bytes;
  initial_telemetry.snapshot_live_count = initial_snapshot.metrics.live_count;
  initial_telemetry.snapshot_peak_live_count = initial_snapshot.metrics.peak_live_count;
  initial_telemetry.snapshot_live_owned_bytes = initial_snapshot.metrics.live_owned_bytes;
  initial_telemetry.snapshot_peak_live_owned_bytes = initial_snapshot.metrics.peak_live_owned_bytes;
  mapping_telemetry_->initialize(initial_telemetry);
  auto process_mapping = [this, mapping_actor, telemetry = mapping_telemetry_,
                          lifecycle_observer = mapping_lifecycle_observer_,
                          store = &world_snapshot_store_,
                          command_store = &command_bundle_store_,
                          ros_clock,
                          epoch_ready = &localization_epoch_ready_]
      (PendingRegisteredScan&& pending) mutable {
    const auto callback_started = std::chrono::steady_clock::now();
    try {
      if (!pending.message) {
        throw std::invalid_argument("mapping worker received a null RegisteredScan");
      }
      const auto decode_started = std::chrono::steady_clock::now();
      auto decoded = std::make_unique<navigation_mapping::PointCloud>();
      if (!decodeCloud(pending.message->points, *decoded)) {
        throw std::invalid_argument("mapping worker could not decode registered points");
      }
      std::unique_ptr<navigation_mapping::PointCloud> decoded_free_space;
      if (pending.message->visibility_observation_present) {
        decoded_free_space = std::make_unique<navigation_mapping::PointCloud>();
        if (!decodeCloud(
                pending.message->free_space_endpoints,
                *decoded_free_space, false)) {
          throw std::invalid_argument("mapping worker could not decode visibility endpoints");
        }
      }
      nav_msgs::msg::Odometry corrected;
      corrected.header = pending.message->header;
      corrected.child_frame_id = pending.message->body_frame_id;
      corrected.pose = pending.message->corrected_pose;
      navigation_mapping::MappingObservation observation{
          std::move(decoded), std::move(corrected), pending.localization_epoch,
          pending.scan_sequence, pending.stamp_ns,
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - decode_started).count(),
          nullptr};
      if (decoded_free_space) {
        observation.free_space_endpoints = std::move(decoded_free_space);
      }
      const auto result = mapping_actor->process(observation);
      if (lifecycle_observer) {
        lifecycle_observer->onMutableMapUpdated(observation.stamp_ns);
      }
      MappingTelemetrySnapshot next = telemetry->snapshot();
      next.map = result.diagnostics;
      next.last_update_attempt_stamp_ns = observation.stamp_ns;
      next.snapshot_export_us = result.snapshot_export_us;
      next.snapshot_export_mode = static_cast<std::uint64_t>(result.snapshot_export_mode);
      next.snapshot_full_export_reason =
          static_cast<std::uint64_t>(result.snapshot_full_export_reason);
      next.snapshot_export_base_cells = result.snapshot_export_base_cells;
      next.snapshot_export_inflated_cells = result.snapshot_export_inflated_cells;
      next.snapshot_patch_depth = result.snapshot_patch_depth;
      next.pointcloud_decode_us = observation.pointcloud_decode_us;
      next.world_snapshot_published = static_cast<bool>(result.snapshot);
      if (result.snapshot) {
        const auto expected_bundle = command_store->load();
        bool retain_validated_bundle = false;
        const auto terminal_generation = terminal_bundle_generation_.load(
            std::memory_order_acquire);
        const bool terminal_bundle_observed = expected_bundle &&
            terminal_generation != 0U &&
            expected_bundle->bundle_generation == terminal_generation;
        if (terminal_bundle_observed) {
          // The executable lease may have ended by the time this map callback
          // runs. An already observed terminal command needs one final safe
          // handover sample, not a revalidation of future trajectory time.
          // Keep this exception narrow: only the declared endpoint is sampled,
          // and it must still be finite and known-free in the new snapshot.
          const auto terminal_endpoint = expected_bundle->sampleAtDeclaredEnd();
          const auto terminal_state = terminal_endpoint
              ? result.snapshot->classify(
                    terminal_endpoint->position_world,
                    navigation_world_model::GridLayer::kInflated)
              : navigation_world_model::CellState::kOutOfMap;
          retain_validated_bundle = terminal_endpoint.has_value() &&
              terminal_endpoint->finished && terminal_endpoint->finite() &&
              navigation_world_model::isCellTraversable(
                  terminal_state,
                  navigation_world_model::UnknownPolicy::kRequireKnownFree);
          if (!retain_validated_bundle) {
            RCLCPP_WARN(
                this->get_logger(),
                "terminal command retention rejected generation=%lu state=%d endpoint=%d",
                static_cast<unsigned long>(expected_bundle->bundle_generation),
                static_cast<int>(terminal_state), terminal_endpoint.has_value() ? 1 : 0);
          }
        } else if (expected_bundle && planner_) {
          const auto committed_before = planner_->committedSnapshot();
          // The execution bundle is exported from this backend generation. Do
          // not require the backend's historical certificate to equal the
          // execution certificate: after the first transfer, the execution
          // bundle may already be certified on an intermediate world revision
          // while the planner still owns its original planning certificate.
          const bool backend_matches_bundle =
              committed_before.generation == expected_bundle->bundle_generation;
          if (backend_matches_bundle) {
            const auto validation = planner_->validateCommittedTrajectory(
                result.snapshot, ros_clock->now().seconds());
            if (validation.reused_unchanged_certificate) {
              ++next.command_revalidation_fast_path_count;
            } else {
              ++next.command_revalidation_full_count;
            }
            const auto committed_after = planner_->committedSnapshot();
            retain_validated_bundle = validation.valid &&
                committed_after.generation == expected_bundle->bundle_generation &&
                navigation_world_model::sameWorldSnapshotIdentity(
                    validation.validated_world, result.snapshot->identity());
            if (!validation.valid) {
              RCLCPP_WARN(
                  this->get_logger(),
                  "command recertification rejected generation=%lu on world revision=%lu "
                "at t=%.3f cell=%d position=(%.3f,%.3f,%.3f) "
                "failure_code=%d role=%d samples=%zu segments=%zu",
                  static_cast<unsigned long>(expected_bundle->bundle_generation),
                  static_cast<unsigned long>(result.snapshot->identity().revision),
                  validation.first_blocked_time_s,
                  validation.first_blocked_cell_state,
                  validation.first_blocked_position.x(),
                  validation.first_blocked_position.y(),
                  validation.first_blocked_position.z(),
                  validation.failure_code, validation.blocked_role,
                  validation.sample_count, validation.segment_count);
            }
          }
        }
        // The immutable world publication and the dependent execution
        // certificate transition share one gate.  A retained bundle is copied
        // with the new identity only when the exact pointer was validated on
        // this snapshot; otherwise it is cleared fail-closed.
        const bool finalized = store->publishAndFinalize(
            result.snapshot,
            [command_store, expected_bundle, retain_validated_bundle,
             identity = result.snapshot->identity(),
             refreshed_valid_until_ns = [&] {
               const auto now_ns = ros_clock->now().nanoseconds();
               return now_ns > std::numeric_limits<std::int64_t>::max() -
                          data_freshness_window_ns_
                          ? std::numeric_limits<std::int64_t>::max()
                          : now_ns + data_freshness_window_ns_;
             }()] {
              return command_store->publishWorldIdentity(
                  identity, expected_bundle, retain_validated_bundle,
                  refreshed_valid_until_ns);
            });
        if (!finalized) {
          command_store->invalidate();
          next.map_update_us = result.map_update_us;
          next.mapping_callback_total_us =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - callback_started).count();
          telemetry->recordUpdate(std::move(next));
          RCLCPP_ERROR(
              this->get_logger(),
              "world snapshot publication could not finalize the execution certificate; "
              "mapping worker will fail-stop because the new world was not published");
          throw std::runtime_error(
              "world snapshot publication could not finalize its execution certificate");
        }
        if (expected_bundle && retain_validated_bundle) {
          const auto recertified_bundle = command_store->load();
          const auto now_ns = ros_clock->now().nanoseconds();
          const auto suspended_generation =
              world_freshness_suspended_bundle_generation_.load(
                  std::memory_order_acquire);
          if (recertified_bundle && worldFreshnessSuspendedCommandMayResume(
                  suspended_generation,
                  recertified_bundle->bundle_generation,
                  recertified_bundle->localization_epoch,
                  recertified_bundle->goal_epoch,
                  active_localization_epoch_.load(std::memory_order_acquire),
                  active_goal_epoch_.load(std::memory_order_acquire),
                  recertified_bundle->valid_until_ns,
                  now_ns,
                  recertified_bundle->valid(),
                  planner_failure_latched_.load(std::memory_order_acquire),
                  command_execution_lease_failure_latch_.allowsCommandExposure())) {
            std::lock_guard<std::mutex> command_lock(
                command_execution_lease_failure_latch_.transitionMutex());
            if (worldFreshnessSuspendedCommandMayResume(
                    world_freshness_suspended_bundle_generation_.load(
                        std::memory_order_acquire),
                    recertified_bundle->bundle_generation,
                    recertified_bundle->localization_epoch,
                    recertified_bundle->goal_epoch,
                    active_localization_epoch_.load(std::memory_order_acquire),
                    active_goal_epoch_.load(std::memory_order_acquire),
                    recertified_bundle->valid_until_ns,
                    now_ns,
                    recertified_bundle->valid(),
                    planner_failure_latched_.load(std::memory_order_acquire),
                    command_execution_lease_failure_latch_.allowsCommandExposure())) {
              planner_command_available_.store(true, std::memory_order_release);
              command_goal_epoch_.store(
                  recertified_bundle->goal_epoch, std::memory_order_release);
              safety_suffix_active_.store(
                  world_freshness_suspended_safety_suffix_active_.load(
                      std::memory_order_acquire),
                  std::memory_order_release);
              world_freshness_suspended_bundle_generation_.store(
                  0U, std::memory_order_release);
              world_freshness_suspended_safety_suffix_active_.store(
                  false, std::memory_order_release);
              ++world_freshness_command_recovery_count_;
              RCLCPP_INFO(
                  this->get_logger(),
                  "fresh world recertified suspended command generation=%lu; "
                  "command publication resumed without replacing the bundle",
                  static_cast<unsigned long>(recertified_bundle->bundle_generation));
            }
          }
        }
        if (expected_bundle && !retain_validated_bundle &&
            expected_bundle->localization_epoch ==
                active_localization_epoch_.load(std::memory_order_acquire) &&
            expected_bundle->goal_epoch == active_goal_epoch_.load(std::memory_order_acquire)) {
          // The latest immutable map invalidated the currently exposed
          // command. This is recoverable only through a new measured-state
          // PlanFromRest solve; allowing the next timer tick to enter
          // ReplanOnce with no committed bundle turns a map change into an
          // unconditional emergency result and prevents recovery.
          {
            std::lock_guard<std::mutex> lock(input_mutex_);
            if (active_goal_ &&
                active_goal_->request_id == expected_bundle->request_id) {
              restart_from_rest_ = false;
              hot_goal_transition_ = false;
              skip_replan_once_ = false;
            }
          }
          {
            std::lock_guard<std::mutex> command_lock(
                command_execution_lease_failure_latch_.transitionMutex());
            planner_command_available_.store(false);
            planner_failure_latched_.store(true);
            safety_suffix_active_.store(false);
            execution_recovery_state_.store(
                ExecutionRecoveryState::kPx4Hold, std::memory_order_release);
            command_goal_epoch_.store(0U);
            trajectory_finished_.store(false);
            trajectory_reaches_goal_.store(false);
            terminal_bundle_generation_.store(0U);
          }
          RCLCPP_WARN(
              this->get_logger(),
              "world revision=%lu invalidated active command generation=%lu; "
              "entering PX4 Hold because no latest-world-certified brake remains",
              static_cast<unsigned long>(result.world_revision),
              static_cast<unsigned long>(expected_bundle->bundle_generation));
        }
        epoch_ready->store(true, std::memory_order_release);
        next.world_generation = result.world_generation;
        next.world_revision = result.world_revision;
        next.observation_stamp_ns = result.observation_stamp_ns;
        next.snapshot_bytes = result.snapshot_metrics.bytes;
        next.snapshot_owned_bytes = result.snapshot_metrics.owned_bytes;
        next.snapshot_shared_metadata_bytes = result.snapshot_metrics.shared_metadata_bytes;
        next.snapshot_live_count = result.snapshot_metrics.live_count;
        next.snapshot_peak_live_count = result.snapshot_metrics.peak_live_count;
        next.snapshot_live_owned_bytes = result.snapshot_metrics.live_owned_bytes;
        next.snapshot_peak_live_owned_bytes = result.snapshot_metrics.peak_live_owned_bytes;
      }
      next.map_update_us = result.map_update_us;
      // Include backend processing, snapshot construction, certificate
      // revalidation and publication finalization in the callback envelope.
      next.mapping_callback_total_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - callback_started).count();
      telemetry->recordUpdate(std::move(next));
    } catch (...) {
      telemetry->recordCallbackFailure(
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - callback_started).count());
      throw;
    }
  };
  auto validate_mapping = [ros_clock, telemetry = mapping_telemetry_,
                           active_epoch = &active_localization_epoch_,
                           maximum_age_ns = data_freshness_window_ns_](
      const PendingRegisteredScan& pending) {
    if (!pending.message) {
      telemetry->recordDiscard(false, false, true);
      return false;
    }
    const auto& pose = pending.message->corrected_pose.pose;
    const Eigen::Quaterniond q{
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    const auto freshness = navigation_execution::classifyTimestampFreshness(
        ros_clock->now().nanoseconds(), pending.stamp_ns, maximum_age_ns);
    const bool invalid = pending.message->points.width == 0U ||
                         pending.localization_epoch !=
                             active_epoch->load(std::memory_order_acquire) ||
                         pending.localization_epoch == 0U ||
                         pending.stamp_ns <= 0 ||
                         navigation_common::rosTimeToNanoseconds(
                             pending.message->header.stamp).value_or(0) !=
                             pending.stamp_ns ||
                         !std::isfinite(pose.position.x) ||
                         !std::isfinite(pose.position.y) ||
                         !std::isfinite(pose.position.z) || !q.coeffs().allFinite() ||
                         !finiteNonzeroQuaternion(q) ||
                         freshness == navigation_execution::TimestampFreshness::INVALID;
    const bool stale = freshness == navigation_execution::TimestampFreshness::STALE;
    const bool future = freshness == navigation_execution::TimestampFreshness::FUTURE;
    if (invalid || stale || future) telemetry->recordDiscard(stale, future, invalid);
    return !invalid && !stale && !future;
  };
  mapping_worker_ = std::make_unique<navigation_mapping::MappingWorker<PendingRegisteredScan>>(
      observation_accounting_, std::move(process_mapping),
      mappingFailStop, std::move(validate_mapping),
      [publisher = diagnostics_publisher_, ros_clock,
       telemetry = mapping_telemetry_, accounting = &observation_accounting_,
       freshness_rejection_count = &world_snapshot_freshness_rejection_count_]() {
        const auto mapping = telemetry->snapshot();
        const auto lifecycle = accounting->snapshot();
        diagnostic_msgs::msg::DiagnosticArray diagnostics;
        diagnostics.header.stamp = ros_clock->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "navigation_mapping/world_model";
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = std::string("PUBLISHED_") +
            std::string(navigation_mapping::worldUpdateOutcomeName(mapping.map.update_outcome));
        const auto add_value = [&status](const std::string& key, std::uint64_t value) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = std::to_string(value);
          status.values.push_back(std::move(item));
        };
        const auto add_duration = [&status](const std::string& key, std::int64_t value) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = std::to_string(std::max<std::int64_t>(0, value));
          status.values.push_back(std::move(item));
        };
        const auto add_text = [&status](const std::string& key, const std::string& value) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = value;
          status.values.push_back(std::move(item));
        };
        const auto& map = mapping.map;
        add_value("world_generation", mapping.world_generation);
        add_value("world_revision", mapping.world_revision);
        add_value("observation_stamp_ns", mapping.observation_stamp_ns);
        add_value("last_update_attempt_stamp_ns", mapping.last_update_attempt_stamp_ns);
        add_value("world_snapshot_published", mapping.world_snapshot_published ? 1U : 0U);
        add_value("world_snapshot_published_count", mapping.world_snapshot_published_count);
        add_value("world_snapshot_deferred_count", mapping.world_snapshot_deferred_count);
        add_value("world_snapshot_full_export_count", mapping.world_snapshot_full_export_count);
        add_value("world_snapshot_patch_export_count", mapping.world_snapshot_patch_export_count);
        add_value("world_snapshot_export_mode", mapping.snapshot_export_mode);
        add_value("world_snapshot_full_export_reason", mapping.snapshot_full_export_reason);
        add_value("world_snapshot_export_base_cells", mapping.snapshot_export_base_cells);
        add_value("world_snapshot_export_inflated_cells",
                  mapping.snapshot_export_inflated_cells);
        add_value("world_snapshot_patch_depth", mapping.snapshot_patch_depth);
        addObservationAccountingValues(status, lifecycle);
        add_text("mapping_update_outcome",
                 std::string(navigation_mapping::worldUpdateOutcomeName(map.update_outcome)));
        add_value("mapping_outcome_updated_count", mapping.outcome_updated);
        add_value("mapping_outcome_accumulated_count", mapping.outcome_accumulated);
        add_value("mapping_outcome_slide_only_count", mapping.outcome_slide_only);
        add_value("mapping_outcome_empty_cloud_count", mapping.outcome_empty_cloud);
        add_value("mapping_outcome_callback_owned_count", mapping.outcome_callback_owned);
        add_value("mapping_outcome_below_ground_count", mapping.outcome_below_ground);
        add_value("mapping_outcome_above_ceiling_count", mapping.outcome_above_ceiling);
        add_value("command_revalidation_fast_path_count",
                  mapping.command_revalidation_fast_path_count);
        add_value("command_revalidation_full_count",
                  mapping.command_revalidation_full_count);
        add_value("observation_accounting_valid",
                  lifecycle.allInvariantsHold() ? 1U : 0U);
        add_value("observation_accounting_violation_count", lifecycle.violation_count);
        add_value("mapping_input_point_count", map.endpoint_count);
        add_value("mapping_free_space_endpoint_count",
                  map.free_space_endpoint_count);
        add_value("mapping_free_space_processed_count",
                  map.free_space_processed_count);
        add_value("mapping_free_space_skipped_count",
                  map.free_space_skipped_count);
        add_value("mapping_allocated_voxel_count", map.allocated_voxel_count);
        add_value("mapping_body_neighborhood_cells_cleared",
                  map.body_neighborhood_cells_cleared);
        add_value("world_snapshot_freshness_rejection_count",
                  freshness_rejection_count->load());
        add_value("world_snapshot_bytes", mapping.snapshot_bytes);
        add_value("world_snapshot_owned_bytes", mapping.snapshot_owned_bytes);
        add_value("world_snapshot_shared_metadata_bytes",
                  mapping.snapshot_shared_metadata_bytes);
        add_value("world_snapshot_live_count", mapping.snapshot_live_count);
        add_value("world_snapshot_peak_live_count", mapping.snapshot_peak_live_count);
        add_value("world_snapshot_live_owned_bytes", mapping.snapshot_live_owned_bytes);
        add_value("world_snapshot_peak_live_owned_bytes", mapping.snapshot_peak_live_owned_bytes);
        add_duration("ros_pointcloud_decode_us", mapping.pointcloud_decode_us);
        add_duration("mapping_raycast_us", map.raycast_us);
        add_duration("mapping_probability_update_us", map.probability_update_us);
        add_duration("mapping_inflation_us", map.inflation_us);
        add_duration("mapping_slide_us", map.slide_us);
        add_duration("mapping_total_update_us", map.map_update_us);
        add_duration("world_snapshot_export_us", mapping.snapshot_export_us);
        add_duration("mapping_callback_total_us", mapping.mapping_callback_total_us);
        diagnostics.status.push_back(std::move(status));
        publisher->publish(diagnostics);
      });
  auto planner = std::make_unique<navigation_planning_backend::PlannerFacade>(
      planner_config_path_, world_snapshot_store_.load().view, mission_limits,
      world_snapshot_store_, [this]() { return now().seconds(); });
  planner_ = planner.get();
  const double planner_period_s = 1.0 / planner_rate_hz_;
  const double solve_deadline_s = planner_->solveDeadlineSeconds();
  if (!std::isfinite(solve_deadline_s) || solve_deadline_s <= 0.0 ||
      solve_deadline_s >= planner_period_s) {
    throw std::invalid_argument(
        "navigation_runtime.planner_rate_hz must leave a complete timer period "
        "for planner.solve_deadline_s");
  }
  planning_worker_ = std::make_unique<
      PlanningWorker<navigation_planning_backend::PlannerFacade>>(
      std::move(planner), [this](std::exception_ptr failure) {
        planner_failure_latched_.store(true, std::memory_order_release);
        planner_command_available_.store(false, std::memory_order_release);
        try {
          if (failure) std::rethrow_exception(failure);
        } catch (const std::exception& error) {
          RCLCPP_ERROR(get_logger(), "planning worker failed: %s", error.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "planning worker failed with unknown exception");
        }
      });
  mapping_worker_->setStrictlyIncreasingOrderKey(
      [](const PendingRegisteredScan& pending) { return pending.stamp_ns; });
  mapping_worker_->start();
  planning_worker_->start();

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  propagated_state_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions propagated_state_options;
  propagated_state_options.callback_group = propagated_state_callback_group_;
  registered_scan_subscription_ = create_subscription<
      navigation_contracts::msg::RegisteredScan>(
      registered_scan_topic_, qos,
      std::bind(&NavigationRuntimeNode::onRegisteredScan, this, std::placeholders::_1));
  estimator_health_subscription_ = create_subscription<
      navigation_contracts::msg::EstimatorHealth>(
      "/lio/health", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&NavigationRuntimeNode::onEstimatorHealth, this, std::placeholders::_1));
  propagated_odometry_subscription_ = create_subscription<
      navigation_contracts::msg::PropagatedOdometry>(
      propagated_odometry_topic_, qos,
      std::bind(&NavigationRuntimeNode::onPropagatedOdometry, this, std::placeholders::_1),
      propagated_state_options);
  goal_subscription_ = create_subscription<navigation_contracts::msg::NavigationGoal>(
      goal_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&NavigationRuntimeNode::onGoal, this, std::placeholders::_1));
  status_subscription_ =
      create_subscription<navigation_contracts::msg::NavigationModeStatus>(
          status_topic_,
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
          std::bind(&NavigationRuntimeNode::onModeStatus, this, std::placeholders::_1));
  command_publisher_ = create_publisher<navigation_contracts::msg::NavigationCommand>(
      command_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  end_to_end_samples_ms_.reserve(256);

  // The timer is a non-blocking scheduler. Mutable solve execution is owned by
  // the bounded planning worker; command sampling reads only the immutable
  // committed bundle store.
  planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  command_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  planning_period_us_ =
      std::chrono::duration_cast<std::chrono::microseconds>(*planning_period).count();
  planning_timer_ = create_wall_timer(
      *planning_period, std::bind(&NavigationRuntimeNode::schedulePlanningCycle, this),
      planning_callback_group_);
  command_timer_ = create_wall_timer(
      *command_period, std::bind(&NavigationRuntimeNode::publishCommand, this),
      command_callback_group_);
  RCLCPP_INFO(get_logger(),
              "planner backend runtime ready: registered_scan=%s propagated_odom=%s goal=%s "
              "output=%s planner=%.1fHz command=%.1fHz snapshot=%.2fs "
              "planning_input_source_receive=%.2fs command_stream=%.2fs "
              "stopped_recovery=%.1fs",
              registered_scan_topic_.c_str(),
              propagated_odometry_topic_.c_str(), goal_topic_.c_str(),
              command_topic_.c_str(), planner_rate_hz_, command_rate_hz_,
              mapping_snapshot_publication_period_s_, data_freshness_window_s_,
              command_stream_timeout_s_, stopped_recovery_timeout_s_);
}

NavigationRuntimeNode::~NavigationRuntimeNode() {
  accepting_observations_.store(false);
  if (planning_timer_) planning_timer_->cancel();
  if (command_timer_) command_timer_->cancel();
  if (planning_worker_) planning_worker_->shutdown();
  planner_ = nullptr;
  registered_scan_subscription_.reset();
  estimator_health_subscription_.reset();
  propagated_odometry_subscription_.reset();
  goal_subscription_.reset();
  status_subscription_.reset();
  if (mapping_worker_) mapping_worker_->shutdown();
  if (mapping_lifecycle_observer_) {
    mapping_lifecycle_observer_->onShutdownComplete(observation_accounting_.snapshot());
  }
}

bool NavigationRuntimeNode::decodeCloud(const sensor_msgs::msg::PointCloud2& message,
                                      navigation_mapping::PointCloud& output,
                                      const bool require_nonempty) {
  constexpr std::uint64_t kMaximumPointCount = 2'000'000U;
  const std::uint64_t row_payload_bytes =
      static_cast<std::uint64_t>(message.point_step) * message.width;
  const std::uint64_t storage_bytes =
      static_cast<std::uint64_t>(message.row_step) * message.height;
  const std::uint64_t point_count =
      static_cast<std::uint64_t>(message.width) * message.height;
  // PointCloud2Iterator reads native-endian float storage.  This transport
  // boundary has no byte-swap implementation, so accepting a big-endian
  // cloud would turn valid bytes into plausible but incorrect geometry.
  if (message.is_bigendian || !hasFloatField(message, "x") ||
      !hasFloatField(message, "y") ||
      !hasFloatField(message, "z") || message.point_step == 0U ||
      message.row_step < row_payload_bytes || storage_bytes > message.data.size() ||
      point_count > kMaximumPointCount) {
    return false;
  }
  output.clear();
  output.reserve(static_cast<std::size_t>(point_count));
  try {
    sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
    if (hasFloatField(message, "intensity")) {
      sensor_msgs::PointCloud2ConstIterator<float> intensity(message, "intensity");
      for (; x != x.end(); ++x, ++y, ++z, ++intensity) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z) &&
            std::isfinite(*intensity)) {
          output.emplace_back(*x, *y, *z, *intensity);
        }
      }
    } else {
      // PointCloud2 does not require an intensity channel.  Zero is the
      // neutral product value for an absent channel; when the channel exists,
      // preserve its measured value so the configured backend filter remains
      // meaningful.  The mapping contract rejects non-finite values below.
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          output.emplace_back(*x, *y, *z, 0.0F);
        }
      }
    }
  } catch (const std::exception&) {
    output.clear();
    return false;
  }
  return !require_nonempty || !output.empty();
}

void NavigationRuntimeNode::resetForLocalizationEpochLocked(
    const std::uint64_t localization_epoch) {
  if (localization_epoch == 0U) return;
  const auto current = active_localization_epoch_.load(std::memory_order_acquire);
  if (localization_epoch <= current) return;

  localization_epoch_ready_.store(false, std::memory_order_release);
  // Match the goal transition ordering: cancel the solve before changing the
  // epoch exposed to the planning callback, drain the mapping barrier, then
  // clear command exposure under the execution transition lock.
  if (planning_worker_) planning_worker_->cancelActive();
  if (mapping_worker_) mapping_worker_->reset();
  active_localization_epoch_.store(localization_epoch, std::memory_order_release);
  last_propagated_state_stamp_ns_.store(0, std::memory_order_release);
  last_propagated_state_sequence_.store(0U, std::memory_order_release);
  {
    std::lock_guard<std::mutex> derivative_lock(propagated_derivative_mutex_);
    propagated_derivative_estimator_.reset();
  }
  execution_state_store_.resetForLocalizationEpoch(localization_epoch);
  command_bundle_store_.invalidate();
  last_registered_scan_epoch_.store(localization_epoch, std::memory_order_release);
  last_registered_scan_sequence_.store(0U, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    const auto next_goal_epoch = advanceMonotonicId(active_goal_epoch_);
    if (next_goal_epoch) {
      (void)command_bundle_store_.setActiveGoalEpoch(*next_goal_epoch, false);
    } else {
      active_goal_.reset();
      command_bundle_store_.invalidate();
      RCLCPP_ERROR(get_logger(), "active goal epoch exhausted during localization reset");
    }
    new_goal_ = active_goal_.has_value();
    hot_goal_transition_ = false;
    restart_from_rest_ = false;
    skip_replan_once_ = false;
    trajectory_finished_.store(false);
    trajectory_reaches_goal_.store(false);
    terminal_bundle_generation_.store(0U);
  }
  {
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    planner_command_available_.store(false);
    // Keep the command path fail-closed until the new epoch publishes its
    // first valid world snapshot. This evidence barrier is recoverable: the
    // active goal may be planned again after that snapshot arrives.
    planner_failure_latched_.store(false);
    command_execution_lease_failure_latch_.resetForNewGoalWithinTransition();
    safety_suffix_active_.store(false);
    execution_recovery_state_.store(
        ExecutionRecoveryState::kInitialHold, std::memory_order_release);
    command_goal_epoch_.store(0U);
  }
  RCLCPP_WARN(get_logger(),
              "Localization epoch changed to %lu; old mapping and command state invalidated",
              static_cast<unsigned long>(localization_epoch));
}

void NavigationRuntimeNode::onRegisteredScan(
    const navigation_contracts::msg::RegisteredScan::ConstSharedPtr& message) {
  if (!message) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  const auto observation_stamp_ns = navigation_common::rosTimeToNanoseconds(
      message->header.stamp).value_or(0);
  const auto& free_space = message->free_space_endpoints;
  const bool has_free_space_payload =
      free_space.width != 0U || free_space.height != 0U || !free_space.data.empty();
  const bool visibility_contract_valid = message->visibility_observation_present
      ? message->visibility_no_return_count ==
            free_space.width * free_space.height &&
            message->visibility_no_return_count <=
                message->visibility_source_ray_count &&
            message->visibility_stamp_skew_ns == 0
      : !has_free_space_payload && message->visibility_source_ray_count == 0U &&
            message->visibility_no_return_count == 0U &&
            message->visibility_stamp_skew_ns == 0;
  if (!accepting_observations_.load(std::memory_order_acquire) ||
      message->localization_epoch == 0U ||
      message->scan_sequence == 0U ||
      message->header.frame_id != planning_frame_ ||
      message->points.header.frame_id != message->header.frame_id ||
      !visibility_contract_valid ||
      navigation_common::rosTimeToNanoseconds(message->points.header.stamp).value_or(0) !=
          observation_stamp_ns ||
      (message->visibility_observation_present &&
       (free_space.header.frame_id != message->header.frame_id ||
        navigation_common::rosTimeToNanoseconds(free_space.header.stamp).value_or(0) !=
            observation_stamp_ns)) ||
      message->body_frame_id != body_frame_id_) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  const auto& pose = message->corrected_pose.pose;
  const Eigen::Quaterniond quaternion{
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
  if (observation_stamp_ns <= 0 ||
      !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !finiteNonzeroQuaternion(quaternion)) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  std::lock_guard<std::mutex> transition_lock(localization_transition_mutex_);
  const auto active_epoch = active_localization_epoch_.load(std::memory_order_acquire);
  if (message->localization_epoch < active_epoch) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  if (message->localization_epoch > active_epoch) {
    resetForLocalizationEpochLocked(message->localization_epoch);
  }
  const auto sequence_epoch = last_registered_scan_epoch_.load(std::memory_order_acquire);
  const auto previous_scan_sequence =
      last_registered_scan_sequence_.load(std::memory_order_acquire);
  if (sequence_epoch == message->localization_epoch &&
      message->scan_sequence <= previous_scan_sequence) {
    observation_accounting_.recordRejectedBeforeInbox();
    return;
  }
  last_registered_scan_epoch_.store(message->localization_epoch, std::memory_order_release);
  last_registered_scan_sequence_.store(message->scan_sequence, std::memory_order_release);
  observation_accounting_.recordAcceptedToInbox();
  (void)mapping_worker_->submitFromWaiting(PendingRegisteredScan{
      message, observation_stamp_ns, message->localization_epoch,
      message->scan_sequence});
}

void NavigationRuntimeNode::onEstimatorHealth(
    const navigation_contracts::msg::EstimatorHealth::ConstSharedPtr& message) {
  if (!message || message->localization_epoch == 0U) return;
  std::lock_guard<std::mutex> transition_lock(localization_transition_mutex_);
  const auto active_epoch = active_localization_epoch_.load(std::memory_order_acquire);
  if (message->localization_epoch <= active_epoch) return;
  // Health announces the public-frame transition early; command exposure stays
  // disabled until a RegisteredScan of this epoch is accepted and mapped.
  resetForLocalizationEpochLocked(message->localization_epoch);
}

void NavigationRuntimeNode::onPropagatedOdometry(
    const navigation_contracts::msg::PropagatedOdometry::ConstSharedPtr& message) {
  if (!message || message->localization_epoch == 0U || message->sequence == 0U) {
    ++invalid_execution_state_count_;
    return;
  }
  const auto& odometry = message->odometry;
  if (odometry.header.frame_id != planning_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Dropping propagated odometry frame '%s'; expected '%s'",
                         odometry.header.frame_id.c_str(), planning_frame_.c_str());
    return;
  }
  std::lock_guard<std::mutex> transition_lock(localization_transition_mutex_);
  const auto active_epoch = active_localization_epoch_.load(std::memory_order_acquire);
  if (message->localization_epoch > active_epoch) {
    resetForLocalizationEpochLocked(message->localization_epoch);
  }
  if (message->localization_epoch !=
          active_localization_epoch_.load(std::memory_order_acquire) ||
      message->sequence <=
          last_propagated_state_sequence_.load(std::memory_order_acquire)) {
    ++invalid_execution_state_count_;
    return;
  }
  const auto stamp_ns =
      navigation_common::rosTimeToNanoseconds(odometry.header.stamp).value_or(0);
  const auto previous_stamp_ns =
      last_propagated_state_stamp_ns_.load(std::memory_order_acquire);
  if (stamp_ns <= 0 || !propagatedOdometryFinite(odometry) ||
      (previous_stamp_ns > 0 && stamp_ns <= previous_stamp_ns)) {
    ++invalid_execution_state_count_;
    return;
  }
  const Eigen::Quaterniond orientation{
      odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
      odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z};
  if (!finiteNonzeroQuaternion(orientation) ||
      odometry.child_frame_id != body_frame_id_) {
    ++invalid_execution_state_count_;
    return;
  }
  navigation_planning::KinematicState typed_state;
  typed_state.position_world = Eigen::Vector3d{
      odometry.pose.pose.position.x, odometry.pose.pose.position.y,
      odometry.pose.pose.position.z};
  const double orientation_scale = orientation.coeffs().cwiseAbs().maxCoeff();
  typed_state.orientation_world_body = Eigen::Quaterniond(
      orientation.w() / orientation_scale, orientation.x() / orientation_scale,
      orientation.y() / orientation_scale, orientation.z() / orientation_scale).normalized();
  const auto rotation = typed_state.orientation_world_body.toRotationMatrix();
  typed_state.yaw_rad = std::atan2(rotation(1, 0), rotation(0, 0));
  typed_state.velocity_world = typed_state.orientation_world_body * Eigen::Vector3d{
      odometry.twist.twist.linear.x, odometry.twist.twist.linear.y,
      odometry.twist.twist.linear.z};
  {
    std::lock_guard<std::mutex> derivative_lock(propagated_derivative_mutex_);
    const auto derivative = propagated_derivative_estimator_.update(
        stamp_ns, message->localization_epoch, typed_state.velocity_world);
    typed_state.acceleration_world = derivative.acceleration_world;
    typed_state.jerk_world = derivative.jerk_world;
    typed_state.acceleration_estimated = derivative.acceleration_estimated;
    typed_state.jerk_estimated = derivative.jerk_estimated;
  }
  typed_state.source_stamp_ns = stamp_ns;
  typed_state.receive_stamp_ns = navigation_common::steadyClockNowNanoseconds();
  typed_state.localization_epoch = message->localization_epoch;
  typed_state.world_frame_id = odometry.header.frame_id;
  typed_state.body_frame_id = odometry.child_frame_id;
  if (execution_state_store_.publish(std::move(typed_state))) {
    last_propagated_state_stamp_ns_.store(stamp_ns, std::memory_order_release);
    last_propagated_state_sequence_.store(message->sequence, std::memory_order_release);
  }
}

void NavigationRuntimeNode::onGoal(const navigation_contracts::msg::NavigationGoal::ConstSharedPtr& message) {
  if (!message) {
    RCLCPP_ERROR(get_logger(), "rejected null navigation goal");
    return;
  }
  const auto route = decodeRouteSnapshot(*message);
  const bool route_mirrors_valid =
      route.has_value() && routeSnapshotMatchesGoalMirrors(*route, *message);
  if (!route.has_value() || !route_mirrors_valid) {
    if (planning_worker_) planning_worker_->cancelActive();
    command_bundle_store_.invalidate();
    planner_command_available_.store(false);
    planner_failure_latched_.store(true);
    safety_suffix_active_.store(false);
    RCLCPP_ERROR(get_logger(),
                 "rejected navigation goal: %s (mission_len=%zu frame_len=%zu "
                 "route_mission_len=%zu route_frame_len=%zu request=%llu "
                 "route_request=%llu waypoints=%zu ids=%zu radii=%zu behaviors=%zu "
                 "active=%u measured_segment=%u)",
                 route.has_value() ? "immutable route mirrors are invalid"
                                   : "immutable route snapshot is invalid",
                 message->mission_id.size(), message->header.frame_id.size(),
                 message->route.mission_id.size(), message->route.frame_id.size(),
                 static_cast<unsigned long long>(message->request_id),
                 static_cast<unsigned long long>(message->route.request_id),
                 message->route.waypoint_positions.size(),
                 message->route.waypoint_ids.size(),
                 message->route.waypoint_acceptance_radii_m.size(),
                 message->route.waypoint_behaviors.size(),
                 message->route.active_waypoint_index,
                 message->route.measured_segment_index);
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  // A planner backend plan is owned by the mission waypoint identity.  The
  // continuation point is only look-ahead metadata; treating it as the
  // current goal makes a repeated waypoint publication look like a new
  // request (or, conversely, hides the actual waypoint transition).
  const bool same_checkpoint = active_goal_.has_value() &&
      active_goal_->mission_id == message->mission_id &&
      active_goal_->waypoint_index == message->waypoint_index;
  const bool same_logical_goal = same_checkpoint &&
      active_goal_->request_id == message->request_id;
  const bool reuse_completed_stop = same_checkpoint && !same_logical_goal &&
      message->behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP &&
      planner_command_available_.load() && trajectory_reaches_goal_.load() &&
      !planner_failure_latched_.load() && !safety_suffix_active_.load();
  const bool previous_safety_suffix_active = safety_suffix_active_.load();
  const bool can_hot_retarget = canHotRetargetAtWaypointTransition(
      same_logical_goal,
      active_goal_.has_value() &&
          active_goal_->behavior ==
              navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH,
      planner_command_available_.load(), planner_failure_latched_.load(),
      safety_suffix_active_.load());
  const auto previous_goal_epoch = active_goal_epoch_.load(std::memory_order_acquire);
  bool effective_hot_retarget = can_hot_retarget;
  if (!same_logical_goal) {
    // Cancel before exposing the new waypoint identity. The planner commit gate
    // guarantees that a solve for the previous waypoint cannot publish a new
    // candidate after this callback has invalidated it. A hot-retarget may keep
    // the current bundle only until a newer world identity invalidates it; the
    // execution store never relabels a stale-world certificate as current.
    if (planning_worker_) planning_worker_->cancelActive();
    const auto next_goal_epoch = advanceMonotonicId(active_goal_epoch_);
    if (!next_goal_epoch) {
      command_bundle_store_.invalidate();
      planner_command_available_.store(false);
      planner_failure_latched_.store(true);
      safety_suffix_active_.store(false);
      RCLCPP_ERROR(get_logger(), "rejected navigation goal: active goal epoch exhausted");
      return;
    }
    const bool retain_bundle = reuse_completed_stop || can_hot_retarget;
    if (!command_bundle_store_.setActiveGoalEpoch(*next_goal_epoch, retain_bundle)) {
      effective_hot_retarget = false;
    }
    if (effective_hot_retarget &&
        !command_bundle_store_.rebindRetainedBundle(
            *next_goal_epoch, message->request_id, previous_goal_epoch)) {
      // The physical command was not proven to belong to the exact previous
      // goal identity. Clear it and use the measured-state PlanFromRest path.
      effective_hot_retarget = false;
      command_bundle_store_.invalidate();
    }
  }
  active_goal_ = *message;
  if (reuse_completed_stop) {
    // The mission controller may republish a STOP checkpoint when one noisy
    // velocity sample temporarily breaks its hold confirmation. The completed
    // planner backend command already terminates at this same checkpoint and PX4 is
    // actively holding it; replacing it with a zero-distance PlanFromRest
    // creates a singular yaw problem and unnecessarily drops position hold.
    new_goal_ = false;
    hot_goal_transition_ = false;
    restart_from_rest_ = false;
    skip_replan_once_ = false;
    trajectory_finished_.store(true);
    {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      command_goal_epoch_.store(active_goal_epoch_.load());
    }
    return;
  }
  if (!same_logical_goal) {
    {
      // Global order is input_mutex_ -> execution transition. Command sampling
      // snapshots input and releases it before taking the transition lock.
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      command_execution_lease_failure_latch_.resetForNewGoalWithinTransition();
      planner_failure_latched_.store(false);
      safety_suffix_active_.store(
          effective_hot_retarget && previous_safety_suffix_active);
      if (effective_hot_retarget) {
        command_goal_epoch_.store(active_goal_epoch_.load());
      } else {
        planner_command_available_.store(false);
        command_goal_epoch_.store(0U);
      }
    }
    plan_from_rest_failure_budget_.reset();
    plan_from_rest_first_failure_steady_ns_ = 0;
    hot_goal_transition_ = effective_hot_retarget;
    new_goal_ = !effective_hot_retarget;
    restart_from_rest_ = false;
    skip_replan_once_ = false;
    trajectory_finished_.store(false);
    trajectory_reaches_goal_.store(false);
    terminal_bundle_generation_.store(0U);
    execution_recovery_state_.store(
        effective_hot_retarget ? ExecutionRecoveryState::kTrackMain
                               : ExecutionRecoveryState::kInitialHold,
        std::memory_order_release);
  }
}

void NavigationRuntimeNode::onModeStatus(
    const navigation_contracts::msg::NavigationModeStatus::ConstSharedPtr& message) {
  if (!message) {
    RCLCPP_ERROR(get_logger(), "rejected null navigation mode status");
    return;
  }
  if (message->state == navigation_contracts::msg::NavigationModeStatus::ACTIVE ||
      message->state == navigation_contracts::msg::NavigationModeStatus::BRAKING) {
    return;
  }
  std::lock_guard<std::mutex> lock(input_mutex_);
  if (!active_goal_ || active_goal_->mission_id != message->mission_id ||
      active_goal_->waypoint_index != message->waypoint_index ||
      active_goal_->request_id != message->request_id) {
    return;
  }
  RCLCPP_INFO(get_logger(),
              "Cancelling planner backend goal after terminal mission status state=%u reason=%u "
              "mission=%s waypoint=%u request=%lu",
              message->state, message->reason, message->mission_id.c_str(),
              message->waypoint_index, static_cast<unsigned long>(message->request_id));
  if (planning_worker_) planning_worker_->cancelActive();
  const auto next_goal_epoch = advanceMonotonicId(active_goal_epoch_);
  if (next_goal_epoch) {
    (void)command_bundle_store_.setActiveGoalEpoch(*next_goal_epoch, false);
  } else {
    command_bundle_store_.invalidate();
    RCLCPP_ERROR(get_logger(), "active goal epoch exhausted while clearing terminal goal");
  }
  active_goal_.reset();
  new_goal_ = false;
  hot_goal_transition_ = false;
  restart_from_rest_ = false;
  skip_replan_once_ = false;
  {
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    planner_command_available_.store(false);
    planner_failure_latched_.store(false);
    safety_suffix_active_.store(false);
    command_goal_epoch_.store(0U);
  }
  plan_from_rest_failure_budget_.reset();
  plan_from_rest_first_failure_steady_ns_ = 0;
  trajectory_finished_.store(false);
  trajectory_reaches_goal_.store(false);
  terminal_bundle_generation_.store(0U);
  execution_recovery_state_.store(
      ExecutionRecoveryState::kInitialHold, std::memory_order_release);
}

bool NavigationRuntimeNode::commitPlannerCandidate(
    const navigation_contracts::msg::NavigationGoal& goal,
    const std::uint64_t goal_epoch,
    const std::uint64_t localization_epoch,
    const std::int64_t now_ns,
    const PlanningKey& scheduled_key) {
  if (now_ns <= 0 || goal_epoch == 0U || localization_epoch == 0U ||
      goal.request_id == 0U) {
    return false;
  }
  const auto latest_bundle_before_commit = command_bundle_store_.load();
  const auto latest_world_before_commit = world_snapshot_store_.load();
  const auto latest_bundle_generation = latest_bundle_before_commit
      ? latest_bundle_before_commit->bundle_generation
      : 0U;
  bool route_identity_current = false;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    route_identity_current = active_goal_ &&
        active_goal_->request_id == scheduled_key.request_id &&
        active_goal_->route.route_revision == scheduled_key.route_revision;
  }
  if (!scheduled_key.valid() || !route_identity_current ||
      scheduled_key.localization_epoch != localization_epoch ||
      scheduled_key.goal_epoch != goal_epoch ||
      scheduled_key.request_id != goal.request_id ||
      active_localization_epoch_.load(std::memory_order_acquire) != localization_epoch ||
      active_goal_epoch_.load(std::memory_order_acquire) != goal_epoch ||
      latest_bundle_generation != scheduled_key.committed_bundle_generation ||
      !latest_world_before_commit ||
      latest_world_before_commit.identity.generation !=
          scheduled_key.pinned_world_generation) {
    planner_->discardCommandCandidate();
    RCLCPP_WARN(get_logger(),
                "execution boundary rejected stale planning result request=%lu "
                "route_revision=%lu committed_generation=%lu world_generation=%lu",
                static_cast<unsigned long>(scheduled_key.request_id),
                static_cast<unsigned long>(scheduled_key.route_revision),
                static_cast<unsigned long>(scheduled_key.committed_bundle_generation),
                static_cast<unsigned long>(scheduled_key.pinned_world_generation));
    return false;
  }
  const auto maximum_age_ns = data_freshness_window_ns_;
  if (maximum_age_ns <= 0 || now_ns > std::numeric_limits<std::int64_t>::max() - maximum_age_ns) {
    return false;
  }
  const auto candidate = planner_->exportCommandCandidate(
      localization_epoch, goal_epoch, goal.request_id, now_ns,
      now_ns + maximum_age_ns);
  if (!candidate) {
    RCLCPP_WARN(get_logger(),
                "execution boundary rejected candidate export mission=%s waypoint=%u "
                "request=%lu now_ns=%lld",
                goal.mission_id.c_str(), goal.waypoint_index,
                static_cast<unsigned long>(goal.request_id),
                static_cast<long long>(now_ns));
    return false;
  }
  const auto latest_world = world_snapshot_store_.load();
  const auto world_freshness = latest_world
      ? navigation_execution::classifyTimestampFreshness(
            now_ns, latest_world.identity.observation_stamp_ns, maximum_age_ns)
      : navigation_execution::TimestampFreshness::INVALID;
  if (world_freshness != navigation_execution::TimestampFreshness::VALID) {
    ++world_snapshot_freshness_rejection_count_;
    planner_->discardCommandCandidate();
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "execution boundary rejected candidate because the world snapshot is not fresh");
    return false;
  }
  const auto candidate_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      *candidate);
  if (!navigation_planning::candidateHasRequiredMainReserve(
          *candidate_ptr, now().seconds())) {
    planner_->discardCommandCandidate();
    RCLCPP_WARN(
        get_logger(),
        "execution boundary rejected candidate with insufficient MAIN reserve: "
        "kind=%d remaining=%.3f required=%.3f",
        static_cast<int>(candidate_ptr->kind),
        navigation_planning::remainingMainHorizonAtCommit(
            *candidate_ptr, now().seconds()),
        navigation_planning::PlanningTimingContract::kMinimumMainReserveS);
    return false;
  }
  // A solve can finish after the state that seeded it has been replaced. Do
  // not admit a candidate whose first executable sample is detached from the
  // latest propagated vehicle state. This reuses the same product geometric
  // envelope consumed by PX4; it is not a second tunable tracking parameter.
  const auto measured_state = execution_state_store_.load();
  const auto state_freshness = measured_state
      ? navigation_execution::classifyTimestampFreshness(
            now_ns, measured_state->state.source_stamp_ns, data_freshness_window_ns_)
      : navigation_execution::TimestampFreshness::INVALID;
  const auto candidate_sample = candidate_ptr->sample(now_ns);
  const bool candidate_awaiting_activation =
      candidate_ptr->valid() && now_ns < candidate_ptr->valid_from_ns;
  const double candidate_anchor_error_m = measured_state && candidate_sample
      ? (candidate_sample->position_world - measured_state->state.position_world).norm()
      : std::numeric_limits<double>::infinity();
  if (!measured_state || state_freshness != navigation_execution::TimestampFreshness::VALID ||
      !measured_state->state.finite() ||
      (!candidate_sample && !candidate_awaiting_activation) ||
      (!candidate_awaiting_activation &&
       (!std::isfinite(candidate_anchor_error_m) ||
        candidate_anchor_error_m > navigation_contracts::kCommandAnchorErrorLimitM))) {
    planner_->discardCommandCandidate();
    const char* rejection_reason = !measured_state
        ? "MISSING_EXECUTION_STATE"
        : state_freshness != navigation_execution::TimestampFreshness::VALID
        ? "EXECUTION_STATE_NOT_FRESH"
        : !measured_state->state.finite()
        ? "EXECUTION_STATE_NONFINITE"
        : !candidate_sample && !candidate_awaiting_activation
        ? "CANDIDATE_SAMPLE_INVALID"
        : !candidate_awaiting_activation && !std::isfinite(candidate_anchor_error_m)
        ? "ANCHOR_ERROR_NONFINITE"
        : "ANCHOR_ERROR_OVER_LIMIT";
    RCLCPP_WARN(get_logger(),
                "execution boundary rejected candidate reason=%s anchor_error=%.3f m "
                "state_present=%d state_finite=%d state_freshness=%d candidate_valid=%d "
                "candidate_sample=%d now_ns=%lld valid_from_ns=%lld valid_until_ns=%lld "
                "start_wall_time=%.9f duration=%.9f measured=(%.3f,%.3f,%.3f) "
                "candidate=(%.3f,%.3f,%.3f)",
                rejection_reason, candidate_anchor_error_m,
                measured_state ? 1 : 0,
                measured_state && measured_state->state.finite() ? 1 : 0,
                static_cast<int>(state_freshness), candidate_ptr->valid() ? 1 : 0,
                candidate_sample ? 1 : 0, static_cast<long long>(now_ns),
                static_cast<long long>(candidate_ptr->valid_from_ns),
                static_cast<long long>(candidate_ptr->valid_until_ns),
                candidate_ptr->start_wall_time_s, candidate_ptr->duration_s,
                measured_state ? measured_state->state.position_world.x() : 0.0,
                measured_state ? measured_state->state.position_world.y() : 0.0,
                measured_state ? measured_state->state.position_world.z() : 0.0,
                candidate_sample ? candidate_sample->position_world.x() : 0.0,
                candidate_sample ? candidate_sample->position_world.y() : 0.0,
                candidate_sample ? candidate_sample->position_world.z() : 0.0);
    return false;
  }
  const auto transaction_id = advanceMonotonicId(execution_transaction_id_);
  if (!transaction_id) {
    planner_->discardCommandCandidate();
    command_bundle_store_.invalidate();
    planner_command_available_.store(false);
    planner_failure_latched_.store(true);
    RCLCPP_ERROR(get_logger(), "execution transaction id exhausted");
    return false;
  }
  const navigation_execution::CommitToken token{
      candidate_ptr->world_identity, goal_epoch, *transaction_id};
  if (command_bundle_store_.tryCommit(token, candidate_ptr) !=
      navigation_execution::CommitDecision::kCommitted) {
    planner_->discardCommandCandidate();
    return false;
  }
  if (!planner_->acknowledgeCommandCandidate(candidate_ptr->bundle_generation)) {
    // The execution store is invalidated below, but the planner's staged
    // candidate is a separate ownership boundary. Clear it as well so an
    // ACK failure cannot leave an unexposed candidate to be mistaken for the
    // next solve's result.
    planner_->discardCommandCandidate();
    command_bundle_store_.invalidate();
    RCLCPP_ERROR(get_logger(),
                 "execution committed candidate generation=%lu but planner history ACK failed; "
                 "command exposure is cleared",
                 static_cast<unsigned long>(candidate_ptr->bundle_generation));
    return false;
  }
  world_freshness_suspended_bundle_generation_.store(0U, std::memory_order_release);
  world_freshness_suspended_safety_suffix_active_.store(
      false, std::memory_order_release);
  return true;
}

void NavigationRuntimeNode::suspendCommandForWorldFreshness() {
  std::lock_guard<std::mutex> command_lock(
      command_execution_lease_failure_latch_.transitionMutex());
  if (planner_command_available_.load(std::memory_order_acquire)) {
    const auto bundle = command_bundle_store_.load();
    if (bundle && bundle->localization_epoch ==
                      active_localization_epoch_.load(std::memory_order_acquire) &&
        bundle->goal_epoch == active_goal_epoch_.load(std::memory_order_acquire)) {
      world_freshness_suspended_bundle_generation_.store(
          bundle->bundle_generation, std::memory_order_release);
      world_freshness_suspended_safety_suffix_active_.store(
          safety_suffix_active_.load(std::memory_order_acquire),
          std::memory_order_release);
      ++world_freshness_command_suspend_count_;
    }
  }
  planner_command_available_.store(false, std::memory_order_release);
  planner_failure_latched_.store(false, std::memory_order_release);
  safety_suffix_active_.store(false, std::memory_order_release);
  command_goal_epoch_.store(0U, std::memory_order_release);
}

std::optional<PlanningKey> NavigationRuntimeNode::currentPlanningKey() {
  std::optional<navigation_contracts::msg::NavigationGoal> goal;
  bool safety_renewal = false;
  std::uint32_t recovery_level = 0U;
  std::uint64_t goal_epoch = 0U;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    goal = active_goal_;
    safety_renewal = restart_from_rest_ ||
        safety_suffix_active_.load(std::memory_order_acquire);
    recovery_level = plan_from_rest_failure_budget_.failureCount();
    goal_epoch = active_goal_epoch_.load(std::memory_order_acquire);
  }
  const auto execution = execution_state_store_.load();
  const auto world = world_snapshot_store_.load();
  if (!goal || !execution || !world || goal_epoch == 0U ||
      goal->route.route_revision == 0U) {
    return std::nullopt;
  }
  const auto bundle = command_bundle_store_.load();
  PlanningKey key;
  key.localization_epoch = active_localization_epoch_.load(std::memory_order_acquire);
  key.goal_epoch = goal_epoch;
  key.request_id = goal->request_id;
  key.route_revision = goal->route.route_revision;
  key.committed_bundle_generation = bundle ? bundle->bundle_generation : 0U;
  key.pinned_world_generation = world.identity.generation;
  key.pinned_world_revision = world.identity.revision;
  key.start_mode = bundle && planner_command_available_.load(std::memory_order_acquire) &&
          !safety_renewal
      ? PlanningStartMode::kCommittedFutureState
      : PlanningStartMode::kStoppedMeasuredState;
  key.anchor_stamp_ns = execution->state.source_stamp_ns;
  key.dynamics_hash = dynamics_hash_;
  key.recovery_level = recovery_level;
  if (!key.valid() || execution->state.localization_epoch != key.localization_epoch ||
      world.identity.localization_epoch != key.localization_epoch) {
    return std::nullopt;
  }
  return key;
}

void NavigationRuntimeNode::schedulePlanningCycle() {
  if (!accepting_observations_.load(std::memory_order_acquire) || !planning_worker_) return;
  const auto key = currentPlanningKey();
  if (!key) return;
  bool goal_transition = false;
  bool safety_renewal = false;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    goal_transition = new_goal_ || hot_goal_transition_;
    safety_renewal = restart_from_rest_ ||
        safety_suffix_active_.load(std::memory_order_acquire);
  }
  const auto priority = PlanningSupervisor::classifyPriority(
      false, false, goal_transition, safety_renewal);
  (void)planning_worker_->submit(
      *key, priority,
      [this, scheduled_key = *key](
          navigation_planning_backend::PlannerFacade&, std::stop_token stop) {
        if (stop.stop_requested()) return;
        const auto current = currentPlanningKey();
        if (!current || !PlanningSupervisor::resultStillCurrent(scheduled_key, *current)) {
          return;
        }
        runCycle(scheduled_key);
      });
}

void NavigationRuntimeNode::runCycle(const PlanningKey& scheduled_key) {
  const auto cycle_started = std::chrono::steady_clock::now();
  const auto cycle_started_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      cycle_started.time_since_epoch()).count();
  if (last_cycle_started_steady_ns_ > 0) {
    const auto interval_us = (cycle_started_ns - last_cycle_started_steady_ns_) / 1000;
    last_planning_scheduling_gap_us_ =
        std::max<std::int64_t>(0, interval_us - planning_period_us_);
  }
  last_cycle_started_steady_ns_ = cycle_started_ns;
  ++cycle_count_;
  std::shared_ptr<const navigation_execution::ExecutionStateLease> propagated_state;
  std::optional<navigation_contracts::msg::NavigationGoal> goal;
  bool new_goal = false;
  bool hot_goal_transition = false;
  bool restart_from_rest = false;
  std::uint64_t goal_epoch = 0;
  const auto input_lock_started = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    last_input_lock_wait_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - input_lock_started).count();
    goal = active_goal_;
    new_goal = new_goal_;
    hot_goal_transition = hot_goal_transition_;
    restart_from_rest = restart_from_rest_;
    goal_epoch = active_goal_epoch_.load();
  }
  // Do not acquire the state-store mutex while holding input_mutex_: the
  // odometry callback publishes to the store before taking input_mutex_.
  propagated_state = execution_state_store_.load();
  const auto now_ns = now().nanoseconds();
  const auto maximum_age_ns = data_freshness_window_ns_;
  const auto mapping = mapping_telemetry_->snapshot();
  const auto cloud_stamp_ns = mapping.observation_stamp_ns;
  const auto corrected_stamp_ns = mapping.observation_stamp_ns;

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_runtime/planner";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = goal ? "TRACKING" : "MAP_READY";
  const auto add_value = [&status](const std::string& key, std::uint64_t value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  };
  const auto add_signed_value = [&status](const std::string& key, std::int64_t value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  };
  const auto add_duration = [&status](const std::string& key, std::int64_t value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(std::max<std::int64_t>(0, value));
    status.values.push_back(std::move(item));
  };
  const auto add_double_value = [&status](const std::string& key, double value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::isfinite(value) ? std::to_string(value) : "invalid";
    status.values.push_back(std::move(item));
  };
  const auto& map_diagnostics = mapping.map;
  const auto accounting = observation_accounting_.snapshot();
  addObservationAccountingValues(status, accounting);
  add_value("cycle_count", cycle_count_);
  add_value("trajectory_publish_count", cycle_success_count_);
  add_value("optimizer_deferred_count", optimizer_deferred_count_);
  add_value("optimizer_renewal_due_count", optimizer_renewal_due_count_);
  add_value("execution_recovery_state", static_cast<std::uint64_t>(
      execution_recovery_state_.load(std::memory_order_acquire)));
  add_value("stale_input_count", stale_input_count_);
  add_value("stale_mapping_input_count",
            stale_mapping_input_count_.load() + mapping.discarded_stale);
  add_value("future_mapping_input_count",
            future_mapping_input_count_.load() + mapping.discarded_future);
  add_value("worker_discarded_stale_count", mapping.discarded_stale);
  add_value("worker_discarded_future_count", mapping.discarded_future);
  add_value("worker_discarded_invalid_count", mapping.discarded_invalid);
  add_value("stale_execution_state_count", stale_execution_state_count_);
  add_value("future_execution_state_count", future_execution_state_count_);
  add_value("invalid_corrected_pose_count",
            invalid_corrected_pose_count_.load() + mapping.discarded_invalid);
  add_value("invalid_execution_state_count", invalid_execution_state_count_);
  if (propagated_state) {
    add_value("execution_acceleration_estimated",
              propagated_state->state.acceleration_estimated ? 1U : 0U);
    add_value("execution_jerk_estimated",
              propagated_state->state.jerk_estimated ? 1U : 0U);
  }
  add_value("world_snapshot_freshness_rejection_count",
            world_snapshot_freshness_rejection_count_.load());
  add_value("world_freshness_command_suspend_count",
            world_freshness_command_suspend_count_.load());
  add_value("world_freshness_command_recovery_count",
            world_freshness_command_recovery_count_.load());
  add_value("world_freshness_suspended_bundle_generation",
            world_freshness_suspended_bundle_generation_.load());
  add_value("command_execution_lease_rejection_count",
            command_execution_lease_rejection_count_);
  add_value("command_execution_lease_terminal_latch_count",
            command_execution_lease_terminal_latch_count_);
  add_value("command_execution_lease_failed",
            command_execution_lease_failure_latch_.latched() ? 1U : 0U);
  add_value("command_execution_lease_reason", command_execution_lease_reason_.load());
  add_signed_value("command_execution_source_age_us",
                   command_execution_source_age_us_.load());
  add_signed_value("command_execution_receive_age_us",
                   command_execution_receive_age_us_.load());
  add_value("processing_exception_count", map_update_exception_count_);
  add_value("mapping_input_point_count", map_diagnostics.endpoint_count);
  add_value("mapping_free_space_endpoint_count",
            map_diagnostics.free_space_endpoint_count);
  add_value("mapping_free_space_processed_count",
            map_diagnostics.free_space_processed_count);
  add_value("mapping_free_space_skipped_count",
            map_diagnostics.free_space_skipped_count);
  add_value("mapping_allocated_voxel_count", map_diagnostics.allocated_voxel_count);
  add_value("mapping_body_neighborhood_cells_cleared",
            map_diagnostics.body_neighborhood_cells_cleared);
  add_value("localization_epoch",
            active_localization_epoch_.load(std::memory_order_acquire));
  add_value("localization_epoch_ready",
            localization_epoch_ready_.load(std::memory_order_acquire) ? 1U : 0U);
  add_value("registered_scan_epoch",
            last_registered_scan_epoch_.load(std::memory_order_acquire));
  add_value("registered_scan_sequence",
            last_registered_scan_sequence_.load(std::memory_order_acquire));
  add_value("world_generation", mapping.world_generation);
  add_value("world_revision", mapping.world_revision);
  add_value("world_snapshot_published", mapping.world_snapshot_published ? 1U : 0U);
  add_value("world_snapshot_published_count", mapping.world_snapshot_published_count);
  add_value("world_snapshot_deferred_count", mapping.world_snapshot_deferred_count);
  add_value("world_snapshot_full_export_count", mapping.world_snapshot_full_export_count);
  add_value("world_snapshot_patch_export_count", mapping.world_snapshot_patch_export_count);
  add_value("world_snapshot_export_mode", mapping.snapshot_export_mode);
  add_value("world_snapshot_full_export_reason", mapping.snapshot_full_export_reason);
  add_value("world_snapshot_export_base_cells", mapping.snapshot_export_base_cells);
  add_value("world_snapshot_export_inflated_cells", mapping.snapshot_export_inflated_cells);
  add_value("world_snapshot_patch_depth", mapping.snapshot_patch_depth);
  add_value("world_snapshot_bytes", mapping.snapshot_bytes);
  add_value("world_snapshot_owned_bytes", mapping.snapshot_owned_bytes);
  add_value("world_snapshot_shared_metadata_bytes", mapping.snapshot_shared_metadata_bytes);
  add_value("world_snapshot_live_count", mapping.snapshot_live_count);
  add_value("world_snapshot_peak_live_count", mapping.snapshot_peak_live_count);
  add_value("world_snapshot_live_owned_bytes", mapping.snapshot_live_owned_bytes);
  add_value("world_snapshot_peak_live_owned_bytes", mapping.snapshot_peak_live_owned_bytes);
  add_duration("mapping_input_lock_wait_us", last_input_lock_wait_us_);
  add_duration("command_transition_lock_wait_us",
               last_command_transition_lock_wait_us_.load(std::memory_order_acquire));
  add_duration("command_store_publish_us",
               last_command_store_publish_us_.load(std::memory_order_acquire));
  add_duration("command_transport_publish_us",
               last_publish_us_.load(std::memory_order_acquire));
  add_duration("planning_scheduling_gap_us", last_planning_scheduling_gap_us_);
  if (goal.has_value()) {
    const auto diagnostic_route = decodeRouteSnapshot(*goal);
    const bool diagnostic_route_valid = diagnostic_route.has_value() &&
        routeSnapshotMatchesGoalMirrors(*diagnostic_route, *goal);
    add_value("route_snapshot_valid", diagnostic_route_valid ? 1U : 0U);
    add_value("route_revision", goal->route.route_revision);
    add_value("route_active_waypoint_index", goal->route.active_waypoint_index);
    add_double_value("route_measured_progress_arc_m",
                     goal->route.measured_progress_arc_m);
    add_double_value("route_measured_projection_arc_m",
                     goal->route.measured_projection_arc_m);
    add_double_value("route_measured_lateral_error_m",
                     goal->route.measured_lateral_error_m);
  }
  diagnostics.status.push_back(std::move(status));
  diagnostics_publisher_->publish(diagnostics);

  // A localization reset has invalidated the previous world and command
  // epoch. Do not feed stale propagated state into planner backend until the mapping
  // worker has published the first snapshot of the new epoch.
  if (!localization_epoch_ready_.load(std::memory_order_acquire)) return;

  // A fresh propagated state cannot make an old map safe. Require the
  // published world snapshot to be fresh in the same ROS time domain before
  // solving or retaining any executable trajectory. Candidate admission and
  // command publication repeat this check because either boundary can race
  // the mapping worker.
  const auto latest_world = world_snapshot_store_.load();
  const auto world_freshness = latest_world
      ? navigation_execution::classifyTimestampFreshness(
            now_ns, latest_world.identity.observation_stamp_ns, maximum_age_ns)
      : navigation_execution::TimestampFreshness::INVALID;
  if (world_freshness != navigation_execution::TimestampFreshness::VALID) {
    ++world_snapshot_freshness_rejection_count_;
    if (planning_worker_) planning_worker_->cancelActive();
    // A stale world is a recoverable evidence gap, not a terminal planner
    // request failure. Preserve only the exact suspended generation so a
    // later fresh-world certificate can restore it without a new solve.
    suspendCommandForWorldFreshness();
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "planner cycle stopped because the world snapshot is not fresh");
    return;
  }

  // Mapping runs independently in its sole-owner worker. Planner execution state is independently
  // sourced from the latest propagated odometry and may be unavailable or
  // invalid without suppressing a corrected observation from ROG-Map.
  if (!propagated_state) return;
  const auto execution_stamp_ns = propagated_state->state.source_stamp_ns;
  const auto execution_freshness =
      navigation_execution::classifyTimestampFreshness(now_ns, execution_stamp_ns, maximum_age_ns);
  if (execution_freshness != navigation_execution::TimestampFreshness::VALID) {
    ++stale_input_count_;
    if (execution_freshness == navigation_execution::TimestampFreshness::STALE) ++stale_execution_state_count_;
    if (execution_freshness == navigation_execution::TimestampFreshness::FUTURE) ++future_execution_state_count_;
    if (execution_freshness == navigation_execution::TimestampFreshness::INVALID) ++invalid_execution_state_count_;
    return;
  }
  const auto& execution_state = propagated_state->state;
  if (!execution_state.finite() || !planner_->setState(execution_state)) {
    ++invalid_execution_state_count_;
    return;
  }

  // planner backend may produce a successful local trajectory ending at the current
  // sensing frontier rather than at the mission goal. Once it finishes,
  // restart PlanFromRest for the same logical goal so newly observed map
  // cells can extend the route. Mapping has already published this cycle.
  const bool completed_trajectory = trajectory_finished_.exchange(false);
  const double measured_speed_mps = execution_state.velocity_world.norm();
  auto recovery_state = execution_recovery_state_.load(std::memory_order_acquire);
  if (recovery_state == ExecutionRecoveryState::kTrackBackup ||
      recovery_state == ExecutionRecoveryState::kEmergencyBrake) {
    if (!completed_trajectory || !std::isfinite(measured_speed_mps) ||
        measured_speed_mps > 0.15) {
      // BACKUP and emergency are one-way while moving. Do not let a queued or
      // periodic nominal solve replace the braking command.
      return;
    }
    recovery_state = transitionExecutionRecovery(
        recovery_state, ExecutionRecoveryEvent::kCertifiedStopObserved);
    execution_recovery_state_.store(recovery_state, std::memory_order_release);
  } else if (recovery_state == ExecutionRecoveryState::kStoppedRecovery &&
             (!std::isfinite(measured_speed_mps) || measured_speed_mps > 0.15)) {
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      plan_from_rest_failure_budget_.reset();
      plan_from_rest_first_failure_steady_ns_ = 0;
    }
    execution_recovery_state_.store(
        transitionExecutionRecovery(
            recovery_state, ExecutionRecoveryEvent::kMotionObserved),
        std::memory_order_release);
    return;
  } else if (recovery_state == ExecutionRecoveryState::kPx4Hold) {
    return;
  }
  bool completed_trajectory_reaches_goal = trajectory_reaches_goal_.load();
  const bool pass_through_goal = goal &&
      goal->behavior ==
          navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH;
  const bool stop_goal = goal &&
      goal->behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP;
  if (completed_trajectory && goal) {
    // The completion flag can be cleared by a concurrent replan before this
    // callback observes the publisher's terminal sample. Recompute it from
    // the immutable committed candidate so a terminal command is not
    // mistaken for a frontier trajectory and restarted indefinitely.
    const auto committed_bundle = command_bundle_store_.load();
    const auto& target_message = plannerTarget(*goal);
    const Eigen::Vector3d target_position{
      pointFromMessage(target_message, 0),
      pointFromMessage(target_message, 1),
      pointFromMessage(target_message, 2)};
    const auto endpoint = committed_bundle &&
        committed_bundle->hasDeclaredEndpointMetadata()
        ? committed_bundle->sampleAtDeclaredEnd()
        : std::nullopt;
    const double completion_tolerance = goalCompletionTolerance(*goal);
    completed_trajectory_reaches_goal = endpoint.has_value() &&
        (endpoint->position_world - target_position).norm() <= completion_tolerance;
    trajectory_reaches_goal_.store(completed_trajectory_reaches_goal);
    terminal_bundle_generation_.store(
        completed_trajectory_reaches_goal && stop_goal && committed_bundle
            ? committed_bundle->bundle_generation
            : 0U,
        std::memory_order_release);
    RCLCPP_INFO(get_logger(),
                "planner backend trajectory completion observed reaches_goal=%d goal=(%.2f,%.2f,%.2f)",
                completed_trajectory_reaches_goal ? 1 : 0,
                pointFromMessage(plannerTarget(*goal), 0),
                pointFromMessage(plannerTarget(*goal), 1),
                pointFromMessage(plannerTarget(*goal), 2));
  }
  const bool continue_completed_pass_through =
      completedPassThroughRequiresContinuation(
          completed_trajectory, completed_trajectory_reaches_goal,
          pass_through_goal);
  if (completed_trajectory && goal && completed_trajectory_reaches_goal &&
      !continue_completed_pass_through) {
    return;
  }
  if (completed_trajectory && goal &&
      (!completed_trajectory_reaches_goal || continue_completed_pass_through)) {
    const auto& target_message = plannerTarget(*goal);
    const double dx = pointFromMessage(target_message, 0) - execution_state.position_world.x();
    const double dy = pointFromMessage(target_message, 1) - execution_state.position_world.y();
    const double dz = pointFromMessage(target_message, 2) - execution_state.position_world.z();
    if (continue_completed_pass_through ||
        std::sqrt(dx * dx + dy * dy + dz * dz) >
            goalCompletionTolerance(*goal)) {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
          active_goal_->waypoint_index == goal->waypoint_index &&
          active_goal_->request_id == goal->request_id) {
        restart_from_rest_ = true;
        hot_goal_transition_ = false;
        restart_from_rest = true;
        skip_replan_once_ = false;
      }
      RCLCPP_INFO(get_logger(),
                  "planner backend finite trajectory requires continuation; restarting PlanFromRest "
                  "pass_through=%d goal=(%.2f,%.2f,%.2f) vehicle=(%.2f,%.2f,%.2f)",
                  continue_completed_pass_through ? 1 : 0,
                  pointFromMessage(target_message, 0), pointFromMessage(target_message, 1),
                  pointFromMessage(target_message, 2), execution_state.position_world.x(),
                  execution_state.position_world.y(), execution_state.position_world.z());
    }
  }

  if (!goal) return;
  const auto route_snapshot = decodeRouteSnapshot(*goal);
  if (!route_snapshot.has_value() ||
      !routeSnapshotMatchesGoalMirrors(*route_snapshot, *goal)) {
    if (planning_worker_) planning_worker_->cancelActive();
    command_bundle_store_.invalidate();
    planner_command_available_.store(false);
    planner_failure_latched_.store(true);
    safety_suffix_active_.store(false);
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "planner cycle rejected an invalid immutable route snapshot");
    return;
  }
  // A terminal bundle already observed by the command publisher is a bounded
  // endpoint hold, not a frontier trajectory.  Do not start another
  // PlanFromRest while that exact bundle is waiting for mission acceptance.
  if (terminalHoldIsPending(
          planner_command_available_.load(std::memory_order_acquire),
          trajectory_reaches_goal_.load(std::memory_order_acquire),
          stop_goal,
          terminal_bundle_generation_.load(std::memory_order_acquire))) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
        active_goal_->waypoint_index == goal->waypoint_index &&
        active_goal_->request_id == goal->request_id) {
      restart_from_rest_ = false;
      hot_goal_transition_ = false;
    }
    return;
  }
  // A terminal planner failure remains terminal for this request until the
  // mission controller acknowledges it and cancels the goal. Without this
  // gate the next planning timer could perform a fourth solve and overwrite
  // the fail-closed state with a newly discovered frontier trajectory.
  if (planner_failure_latched_.load()) return;
  // While the command publisher drains a committed safety suffix, keep
  // replanning from the current vehicle state.  A validated main trajectory
  // may replace the suffix and recover the mission; failed solves leave the
  // frozen suffix untouched.  Stopping planner callbacks here would make
  // recovery impossible by construction and force every transient hot-replan
  // miss to end in hold/handover.

  // Planner execution state is independently owned by propagated odometry;
  // ROG-Map's corrected scan-epoch pose is mapping-only.
  // Always retain the current mission checkpoint as planner backend's geometric target.
  // PASS_THROUGH additionally supplies next_target as a bounded terminal
  // tangent; it is never used as a replacement geometric endpoint.
  const auto& active_route_waypoint =
      route_snapshot->waypoints[route_snapshot->active_waypoint_index];
  const Eigen::Vector3d target = active_route_waypoint.position_enu;
  const auto planner_started = std::chrono::steady_clock::now();
  // This is the internal planner backend FSM boundary: each mission waypoint
  // enters PlanFromRest once. Later timer ticks retain a latest-world-certified
  // MAIN until its planner-owned backup transition is close enough that one
  // scheduler period, one complete solve deadline, and the future hot-splice
  // interval must be reserved. Map recertification and command sampling remain
  // independent of this expensive optimizer renewal policy.
  const bool plan_from_rest = new_goal || restart_from_rest;
  const auto transition_bundle = command_bundle_store_.load();
  const double transition_elapsed_s = transition_bundle
      ? now().seconds() - transition_bundle->start_wall_time_s
      : std::numeric_limits<double>::quiet_NaN();
  const double planning_interval_s = static_cast<double>(planning_period_us_) * 1.0e-6;
  const auto transition_sample = transition_bundle
      ? transition_bundle->sample(now_ns)
      : std::nullopt;
  const double transition_anchor_error_m = transition_sample
      ? (transition_sample->position_world - execution_state.position_world).norm()
      : std::numeric_limits<double>::infinity();
  const auto transition_role = transition_sample
      ? transition_sample->role
      : transition_bundle ? transition_bundle->role
                          : navigation_planning::CandidateRole::kEmergency;
  const bool anchor_recovery_due = commandAnchorRecoveryDue(
      planner_command_available_.load(std::memory_order_acquire), transition_role,
      transition_anchor_error_m,
      navigation_contracts::kCommandAnchorErrorLimitM);
  const bool command_lease_recovery_due = transition_bundle && commandLeaseRenewalDue(
      planner_command_available_.load(std::memory_order_acquire), now_ns,
      transition_bundle->valid_until_ns,
      planning_interval_s + planner_->solveDeadlineSeconds());
  // Anchor pressure, lease renewal and hot retargeting may schedule a solve,
  // but every moving nominal renewal is anchored to committed future PVAJ.
  // Exceeding the certificate is handled by the one-shot emergency path after
  // retained validation; it never authorizes measured-state nominal planning.
  const bool anchor_renewal_replan = anchor_recovery_due && !plan_from_rest;
  // A short execution lease is an exposure/renewal boundary, not evidence
  // that the executing trajectory has lost its measured-state anchor. Renew
  // it with the normal continuous ReplanOnce path. Treating this as
  // PlanFromRest repeatedly re-seeds from the transient measured velocity
  // before the previous trajectory can turn toward its goal.
  const bool lease_renewal_replan = command_lease_recovery_due &&
      !plan_from_rest && !hot_goal_transition && !anchor_renewal_replan;
  const bool plan_from_rest_with_transition = plan_from_rest;
  const bool replan_for_new_goal = hotRetargetUsesCommittedFutureState(
      hot_goal_transition,
      planner_command_available_.load(std::memory_order_acquire),
      transition_role, transition_anchor_error_m,
      navigation_contracts::kCommandAnchorErrorLimitM);
  if (new_goal) {
    // MissionController has invalidated the previous waypoint already. Do
    // not publish that waypoint while PlanFromRest runs.
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    planner_command_available_.store(false);
    planner_failure_latched_.store(false);
    trajectory_reaches_goal_.store(false);
    terminal_bundle_generation_.store(0U);
  }
  if (!plan_from_rest_with_transition && !replan_for_new_goal && skip_replan_once_) {
    skip_replan_once_ = false;
    return;
  }
  const bool forced_transition = plan_from_rest_with_transition ||
      lease_renewal_replan || replan_for_new_goal || anchor_renewal_replan;
  const auto renewal_decision = classifyPlannerRenewal(
      forced_transition,
      planner_command_available_.load(std::memory_order_acquire),
      safety_suffix_active_.load(std::memory_order_acquire),
      transition_sample ? transition_sample->role
                        : transition_bundle ? transition_bundle->role
                                            : navigation_planning::CandidateRole::kEmergency,
      transition_bundle && transition_sample &&
          transition_bundle->hasTrajectoryMetadata(),
      transition_elapsed_s,
      transition_bundle ? transition_bundle->backup_start_time_s
                        : std::numeric_limits<double>::quiet_NaN(),
      planner_->solveDeadlineSeconds(), planner_->replanForwardSeconds(),
      planning_interval_s);
  if (!renewal_decision.run_optimizer) {
    ++optimizer_deferred_count_;
    return;
  }
  if (renewal_decision.reason == PlannerRenewalReason::kRenewalDue) {
    ++optimizer_renewal_due_count_;
  }
  const double recovery_scale = plan_from_rest_with_transition
      ? plannerRecoveryVelocityScale(plan_from_rest_failure_budget_.failureCount())
      : 1.0;
  planner_->setRecoveryVelocityScale(recovery_scale);
  navigation_planning::PlannerStatus result = navigation_planning::PlannerStatus::kFailed;
  const auto solve_generation_value = advanceMonotonicId(planner_solve_generation_);
  if (!solve_generation_value) {
    command_bundle_store_.invalidate();
    planner_command_available_.store(false);
    planner_failure_latched_.store(true);
    RCLCPP_ERROR(get_logger(), "planner solve generation exhausted");
    return;
  }
  const std::uint64_t solve_generation = *solve_generation_value;
  const std::uint64_t localization_epoch_at_solve =
      active_localization_epoch_.load(std::memory_order_acquire);
  const std::uint64_t committed_generation_before_solve =
      planner_->committedGeneration();
  const auto pinned_world = world_snapshot_store_.load();
  if (!pinned_world) {
    RCLCPP_ERROR(get_logger(), "planner backend cannot solve without a published WorldModel snapshot");
    return;
  }
  planner_->setWorldModelView(pinned_world.view);
  planner_->setGoalAcceptanceRadius(active_route_waypoint.acceptance_radius_m);
  if (!planner_->setRouteSnapshot(*route_snapshot)) {
    planner_->cancelActiveSolve();
    command_bundle_store_.invalidate();
    planner_command_available_.store(false);
    planner_failure_latched_.store(true);
    RCLCPP_ERROR(get_logger(), "planner rejected the immutable route snapshot");
    return;
  }
  planner_->setCommandIdentity(
      localization_epoch_at_solve, goal_epoch, goal->request_id);
  // Reset diagnostic-only optimizer evidence so a solve that bypasses EXP
  // cannot inherit retry metrics from the previous planning generation.
  planner_->resetOptimizationDiagnostics();
  planner_solve_started_steady_ns_.store(navigation_common::steadyClockNowNanoseconds());
  active_planner_solve_generation_.store(solve_generation);
  planner_->resetSolveCancellation();
  const auto solve_started_ros_ns = now().nanoseconds();
  const double execution_age_at_solve_ms =
      executionStateAgeMs(solve_started_ros_ns, execution_stamp_ns);
  try {
    result = plan_from_rest_with_transition
                 ? planner_->planFromRest(target, 0.0, true)
                 : planner_->replanOnce(target, 0.0, replan_for_new_goal);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "planner backend planner exception: %s", error.what());
    result = navigation_planning::PlannerStatus::kEmergency;
  }
  std::uint64_t expected_active_generation = solve_generation;
  active_planner_solve_generation_.compare_exchange_strong(
      expected_active_generation, 0U);
  last_planner_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - planner_started).count();
  if (timed_out_planner_solve_generation_.load() == solve_generation) {
    RCLCPP_ERROR(get_logger(),
                 "Discarding planner backend solve generation=%lu after planner watchdog timeout",
                 static_cast<unsigned long>(solve_generation));
    planner_->discardCommandCandidate();
    return;
  }
  if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
      active_localization_epoch_.load(std::memory_order_acquire) !=
          localization_epoch_at_solve) {
    RCLCPP_WARN(get_logger(),
                "Discarding planner backend solve generation=%lu after localization epoch transition",
                static_cast<unsigned long>(solve_generation));
    planner_->discardCommandCandidate();
    return;
  }
  const auto planner_diagnostics = planner_->diagnostics();
  const auto& exp_diagnostics = planner_diagnostics.optimization;
  const auto& backup_diagnostics = planner_diagnostics.backup_certificate;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    // A terminal mission status or a newer waypoint may arrive while the
    // optimizer is running. The completed solve is then stale: never expose
    // its internally committed trajectory to the command publisher.
    if (!active_goal_ || active_goal_->mission_id != goal->mission_id ||
        active_goal_->waypoint_index != goal->waypoint_index ||
        active_goal_->request_id != goal->request_id) {
      planner_->discardCommandCandidate();
      return;
    }
  }
  // The backend stages a certified candidate; the execution store commits it
  // once below.  Its planning-history generation intentionally changes only
  // after the execution commit ACK, so it is not evidence of a ready command.
  const bool solve_committed_new_generation = planner_->hasStagedCommandCandidate();
  const auto disposition = classifyPlannerResult(
      result, plan_from_rest_with_transition,
      planner_command_available_.load(),
      solve_committed_new_generation);
  if (disposition != PlannerResultDisposition::CommandReady) {
    // A staged candidate is private planner state until the execution store
    // commits it. Every other disposition is discard-only, including an
    // emergency candidate that cannot pass the normal command boundary.
    planner_->discardCommandCandidate();
  }
  if (disposition == PlannerResultDisposition::FailClosed) {
    planner_failure_latched_.store(true);
    trajectory_reaches_goal_.store(false);
    terminal_bundle_generation_.store(0U);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "planner backend planning failed (%d)", static_cast<int>(result));
  }
  if (disposition == PlannerResultDisposition::RestartFromRest) {
    planner_failure_latched_.store(false);
    trajectory_reaches_goal_.store(false);
    terminal_bundle_generation_.store(0U);
    trajectory_finished_.store(true);
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
        active_goal_->waypoint_index == goal->waypoint_index &&
        active_goal_->request_id == goal->request_id) {
      restart_from_rest_ = true;
      hot_goal_transition_ = false;
    }
    RCLCPP_INFO(get_logger(),
                "planner backend local trajectory boundary reached; scheduling PlanFromRest");
  }
  if (disposition == PlannerResultDisposition::RetryFromRest) {
    const bool terminal_hold_pending = terminalHoldIsPending(
        planner_command_available_.load(std::memory_order_acquire),
        trajectory_reaches_goal_.load(std::memory_order_acquire),
        stop_goal,
        terminal_bundle_generation_.load(std::memory_order_acquire));
    if (!terminal_hold_pending) {
      trajectory_reaches_goal_.store(false);
      terminal_bundle_generation_.store(0U);
    }
    bool failure_budget_exhausted = false;
    std::uint32_t failure_count = 0U;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
          active_goal_->waypoint_index == goal->waypoint_index &&
          active_goal_->request_id == goal->request_id) {
        const auto failure_now_ns = navigation_common::steadyClockNowNanoseconds();
        if (plan_from_rest_first_failure_steady_ns_ == 0) {
          plan_from_rest_first_failure_steady_ns_ = failure_now_ns;
        }
        plan_from_rest_failure_budget_.recordFailure();
        const double failure_window_s = static_cast<double>(
            static_cast<long double>(failure_now_ns) -
            static_cast<long double>(plan_from_rest_first_failure_steady_ns_)) * 1.0e-9;
        const auto timeout_state = execution_recovery_state_.load(
            std::memory_order_acquire);
        failure_budget_exhausted = stoppedPlanningTimeoutMayFailClosed(
            timeout_state,
            std::isfinite(measured_speed_mps) && measured_speed_mps <=
                navigation_planning::PlanningTimingContract::kStationarySpeedMps,
            failure_window_s, stopped_recovery_timeout_s_);
        failure_count = plan_from_rest_failure_budget_.failureCount();
      }
    }
    planner_failure_latched_.store(failure_budget_exhausted);
    if (failure_budget_exhausted) {
      RCLCPP_ERROR(get_logger(),
                   "planner backend PlanFromRest failed %u consecutive times; fail-closed for "
                   "mission=%s waypoint=%u",
                   failure_count, goal->mission_id.c_str(), goal->waypoint_index);
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "planner backend PlanFromRest transient failure (%d); retry %u/%u",
          static_cast<int>(result),
          failure_count, max_plan_from_rest_failures_);
    }
  }
  if (disposition == PlannerResultDisposition::RetainCommittedCommand ||
      disposition == PlannerResultDisposition::ValidateRetainedCommand) {
    const bool validate_without_new_commit =
        disposition == PlannerResultDisposition::ValidateRetainedCommand;
    const auto committed_bundle = command_bundle_store_.load();
    const bool committed = committed_bundle &&
        committed_bundle->hasTrajectoryMetadata() &&
        committed_bundle->localization_epoch == localization_epoch_at_solve &&
        committed_bundle->goal_epoch == goal_epoch;
    const bool backup_available = committed && committed_bundle->backup_available;
    const double backup_start_s = committed
        ? committed_bundle->backup_start_time_s : 0.0;

    const double elapsed_s = committed
                                 ? now().seconds() - committed_bundle->start_wall_time_s
                                 : std::numeric_limits<double>::infinity();
    const double total_duration_s = committed
        ? committed_bundle->duration_s : 0.0;
    const double clamped_elapsed_s =
        std::clamp(elapsed_s, 0.0, std::max(0.0, total_duration_s));
    const auto sampleCommittedBundle =
        [&](const double trajectory_time_s,
            navigation_planning::TrajectoryPoint& output) {
          if (!committed) return false;
          const auto stamp_ns = navigation_common::secondsSumToNanoseconds(
              committed_bundle->start_wall_time_s, trajectory_time_s);
          if (!stamp_ns) return false;
          const auto sample = committed_bundle->sample(*stamp_ns);
          if (!sample) return false;
          output = *sample;
          return true;
        };
    navigation_planning::TrajectoryPoint command_anchor_sample;
    const bool command_anchor_valid = sampleCommittedBundle(
        clamped_elapsed_s, command_anchor_sample);
    const Eigen::Vector3d command_anchor = command_anchor_valid
        ? command_anchor_sample.position_world : Eigen::Vector3d::Zero();
    // ReplanOnce may run for more than a second while command publication and
    // vehicle motion continue concurrently. The planner state captured before
    // that solve is therefore stale by construction. Re-read the immutable
    // execution lease after the solve and apply the same dual-clock contract
    // used by command publication; a stale receive must not rescue a retained
    // or emergency command.
    const auto retained_execution_state = execution_state_store_.load();
    const auto retained_state_freshness = retained_execution_state
        ? navigation_contracts::evaluateExecutionStateFreshness(
              now().nanoseconds(), retained_execution_state->state.source_stamp_ns,
              navigation_common::steadyClockNowNanoseconds(),
              retained_execution_state->state.receive_stamp_ns,
              data_freshness_window_s_)
        : navigation_contracts::ExecutionStateFreshness{};
    Eigen::Vector3d current_vehicle_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_vehicle_velocity = Eigen::Vector3d::Zero();
    const bool fresh_vehicle_state = retained_execution_state &&
                                     retained_execution_state->state.finite() &&
                                     retained_state_freshness.valid();
    const double latest_vehicle_state_age_s = retained_execution_state
        ? retained_state_freshness.source_age_ms * 1.0e-3
        : std::numeric_limits<double>::infinity();
    if (fresh_vehicle_state) {
      current_vehicle_position = retained_execution_state->state.position_world;
      current_vehicle_velocity = retained_execution_state->state.velocity_world;
    }
    const double anchor_error_m = !command_anchor_valid || !fresh_vehicle_state
                                      ? std::numeric_limits<double>::infinity()
                                      : (command_anchor - current_vehicle_position).norm();
    bool sampled_path_clear = committed;
    double first_blocked_sample_s = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d first_blocked_sample = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    navigation_world_model::CellState first_blocked_grid =
        navigation_world_model::CellState::kUnknown;
    if (sampled_path_clear) {
      const auto latest_world = world_snapshot_store_.latest();
      const auto validation = latest_world
          ? planner_->validateCommittedTrajectory(latest_world.view, now().seconds())
          : navigation_planning::TrajectoryValidationResult{};
      sampled_path_clear = validation.valid;
      if (!sampled_path_clear) {
        first_blocked_sample_s = std::isfinite(validation.first_blocked_time_s)
            ? validation.first_blocked_time_s : clamped_elapsed_s;
        first_blocked_sample = validation.first_blocked_position;
        if (!first_blocked_sample.allFinite()) {
          navigation_planning::TrajectoryPoint blocked_sample;
          if (sampleCommittedBundle(std::clamp(
                  first_blocked_sample_s, 0.0, total_duration_s), blocked_sample)) {
            first_blocked_sample = blocked_sample.position_world;
          }
        }
        const auto state = latest_world
            ? latest_world.view->classify(
                  first_blocked_sample,
                  navigation_world_model::GridLayer::kInflated)
            : navigation_world_model::CellState::kOutOfMap;
        first_blocked_grid = state;
      }
    }
    // If replanning fails after the main-to-backup switch, the usable safety
    // suffix starts at the current command anchor, not in the past.
    const double safety_transition_s = backup_available
                                           ? std::max(backup_start_s, clamped_elapsed_s)
                                           : clamped_elapsed_s;
    const double retained_tracking_limit_m = retainedCommandTrackingLimit(
        planner_->trackingErrorBudgetMeters(),
        navigation_contracts::kCommandAnchorErrorLimitM);
    const double relative_anchor_speed_mps =
        command_anchor_valid && fresh_vehicle_state
            ? (command_anchor_sample.velocity_world - current_vehicle_velocity).norm()
            : std::numeric_limits<double>::quiet_NaN();
    const double projected_anchor_error_m =
        projectedRetainedAnchorErrorUpperBound(
            anchor_error_m, relative_anchor_speed_mps,
            static_cast<double>(planning_period_us_) * 1.0e-6);
    bool use_safety_suffix = committedSafetySuffixIsUsable(
        backup_available, elapsed_s, total_duration_s,
        safety_transition_s,
        projected_anchor_error_m, retained_tracking_limit_m, sampled_path_clear);
    // A measured-state PlanFromRest attempt may fail while the currently
    // executing bundle is still a fresh, continuously certified bridge. Keep
    // that bridge alive until the bounded recovery budget is exhausted; a
    // single optimizer miss must not convert an otherwise safe recovery
    // opportunity into an immediate PX4 handover. This does not extend the
    // command lease and does not allow a non-finite, stale, blocked, or
    // over-error bundle to remain exposed.
    const bool recovery_bridge_usable = plan_from_rest_with_transition &&
        !planner_failure_latched_.load(std::memory_order_acquire) && committed &&
        fresh_vehicle_state && command_anchor_valid && sampled_path_clear &&
        std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
        std::isfinite(total_duration_s) && elapsed_s <= total_duration_s + 1.0e-9 &&
        std::isfinite(anchor_error_m) && anchor_error_m <= retained_tracking_limit_m;
    bool emergency_brake_committed = false;
    const bool tracking_certificate_exceeded =
        std::isfinite(anchor_error_m) &&
        anchor_error_m > retained_tracking_limit_m;
    if (measuredStateEmergencyMayReplaceCommittedCommand(
            validate_without_new_commit, use_safety_suffix,
            fresh_vehicle_state, committed, command_anchor_valid,
            tracking_certificate_exceeded,
            execution_recovery_state_.load(std::memory_order_acquire),
            committed_bundle
                ? committed_bundle->role
                : navigation_planning::CandidateRole::kEmergency)) {
      // This is the only measured-state moving transition. Propagated P/V and
      // bounded finite-difference A/J form the complete brake boundary; yaw
      // rate remains continuous with the exact command-clock sample because
      // propagated odometry does not expose yaw rate.
      navigation_planning::TrajectoryPoint emergency_command = command_anchor_sample;
      if (retained_execution_state) {
        emergency_command.position_world =
            retained_execution_state->state.position_world;
        emergency_command.velocity_world =
            retained_execution_state->state.velocity_world;
        emergency_command.acceleration_world =
            retained_execution_state->state.acceleration_world;
        emergency_command.jerk_world =
            retained_execution_state->state.jerk_world;
        emergency_command.yaw = retained_execution_state->state.yaw_rad;
        emergency_brake_committed = planner_->commitEmergencyBrake(
            emergency_command, now().seconds());
      }
      use_safety_suffix = emergency_brake_committed;
      execution_recovery_state_.store(
          transitionExecutionRecovery(
              execution_recovery_state_.load(std::memory_order_acquire),
              emergency_brake_committed
                  ? ExecutionRecoveryEvent::kEmergencyCommitted
                  : ExecutionRecoveryEvent::kEmergencyCertificationFailed),
          std::memory_order_release);
    }
    if (emergency_brake_committed &&
        !commitPlannerCandidate(*goal, goal_epoch, localization_epoch_at_solve,
                                now().nanoseconds(), scheduled_key)) {
      emergency_brake_committed = false;
      use_safety_suffix = false;
      RCLCPP_ERROR(get_logger(),
                   "execution boundary rejected the one-shot measured emergency candidate; "
                   "clearing command exposure");
      execution_recovery_state_.store(
          ExecutionRecoveryState::kPx4Hold, std::memory_order_release);
    }
    const auto retained_transition = retainedValidationTransition(use_safety_suffix);
    // A visible main-only trajectory remains a MAIN command. Only an actual
    // atomic main-to-backup bundle is marked safety-owned at the PX4 boundary.
    {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
          active_localization_epoch_.load(std::memory_order_acquire) !=
              localization_epoch_at_solve ||
          active_goal_epoch_.load() != goal_epoch) {
        // A newer goal owns command state now. This old solve is discard-only:
        // never invalidate a deliberately transferred hot-retarget command.
      } else if (!command_execution_lease_failure_latch_.allowsCommandExposure()) {
        planner_command_available_.store(false);
        planner_failure_latched_.store(true);
        safety_suffix_active_.store(false);
      } else if (recovery_bridge_usable) {
        planner_command_available_.store(true);
        command_goal_epoch_.store(goal_epoch);
        planner_failure_latched_.store(false);
        safety_suffix_active_.store(false);
        trajectory_finished_.store(false);
      } else if (!validate_without_new_commit ||
                 retained_transition == RetainedValidationTransition::FailClosed) {
        safety_suffix_active_.store(
            use_safety_suffix && (backup_available || emergency_brake_committed));
        if (use_safety_suffix && backup_available && !emergency_brake_committed) {
          execution_recovery_state_.store(
              transitionExecutionRecovery(
                  execution_recovery_state_.load(std::memory_order_acquire),
                  ExecutionRecoveryEvent::kBackupActivated),
              std::memory_order_release);
        }
        planner_failure_latched_.store(!use_safety_suffix);
        if (!use_safety_suffix) {
          planner_command_available_.store(false);
          command_goal_epoch_.store(0U);
        } else if (emergency_brake_committed) {
          planner_command_available_.store(true);
          command_goal_epoch_.store(goal_epoch);
          trajectory_finished_.store(false);
        }
      }
    }
    if (emergency_brake_committed &&
        command_execution_lease_failure_latch_.allowsCommandExposure()) {
      RCLCPP_WARN(get_logger(),
                  "planner backend replaced the exceeded-anchor command with one "
                  "measured-state emergency brake");
    }
    if (use_safety_suffix && validate_without_new_commit) {
      RCLCPP_DEBUG(get_logger(),
                   "planner backend reported NO_NEED; retained committed command remains "
                   "latest-world valid without a new commit");
    } else if (recovery_bridge_usable) {
      RCLCPP_WARN(get_logger(),
                  "planner backend recovery solve missed; retaining the fresh certified "
                  "command bridge while bounded recovery continues");
    } else if (use_safety_suffix) {
      RCLCPP_WARN(get_logger(),
                  "planner backend hot replan failed (%d); retaining visible committed trajectory "
                  "backup=%d elapsed=%.3f backup_start=%.3f end=%.3f "
                  "anchor_error=%.3f projected_anchor_error=%.3f "
                  "relative_anchor_speed=%.3f tracking_limit=%.3f",
                  static_cast<int>(result), backup_available, elapsed_s, safety_transition_s,
                  total_duration_s, anchor_error_m, projected_anchor_error_m,
                  relative_anchor_speed_mps, retained_tracking_limit_m);
    } else {
      RCLCPP_ERROR(get_logger(),
                   "planner backend hot replan failed without a valid safety suffix: backup=%d "
                   "elapsed=%.3f backup_start=%.3f end=%.3f anchor_error=%.3f "
                   "projected_anchor_error=%.3f relative_anchor_speed=%.3f "
                   "tracking_limit=%.3f "
                   "state_age=%.3f clear=%d blocked_t=%.3f blocked_grid=%d "
                   "blocked=(%.2f,%.2f,%.2f)",
                   backup_available, elapsed_s,
                   safety_transition_s, total_duration_s,
                   anchor_error_m, projected_anchor_error_m,
                   relative_anchor_speed_mps, retained_tracking_limit_m,
                   latest_vehicle_state_age_s, sampled_path_clear,
                   first_blocked_sample_s, static_cast<int>(first_blocked_grid),
                   first_blocked_sample.x(), first_blocked_sample.y(),
                   first_blocked_sample.z());
    }
  }
  if (disposition == PlannerResultDisposition::CommandReady) {
    {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (!nominalPlanningAllowed(
              execution_recovery_state_.load(std::memory_order_acquire))) {
        planner_->discardCommandCandidate();
        return;
      }
      if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
          active_localization_epoch_.load(std::memory_order_acquire) !=
              localization_epoch_at_solve ||
          active_goal_epoch_.load() != goal_epoch) {
        planner_->discardCommandCandidate();
        return;
      }
      if (!command_execution_lease_failure_latch_.allowsCommandExposure()) {
        planner_command_available_.store(false);
        planner_failure_latched_.store(true);
        safety_suffix_active_.store(false);
        planner_->discardCommandCandidate();
        return;
      }
      if (timed_out_planner_solve_generation_.load() == solve_generation) {
        planner_command_available_.store(false);
        planner_failure_latched_.store(true);
        safety_suffix_active_.store(false);
        planner_->discardCommandCandidate();
        return;
      }
      planner_command_available_.store(false);
      command_goal_epoch_.store(0U);
      planner_failure_latched_.store(false);
      safety_suffix_active_.store(false);
      trajectory_finished_.store(false);
      terminal_bundle_generation_.store(0U);
    }
    if (!commitPlannerCandidate(*goal, goal_epoch, localization_epoch_at_solve,
                                now().nanoseconds(), scheduled_key)) {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      planner_command_available_.store(false);
      command_goal_epoch_.store(0U);
      safety_suffix_active_.store(false);
      RCLCPP_WARN(get_logger(),
                  "execution boundary rejected planner candidate after solve; "
                  "command remains fail-closed");
      return;
    }
    {
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
          active_localization_epoch_.load(std::memory_order_acquire) !=
              localization_epoch_at_solve ||
          active_goal_epoch_.load() != goal_epoch ||
          !command_execution_lease_failure_latch_.allowsCommandExposure()) {
        planner_command_available_.store(false);
        command_goal_epoch_.store(0U);
        safety_suffix_active_.store(false);
        return;
      }
      planner_command_available_.store(true);
      command_goal_epoch_.store(goal_epoch);
      execution_recovery_state_.store(
          transitionExecutionRecovery(
              execution_recovery_state_.load(std::memory_order_acquire),
              ExecutionRecoveryEvent::kMainCommitted),
          std::memory_order_release);
    }
    const auto committed_bundle = command_bundle_store_.load();
    const auto committed_end_sample = committed_bundle &&
        committed_bundle->hasDeclaredEndpointMetadata()
        ? committed_bundle->sampleAtDeclaredEnd()
        : std::nullopt;
    const double completion_tolerance = goalCompletionTolerance(*goal);
    trajectory_reaches_goal_.store(
        committed_end_sample.has_value() &&
        (committed_end_sample->position_world - target).norm() <= completion_tolerance);
    if (committed_bundle && committed_bundle->hasDeclaredEndpointMetadata() &&
        (!committed_end_sample ||
         (committed_end_sample->position_world - target).norm() > completion_tolerance)) {
      RCLCPP_WARN(get_logger(),
                  "planner backend terminal endpoint check failed sample=%d start=%.9f "
                  "duration=%.9f backup_start=%.9f valid_from=%ld valid_until=%ld target=(%.2f,%.2f,%.2f)",
                  committed_end_sample.has_value() ? 1 : 0,
                  committed_bundle->start_wall_time_s, committed_bundle->duration_s,
                  committed_bundle->backup_start_time_s,
                  static_cast<long>(committed_bundle->valid_from_ns),
                  static_cast<long>(committed_bundle->valid_until_ns),
                  target.x(), target.y(), target.z());
    }
    if (plan_from_rest_with_transition) skip_replan_once_ = true;
    std::lock_guard<std::mutex> lock(input_mutex_);
    plan_from_rest_failure_budget_.reset();
    plan_from_rest_first_failure_steady_ns_ = 0;
    // Do not fall back to hot replan after a failed new-goal attempt.  That
    // would keep publishing the previous waypoint while the mission has
    // already advanced.
    if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
        active_goal_->waypoint_index == goal->waypoint_index &&
        active_goal_->request_id == goal->request_id) {
      if (new_goal) new_goal_ = false;
      if (clearHotGoalTransitionAfterCommit(
              plan_from_rest_with_transition, replan_for_new_goal)) {
        hot_goal_transition_ = false;
      }
      if (restart_from_rest) restart_from_rest_ = false;
    }
  }

  // Emit one decision record for every solve generation.  Successful hot
  // replans are as important as failures for latency distributions and for
  // proving that sampled commands came from one committed generation.
  {
    const auto committed_snapshot = planner_->committedSnapshot();
    const auto& committed = committed_snapshot.position;
    const auto committed_generation = committed_snapshot.generation;
    const auto& committed_certificate = committed_snapshot.certificate;
    const auto& commit_diagnostics = committed_snapshot.diagnostics;
    const bool commit_observed_this_cycle = commitObservedThisCycle(
        committed_generation_before_solve, committed_generation,
        commit_diagnostics.generation);
    const bool has_committed_bundle = committed_generation > 0U && !committed.empty();
    navigation_planning::TrajectoryPoint committed_end_sample;
    const bool committed_end_valid = has_committed_bundle && committed.sample(
        committed.duration_s, committed_end_sample);
    const Eigen::Vector3d committed_end = committed_end_valid
        ? committed_end_sample.position_world : Eigen::Vector3d::Zero();
    const double endpoint_error = !has_committed_bundle
                                      ? std::numeric_limits<double>::infinity()
                                      : (committed_end - target).norm();
    const auto robot_grid_type = pinned_world.view->classify(
        execution_state.position_world, navigation_world_model::GridLayer::kEvidence);
    const auto robot_inflated_grid_type = pinned_world.view->classify(
        execution_state.position_world, navigation_world_model::GridLayer::kInflated);
    const Eigen::Vector3d planning_target = planner_diagnostics.planning_goal.allFinite()
        ? planner_diagnostics.planning_goal : target;
    const auto target_grid_type = pinned_world.view->classify(
        target, navigation_world_model::GridLayer::kEvidence);
    const auto target_inflated_grid_type = pinned_world.view->classify(
        target, navigation_world_model::GridLayer::kInflated);
    const auto planning_target_grid_type = pinned_world.view->classify(
        planning_target, navigation_world_model::GridLayer::kEvidence);
    const auto planning_target_inflated_grid_type = pinned_world.view->classify(
        planning_target, navigation_world_model::GridLayer::kInflated);
    // WorldModel deliberately has no nearest-known-free or nearest-occupied
    // query. Keep these optional diagnostics unavailable rather than reaching
    // back into worker-owned mutable map state.
    const double nearest_known_free_distance =
        std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d nearest_occupied = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    const double nearest_occupied_distance =
        std::numeric_limits<double>::quiet_NaN();
    const int solve_stage = planner_diagnostics.solve_stage;
    const int replan_return_code = planner_diagnostics.replan_return_code;
    const int commit_decision = planner_diagnostics.commit_decision;
    const double planner_elapsed_ms = static_cast<double>(last_planner_us_) * 1.0e-3;
    const bool solve_deadline_exceeded =
        planner_elapsed_ms > planner_diagnostics.solve_deadline_s * 1000.0;
    const auto trace_now_ns = now().nanoseconds();
    const double execution_age_at_trace_ms =
        executionStateAgeMs(trace_now_ns, execution_stamp_ns);
    RCLCPP_INFO(get_logger(),
                "planner backend decision_trace cycle=%lu solve_generation=%lu committed_generation=%lu "
                "pinned_world_generation=%lu pinned_world_revision=%lu "
                "pinned_world_stamp_ns=%ld "
                "certificate_world_generation=%lu certificate_world_revision=%lu "
                "certificate_world_stamp_ns=%ld "
                "cloud_stamp_ns=%ld corrected_stamp_ns=%ld propagated_stamp_ns=%ld "
                "state_age_ms=%.3f mode=%s result=%d replan_code=%d commit_decision=%d "
                "solve_stage=%d solve_stage_name=%s solve_elapsed_ms=%.3f "
                "solve_deadline_exceeded=%d target=(%.2f,%.2f,%.2f) "
                "planning_target=(%.2f,%.2f,%.2f) goal_radius=%.3f goal_adjusted=%d "
                "target_grid=%d target_inf_grid=%d "
                "planning_target_grid=%d planning_target_inf_grid=%d "
                "committed_end=(%.2f,%.2f,%.2f) endpoint_error=%.3f command=%d failure=%d "
                "exp_frontend_ms=%.3f exp_opt_ms=%.3f backup_frontend_ms=%.3f "
                "backup_opt_ms=%.3f robot_grid=%d robot_inf_grid=%d nearest_free_m=%.3f "
                "nearest_occ_m=%.3f nearest_occ=(%.2f,%.2f,%.2f)",
                static_cast<unsigned long>(cycle_count_),
                static_cast<unsigned long>(solve_generation),
                static_cast<unsigned long>(committed_generation),
                static_cast<unsigned long>(pinned_world.identity.generation),
                static_cast<unsigned long>(pinned_world.identity.revision),
                static_cast<long>(pinned_world.identity.observation_stamp_ns),
                static_cast<unsigned long>(committed_certificate.validated_world.generation),
                static_cast<unsigned long>(committed_certificate.validated_world.revision),
                static_cast<long>(committed_certificate.validated_world.observation_stamp_ns),
                static_cast<long>(cloud_stamp_ns), static_cast<long>(corrected_stamp_ns),
                static_cast<long>(execution_stamp_ns),
                execution_age_at_trace_ms,
                plan_from_rest_with_transition ? "PlanFromRest" : "ReplanOnce",
                static_cast<int>(result),
                replan_return_code, commit_decision, solve_stage,
                planner_diagnostics.solve_stage_name.c_str(), planner_elapsed_ms,
                solve_deadline_exceeded, target.x(), target.y(),
                target.z(), planning_target.x(), planning_target.y(), planning_target.z(),
                planner_diagnostics.goal_acceptance_radius_m,
                planner_diagnostics.goal_endpoint_adjusted ? 1 : 0,
                static_cast<int>(target_grid_type), static_cast<int>(target_inflated_grid_type),
                static_cast<int>(planning_target_grid_type),
                static_cast<int>(planning_target_inflated_grid_type),
                committed_end.x(), committed_end.y(), committed_end.z(),
                endpoint_error, planner_command_available_.load(), planner_failure_latched_.load(),
                planner_diagnostics.module_time_us[0] * 1.0e-3,
                planner_diagnostics.module_time_us[1] * 1.0e-3,
                planner_diagnostics.module_time_us[2] * 1.0e-3,
                planner_diagnostics.module_time_us[3] * 1.0e-3,
                static_cast<int>(robot_grid_type), static_cast<int>(robot_inflated_grid_type),
                nearest_known_free_distance, nearest_occupied_distance,
                nearest_occupied.x(), nearest_occupied.y(), nearest_occupied.z());

    if (commit_observed_this_cycle) {
      RCLCPP_INFO(get_logger(),
                  "planner backend commit_trace generation=%lu previous_generation=%lu "
                  "start_wall_time=%.9f execution_stamp_ns=%ld "
                  "execution_age_at_solve_ms=%.3f execution_age_at_trace_ms=%.3f "
                  "execution_p=[%.6f,%.6f,%.6f] execution_v=[%.6f,%.6f,%.6f] "
                  "candidate_start_p=[%.6f,%.6f,%.6f] "
                  "candidate_start_v=[%.6f,%.6f,%.6f] previous_valid=%d "
                  "candidate_start_a=[%.6f,%.6f,%.6f] "
                  "candidate_start_j=[%.6f,%.6f,%.6f] "
                  "previous_sample_tt=%.6f splice_p=%.6f splice_v=%.6f "
                  "splice_a=%.6f splice_j=%.6f splice_yaw=%.6f "
                  "splice_yaw_rate=%.6f",
                  static_cast<unsigned long>(commit_diagnostics.generation),
                  static_cast<unsigned long>(commit_diagnostics.previous_generation),
                  commit_diagnostics.candidate_start_wall_time_s,
                  static_cast<long>(execution_stamp_ns),
                  execution_age_at_solve_ms,
                  execution_age_at_trace_ms,
                  execution_state.position_world.x(), execution_state.position_world.y(),
                  execution_state.position_world.z(), execution_state.velocity_world.x(),
                  execution_state.velocity_world.y(), execution_state.velocity_world.z(),
                  commit_diagnostics.candidate_start_position.x(),
                  commit_diagnostics.candidate_start_position.y(),
                  commit_diagnostics.candidate_start_position.z(),
                  commit_diagnostics.candidate_start_velocity.x(),
                  commit_diagnostics.candidate_start_velocity.y(),
                  commit_diagnostics.candidate_start_velocity.z(),
                  commit_diagnostics.previous_valid ? 1 : 0,
                  commit_diagnostics.candidate_start_acceleration.x(),
                  commit_diagnostics.candidate_start_acceleration.y(),
                  commit_diagnostics.candidate_start_acceleration.z(),
                  commit_diagnostics.candidate_start_jerk.x(),
                  commit_diagnostics.candidate_start_jerk.y(),
                  commit_diagnostics.candidate_start_jerk.z(),
                  commit_diagnostics.previous_sample_time_s,
                  commit_diagnostics.position_residual.norm(),
                  commit_diagnostics.velocity_residual.norm(),
                  commit_diagnostics.acceleration_residual.norm(),
                  commit_diagnostics.jerk_residual.norm(),
                  commit_diagnostics.yaw_residual,
                  commit_diagnostics.yaw_rate_residual);
    }

    diagnostic_msgs::msg::DiagnosticArray trace_diagnostics;
    trace_diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus trace_status;
    trace_status.name = "navigation_runtime/planner";
    trace_status.level = result == navigation_planning::PlannerStatus::kSuccess
                             ? diagnostic_msgs::msg::DiagnosticStatus::OK
                             : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    trace_status.message = "DECISION_TRACE";
    const auto add_trace_value = [&trace_status](const std::string& key,
                                                  const auto& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      trace_status.values.push_back(std::move(item));
    };
    const auto add_trace_vector = [&trace_status](const std::string& key,
                                                   const auto& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      std::ostringstream stream;
      stream << std::setprecision(17) << '[' << value.x() << ',' << value.y()
             << ',' << value.z() << ']';
      item.value = stream.str();
      trace_status.values.push_back(std::move(item));
    };
    add_trace_value("planning_cycle_id", cycle_count_);
    add_trace_value("bundle_id", committed_generation);
    add_trace_value("solve_generation", solve_generation);
    add_trace_value("commit_observed_this_cycle", commit_observed_this_cycle ? 1 : 0);
    add_trace_value("execution_stamp_ns", execution_stamp_ns);
    add_trace_value("state_age_at_solve_ms",
                    execution_age_at_solve_ms);
    add_trace_value("state_age_at_trace_ms", execution_age_at_trace_ms);
    add_trace_vector("planning_state_position", execution_state.position_world);
    add_trace_vector("planning_state_velocity", execution_state.velocity_world);
    add_trace_vector("planning_state_acceleration", execution_state.acceleration_world);
    add_trace_vector("planning_state_jerk", execution_state.jerk_world);
    add_trace_value("planning_state_acceleration_estimated",
                    execution_state.acceleration_estimated ? 1 : 0);
    add_trace_value("planning_state_jerk_estimated",
                    execution_state.jerk_estimated ? 1 : 0);
    add_trace_vector("requested_goal", target);
    add_trace_vector("planning_goal", planning_target);
    add_trace_value("goal_acceptance_radius_m",
                    planner_diagnostics.goal_acceptance_radius_m);
    add_trace_value("goal_endpoint_adjusted",
                    planner_diagnostics.goal_endpoint_adjusted ? 1 : 0);
    add_trace_value("target_grid_state", static_cast<int>(target_grid_type));
    add_trace_value("target_inflated_grid_state", static_cast<int>(target_inflated_grid_type));
    if (commit_observed_this_cycle) {
      add_trace_value("commit_previous_generation",
                      commit_diagnostics.previous_generation);
      add_trace_value("candidate_start_wall_time_s",
                      commit_diagnostics.candidate_start_wall_time_s);
      add_trace_vector("candidate_start_position",
                       commit_diagnostics.candidate_start_position);
      add_trace_vector("candidate_start_velocity",
                       commit_diagnostics.candidate_start_velocity);
      add_trace_vector("candidate_start_acceleration",
                       commit_diagnostics.candidate_start_acceleration);
      add_trace_vector("candidate_start_jerk",
                       commit_diagnostics.candidate_start_jerk);
      add_trace_value("splice_previous_valid",
                      commit_diagnostics.previous_valid ? 1 : 0);
      add_trace_value("splice_previous_sample_tt_s",
                      commit_diagnostics.previous_sample_time_s);
      add_trace_value("splice_position_residual_m",
                      commit_diagnostics.position_residual.norm());
      add_trace_value("splice_velocity_residual_mps",
                      commit_diagnostics.velocity_residual.norm());
      add_trace_value("splice_acceleration_residual_mps2",
                      commit_diagnostics.acceleration_residual.norm());
      add_trace_value("splice_jerk_residual_mps3",
                      commit_diagnostics.jerk_residual.norm());
      add_trace_value("splice_yaw_residual_rad", commit_diagnostics.yaw_residual);
      add_trace_value("splice_yaw_rate_residual_radps",
                      commit_diagnostics.yaw_rate_residual);
    }
    add_trace_value("pinned_world_generation", pinned_world.identity.generation);
    add_trace_value("pinned_world_revision", pinned_world.identity.revision);
    add_trace_value("pinned_world_stamp_ns", pinned_world.identity.observation_stamp_ns);
      add_trace_value("certificate_world_generation",
                    committed_certificate.validated_world.generation);
    add_trace_value("certificate_world_revision",
                    committed_certificate.validated_world.revision);
    add_trace_value("certificate_world_stamp_ns",
                    committed_certificate.validated_world.observation_stamp_ns);
    add_trace_value("candidate_result", static_cast<int>(result));
    add_trace_value("replan_code", replan_return_code);
    add_trace_value("commit_decision", commit_decision);
    add_trace_value("solve_stage", solve_stage);
    {
      diagnostic_msgs::msg::KeyValue item;
      item.key = "solve_stage_name";
      item.value = planner_diagnostics.solve_stage_name;
      trace_status.values.push_back(std::move(item));
    }
    add_trace_value("planning_latency_ms", planner_elapsed_ms);
    // Keep the canonical total in the same microsecond unit as all planner
    // stage fields. `planning_latency_ms` remains for the rolling trace, while
    // this field feeds the report's percentile table without reconstructing a
    // total by summing stages that may overlap or be skipped.
    add_trace_value("planning_total_us", last_planner_us_);
    // Keep planner stage timings explicit and unit-labelled in the structured
    // trace.  The raw planner backend log already exposes these values, but without
    // publishing them here the report cannot identify which stage causes a
    // solve deadline overrun.
    add_trace_value("exp_frontend_us",
                    planner_diagnostics.module_time_us[0]);
    add_trace_value("exp_opt_us",
                    planner_diagnostics.module_time_us[1]);
    add_trace_value("backup_frontend_us",
                    planner_diagnostics.module_time_us[2]);
    add_trace_value("backup_opt_us",
                    planner_diagnostics.module_time_us[3]);
    add_trace_value("backup_certificate_attempted",
                    backup_diagnostics.attempted ? 1 : 0);
    add_trace_value("backup_switch_candidate_count",
                    backup_diagnostics.switch_candidate_count);
    add_trace_value("backup_feasible_seed_count",
                    backup_diagnostics.feasible_seed_count);
    add_trace_value("backup_visibility_hull_pass_count",
                    backup_diagnostics.visibility_hull_pass_count);
    add_trace_value("backup_aligned_sfc_built_count",
                    backup_diagnostics.aligned_sfc_built_count);
    add_trace_value("backup_aligned_hull_pass_count",
                    backup_diagnostics.aligned_hull_pass_count);
    add_trace_value("backup_known_free_check_count",
                    backup_diagnostics.known_free_check_count);
    add_trace_value("backup_known_free_pass_count",
                    backup_diagnostics.known_free_pass_count);
    add_trace_value("backup_certificate_selected",
                    backup_diagnostics.selected ? 1 : 0);
    add_trace_value("backup_last_reject_stage",
                    backup_diagnostics.last_reject_stage);
    add_trace_value("backup_last_known_free_failure_code",
                    backup_diagnostics.last_known_free_failure_code);
    add_trace_value("backup_last_known_free_cell_state",
                    backup_diagnostics.last_known_free_cell_state);
    add_trace_value("backup_last_known_free_blocked_role",
                    backup_diagnostics.last_known_free_blocked_role);
    add_trace_value("backup_last_known_free_first_blocked_time_s",
                    backup_diagnostics.last_known_free_first_blocked_time_s);
    add_trace_vector("backup_last_known_free_blocked_position",
                     backup_diagnostics.last_known_free_blocked_position);
    add_trace_value("backup_last_seed_switch_time_s",
                    backup_diagnostics.last_seed_switch_time_s);
    add_trace_value("backup_last_seed_duration_s",
                    backup_diagnostics.last_seed_duration_s);
    add_trace_value("backup_last_seed_initial_velocity_mps",
                    backup_diagnostics.last_seed_initial_velocity_mps);
    add_trace_value("backup_last_seed_max_velocity_mps",
                    backup_diagnostics.last_seed_max_velocity_mps);
    add_trace_value("backup_last_seed_max_acceleration_mps2",
                    backup_diagnostics.last_seed_max_acceleration_mps2);
    add_trace_value("backup_last_seed_max_jerk_mps3",
                    backup_diagnostics.last_seed_max_jerk_mps3);
    add_trace_vector("backup_last_seed_endpoint",
                     backup_diagnostics.last_seed_endpoint);
    add_trace_value("exp_diagnostics_valid", exp_diagnostics.valid ? 1 : 0);
    add_trace_value("exp_used_certified_seed",
                    exp_diagnostics.used_certified_seed ? 1 : 0);
    add_trace_value("exp_certified_seed_failure_stage",
                    exp_diagnostics.certified_seed_failure_stage);
    add_trace_value("exp_corridor_seed_build_failure_stage",
                    exp_diagnostics.corridor_seed_build_failure_stage);
    add_trace_value("exp_corridor_seed_retry_attempt_count",
                    exp_diagnostics.corridor_seed_retry_attempt_count);
    add_trace_value("exp_corridor_seed_retry_build_valid_count",
                    exp_diagnostics.corridor_seed_retry_build_valid_count);
    add_trace_value("exp_corridor_seed_retry_last_certificate_stage",
                    exp_diagnostics.corridor_seed_retry_last_certificate_stage);
    add_trace_value("exp_corridor_seed_selected_mode",
                    exp_diagnostics.corridor_seed_selected_mode);
    add_trace_value("exp_corridor_seed_selected_max_duration_scale",
                    exp_diagnostics.corridor_seed_selected_max_duration_scale);
    add_trace_value("exp_lbfgs_attempt_count", exp_diagnostics.lbfgs_attempt_count);
    add_trace_value("exp_lbfgs_evaluation_count",
                    exp_diagnostics.lbfgs_evaluation_count);
    add_trace_value("exp_lbfgs_first_attempt_evaluation_count",
                    exp_diagnostics.lbfgs_first_attempt_evaluation_count);
    add_trace_value("exp_lbfgs_last_attempt_evaluation_count",
                    exp_diagnostics.lbfgs_last_attempt_evaluation_count);
    add_trace_value("exp_retry_count", exp_diagnostics.retry_count);
    add_trace_value("exp_retry_violation_mask", exp_diagnostics.retry_violation_mask);
    add_trace_value("exp_retry_stop_reason", exp_diagnostics.retry_stop_reason);
    add_trace_value("exp_lbfgs_first_return_code",
                    exp_diagnostics.first_lbfgs_return_code);
    add_trace_value("exp_lbfgs_last_return_code",
                    exp_diagnostics.last_lbfgs_return_code);
    add_trace_value("exp_lbfgs_cancelled", exp_diagnostics.cancelled ? 1 : 0);
    add_trace_value("exp_initial_normalized_dynamic_violation",
                    exp_diagnostics.initial_normalized_dynamic_violation);
    add_trace_value("exp_best_normalized_dynamic_violation",
                    exp_diagnostics.best_normalized_dynamic_violation);
    add_trace_value("exp_final_normalized_dynamic_violation",
                    exp_diagnostics.final_normalized_dynamic_violation);
    // Keep the historical generic names populated for report compatibility;
    // these are the last independently evaluated MINCO candidate extrema,
    // including a candidate rejected by a hard gate.
    add_trace_value("maximum_velocity_mps",
                    exp_diagnostics.last_candidate_maximum_velocity_mps);
    add_trace_value("maximum_acceleration_mps2",
                    exp_diagnostics.last_candidate_maximum_acceleration_mps2);
    add_trace_value("maximum_jerk_mps3",
                    exp_diagnostics.last_candidate_maximum_jerk_mps3);
    add_trace_value("exp_certified_seed_maximum_velocity_mps",
                    exp_diagnostics.certified_seed_maximum_velocity_mps);
    add_trace_value("exp_certified_seed_maximum_acceleration_mps2",
                    exp_diagnostics.certified_seed_maximum_acceleration_mps2);
    add_trace_value("exp_certified_seed_maximum_jerk_mps3",
                    exp_diagnostics.certified_seed_maximum_jerk_mps3);
    add_trace_value("exp_initial_duration_s", exp_diagnostics.initial_duration_s);
    add_trace_value("exp_initial_minimum_piece_duration_s",
                    exp_diagnostics.initial_minimum_piece_duration_s);
    add_trace_value("exp_initial_maximum_piece_duration_s",
                    exp_diagnostics.initial_maximum_piece_duration_s);
    add_trace_value("exp_final_duration_s", exp_diagnostics.final_duration_s);
    add_trace_value("exp_retry_duration_lower_bound_min_s",
                    exp_diagnostics.retry_duration_lower_bound_min_s);
    add_trace_value("exp_retry_duration_lower_bound_max_s",
                    exp_diagnostics.retry_duration_lower_bound_max_s);
    add_trace_value("exp_retry_free_duration_seed_min_s",
                    exp_diagnostics.retry_free_duration_seed_min_s);
    add_trace_value("exp_retry_free_duration_seed_max_s",
                    exp_diagnostics.retry_free_duration_seed_max_s);
    add_trace_value("guide_path_length_m", planner_diagnostics.latest_guide_path_length_m);
    add_trace_value("guide_duration_s", planner_diagnostics.latest_guide_duration_s);
    add_trace_value("required_lookahead_m", planner_diagnostics.required_lookahead_m);
    add_trace_value("certified_lookahead_m", planner_diagnostics.certified_lookahead_m);
    add_trace_value("lookahead_complete", planner_diagnostics.lookahead_complete ? 1 : 0);
    add_trace_value("route_yaw_source", planner_diagnostics.route_yaw_source);
    add_trace_value("route_yaw_target_rad", planner_diagnostics.route_yaw_target_rad);
    add_trace_value("route_yaw_lookahead_m", planner_diagnostics.route_yaw_lookahead_m);
    add_trace_value("route_yaw_progress_arc_m",
                    planner_diagnostics.route_yaw_progress_arc_m);
    add_trace_value("yaw_rate_limit_rad_s", planner_diagnostics.yaw_rate_limit_rad_s);
    add_trace_value("yaw_acceleration_limit_rad_s2",
                    planner_diagnostics.yaw_acceleration_limit_rad_s2);
    add_trace_value("candidate_maximum_yaw_rate_rad_s",
                    planner_diagnostics.candidate_maximum_yaw_rate_rad_s);
    add_trace_value("candidate_maximum_yaw_acceleration_rad_s2",
                    planner_diagnostics.candidate_maximum_yaw_acceleration_rad_s2);
    add_trace_value("exp_retry_budget_remaining_us",
                    exp_diagnostics.retry_budget_remaining_us);
    add_trace_value("exp_nonfinite_evaluation_count",
                    exp_diagnostics.nonfinite_evaluation_count);
    add_trace_value("exp_first_nonfinite_stage",
                    exp_diagnostics.first_nonfinite_stage);
    add_trace_value("exp_first_nonfinite_value_mask",
                    exp_diagnostics.first_nonfinite_value_mask);
    add_trace_value("exp_first_nonfinite_attempt",
                    exp_diagnostics.first_nonfinite_attempt);
    add_trace_value("exp_first_nonfinite_iteration",
                    exp_diagnostics.first_nonfinite_iteration);
    add_trace_value("exp_first_nonfinite_min_duration_s",
                    exp_diagnostics.first_nonfinite_min_duration_s);
    add_trace_value("exp_first_nonfinite_max_duration_s",
                    exp_diagnostics.first_nonfinite_max_duration_s);
    add_trace_value("exp_first_nonfinite_cost",
                    exp_diagnostics.first_nonfinite_cost);
    add_trace_value("exp_first_nonfinite_gradient_norm",
                    exp_diagnostics.first_nonfinite_gradient_norm);
    add_trace_value("solve_deadline_exceeded", solve_deadline_exceeded ? 1 : 0);
    add_trace_value("command_available", planner_command_available_.load() ? 1 : 0);
    add_trace_value("planner_failure_latched", planner_failure_latched_.load() ? 1 : 0);
    trace_diagnostics.status.push_back(std::move(trace_status));

    // Publish the actual immutable bundle geometry used by the command
    // sampler.  A rejected command is deliberately represented as
    // path_available=false; its terminal hold sample must never be rendered
    // as a safety trajectory at the takeoff origin.  The report can therefore
    // show nominal and backup on one common map without reconstructing paths
    // from repeated NavigationCommand samples.
    diagnostic_msgs::msg::DiagnosticStatus path_status;
    path_status.name = "navigation_runtime/planner_path";
    path_status.level = has_committed_bundle
                            ? diagnostic_msgs::msg::DiagnosticStatus::OK
                            : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    path_status.message = has_committed_bundle ? "PATH_SNAPSHOT" : "NO_EXECUTABLE_PATH";
    diagnostic_msgs::msg::KeyValue path_item;
    path_item.key = "planner_path_snapshot_json";
    path_item.value = plannerPathSnapshotJson(
        committed_snapshot, cycle_count_, solve_generation,
        static_cast<int>(result), replan_return_code, commit_decision,
        planner_diagnostics.solve_stage_name, target, planning_target,
        planner_diagnostics.goal_acceptance_radius_m,
        planner_diagnostics.goal_endpoint_adjusted, robot_grid_type,
        robot_inflated_grid_type, target_grid_type, target_inflated_grid_type,
        safety_suffix_active_.load());
    path_status.values.push_back(std::move(path_item));
    trace_diagnostics.status.push_back(std::move(path_status));
    diagnostics_publisher_->publish(trace_diagnostics);
  }

  const auto metrics_now = std::chrono::steady_clock::now();
  const double planner_cycle_ms = static_cast<double>(
      std::chrono::duration_cast<std::chrono::microseconds>(metrics_now - cycle_started).count()) /
                                 1000.0;
  if (end_to_end_samples_ms_.size() == 256U) end_to_end_samples_ms_.erase(end_to_end_samples_ms_.begin());
  end_to_end_samples_ms_.push_back(planner_cycle_ms);
  if (metrics_now - metrics_log_time_ >= std::chrono::seconds(1)) {
    auto sorted = end_to_end_samples_ms_;
    std::sort(sorted.begin(), sorted.end());
    const auto percentile = [&](double fraction) {
      if (sorted.empty()) return 0.0;
      const auto index = std::min(sorted.size() - 1U,
                                  static_cast<std::size_t>(fraction * sorted.size()));
      return sorted[index];
    };
    const auto committed = planner_->committedSnapshot();
    const auto planner_diagnostics = planner_->diagnostics();
    const auto committed_generation = committed.generation;
    const bool backup_available = committed.backup_available;
    const double backup_start_s = committed.backup_start_time_s;
    const auto guide_end = planner_diagnostics.latest_guide_end;
    const auto guide_min = planner_diagnostics.latest_guide_min;
    const auto guide_max = planner_diagnostics.latest_guide_max;
    const bool committed_valid = committed_generation > 0U && !committed.empty();
    const double committed_duration = committed.position.duration_s;
    const auto sample_position = [&committed, committed_valid](const double time_s) {
      navigation_planning::TrajectoryPoint sample;
      return committed_valid && committed.position.sample(time_s, sample)
          ? sample.position_world : Eigen::Vector3d::Zero();
    };
    const auto committed_start = sample_position(0.0);
    const auto committed_end = sample_position(committed_duration);
    const auto committed_quarter = sample_position(committed_duration * 0.25);
    const auto committed_half = sample_position(committed_duration * 0.50);
    const auto committed_three_quarter = sample_position(committed_duration * 0.75);
    const auto main_end = !committed_valid || !backup_available
                              ? committed_end
                              : sample_position(std::clamp(backup_start_s, 0.0, committed_duration));
    RCLCPP_INFO(get_logger(),
                "planner_cycle_metrics cycles=%lu commands=%lu dropped_cloud=%lu "
                "planner_cycle_ms=%.3f p50_ms=%.3f p95_ms=%.3f p99_ms=%.3f "
                "map_us=%ld "
                "planner_us=%ld publish_us=%ld store_publish_us=%ld transition_lock_us=%ld "
                "input_lock_us=%ld target=(%.2f,%.2f,%.2f) "
                "robot=(%.2f,%.2f,%.2f) committed_start=(%.2f,%.2f,%.2f) "
                "committed_q1=(%.2f,%.2f,%.2f) committed_mid=(%.2f,%.2f,%.2f) "
                "committed_q3=(%.2f,%.2f,%.2f) committed_end=(%.2f,%.2f,%.2f) "
                "main_end=(%.2f,%.2f,%.2f) backup_start=%.3f "
                "guide_end=(%.2f,%.2f,%.2f) "
                "guide_bounds=[(%.2f,%.2f,%.2f),(%.2f,%.2f,%.2f)] "
                "committed_duration=%.3f",
                static_cast<unsigned long>(cycle_count_),
                static_cast<unsigned long>(command_publish_count_.load()),
                static_cast<unsigned long>(accounting.replaced_waiting +
                                           accounting.replaced_ready), planner_cycle_ms,
                percentile(0.50), percentile(0.95), percentile(0.99),
                static_cast<long>(mapping.map_update_us), static_cast<long>(last_planner_us_),
                static_cast<long>(last_publish_us_.load()),
                static_cast<long>(last_command_store_publish_us_.load()),
                static_cast<long>(last_command_transition_lock_wait_us_.load()),
                static_cast<long>(last_input_lock_wait_us_), target.x(), target.y(), target.z(),
                execution_state.position_world.x(), execution_state.position_world.y(),
                execution_state.position_world.z(),
                committed_start.x(), committed_start.y(), committed_start.z(),
                committed_quarter.x(), committed_quarter.y(), committed_quarter.z(),
                committed_half.x(), committed_half.y(), committed_half.z(),
                committed_three_quarter.x(), committed_three_quarter.y(), committed_three_quarter.z(),
                committed_end.x(), committed_end.y(), committed_end.z(),
                main_end.x(), main_end.y(), main_end.z(), backup_start_s,
                guide_end.x(), guide_end.y(), guide_end.z(),
                guide_min.x(), guide_min.y(), guide_min.z(),
                guide_max.x(), guide_max.y(), guide_max.z(),
                committed_duration);
    metrics_log_time_ = metrics_now;
  }
}

void NavigationRuntimeNode::publishCommand() {
  const auto command_ros_time = now();
  const double now_seconds = command_ros_time.seconds();
  if (!std::isfinite(now_seconds)) return;
  const auto localization_epoch_at_command =
      active_localization_epoch_.load(std::memory_order_acquire);
  if (!localization_epoch_ready_.load(std::memory_order_acquire)) return;

  // Do not keep publishing a command from a fresh odometry stream when the
  // world evidence has stopped advancing. This is a recoverable fail-closed
  // condition: the next fresh mapping snapshot may resume planning.
  const auto latest_world = world_snapshot_store_.load();
  const auto world_freshness = latest_world
      ? navigation_execution::classifyTimestampFreshness(
            command_ros_time.nanoseconds(), latest_world.identity.observation_stamp_ns,
            data_freshness_window_ns_)
      : navigation_execution::TimestampFreshness::INVALID;
  if (world_freshness != navigation_execution::TimestampFreshness::VALID) {
    ++world_snapshot_freshness_rejection_count_;
    if (planning_worker_) planning_worker_->cancelActive();
    // No command is published from this callback. A later fresh snapshot may
    // resume only this exact bundle after successful world recertification.
    suspendCommandForWorldFreshness();
    return;
  }

  const std::uint64_t active_solve = active_planner_solve_generation_.load();
  if (active_solve != 0U) {
    const std::int64_t solve_age_ns =
        navigation_common::steadyClockNowNanoseconds() -
        planner_solve_started_steady_ns_.load();
    if (solve_age_ns > planner_watchdog_timeout_ns_) {
      const std::uint64_t previous_timeout =
          timed_out_planner_solve_generation_.exchange(active_solve);
      if (previous_timeout != active_solve) {
        if (planning_worker_) planning_worker_->cancelActive();
        const int timeout_stage = planner_->solveStage();
        const std::size_t timeout_point_count = planner_->solvePointCount();
        {
          std::lock_guard<std::mutex> command_lock(
              command_execution_lease_failure_latch_.transitionMutex());
          planner_command_available_.store(false);
          planner_failure_latched_.store(true);
          safety_suffix_active_.store(false);
        }
        RCLCPP_ERROR(get_logger(),
                     "planner backend planner watchdog timed out generation=%lu age=%.3f s stage=%d points=%zu; "
                     "invalidating committed main trajectory",
                     static_cast<unsigned long>(active_solve),
                     static_cast<double>(solve_age_ns) / 1e9,
                     timeout_stage,
                     timeout_point_count);
      }
    }
  }

  Eigen::Matrix<double, 3, 4> pvaj = Eigen::Matrix<double, 3, 4>::Zero();
  double yaw = 0.0;
  double yaw_dot = 0.0;
  bool on_backup_traj = false;
  bool traj_finish = false;
  std::uint64_t trajectory_generation = 0;
  double trajectory_time_s = 0.0;
  navigation_world_model::WorldSnapshotIdentity command_world_identity{};
  std::shared_ptr<const navigation_planning::CandidateBundle> sampled_bundle;
  bool safety_suffix_active = safety_suffix_active_.load();
  bool planner_failed = planner_failure_latched_.load();
  bool sampled_command_valid = false;
  if (!planner_command_available_.load() && !planner_failed &&
      command_execution_lease_failure_latch_.allowsCommandExposure()) return;
  std::optional<navigation_contracts::msg::NavigationGoal> command_goal;
  std::uint64_t goal_epoch_at_command = 0U;
  std::shared_ptr<const navigation_execution::ExecutionStateLease> execution_state;
  std::int64_t execution_receive_steady_ns = 0;
  std::uint64_t execution_sequence = 0;
  execution_state = execution_state_store_.load();
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    command_goal = active_goal_;
    goal_epoch_at_command = active_goal_epoch_.load(std::memory_order_acquire);
  }
  execution_receive_steady_ns = execution_state ? execution_state->state.receive_stamp_ns : 0;
  execution_sequence = execution_state ? execution_state->ingress_sequence : 0;
  const auto execution_source_ns = execution_state ? execution_state->state.source_stamp_ns : 0;
  const auto execution_freshness = navigation_contracts::evaluateExecutionStateFreshness(
      command_ros_time.nanoseconds(), execution_source_ns,
      navigation_common::steadyClockNowNanoseconds(), execution_receive_steady_ns,
      data_freshness_window_s_);
  command_execution_lease_reason_.store(static_cast<int>(execution_freshness.reason));
  command_execution_source_age_us_.store(static_cast<std::int64_t>(
      execution_freshness.source_age_ms * 1000.0));
  command_execution_receive_age_us_.store(static_cast<std::int64_t>(
      execution_freshness.receive_age_ms * 1000.0));
  const auto transition_lock_wait_started = std::chrono::steady_clock::now();
  {
    std::unique_lock<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    last_command_transition_lock_wait_us_.store(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - transition_lock_wait_started).count(),
        std::memory_order_release);
    if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
        active_localization_epoch_.load(std::memory_order_acquire) !=
            localization_epoch_at_command) {
      planner_command_available_.store(false);
      planner_failure_latched_.store(true);
      safety_suffix_active_.store(false);
      command_goal_epoch_.store(0U);
      return;
    }
    const bool lease_already_failed =
        !command_execution_lease_failure_latch_.allowsCommandExposure();
    if (!execution_freshness.valid() || lease_already_failed) {
      ++command_execution_lease_rejection_count_;
      const bool first_failure = !execution_freshness.valid() &&
          command_execution_lease_failure_latch_.tryLatch();
      if (first_failure) {
        ++command_execution_lease_terminal_latch_count_;
      }
      // Clearing executable command state is unconditional. The latch owns
      // one-time cancellation/logging only; a solve that races past cancellation
      // must never resurrect a nominal command for this failed goal.
      planner_command_available_.store(false);
      planner_failure_latched_.store(true);
      safety_suffix_active_.store(false);
      planner_failed = true;
      safety_suffix_active = false;
      if (first_failure) {
        command_lock.unlock();
        if (planning_worker_) planning_worker_->cancelActive();
        RCLCPP_ERROR(
            get_logger(),
            "planner backend execution-state lease failed reason=%s source_age_ms=%.3f "
            "receive_age_ms=%.3f sequence=%lu active_solve=%lu; publishing terminal EMER",
            navigation_contracts::executionStateFreshnessReasonName(execution_freshness.reason),
            execution_freshness.source_age_ms, execution_freshness.receive_age_ms,
            static_cast<unsigned long>(execution_sequence),
            static_cast<unsigned long>(active_solve));
        command_lock.lock();
        planner_command_available_.store(false);
        planner_failure_latched_.store(true);
        safety_suffix_active_.store(false);
      }
    } else {
      // These three flags form one executable command decision. Reload them
      // only after acquiring the same serialization lock used by solve exposure.
      planner_failed = planner_failure_latched_.load();
      safety_suffix_active = safety_suffix_active_.load();
    }
  }
  if (planner_command_available_.load() &&
      command_goal_epoch_.load() != active_goal_epoch_.load()) {
    return;
  }
  if (planner_command_available_.load()) {
    const auto sample = command_sampler_.sample(
        command_ros_time.nanoseconds(), goal_epoch_at_command);
    if (!sample) {
      if (sample.awaiting_activation) {
        // A committed candidate may start a few milliseconds after the
        // publication tick that committed it. Keep the immutable bundle and
        // wait for its declared sample-validity boundary; do not turn a
        // scheduling lead into a planner failure or a synthetic emergency.
        return;
      }
      if (!sample.bundle) {
        // A newer world identity clears the old execution certificate before
        // recertification. This is a normal pending state, not a planner
        // failure and must not latch a terminal emergency decision.
        planner_command_available_.store(false);
        command_goal_epoch_.store(0U);
        safety_suffix_active_.store(false);
        return;
      }
      const auto terminal_endpoint = sample.bundle->sampleAtDeclaredEnd();
      const auto terminal_generation = terminal_bundle_generation_.load(
          std::memory_order_acquire);
      const auto latest_terminal_world = world_snapshot_store_.load();
      const bool endpoint_valid = terminal_endpoint.has_value() &&
                                  terminal_endpoint->finished &&
                                  terminal_endpoint->finite();
      bool endpoint_reaches_goal = false;
      if (endpoint_valid && command_goal) {
        const auto& target_message = plannerTarget(*command_goal);
        const Eigen::Vector3d target_position{
            pointFromMessage(target_message, 0),
            pointFromMessage(target_message, 1),
            pointFromMessage(target_message, 2)};
        endpoint_reaches_goal =
            (terminal_endpoint->position_world - target_position).norm() <=
            goalCompletionTolerance(*command_goal);
      }
      const bool endpoint_known_free = endpoint_valid && latest_terminal_world &&
          navigation_world_model::isCellTraversable(
              latest_terminal_world.view->classify(
                  terminal_endpoint->position_world,
                  navigation_world_model::GridLayer::kInflated),
              navigation_world_model::UnknownPolicy::kRequireKnownFree);
      const bool endpoint_near_execution_state = endpoint_valid && execution_state &&
          execution_freshness.valid() &&
          (terminal_endpoint->position_world -
           execution_state->state.position_world).norm() <=
          navigation_contracts::kCommandAnchorErrorLimitM;
      const bool observed_terminal_bundle =
          terminal_generation != 0U &&
          terminal_generation == sample.bundle->bundle_generation;
      const bool can_replay_expired_endpoint = expiredEndpointMayBeReplayed(
          endpoint_valid, endpoint_known_free, endpoint_near_execution_state);
      if (can_replay_expired_endpoint) {
        // The normal lease has expired. Publish only this bundle's exact
        // declared endpoint as STATUS_COMPLETED; never extend the trajectory's
        // executable interval or expose a future sample. A goal endpoint is a
        // bounded hold; a non-goal endpoint is a local-frontier completion
        // signal that causes the planner to start PlanFromRest again.
        const auto& point = *terminal_endpoint;
        pvaj.col(0) = point.position_world;
        pvaj.col(1) = point.velocity_world;
        pvaj.col(2) = point.acceleration_world;
        pvaj.col(3) = point.jerk_world;
        yaw = point.yaw;
        yaw_dot = point.yaw_rate;
        trajectory_generation = sample.bundle->bundle_generation;
        trajectory_time_s = point.trajectory_time_s;
        command_world_identity = sample.bundle->world_identity;
        sampled_bundle = sample.bundle;
        on_backup_traj = point.role != navigation_planning::CandidateRole::kMain;
        traj_finish = true;
        sampled_command_valid = true;
        RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "publishing exact endpoint for expired completed bundle generation=%lu "
            "goal=%d observed=%d known_free=%d near_execution=%d",
            static_cast<unsigned long>(trajectory_generation), endpoint_reaches_goal ? 1 : 0,
            observed_terminal_bundle ? 1 : 0,
            endpoint_known_free ? 1 : 0, endpoint_near_execution_state ? 1 : 0);
      } else {
        planner_command_available_.store(false);
        planner_failure_latched_.store(true);
        safety_suffix_active_.store(false);
        planner_failed = true;
        safety_suffix_active = false;
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "execution boundary invalidated the committed command sample");
      }
    } else {
      const auto& point = *sample.point;
      pvaj.col(0) = point.position_world;
      pvaj.col(1) = point.velocity_world;
      pvaj.col(2) = point.acceleration_world;
      pvaj.col(3) = point.jerk_world;
      yaw = point.yaw;
      yaw_dot = point.yaw_rate;
      trajectory_generation = sample.bundle->bundle_generation;
      trajectory_time_s = point.trajectory_time_s;
      command_world_identity = sample.bundle->world_identity;
      sampled_bundle = sample.bundle;
      on_backup_traj = point.role != navigation_planning::CandidateRole::kMain;
      if (point.role == navigation_planning::CandidateRole::kBackup) {
        execution_recovery_state_.store(
            transitionExecutionRecovery(
                execution_recovery_state_.load(std::memory_order_acquire),
                ExecutionRecoveryEvent::kBackupActivated),
            std::memory_order_release);
      } else if (point.role == navigation_planning::CandidateRole::kEmergency) {
        execution_recovery_state_.store(
            transitionExecutionRecovery(
                execution_recovery_state_.load(std::memory_order_acquire),
                ExecutionRecoveryEvent::kEmergencyCommitted),
            std::memory_order_release);
      }
      traj_finish = point.finished;
      sampled_command_valid = true;
      if (traj_finish) {
        bool endpoint_reaches_goal = false;
        if (command_goal) {
          const auto& target_message = plannerTarget(*command_goal);
          const Eigen::Vector3d target_position{
              pointFromMessage(target_message, 0),
              pointFromMessage(target_message, 1),
              pointFromMessage(target_message, 2)};
          endpoint_reaches_goal =
              (point.position_world - target_position).norm() <=
              goalCompletionTolerance(*command_goal);
        }
        terminal_bundle_generation_.store(
            endpoint_reaches_goal && command_goal &&
                    command_goal->behavior ==
                        navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP
                ? sample.bundle->bundle_generation
                : 0U,
            std::memory_order_release);
      }
    }
    // The safety suffix contains the dynamically continuous main prefix up to
    // planner backend's backup switch plus the braking polynomial. Once frozen by a
    // failed hot replan, the whole suffix is safety-owned.
    if (sampled_command_valid && safety_suffix_active) on_backup_traj = true;
    if (traj_finish) trajectory_finished_.store(true);
  } else {
    // A rest-to-rest solve can fail before a command has ever been committed. Emit an
    // explicit terminal status so External Mode enters its PX4 Hold path;
    // never synthesize a zero-velocity nominal command from this state.
    pvaj.setZero();
  }

  if (command_world_identity.generation == 0U) {
    command_world_identity = world_snapshot_store_.load().identity;
  }
  navigation_contracts::msg::NavigationCommand command;
  command.header.frame_id = planning_frame_;
  command.header.stamp = navigation_common::secondsToRosTime(now_seconds).value_or(
      builtin_interfaces::msg::Time{});
  command.localization_epoch = localization_epoch_at_command;
  command.goal_epoch = goal_epoch_at_command;
  if (command_goal) {
    command.mission_id = command_goal->mission_id;
    command.waypoint_index = command_goal->waypoint_index;
    command.request_id = command_goal->request_id;
  }
  command.world_generation = command_world_identity.generation;
  command.world_revision = command_world_identity.revision;
  command.world_observation_stamp = navigation_common::nanosecondsToRosTime(
      command_world_identity.observation_stamp_ns).value_or(builtin_interfaces::msg::Time{});
  command.bundle_generation = trajectory_generation;
  const auto command_id = advanceMonotonicId(command_id_);
  if (!command_id) {
    command_bundle_store_.invalidate();
    planner_command_available_.store(false);
    planner_failure_latched_.store(true);
    RCLCPP_ERROR(get_logger(), "command sample id exhausted");
    return;
  }
  command.sample_id = *command_id;
  command.trajectory_time_s = trajectory_time_s;
  command.state_source_stamp = execution_state
      ? navigation_common::nanosecondsToRosTime(execution_state->state.source_stamp_ns).value_or(
          builtin_interfaces::msg::Time{})
      : builtin_interfaces::msg::Time{};
  const auto command_time_ns = command_ros_time.nanoseconds();
  const auto valid_until_ns = command_time_ns >
          std::numeric_limits<std::int64_t>::max() - command_stream_timeout_ns_
      ? std::numeric_limits<std::int64_t>::max()
      : command_time_ns + command_stream_timeout_ns_;
  command.valid_until = navigation_common::nanosecondsToRosTime(valid_until_ns).value_or(
      builtin_interfaces::msg::Time{});
  const bool main_trajectory_rejected = planner_failed && !on_backup_traj;
  command.status = main_trajectory_rejected
                       ? navigation_contracts::msg::NavigationCommand::STATUS_REJECTED
                       : traj_finish
                       ? navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED
                       : navigation_contracts::msg::NavigationCommand::STATUS_READY;
  command.role = main_trajectory_rejected
                     ? navigation_contracts::msg::NavigationCommand::ROLE_EMERGENCY
                     : on_backup_traj
                     ? navigation_contracts::msg::NavigationCommand::ROLE_BACKUP
                     : navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
  command.reason_code = main_trajectory_rejected ? 1U : 0U;
  command.position.x = pvaj(0, 0);
  command.position.y = pvaj(1, 0);
  command.position.z = pvaj(2, 0);
  command.velocity.x = pvaj(0, 1);
  command.velocity.y = pvaj(1, 1);
  command.velocity.z = pvaj(2, 1);
  command.acceleration.x = pvaj(0, 2);
  command.acceleration.y = pvaj(1, 2);
  command.acceleration.z = pvaj(2, 2);
  command.jerk.x = pvaj(0, 3);
  command.jerk.y = pvaj(1, 3);
  command.jerk.z = pvaj(2, 3);
  command.yaw = yaw;
  command.yaw_rate = yaw_dot;
  const auto publish_ros_command = [this, &command] {
    const auto publish_started = std::chrono::steady_clock::now();
    command_publisher_->publish(command);
    last_publish_us_.store(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - publish_started).count(),
        std::memory_order_release);
  };
  if (sampled_command_valid) {
    bool exposed = false;
    const auto store_publish_started = std::chrono::steady_clock::now();
    {
      // Recheck the non-store execution state immediately before entering the
      // store's pointer/world transaction. Sampling and message assembly are
      // intentionally outside this lock. The final store transaction keeps
      // this transition lock while the exposure callback runs so a lease or
      // goal transition cannot complete between validation and transport;
      // its duration is recorded as store_publish_us for latency monitoring.
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (localization_epoch_ready_.load(std::memory_order_acquire) &&
          active_localization_epoch_.load(std::memory_order_acquire) ==
              localization_epoch_at_command &&
          active_goal_epoch_.load(std::memory_order_acquire) == goal_epoch_at_command &&
          command_goal_epoch_.load(std::memory_order_acquire) == goal_epoch_at_command &&
          planner_command_available_.load(std::memory_order_acquire) &&
          command_execution_lease_failure_latch_.allowsCommandExposure()) {
        exposed = command_bundle_store_.publishIfCurrent(
            sampled_bundle, goal_epoch_at_command, publish_ros_command);
      }
    }
    last_command_store_publish_us_.store(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - store_publish_started).count(),
        std::memory_order_release);
    if (!exposed) {
      // The world or goal advanced between sampling and publication. Never
      // expose the stale pointer. Mapping recertification deliberately copies
      // the same immutable bundle with a newer world identity, however, and a
      // planner commit may also supersede this sample. Preserve availability
      // only when the store now owns a non-older, valid bundle for the same
      // active epochs; the next timer tick will sample and transactionally
      // expose that exact pointer. Every invalidation/epoch/lease case still
      // clears the command fail-closed.
      const auto current_bundle = command_bundle_store_.load();
      const auto current_localization_epoch =
          active_localization_epoch_.load(std::memory_order_acquire);
      const auto current_goal_epoch =
          active_goal_epoch_.load(std::memory_order_acquire);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const bool superseded_by_valid_bundle = current_bundle &&
          supersedingBundleMayRemainAvailable(
              sampled_bundle ? sampled_bundle->bundle_generation : 0U,
              current_bundle->bundle_generation,
              current_bundle->localization_epoch,
              current_bundle->goal_epoch,
              current_localization_epoch,
              current_goal_epoch,
              current_bundle->valid_until_ns,
              command_ros_time.nanoseconds(),
              current_bundle->valid(),
              planner_failure_latched_.load(std::memory_order_acquire),
              command_execution_lease_failure_latch_.allowsCommandExposure());
      if (!superseded_by_valid_bundle) {
        planner_command_available_.store(false);
        command_goal_epoch_.store(0U);
        safety_suffix_active_.store(false);
      }
      return;
    }
  } else {
    publish_ros_command();
  }
  ++command_publish_count_;
  ++cycle_success_count_;
}

}  // namespace navigation_runtime
