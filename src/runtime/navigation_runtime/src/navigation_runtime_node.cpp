#include "navigation_runtime/navigation_runtime_node.hpp"
#include "navigation_runtime/mission_dynamics.hpp"
#include "navigation_runtime/mapping_fail_stop.hpp"
#include "navigation_runtime/commit_trace.hpp"

#include <navigation_mapping/mapping_actor.hpp>

#include "navigation_runtime/planner_fsm.hpp"
#include "navigation_runtime/planning_supervisor.hpp"
#include "navigation_runtime/planning_worker.hpp"
#include "navigation_runtime/certified_continuation.hpp"
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

bool goalIdentityNewer(const navigation_contracts::msg::NavigationGoal& candidate,
                       const navigation_contracts::msg::NavigationGoal& current) {
  if (candidate.mission_id != current.mission_id) return false;
  if (candidate.request_id != current.request_id) {
    return candidate.request_id > current.request_id;
  }
  if (candidate.waypoint_index != current.waypoint_index) {
    return candidate.waypoint_index > current.waypoint_index;
  }
  return candidate.route.route_revision > current.route.route_revision;
}

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

std::optional<std::uint64_t> certifiedMainContinuationBoundary(
    const navigation_planning::CandidateBundle& candidate,
    const navigation_contracts::msg::NavigationGoal& goal,
    const std::uint64_t localization_epoch,
    const std::uint64_t goal_epoch,
    const bool trajectory_finished) noexcept {
  if (goal.behavior != navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH ||
      !candidate.route_boundary_event.has_value() ||
      !candidate.route_boundary_constraint.has_value()) {
    return std::nullopt;
  }
  const auto route = decodeRouteSnapshot(goal);
  const auto& boundary = *candidate.route_boundary_event;
  if (!route) {
    return std::nullopt;
  }
  const auto boundary_sample = candidate.sampleAtDeclaredStamp(boundary.boundary_stamp_ns);
  if (!boundary_sample) {
    return std::nullopt;
  }

  const auto boundary_offset_ns = boundary.boundary_stamp_ns - candidate.declared_start_ns;
  const auto canonical_offset_ns = [](const double seconds)
      -> std::optional<std::int64_t> {
    if (!std::isfinite(seconds) || seconds < 0.0) return std::nullopt;
    const long double nanos = static_cast<long double>(seconds) * 1.0e9L;
    if (!std::isfinite(nanos) || nanos < 0.0L ||
        nanos > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      return std::nullopt;
    }
    return static_cast<std::int64_t>(std::llround(nanos));
  };
  // Use the producer's canonical role interval rather than a duration magic
  // number: the boundary must have a strictly positive MAIN remainder before
  // the role interval ends (backup switch or declared endpoint).
  std::int64_t main_interval_begin_ns = -1;
  std::int64_t main_interval_end_ns = -1;
  for (const auto& interval : candidate.role_schedule) {
    const auto begin_ns = canonical_offset_ns(interval.begin_time_s);
    const auto end_ns = canonical_offset_ns(interval.end_time_s);
    if (begin_ns && end_ns && interval.role == navigation_planning::CandidateRole::kMain &&
        boundary_offset_ns >= *begin_ns && boundary_offset_ns < *end_ns) {
      main_interval_begin_ns = *begin_ns;
      main_interval_end_ns = *end_ns;
      break;
    }
  }
  const CertifiedMainContinuationBoundaryFacts facts{
      true,
      navigation_mission::passThroughNextWaypointIsCoincidentStop(*route),
      trajectory_finished,
      true,
      true,
      candidate.localization_epoch,
      localization_epoch,
      candidate.goal_epoch,
      goal_epoch,
      candidate.request_id,
      goal.request_id,
      boundary.kind,
      boundary_sample->role,
      boundary.junction_index,
      candidate.route_boundary_constraint->junction_index,
      goal.waypoint_index,
      boundary.boundary_stamp_ns,
      candidate.declared_start_ns,
      candidate.declared_end_ns,
      main_interval_begin_ns,
      main_interval_end_ns};
  if (!certifiedMainContinuationBoundaryEligible(facts)) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(boundary.boundary_stamp_ns);
}

bool completionWitnessMatchesCurrentExecution(
    const TrajectoryCompletionWitness& witness,
    const navigation_execution::ExecutionTimelineSnapshot& timeline,
    const std::shared_ptr<const navigation_planning::CandidateBundle>& expected_bundle,
    const std::optional<navigation_contracts::msg::NavigationGoal>& executing_goal,
    const std::uint64_t command_goal_epoch,
    const std::uint64_t localization_epoch,
    const bool episode_failure_latched,
    const bool command_exposure_allowed) noexcept {
  if (episode_failure_latched || !command_exposure_allowed || !expected_bundle ||
      !timeline.active ||
      timeline.active.get() != expected_bundle.get() || !executing_goal ||
      executing_goal->mission_id != witness.mission_id ||
      executing_goal->waypoint_index != witness.waypoint_index ||
      executing_goal->request_id != witness.request_id ||
      command_goal_epoch != witness.goal_epoch ||
      localization_epoch != witness.localization_epoch) {
    return false;
  }
  return completionWitnessMatches(
      witness, timeline.active->bundle_generation,
      timeline.version,
      timeline.active->localization_epoch, timeline.active->goal_epoch,
      timeline.active->request_id, executing_goal->mission_id,
      executing_goal->waypoint_index);
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
    const bool safety_suffix_active,
    const bool terminal_stop) {
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
  output << ",\"terminal_stop\":" << (terminal_stop ? "true" : "false");
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
  const auto inject_failed_replan_cycle_id = declare_parameter(
      "navigation_runtime.inject_failed_replan_cycle_id", std::int64_t{0});
  inject_failed_replan_cycle_id_ = inject_failed_replan_cycle_id > 0
      ? static_cast<std::uint64_t>(inject_failed_replan_cycle_id) : 0U;
  inject_failed_replan_once_ = declare_parameter(
      "navigation_runtime.inject_failed_replan_once", false);
  inject_failed_replan_when_safe_ = declare_parameter(
      "navigation_runtime.inject_failed_replan_when_safe", false);
  inject_failed_replan_after_handoff_ = declare_parameter(
      "navigation_runtime.inject_failed_replan_after_handoff", false);
  inject_failed_replan_repeated_ = declare_parameter(
      "navigation_runtime.inject_failed_replan_repeated", false);
  inject_failed_plan_from_rest_repeated_ = declare_parameter(
      "navigation_runtime.inject_failed_plan_from_rest_repeated", false);
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
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/navigation/diagnostics", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
  std::optional<navigation_planning::DynamicLimits> mission_limits;
  if (!mission_file.empty()) {
    mission_limits = loadMissionDynamicLimits(mission_file, planning_frame_);
    mission_dynamic_limits_ = *mission_limits;
    RCLCPP_INFO(
        get_logger(),
        "Applying mission dynamics to planner backend before optimizer construction: "
        "velocity=%.3f acceleration=%.3f jerk=%.3f",
        mission_limits->intent.requested_cruise_speed_mps,
        mission_limits->vehicle.maximum_acceleration_mps2,
        mission_limits->vehicle.maximum_jerk_mps3);
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
          nullptr, std::nullopt, 0U, 0};
      if (pending.message->sensor_origin_valid) {
        const auto& origin = pending.message->sensor_origin_pose.position;
        observation.sensor_origin_world =
            navigation_world_model::Point3{origin.x, origin.y, origin.z};
        observation.sensor_origin_localization_epoch =
            pending.localization_epoch;
        observation.sensor_origin_stamp_ns = pending.stamp_ns;
      }
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
        // Recertification consumes only the immutable CandidateBundle snapshot
        // and the immutable WorldModelView published above.  Never hold the
        // PlanningWorker/backend mutex across this callback: map ingestion
        // must not serialize behind optimizer work.  The execution store's
        // publishAndFinalize gate below linearizes the retained pointer with
        // the world identity; a concurrent planner commit is either retained
        // by its exact generation/protected-region proof or rejected on the
        // next immutable callback.
        const auto execution_timeline = command_store->snapshot();
        const auto expected_bundle = execution_timeline.active;
        const auto expected_pending = execution_timeline.pending;
        std::optional<navigation_contracts::msg::NavigationGoal> expected_goal;
        if (expected_bundle) {
          std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
          std::lock_guard<std::mutex> input_lock(input_mutex_);
          const bool executing_matches = executing_goal_ &&
              executing_goal_->request_id == expected_bundle->request_id &&
              expected_bundle->goal_epoch ==
                  command_goal_epoch_.load(std::memory_order_acquire) &&
              active_localization_epoch_.load(std::memory_order_acquire) ==
                  expected_bundle->localization_epoch;
          const bool desired_matches = active_goal_ &&
              active_goal_->request_id == expected_bundle->request_id &&
              active_goal_epoch_.load(std::memory_order_acquire) ==
                  expected_bundle->goal_epoch &&
              active_localization_epoch_.load(std::memory_order_acquire) ==
                  expected_bundle->localization_epoch;
          if (executing_matches) {
            expected_goal = *executing_goal_;
          } else if (desired_matches) {
            expected_goal = *active_goal_;
          }
        }
        bool retain_validated_bundle = false;
        bool retain_validated_pending = false;
        if (expected_pending && expected_pending->valid() &&
            expected_pending->protected_region.valid() &&
            expected_pending->world_identity.localization_epoch ==
                result.snapshot->identity().localization_epoch &&
            expected_pending->world_identity.generation ==
                result.snapshot->identity().generation &&
            expected_pending->world_identity.revision <
                result.snapshot->identity().revision &&
            !result.snapshot->changedRegionIntersectsSince(
                expected_pending->world_identity,
                expected_pending->protected_region)) {
          retain_validated_pending = true;
        }
        if (expected_pending && expected_pending->valid() &&
            !retain_validated_pending) {
          // Changed-region provenance is only a fast path.  A pending
          // successor has a planner-owned immutable candidate as well, so
          // revalidate that exact generation before dropping it.  This keeps
          // a valid scheduled handover alive when a LiDAR update touches the
          // protected region but does not actually block the candidate.
          if (planner_) {
            const double authorization_wall_time_s = ros_clock->now().seconds();
            const auto validation = planner_->validateStagedCommandCandidate(
                result.snapshot, authorization_wall_time_s,
                expected_pending->bundle_generation);
            if (validation.valid) {
              retain_validated_pending = true;
              ++next.command_revalidation_full_count;
              RCLCPP_DEBUG(
                  this->get_logger(),
                  "retaining pending generation=%lu after full immutable candidate "
                  "revalidation samples=%lu",
                  static_cast<unsigned long>(expected_pending->bundle_generation),
                  static_cast<unsigned long>(validation.sample_count));
            } else {
              RCLCPP_WARN(
                  this->get_logger(),
                  "pending generation=%lu failed immutable candidate revalidation "
                  "failure=%d blocked_role=%d samples=%lu; dropping on world revision=%lu",
                  static_cast<unsigned long>(expected_pending->bundle_generation),
                  validation.failure_code, validation.blocked_role,
                  static_cast<unsigned long>(validation.sample_count),
                  static_cast<unsigned long>(result.snapshot->identity().revision));
            }
          } else {
            RCLCPP_WARN(
                this->get_logger(),
                "pending successor has no planner validator; dropping generation=%lu "
                "on world revision=%lu",
                static_cast<unsigned long>(expected_pending->bundle_generation),
                static_cast<unsigned long>(result.snapshot->identity().revision));
          }
        }
        const auto terminal_generation = terminal_bundle_generation_.load(
            std::memory_order_acquire);
        const bool terminal_bundle_observed = expected_bundle &&
            terminal_generation != 0U &&
            expected_bundle->bundle_generation == terminal_generation;
        const auto terminal_endpoint = expected_bundle
            ? expected_bundle->sampleAtDeclaredEnd() : std::nullopt;
        const auto endpoint_declared_end_ns = expected_bundle &&
            expected_bundle->declared_end_ns > 0
            ? std::optional<std::int64_t>{expected_bundle->declared_end_ns}
            : expected_bundle
                ? navigation_common::secondsToNanoseconds(
                      expected_bundle->start_wall_time_s + expected_bundle->duration_s)
                : std::nullopt;
        const auto endpoint_execution_state = execution_state_store_.load();
        const auto endpoint_execution_freshness = endpoint_execution_state
            ? navigation_contracts::evaluateExecutionStateFreshness(
                  ros_clock->now().nanoseconds(),
                  endpoint_execution_state->state.source_stamp_ns,
                  navigation_common::steadyClockNowNanoseconds(),
                  endpoint_execution_state->state.receive_stamp_ns,
                  data_freshness_window_s_)
            : navigation_contracts::ExecutionStateFreshness{};
        const bool endpoint_near_current_execution = endpoint_execution_state &&
            endpoint_execution_freshness.valid() && terminal_endpoint &&
            endpoint_execution_state->state.localization_epoch ==
                expected_bundle->localization_epoch &&
            (terminal_endpoint->position_world -
                endpoint_execution_state->state.position_world).norm() <=
                navigation_contracts::kCommandAnchorErrorLimitM;
        const bool expired_recovery_endpoint = expected_bundle &&
            endpoint_declared_end_ns &&
            ros_clock->now().nanoseconds() >= *endpoint_declared_end_ns &&
            terminal_endpoint && terminal_endpoint->finished &&
            terminal_endpoint->finite() && endpoint_near_current_execution &&
            navigation_world_model::isCellTraversable(
                result.snapshot->classify(
                    terminal_endpoint->position_world,
                    navigation_world_model::GridLayer::kInflated),
                navigation_world_model::UnknownPolicy::kRequireKnownFree) &&
            ((expected_bundle->kind ==
                  navigation_planning::CandidateBundleKind::kMainWithBackup &&
              terminal_endpoint->role == navigation_planning::CandidateRole::kBackup) ||
             (expected_bundle->kind ==
                  navigation_planning::CandidateBundleKind::kEmergencyBrake &&
              terminal_endpoint->role == navigation_planning::CandidateRole::kEmergency));
        const bool effective_terminal_route = expected_goal && [&] {
          const auto route = decodeRouteSnapshot(*expected_goal);
          return route.has_value() &&
              (expected_goal->behavior ==
                   navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP ||
               navigation_mission::passThroughNextWaypointIsCoincidentStop(*route));
        }();
        const auto terminal_declared_end_ns = endpoint_declared_end_ns;
        const bool expired_terminal_endpoint = expected_bundle && expected_goal &&
            effective_terminal_route && terminal_endpoint &&
            terminal_declared_end_ns &&
            ros_clock->now().nanoseconds() >= *terminal_declared_end_ns &&
            terminal_endpoint->finished && terminal_endpoint->finite() &&
            (terminal_endpoint->position_world -
                Eigen::Vector3d{pointFromMessage(plannerTarget(*expected_goal), 0),
                                pointFromMessage(plannerTarget(*expected_goal), 1),
                                pointFromMessage(plannerTarget(*expected_goal), 2)}).norm() <=
                goalCompletionTolerance(*expected_goal) &&
            terminalStopEndpointContractValid(
                effective_terminal_route, expected_bundle->kind,
                expected_bundle->role, terminal_endpoint->role);
        if (terminal_bundle_observed || expired_terminal_endpoint) {
          // The executable lease may have ended by the time this map callback
          // runs. An observed or already-expired terminal command needs one
          // final safe handover sample, not a revalidation of future
          // trajectory time. Keep this exception narrow: only the declared
          // endpoint is sampled, and it must still be finite and known-free
          // in the new snapshot. The expired-endpoint branch closes the
          // callback-order race where the command publisher has just emitted
          // its terminal sample but has not yet published the derived witness.
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
        } else if (expired_recovery_endpoint) {
          // A non-terminal BACKUP or EMERGENCY endpoint can expire between
          // the command timer and this mapping callback. Preserve only the
          // endpoint witness so CommandSampler can emit its bounded
          // STOPPED_HOLD; the next planner cycle must observe the measured
          // stop and restart from the current state. This never extends the
          // polynomial lease or treats the endpoint as mission completion.
          retain_validated_bundle = true;
          ++next.command_revalidation_fast_path_count;
          RCLCPP_INFO(
              this->get_logger(),
              "retaining expired recovery endpoint generation=%lu as bounded "
              "STOPPED_HOLD endpoint=(%.3f,%.3f,%.3f)",
              static_cast<unsigned long>(expected_bundle->bundle_generation),
              terminal_endpoint->position_world.x(),
              terminal_endpoint->position_world.y(),
              terminal_endpoint->position_world.z());
        } else if (expected_bundle && expected_bundle->protected_region.valid() &&
                   expected_bundle->world_identity.localization_epoch ==
                       result.snapshot->identity().localization_epoch &&
                   expected_bundle->world_identity.generation ==
                       result.snapshot->identity().generation &&
                   expected_bundle->world_identity.revision <=
                       result.snapshot->identity().revision &&
                   !result.snapshot->changedRegionIntersectsSince(
                       expected_bundle->world_identity,
                       expected_bundle->protected_region)) {
          // Mapping owns only immutable world/candidate values.  A planner
          // backend validation call here would race with optimizer state and
          // reintroduce the callback-wide mutex that this path is designed to
          // avoid.  The changed-region proof is therefore the only retention
          // fast path; an intersecting/ambiguous update fails closed.
          retain_validated_bundle = true;
          ++next.command_revalidation_fast_path_count;
          RCLCPP_DEBUG(
              this->get_logger(),
              "retaining generation=%lu on immutable disjoint-region proof",
              static_cast<unsigned long>(expected_bundle->bundle_generation));
        } else if (expected_bundle && planner_) {
          // The changed-region proof is a fast path, not the only safe path.
          // LiDAR updates naturally touch the currently traversed corridor in
          // open space, so rejecting every intersecting update would erase a
          // still-safe command at the next observation.  Revalidate the exact
          // execution generation against the new immutable snapshot before
          // deciding whether the command must be removed.  Prefer the staged
          // candidate while activation has not yet synchronized planner
          // history; otherwise validate the exact committed planner copy.
          const double authorization_wall_time_s = ros_clock->now().seconds();
          const auto staged_validation = planner_->validateStagedCommandCandidate(
              result.snapshot, authorization_wall_time_s,
              expected_bundle->bundle_generation);
          const auto committed_validation = staged_validation.valid
              ? navigation_planning::TrajectoryValidationResult{}
              : planner_->validateCommittedTrajectory(
                    result.snapshot, authorization_wall_time_s,
                    expected_bundle->bundle_generation);
          const auto& validation = staged_validation.valid
              ? staged_validation : committed_validation;
          if (validation.valid) {
            retain_validated_bundle = true;
            ++next.command_revalidation_full_count;
            RCLCPP_DEBUG(
                this->get_logger(),
                "retaining generation=%lu after full immutable candidate revalidation "
                "stage=%d samples=%lu",
                static_cast<unsigned long>(expected_bundle->bundle_generation),
                staged_validation.valid ? 1 : 2,
                static_cast<unsigned long>(validation.sample_count));
          } else {
            RCLCPP_WARN(
                this->get_logger(),
                "active generation=%lu failed full immutable candidate revalidation "
                "failure=%d blocked_role=%d samples=%lu",
                static_cast<unsigned long>(expected_bundle->bundle_generation),
                validation.failure_code, validation.blocked_role,
                static_cast<unsigned long>(validation.sample_count));
          }
        }
        // The immutable world publication and the dependent execution
        // certificate transition share one gate.  A retained bundle is copied
        // with the new identity only when the exact pointer was validated on
        // this snapshot; otherwise it is cleared fail-closed.
        const auto publication_decision = store->publishAndFinalizeDecision(
            result.snapshot,
            [command_store, expected_bundle, expected_pending,
             execution_timeline_version = execution_timeline.version,
             retain_validated_bundle, retain_validated_pending,
             identity = result.snapshot->identity(),
             refreshed_valid_until_ns = [&] {
               const auto now_ns = ros_clock->now().nanoseconds();
               return now_ns > std::numeric_limits<std::int64_t>::max() -
                          data_freshness_window_ns_
                          ? std::numeric_limits<std::int64_t>::max()
                          : now_ns + data_freshness_window_ns_;
             }()] {
              return command_store->publishWorldIdentityIfCurrent(
                  identity, execution_timeline_version,
                  expected_bundle, retain_validated_bundle,
                  refreshed_valid_until_ns,
                  expected_pending, retain_validated_pending);
            });
        if (publication_decision == navigation_world_model::WorldCommitDecision::kSuperseded) {
          next.map_update_us = result.map_update_us;
          next.mapping_callback_total_us =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - callback_started).count();
          telemetry->recordUpdate(std::move(next));
          RCLCPP_DEBUG(
              this->get_logger(),
              "world refresh superseded by a newer execution timeline; preserving active/pending bundle");
          return;
        }
        if (publication_decision != navigation_world_model::WorldCommitDecision::kCommitted) {
          command_store->invalidate();
          next.map_update_us = result.map_update_us;
          next.mapping_callback_total_us =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::steady_clock::now() - callback_started).count();
          telemetry->recordUpdate(std::move(next));
          RCLCPP_ERROR(
              this->get_logger(),
              "world snapshot publication could not finalize the execution certificate "
              "decision=%d; mapping worker will fail-stop because the new world was not published",
              static_cast<int>(publication_decision));
          throw std::runtime_error(
              "world snapshot publication could not finalize its execution certificate");
        }
        if (expected_bundle && retain_validated_bundle) {
          const auto recertified_bundle = command_store->load();
          const auto now_ns = ros_clock->now().nanoseconds();
          const auto episode = execution_episode_.snapshot();
          const auto suspended_generation =
              world_freshness_suspended_bundle_generation_.load(
                  std::memory_order_acquire);
          if (recertified_bundle && worldFreshnessSuspendedCommandMayResume(
                  suspended_generation,
                  recertified_bundle->bundle_generation,
                  recertified_bundle->localization_epoch,
                  recertified_bundle->goal_epoch,
                  active_localization_epoch_.load(std::memory_order_acquire),
                  recertified_bundle->goal_epoch,
                  recertified_bundle->valid_until_ns,
                  now_ns,
                  recertified_bundle->valid(),
                  episode.failure_latched,
                  command_execution_lease_failure_latch_.allowsCommandExposure())) {
            std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
            std::lock_guard<std::mutex> input_lock(input_mutex_);
            std::lock_guard<std::mutex> command_lock(
                command_execution_lease_failure_latch_.transitionMutex());
            const auto current_executing_goal = executing_goal_;
            if (current_executing_goal &&
                executingCommandIdentityMatchesLocked(
                    *current_executing_goal, recertified_bundle->goal_epoch,
                    recertified_bundle->localization_epoch,
                    recertified_bundle->bundle_generation) &&
                worldFreshnessSuspendedCommandMayResume(
                    world_freshness_suspended_bundle_generation_.load(
                        std::memory_order_acquire),
                    recertified_bundle->bundle_generation,
                    recertified_bundle->localization_epoch,
                    recertified_bundle->goal_epoch,
                    active_localization_epoch_.load(std::memory_order_acquire),
                    recertified_bundle->goal_epoch,
                    recertified_bundle->valid_until_ns,
                    now_ns,
                    recertified_bundle->valid(),
                    execution_episode_.snapshot().failure_latched,
                    command_execution_lease_failure_latch_.allowsCommandExposure())) {
              command_goal_epoch_.store(
                  recertified_bundle->goal_epoch, std::memory_order_release);
              execution_episode_.roleObserved(
                  recertified_bundle->role, recertified_bundle->bundle_generation);
              execution_episode_.setSafetySuffix(
                  world_freshness_suspended_safety_suffix_active_.load(
                      std::memory_order_acquire));
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
        if (expected_bundle && !retain_validated_bundle) {
          // The latest immutable map invalidated the currently exposed
          // command. This is recoverable only through a new measured-state
          // PlanFromRest solve; allowing the next timer tick to enter
          // ReplanOnce with no committed bundle turns a map change into an
          // unconditional emergency result and prevents recovery.
          bool invalidated_current = false;
          {
            std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
            std::lock_guard<std::mutex> input_lock(input_mutex_);
            std::lock_guard<std::mutex> command_lock(
                command_execution_lease_failure_latch_.transitionMutex());
            const auto current_bundle = command_store->load();
            const bool exact_goal = expected_goal && executing_goal_ &&
                executing_goal_->mission_id == expected_goal->mission_id &&
                executing_goal_->waypoint_index == expected_goal->waypoint_index &&
                executing_goal_->request_id == expected_goal->request_id;
            const bool exact_epoch =
                expected_bundle->localization_epoch ==
                    active_localization_epoch_.load(std::memory_order_acquire) &&
                expected_bundle->goal_epoch ==
                    command_goal_epoch_.load(std::memory_order_acquire);
            const bool exact_bundle = current_bundle &&
                current_bundle.get() == expected_bundle.get() &&
                current_bundle->bundle_generation == expected_bundle->bundle_generation;
            if (exact_goal && exact_epoch && exact_bundle) {
              execution_episode_.clearRestartFromRest();
              hot_goal_transition_ = false;
              skip_replan_once_.store(false, std::memory_order_release);
              pending_goal_owner_.clearGoal();
              (void)command_store->invalidateIfCurrent(execution_timeline);
              failClosedLocked();
              command_goal_epoch_.store(0U);
              invalidated_current = true;
            }
          }
          if (invalidated_current) {
            RCLCPP_WARN(
                this->get_logger(),
                "world revision=%lu invalidated active command generation=%lu; "
                "entering PX4 Hold because no latest-world-certified brake remains",
                static_cast<unsigned long>(result.world_revision),
                static_cast<unsigned long>(expected_bundle->bundle_generation));
          } else {
            RCLCPP_DEBUG(
                this->get_logger(),
                "world invalidation became stale before lifecycle transition; preserving newer goal");
          }
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
  const bool mapping_requires_sensor_origin = mapping_configuration.sensor_origin_required;
  auto validate_mapping = [ros_clock, telemetry = mapping_telemetry_,
                           active_epoch = &active_localization_epoch_,
                           maximum_age_ns = data_freshness_window_ns_,
                           mapping_requires_sensor_origin](
      const PendingRegisteredScan& pending) {
    if (!pending.message) {
      telemetry->recordDiscard(false, false, true);
      return false;
    }
    const auto& pose = pending.message->corrected_pose.pose;
    const Eigen::Quaterniond q{
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
    const auto& sensor_origin = pending.message->sensor_origin_pose;
    const double sensor_orientation_norm = std::sqrt(
        sensor_origin.orientation.w * sensor_origin.orientation.w +
        sensor_origin.orientation.x * sensor_origin.orientation.x +
        sensor_origin.orientation.y * sensor_origin.orientation.y +
        sensor_origin.orientation.z * sensor_origin.orientation.z);
    const bool missing_sensor_origin = mapping_requires_sensor_origin &&
        !pending.message->sensor_origin_valid;
    const bool invalid_sensor_origin = pending.message->sensor_origin_valid &&
        (!std::isfinite(sensor_origin.position.x) ||
         !std::isfinite(sensor_origin.position.y) ||
         !std::isfinite(sensor_origin.position.z) ||
         !std::isfinite(sensor_orientation_norm) || sensor_orientation_norm <= 1.0e-9);
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
                         missing_sensor_origin || invalid_sensor_origin ||
                         freshness == navigation_execution::TimestampFreshness::INVALID;
    const bool stale = freshness == navigation_execution::TimestampFreshness::STALE;
    const bool future = freshness == navigation_execution::TimestampFreshness::FUTURE;
    const auto rejection_reason = missing_sensor_origin
        ? navigation_mapping::MappingObservationRejectionReason::kMissingSensorOrigin
        : invalid_sensor_origin
            ? navigation_mapping::MappingObservationRejectionReason::kSensorOriginContractMismatch
            : invalid
                ? navigation_mapping::MappingObservationRejectionReason::kMalformedObservation
                : navigation_mapping::MappingObservationRejectionReason::kNone;
    if (invalid || stale || future) {
      telemetry->recordDiscard(stale, future, invalid, rejection_reason);
    }
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
        const auto add_signed_value = [&status](const std::string& key,
                                                std::int64_t value) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = std::to_string(value);
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
        add_value("mapping_traversed_segment_count", map.traversed_segment_count);
        add_value("mapping_discarded_missing_sensor_origin",
                  mapping.discarded_missing_sensor_origin);
        add_value("mapping_discarded_sensor_origin_contract",
                  mapping.discarded_sensor_origin_contract);
        add_signed_value("mapping_traversed_latest_stamp_ns",
                         map.traversed_latest_stamp_ns);
        add_value("mapping_traversed_chain_reset_reason",
                  map.traversed_chain_reset_reason);
        add_value("mapping_traversed_latest_age_ms",
                  std::isfinite(map.traversed_latest_age_s)
                      ? static_cast<std::uint64_t>(std::max(0.0,
                          map.traversed_latest_age_s) * 1000.0) : 0U);
        add_signed_value("mapping_sensor_origin_x_mm",
                         std::isfinite(map.sensor_origin_world.x())
                             ? static_cast<std::int64_t>(std::llround(
                                 map.sensor_origin_world.x() * 1000.0)) : 0);
        add_signed_value("mapping_sensor_origin_y_mm",
                         std::isfinite(map.sensor_origin_world.y())
                             ? static_cast<std::int64_t>(std::llround(
                                 map.sensor_origin_world.y() * 1000.0)) : 0);
        add_signed_value("mapping_sensor_origin_z_mm",
                         std::isfinite(map.sensor_origin_world.z())
                             ? static_cast<std::int64_t>(std::llround(
                                 map.sensor_origin_world.z() * 1000.0)) : 0);
        add_signed_value("mapping_base_pose_x_mm",
                         std::isfinite(map.base_pose_world.x())
                             ? static_cast<std::int64_t>(std::llround(
                                 map.base_pose_world.x() * 1000.0)) : 0);
        add_signed_value("mapping_base_pose_y_mm",
                         std::isfinite(map.base_pose_world.y())
                             ? static_cast<std::int64_t>(std::llround(
                                 map.base_pose_world.y() * 1000.0)) : 0);
        add_signed_value("mapping_base_pose_z_mm",
                         std::isfinite(map.base_pose_world.z())
                             ? static_cast<std::int64_t>(std::llround(
                                 map.base_pose_world.z() * 1000.0)) : 0);
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
  const double solve_deadline_s = planner_->solveDeadlineSeconds();
  if (!plannerPeriodCoversSolveBudget(planner_rate_hz_, solve_deadline_s)) {
    throw std::invalid_argument(
        "navigation_runtime.planner_rate_hz must leave a complete timer period "
        "for planner.solve_deadline_s");
  }
  planning_worker_ = std::make_unique<
      PlanningWorker<navigation_planning_backend::PlannerFacade>>(
      std::move(planner), [this](std::exception_ptr failure) {
        std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
        std::lock_guard<std::mutex> input_lock(input_mutex_);
        std::lock_guard<std::mutex> command_lock(
            command_execution_lease_failure_latch_.transitionMutex());
        failClosedLocked();
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

void NavigationRuntimeNode::applyExecutionRecoveryEventLocked(
    const ExecutionRecoveryEvent event) noexcept {
  navigation_runtime::applyExecutionRecoveryEventLocked(
      execution_recovery_state_, event);
}

void NavigationRuntimeNode::failClosedLocked() noexcept {
  trajectory_completion_witness_.reset();
  trajectory_reaches_goal_.store(false, std::memory_order_release);
  terminal_bundle_generation_.store(0U, std::memory_order_release);
  execution_episode_.failClosed();
  execution_recovery_state_.store(
      ExecutionRecoveryState::kPx4Hold, std::memory_order_release);
}

bool NavigationRuntimeNode::desiredGoalIdentityMatchesLocked(
    const navigation_contracts::msg::NavigationGoal& goal,
    const std::uint64_t goal_epoch,
    const std::uint64_t localization_epoch,
    const std::uint64_t bundle_generation) const noexcept {
  if (!active_goal_ || active_goal_->mission_id != goal.mission_id ||
      active_goal_->waypoint_index != goal.waypoint_index ||
      active_goal_->request_id != goal.request_id ||
      active_goal_epoch_.load(std::memory_order_acquire) != goal_epoch ||
      active_localization_epoch_.load(std::memory_order_acquire) != localization_epoch) {
    return false;
  }
  if (bundle_generation == 0U) return true;
  const auto bundle = command_bundle_store_.load();
  return bundle && bundle->bundle_generation == bundle_generation &&
         bundle->goal_epoch == goal_epoch &&
         bundle->localization_epoch == localization_epoch &&
         bundle->request_id == goal.request_id;
}

bool NavigationRuntimeNode::executingCommandIdentityMatchesLocked(
    const navigation_contracts::msg::NavigationGoal& goal,
    const std::uint64_t goal_epoch,
    const std::uint64_t localization_epoch,
    const std::uint64_t bundle_generation) const noexcept {
  if (!executing_goal_ || executing_goal_->mission_id != goal.mission_id ||
      executing_goal_->waypoint_index != goal.waypoint_index ||
      executing_goal_->request_id != goal.request_id ||
      command_goal_epoch_.load(std::memory_order_acquire) != goal_epoch ||
      active_localization_epoch_.load(std::memory_order_acquire) != localization_epoch) {
    return false;
  }
  if (bundle_generation == 0U) return true;
  const auto bundle = command_bundle_store_.load();
  return bundle && bundle->bundle_generation == bundle_generation &&
         bundle->goal_epoch == goal_epoch &&
         bundle->localization_epoch == localization_epoch &&
         bundle->request_id == goal.request_id;
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
  execution_episode_.reset(localization_epoch);
  command_bundle_store_.invalidate();
  last_registered_scan_epoch_.store(localization_epoch, std::memory_order_release);
  last_registered_scan_sequence_.store(0U, std::memory_order_release);
  bool foreign_hold = false;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    // A localization epoch invalidates every goal identity, including a
    // request waiting behind a certified safety suffix.
    pending_goal_owner_.clearGoal();
    deferred_terminal_status_.reset();
    foreign_hold = foreign_mission_hold_after_stop_;
    const auto next_goal_epoch = advanceMonotonicId(active_goal_epoch_);
    if (next_goal_epoch) {
      (void)command_bundle_store_.setActiveGoalEpoch(*next_goal_epoch, false);
    } else {
      active_goal_.reset();
      executing_goal_.reset();
      pending_goal_owner_.clearGoal();
      deferred_terminal_status_.reset();
      command_bundle_store_.invalidate();
      RCLCPP_ERROR(get_logger(), "active goal epoch exhausted during localization reset");
    }
    if (foreign_hold) {
      // A localization reset must not revive the old mission after a foreign
      // control-authority violation. Keep the latch and discard its identity.
      active_goal_.reset();
      executing_goal_.reset();
    }
    new_goal_ = active_goal_.has_value();
    hot_goal_transition_ = false;
    execution_episode_.clearRestartFromRest();
    skip_replan_once_.store(false, std::memory_order_release);
    trajectory_completion_witness_.reset();
    trajectory_reaches_goal_.store(false);
    terminal_bundle_generation_.store(0U);
    // Keep the command path fail-closed until the new epoch publishes its
    // first valid world snapshot. This evidence barrier is recoverable: the
    // active goal may be planned again after that snapshot arrives.
    command_execution_lease_failure_latch_.resetForNewGoalWithinTransition();
    execution_recovery_state_.store(
        foreign_hold ? ExecutionRecoveryState::kPx4Hold
                     : ExecutionRecoveryState::kInitialHold,
        std::memory_order_release);
    if (foreign_hold) {
      failClosedLocked();
    }
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

void NavigationRuntimeNode::onGoal(
    const navigation_contracts::msg::NavigationGoal::ConstSharedPtr& message) {
  if (!message) {
    RCLCPP_ERROR(get_logger(), "rejected null navigation goal");
    return;
  }
  const auto route = decodeRouteSnapshot(*message);
  const bool route_mirrors_valid =
      route.has_value() && routeSnapshotMatchesGoalMirrors(*route, *message);
  if (!route.has_value() || !route_mirrors_valid) {
    // This message was never accepted as a lifecycle event.  It must not
    // revoke a certified active command or a newer pending goal.
    RCLCPP_ERROR(get_logger(), "rejected navigation goal: immutable route snapshot is invalid");
    return;
  }
  {
    std::unique_lock<std::mutex> localization_lock(localization_transition_mutex_);
    std::lock_guard<std::mutex> lock(input_mutex_);
    applyValidatedGoalLocked(message);
  }
        if (consumeForeignMissionCancelIfCurrent() && planning_worker_) {
          planning_worker_->cancelActive();
        }
}

void NavigationRuntimeNode::transitionForeignMissionLocked(
    const bool defer_until_certified_stop) {
  // This helper is called only with input_mutex_ -> transitionMutex() held.
  // A foreign mission is not ordered by request_id: it is a control-authority
  // violation.  While moving, preserve the certified suffix and defer the
  // same complete transition until measured stop; otherwise fail closed now.
  if (defer_until_certified_stop) {
    foreign_mission_hold_after_stop_ = true;
    pending_goal_owner_.clearGoal();
    deferred_terminal_status_.reset();
    return;
  }
  // The worker interrupt is deliberately post-action: cancelActive() may
  // enter backend code and must not run while lifecycle locks are held.
  foreign_cancel_target_epoch_ = active_goal_epoch_.load(std::memory_order_acquire);
  foreign_cancel_localization_epoch_ =
      active_localization_epoch_.load(std::memory_order_acquire);
  foreign_mission_cancel_pending_.store(true, std::memory_order_release);
  pending_goal_owner_.clearGoal();
  deferred_terminal_status_.reset();
  foreign_mission_hold_after_stop_ = false;
  const auto next_goal_epoch = advanceMonotonicId(active_goal_epoch_);
  if (next_goal_epoch) {
    foreign_cancel_transition_epoch_ = *next_goal_epoch;
    (void)command_bundle_store_.setActiveGoalEpoch(*next_goal_epoch, false);
  } else {
    command_bundle_store_.invalidate();
  }
  active_goal_.reset();
  executing_goal_.reset();
  command_bundle_store_.invalidate();
  new_goal_ = false;
  hot_goal_transition_ = false;
  execution_episode_.clearRestartFromRest();
  failClosedLocked();
  plan_from_rest_first_failure_steady_ns_ = 0;
  skip_replan_once_.store(false, std::memory_order_release);
  trajectory_reaches_goal_.store(false, std::memory_order_release);
  terminal_bundle_generation_.store(0U, std::memory_order_release);
  command_goal_epoch_.store(0U, std::memory_order_release);
}

bool NavigationRuntimeNode::consumeForeignMissionCancelIfCurrentLocked() {
  if (!foreign_mission_cancel_pending_.load(std::memory_order_acquire)) return false;
  const bool unchanged = !active_goal_.has_value() &&
      active_goal_epoch_.load(std::memory_order_acquire) ==
          foreign_cancel_transition_epoch_ &&
      active_localization_epoch_.load(std::memory_order_acquire) ==
          foreign_cancel_localization_epoch_;
  foreign_mission_cancel_pending_.store(false, std::memory_order_release);
  return unchanged;
}

bool NavigationRuntimeNode::consumeForeignMissionCancelIfCurrent() {
  std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
  std::lock_guard<std::mutex> input_lock(input_mutex_);
  return consumeForeignMissionCancelIfCurrentLocked();
}

void NavigationRuntimeNode::applyValidatedGoalLocked(
    const navigation_contracts::msg::NavigationGoal::ConstSharedPtr& message,
    const bool execution_transition_held) {
  // All command-lifecycle decisions are serialized after input_mutex_.  This
  // prevents a publisher from clearing suffix ownership between the suffix
  // read and pending-goal mutation.
  std::unique_lock<std::mutex> command_lock(
      command_execution_lease_failure_latch_.transitionMutex(), std::defer_lock);
  if (!execution_transition_held) command_lock.lock();
  if (foreign_mission_hold_after_stop_) {
    // A foreign mission latch is terminal for the current control authority.
    // No inbound goal may replace the cleared/held identity until an explicit
    // authority reset resolves the latch after measured-stop evidence.
    RCLCPP_ERROR(get_logger(),
                 "rejected navigation goal while foreign mission Hold latch is active");
    return;
  }
  // A planner backend plan is owned by the mission waypoint identity.  The
  // continuation point is only look-ahead metadata; treating it as the
  // current goal makes a repeated waypoint publication look like a new
  // request (or, conversely, hides the actual waypoint transition).
  const bool same_checkpoint = active_goal_.has_value() &&
      active_goal_->mission_id == message->mission_id &&
      active_goal_->waypoint_index == message->waypoint_index;
  const bool same_logical_goal = same_checkpoint &&
      active_goal_->request_id == message->request_id;
  if (active_goal_.has_value() && !same_logical_goal) {
    if (message->mission_id == active_goal_->mission_id &&
        !goalIdentityNewer(*message, *active_goal_)) {
      RCLCPP_WARN(get_logger(), "rejected stale/non-newer navigation goal request=%lu",
                  static_cast<unsigned long>(message->request_id));
      return;
    }
  }
  const auto episode = execution_episode_.snapshot();
  const bool previous_safety_suffix_active = episode.safety_suffix_active;
  const bool can_hot_retarget = canHotRetargetAtWaypointTransition(
      same_logical_goal,
      active_goal_.has_value() &&
          active_goal_->behavior ==
              navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH,
      episode.command_available, episode.failure_latched,
      episode.safety_suffix_active);
  bool effective_hot_retarget = can_hot_retarget;
  if (!same_logical_goal) {
    if (previous_safety_suffix_active) {
      // A moving BACKUP/EMERGENCY suffix owns the command until certified
      // stop. External Mode publishes a goal once, so returning here would
      // lose the request permanently. Keep one validated, monotonic pending
      // request without rebinding or revoking the suffix.
      if (message->mission_id != active_goal_->mission_id) {
        // A new mission has no ordering relation to the moving suffix. Keep
        // the certified suffix, but remember one control-authority violation
        // for the measured-stop transaction; do not queue or require retry.
        transitionForeignMissionLocked(true);
        RCLCPP_ERROR(get_logger(),
                     "foreign mission observed while safety suffix drains; "
                     "holding after certified stop active=%s incoming=%s",
                     active_goal_->mission_id.c_str(), message->mission_id.c_str());
        return;
      }
      const bool accepted_pending = pending_goal_owner_.enqueueGoal(
          message, active_goal_,
          episode.safety_suffix_active);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "retaining pending navigation goal request=%lu while safety suffix drains accepted=%d",
          static_cast<unsigned long>(message->request_id), accepted_pending ? 1 : 0);
      return;
    }
    if (active_goal_.has_value() &&
        message->mission_id != active_goal_->mission_id) {
      // Cross-mission transition is one atomic fail-closed boundary. The
      // previous command is not a certified moving suffix here, so invalidate
      // every owner (worker, store, pending slot, active identity, recovery).
      transitionForeignMissionLocked(false);
      RCLCPP_ERROR(get_logger(),
                   "rejected cross-mission goal and entered PX4 Hold");
      return;
    }
    // Cancel before exposing the new waypoint identity. The planner commit gate
    // guarantees that a solve for the previous waypoint cannot publish a new
    // candidate after this callback has invalidated it. A hot-retarget may keep
    // the current bundle only until a newer world identity invalidates it; the
    // execution store never relabels a stale-world certificate as current.
    if (planning_worker_) planning_worker_->cancelActive();
    const auto next_goal_epoch = advanceMonotonicId(active_goal_epoch_);
    if (!next_goal_epoch) {
      command_bundle_store_.invalidate();
      pending_goal_owner_.clearGoal();
      deferred_terminal_status_.reset();
      active_goal_.reset();
      executing_goal_.reset();
      failClosedLocked();
      RCLCPP_ERROR(get_logger(), "rejected navigation goal: active goal epoch exhausted");
      return;
    }
    const bool retain_bundle = can_hot_retarget;
    if (!command_bundle_store_.setActiveGoalEpoch(*next_goal_epoch, retain_bundle)) {
      effective_hot_retarget = false;
    }
  }
  active_goal_ = *message;
  if (!same_logical_goal) {
    // Global order is input_mutex_ -> execution transition. Command sampling
    // snapshots input and releases it before taking the transition lock.
    command_execution_lease_failure_latch_.resetForNewGoalWithinTransition();
    if (effective_hot_retarget) {
      // The retained command still belongs to the previous execution
      // identity.  The successor owns the new identity only after timeline
      // activation; do not relabel the old bundle here.
    } else {
      executing_goal_ = *message;
      command_goal_epoch_.store(0U);
    }
    plan_from_rest_first_failure_steady_ns_ = 0;
    hot_goal_transition_ = effective_hot_retarget;
    new_goal_ = !effective_hot_retarget;
    execution_episode_.clearRestartFromRest();
    skip_replan_once_.store(false, std::memory_order_release);
    trajectory_completion_witness_.reset();
    trajectory_reaches_goal_.store(false);
    terminal_bundle_generation_.store(0U);
    execution_recovery_state_.store(
        effective_hot_retarget ? ExecutionRecoveryState::kTrackMain
                               : ExecutionRecoveryState::kInitialHold,
        std::memory_order_release);
    execution_episode_.beginGoal(
        active_localization_epoch_.load(std::memory_order_acquire),
        active_goal_epoch_.load(std::memory_order_acquire),
        message->request_id, effective_hot_retarget);
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
  std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
  std::lock_guard<std::mutex> lock(input_mutex_);
  std::unique_lock<std::mutex> command_lock(
      command_execution_lease_failure_latch_.transitionMutex());
  const bool matches_pending = pending_goal_owner_.goalMatchesStatus(
      message->mission_id, message->waypoint_index, message->request_id);
  const bool matches_active = active_goal_.has_value() &&
      active_goal_->mission_id == message->mission_id &&
      active_goal_->waypoint_index == message->waypoint_index &&
      active_goal_->request_id == message->request_id;
  const bool safety_suffix_active =
      execution_episode_.snapshot().safety_suffix_active;
  if (pendingGoalTerminalStatusMayClear(matches_pending, safety_suffix_active)) {
    // A terminal status for a queued request consumes exactly that request;
    // it may never clear the active suffix unless the active identity is the
    // same complete {mission, waypoint, request} tuple.
    pending_goal_owner_.clearGoal();
  } else if (matches_pending) {
    // External Mode may report a terminal status for the already-published
    // next request before the current BACKUP/EMERGENCY suffix reaches its
    // measured stop. Keep that request in the sole pending owner; otherwise
    // the stop boundary has nothing to promote.
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "retaining terminal status for pending request=%lu while safety suffix drains",
        static_cast<unsigned long>(message->request_id));
  }
  if (!matches_active) {
    return;
  }
  if (safety_suffix_active) {
    // Preserve command identity while BACKUP/EMERGENCY drains. The terminal
    // transition is applied by the certified-stop boundary below.
    deferred_terminal_status_ = *active_goal_;
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
  executing_goal_.reset();
  pending_goal_owner_.clearGoal();
  deferred_terminal_status_.reset();
  execution_episode_.clearGoal(
      active_localization_epoch_.load(std::memory_order_acquire));
  foreign_mission_hold_after_stop_ = false;
  new_goal_ = false;
  hot_goal_transition_ = false;
  execution_episode_.clearRestartFromRest();
  skip_replan_once_.store(false, std::memory_order_release);
  command_goal_epoch_.store(0U);
  plan_from_rest_first_failure_steady_ns_ = 0;
  trajectory_completion_witness_.reset();
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
    const PlanningKey& scheduled_key,
    const std::optional<navigation_planning::CandidateBundle>& planned_candidate) {
  // Keep rejection evidence in the same structured cycle record as planner
  // evidence. These codes are diagnostics only: every non-success path below
  // remains fail-closed and leaves the execution-owned active timeline in
  // place.
  enum : int {
    kBoundaryAccepted = 0,
    kInvalidArguments = 1,
    kStaleIdentity = 2,
    kInvalidFreshnessWindow = 3,
    kCandidateExportFailed = 4,
    kWorldNotFresh = 5,
    kActivationWindow = 6,
    kNonExactSuccessorStart = 7,
    kInvalidCandidate = 8,
    kAnchorUnavailable = 9,
    kAnchorMismatch = 10,
    kEndpointMetadata = 11,
    kRouteBoundary = 12,
    kMainReserve = 13,
    kExecutionState = 14,
    kTransactionIdExhausted = 15,
    kStageOrCommitRejected = 16,
  };
  last_execution_boundary_rejection_.store(
      kBoundaryAccepted, std::memory_order_release);
  const auto reject = [this](const int reason) {
    last_execution_boundary_rejection_.store(reason, std::memory_order_release);
    return false;
  };
  if (now_ns <= 0 || goal_epoch == 0U || localization_epoch == 0U ||
      goal.request_id == 0U) {
    return reject(kInvalidArguments);
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
    return reject(kStaleIdentity);
  }
  const auto maximum_age_ns = data_freshness_window_ns_;
  if (maximum_age_ns <= 0 || now_ns > std::numeric_limits<std::int64_t>::max() - maximum_age_ns) {
    return reject(kInvalidFreshnessWindow);
  }
  // Successors are admitted for a future splice. The active command remains
  // the only executable object until this activation boundary is reached.
  // Measured-state emergency braking is different: it is the fail-closed
  // replacement for an already invalid/exceeded command and must take effect
  // immediately from its measured boundary.
  const bool has_active_bundle = static_cast<bool>(latest_bundle_before_commit);
  if (now_ns > std::numeric_limits<std::int64_t>::max() - maximum_age_ns) {
    planner_->discardCommandCandidate();
    return reject(kInvalidFreshnessWindow);
  }
  auto candidate = planned_candidate.has_value()
      ? planned_candidate
      : planner_->exportCommandCandidate(
            localization_epoch, goal_epoch, goal.request_id, now_ns,
            now_ns + maximum_age_ns);
  if (!candidate) {
    RCLCPP_WARN(get_logger(),
                "execution boundary rejected candidate export mission=%s waypoint=%u "
                "request=%lu now_ns=%lld",
                goal.mission_id.c_str(), goal.waypoint_index,
                static_cast<unsigned long>(goal.request_id),
                static_cast<long long>(now_ns));
    return reject(kCandidateExportFailed);
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
    return reject(kWorldNotFresh);
  }
  const bool emergency_candidate = candidate->kind ==
      navigation_planning::CandidateBundleKind::kEmergencyBrake;
  const auto execution_recovery_state = execution_recovery_state_.load(
      std::memory_order_acquire);
  // An expired recovery endpoint is a bounded STOPPED_HOLD, not a future
  // execution timeline. A measured-state replacement must cut over now and
  // validate its first sample against measured state; reserving an anchor on
  // the expired polynomial would reject the recovery that resumes the mission.
  const bool replacing_stopped_recovery =
      execution_recovery_state == ExecutionRecoveryState::kStoppedRecovery;
  const bool schedule_successor = has_active_bundle && !emergency_candidate &&
                                  !replacing_stopped_recovery;
  const auto activation_ns = schedule_successor
      ? candidate->declared_start_ns
      : now_ns;
  const auto activation_guard_ns = static_cast<std::int64_t>(
      navigation_planning::PlanningTimingContract::kCommitGuardS * 1.0e9);
  if (activation_ns <= 0 ||
      activation_ns > std::numeric_limits<std::int64_t>::max() - maximum_age_ns ||
      (schedule_successor &&
       (activation_guard_ns <= 0 ||
        now_ns > std::numeric_limits<std::int64_t>::max() - activation_guard_ns ||
        activation_ns <= now_ns + activation_guard_ns)) ||
      candidate->valid_from_ns > activation_ns || candidate->declared_end_ns < activation_ns) {
    planner_->discardCommandCandidate();
    return reject(kActivationWindow);
  }
  candidate->valid_from_ns = activation_ns;
  candidate->valid_until_ns = std::min(
      candidate->declared_end_ns, activation_ns + maximum_age_ns);
  candidate->activation_stamp_ns = activation_ns;
  const auto candidate_start_ns = navigation_common::secondsToNanoseconds(
      candidate->start_wall_time_s);
  if (schedule_successor &&
      (!candidate_start_ns || *candidate_start_ns != activation_ns ||
       candidate->declared_start_ns != activation_ns)) {
    planner_->discardCommandCandidate();
    RCLCPP_WARN(
        get_logger(),
        "execution boundary rejected successor with non-exact declared start "
        "declared=%lld rounded=%lld activation=%lld",
        static_cast<long long>(candidate->declared_start_ns),
        candidate_start_ns ? static_cast<long long>(*candidate_start_ns) : 0LL,
        static_cast<long long>(activation_ns));
    return reject(kNonExactSuccessorStart);
  }
  if (!candidate->valid()) {
    planner_->discardCommandCandidate();
    return reject(kInvalidCandidate);
  }
  const auto candidate_ptr = std::make_shared<const navigation_planning::CandidateBundle>(
      *candidate);
  // The execution timeline owns the future-state handoff. Analytic matching
  // and lease admission are separate contracts; a boolean late gate must not
  // conflate a polynomial mismatch with an expired old command.
  std::optional<navigation_execution::ExecutionAnchor> anchor;
  if (schedule_successor) {
    anchor = command_bundle_store_.reserveAnchor(now_ns, activation_ns);
    if (!anchor) {
      planner_->discardCommandCandidate();
      RCLCPP_WARN(get_logger(),
                  "execution boundary rejected successor: active command cannot reach activation "
                  "now_ns=%lld activation_ns=%lld",
                  static_cast<long long>(now_ns), static_cast<long long>(activation_ns));
      return reject(kAnchorUnavailable);
    }
    const auto anchor_match = navigation_execution::candidateMatchesAnchor(
        *candidate_ptr, *anchor);
    if (anchor_match != navigation_execution::AnchorMatchResult::kMatch) {
      planner_->discardCommandCandidate();
      RCLCPP_WARN(get_logger(),
                  "execution boundary rejected successor anchor match result=%d "
                  "active_generation=%lu candidate_generation=%lu activation_ns=%lld",
                  static_cast<int>(anchor_match),
                  static_cast<unsigned long>(anchor->active_bundle_generation),
                  static_cast<unsigned long>(candidate_ptr->bundle_generation),
                  static_cast<long long>(activation_ns));
      return reject(kAnchorMismatch);
    }
  }
  const bool stop_goal = goal.behavior ==
      navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP;
  const bool pass_through_goal = goal.behavior ==
      navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH;
  // Endpoint metadata is an execution contract for both terminal STOP and
  // finite PASS_THROUGH bundles. A missing/unsampleable declared endpoint must
  // reject the candidate; warning and committing it would leave downstream
  // waypoint completion dependent on an unavailable sample. PASS_THROUGH's
  // outgoing lookahead is not itself compared with the requested waypoint.
  if ((stop_goal || pass_through_goal) &&
      (!candidate_ptr->hasDeclaredEndpointMetadata() ||
       !candidate_ptr->sampleAtDeclaredEnd())) {
    planner_->discardCommandCandidate();
    RCLCPP_WARN(
        get_logger(),
        "execution boundary rejected candidate without a valid declared endpoint "
        "for waypoint behavior=%s",
        stop_goal ? "STOP" : "PASS_THROUGH");
    return reject(kEndpointMetadata);
  }
  if (pass_through_goal && candidate_ptr->route_boundary_event.has_value()) {
    const auto& boundary = *candidate_ptr->route_boundary_event;
    const auto& constraint = candidate_ptr->route_boundary_constraint;
    const auto endpoint = candidate_ptr->sampleAtDeclaredEnd();
    const auto route_snapshot = decodeRouteSnapshot(goal);
    const bool coincident_terminal_pass_through = route_snapshot.has_value() &&
        navigation_mission::passThroughNextWaypointIsCoincidentStop(
            *route_snapshot);
    const bool constraint_valid = constraint && constraint->valid();
    const bool boundary_valid = boundary.valid();
    // A PASS_THROUGH immediately followed by a STOP at the same physical
    // point is one terminal mission boundary. The planner intentionally emits
    // a terminal event there to suppress outgoing velocity; accepting it here
    // keeps the runtime and route-progress semantic contract aligned.
    const auto expected_boundary_kind = coincident_terminal_pass_through
        ? navigation_planning::RouteBoundaryEventKind::kTerminalStop
        : navigation_planning::RouteBoundaryEventKind::kPassThrough;
    const bool boundary_kind_valid =
        boundary.kind == expected_boundary_kind;
    const bool boundary_junction_valid = route_snapshot.has_value() &&
        boundary.junction_index == route_snapshot->active_waypoint_index &&
        constraint_valid &&
        constraint->junction_index == route_snapshot->active_waypoint_index;
    const bool boundary_stamp_valid =
        boundary.boundary_stamp_ns >= candidate_ptr->declared_start_ns &&
        boundary.boundary_stamp_ns <= candidate_ptr->declared_end_ns;
    const auto boundary_sample = boundary_stamp_valid
        ? candidate_ptr->sampleAtDeclaredStamp(boundary.boundary_stamp_ns)
        : std::nullopt;
    const bool endpoint_valid = endpoint && endpoint->finite();
    const bool boundary_sample_valid = boundary_sample && boundary_sample->finite();
    const bool boundary_crossing_contained = constraint_valid &&
        boundary_sample_valid && constraint->contains(boundary_sample->position_world);
    const bool boundary_position_matches = boundary_sample_valid &&
        (boundary_sample->position_world - boundary.position_world).norm() <= 1.0e-4;
    const bool boundary_role_valid = boundary_sample_valid &&
        boundary_sample->role == navigation_planning::CandidateRole::kMain;
    if (!constraint_valid || !boundary_valid || !boundary_kind_valid ||
        !boundary_junction_valid ||
        !boundary_stamp_valid || !endpoint_valid || !boundary_crossing_contained ||
        !boundary_position_matches || !boundary_role_valid) {
      planner_->discardCommandCandidate();
      RCLCPP_WARN(get_logger(),
                  "execution boundary rejected pass-through without a hard route boundary "
                  "junction=%zu endpoint=(%.3f,%.3f,%.3f) constraint_valid=%d "
                  "boundary_valid=%d kind=%d expected_kind=%d stamp_valid=%d endpoint_valid=%d "
                  "contained=%d boundary_stamp=%lld declared_end=%lld "
                  "boundary_sample=(%.9f,%.9f,%.9f) boundary_event=(%.9f,%.9f,%.9f) "
                  "volume_min=(%.3f,%.3f,%.3f) volume_max=(%.3f,%.3f,%.3f)",
                  boundary.junction_index,
                  endpoint ? endpoint->position_world.x() : 0.0,
                  endpoint ? endpoint->position_world.y() : 0.0,
                  endpoint ? endpoint->position_world.z() : 0.0,
                  constraint_valid ? 1 : 0, boundary_valid ? 1 : 0,
                  static_cast<int>(boundary.kind),
                  static_cast<int>(expected_boundary_kind),
                  boundary_stamp_valid ? 1 : 0,
                  endpoint_valid ? 1 : 0, boundary_crossing_contained ? 1 : 0,
                  static_cast<long long>(boundary.boundary_stamp_ns),
                  static_cast<long long>(candidate_ptr->declared_end_ns),
                  boundary_sample ? boundary_sample->position_world.x() : 0.0,
                  boundary_sample ? boundary_sample->position_world.y() : 0.0,
                  boundary_sample ? boundary_sample->position_world.z() : 0.0,
                  boundary.position_world.x(), boundary.position_world.y(),
                  boundary.position_world.z(),
                  constraint ? constraint->admissible_volume.minimum.x() : 0.0,
                  constraint ? constraint->admissible_volume.minimum.y() : 0.0,
                  constraint ? constraint->admissible_volume.minimum.z() : 0.0,
                  constraint ? constraint->admissible_volume.maximum.x() : 0.0,
                  constraint ? constraint->admissible_volume.maximum.y() : 0.0,
                  constraint ? constraint->admissible_volume.maximum.z() : 0.0);
      return reject(kRouteBoundary);
    }
  }
  const double activation_wall_time_s = static_cast<double>(activation_ns) * 1.0e-9;
  if (!navigation_planning::candidateHasRequiredMainReserve(
          *candidate_ptr, activation_wall_time_s)) {
    planner_->discardCommandCandidate();
    const double commit_wall_time_s = now().seconds();
    RCLCPP_WARN(
        get_logger(),
        "execution boundary rejected candidate with insufficient MAIN reserve: "
        "kind=%d start=%.6f duration=%.3f backup_start=%.3f commit=%.6f "
        "candidate_age=%.3f remaining=%.3f required=%.3f",
        static_cast<int>(candidate_ptr->kind),
        candidate_ptr->start_wall_time_s, candidate_ptr->duration_s,
        candidate_ptr->backup_start_time_s, commit_wall_time_s,
        commit_wall_time_s - candidate_ptr->start_wall_time_s,
        navigation_planning::remainingMainHorizonAtCommit(
            *candidate_ptr, activation_wall_time_s),
        navigation_planning::PlanningTimingContract::kMinimumMainReserveS);
    return reject(kMainReserve);
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
  const auto candidate_sample = candidate_ptr->sample(activation_ns);
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
    return reject(kExecutionState);
  }
  const auto transaction_id = advanceMonotonicId(execution_transaction_id_);
  if (!transaction_id) {
    planner_->discardCommandCandidate();
    std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    std::lock_guard<std::mutex> command_lock(
    command_execution_lease_failure_latch_.transitionMutex());
    command_bundle_store_.invalidate();
    failClosedLocked();
    RCLCPP_ERROR(get_logger(), "execution transaction id exhausted");
    return reject(kTransactionIdExhausted);
  }
  const navigation_execution::CommitToken token{
      candidate_ptr->world_identity, goal_epoch, *transaction_id};
  const auto stage_decision = anchor
      ? static_cast<int>(command_bundle_store_.stagePending(token, *anchor, candidate_ptr))
      : static_cast<int>(command_bundle_store_.tryCommit(token, candidate_ptr));
  const bool staged = stage_decision == 0;
  if (!staged) {
    planner_->discardCommandCandidate();
    last_execution_boundary_rejection_.store(
        kStageOrCommitRejected * 100 + stage_decision, std::memory_order_release);
    RCLCPP_WARN(get_logger(),
                "execution successor generation=%lu was not admitted; active command remains authoritative "
                "store_decision=%d",
                static_cast<unsigned long>(candidate_ptr->bundle_generation), stage_decision);
    return false;
  }
  world_freshness_suspended_bundle_generation_.store(0U, std::memory_order_release);
  world_freshness_suspended_safety_suffix_active_.store(
      false, std::memory_order_release);
  const auto committed_timeline = command_bundle_store_.snapshot();
  if (!anchor) {
    // The execution store is the sole command-authority cutover. Emergency
    // replacement is already committed synchronously at this worker-owned
    // measured boundary, so synchronize planner warm-start history before the
    // next PlanFromRest attempt. Future successors still use the activation
    // queue and remain pending until their exact producer boundary.
    bool planner_activation_queued = true;
    if (emergency_candidate) {
      planner_->onExecutionTimelineActivated(candidate_ptr->bundle_generation);
      planner_timeline_activation_generation_.store(
          candidate_ptr->bundle_generation, std::memory_order_release);
    } else {
      planner_activation_queued = queueExecutionTimelineActivation(
          candidate_ptr->bundle_generation);
    }
    if (!planner_activation_queued) {
      if (committed_timeline.active.get() == candidate_ptr.get()) {
        (void)clearCommandForCurrentIdentity(
            goal, goal_epoch, localization_epoch, committed_timeline);
      }
      planner_->discardCommandCandidate();
      return false;
    }
    bool command_identity_committed = false;
    {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const auto current_bundle = command_bundle_store_.load();
      const bool current_identity = current_bundle &&
          current_bundle.get() == candidate_ptr.get() &&
          active_goal_ && active_goal_->mission_id == goal.mission_id &&
          active_goal_->waypoint_index == goal.waypoint_index &&
          active_goal_->request_id == goal.request_id &&
          active_goal_epoch_.load(std::memory_order_acquire) == goal_epoch &&
          active_localization_epoch_.load(std::memory_order_acquire) ==
              localization_epoch &&
          command_execution_lease_failure_latch_.allowsCommandExposure();
      if (current_identity) {
        command_goal_epoch_.store(goal_epoch, std::memory_order_release);
        // An immediate commit is also the execution-identity cutover. This
        // matters for measured emergency replacements: the normal
        // CommandReady tail is not reached to publish executing_goal_.
        executing_goal_ = goal;
        execution_episode_.commandCommitted(*candidate_ptr);
        command_identity_committed = true;
      } else if (current_bundle && current_bundle.get() == candidate_ptr.get()) {
        // A goal/lease transition won after the store commit. Revoke only the
        // stale candidate pointer; never fail-close a newer command identity.
        (void)command_bundle_store_.invalidateIfCurrent(committed_timeline);
      }
    }
    if (!command_identity_committed) {
      planner_->discardCommandCandidate();
      return false;
    }
  }
  return true;
}

bool NavigationRuntimeNode::clearCommandForCurrentIdentity(
    const navigation_contracts::msg::NavigationGoal& command_goal,
    const std::uint64_t goal_epoch_at_command,
    const std::uint64_t localization_epoch_at_command,
    const navigation_execution::ExecutionTimelineSnapshot& expected) {
  std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
  std::lock_guard<std::mutex> input_lock(input_mutex_);
  std::lock_guard<std::mutex> command_lock(
      command_execution_lease_failure_latch_.transitionMutex());
  const bool goal_current = active_goal_ &&
      active_goal_->mission_id == command_goal.mission_id &&
      active_goal_->waypoint_index == command_goal.waypoint_index &&
      active_goal_->request_id == command_goal.request_id &&
      active_goal_epoch_.load(std::memory_order_acquire) == goal_epoch_at_command &&
      active_localization_epoch_.load(std::memory_order_acquire) ==
          localization_epoch_at_command;
  if (!goal_current || !command_bundle_store_.invalidateIfCurrent(expected)) return false;
  pending_goal_owner_.clearGoal();
  command_goal_epoch_.store(0U, std::memory_order_release);
  failClosedLocked();
  return true;
}

void NavigationRuntimeNode::suspendCommandForWorldFreshness() {
  std::lock_guard<std::mutex> command_lock(
      command_execution_lease_failure_latch_.transitionMutex());
  const auto episode = execution_episode_.snapshot();
  if (episode.command_available) {
    const auto bundle = command_bundle_store_.load();
    if (bundle && bundle->localization_epoch ==
                      active_localization_epoch_.load(std::memory_order_acquire) &&
        bundle->goal_epoch == command_goal_epoch_.load(std::memory_order_acquire)) {
      world_freshness_suspended_bundle_generation_.store(
          bundle->bundle_generation, std::memory_order_release);
      world_freshness_suspended_safety_suffix_active_.store(
          episode.safety_suffix_active,
          std::memory_order_release);
      ++world_freshness_command_suspend_count_;
    }
  }
  // Keep the execution identity while publication is suspended.  A fresh
  // world may recertify and resume this exact bundle, including across a
  // desired pass-through transition whose successor is still pending.
  execution_episode_.suspendCommand();
}

std::optional<PlanningKey> NavigationRuntimeNode::currentPlanningKey() {
  std::optional<navigation_contracts::msg::NavigationGoal> goal;
  std::optional<navigation_contracts::msg::NavigationGoal> executing_goal;
  ExecutionEpisodeSnapshot episode;
  ExecutionRecoveryState recovery_state = ExecutionRecoveryState::kPx4Hold;
  std::shared_ptr<const navigation_planning::CandidateBundle> bundle;
  bool pending_bundle = false;
  std::uint64_t goal_epoch = 0U;
  std::uint64_t localization_epoch = 0U;
  std::uint64_t command_goal_epoch = 0U;
  {
    std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    goal = active_goal_;
    executing_goal = this->executing_goal_;
    episode = execution_episode_.snapshot();
    recovery_state = execution_recovery_state_.load(std::memory_order_acquire);
    goal_epoch = active_goal_epoch_.load(std::memory_order_acquire);
    localization_epoch = active_localization_epoch_.load(std::memory_order_acquire);
    command_goal_epoch = command_goal_epoch_.load(std::memory_order_acquire);
    bundle = command_bundle_store_.load();
    pending_bundle = command_bundle_store_.hasPending();
  }
  const auto execution = execution_state_store_.load();
  const auto world = world_snapshot_store_.load();
  if (!goal || !execution || !world || goal_epoch == 0U ||
      goal->route.route_revision == 0U) {
    return std::nullopt;
  }
  // A successor already staged owns the next activation slot. Do not start a
  // second solve against the same active generation while that slot is
  // waiting; after activation the next timer observes the new generation and
  // creates a fresh execution anchor.
  if (pending_bundle) return std::nullopt;
  if (!executionRecoveryStateKnown(recovery_state) ||
      recovery_state == ExecutionRecoveryState::kPx4Hold ||
      episode.failure_latched) {
    return std::nullopt;
  }
  const bool recovery_requires_command =
      recovery_state == ExecutionRecoveryState::kTrackBackup ||
      recovery_state == ExecutionRecoveryState::kEmergencyBrake;
  const bool command_identity_current = bundle && executing_goal &&
      command_goal_epoch != 0U &&
      episode.active_generation == bundle->bundle_generation &&
      bundle->localization_epoch == localization_epoch &&
      bundle->goal_epoch == command_goal_epoch &&
      bundle->request_id == executing_goal->request_id;
  if ((episode.command_available || recovery_requires_command) &&
      !command_identity_current) {
    // A desired hot-retarget may coexist with an older executing bundle, but
    // the executing bundle itself must never be relabeled or partially read.
    return std::nullopt;
  }
  bool safety_renewal = episode.restart_from_rest || episode.safety_suffix_active;
  const auto route_snapshot = decodeRouteSnapshot(*goal);
  const bool coincident_pass_through_stop = route_snapshot.has_value() &&
      navigation_mission::passThroughNextWaypointIsCoincidentStop(*route_snapshot);
  const bool effective_terminal_stop =
      goal->behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP ||
      coincident_pass_through_stop;
  const bool committed_terminal_hold = bundle &&
      committedTerminalBundleHoldIsPending(
          episode.command_available,
          effective_terminal_stop,
          bundle->kind == navigation_planning::CandidateBundleKind::kTerminalStop &&
              bundle->terminal_stop,
          bundle->role == navigation_planning::CandidateRole::kMain,
          bundle->request_id == goal->request_id && bundle->goal_epoch == goal_epoch,
          bundle->bundle_generation);
  if (terminalHoldIsPending(
          episode.command_available,
          trajectory_reaches_goal_.load(std::memory_order_acquire),
          effective_terminal_stop,
          terminal_bundle_generation_.load(std::memory_order_acquire)) ||
      committed_terminal_hold) {
    // A certified terminal endpoint is the execution timeline's final
    // command for this request. Do not let the periodic timer create a
    // replacement solve while PX4 is settling or acknowledging the hold.
    return std::nullopt;
  }
  safety_renewal = safety_renewal ||
      recovery_state == ExecutionRecoveryState::kTrackBackup ||
      recovery_state == ExecutionRecoveryState::kEmergencyBrake;
  PlanningKey key;
  key.localization_epoch = localization_epoch;
  key.goal_epoch = goal_epoch;
  key.request_id = goal->request_id;
  key.route_revision = goal->route.route_revision;
  key.committed_bundle_generation = bundle ? bundle->bundle_generation : 0U;
  key.pinned_world_generation = world.identity.generation;
  key.pinned_world_revision = world.identity.revision;
  key.start_mode = bundle &&
          episode.command_available &&
          !safety_renewal
      ? PlanningStartMode::kCommittedFutureState
      : PlanningStartMode::kStoppedMeasuredState;
  key.anchor_stamp_ns = execution->state.source_stamp_ns;
  key.dynamics_hash = dynamics_hash_;
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
    const auto episode = execution_episode_.snapshot();
    safety_renewal = episode.restart_from_rest || episode.safety_suffix_active;
  }
  const auto priority = PlanningSupervisor::classifyPriority(
      false, false, goal_transition, safety_renewal);
  (void)planning_worker_->submit(
      *key, priority,
      [this, scheduled_key = *key](
          navigation_planning_backend::PlannerFacade& planner, std::stop_token stop) {
        if (stop.stop_requested()) return;
        applyQueuedExecutionTimelineActivations(planner);
        if (stop.stop_requested()) return;
        const auto current = currentPlanningKey();
        if (!current || !PlanningSupervisor::resultStillCurrent(scheduled_key, *current)) {
          return;
        }
        runCycle(scheduled_key);
      });
}

bool NavigationRuntimeNode::queueExecutionTimelineActivation(
    const std::uint64_t generation) noexcept {
  if (generation == 0U ||
      generation <= planner_timeline_activation_generation_.load(
          std::memory_order_acquire)) {
    return generation != 0U;
  }
  try {
    std::lock_guard<std::mutex> lock(planner_timeline_activation_mutex_);
    if (generation <= planner_timeline_activation_generation_.load(
            std::memory_order_acquire)) {
      return true;
    }
    if (std::find(queued_planner_timeline_activations_.begin(),
                  queued_planner_timeline_activations_.end(), generation) ==
        queued_planner_timeline_activations_.end()) {
      queued_planner_timeline_activations_.push_back(generation);
    }
    return true;
  } catch (...) {
    // Losing planner-history synchronization is fail-closed: the immutable
    // execution timeline remains authoritative and the next solve will not
    // consume an unqueued activation.
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "failed to queue execution timeline activation generation=%lu",
        static_cast<unsigned long>(generation));
    return false;
  }
}

void NavigationRuntimeNode::applyQueuedExecutionTimelineActivations(
    navigation_planning_backend::PlannerFacade& planner) noexcept {
  std::deque<std::uint64_t> activations;
  {
    std::lock_guard<std::mutex> lock(planner_timeline_activation_mutex_);
    activations.swap(queued_planner_timeline_activations_);
  }
  std::sort(activations.begin(), activations.end());
  for (const auto generation : activations) {
    if (generation <= planner_timeline_activation_generation_.load(
            std::memory_order_acquire)) {
      continue;
    }
    planner.onExecutionTimelineActivated(generation);
    planner_timeline_activation_generation_.store(
        generation, std::memory_order_release);
  }
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
  std::optional<navigation_contracts::msg::NavigationGoal> cycle_goal;
  bool new_goal = false;
  bool hot_goal_transition = false;
  bool restart_from_rest = false;
  std::uint64_t goal_epoch = 0;
  std::uint64_t goal_epoch_at_cycle = 0U;
  std::uint64_t localization_epoch_at_cycle = 0U;
  std::optional<TrajectoryCompletionWitness> completion_witness_at_cycle;
  std::shared_ptr<const navigation_planning::CandidateBundle>
      completed_bundle_at_cycle;
  bool completed_trajectory = false;
  navigation_execution::ExecutionTimelineSnapshot expected_timeline_at_cycle;
  const auto input_lock_started = std::chrono::steady_clock::now();
  {
    // Capture the planner callback's ownership identity once.  The helper used
    // by early failure paths rechecks this identity under the same lock order;
    // it must never load a newer bundle at the failure site and clear it as if
    // it belonged to this callback.
    std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
    std::lock_guard<std::mutex> lock(input_mutex_);
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    last_input_lock_wait_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - input_lock_started).count();
    goal = active_goal_;
    cycle_goal = goal;
    new_goal = new_goal_;
    hot_goal_transition = hot_goal_transition_;
    restart_from_rest = execution_episode_.snapshot().restart_from_rest;
    goal_epoch = active_goal_epoch_.load();
    goal_epoch_at_cycle = goal_epoch;
    localization_epoch_at_cycle = active_localization_epoch_.load(
        std::memory_order_acquire);
    expected_timeline_at_cycle = command_bundle_store_.snapshot();
    if (trajectory_completion_witness_) {
      const auto& witness = *trajectory_completion_witness_;
      if (completionWitnessMatchesCurrentExecution(
              witness, expected_timeline_at_cycle,
              expected_timeline_at_cycle.active, executing_goal_,
              command_goal_epoch_.load(std::memory_order_acquire),
              localization_epoch_at_cycle,
              execution_episode_.snapshot().failure_latched,
              command_execution_lease_failure_latch_.allowsCommandExposure())) {
        completion_witness_at_cycle = witness;
        completed_bundle_at_cycle = expected_timeline_at_cycle.active;
        completed_trajectory = true;
      } else {
        trajectory_completion_witness_.reset();
      }
    }
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
  const auto episode = execution_episode_.snapshot();
  add_value("execution_episode_phase",
            static_cast<std::uint64_t>(episode.phase));
  add_value("execution_episode_generation", episode.active_generation);
  add_value("execution_episode_goal_epoch", episode.goal_epoch);
  add_value("execution_episode_command_available",
            episode.command_available ? 1U : 0U);
  add_value("execution_episode_failure_latched",
            episode.failure_latched ? 1U : 0U);
  add_value("planning_outcome", static_cast<std::uint64_t>(
      last_planning_outcome_.load(std::memory_order_acquire)));
  add_value("planning_failure_stage", static_cast<std::uint64_t>(
      last_planning_failure_stage_.load(std::memory_order_acquire)));
  add_value("planning_failure_reason", static_cast<std::uint64_t>(
      last_planning_failure_reason_.load(std::memory_order_acquire)));
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
  add_value("mapping_traversed_segment_count",
            map_diagnostics.traversed_segment_count);
  add_signed_value("mapping_traversed_latest_stamp_ns",
                   map_diagnostics.traversed_latest_stamp_ns);
  add_value("mapping_traversed_chain_reset_reason",
            map_diagnostics.traversed_chain_reset_reason);
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
  //
  // This is deliberately a peek. A finished BACKUP endpoint can be sampled before
  // the measured vehicle velocity has settled below the certified-stop threshold;
  // consuming the witness in that interval loses the only event that can advance
  // a pending handoff. The witness is cleared only after the recovery gate has
  // accepted this cycle for processing.
  bool completed_trajectory_for_planning = completed_trajectory;
  std::shared_ptr<const navigation_contracts::msg::NavigationGoal>
      pending_handoff_goal;
  PlanningKey effective_scheduled_key = scheduled_key;
  const double measured_speed_mps = execution_state.velocity_world.norm();
  auto route_snapshot = goal ? decodeRouteSnapshot(*goal) : std::nullopt;
  bool coincident_pass_through_stop = route_snapshot.has_value() &&
      navigation_mission::passThroughNextWaypointIsCoincidentStop(*route_snapshot);
  bool pass_through_goal = goal &&
      goal->behavior ==
          navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH;
  bool stop_goal = goal &&
      goal->behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP;
  bool effective_terminal_stop = stop_goal || coincident_pass_through_stop;
  const auto refresh_goal_metadata = [&]() {
    route_snapshot = goal ? decodeRouteSnapshot(*goal) : std::nullopt;
    coincident_pass_through_stop = route_snapshot.has_value() &&
        navigation_mission::passThroughNextWaypointIsCoincidentStop(*route_snapshot);
    pass_through_goal = goal &&
        goal->behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH;
    stop_goal = goal &&
        goal->behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP;
    effective_terminal_stop = stop_goal || coincident_pass_through_stop;
  };
  if (completed_trajectory && completion_witness_at_cycle &&
      completed_bundle_at_cycle) {
    bool completion_identity_current = false;
    {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const auto current_timeline = command_bundle_store_.snapshot();
      const auto witness = completion_witness_at_cycle.value_or(
          TrajectoryCompletionWitness{});
      completion_identity_current = completionWitnessMatchesCurrentExecution(
          witness, current_timeline, completed_bundle_at_cycle, executing_goal_,
          command_goal_epoch_.load(std::memory_order_acquire),
          localization_epoch_at_cycle,
          execution_episode_.snapshot().failure_latched,
          command_execution_lease_failure_latch_.allowsCommandExposure());
      if (!completion_identity_current && trajectory_completion_witness_ &&
          *trajectory_completion_witness_ == witness) {
        trajectory_completion_witness_.reset();
      }
    }
    if (!completion_identity_current) {
      completed_trajectory = false;
      completed_trajectory_for_planning = false;
      completion_witness_at_cycle.reset();
      completed_bundle_at_cycle.reset();
    }
  }
  auto recovery_state = execution_recovery_state_.load(std::memory_order_acquire);
  if (recovery_state == ExecutionRecoveryState::kTrackBackup ||
      recovery_state == ExecutionRecoveryState::kEmergencyBrake) {
    const bool emergency_stop_completed =
        recovery_state == ExecutionRecoveryState::kEmergencyBrake;
    const auto episode = execution_episode_.snapshot();
    const bool backup_terminal_hold_pending = terminalHoldIsPending(
        episode.command_available,
        trajectory_reaches_goal_.load(std::memory_order_acquire),
        effective_terminal_stop,
        terminal_bundle_generation_.load(std::memory_order_acquire));
    const bool backup_stop_requires_restart = backupStopNeedsMeasuredRestart(
        recovery_state, completed_trajectory, backup_terminal_hold_pending);
    if (backup_stop_requires_restart) {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const auto stopped_bundle = command_bundle_store_.load();
      const auto stopped_execution_goal = executing_goal_;
      const auto stopped_goal_epoch = command_goal_epoch_.load(std::memory_order_acquire);
      if (!stopped_execution_goal || !stopped_bundle ||
          !executingCommandIdentityMatchesLocked(
              *stopped_execution_goal, stopped_goal_epoch,
              localization_epoch_at_cycle, stopped_bundle->bundle_generation)) {
        return;
      }
      execution_episode_.requestRestartFromRest();
      hot_goal_transition_ = false;
      restart_from_rest = true;
      trajectory_reaches_goal_.store(false, std::memory_order_release);
      terminal_bundle_generation_.store(0U, std::memory_order_release);
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "certified BACKUP endpoint is outside waypoint acceptance; waiting for measured stop "
          "before PlanFromRest");
    }
    if (!completed_trajectory || !std::isfinite(measured_speed_mps) ||
        measured_speed_mps > 0.15) {
      // BACKUP and emergency are one-way while moving. Do not let a queued or
      // periodic nominal solve replace the braking command. A non-terminal
      // BACKUP has already recorded the measured-stop restart above, so this
      // return only waits for the velocity gate to settle.
      return;
    }
    {
      // Linearization order is input_mutex_ -> execution transition. The
      // pending slot is selected and consumed while both locks are held, so a
      // callback cannot arrive between extraction, suffix clearing, and goal
      // activation. Do not call public onGoal recursively here.
      std::unique_lock<std::mutex> localization_lock(localization_transition_mutex_);
      std::unique_lock<std::mutex> input_lock(input_mutex_);
      std::unique_lock<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const auto stopped_bundle = command_bundle_store_.load();
      const auto stopped_execution_goal = executing_goal_;
      const auto stopped_goal_epoch = command_goal_epoch_.load(std::memory_order_acquire);
      const bool stop_identity_current = stopped_execution_goal && stopped_bundle &&
          executingCommandIdentityMatchesLocked(
              *stopped_execution_goal, stopped_goal_epoch, localization_epoch_at_cycle,
              stopped_bundle->bundle_generation);
      if (!stop_identity_current) return;
      applyExecutionRecoveryEventLocked(
          ExecutionRecoveryEvent::kCertifiedStopObserved);
      recovery_state = execution_recovery_state_.load(std::memory_order_acquire);
      if (foreign_mission_hold_after_stop_) {
        // Foreign mission identity is a control-authority violation.  The
        // certified suffix has now stopped, so perform one complete transition
        // to PX4 Hold and revoke every goal owner atomically.
        transitionForeignMissionLocked(false);
        const bool cancel_foreign_worker = consumeForeignMissionCancelIfCurrentLocked();
        command_lock.unlock();
        input_lock.unlock();
        localization_lock.unlock();
        if (cancel_foreign_worker && planning_worker_) {
          planning_worker_->cancelActive();
        }
        return;
      }
      if (auto pending = pending_goal_owner_.goalSnapshot(); pending) {
        command_goal_epoch_.store(0U, std::memory_order_release);
        // Keep ownership of the pending request until the replacement
        // candidate commits. Solving in this same cycle avoids the old
        // promote-then-wait command-lease gap.
        applyValidatedGoalLocked(pending, true);
        pending_handoff_goal = pending;
        goal = *pending;
        refresh_goal_metadata();
        goal_epoch = active_goal_epoch_.load(std::memory_order_acquire);
        new_goal = true;
        hot_goal_transition = false;
        restart_from_rest = false;
        completed_trajectory_for_planning = false;
        recovery_state = execution_recovery_state_.load(std::memory_order_acquire);
      }
      if (deferred_terminal_status_.has_value() && goal.has_value() &&
          deferred_terminal_status_->mission_id == goal->mission_id &&
          deferred_terminal_status_->waypoint_index == goal->waypoint_index &&
          deferred_terminal_status_->request_id == goal->request_id) {
        if (planning_worker_) planning_worker_->cancelActive();
        active_goal_.reset();
        executing_goal_.reset();
        pending_goal_owner_.clearGoal();
        execution_episode_.clearGoal(
            active_localization_epoch_.load(std::memory_order_acquire));
        deferred_terminal_status_.reset();
        command_bundle_store_.invalidate();
        command_goal_epoch_.store(0U, std::memory_order_release);
        failClosedLocked();
        return;
      }
      // A terminal status may have cleared the old active identity while this
      // cycle was executing. Do not continue the stale local snapshot after
      // certified stop; the next cycle must observe the cleared state.
      if (!goal.has_value() || !active_goal_.has_value() ||
          active_goal_->mission_id != goal->mission_id ||
          active_goal_->waypoint_index != goal->waypoint_index ||
          active_goal_->request_id != goal->request_id) {
        return;
      }
      if (emergency_stop_completed) {
        // An emergency brake is a safety stop, not proof that the nominal
        // waypoint was completed. Even when its endpoint happens to fall
        // inside the acceptance ball, resume the same goal from the measured
        // stopped state. Otherwise completion bookkeeping can turn a recovery
        // endpoint into a terminal STOP hold and External Mode eventually
        // hands over to PX4 Hold without another nominal solve.
        execution_episode_.requestRestartFromRest();
        hot_goal_transition_ = false;
        restart_from_rest = true;
        trajectory_reaches_goal_.store(false, std::memory_order_release);
        terminal_bundle_generation_.store(0U, std::memory_order_release);
        RCLCPP_WARN(
            get_logger(),
            "emergency brake reached a certified stop; replanning the active waypoint "
            "from measured state instead of treating the safety endpoint as terminal");
      } else if (backup_stop_requires_restart) {
        // The BACKUP suffix is complete but did not prove waypoint acceptance.
        // Continue the same goal from the measured stop. This is distinct
        // from MotionObserved: no new emergency command is inferred and no
        // nominal command is exposed before the stop gate.
        execution_episode_.requestRestartFromRest();
        hot_goal_transition_ = false;
        restart_from_rest = true;
        trajectory_reaches_goal_.store(false, std::memory_order_release);
        terminal_bundle_generation_.store(0U, std::memory_order_release);
        RCLCPP_INFO(
            get_logger(),
            "certified BACKUP stop did not reach waypoint acceptance; replanning active waypoint "
            "from measured stop");
      }
    }
  } else if (recovery_state == ExecutionRecoveryState::kStoppedRecovery &&
             (!std::isfinite(measured_speed_mps) || measured_speed_mps > 0.15)) {
    bool emergency_replan_pending = false;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      emergency_replan_pending = execution_episode_.snapshot().restart_from_rest;
    }
    if (emergency_replan_pending) {
      // The emergency endpoint may be sampled as completed before PX4's
      // measured velocity has settled again. Keep the recovery episode alive
      // and let the exact endpoint hold remain authoritative; the next cycle
      // will run PlanFromRest once the measured stop gate is satisfied. Treating
      // this short post-brake residual as a new violation would erase the
      // recovery request and publish a stale STATUS_REJECTED command.
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "emergency recovery endpoint still settling speed=%.3f m/s; waiting for measured stop",
          measured_speed_mps);
      return;
    }
    const auto stopped_bundle = command_bundle_store_.load();
    const bool stopped_bundle_identity_matches = stopped_bundle && goal &&
        stopped_bundle->request_id == goal->request_id &&
        stopped_bundle->goal_epoch == goal_epoch;
    const bool committed_terminal_hold_pending = committedTerminalBundleHoldIsPending(
        execution_episode_.snapshot().command_available,
        effective_terminal_stop,
        stopped_bundle && stopped_bundle->kind == navigation_planning::CandidateBundleKind::kTerminalStop &&
            stopped_bundle->terminal_stop,
        stopped_bundle && stopped_bundle->role == navigation_planning::CandidateRole::kMain,
        stopped_bundle_identity_matches,
        stopped_bundle ? stopped_bundle->bundle_generation : 0U);
    const bool terminal_hold_pending = terminalHoldIsPending(
        execution_episode_.snapshot().command_available,
        trajectory_reaches_goal_.load(std::memory_order_acquire),
        effective_terminal_stop,
        terminal_bundle_generation_.load(std::memory_order_acquire)) ||
        committed_terminal_hold_pending;
    if (terminal_hold_pending) {
      // A STOP bundle has an atomically committed, known-free terminal
      // endpoint. PX4 can still report a small residual velocity during the
      // first samples after the endpoint; keep replaying that exact endpoint
      // until the vehicle settles. This does not authorize a new trajectory
      // or an enum-only emergency transition. The normal terminal hold and
      // mission acknowledgement path remains responsible for completion.
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "terminal STOP endpoint still settling speed=%.3f m/s; retaining exact endpoint hold",
          measured_speed_mps);
      return;
    }
    // Motion observed after a certified stop is not an authorization to enter
    // EmergencyBrake by enum alone. There is no emergency candidate
    // transaction in this branch, so fail closed to PX4 Hold; a future
    // emergency request must create, certify and commit its command before the
    // state can become kEmergencyBrake.
    {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const auto stopped_bundle = command_bundle_store_.load();
      const auto stopped_execution_goal = executing_goal_;
      const auto stopped_goal_epoch = command_goal_epoch_.load(std::memory_order_acquire);
      if (!stopped_execution_goal || !stopped_bundle ||
          !executingCommandIdentityMatchesLocked(
              *stopped_execution_goal, stopped_goal_epoch,
              localization_epoch_at_cycle, stopped_bundle->bundle_generation)) {
        return;
      }
      plan_from_rest_first_failure_steady_ns_ = 0;
      command_bundle_store_.invalidate();
      command_goal_epoch_.store(0U, std::memory_order_release);
      failClosedLocked();
    }
    RCLCPP_ERROR(
        get_logger(),
        "motion observed after certified stop without an atomically committed "
        "emergency candidate; entering PX4 Hold");
    return;
  } else if (recovery_state == ExecutionRecoveryState::kStoppedRecovery) {
    // A terminal STOP may have received the next waypoint while its certified
    // safety suffix was still draining. Once the measured vehicle is
    // stationary, linearize the stop boundary before the old terminal command
    // can be sampled again: promote the newest pending goal and let its next
    // cycle perform PlanFromRest. Without this transition the pending goal
    // remains blocked behind safety_suffix_active forever and External Mode
    // eventually hands over to PX4 Hold even though the vehicle is settled.
    std::unique_lock<std::mutex> localization_lock(localization_transition_mutex_);
    std::unique_lock<std::mutex> input_lock(input_mutex_);
    std::unique_lock<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    if (foreign_mission_hold_after_stop_) {
      transitionForeignMissionLocked(false);
      const bool cancel_foreign_worker = consumeForeignMissionCancelIfCurrentLocked();
      command_lock.unlock();
      input_lock.unlock();
      localization_lock.unlock();
      if (cancel_foreign_worker && planning_worker_) {
        planning_worker_->cancelActive();
      }
      return;
    }
    if (auto pending = pending_goal_owner_.goalSnapshot(); pending) {
      command_goal_epoch_.store(0U, std::memory_order_release);
      // Keep the pending request until the replacement candidate commits in
      // this cycle; a failed solve can then retry the same active identity.
      applyValidatedGoalLocked(pending, true);
      pending_handoff_goal = pending;
      goal = *pending;
      refresh_goal_metadata();
      goal_epoch = active_goal_epoch_.load(std::memory_order_acquire);
      new_goal = true;
      hot_goal_transition = false;
      restart_from_rest = false;
      completed_trajectory_for_planning = false;
      recovery_state = execution_recovery_state_.load(std::memory_order_acquire);
      RCLCPP_INFO(get_logger(),
                  "preparing pending navigation goal after certified stop request=%lu",
                  static_cast<unsigned long>(pending->request_id));
    }
    if (deferred_terminal_status_.has_value() && goal.has_value() &&
        deferred_terminal_status_->mission_id == goal->mission_id &&
        deferred_terminal_status_->waypoint_index == goal->waypoint_index &&
        deferred_terminal_status_->request_id == goal->request_id) {
      if (planning_worker_) planning_worker_->cancelActive();
      active_goal_.reset();
      executing_goal_.reset();
      pending_goal_owner_.clearGoal();
      execution_episode_.clearGoal(
          active_localization_epoch_.load(std::memory_order_acquire));
      deferred_terminal_status_.reset();
      command_bundle_store_.invalidate();
      command_goal_epoch_.store(0U, std::memory_order_release);
      failClosedLocked();
      return;
    }
  } else if (recovery_state == ExecutionRecoveryState::kPx4Hold) {
    return;
  }
  if (pending_handoff_goal) {
    // The pending goal was promoted under the recovery transition locks. The
    // worker callback entered with the previous immutable key, so re-read the
    // actual current key after promotion instead of passing the old request,
    // bundle generation, or world pin into candidate admission.
    const auto refreshed_key = currentPlanningKey();
    if (!refreshed_key) {
      planner_->discardCommandCandidate();
      RCLCPP_WARN(get_logger(),
                  "pending recovery handoff was promoted but no fresh planning key is valid");
      return;
    }
    effective_scheduled_key = *refreshed_key;
  }
  if (completed_trajectory && completion_witness_at_cycle &&
      completed_bundle_at_cycle) {
    bool completion_identity_current = false;
    {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const auto witness = completion_witness_at_cycle.value_or(
          TrajectoryCompletionWitness{});
      const auto current_timeline = command_bundle_store_.snapshot();
      completion_identity_current = completionWitnessMatchesCurrentExecution(
          witness, current_timeline, completed_bundle_at_cycle, executing_goal_,
          command_goal_epoch_.load(std::memory_order_acquire),
          localization_epoch_at_cycle,
          execution_episode_.snapshot().failure_latched,
          command_execution_lease_failure_latch_.allowsCommandExposure());
      if (completion_identity_current && trajectory_completion_witness_ &&
          *trajectory_completion_witness_ == witness) {
        trajectory_completion_witness_.reset();
      }
    }
    if (!completion_identity_current) {
      // A newer goal, execution epoch, or bundle owns the command now. The
      // stale completion is deliberately a no-op and cannot trigger a solve
      // against the replacement identity.
      completed_trajectory = false;
      completed_trajectory_for_planning = false;
      completion_witness_at_cycle.reset();
      completed_bundle_at_cycle.reset();
    }
  }
  bool completed_trajectory_reaches_goal = trajectory_reaches_goal_.load();
  const bool has_outgoing_route = goal && !coincident_pass_through_stop &&
      static_cast<std::size_t>(goal->route.active_waypoint_index + 1U) <
      goal->route.waypoint_positions.size();
  bool completed_endpoint_valid = false;
  if (completed_trajectory_for_planning && goal) {
    // The completion witness can be replaced by a concurrent replan before this
    // callback observes the publisher's terminal sample. Recompute it from
    // the immutable committed candidate so a terminal command is not
    // mistaken for a frontier trajectory and restarted indefinitely.
    const auto committed_bundle = completed_bundle_at_cycle;
    const auto completion_witness = completion_witness_at_cycle.value_or(
        TrajectoryCompletionWitness{});
    const auto endpoint = committed_bundle &&
        committed_bundle->hasDeclaredEndpointMetadata()
        ? committed_bundle->sampleAtDeclaredEnd()
        : std::nullopt;
    completed_endpoint_valid = endpoint.has_value();
    bool desired_identity_current = false;
    {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      desired_identity_current = goal && desiredGoalIdentityMatchesLocked(
          *goal, goal_epoch, localization_epoch_at_cycle);
    }
    const bool completion_goal_matches = desired_identity_current && goal &&
        goal->request_id == completion_witness.request_id &&
        goal_epoch == completion_witness.goal_epoch &&
        localization_epoch_at_cycle == completion_witness.localization_epoch;
    double endpoint_error = std::numeric_limits<double>::infinity();
    double measured_goal_error = std::numeric_limits<double>::infinity();
    double completion_tolerance = std::numeric_limits<double>::quiet_NaN();
    bool terminal_endpoint_contract = false;
    if (completion_goal_matches) {
      const auto& target_message = plannerTarget(*goal);
      const Eigen::Vector3d target_position{
        pointFromMessage(target_message, 0),
        pointFromMessage(target_message, 1),
        pointFromMessage(target_message, 2)};
      completion_tolerance = goalCompletionTolerance(*goal);
      measured_goal_error = execution_state.finite()
          ? (execution_state.position_world - target_position).norm()
          : std::numeric_limits<double>::infinity();
      // PASS_THROUGH endpoint completion is a route-boundary continuation event;
      // only STOP owns a terminal waypoint comparison here.
      endpoint_error = endpoint.has_value()
          ? (endpoint->position_world - target_position).norm()
          : std::numeric_limits<double>::infinity();
      terminal_endpoint_contract = committed_bundle && endpoint &&
          terminalStopEndpointContractValid(
              effective_terminal_stop, committed_bundle->kind,
              committed_bundle->role, endpoint->role);
    }
    completed_trajectory_reaches_goal = completion_goal_matches &&
        terminalStopCompletionObserved(
          effective_terminal_stop, terminal_endpoint_contract, completed_endpoint_valid,
          endpoint_error, measured_goal_error, completion_tolerance);
    bool completion_identity_current = false;
    if (completion_goal_matches) {
      RCLCPP_INFO(get_logger(),
                  "planner backend trajectory completion observed reaches_goal=%d endpoint_error=%.3f "
                  "measured_goal_error=%.3f goal=(%.2f,%.2f,%.2f)",
                  completed_trajectory_reaches_goal ? 1 : 0,
                  endpoint_error, measured_goal_error,
                  pointFromMessage(plannerTarget(*goal), 0),
                  pointFromMessage(plannerTarget(*goal), 1),
                  pointFromMessage(plannerTarget(*goal), 2));
    } else {
      RCLCPP_INFO(
          get_logger(),
          "planner completion witness belongs to executing request=%lu; desired request=%lu "
          "changed, so terminal comparison is suppressed",
          static_cast<unsigned long>(completion_witness.request_id),
          static_cast<unsigned long>(goal->request_id));
    }
    if (completed_trajectory_reaches_goal && completion_goal_matches) {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const auto committed_execution_goal = executing_goal_;
      const auto committed_goal_epoch = command_goal_epoch_.load(std::memory_order_acquire);
      if (committed_bundle && committed_execution_goal &&
          executingCommandIdentityMatchesLocked(
              *committed_execution_goal, committed_goal_epoch,
              localization_epoch_at_cycle, committed_bundle->bundle_generation)) {
        trajectory_reaches_goal_.store(completed_trajectory_reaches_goal,
                                       std::memory_order_release);
        terminal_bundle_generation_.store(
            completed_trajectory_reaches_goal && effective_terminal_stop &&
                terminal_endpoint_contract
                ? committed_bundle->bundle_generation
                : 0U,
            std::memory_order_release);
        completion_identity_current = true;
        applyExecutionRecoveryEventLocked(
            ExecutionRecoveryEvent::kTerminalStopCompleted);
        recovery_state = execution_recovery_state_.load(std::memory_order_acquire);
      }
    }
    if (!completion_identity_current) completed_trajectory_reaches_goal = false;
  }
  const bool continue_completed_pass_through =
      completedPassThroughRequiresContinuation(
          completed_trajectory, completed_endpoint_valid,
          pass_through_goal, has_outgoing_route);
  if (completed_trajectory_for_planning && goal && completed_trajectory_reaches_goal &&
      !continue_completed_pass_through) {
    return;
  }
  if (completed_trajectory_for_planning && goal &&
      (!completed_trajectory_reaches_goal || continue_completed_pass_through)) {
    const auto& target_message = plannerTarget(*goal);
    const double dx = pointFromMessage(target_message, 0) - execution_state.position_world.x();
    const double dy = pointFromMessage(target_message, 1) - execution_state.position_world.y();
    const double dz = pointFromMessage(target_message, 2) - execution_state.position_world.z();
    if (continue_completed_pass_through ||
        std::sqrt(dx * dx + dy * dy + dz * dz) >
            goalCompletionTolerance(*goal)) {
      bool restart_requested = false;
      {
        std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
        std::lock_guard<std::mutex> input_lock(input_mutex_);
        std::lock_guard<std::mutex> command_lock(
            command_execution_lease_failure_latch_.transitionMutex());
        if (desiredGoalIdentityMatchesLocked(
                *goal, goal_epoch, localization_epoch_at_cycle)) {
          execution_episode_.requestRestartFromRest();
          hot_goal_transition_ = false;
          restart_from_rest = true;
          skip_replan_once_.store(false, std::memory_order_release);
          restart_requested = true;
        }
      }
      if (restart_requested) {
        RCLCPP_INFO(get_logger(),
                    "planner backend finite trajectory requires continuation; restarting PlanFromRest "
                    "pass_through=%d goal=(%.2f,%.2f,%.2f) vehicle=(%.2f,%.2f,%.2f)",
                    continue_completed_pass_through ? 1 : 0,
                    pointFromMessage(target_message, 0), pointFromMessage(target_message, 1),
                    pointFromMessage(target_message, 2), execution_state.position_world.x(),
                    execution_state.position_world.y(), execution_state.position_world.z());
      }
    }
  }

  if (!goal) return;
  if (!route_snapshot.has_value() ||
      !routeSnapshotMatchesGoalMirrors(*route_snapshot, *goal)) {
    if (cycle_goal) {
      (void)clearCommandForCurrentIdentity(
          *cycle_goal, goal_epoch_at_cycle, localization_epoch_at_cycle,
          expected_timeline_at_cycle);
    }
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "planner cycle rejected an invalid immutable route snapshot");
    return;
  }
  // A terminal bundle already observed by the command publisher is a bounded
  // endpoint hold, not a frontier trajectory.  Do not start another
  // PlanFromRest while that exact bundle is waiting for mission acceptance.
  if (terminalHoldIsPending(
          execution_episode_.snapshot().command_available,
          trajectory_reaches_goal_.load(std::memory_order_acquire),
          effective_terminal_stop,
          terminal_bundle_generation_.load(std::memory_order_acquire))) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (active_goal_ && active_goal_->mission_id == goal->mission_id &&
        active_goal_->waypoint_index == goal->waypoint_index &&
        active_goal_->request_id == goal->request_id) {
      execution_episode_.clearRestartFromRest();
      hot_goal_transition_ = false;
    }
    return;
  }
  // A terminal planner failure remains terminal for this request until the
  // mission controller acknowledges it and cancels the goal. Without this
  // gate the next planning timer could perform a fourth solve and overwrite
  // the fail-closed state with a newly discovered frontier trajectory.
  if (execution_episode_.snapshot().failure_latched) return;
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
  const double retained_tracking_limit_m = retainedCommandTrackingLimit(
      planner_->trackingErrorBudgetMeters(),
      navigation_contracts::kCommandAnchorErrorLimitM);
  const bool anchor_recovery_due = commandAnchorRecoveryDue(
      execution_episode_.snapshot().command_available, transition_role,
      transition_anchor_error_m,
      retained_tracking_limit_m);
  // Anchor pressure and hot retargeting may schedule a solve,
  // but every moving nominal renewal is anchored to committed future PVAJ.
  // Exceeding the certificate is handled by the one-shot emergency path after
  // retained validation; it never authorizes measured-state nominal planning.
  const bool anchor_renewal_replan = anchor_recovery_due && !plan_from_rest;
  const bool stop_waypoint = goal &&
      goal->behavior == navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP;
  const double terminal_stop_relative_speed_mps = transition_sample &&
      execution_state.finite()
      ? (transition_sample->velocity_world - execution_state.velocity_world).norm()
      : std::numeric_limits<double>::quiet_NaN();
  const double terminal_stop_projected_anchor_error_m =
      projectedRetainedAnchorErrorUpperBound(
          transition_anchor_error_m, terminal_stop_relative_speed_mps,
          planning_interval_s);
  const bool terminal_stop_current_state_known_free = latest_world &&
      latest_world.view && execution_state.finite() &&
      latest_world.view->classify(
          execution_state.position_world,
          navigation_world_model::GridLayer::kInflated) ==
          navigation_world_model::CellState::kKnownFree;
  const bool terminal_stop_projected_tracking_certificate_exceeded =
      terminal_stop_current_state_known_free &&
      std::isfinite(terminal_stop_projected_anchor_error_m) &&
      terminal_stop_projected_anchor_error_m > retained_tracking_limit_m;
  const bool terminal_stop_recovery_due = anchor_recovery_due ||
      terminal_stop_projected_tracking_certificate_exceeded;
  const bool terminal_stop_anchor_deferred = transition_bundle &&
      terminalStopMayDeferAnchorRecovery(
          stop_waypoint,
          execution_episode_.snapshot().command_available,
          transition_bundle->hasDeclaredEndpointMetadata(),
          transition_bundle->terminal_stop,
          transition_role,
          terminal_stop_recovery_due,
          transition_anchor_error_m,
          retained_tracking_limit_m,
          terminal_stop_projected_tracking_certificate_exceeded);
  if (terminal_stop_anchor_deferred) {
    ++optimizer_deferred_count_;
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "deferring terminal STOP optimizer renewal anchor=%d error=%.3f limit=%.3f",
        terminal_stop_anchor_deferred ? 1 : 0,
        transition_anchor_error_m, retained_tracking_limit_m);
    return;
  }
  const bool plan_from_rest_with_transition = plan_from_rest;
  if (plan_from_rest_with_transition &&
      (!std::isfinite(measured_speed_mps) ||
       measured_speed_mps > navigation_planning::PlanningTimingContract::kStationarySpeedMps)) {
    // A moving vehicle may only renew from an execution anchor. Initial
    // planning is a stopped-state operation; retaining this request here
    // would recreate the old moving-PlanFromRest stop/go loop.
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "initial planning deferred until measured state is stationary speed=%.3f",
        measured_speed_mps);
    return;
  }
  const bool replan_for_new_goal = hotRetargetUsesCommittedFutureState(
      hot_goal_transition,
      execution_episode_.snapshot().command_available,
      transition_role, transition_anchor_error_m,
      retained_tracking_limit_m);
  if (new_goal) {
    // MissionController has invalidated the previous waypoint already. Do
    // not publish that waypoint while PlanFromRest runs.
    std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    if (!goal || !desiredGoalIdentityMatchesLocked(
            *goal, goal_epoch, localization_epoch_at_cycle)) {
      return;
    }
    const auto current_bundle = command_bundle_store_.load();
    const auto terminal_generation = terminal_bundle_generation_.load(
        std::memory_order_acquire);
    const bool terminal_hold_belongs_to_current_goal = current_bundle &&
        terminal_generation != 0U &&
        current_bundle->bundle_generation == terminal_generation &&
        current_bundle->request_id == goal->request_id &&
        current_bundle->goal_epoch == goal_epoch &&
        terminalHoldIsPending(
            execution_episode_.snapshot().command_available,
            trajectory_reaches_goal_.load(std::memory_order_acquire),
            effective_terminal_stop, terminal_generation);
    if (!terminal_hold_belongs_to_current_goal) {
      execution_episode_.suspendCommand();
      trajectory_reaches_goal_.store(false);
      terminal_bundle_generation_.store(0U);
    }
  }
  if (!plan_from_rest_with_transition && !replan_for_new_goal &&
      !anchor_renewal_replan &&
      skip_replan_once_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  const bool forced_transition = plan_from_rest_with_transition ||
      replan_for_new_goal || anchor_renewal_replan;
  const auto renewal_episode = execution_episode_.snapshot();
  const auto renewal_decision = classifyPlannerRenewal(
      forced_transition,
      renewal_episode.command_available,
      renewal_episode.safety_suffix_active,
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
  const double remaining_route_m = [&]() {
    const double euclidean_distance = execution_state.finite()
        ? (target - execution_state.position_world).norm()
        : std::numeric_limits<double>::quiet_NaN();
    if (route_snapshot && execution_state.finite() &&
        route_snapshot->active_waypoint_index <
            route_snapshot->waypoint_arc_lengths_m.size()) {
      const double target_arc = route_snapshot->waypoint_arc_lengths_m[
          route_snapshot->active_waypoint_index];
      double best_arc = route_snapshot->measured_progress.valid &&
          std::isfinite(route_snapshot->measured_progress.progress_arc_m)
          ? route_snapshot->measured_progress.progress_arc_m : 0.0;
      double best_distance_squared = std::numeric_limits<double>::infinity();
      for (const auto& segment : route_snapshot->segments) {
        if (!segment.start.allFinite() || !segment.end.allFinite() ||
            !std::isfinite(segment.length_m) || segment.length_m <= 0.0 ||
            !std::isfinite(segment.start_arc_m) ||
            !std::isfinite(segment.end_arc_m)) {
          continue;
        }
        const Eigen::Vector3d delta = execution_state.position_world - segment.start;
        const double fraction = std::clamp(
            delta.dot(segment.tangent) / segment.length_m, 0.0, 1.0);
        const Eigen::Vector3d projection = segment.start + fraction *
            (segment.end - segment.start);
        const double distance_squared =
            (execution_state.position_world - projection).squaredNorm();
        if (!std::isfinite(distance_squared) ||
            distance_squared >= best_distance_squared) {
          continue;
        }
        best_distance_squared = distance_squared;
        best_arc = segment.start_arc_m + fraction * segment.length_m;
      }
      if (std::isfinite(target_arc) && std::isfinite(best_arc)) {
        const double route_remaining = std::max(0.0, target_arc - best_arc);
        // The Euclidean lower bound makes the phase conservative when the
        // vehicle is temporarily offset by an obstacle detour: it can enter
        // the slow approach early, never late, without changing the exact
        // route, backup, or dynamic certificates.
        return std::min(route_remaining, euclidean_distance);
      }
    }
    return euclidean_distance;
  }();
  const double effective_cruise_speed_mps =
      planner_->diagnostics().effective_cruise_speed_mps;
  if (!std::isfinite(effective_cruise_speed_mps) ||
      effective_cruise_speed_mps <= 0.0) {
    throw std::logic_error(
        "planner control envelope did not provide a finite effective cruise speed");
  }
  const bool terminal_stop_approach_due = plannerTerminalStopApproachDue(
      stop_waypoint, remaining_route_m,
      effective_cruise_speed_mps,
      mission_dynamic_limits_.vehicle.maximum_acceleration_mps2,
      mission_dynamic_limits_.vehicle.maximum_jerk_mps3);
  if (plan_from_rest_with_transition && stop_waypoint) {
    const double stop_distance = plannerTerminalStopBrakingDistanceM(
        effective_cruise_speed_mps,
        mission_dynamic_limits_.vehicle.maximum_acceleration_mps2,
        mission_dynamic_limits_.vehicle.maximum_jerk_mps3);
    RCLCPP_INFO(
        get_logger(),
        "terminal STOP phase remaining_route=%.3f braking_horizon=%.3f approach=%d",
        remaining_route_m, stop_distance,
        terminal_stop_approach_due ? 1 : 0);
  }
  navigation_planning::PlannerStatus result = navigation_planning::PlannerStatus::kFailed;
  const auto solve_generation_value = advanceMonotonicId(planner_solve_generation_);
  if (!solve_generation_value) {
    if (cycle_goal) {
      (void)clearCommandForCurrentIdentity(
          *cycle_goal, goal_epoch_at_cycle, localization_epoch_at_cycle,
          expected_timeline_at_cycle);
    }
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
    if (cycle_goal) {
      (void)clearCommandForCurrentIdentity(
          *cycle_goal, goal_epoch_at_cycle, localization_epoch_at_cycle,
          expected_timeline_at_cycle);
    }
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
  navigation_planning::PlanningRequest planning_request;
  // PlanningRequest::start_mode selects the explicit
  // planSuccessorFromExecutionAnchor lifecycle for every moving renewal;
  // only a stationary transition selects the initial stopped-state operation.
  planning_request.key = effective_scheduled_key;
  planning_request.key.start_mode = plan_from_rest_with_transition
      ? navigation_planning::PlanningStartMode::kStoppedMeasuredState
      : effective_scheduled_key.start_mode;
  planning_request.goal.localization_epoch = localization_epoch_at_solve;
  planning_request.goal.goal_epoch = goal_epoch;
  planning_request.goal.mission_id = goal->mission_id;
  planning_request.goal.waypoint_index = goal->waypoint_index;
  planning_request.goal.request_id = goal->request_id;
  planning_request.goal.target_world = target;
  planning_request.start_state = execution_state;
  planning_request.history.previous_bundle_generation =
      transition_bundle ? transition_bundle->bundle_generation : 0U;
  // PlanningHistory describes a prior executable bundle, not the measured
  // start state. Keep it empty for the initial stopped-state request; putting
  // the current velocity beside generation zero makes the request appear to
  // reference a non-existent prior command and fails the typed contract.
  planning_request.history.previous_velocity_world = transition_bundle
      ? execution_state.velocity_world
      : Eigen::Vector3d::Zero();
  planning_request.route_snapshot = *route_snapshot;
  planning_request.world = pinned_world.view;
  planning_request.dynamics = mission_dynamic_limits_;
  if (planning_request.key.start_mode ==
      navigation_planning::PlanningStartMode::kCommittedFutureState) {
    const auto activation_lead_ns = static_cast<std::int64_t>(
        navigation_planning::PlanningTimingContract::kStitchDurationS * 1.0e9);
    if (activation_lead_ns <= 0 ||
        now_ns > std::numeric_limits<std::int64_t>::max() - activation_lead_ns) {
      last_planning_outcome_.store(
          static_cast<std::uint8_t>(
              navigation_planning::CompletePlanningOutcome::kNoCompleteBundle),
          std::memory_order_release);
      last_planning_failure_stage_.store(
          static_cast<std::uint8_t>(
              navigation_planning::PlanningFailureStage::kCommitRecertification),
          std::memory_order_release);
      last_planning_failure_reason_.store(
          static_cast<std::uint8_t>(
              navigation_planning::PlanningFailureReason::kNoCompleteBundleAtDeadline),
          std::memory_order_release);
      return;
    }
    const auto activation_stamp_ns = now_ns + activation_lead_ns;
    const auto anchor = command_bundle_store_.reserveAnchor(
        execution_stamp_ns, activation_stamp_ns);
    if (!anchor) {
      last_planning_outcome_.store(
          static_cast<std::uint8_t>(
              navigation_planning::CompletePlanningOutcome::kNoCompleteBundle),
          std::memory_order_release);
      last_planning_failure_stage_.store(
          static_cast<std::uint8_t>(
              navigation_planning::PlanningFailureStage::kCommitRecertification),
          std::memory_order_release);
      last_planning_failure_reason_.store(
          static_cast<std::uint8_t>(
              navigation_planning::PlanningFailureReason::kNoCompleteBundleAtDeadline),
          std::memory_order_release);
      return;
    }
    planning_request.anchor = *anchor;
    planning_request.activation_stamp_ns = activation_stamp_ns;
  }
  planning_request.budget.deadline = navigation_planning::PlanningBudget::Clock::now() +
      std::chrono::duration_cast<navigation_planning::PlanningBudget::Clock::duration>(
          std::chrono::duration<double>(planner_->solveDeadlineSeconds()));
  navigation_planning::PlanningOutcome planning_outcome;
  try {
    planning_outcome = planner_->plan(planning_request);
    if (planning_outcome.outcome ==
            navigation_planning::CompletePlanningOutcome::kRetainedCommittedBundle) {
      result = navigation_planning::PlannerStatus::kNoNeed;
    } else if (navigation_planning::completePlanningSucceeded(
                   planning_outcome.outcome)) {
      result = navigation_planning::PlannerStatus::kSuccess;
    } else {
      result = navigation_planning::PlannerStatus::kFailed;
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "planner backend planner exception: %s", error.what());
    result = navigation_planning::PlannerStatus::kEmergency;
    planning_outcome.outcome =
        navigation_planning::CompletePlanningOutcome::kNoCompleteBundle;
    planning_outcome.failure_stage =
        navigation_planning::PlanningFailureStage::kInput;
    planning_outcome.failure_reason =
        navigation_planning::PlanningFailureReason::kInvalidInput;
  }
  // Test/debug-only observability hook. It is deliberately applied after the
  // real planner call and before admission, so the old committed bundle is
  // still available to the normal retained-command path. With the default
  // cycle id of zero this branch is unreachable in product operation.
  const bool injected_replan_failure = inject_failed_replan_once_ &&
      inject_failed_replan_cycle_id_ != 0U &&
      cycle_count_ == inject_failed_replan_cycle_id_ &&
      !plan_from_rest_with_transition;
  const bool safe_margin_injection = inject_failed_replan_when_safe_ &&
      inject_failed_replan_once_ && !plan_from_rest_with_transition &&
      execution_recovery_state_.load(std::memory_order_acquire) ==
          ExecutionRecoveryState::kTrackMain &&
      transition_bundle && transition_sample &&
      transition_role == navigation_planning::CandidateRole::kMain &&
      !execution_episode_.snapshot().safety_suffix_active &&
      transition_bundle->backup_available &&
      std::isfinite(transition_elapsed_s) &&
      transition_bundle->backup_start_time_s - transition_elapsed_s >= 1.5 &&
      std::isfinite(retained_tracking_limit_m) &&
      retained_tracking_limit_m > 0.0 &&
      std::isfinite(transition_anchor_error_m) &&
      transition_anchor_error_m <= 0.40 * retained_tracking_limit_m &&
      latest_world && world_freshness == navigation_execution::TimestampFreshness::VALID &&
      execution_freshness == navigation_execution::TimestampFreshness::VALID &&
      planner_->validateCommittedTrajectory(latest_world.view, now().seconds()).valid;
  const bool handoff_safe_margin_injection = inject_failed_replan_after_handoff_ &&
      inject_failed_replan_once_ && !plan_from_rest_with_transition &&
      replan_for_new_goal && execution_recovery_state_.load(std::memory_order_acquire) ==
          ExecutionRecoveryState::kTrackMain &&
      transition_bundle && transition_sample &&
      transition_role == navigation_planning::CandidateRole::kMain &&
      !execution_episode_.snapshot().safety_suffix_active &&
      transition_bundle->backup_available &&
      std::isfinite(transition_elapsed_s) &&
      transition_bundle->backup_start_time_s - transition_elapsed_s >= 1.5 &&
      std::isfinite(retained_tracking_limit_m) && retained_tracking_limit_m > 0.0 &&
      std::isfinite(transition_anchor_error_m) &&
      transition_anchor_error_m <= 0.40 * retained_tracking_limit_m &&
      latest_world && world_freshness == navigation_execution::TimestampFreshness::VALID &&
      execution_freshness == navigation_execution::TimestampFreshness::VALID &&
      planner_->validateCommittedTrajectory(latest_world.view, now().seconds()).valid;
  const bool repeated_replan_failure = inject_failed_replan_repeated_ &&
      !plan_from_rest_with_transition && transition_bundle && transition_sample &&
      transition_role == navigation_planning::CandidateRole::kMain &&
      execution_recovery_state_.load(std::memory_order_acquire) ==
          ExecutionRecoveryState::kTrackMain &&
      !execution_episode_.snapshot().safety_suffix_active &&
      transition_bundle->backup_available && std::isfinite(transition_elapsed_s) &&
      transition_elapsed_s < transition_bundle->backup_start_time_s &&
      std::isfinite(transition_anchor_error_m) &&
      transition_anchor_error_m <= retained_tracking_limit_m && latest_world &&
      world_freshness == navigation_execution::TimestampFreshness::VALID &&
      execution_freshness == navigation_execution::TimestampFreshness::VALID &&
      planner_->validateCommittedTrajectory(latest_world.view, now().seconds()).valid;
  const bool repeated_plan_from_rest_failure = inject_failed_plan_from_rest_repeated_ &&
      plan_from_rest_with_transition &&
      execution_recovery_state_.load(std::memory_order_acquire) ==
          ExecutionRecoveryState::kStoppedRecovery;
  const bool injected_failure = injected_replan_failure || safe_margin_injection ||
      handoff_safe_margin_injection || repeated_replan_failure ||
      repeated_plan_from_rest_failure;
  if (injected_failure) {
    if (!inject_failed_replan_repeated_ &&
        !inject_failed_plan_from_rest_repeated_) {
      inject_failed_replan_once_ = false;
    }
    result = navigation_planning::PlannerStatus::kFailed;
    planning_outcome.outcome =
        navigation_planning::CompletePlanningOutcome::kNoCompleteBundle;
    planning_outcome.failure_stage = navigation_planning::PlanningFailureStage::kInput;
    planning_outcome.failure_reason = navigation_planning::PlanningFailureReason::kInvalidInput;
    RCLCPP_WARN(get_logger(),
                "diagnostic injection converted planning cycle=%lu to a failed planning result",
                static_cast<unsigned long>(cycle_count_));
  }
  last_planning_outcome_.store(static_cast<int>(planning_outcome.outcome),
                               std::memory_order_release);
  last_planning_failure_stage_.store(
      static_cast<int>(planning_outcome.failure_stage), std::memory_order_release);
  last_planning_failure_reason_.store(
      static_cast<int>(planning_outcome.failure_reason), std::memory_order_release);
  std::uint64_t expected_active_generation = solve_generation;
  active_planner_solve_generation_.compare_exchange_strong(
      expected_active_generation, 0U);
  last_planner_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - planner_started).count();
  const bool timed_out =
      timed_out_planner_solve_generation_.load() == solve_generation;
  const bool stopped_recovery_retry =
      plan_from_rest_with_transition &&
      execution_recovery_state_.load(std::memory_order_acquire) ==
          ExecutionRecoveryState::kStoppedRecovery;
  if (timed_out && !stopped_recovery_retry) {
    RCLCPP_ERROR(get_logger(),
                 "Discarding planner backend solve generation=%lu after planner watchdog timeout",
                 static_cast<unsigned long>(solve_generation));
    planner_->discardCommandCandidate();
    return;
  }
  if (timed_out) {
    // The immutable STOPPED_HOLD remains exposed while the measured-state
    // solve is retried. Treat the canceled solve as a bounded PlanFromRest
    // failure so the stopped-recovery deadline can decide whether to continue
    // or fail closed.
    result = navigation_planning::PlannerStatus::kFailed;
    planning_outcome.outcome =
        navigation_planning::CompletePlanningOutcome::kNoCompleteBundle;
    planning_outcome.failure_stage =
        navigation_planning::PlanningFailureStage::kDeadline;
    planning_outcome.failure_reason =
        navigation_planning::PlanningFailureReason::kNoCompleteBundleAtDeadline;
    RCLCPP_WARN(
        get_logger(),
        "planner backend stopped-recovery solve generation=%lu exceeded watchdog; "
        "retaining bounded hold within the stopped-recovery deadline",
        static_cast<unsigned long>(solve_generation));
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
  if (terminalHoldIsPending(
          execution_episode_.snapshot().command_available,
          trajectory_reaches_goal_.load(std::memory_order_acquire),
          effective_terminal_stop,
          terminal_bundle_generation_.load(std::memory_order_acquire))) {
    // The command publisher may have observed the terminal endpoint while
    // this solve was already in flight. The execution witness now owns the
    // request; discard every late planner disposition, including retry and
    // fail-closed paths, so it cannot erase the terminal hold or schedule a
    // replacement solve.
    planner_->discardCommandCandidate();
    return;
  }
  // The backend stages a certified candidate; the execution store commits it
  // once below.  Its planning-history generation intentionally changes only
  // after the execution commit ACK, so it is not evidence of a ready command.
  const bool solve_committed_new_generation = planner_->hasStagedCommandCandidate();
  const auto disposition = classifyPlannerResult(
      result, plan_from_rest_with_transition,
      execution_episode_.snapshot().command_available && !stopped_recovery_retry,
      solve_committed_new_generation);
  if (disposition != PlannerResultDisposition::CommandReady) {
    // A staged candidate is private planner state until the execution store
    // commits it. Every other disposition is discard-only, including an
    // emergency candidate that cannot pass the normal command boundary.
    planner_->discardCommandCandidate();
  }
  if (disposition == PlannerResultDisposition::FailClosed) {
    std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    if (goal && desiredGoalIdentityMatchesLocked(
            *goal, goal_epoch, localization_epoch_at_solve)) {
      failClosedLocked();
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "planner backend planning failed (%d)", static_cast<int>(result));
  }
  if (disposition == PlannerResultDisposition::RestartFromRest) {
    bool restart_identity_current = false;
    {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      restart_identity_current = goal && desiredGoalIdentityMatchesLocked(
          *goal, goal_epoch, localization_epoch_at_solve);
      if (restart_identity_current) {
        trajectory_reaches_goal_.store(false);
        terminal_bundle_generation_.store(0U);
        const auto committed_timeline = command_bundle_store_.snapshot();
        const auto committed_bundle = committed_timeline.active;
        const auto committed_goal_epoch = command_goal_epoch_.load(
            std::memory_order_acquire);
        const auto execution_episode = execution_episode_.snapshot();
        const bool command_exposure_allowed =
            command_execution_lease_failure_latch_.allowsCommandExposure();
        const bool scheduled_bundle_current = committed_bundle &&
            effective_scheduled_key.committed_bundle_generation != 0U &&
            committed_bundle->bundle_generation ==
                effective_scheduled_key.committed_bundle_generation;
        const bool completion_identity_current = !execution_episode.failure_latched &&
            command_exposure_allowed && scheduled_bundle_current &&
            executing_goal_ &&
            executingCommandIdentityMatchesLocked(
                *executing_goal_, committed_goal_epoch, localization_epoch_at_solve,
                committed_bundle->bundle_generation);
        if (completion_identity_current) {
          trajectory_completion_witness_ = TrajectoryCompletionWitness{
              committed_bundle->bundle_generation,
              committed_timeline.version,
              committed_bundle->localization_epoch,
              committed_bundle->goal_epoch,
              committed_bundle->request_id,
              executing_goal_->mission_id,
              executing_goal_->waypoint_index};
        } else {
          // A restart result without a current committed generation is only
          // restart intent; it is not evidence that a trajectory completed.
          trajectory_completion_witness_.reset();
        }
        execution_episode_.requestRestartFromRest();
        hot_goal_transition_ = false;
      }
    }
    if (!restart_identity_current) {
      RCLCPP_DEBUG(get_logger(),
                   "discarding stale planner local-boundary result for mission=%s waypoint=%u request=%lu",
                   goal->mission_id.c_str(), goal->waypoint_index,
                   static_cast<unsigned long>(goal->request_id));
      return;
    }
    RCLCPP_INFO(get_logger(),
                "planner backend local trajectory boundary reached; scheduling PlanFromRest");
  }
  if (disposition == PlannerResultDisposition::RetryFromRest) {
    bool stopped_recovery_timeout = false;
    bool failure_identity_current = false;
    bool terminal_hold_pending = false;
    double failure_window_s = std::numeric_limits<double>::quiet_NaN();
    {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (goal && desiredGoalIdentityMatchesLocked(
              *goal, goal_epoch, localization_epoch_at_solve)) {
        failure_identity_current = true;
        terminal_hold_pending = terminalHoldIsPending(
            execution_episode_.snapshot().command_available,
            trajectory_reaches_goal_.load(std::memory_order_acquire),
            effective_terminal_stop,
            terminal_bundle_generation_.load(std::memory_order_acquire));
        if (!terminal_hold_pending) {
          trajectory_reaches_goal_.store(false);
          terminal_bundle_generation_.store(0U);
        }
        const auto failure_now_ns = navigation_common::steadyClockNowNanoseconds();
        if (plan_from_rest_first_failure_steady_ns_ == 0) {
          plan_from_rest_first_failure_steady_ns_ = failure_now_ns;
        }
        failure_window_s = static_cast<double>(
            static_cast<long double>(failure_now_ns) -
            static_cast<long double>(plan_from_rest_first_failure_steady_ns_)) * 1.0e-9;
        const auto timeout_state = execution_recovery_state_.load(
            std::memory_order_acquire);
        stopped_recovery_timeout = stoppedPlanningTimeoutMayFailClosed(
            timeout_state,
            std::isfinite(measured_speed_mps) && measured_speed_mps <=
                navigation_planning::PlanningTimingContract::kStationarySpeedMps,
            failure_window_s, stopped_recovery_timeout_s_);
        if (stopped_recovery_timeout) failClosedLocked();
      }
    }
    if (!failure_identity_current) {
      RCLCPP_DEBUG(get_logger(),
                   "discarding stale PlanFromRest failure for mission=%s waypoint=%u request=%lu",
                   goal->mission_id.c_str(), goal->waypoint_index,
                   static_cast<unsigned long>(goal->request_id));
    } else if (stopped_recovery_timeout) {
      RCLCPP_ERROR(get_logger(),
                   "planner backend PlanFromRest recovery timeout elapsed=%.3f timeout=%.3f s; "
                   "fail-closed for mission=%s waypoint=%u",
                   failure_window_s, stopped_recovery_timeout_s_,
                   goal->mission_id.c_str(), goal->waypoint_index);
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "planner backend PlanFromRest transient failure (%d); retrying within stopped "
          "recovery deadline elapsed=%.3f timeout=%.3f s",
          static_cast<int>(result), failure_window_s, stopped_recovery_timeout_s_);
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
    const auto retained_validation_now_ns = now().nanoseconds();
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
    // Diagnostic-only temporal decomposition. The recovery predicates below
    // intentionally continue to use command_anchor_sample/anchor_error_m,
    // preserving the production evidence-phase decision. These samples use
    // the same immutable committed evaluator at the two exact timestamps and
    // do not widen the executable lease.
    navigation_planning::TrajectoryPoint command_sample_at_now;
    navigation_planning::TrajectoryPoint command_sample_at_state_source;
    const auto sampleCommittedBundleAtDeclaredStamp =
        [&](const std::int64_t stamp_ns,
            navigation_planning::TrajectoryPoint& output) {
          if (!committed || stamp_ns <= 0) return false;
          const auto sample = committed_bundle->sampleAtDeclaredStamp(stamp_ns);
          if (!sample) return false;
          output = *sample;
          return true;
        };
    const bool temporal_command_now_valid =
        sampleCommittedBundleAtDeclaredStamp(
            retained_validation_now_ns, command_sample_at_now);
    const bool temporal_command_source_valid = retained_execution_state &&
        sampleCommittedBundleAtDeclaredStamp(
            retained_execution_state->state.source_stamp_ns,
            command_sample_at_state_source);
    const bool temporal_state_valid = retained_execution_state &&
        retained_execution_state->state.position_world.allFinite() &&
        retained_execution_state->state.velocity_world.allFinite();
    const double anchor_error_raw_m = temporal_command_now_valid && temporal_state_valid
        ? (command_sample_at_now.position_world -
           retained_execution_state->state.position_world).norm()
        : std::numeric_limits<double>::quiet_NaN();
    const double anchor_error_time_aligned_m =
        temporal_command_source_valid && temporal_state_valid
        ? (command_sample_at_state_source.position_world -
           retained_execution_state->state.position_world).norm()
        : std::numeric_limits<double>::quiet_NaN();
    const double command_motion_over_state_age_m =
        temporal_command_now_valid && temporal_command_source_valid
        ? (command_sample_at_now.position_world -
           command_sample_at_state_source.position_world).norm()
        : std::numeric_limits<double>::quiet_NaN();
    const double velocity_residual_time_aligned_mps =
        temporal_command_source_valid && temporal_state_valid
        ? (command_sample_at_state_source.velocity_world -
           retained_execution_state->state.velocity_world).norm()
        : std::numeric_limits<double>::quiet_NaN();
    causal_evaluation_now_ns_.store(retained_validation_now_ns, std::memory_order_release);
    causal_execution_state_source_stamp_ns_.store(
        retained_execution_state ? retained_execution_state->state.source_stamp_ns : 0,
        std::memory_order_release);
    causal_execution_state_receive_stamp_ns_.store(
        retained_execution_state ? retained_execution_state->state.receive_stamp_ns : 0,
        std::memory_order_release);
    causal_execution_state_source_age_ms_.store(
        retained_state_freshness.source_age_ms, std::memory_order_release);
    causal_execution_state_receive_age_ms_.store(
        retained_state_freshness.receive_age_ms, std::memory_order_release);
    causal_committed_bundle_start_stamp_ns_.store(
        committed ? committed_bundle->declared_start_ns : 0,
        std::memory_order_release);
    const auto storeVector = [](const Eigen::Vector3d& value,
                                std::atomic<double>& x,
                                std::atomic<double>& y,
                                std::atomic<double>& z) {
      x.store(value.x(), std::memory_order_release);
      y.store(value.y(), std::memory_order_release);
      z.store(value.z(), std::memory_order_release);
    };
    const Eigen::Vector3d nan_vector =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    storeVector(temporal_state_valid
                    ? retained_execution_state->state.position_world : nan_vector,
                causal_measured_position_x_, causal_measured_position_y_,
                causal_measured_position_z_);
    storeVector(temporal_state_valid
                    ? retained_execution_state->state.velocity_world : nan_vector,
                causal_measured_velocity_x_, causal_measured_velocity_y_,
                causal_measured_velocity_z_);
    storeVector(temporal_command_now_valid
                    ? command_sample_at_now.position_world : nan_vector,
                causal_command_now_position_x_, causal_command_now_position_y_,
                causal_command_now_position_z_);
    storeVector(temporal_command_now_valid
                    ? command_sample_at_now.velocity_world : nan_vector,
                causal_command_now_velocity_x_, causal_command_now_velocity_y_,
                causal_command_now_velocity_z_);
    storeVector(temporal_command_source_valid
                    ? command_sample_at_state_source.position_world : nan_vector,
                causal_command_source_position_x_, causal_command_source_position_y_,
                causal_command_source_position_z_);
    storeVector(temporal_command_source_valid
                    ? command_sample_at_state_source.velocity_world : nan_vector,
                causal_command_source_velocity_x_, causal_command_source_velocity_y_,
                causal_command_source_velocity_z_);
    causal_anchor_error_raw_m_.store(anchor_error_raw_m, std::memory_order_release);
    causal_anchor_error_time_aligned_m_.store(
        anchor_error_time_aligned_m, std::memory_order_release);
    causal_command_motion_over_state_age_m_.store(
        command_motion_over_state_age_m, std::memory_order_release);
    causal_velocity_residual_time_aligned_mps_.store(
        velocity_residual_time_aligned_mps, std::memory_order_release);
    causal_retained_elapsed_s_.store(elapsed_s, std::memory_order_release);
    causal_committed_bundle_duration_s_.store(total_duration_s, std::memory_order_release);
    causal_validate_without_new_commit_.store(
        validate_without_new_commit, std::memory_order_release);
    causal_retained_fresh_vehicle_state_.store(
        fresh_vehicle_state, std::memory_order_release);
    causal_retained_committed_command_available_.store(
        committed, std::memory_order_release);
    causal_retained_command_anchor_valid_.store(
        command_anchor_valid, std::memory_order_release);
    causal_retained_safety_trajectory_available_.store(
        backup_available, std::memory_order_release);
    causal_retained_terminal_stop_.store(
        transition_bundle && transition_bundle->terminal_stop,
        std::memory_order_release);
    causal_retained_committed_role_.store(
        committed_bundle ? static_cast<int>(committed_bundle->role) : -1,
        std::memory_order_release);
    causal_retained_recovery_state_before_.store(
        static_cast<std::uint8_t>(execution_recovery_state_.load(
            std::memory_order_acquire)), std::memory_order_release);
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
    const auto latest_world = world_snapshot_store_.latest();
    const bool current_vehicle_state_known_free = fresh_vehicle_state && latest_world &&
        latest_world.view &&
        latest_world.view->classify(
            current_vehicle_position,
            navigation_world_model::GridLayer::kInflated) ==
            navigation_world_model::CellState::kKnownFree;
    causal_current_vehicle_state_known_free_.store(
        current_vehicle_state_known_free, std::memory_order_release);
    bool sampled_path_clear = committed;
    double first_blocked_sample_s = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d first_blocked_sample = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    navigation_world_model::CellState first_blocked_grid =
        navigation_world_model::CellState::kUnknown;
    if (sampled_path_clear) {
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
    causal_committed_safety_transition_time_s_.store(
        safety_transition_s, std::memory_order_release);
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
        anchor_error_m, retained_tracking_limit_m, sampled_path_clear);
    // A measured-state PlanFromRest attempt may fail while the currently
    // executing bundle is still a fresh, continuously certified bridge. Keep
    // that bridge alive until the bounded recovery budget is exhausted; a
    // single optimizer miss must not convert an otherwise safe recovery
    // opportunity into an immediate PX4 handover. This does not extend the
    // command lease and does not allow a non-finite, stale, blocked, or
    // over-error bundle to remain exposed.
    const bool recovery_bridge_usable = plan_from_rest_with_transition &&
        !execution_episode_.snapshot().failure_latched && committed &&
        fresh_vehicle_state && command_anchor_valid && sampled_path_clear &&
        std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
        std::isfinite(total_duration_s) && elapsed_s <= total_duration_s + 1.0e-9 &&
        std::isfinite(anchor_error_m) && anchor_error_m <= retained_tracking_limit_m;
    bool emergency_brake_committed = false;
    bool emergency_certification_failed = false;
    bool emergency_boundary_failed = false;
    const bool tracking_certificate_exceeded =
        std::isfinite(anchor_error_m) &&
        anchor_error_m > retained_tracking_limit_m;
    const bool projected_tracking_certificate_exceeded =
        std::isfinite(projected_anchor_error_m) &&
        projected_anchor_error_m > retained_tracking_limit_m;
    const bool emergency_authorized = measuredStateEmergencyMayReplaceCommittedCommand(
        validate_without_new_commit, use_safety_suffix,
        fresh_vehicle_state, committed, command_anchor_valid,
            tracking_certificate_exceeded,
            execution_recovery_state_.load(std::memory_order_acquire),
            committed_bundle
                ? committed_bundle->role
            : navigation_planning::CandidateRole::kEmergency,
            projected_tracking_certificate_exceeded,
            current_vehicle_state_known_free,
            backup_available,
            transition_bundle && transition_bundle->terminal_stop);
    std::uint8_t emergency_authorization_reason =
        navigation_contracts::msg::NavigationCommand::EMERGENCY_AUTHORIZATION_NONE;
    if (emergency_authorized) {
      if (!use_safety_suffix && tracking_certificate_exceeded) {
        emergency_authorization_reason = navigation_contracts::msg::NavigationCommand::
            EMERGENCY_AUTHORIZATION_ACTUAL_ANCHOR_CERTIFICATE_EXCEEDED;
      } else if (!tracking_certificate_exceeded &&
                 projected_tracking_certificate_exceeded) {
        emergency_authorization_reason = navigation_contracts::msg::NavigationCommand::
            EMERGENCY_AUTHORIZATION_PROJECTED_MAIN_ONLY_CERTIFICATE_EXCEEDED;
      } else {
        emergency_authorization_reason = navigation_contracts::msg::NavigationCommand::
            EMERGENCY_AUTHORIZATION_OTHER_INVALID;
      }
    }
    causal_anchor_error_m_.store(anchor_error_m, std::memory_order_release);
    causal_projected_anchor_error_m_.store(projected_anchor_error_m,
                                           std::memory_order_release);
    causal_retained_tracking_limit_m_.store(retained_tracking_limit_m,
                                             std::memory_order_release);
    causal_relative_anchor_speed_mps_.store(relative_anchor_speed_mps,
                                            std::memory_order_release);
    causal_backup_available_.store(backup_available, std::memory_order_release);
    causal_time_to_backup_start_s_.store(
        backup_available ? backup_start_s - elapsed_s : std::numeric_limits<double>::quiet_NaN(),
        std::memory_order_release);
    causal_committed_suffix_usable_.store(use_safety_suffix,
                                          std::memory_order_release);
    causal_sampled_path_clear_.store(sampled_path_clear,
                                     std::memory_order_release);
    causal_tracking_certificate_exceeded_.store(tracking_certificate_exceeded,
                                                std::memory_order_release);
    causal_projected_tracking_certificate_exceeded_.store(
        projected_tracking_certificate_exceeded, std::memory_order_release);
    causal_emergency_authorization_reason_.store(
        emergency_authorization_reason, std::memory_order_release);
    causal_planning_cycle_id_.store(cycle_count_, std::memory_order_release);
    causal_timestamp_ns_.store(now().nanoseconds(), std::memory_order_release);
    causal_emergency_candidate_commit_result_.store(0, std::memory_order_release);
    if (emergency_authorized) {
      // This is the only measured-state moving transition. Propagated P/V and
      // Propagated odometry does not expose measured A/J. Keep P/V and yaw
      // continuous, but do not promote finite-difference estimates into the
      // emergency command boundary.
      navigation_planning::TrajectoryPoint emergency_command =
          makeMeasuredEmergencyBoundary(
              command_anchor_sample,
              retained_execution_state &&
                  retained_execution_state->state.acceleration_estimated,
              retained_execution_state &&
                  retained_execution_state->state.jerk_estimated);
      if (retained_execution_state) {
        emergency_command.position_world =
            retained_execution_state->state.position_world;
        emergency_command.velocity_world =
            retained_execution_state->state.velocity_world;
        if (!retained_execution_state->state.acceleration_estimated) {
          emergency_command.acceleration_world =
              retained_execution_state->state.acceleration_world;
        }
        if (!retained_execution_state->state.jerk_estimated) {
          emergency_command.jerk_world =
              retained_execution_state->state.jerk_world;
        }
        emergency_command.yaw = retained_execution_state->state.yaw_rad;
        const double measured_altitude_m =
            emergency_command.position_world.z();
        const double command_anchor_altitude_m =
            command_anchor_sample.position_world.z();
        const double terminal_altitude_m =
            plannerEmergencyTerminalAltitude(
                measured_altitude_m, command_anchor_altitude_m,
                retained_tracking_limit_m);
        RCLCPP_WARN(
            get_logger(),
            "one-shot emergency altitude anchor measured=%.3f command=%.3f terminal=%.3f limit=%.3f",
            measured_altitude_m, command_anchor_altitude_m,
            terminal_altitude_m, retained_tracking_limit_m);
      emergency_brake_committed = planner_->commitEmergencyBrake(
          emergency_command, now().seconds(), terminal_altitude_m);
      }
      use_safety_suffix = emergency_brake_committed;
      emergency_certification_failed = !emergency_brake_committed;
      causal_emergency_candidate_commit_result_.store(
          emergency_brake_committed ? 1 : 2, std::memory_order_release);
      if (projected_tracking_certificate_exceeded && !tracking_certificate_exceeded) {
        RCLCPP_WARN(get_logger(),
                    "planner backend projected retained-command anchor beyond the "
                    "tracking envelope; committing one-shot measured-state recovery "
                    "before the next validation boundary anchor=%.3f projected=%.3f limit=%.3f",
                    anchor_error_m, projected_anchor_error_m, retained_tracking_limit_m);
      }
    }
    if (emergency_brake_committed &&
        !commitPlannerCandidate(*goal, goal_epoch, localization_epoch_at_solve,
                                now().nanoseconds(), effective_scheduled_key)) {
      emergency_brake_committed = false;
      use_safety_suffix = false;
      emergency_boundary_failed = true;
      RCLCPP_ERROR(get_logger(),
                   "execution boundary rejected the one-shot measured emergency candidate; "
                   "clearing command exposure");
    }
    const auto retained_transition = retainedValidationTransition(use_safety_suffix);
    // A visible main-only trajectory remains a MAIN command. Only an actual
    // atomic main-to-backup bundle is marked safety-owned at the PX4 boundary.
    {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
          active_localization_epoch_.load(std::memory_order_acquire) !=
              localization_epoch_at_solve ||
          active_goal_epoch_.load() != goal_epoch) {
        // A newer goal owns command state now. This old solve is discard-only:
        // never invalidate a deliberately transferred hot-retarget command.
      } else if (!command_execution_lease_failure_latch_.allowsCommandExposure()) {
        failClosedLocked();
      } else if (emergency_certification_failed) {
        applyExecutionRecoveryEventLocked(
            ExecutionRecoveryEvent::kEmergencyCertificationFailed);
        failClosedLocked();
      } else if (emergency_boundary_failed) {
        failClosedLocked();
      } else if (emergency_brake_committed) {
        const auto emergency_bundle = command_bundle_store_.load();
        if (emergency_bundle &&
            emergency_bundle->kind ==
                navigation_planning::CandidateBundleKind::kEmergencyBrake &&
            desiredGoalIdentityMatchesLocked(
                *goal, goal_epoch, localization_epoch_at_solve,
                emergency_bundle->bundle_generation)) {
          applyExecutionRecoveryEventLocked(
              ExecutionRecoveryEvent::kEmergencyCommitted);
        }
      } else if (recovery_bridge_usable) {
        // A hot-retarget recovery bridge may still be the previous physical
        // bundle.  Its execution epoch is immutable until the staged
        // successor activation boundary; never relabel it with the desired
        // goal epoch merely because the recovery solve missed.
        const auto retained_execution_bundle = command_bundle_store_.load();
        if (retained_execution_bundle &&
            retained_execution_bundle->goal_epoch == goal_epoch) {
          command_goal_epoch_.store(goal_epoch);
        }
        const auto retained_execution_goal = executing_goal_;
        const auto retained_command_epoch =
            command_goal_epoch_.load(std::memory_order_acquire);
        if (retained_execution_bundle && retained_execution_goal &&
            executingCommandIdentityMatchesLocked(
                *retained_execution_goal, retained_command_epoch,
                localization_epoch_at_solve,
                retained_execution_bundle->bundle_generation)) {
          execution_episode_.roleObserved(
              retained_execution_bundle->role,
              retained_execution_bundle->bundle_generation);
          execution_episode_.setSafetySuffix(use_safety_suffix);
        }
        trajectory_completion_witness_.reset();
      } else if (use_safety_suffix && validate_without_new_commit) {
        const auto retained_execution_goal = executing_goal_;
        const auto retained_command_epoch =
            command_goal_epoch_.load(std::memory_order_acquire);
        const bool retained_identity_current = committed_bundle &&
            retained_execution_goal && executingCommandIdentityMatchesLocked(
                *retained_execution_goal, retained_command_epoch,
                localization_epoch_at_solve, committed_bundle->bundle_generation);
        if (retained_identity_current) {
          execution_episode_.roleObserved(
              committed_bundle->role, committed_bundle->bundle_generation);
          execution_episode_.setSafetySuffix(true);
        }
      } else if (!validate_without_new_commit ||
                 retained_transition == RetainedValidationTransition::FailClosed) {
        const auto retained_execution_goal = executing_goal_;
        const auto retained_command_epoch =
            command_goal_epoch_.load(std::memory_order_acquire);
        const bool retained_identity_current = committed_bundle &&
            retained_execution_goal && executingCommandIdentityMatchesLocked(
                *retained_execution_goal, retained_command_epoch,
                localization_epoch_at_solve, committed_bundle->bundle_generation);
        if (use_safety_suffix && backup_available && !emergency_brake_committed &&
            retained_identity_current) {
          applyExecutionRecoveryEventLocked(
              ExecutionRecoveryEvent::kBackupActivated);
        }
        if (!use_safety_suffix) {
          command_goal_epoch_.store(0U);
          failClosedLocked();
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
        planner_->discardCommandCandidate();
        return;
      }
      if (timed_out_planner_solve_generation_.load() == solve_generation) {
        planner_->discardCommandCandidate();
        return;
      }
      // Keep the currently exposed command authoritative while the candidate
      // is exported, revalidated and committed.  Clearing availability here
      // creates an observable replacement window in which the sampler can see
      // no command even though the old immutable bundle is still executable.
      // A successful commit below swaps the store pointer first. A rejected
      // candidate leaves all previous command state untouched.
    }
    if (!commitPlannerCandidate(*goal, goal_epoch, localization_epoch_at_solve,
                                now().nanoseconds(), effective_scheduled_key,
                                planning_outcome.candidate)) {
      RCLCPP_WARN(get_logger(),
                  "execution boundary rejected planner candidate after solve; "
                  "retaining the previously exposed command when still current");
      return;
    }
    bool successor_staged = false;
    {
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
          active_localization_epoch_.load(std::memory_order_acquire) !=
              localization_epoch_at_solve || active_goal_epoch_.load() != goal_epoch) {
        // This solve is stale. It must be discard-only and cannot clear a
        // newer command identity that won the execution-store race.
        return;
      }
      if (!command_execution_lease_failure_latch_.allowsCommandExposure()) {
        return;
      }
      successor_staged = command_bundle_store_.hasPending();
      if (!successor_staged) {
        const auto committed_bundle = command_bundle_store_.load();
        if (!committed_bundle || !desiredGoalIdentityMatchesLocked(
                *goal, goal_epoch, localization_epoch_at_solve,
                committed_bundle->bundle_generation)) {
          // The store or desired identity changed after this solve. Treat the
          // callback as stale and leave the newer command identity untouched.
          return;
        }
        command_goal_epoch_.store(goal_epoch);
        // An immediate commit is the ownership boundary: a prior BACKUP
        // suffix and its completion markers must not contaminate the new
        // MAIN command sampled by the publisher.
        trajectory_completion_witness_.reset();
        trajectory_reaches_goal_.store(false, std::memory_order_release);
        terminal_bundle_generation_.store(0U, std::memory_order_release);
        applyExecutionRecoveryEventLocked(
            ExecutionRecoveryEvent::kMainCommitted);
      }
      if (pending_handoff_goal &&
          !pending_goal_owner_.consumeGoal(pending_handoff_goal)) {
        // The pending pointer is the ownership witness for this two-phase
        // handoff. A disappearance while solving is an ordering violation;
        // keep the validated active command, but make the loss observable.
        RCLCPP_WARN(get_logger(),
                    "committed stopped-suffix replacement but pending goal owner changed");
      }
    }
    if (!successor_staged) {
      const auto committed_bundle = command_bundle_store_.load();
      const auto committed_end_sample = committed_bundle &&
          committed_bundle->hasDeclaredEndpointMetadata()
          ? committed_bundle->sampleAtDeclaredEnd()
          : std::nullopt;
      const double completion_tolerance = goalCompletionTolerance(*goal);
      const bool committed_reaches_goal = stop_goal && committed_end_sample.has_value() &&
          (committed_end_sample->position_world - target).norm() <= completion_tolerance;
      {
        std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
        std::lock_guard<std::mutex> input_lock(input_mutex_);
        std::lock_guard<std::mutex> command_lock(
            command_execution_lease_failure_latch_.transitionMutex());
        const auto current_bundle = command_bundle_store_.load();
        if (current_bundle && committed_bundle &&
            current_bundle.get() == committed_bundle.get() &&
            desiredGoalIdentityMatchesLocked(
                *goal, goal_epoch, localization_epoch_at_solve,
                committed_bundle->bundle_generation)) {
          trajectory_reaches_goal_.store(committed_reaches_goal,
                                         std::memory_order_release);
        } else {
          return;
        }
      }
      if (stop_goal && committed_bundle && committed_bundle->hasDeclaredEndpointMetadata() &&
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
    }
    // A newly committed MAIN already passed the complete candidate and
    // execution-boundary contracts. Give it one scheduler interval to
    // establish a continuous command before evaluating its next renewal
    // deadline. Without this one-shot quiet period, a candidate whose
    // BACKUP switch is only slightly beyond the renewal lead is immediately
    // replaced on the next timer tick, producing a commit/replan train and
    // needlessly increasing hot-splice pressure. A new goal and a hard
    // anchor recovery remain higher priority at the next tick.
    std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    if (!goal || !desiredGoalIdentityMatchesLocked(
            *goal, goal_epoch, localization_epoch_at_solve)) {
      return;
    }
    skip_replan_once_.store(true, std::memory_order_release);
    plan_from_rest_first_failure_steady_ns_ = 0;
    // Do not fall back to hot replan after a failed new-goal attempt.  That
    // would keep publishing the previous waypoint while the mission has
    // already advanced.
    {
      if (new_goal) new_goal_ = false;
      if (!successor_staged && clearHotGoalTransitionAfterCommit(
              plan_from_rest_with_transition, replan_for_new_goal)) {
        hot_goal_transition_ = false;
      }
      if (!successor_staged) executing_goal_ = *goal;
      if (restart_from_rest) execution_episode_.clearRestartFromRest();
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
                endpoint_error, execution_episode_.snapshot().command_available,
                execution_episode_.snapshot().failure_latched,
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
    // Snapshot the identities without holding an input lock while reading the
    // execution timeline.  This preserves the established input -> timeline
    // lock order and makes the trace describe the same post-commit boundary
    // that the sampler will observe next.
    std::optional<navigation_contracts::msg::NavigationGoal> desired_trace_goal;
    std::optional<navigation_contracts::msg::NavigationGoal> executing_trace_goal;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      desired_trace_goal = active_goal_;
      executing_trace_goal = executing_goal_;
    }
    const auto timeline_trace = command_bundle_store_.snapshot();
    const auto transition_kind = classifyGoalTransition(
        desired_trace_goal, executing_trace_goal);
    const auto add_trace_value = [&trace_status](const std::string& key,
                                                  const auto& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = std::to_string(value);
      trace_status.values.push_back(std::move(item));
    };
    const auto add_trace_string = [&trace_status](const std::string& key,
                                                  const std::string_view value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
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
    const auto add_goal_identity = [&add_trace_value, &add_trace_string](
        const std::string_view prefix,
        const std::optional<navigation_contracts::msg::NavigationGoal>& goal,
        const std::uint64_t epoch) {
      const std::string key_prefix(prefix);
      add_trace_value(key_prefix + "_valid", goal ? 1 : 0);
      add_trace_value(key_prefix + "_epoch", epoch);
      if (!goal) return;
      add_trace_string(key_prefix + "_mission_id", goal->mission_id);
      add_trace_value(key_prefix + "_waypoint_index", goal->waypoint_index);
      add_trace_value(key_prefix + "_request_id", goal->request_id);
      add_trace_value(key_prefix + "_route_revision", goal->route.route_revision);
    };
    const auto add_bundle_identity = [&add_trace_value, &add_trace_string](
        const std::string_view prefix,
        const std::shared_ptr<const navigation_planning::CandidateBundle>& bundle) {
      const std::string key_prefix(prefix);
      add_trace_value(key_prefix + "_valid", bundle ? 1 : 0);
      if (!bundle) return;
      add_trace_value(key_prefix + "_localization_epoch", bundle->localization_epoch);
      add_trace_value(key_prefix + "_goal_epoch", bundle->goal_epoch);
      add_trace_value(key_prefix + "_request_id", bundle->request_id);
      add_trace_value(key_prefix + "_bundle_generation", bundle->bundle_generation);
      add_trace_value(key_prefix + "_activation_stamp_ns", bundle->activation_stamp_ns);
      add_trace_value(key_prefix + "_world_generation", bundle->world_identity.generation);
      add_trace_value(key_prefix + "_world_revision", bundle->world_identity.revision);
      add_trace_value(key_prefix + "_world_stamp_ns",
                      bundle->world_identity.observation_stamp_ns);
      add_trace_value(key_prefix + "_role", static_cast<int>(bundle->role));
      add_trace_value(key_prefix + "_backup_available", bundle->backup_available ? 1 : 0);
    };
    add_trace_string("transition_kind", goalTransitionKindName(transition_kind));
    add_goal_identity("desired", desired_trace_goal,
                      active_goal_epoch_.load(std::memory_order_acquire));
    add_goal_identity("executing", executing_trace_goal,
                      command_goal_epoch_.load(std::memory_order_acquire));
    add_bundle_identity("active_execution", timeline_trace.active);
    add_bundle_identity("pending_execution", timeline_trace.pending);
    add_trace_value("active_execution_timeline_version", timeline_trace.version);
    add_trace_value("pending_activation_ns", timeline_trace.pending_activation_ns);
    const auto& active_bundle = timeline_trace.active;
    const bool route_boundary_event_present = active_bundle &&
        active_bundle->route_boundary_event.has_value();
    add_trace_value("route_boundary_event_present", route_boundary_event_present ? 1 : 0);
    if (route_boundary_event_present) {
      const auto& boundary = *active_bundle->route_boundary_event;
      add_trace_value("route_boundary_stamp_ns", boundary.boundary_stamp_ns);
      add_trace_value("route_boundary_kind", static_cast<int>(boundary.kind));
      add_trace_value("route_boundary_constraint_valid",
                      active_bundle->route_boundary_constraint.has_value() &&
                          active_bundle->route_boundary_constraint->valid() ? 1 : 0);
      const auto boundary_sample = active_bundle->sampleAtDeclaredStamp(
          boundary.boundary_stamp_ns);
      add_trace_value("boundary_command_speed_mps",
                      boundary_sample ? boundary_sample->velocity_world.norm() : 0.0);
      add_trace_value("boundary_remaining_main_horizon_s",
                      boundary_sample
                          ? std::max(0.0, active_bundle->duration_s -
                                boundary_sample->trajectory_time_s)
                          : 0.0);
      add_trace_value("terminal_velocity_mps",
                      active_bundle->terminal_stop
                          ? active_bundle->sampleAtDeclaredStamp(active_bundle->declared_end_ns)
                                .value_or(navigation_planning::TrajectoryPoint{}).velocity_world.norm()
                          : 0.0);
    }
    add_trace_value("planning_outcome",
                    last_planning_outcome_.load(std::memory_order_acquire));
    add_trace_value("planning_failure_stage",
                    last_planning_failure_stage_.load(std::memory_order_acquire));
    add_trace_value("planning_failure_reason",
                    last_planning_failure_reason_.load(std::memory_order_acquire));
    add_trace_value("planning_cycle_id", cycle_count_);
    add_trace_value("bundle_id", committed_generation);
    add_trace_value("solve_generation", solve_generation);
    add_trace_value("injected_replan_failure", injected_failure ? 1 : 0);
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
    add_trace_value("execution_boundary_rejection",
                    last_execution_boundary_rejection_.load(std::memory_order_acquire));
    add_trace_value("anchor_error_m", causal_anchor_error_m_.load(std::memory_order_acquire));
    add_trace_value("projected_anchor_error_m",
                    causal_projected_anchor_error_m_.load(std::memory_order_acquire));
    add_trace_value("retained_tracking_limit_m",
                    causal_retained_tracking_limit_m_.load(std::memory_order_acquire));
    add_trace_value("relative_anchor_speed_mps",
                    causal_relative_anchor_speed_mps_.load(std::memory_order_acquire));
    add_trace_value("committed_suffix_usable",
                    causal_committed_suffix_usable_.load(std::memory_order_acquire) ? 1 : 0);
    add_trace_value("sampled_path_clear",
                    causal_sampled_path_clear_.load(std::memory_order_acquire) ? 1 : 0);
    add_trace_value("tracking_certificate_exceeded",
                    causal_tracking_certificate_exceeded_.load(std::memory_order_acquire) ? 1 : 0);
    add_trace_value("projected_tracking_certificate_exceeded",
                    causal_projected_tracking_certificate_exceeded_.load(
                        std::memory_order_acquire) ? 1 : 0);
    add_trace_value("backup_available",
                    causal_backup_available_.load(std::memory_order_acquire) ? 1 : 0);
    add_trace_value("time_to_backup_start_s",
                    causal_time_to_backup_start_s_.load(std::memory_order_acquire));
    add_trace_value("emergency_authorization_reason",
                    causal_emergency_authorization_reason_.load(std::memory_order_acquire));
    add_trace_value("causal_planning_cycle_id",
                    causal_planning_cycle_id_.load(std::memory_order_acquire));
    add_trace_value("causal_timestamp_ns",
                    causal_timestamp_ns_.load(std::memory_order_acquire));
    add_trace_value("emergency_candidate_commit_result",
                    causal_emergency_candidate_commit_result_.load(std::memory_order_acquire));
    add_trace_value("evaluation_now_ns",
                    causal_evaluation_now_ns_.load(std::memory_order_acquire));
    add_trace_value("execution_state_source_stamp_ns",
                    causal_execution_state_source_stamp_ns_.load(std::memory_order_acquire));
    add_trace_value("execution_state_receive_stamp_ns",
                    causal_execution_state_receive_stamp_ns_.load(std::memory_order_acquire));
    add_trace_value("execution_state_source_age_ms",
                    causal_execution_state_source_age_ms_.load(std::memory_order_acquire));
    add_trace_value("execution_state_receive_age_ms",
                    causal_execution_state_receive_age_ms_.load(std::memory_order_acquire));
    add_trace_value("committed_bundle_start_stamp_ns",
                    causal_committed_bundle_start_stamp_ns_.load(std::memory_order_acquire));
    add_trace_vector("measured_position_at_state_source", Eigen::Vector3d{
      causal_measured_position_x_.load(std::memory_order_acquire),
      causal_measured_position_y_.load(std::memory_order_acquire),
      causal_measured_position_z_.load(std::memory_order_acquire)});
    add_trace_vector("measured_velocity_at_state_source", Eigen::Vector3d{
      causal_measured_velocity_x_.load(std::memory_order_acquire),
      causal_measured_velocity_y_.load(std::memory_order_acquire),
      causal_measured_velocity_z_.load(std::memory_order_acquire)});
    add_trace_vector("committed_command_position_at_now", Eigen::Vector3d{
      causal_command_now_position_x_.load(std::memory_order_acquire),
      causal_command_now_position_y_.load(std::memory_order_acquire),
      causal_command_now_position_z_.load(std::memory_order_acquire)});
    add_trace_vector("committed_command_velocity_at_now", Eigen::Vector3d{
      causal_command_now_velocity_x_.load(std::memory_order_acquire),
      causal_command_now_velocity_y_.load(std::memory_order_acquire),
      causal_command_now_velocity_z_.load(std::memory_order_acquire)});
    add_trace_vector("committed_command_position_at_state_source", Eigen::Vector3d{
      causal_command_source_position_x_.load(std::memory_order_acquire),
      causal_command_source_position_y_.load(std::memory_order_acquire),
      causal_command_source_position_z_.load(std::memory_order_acquire)});
    add_trace_vector("committed_command_velocity_at_state_source", Eigen::Vector3d{
      causal_command_source_velocity_x_.load(std::memory_order_acquire),
      causal_command_source_velocity_y_.load(std::memory_order_acquire),
      causal_command_source_velocity_z_.load(std::memory_order_acquire)});
    add_trace_value("anchor_error_raw_m",
                    causal_anchor_error_raw_m_.load(std::memory_order_acquire));
    add_trace_value("anchor_error_time_aligned_m",
                    causal_anchor_error_time_aligned_m_.load(std::memory_order_acquire));
    add_trace_value("command_motion_over_state_age_m",
                    causal_command_motion_over_state_age_m_.load(std::memory_order_acquire));
    add_trace_value("velocity_residual_time_aligned_mps",
                    causal_velocity_residual_time_aligned_mps_.load(std::memory_order_acquire));
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
    add_trace_value("requested_cruise_speed_mps",
                    planner_diagnostics.requested_cruise_speed_mps);
    add_trace_value("effective_cruise_speed_mps",
                    planner_diagnostics.effective_cruise_speed_mps);
    add_trace_value("control_max_velocity_mps",
                    planner_diagnostics.control_max_velocity_mps);
    add_trace_value("control_max_acceleration_mps2",
                    planner_diagnostics.control_max_acceleration_mps2);
    add_trace_value("control_max_jerk_mps3",
                    planner_diagnostics.control_max_jerk_mps3);
    add_trace_value("physical_max_velocity_mps",
                    planner_diagnostics.physical_max_velocity_mps);
    add_trace_value("physical_max_acceleration_mps2",
                    planner_diagnostics.physical_max_acceleration_mps2);
    add_trace_value("physical_max_jerk_mps3",
                    planner_diagnostics.physical_max_jerk_mps3);
    add_trace_value("candidate_max_velocity_mps",
                    planner_diagnostics.candidate_maximum_velocity_mps);
    add_trace_value("candidate_max_acceleration_mps2",
                    planner_diagnostics.candidate_maximum_acceleration_mps2);
    add_trace_value("candidate_max_jerk_mps3",
                    planner_diagnostics.candidate_maximum_jerk_mps3);
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
    const auto episode_trace = execution_episode_.snapshot();
    add_trace_value("command_available", episode_trace.command_available ? 1 : 0);
    add_trace_value("planner_failure_latched", episode_trace.failure_latched ? 1 : 0);
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
        episode_trace.safety_suffix_active, committed_snapshot.terminal_stop);
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
  std::uint64_t localization_epoch_at_command =
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

  // The execution timeline owns the future splice. A successor is staged by
  // the planning worker, then atomically becomes the active command at its
  // producer-declared boundary before this callback samples the command. The
  // captured pointer is the finalizer's identity witness; it avoids loading a
  // second pointer while the store transaction is locked.
  const auto pending_timeline = command_bundle_store_.snapshot();
  const auto pending_bundle = pending_timeline.pending;
  if (pending_bundle && pending_timeline.pending_activation_ns <=
                           command_ros_time.nanoseconds()) {
    bool activated = false;
    {
      // The store operation is nested under the canonical runtime transition
      // locks. Its finalizer only observes this already-coherent state; it
      // must never acquire the locks in the reverse order.
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      activated = command_bundle_store_.activatePendingIfDueAndFinalize(
          command_ros_time.nanoseconds(), pending_timeline,
          [this, pending_bundle](const std::uint64_t generation) {
            if (pending_bundle->bundle_generation != generation ||
                !localization_epoch_ready_.load(std::memory_order_acquire) ||
                !command_execution_lease_failure_latch_.allowsCommandExposure()) {
              return false;
            }
            const auto recovery_state = execution_recovery_state_.load(
                std::memory_order_acquire);
            const auto episode = execution_episode_.snapshot();
            if (!executionRecoveryStateKnown(recovery_state) ||
                !nominalPlanningAllowed(recovery_state) ||
                episode.failure_latched || !active_goal_ ||
                active_goal_->request_id != pending_bundle->request_id ||
                active_goal_epoch_.load(std::memory_order_acquire) !=
                    pending_bundle->goal_epoch ||
                active_localization_epoch_.load(std::memory_order_acquire) !=
                    pending_bundle->localization_epoch) {
              return false;
            }
            // Ordinary successors have no mission/waypoint fields in the
            // immutable bundle. When this is a retained handoff, the pending
            // owner supplies the complete tuple and must agree with active_goal_.
            const auto pending_goal = pending_goal_owner_.goalSnapshot();
            if (pending_goal &&
                (pending_goal->mission_id != active_goal_->mission_id ||
                 pending_goal->waypoint_index != active_goal_->waypoint_index ||
                 pending_goal->request_id != pending_bundle->request_id)) {
              return false;
            }
            // State ownership is finalized after this store transaction, so a
            // fail-closed transition cannot be resurrected by this callback.
            return queueExecutionTimelineActivation(generation);
          });
      if (activated) {
        // The callback validated the exact pending pointer/generation and the
        // same identity/lease state while these locks remained held. Commit
        // the execution episode before releasing the transaction authority.
        command_goal_epoch_.store(
            pending_bundle->goal_epoch, std::memory_order_release);
        execution_episode_.commandCommitted(*pending_bundle);
        executing_goal_ = *active_goal_;
        hot_goal_transition_ = false;
        new_goal_ = false;
        // Activation is an ownership boundary: a completion witness from the
        // predecessor cannot be consumed as evidence for the successor.
        trajectory_completion_witness_.reset();
      }
    }
    if (activated) {
      RCLCPP_INFO(get_logger(),
                  "execution timeline activated successor generation=%lu at ns=%lld",
                  static_cast<unsigned long>(pending_bundle->bundle_generation),
                  static_cast<long long>(command_ros_time.nanoseconds()));
    }
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
        // Do not wait for the backend ownership mutex here. The watchdog is
        // the out-of-band deadline authority and must be able to cancel a job
        // even while the planner is inside an optimizer/certificate call.
        // Detailed stage/point diagnostics are sampled by the worker-owned
        // completion path, not on this real-time command callback.
        constexpr int timeout_stage = -1;
        constexpr std::size_t timeout_point_count = 0U;
        bool retain_safety_suffix = false;
        bool retain_stopped_recovery_hold = false;
        {
          std::lock_guard<std::mutex> command_lock(
              command_execution_lease_failure_latch_.transitionMutex());
          const auto recovery_state = execution_recovery_state_.load(
              std::memory_order_acquire);
          const auto episode = execution_episode_.snapshot();
          retain_safety_suffix = watchdogTimeoutMayRetainSafetySuffix(
              recovery_state,
              episode.command_available,
              episode.safety_suffix_active);
          retain_stopped_recovery_hold = watchdogTimeoutMayRetainStoppedRecoveryHold(
              recovery_state,
              episode.command_available,
              episode.restart_from_rest);
          if (retain_safety_suffix || retain_stopped_recovery_hold) {
            // The timed-out replacement is not allowed to revoke the already
            // certified suffix or bounded stopped-recovery hold. Keep its
            // ownership and let the stopped-recovery deadline govern the next
            // transition.
            if (retain_safety_suffix) {
              execution_episode_.setSafetySuffix(true);
            }
          } else {
            // No certified suffix is available for this solve. A watchdog
            // timeout is therefore a terminal handover, never a stale-command
            // continuation or an ambiguous safety state.
            pending_goal_owner_.clearGoal();
            command_goal_epoch_.store(0U, std::memory_order_release);
            failClosedLocked();
          }
        }
        RCLCPP_ERROR(get_logger(),
                     "planner backend planner watchdog timed out generation=%lu age=%.3f s stage=%d points=%zu; "
                     "preserving certified safety suffix=%d",
                     static_cast<unsigned long>(active_solve),
                     static_cast<double>(solve_age_ns) / 1e9,
                     timeout_stage,
                     timeout_point_count,
                     retain_safety_suffix ? 1 : 0);
        if (retain_stopped_recovery_hold) {
          RCLCPP_WARN(
              get_logger(),
              "planner watchdog retained bounded STOPPED_HOLD for measured-state retry "
              "generation=%lu",
              static_cast<unsigned long>(active_solve));
        }
      }
    }
  }

  Eigen::Matrix<double, 3, 4> pvaj = Eigen::Matrix<double, 3, 4>::Zero();
  double yaw = 0.0;
  double yaw_dot = 0.0;
  bool on_backup_traj = false;
  navigation_planning::CandidateRole sampled_role =
      navigation_planning::CandidateRole::kMain;
  bool traj_finish = false;
  std::uint64_t trajectory_generation = 0;
  double trajectory_time_s = 0.0;
  navigation_world_model::WorldSnapshotIdentity command_world_identity{};
  std::shared_ptr<const navigation_planning::CandidateBundle> sampled_bundle;
  auto episode = execution_episode_.snapshot();
  bool safety_suffix_active = episode.safety_suffix_active;
  bool planner_failed = episode.failure_latched;
  bool planner_available = episode.command_available;
  bool sampled_command_valid = false;
  bool sampled_planned_stop_hold = false;
  if (!planner_available && !planner_failed &&
      command_execution_lease_failure_latch_.allowsCommandExposure()) return;
  std::optional<navigation_contracts::msg::NavigationGoal> command_goal;
  std::optional<navigation_contracts::msg::NavigationGoal> executing_goal;
  std::uint64_t goal_epoch_at_command = 0U;
  std::uint64_t command_goal_epoch_at_command = 0U;
  std::shared_ptr<const navigation_execution::ExecutionStateLease> execution_state;
  std::int64_t execution_receive_steady_ns = 0;
  std::uint64_t execution_sequence = 0;
  execution_state = execution_state_store_.load();
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
    std::unique_lock<std::mutex> localization_lock(localization_transition_mutex_);
    std::unique_lock<std::mutex> input_lock(input_mutex_);
    std::unique_lock<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    last_command_transition_lock_wait_us_.store(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - transition_lock_wait_started).count(),
        std::memory_order_release);
    // These identities and episode facts form one coherent publication
    // decision. In particular, a same-goal successor may commit between
    // callbacks; do not combine its fresh episode with an epoch or executing
    // goal captured before the transition transaction.
    localization_epoch_at_command =
        active_localization_epoch_.load(std::memory_order_acquire);
    command_goal = active_goal_;
    executing_goal = executing_goal_.has_value() ? executing_goal_ : active_goal_;
    goal_epoch_at_command = active_goal_epoch_.load(std::memory_order_acquire);
    command_goal_epoch_at_command = command_goal_epoch_.load(std::memory_order_acquire);
    if (!command_goal) {
      return;
    }
    if (!localization_epoch_ready_.load(std::memory_order_acquire) ||
        active_localization_epoch_.load(std::memory_order_acquire) !=
            localization_epoch_at_command) {
      failClosedLocked();
      pending_goal_owner_.clearGoal();
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
      failClosedLocked();
      planner_failed = true;
      safety_suffix_active = false;
      if (first_failure) {
        command_lock.unlock();
        input_lock.unlock();
        localization_lock.unlock();
        if (planning_worker_) planning_worker_->cancelActive();
        RCLCPP_ERROR(
            get_logger(),
            "planner backend execution-state lease failed reason=%s source_age_ms=%.3f "
            "receive_age_ms=%.3f sequence=%lu active_solve=%lu; publishing terminal EMER",
            navigation_contracts::executionStateFreshnessReasonName(execution_freshness.reason),
            execution_freshness.source_age_ms, execution_freshness.receive_age_ms,
            static_cast<unsigned long>(execution_sequence),
            static_cast<unsigned long>(active_solve));
        localization_lock.lock();
        input_lock.lock();
        command_lock.lock();
        const bool still_current = command_goal && active_goal_ &&
            active_goal_->mission_id == command_goal->mission_id &&
            active_goal_->waypoint_index == command_goal->waypoint_index &&
            active_goal_->request_id == command_goal->request_id &&
            active_goal_epoch_.load(std::memory_order_acquire) == goal_epoch_at_command &&
            active_localization_epoch_.load(std::memory_order_acquire) ==
                localization_epoch_at_command;
        if (!still_current) return;
        failClosedLocked();
        pending_goal_owner_.clearGoal();
      }
    } else {
      // These three flags form one executable command decision. Reload them
      // only after acquiring the same serialization lock used by solve exposure.
      episode = execution_episode_.snapshot();
      planner_available = episode.command_available;
      planner_failed = episode.failure_latched;
      safety_suffix_active = episode.safety_suffix_active;
    }
  }
  if (planner_available && command_goal_epoch_at_command == 0U) {
    return;
  }
  if (planner_available) {
    const auto command_bundle_at_sample = command_bundle_store_.load();
    const auto sample = command_sampler_.sample(
        command_ros_time.nanoseconds(), command_goal_epoch_at_command);
    if (!sample) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "command sampler returned no point stamp_ns=%lld goal_epoch=%lu "
          "active_bundle=%d active_generation=%lu pending=%d pending_generation=%lu "
          "awaiting_activation=%d status=%d timeline_version=%lu",
          static_cast<long long>(command_ros_time.nanoseconds()),
          static_cast<unsigned long>(command_goal_epoch_at_command),
          command_bundle_at_sample ? 1 : 0,
          command_bundle_at_sample
              ? static_cast<unsigned long>(command_bundle_at_sample->bundle_generation)
              : 0UL,
          sample.bundle ? 1 : 0,
          sample.bundle ? static_cast<unsigned long>(sample.bundle->bundle_generation) : 0UL,
          sample.awaiting_activation ? 1 : 0,
          static_cast<int>(sample.status),
          static_cast<unsigned long>(sample.timeline_version));
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
        if (command_goal) {
          const navigation_execution::ExecutionTimelineSnapshot expected_timeline{
              sample.timeline_version, {}, command_bundle_at_sample, {}, 0};
          (void)clearCommandForCurrentIdentity(
              *command_goal, goal_epoch_at_command,
              localization_epoch_at_command, expected_timeline);
        }
        return;
      }
    } else {
      const auto point = *sample.point;
      bool planned_hold_valid = true;
      sampled_planned_stop_hold = sample.planned_stop_hold;
      if (sample.planned_stop_hold) {
        const auto latest_terminal_world = world_snapshot_store_.load();
        const bool endpoint_known_free = latest_terminal_world &&
            navigation_world_model::isCellTraversable(
                latest_terminal_world.view->classify(
                    point.position_world,
                    navigation_world_model::GridLayer::kInflated),
                navigation_world_model::UnknownPolicy::kRequireKnownFree);
        const bool endpoint_near_execution_state = execution_state &&
            execution_freshness.valid() &&
            (point.position_world - execution_state->state.position_world).norm() <=
            navigation_contracts::kCommandAnchorErrorLimitM;
        if (!endpoint_known_free || !endpoint_near_execution_state) {
          planned_hold_valid = false;
          if (command_goal) {
            const navigation_execution::ExecutionTimelineSnapshot expected_timeline{
                sample.timeline_version, {}, sample.bundle, {}, 0};
            (void)clearCommandForCurrentIdentity(
                *command_goal, goal_epoch_at_command,
                localization_epoch_at_command, expected_timeline);
          }
          planner_failed = true;
          safety_suffix_active = false;
          RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "execution boundary rejected planned STOPPED_HOLD endpoint "
              "known_free=%d near_execution=%d",
              endpoint_known_free ? 1 : 0, endpoint_near_execution_state ? 1 : 0);
        }
        pvaj.col(0) = point.position_world;
        pvaj.col(1).setZero();
        pvaj.col(2).setZero();
        pvaj.col(3).setZero();
      } else {
        pvaj.col(0) = point.position_world;
        pvaj.col(1) = point.velocity_world;
        pvaj.col(2) = point.acceleration_world;
        pvaj.col(3) = point.jerk_world;
      }
      yaw = point.yaw;
      yaw_dot = sample.planned_stop_hold ? 0.0 : point.yaw_rate;
      trajectory_generation = sample.bundle->bundle_generation;
      trajectory_time_s = point.trajectory_time_s;
      command_world_identity = sample.bundle->world_identity;
      sampled_bundle = sample.bundle;
      sampled_role = sample.planned_stop_hold
          ? navigation_planning::CandidateRole::kMain : point.role;
      on_backup_traj = sampled_role != navigation_planning::CandidateRole::kMain;
      if (sample.planned_stop_hold) {
        if (planned_hold_valid) {
          std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
          std::lock_guard<std::mutex> input_lock(input_mutex_);
          std::lock_guard<std::mutex> command_lock(
              command_execution_lease_failure_latch_.transitionMutex());
          if (command_goal && executingCommandIdentityMatchesLocked(
                  *command_goal, command_goal_epoch_at_command,
                  localization_epoch_at_command,
                  sample.bundle->bundle_generation)) {
            execution_episode_.stoppedHold(sample.bundle->bundle_generation);
          }
        } else {
          // clearCommandForCurrentIdentity() performs the serialized
          // fail-closed transition when this sampled bundle is still current.
          // A stale sample must not fail-close a newer execution identity.
        }
      } else {
        if (!queueExecutionTimelineActivation(sample.bundle->bundle_generation)) {
          planner_failed = true;
          sampled_command_valid = false;
          {
            std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
            std::lock_guard<std::mutex> input_lock(input_mutex_);
            std::lock_guard<std::mutex> command_lock(
                command_execution_lease_failure_latch_.transitionMutex());
            const auto current_bundle = command_bundle_store_.load();
            if (current_bundle && current_bundle.get() == sample.bundle.get() &&
                command_goal && executingCommandIdentityMatchesLocked(
                    *command_goal, command_goal_epoch_at_command,
                    localization_epoch_at_command,
                    sample.bundle->bundle_generation)) {
              command_bundle_store_.invalidate();
              command_goal_epoch_.store(0U, std::memory_order_release);
              failClosedLocked();
            }
          }
          if (planning_worker_) planning_worker_->cancelActive();
          RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "planner timeline activation synchronization failed generation=%lu",
              static_cast<unsigned long>(sample.bundle->bundle_generation));
          return;
        }
        // The immutable command was already recorded at its execution-store
        // commit/activation boundary. Do not let a stale sampler callback
        // rewrite episode identity outside the transition transaction.
      }
      // The sampled role is the execution-side source of truth for a
      // certified MAIN+BACKUP bundle. World-freshness suspension temporarily
      // clears the derived atomic flag while preserving the immutable bundle;
      // when the resumed sampler observes its actual BACKUP interval, restore
      // the suffix ownership before completion handling. Otherwise a finished
      // backup is misclassified as a nominal frontier and the runtime replans
      // the old waypoint behind the vehicle instead of promoting a pending
      // waypoint at the certified stop boundary.
      const bool sampled_emergency = !sample.planned_stop_hold &&
          point.role == navigation_planning::CandidateRole::kEmergency;
      const bool sampled_safety_suffix = !sample.planned_stop_hold &&
          point.role == navigation_planning::CandidateRole::kBackup;
      const auto sampled_event = sampled_emergency
          ? ExecutionRecoveryEvent::kEmergencyCommitted
          : ExecutionRecoveryEvent::kBackupActivated;
      const auto sampled_recovery_state =
          execution_recovery_state_.load(std::memory_order_acquire);
      const bool sampled_event_needed = (sampled_safety_suffix || sampled_emergency) &&
          transitionExecutionRecovery(sampled_recovery_state, sampled_event) !=
              sampled_recovery_state;
      if (sampled_event_needed) {
        std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
        std::lock_guard<std::mutex> input_lock(input_mutex_);
        std::lock_guard<std::mutex> command_lock(
            command_execution_lease_failure_latch_.transitionMutex());
        const auto current_bundle = command_bundle_store_.load();
        const bool exact_execution_identity = current_bundle && executing_goal &&
            current_bundle.get() == sample.bundle.get() &&
            executingCommandIdentityMatchesLocked(
                *executing_goal, command_goal_epoch_at_command,
                localization_epoch_at_command,
                sample.bundle->bundle_generation);
        if (exact_execution_identity) {
          if (sampled_safety_suffix) execution_episode_.setSafetySuffix(true);
          applyExecutionRecoveryEventLocked(sampled_event);
        }
      }
      traj_finish = sample.planned_stop_hold || point.finished;
      sampled_command_valid = planned_hold_valid;
      if (traj_finish) {
        const auto command_route_snapshot = command_goal
            ? decodeRouteSnapshot(*command_goal) : std::nullopt;
        const bool coincident_pass_through_stop = command_route_snapshot.has_value() &&
            navigation_mission::passThroughNextWaypointIsCoincidentStop(
                *command_route_snapshot);
        const bool effective_terminal_goal = command_goal &&
            (command_goal->behavior ==
                 navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP ||
             coincident_pass_through_stop);
        bool endpoint_reaches_goal = false;
        if (effective_terminal_goal) {
          const auto& target_message = plannerTarget(*command_goal);
          const Eigen::Vector3d target_position{
              pointFromMessage(target_message, 0),
              pointFromMessage(target_message, 1),
              pointFromMessage(target_message, 2)};
          endpoint_reaches_goal =
              (point.position_world - target_position).norm() <=
              goalCompletionTolerance(*command_goal);
        }
        const bool terminal_endpoint_contract =
            terminalStopEndpointContractValid(
                effective_terminal_goal, sample.bundle->kind,
                sample.bundle->role, point.role);
        const bool terminal_hold_committed = endpoint_reaches_goal &&
            terminal_endpoint_contract;
        terminal_bundle_generation_.store(
            terminal_hold_committed ? sample.bundle->bundle_generation : 0U,
            std::memory_order_release);
        if (terminal_hold_committed && planning_worker_) {
          // The terminal witness supersedes any solve that raced with the
          // final command sample. Cancellation is an interrupt only; the
          // worker still owns the planner transaction and its completion path
          // discards any stale staged candidate.
          planning_worker_->cancelActive();
        }
      }
    }
    // The safety suffix contains the dynamically continuous main prefix up to
    // planner backend's backup switch plus the braking polynomial. Once frozen by a
    // failed hot replan, the whole suffix is safety-owned.
    if (sampled_command_valid && safety_suffix_active &&
        sampled_role == navigation_planning::CandidateRole::kMain) {
      on_backup_traj = true;
    }
    if (traj_finish && sampled_command_valid && sampled_bundle) {
      // Completion is recorded only after the sampled immutable bundle is
      // still the active command for the current executing identity.
      // A stale endpoint must not trigger PlanFromRest for a replacement goal.
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const auto current_timeline = command_bundle_store_.snapshot();
      const auto current_bundle = current_timeline.active;
      const auto execution_episode = execution_episode_.snapshot();
      const bool command_exposure_allowed =
          command_execution_lease_failure_latch_.allowsCommandExposure();
      const bool exact_current_bundle = current_bundle &&
          current_bundle.get() == sampled_bundle.get() &&
          current_bundle->bundle_generation == sampled_bundle->bundle_generation &&
          current_bundle->localization_epoch ==
              active_localization_epoch_.load(std::memory_order_acquire);
      const bool exact_execution_identity = executing_goal_ &&
          executingCommandIdentityMatchesLocked(
              *executing_goal_, command_goal_epoch_at_command,
              localization_epoch_at_command, sampled_bundle->bundle_generation);
      if (!execution_episode.failure_latched && command_exposure_allowed &&
          exact_current_bundle && exact_execution_identity) {
        trajectory_completion_witness_ = TrajectoryCompletionWitness{
            sampled_bundle->bundle_generation,
            current_timeline.version,
            sampled_bundle->localization_epoch,
            sampled_bundle->goal_epoch,
            sampled_bundle->request_id,
            executing_goal_->mission_id,
            executing_goal_->waypoint_index};
      }
    }
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
  command.goal_epoch = command_goal_epoch_at_command;
  if (executing_goal) {
    command.mission_id = executing_goal->mission_id;
    command.waypoint_index = executing_goal->waypoint_index;
    command.request_id = executing_goal->request_id;
  }
  command.world_generation = command_world_identity.generation;
  command.world_revision = command_world_identity.revision;
  command.world_observation_stamp = navigation_common::nanosecondsToRosTime(
      command_world_identity.observation_stamp_ns).value_or(builtin_interfaces::msg::Time{});
  command.bundle_generation = trajectory_generation;
  const auto continuation_boundary = sampled_command_valid && sampled_bundle && executing_goal &&
      sampled_role == navigation_planning::CandidateRole::kMain &&
      !sampled_planned_stop_hold && !safety_suffix_active && !traj_finish
      ? certifiedMainContinuationBoundary(
            *sampled_bundle, *executing_goal, localization_epoch_at_command,
            command_goal_epoch_at_command, traj_finish)
      : std::nullopt;
  command.certified_main_continuation = continuation_boundary.has_value();
  command.continuation_boundary_stamp_ns = continuation_boundary.value_or(0U);
  const auto command_id = advanceMonotonicId(command_id_);
  if (!command_id) {
    std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
    std::lock_guard<std::mutex> input_lock(input_mutex_);
    std::lock_guard<std::mutex> command_lock(
        command_execution_lease_failure_latch_.transitionMutex());
    command_bundle_store_.invalidate();
    command_goal_epoch_.store(0U, std::memory_order_release);
    failClosedLocked();
    RCLCPP_ERROR(get_logger(), "command sample id exhausted");
    return;
  }
  command.sample_id = *command_id;
  command.trajectory_time_s = trajectory_time_s;
  command.analytic_sample_role = sampled_command_valid
      ? static_cast<std::uint8_t>(sampled_role)
      : navigation_contracts::msg::NavigationCommand::ANALYTIC_ROLE_UNKNOWN;
  command.backup_available = sampled_bundle && sampled_bundle->backup_available;
  command.backup_start_time_s = command.backup_available
      ? sampled_bundle->backup_start_time_s : 0.0;
  command.time_to_backup_start_s = command.backup_available
      ? sampled_bundle->backup_start_time_s - trajectory_time_s : 0.0;
  command.safety_suffix_active = safety_suffix_active;
  command.execution_recovery_state = static_cast<std::uint8_t>(
      execution_recovery_state_.load(std::memory_order_acquire));
  command.anchor_error_m = causal_anchor_error_m_.load(std::memory_order_acquire);
  command.projected_anchor_error_m =
      causal_projected_anchor_error_m_.load(std::memory_order_acquire);
  command.retained_tracking_limit_m =
      causal_retained_tracking_limit_m_.load(std::memory_order_acquire);
  command.relative_anchor_speed_mps =
      causal_relative_anchor_speed_mps_.load(std::memory_order_acquire);
  command.committed_suffix_usable =
      causal_committed_suffix_usable_.load(std::memory_order_acquire);
  command.sampled_path_clear = causal_sampled_path_clear_.load(std::memory_order_acquire);
  command.tracking_certificate_exceeded =
      causal_tracking_certificate_exceeded_.load(std::memory_order_acquire);
  command.projected_tracking_certificate_exceeded =
      causal_projected_tracking_certificate_exceeded_.load(std::memory_order_acquire);
  command.emergency_authorization_reason =
      causal_emergency_authorization_reason_.load(std::memory_order_acquire);
  command.causal_planning_cycle_id =
      causal_planning_cycle_id_.load(std::memory_order_acquire);
  command.causal_timestamp_ns = static_cast<std::uint64_t>(std::max<std::int64_t>(
      0, causal_timestamp_ns_.load(std::memory_order_acquire)));
  command.emergency_candidate_commit_result = static_cast<std::uint8_t>(std::clamp(
      causal_emergency_candidate_commit_result_.load(std::memory_order_acquire), 0, 255));
  command.evaluation_now_ns = static_cast<std::uint64_t>(std::max<std::int64_t>(
      0, causal_evaluation_now_ns_.load(std::memory_order_acquire)));
  command.execution_state_source_stamp_ns = static_cast<std::uint64_t>(std::max<std::int64_t>(
      0, causal_execution_state_source_stamp_ns_.load(std::memory_order_acquire)));
  command.execution_state_receive_stamp_ns = static_cast<std::uint64_t>(std::max<std::int64_t>(
      0, causal_execution_state_receive_stamp_ns_.load(std::memory_order_acquire)));
  command.execution_state_source_age_ms =
      causal_execution_state_source_age_ms_.load(std::memory_order_acquire);
  command.execution_state_receive_age_ms =
      causal_execution_state_receive_age_ms_.load(std::memory_order_acquire);
  command.committed_bundle_start_stamp_ns = static_cast<std::uint64_t>(std::max<std::int64_t>(
      0, causal_committed_bundle_start_stamp_ns_.load(std::memory_order_acquire)));
  command.measured_position_at_state_source.x =
      causal_measured_position_x_.load(std::memory_order_acquire);
  command.measured_position_at_state_source.y =
      causal_measured_position_y_.load(std::memory_order_acquire);
  command.measured_position_at_state_source.z =
      causal_measured_position_z_.load(std::memory_order_acquire);
  command.measured_velocity_at_state_source.x =
      causal_measured_velocity_x_.load(std::memory_order_acquire);
  command.measured_velocity_at_state_source.y =
      causal_measured_velocity_y_.load(std::memory_order_acquire);
  command.measured_velocity_at_state_source.z =
      causal_measured_velocity_z_.load(std::memory_order_acquire);
  command.committed_command_position_at_now.x =
      causal_command_now_position_x_.load(std::memory_order_acquire);
  command.committed_command_position_at_now.y =
      causal_command_now_position_y_.load(std::memory_order_acquire);
  command.committed_command_position_at_now.z =
      causal_command_now_position_z_.load(std::memory_order_acquire);
  command.committed_command_velocity_at_now.x =
      causal_command_now_velocity_x_.load(std::memory_order_acquire);
  command.committed_command_velocity_at_now.y =
      causal_command_now_velocity_y_.load(std::memory_order_acquire);
  command.committed_command_velocity_at_now.z =
      causal_command_now_velocity_z_.load(std::memory_order_acquire);
  command.committed_command_position_at_state_source.x =
      causal_command_source_position_x_.load(std::memory_order_acquire);
  command.committed_command_position_at_state_source.y =
      causal_command_source_position_y_.load(std::memory_order_acquire);
  command.committed_command_position_at_state_source.z =
      causal_command_source_position_z_.load(std::memory_order_acquire);
  command.committed_command_velocity_at_state_source.x =
      causal_command_source_velocity_x_.load(std::memory_order_acquire);
  command.committed_command_velocity_at_state_source.y =
      causal_command_source_velocity_y_.load(std::memory_order_acquire);
  command.committed_command_velocity_at_state_source.z =
      causal_command_source_velocity_z_.load(std::memory_order_acquire);
  command.anchor_error_raw_m = causal_anchor_error_raw_m_.load(std::memory_order_acquire);
  command.anchor_error_time_aligned_m =
      causal_anchor_error_time_aligned_m_.load(std::memory_order_acquire);
  command.command_motion_over_state_age_m =
      causal_command_motion_over_state_age_m_.load(std::memory_order_acquire);
  command.velocity_residual_time_aligned_mps =
      causal_velocity_residual_time_aligned_mps_.load(std::memory_order_acquire);
  command.retained_elapsed_s = causal_retained_elapsed_s_.load(std::memory_order_acquire);
  command.committed_bundle_duration_s =
      causal_committed_bundle_duration_s_.load(std::memory_order_acquire);
  command.committed_safety_transition_time_s =
      causal_committed_safety_transition_time_s_.load(std::memory_order_acquire);
  command.retained_validate_without_new_commit =
      causal_validate_without_new_commit_.load(std::memory_order_acquire);
  command.retained_fresh_vehicle_state =
      causal_retained_fresh_vehicle_state_.load(std::memory_order_acquire);
  command.retained_committed_command_available =
      causal_retained_committed_command_available_.load(std::memory_order_acquire);
  command.retained_command_anchor_valid =
      causal_retained_command_anchor_valid_.load(std::memory_order_acquire);
  command.current_vehicle_state_known_free =
      causal_current_vehicle_state_known_free_.load(std::memory_order_acquire);
  command.retained_safety_trajectory_available =
      causal_retained_safety_trajectory_available_.load(std::memory_order_acquire);
  command.retained_terminal_stop =
      causal_retained_terminal_stop_.load(std::memory_order_acquire);
  command.retained_committed_role = static_cast<std::int8_t>(std::clamp(
      causal_retained_committed_role_.load(std::memory_order_acquire), -128, 127));
  command.retained_recovery_state_before =
      causal_retained_recovery_state_before_.load(std::memory_order_acquire);
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
  const bool emergency_braking = sampled_command_valid &&
      sampled_role == navigation_planning::CandidateRole::kEmergency;
  // A validated STOPPED_HOLD is the explicit endpoint witness used while a
  // terminal backup settles. Planner failure must not relabel that exact hold
  // as REJECTED; doing so makes External Mode hand over before measured stop.
  // The hold can only be marked valid after the known-free and near-execution
  // checks above. No ordinary MAIN sample receives this exception.
  const bool main_trajectory_rejected = planner_failed && !on_backup_traj &&
      !emergency_braking && !(sampled_planned_stop_hold && sampled_command_valid);
  if (main_trajectory_rejected) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "publishing rejected command planner_failed=%d planner_available=%d "
        "sampled_valid=%d sampled_bundle=%d generation=%lu role=%d "
        "safety_suffix=%d execution_freshness=%d",
        planner_failed ? 1 : 0,
        planner_available ? 1 : 0,
        sampled_command_valid ? 1 : 0, sampled_bundle ? 1 : 0,
        static_cast<unsigned long>(trajectory_generation),
        static_cast<int>(sampled_role), safety_suffix_active ? 1 : 0,
        execution_freshness.valid() ? 1 : 0);
  }
  command.status = emergency_braking
                       ? navigation_contracts::msg::NavigationCommand::STATUS_BRAKING
                       : main_trajectory_rejected
                       ? navigation_contracts::msg::NavigationCommand::STATUS_REJECTED
                       : traj_finish
                       ? navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED
                       : navigation_contracts::msg::NavigationCommand::STATUS_READY;
  command.role = emergency_braking
                     ? navigation_contracts::msg::NavigationCommand::ROLE_EMERGENCY
                     : main_trajectory_rejected
                     ? navigation_contracts::msg::NavigationCommand::ROLE_EMERGENCY
                     : on_backup_traj
                     ? navigation_contracts::msg::NavigationCommand::ROLE_BACKUP
                     : navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
  command.reason_code = emergency_braking ? 2U : main_trajectory_rejected ? 1U : 0U;
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
    // Re-evaluate the execution lease at the publication boundary. The
    // callback-start state may have become stale while sampling and assembling
    // the message; a valid epoch alone is not proof that source/receive time
    // is still admissible.
    const auto final_execution_state = execution_state_store_.load();
    const auto final_publish_now_ns = now().nanoseconds();
    const auto final_execution_freshness = final_execution_state
        ? navigation_contracts::evaluateExecutionStateFreshness(
              final_publish_now_ns,
              final_execution_state->state.source_stamp_ns,
              navigation_common::steadyClockNowNanoseconds(),
              final_execution_state->state.receive_stamp_ns,
              data_freshness_window_s_)
        : navigation_contracts::ExecutionStateFreshness{};
    command_execution_lease_reason_.store(
        static_cast<int>(final_execution_freshness.reason), std::memory_order_release);
    command_execution_source_age_us_.store(static_cast<std::int64_t>(
        final_execution_freshness.source_age_ms * 1000.0), std::memory_order_release);
    command_execution_receive_age_us_.store(static_cast<std::int64_t>(
        final_execution_freshness.receive_age_ms * 1000.0), std::memory_order_release);
    {
      // Recheck the non-store execution state immediately before entering the
      // store's pointer/world transaction. Sampling and message assembly are
      // intentionally outside this lock. The final store transaction keeps
      // the canonical localization/input/transition locks while the exposure
      // callback runs so a lease or goal transition cannot complete between
      // validation and transport;
      // its duration is recorded as store_publish_us for latency monitoring.
      std::lock_guard<std::mutex> localization_lock(localization_transition_mutex_);
      std::lock_guard<std::mutex> input_lock(input_mutex_);
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const auto same_goal = [](const auto& lhs, const auto& rhs) {
        return lhs.has_value() && rhs.has_value() &&
               lhs->mission_id == rhs->mission_id &&
               lhs->waypoint_index == rhs->waypoint_index &&
               lhs->request_id == rhs->request_id;
      };
      const bool goal_identity_current = command_goal && active_goal_ &&
          same_goal(command_goal, active_goal_) &&
          active_goal_epoch_.load(std::memory_order_acquire) == goal_epoch_at_command &&
          active_localization_epoch_.load(std::memory_order_acquire) ==
              localization_epoch_at_command;
      const bool executing_identity_current = executing_goal && executing_goal_ &&
          same_goal(executing_goal, executing_goal_);
      if (localization_epoch_ready_.load(std::memory_order_acquire) &&
          active_localization_epoch_.load(std::memory_order_acquire) ==
              localization_epoch_at_command &&
          active_goal_epoch_.load(std::memory_order_acquire) == goal_epoch_at_command &&
          command_goal_epoch_.load(std::memory_order_acquire) ==
              command_goal_epoch_at_command &&
          goal_identity_current && executing_identity_current &&
          final_execution_state && final_execution_state->state.finite() &&
          final_execution_state->state.localization_epoch ==
              localization_epoch_at_command &&
          final_execution_freshness.valid() &&
          execution_episode_.snapshot().command_available &&
          command_execution_lease_failure_latch_.allowsCommandExposure()) {
        exposed = command_bundle_store_.publishIfCurrent(
            sampled_bundle, command_goal_epoch_at_command, publish_ros_command);
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
      std::lock_guard<std::mutex> command_lock(
          command_execution_lease_failure_latch_.transitionMutex());
      const bool superseded_by_valid_bundle = current_bundle &&
          supersedingBundleMayRemainAvailable(
              sampled_bundle ? sampled_bundle->bundle_generation : 0U,
              current_bundle->bundle_generation,
              current_bundle->localization_epoch,
              current_bundle->goal_epoch,
              current_localization_epoch,
              command_goal_epoch_at_command,
              current_bundle->valid_until_ns,
              command_ros_time.nanoseconds(),
              current_bundle->valid(),
              execution_episode_.snapshot().failure_latched,
              command_execution_lease_failure_latch_.allowsCommandExposure());
      if (!superseded_by_valid_bundle) {
        command_goal_epoch_.store(0U);
        failClosedLocked();
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
