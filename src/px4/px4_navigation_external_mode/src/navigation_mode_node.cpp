#include "px4_navigation_external_mode/navigation_mode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>

#include <px4_ros2/components/node_with_mode.hpp>

namespace px4_navigation_external_mode {
namespace {

constexpr char kModeName[] = "UAV Navigation";
constexpr char kTrajectoryFailureReason[] = "navigation trajectory unavailable or stale";

bool diagnosticValueIsTrue(
    const diagnostic_msgs::msg::DiagnosticStatus& status, std::string_view key) {
  for (const auto& value : status.values) {
    if (value.key == key) return value.value == "true";
  }
  return false;
}

std::string diagnosticValue(
    const diagnostic_msgs::msg::DiagnosticStatus& status, std::string_view key) {
  for (const auto& value : status.values) {
    if (value.key == key) return value.value;
  }
  return {};
}

NavigationMode::OutputMode resolveOutputMode(const std::string& output) {
  if (output == "velocity") return NavigationMode::OutputMode::Velocity;
  if (output == "position_velocity") return NavigationMode::OutputMode::PositionVelocity;
  // PX4's TrajectorySetpoint supports all three fields. `auto` therefore
  // resolves to the strongest contract available on this target.
  return NavigationMode::OutputMode::PositionVelocityAcceleration;
}

const char* outputModeName(NavigationMode::OutputMode mode) {
  switch (mode) {
    case NavigationMode::OutputMode::PositionVelocityAcceleration:
      return "position_velocity_acceleration";
    case NavigationMode::OutputMode::PositionVelocity:
      return "position_velocity";
    case NavigationMode::OutputMode::Velocity:
      return "velocity";
  }
  return "unknown";
}

}  // namespace

NavigationMode::NavigationMode(rclcpp::Node& node)
    : ModeBase(node, Settings{kModeName}),
      trajectory_setpoint_(std::make_shared<px4_ros2::TrajectorySetpointType>(*this)),
      trajectory_topic_(node.declare_parameter<std::string>(
          "navigation.trajectory_topic", "/navigation/trajectory")),
      goal_topic_(node.declare_parameter<std::string>(
          "navigation.goal_topic", "/navigation/goal")),
      planning_frame_(node.declare_parameter<std::string>(
          "navigation.planning_frame", "lio_odom")),
      stale_after_s_(node.declare_parameter<double>(
          "navigation.trajectory_stale_after_s", 0.75)),
      state_stale_after_s_(node.declare_parameter<double>(
          "navigation.state_stale_after_s", 0.5)),
      trajectory_wait_timeout_s_(node.declare_parameter<double>(
          "navigation.trajectory_wait_timeout_s", 2.0)),
      trajectory_preview_s_(node.declare_parameter<double>(
          "navigation.velocity_tracker.trajectory_preview_s", 0.15)),
      lio_health_grace_s_(node.declare_parameter<double>(
          "navigation.lio_health_grace_s", 1.0)),
      velocity_tracker_([&node]() {
        VelocityTrackerConfig config;
        config.position_gain = node.declare_parameter<double>(
            "navigation.velocity_tracker.position_gain", 1.0);
        config.max_velocity_mps = node.declare_parameter<double>(
            "navigation.velocity_tracker.max_velocity_mps", 2.0);
        config.max_acceleration_mps2 = node.declare_parameter<double>(
            "navigation.velocity_tracker.max_acceleration_mps2", 3.0);
        config.max_deceleration_mps2 = node.declare_parameter<double>(
            "navigation.velocity_tracker.max_deceleration_mps2", 3.0);
        config.max_position_error_m = node.declare_parameter<double>(
            "navigation.velocity_tracker.max_position_error_m", 2.0);
        return config;
      }()) {
  if (trajectory_topic_.empty() || planning_frame_.empty() || !std::isfinite(stale_after_s_) ||
      stale_after_s_ <= 0.0 || !std::isfinite(state_stale_after_s_) || state_stale_after_s_ <= 0.0 ||
      !std::isfinite(trajectory_wait_timeout_s_) || trajectory_wait_timeout_s_ <= 0.0 ||
      !std::isfinite(lio_health_grace_s_) || lio_health_grace_s_ < 0.0 ||
      !std::isfinite(trajectory_preview_s_) || trajectory_preview_s_ < 0.0) {
    throw std::invalid_argument("invalid PX4 navigation external mode parameters");
  }
  trajectory_subscription_ = node.create_subscription<
      navigation_interfaces::msg::PlannedTrajectory>(
      trajectory_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const navigation_interfaces::msg::PlannedTrajectory::ConstSharedPtr& message) {
        onTrajectory(message);
      });
  const auto state_topic = node.declare_parameter<std::string>(
      "navigation.state_topic", "/lio/odometry_propagated");
  if (state_topic.empty()) {
    throw std::invalid_argument("navigation.state_topic must not be empty");
  }
  odometry_subscription_ = node.create_subscription<nav_msgs::msg::Odometry>(
      // Propagated odometry is published reliably by FAST-LIO. This state is
      // a hard input to the velocity tracker, so do not downgrade it to
      // best-effort and allow a transient DDS drop to look like stale state.
      // The velocity tracker consumes the newest state. A one-sample queue
      // avoids applying a burst of stale odometry after DDS/executor jitter.
      state_topic, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr& message) { onOdometry(message); });
  lio_diagnostics_subscription_ = node.create_subscription<
      diagnostic_msgs::msg::DiagnosticArray>(
      "/lio/diagnostics", rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort(),
      [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& message) {
        onLioDiagnostics(message);
      });
  planner_heartbeat_subscription_ = node.create_subscription<std_msgs::msg::Empty>(
      "/navigation/planner_heartbeat", rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const std_msgs::msg::Empty::ConstSharedPtr& message) {
        onPlannerHeartbeat(message);
      });

  const auto mission_file = node.declare_parameter<std::string>("navigation.mission_file", "");
  if (!mission_file.empty()) {
    if (goal_topic_.empty()) {
      throw std::invalid_argument("navigation.goal_topic must not be empty for a mission");
    }
    mission_ = loadMission(mission_file, planning_frame_);
    output_mode_ = resolveOutputMode(mission_->control.output);
    RCLCPP_INFO(node.get_logger(), "External Mode output '%s' resolved to '%s'",
                mission_->control.output.c_str(), outputModeName(output_mode_));
    mission_controller_ = std::make_unique<MissionController>(*mission_);
    goal_publisher_ = node.create_publisher<navigation_interfaces::msg::NavigationGoal>(
        goal_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable());
    const auto status_topic = node.declare_parameter<std::string>(
        "navigation.status_topic", "/navigation/mode_status");
    if (status_topic.empty()) {
      throw std::invalid_argument("navigation.status_topic must not be empty for a mission");
    }
    status_publisher_ = node.create_publisher<navigation_interfaces::msg::NavigationModeStatus>(
        status_topic, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local());
    const auto mission_complete_topic = node.declare_parameter<std::string>(
        "navigation.mission_complete_topic", "/navigation/mission_complete");
    if (mission_complete_topic.empty()) {
      throw std::invalid_argument("navigation.mission_complete_topic must not be empty for a mission");
    }
    mission_complete_publisher_ = node.create_publisher<std_msgs::msg::Bool>(
        mission_complete_topic, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable());
    mission_timer_ = node.create_wall_timer(std::chrono::milliseconds{50},
                                            [this]() { updateMission(); });
  }
  setSetpointUpdateRate(50.0F);
}

void NavigationMode::setPositionControlHandover(std::function<void()> callback) {
  position_control_handover_ = std::move(callback);
}

void NavigationMode::publishStatus(std::uint8_t state, std::uint8_t reason) {
  if (!status_publisher_ || !mission_ || !mission_controller_) return;
  navigation_interfaces::msg::NavigationModeStatus status;
  status.header.stamp = node().get_clock()->now();
  status.header.frame_id = planning_frame_;
  status.mission_id = mission_->id;
  status.waypoint_index = static_cast<std::uint32_t>(mission_controller_->activeWaypointIndex());
  status.request_id = mission_controller_->activeRequestId();
  status.state = state;
  status.reason = reason;
  status_publisher_->publish(status);
  last_status_state_ = state;
}

void NavigationMode::onActivate() {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    activation_time_ = node().get_clock()->now();
    last_setpoint_time_ = activation_time_;
    velocity_tracker_.reset();
    pending_trajectory_.reset();
    if (trajectory_.has_value()) {
      trajectory_start_time_ = activation_time_;
    }
    failure_reported_ = false;
    mission_complete_published_ = false;
    // Estimator and odometry freshness are process-level observations, not
    // per-activation state.  Clearing them here makes PX4 see an artificial
    // health gap in the first few hundred milliseconds after a mode
    // re-entry, which immediately activates the external-mode failsafe.
    // Keep the last samples and let the normal freshness checks decide
    // whether they are still usable.
    accepted_world_identity_valid_ = false;
    accepted_world_generation_ = 0U;
    accepted_world_revision_ = 0U;
    odometry_callback_count_ = 0U;
    trajectory_received_count_ = 0U;
    trajectory_accepted_count_ = 0U;
    trajectory_rejected_count_ = 0U;
    setpoint_update_count_ = 0U;
    stale_state_failure_count_ = 0U;
    last_trajectory_receive_ns_ = 0;
    maximum_odometry_callback_gap_us_ = 0;
    last_setpoint_update_ns_ = 0;
    maximum_setpoint_callback_gap_us_ = 0;
    last_metrics_log_ns_ = 0;
    last_state_age_s_ = -1.0;
    mode_active_ = true;
    mission_terminal_ = false;
    handover_requested_ = false;
    completion_position_.reset();
  }
  if (mission_controller_) {
    if (mission_complete_publisher_) {
      std_msgs::msg::Bool status;
      status.data = false;
      mission_complete_publisher_->publish(status);
    }
    mission_controller_->activate(node().get_clock()->now().seconds());
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::ACTIVE,
                  navigation_interfaces::msg::NavigationModeStatus::NONE);
    updateMission();
  }
}

void NavigationMode::onDeactivate() {
  mode_active_ = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    // Deactivation is a terminal boundary for the current mode activation.
    // Keep rejecting late trajectories until onActivate() explicitly starts
    // a new generation; otherwise the runtime can repopulate the cached
    // trajectory while the PX4 mode executor is already handing over.
    failure_reported_ = true;
    trajectory_.reset();
    pending_trajectory_.reset();
    velocity_tracker_.reset();
    accepted_world_identity_valid_ = false;
    accepted_world_generation_ = 0U;
    accepted_world_revision_ = 0U;
  }
  if (mission_controller_) mission_controller_->deactivate();
  if (last_status_state_ != navigation_interfaces::msg::NavigationModeStatus::PAUSED &&
      last_status_state_ != navigation_interfaces::msg::NavigationModeStatus::COMPLETE &&
      last_status_state_ != navigation_interfaces::msg::NavigationModeStatus::FAILED) {
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::PAUSED,
                  navigation_interfaces::msg::NavigationModeStatus::OPERATOR_TAKEOVER);
  }
}

void NavigationMode::checkArmingAndRunConditions(
    px4_ros2::HealthAndArmingCheckReporter& reporter) {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  // The first goal is intentionally published from onActivate(), so a
  // pre-activation trajectory is not an arming requirement. Once active,
  // freshness and estimator/map heartbeats are explicit run conditions.
  if (!mode_active_ || mission_terminal_ || handover_requested_) return;
  const auto now_ns = node().get_clock()->now().nanoseconds();
  const auto stale = [&](std::int64_t stamp_ns, double limit_s) {
    return stamp_ns <= 0 || now_ns < stamp_ns ||
           static_cast<double>(now_ns - stamp_ns) / 1e9 > limit_s;
  };
  if (stale(last_odometry_receive_ns_, state_stale_after_s_)) {
    reporter.armingCheckFailureExt(
        px4_ros2::events::ID("uav_navigation_odometry_stale"),
        px4_ros2::events::Log::Error, "Navigation odometry is stale");
  }
  const bool diagnostics_stale = stale(last_lio_diagnostics_ns_, state_stale_after_s_);
  const bool grace_active = lio_unhealthy_since_ns_ > 0 &&
                            static_cast<double>(now_ns - lio_unhealthy_since_ns_) / 1e9 <=
                                lio_health_grace_s_;
  if (diagnostics_stale || (!lio_health_valid_ && !grace_active)) {
    reporter.armingCheckFailureExt(
        px4_ros2::events::ID("uav_navigation_lio_unhealthy"),
        px4_ros2::events::Log::Error, "FAST-LIO health is stale or invalid");
  }
  const bool waiting_for_airborne = mission_controller_ &&
                                    mission_controller_->waitingForAirborne();
  // During the disarmed warm-up activation the mission deliberately has no
  // goal yet, so the planner has nothing to heartbeat.  Requiring a planner
  // heartbeat here makes PX4 fail the warm-up before the arm/takeoff cycle can
  // complete. Once airborne, the normal heartbeat gate is active.
  if (mission_ && !waiting_for_airborne &&
      (last_planner_heartbeat_ns_ <= 0 ||
       stale(last_planner_heartbeat_ns_, trajectory_wait_timeout_s_))) {
    const double active_s = activation_time_.nanoseconds() > 0
                                ? static_cast<double>(now_ns - activation_time_.nanoseconds()) / 1e9
                                : 0.0;
    if (active_s > trajectory_wait_timeout_s_) {
      reporter.armingCheckFailureExt(
          px4_ros2::events::ID("uav_navigation_planner_heartbeat_stale"),
          px4_ros2::events::Log::Error, "Navigation planner heartbeat is stale");
    }
  }
}

void NavigationMode::onTrajectory(
    const navigation_interfaces::msg::PlannedTrajectory::ConstSharedPtr& message) {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    ++trajectory_received_count_;
    last_trajectory_receive_ns_ = node().get_clock()->now().nanoseconds();
    if (failure_reported_ || mission_terminal_ || handover_requested_) return;
  }
  if (mission_controller_ &&
      !trajectoryMatchesGoal(*message, mission_->id,
                             static_cast<std::uint32_t>(mission_controller_->activeWaypointIndex()),
                             mission_controller_->activeRequestId())) {
    RCLCPP_DEBUG(node().get_logger(), "Ignoring trajectory for an older mission request");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!trajectoryRevisionIsNotOlder(*message, accepted_world_identity_valid_,
                                      accepted_world_generation_, accepted_world_revision_)) {
      RCLCPP_DEBUG(node().get_logger(), "Ignoring trajectory from an older map revision");
      return;
    }
    if (message->success) {
      const auto newest_id = std::max(
          trajectory_.has_value() ? trajectory_->trajectory_id : 0U,
          pending_trajectory_.has_value() ? pending_trajectory_->trajectory_id : 0U);
      if (message->trajectory_id <= newest_id) {
        RCLCPP_DEBUG(node().get_logger(),
                     "Ignoring trajectory id=%lu because id=%lu is already active/pending",
                     static_cast<unsigned long>(message->trajectory_id),
                     static_cast<unsigned long>(newest_id));
        return;
      }
    }
  }
  if (!message->success) {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      ++trajectory_rejected_count_;
      trajectory_.reset();
      pending_trajectory_.reset();
      velocity_tracker_.reset();
      accepted_world_identity_valid_ = false;
      accepted_world_generation_ = message->world_generation;
      accepted_world_revision_ = message->world_revision;
    }
    if (mission_controller_) {
      mission_controller_->onTrajectory(false, message->trajectory_role,
                                        message->safety_plan_kind,
                                        node().get_clock()->now().seconds());
    }
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::PAUSED,
                  navigation_interfaces::msg::NavigationModeStatus::TRAJECTORY_INVALID);
    return;
  }
  const auto validation = validateTrajectory(*message, planning_frame_);
  if (!validation.valid()) {
    RCLCPP_WARN(node().get_logger(), "Rejecting navigation trajectory: %s",
                validation.message.c_str());
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      ++trajectory_rejected_count_;
      trajectory_.reset();
      pending_trajectory_.reset();
      velocity_tracker_.reset();
    }
    if (mission_controller_) {
      mission_controller_->onTrajectory(false, node().get_clock()->now().seconds());
    }
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::PAUSED,
                  navigation_interfaces::msg::NavigationModeStatus::TRAJECTORY_INVALID);
    return;
  }

  const auto accepted_now = node().get_clock()->now();
  const rclcpp::Time valid_from(message->valid_from);
  if (valid_from.nanoseconds() > accepted_now.nanoseconds()) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    pending_trajectory_ = *message;
    return;
  }

  rclcpp::Time accepted_time;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory_ = *message;
    pending_trajectory_.reset();
    ++trajectory_accepted_count_;
    // Every replacement trajectory is generated from the latest state. Its
    // own header timestamp is the only phase source; inheriting elapsed phase
    // from a previous trajectory can put a braking stop metres ahead of the
    // vehicle and repeatedly restart the deceleration envelope.
    double phase_s = 0.0;
    const rclcpp::Time message_start(message->valid_from);
    if (message_start.nanoseconds() > 0) {
      trajectory_start_time_ = message_start;
      phase_s = 0.0;
    } else {
      const rclcpp::Time header_start(message->header.stamp);
      if (header_start.nanoseconds() > 0) {
        phase_s = std::max(0.0, (accepted_now - header_start).seconds());
      }
      phase_s = std::min(phase_s, std::max(0.0, message->duration_s - 0.05));
      trajectory_start_time_ = accepted_now - rclcpp::Duration::from_seconds(phase_s);
    }
    accepted_time = accepted_now;
    failure_reported_ = false;
    if (message->success) {
      accepted_world_identity_valid_ = true;
      accepted_world_generation_ = message->world_generation;
      accepted_world_revision_ = message->world_revision;
    }
  }
  if (mission_controller_) {
    mission_controller_->onTrajectory(message->success, message->trajectory_role,
                                      message->safety_plan_kind, accepted_time.seconds(),
                                      message->duration_s);
  }
  if (message->trajectory_role == navigation_interfaces::msg::PlannedTrajectory::ROLE_SAFETY &&
      message->safety_plan_kind ==
          navigation_interfaces::msg::PlannedTrajectory::SAFETY_KIND_BRAKING_STOP) {
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::BRAKING,
                  navigation_interfaces::msg::NavigationModeStatus::SAFETY_STOP);
  } else {
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::ACTIVE,
                  navigation_interfaces::msg::NavigationModeStatus::NONE);
  }
  RCLCPP_DEBUG(node().get_logger(), "Accepted trajectory generation=%lu revision=%lu duration=%.3f s",
               static_cast<unsigned long>(message->world_generation),
               static_cast<unsigned long>(message->world_revision), message->duration_s);
}

void NavigationMode::updateMission() {
  if (!mission_controller_ || !goal_publisher_) return;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (failure_reported_ || mission_terminal_ || handover_requested_) return;
  }
  std::optional<Eigen::Vector3d> position;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (odometry_.has_value()) {
      const auto& point = odometry_->pose.pose.position;
      position = Eigen::Vector3d{point.x, point.y, point.z};
    }
  }
  const double now_s = node().get_clock()->now().seconds();
  std::optional<Eigen::Vector3d> velocity;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (odometry_.has_value()) {
      const auto& twist = odometry_->twist.twist.linear;
      velocity = Eigen::Vector3d{twist.x, twist.y, twist.z};
    }
  }
  const bool airborne = isArmed() && position.has_value() && position->z() > 0.5;
  handleMissionEvent(mission_controller_->update(now_s, position, airborne, velocity), now_s);
}

void NavigationMode::handleMissionEvent(const MissionControllerEvent& event, double now_s) {
  if (!mission_controller_ || event.type == MissionControllerEvent::Type::None) return;
  if (event.type == MissionControllerEvent::Type::PublishGoal) {
    const auto& waypoint = mission_controller_->activeWaypoint();
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      trajectory_.reset();
      pending_trajectory_.reset();
      velocity_tracker_.reset();
      accepted_world_identity_valid_ = false;
      accepted_world_generation_ = 0U;
      accepted_world_revision_ = 0U;
    }
    navigation_interfaces::msg::NavigationGoal goal;
    goal.header.frame_id = planning_frame_;
    const auto time = node().get_clock()->now();
    goal.header.stamp = time;
    goal.mission_id = mission_->id;
    goal.waypoint_index = static_cast<std::uint32_t>(event.waypoint_index);
    goal.request_id = event.request_id;
    goal.target.x = waypoint.position_enu.x();
    goal.target.y = waypoint.position_enu.y();
    goal.target.z = waypoint.position_enu.z();
    goal.acceptance_radius_m = waypoint.acceptance_radius_m;
    goal.behavior = waypoint.behavior == MissionWaypoint::Behavior::Stop
                        ? navigation_interfaces::msg::NavigationGoal::BEHAVIOR_STOP
                        : navigation_interfaces::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH;
    const auto next_waypoint = mission_controller_->nextWaypoint();
    if (next_waypoint.has_value()) {
      goal.has_next_target = true;
      goal.next_target.x = next_waypoint->position_enu.x();
      goal.next_target.y = next_waypoint->position_enu.y();
      goal.next_target.z = next_waypoint->position_enu.z();
    }
    goal_publisher_->publish(goal);
    RCLCPP_DEBUG(node().get_logger(), "Published mission waypoint %zu (%s)",
                 event.waypoint_index, waypoint.id.c_str());
    return;
  }
  if (event.type == MissionControllerEvent::Type::Complete) {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      mission_terminal_ = true;
      handover_requested_ = true;
      completion_position_ = mission_->waypoints.at(event.waypoint_index).position_enu;
      trajectory_.reset();
      pending_trajectory_.reset();
      velocity_tracker_.reset();
    }
    RCLCPP_INFO(node().get_logger(), "Mission '%s' completed; notifying the supervisor",
                mission_->id.c_str());
    if (mission_complete_publisher_ && !mission_complete_published_) {
      std_msgs::msg::Bool status;
      status.data = true;
      mission_complete_publisher_->publish(status);
      mission_complete_published_ = true;
    }
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::COMPLETE,
                  navigation_interfaces::msg::NavigationModeStatus::NONE);
    completed(px4_ros2::Result::Success);
    return;
  }
  if (event.type == MissionControllerEvent::Type::RequestPositionControl) {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      handover_requested_ = true;
    }
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::PAUSED,
                  navigation_interfaces::msg::NavigationModeStatus::SAFETY_STOP);
    RCLCPP_WARN(node().get_logger(),
                "Safety stop completed at waypoint %zu; handing over to PX4 POSCTL",
                event.waypoint_index);
    if (position_control_handover_) {
      position_control_handover_();
    } else {
      failNavigation("position-control handover callback is unavailable");
    }
    return;
  }
  if (event.type == MissionControllerEvent::Type::Failure) {
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::FAILED,
                  navigation_interfaces::msg::NavigationModeStatus::TRAJECTORY_INVALID);
    RCLCPP_ERROR(node().get_logger(), "Mission '%s' failed at waypoint %zu",
                 mission_->id.c_str(), event.waypoint_index);
    (void)now_s;
    failNavigation("mission controller reported failure");
  }
}

void NavigationMode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& message) {
  const auto& position = message->pose.pose.position;
  const auto& velocity = message->twist.twist.linear;
  if (message->header.frame_id != planning_frame_ || message->header.stamp.sec < 0 ||
      !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z)) {
    RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000,
                         "Rejecting navigation odometry with invalid frame or values");
    return;
  }
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  const auto receive_ns = node().get_clock()->now().nanoseconds();
  if (last_odometry_receive_ns_ > 0 && receive_ns >= last_odometry_receive_ns_) {
    maximum_odometry_callback_gap_us_ = std::max(
        maximum_odometry_callback_gap_us_,
        (receive_ns - last_odometry_receive_ns_) / 1000);
  }
  last_odometry_receive_ns_ = receive_ns;
  ++odometry_callback_count_;
  odometry_ = *message;
}

void NavigationMode::onLioDiagnostics(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& message) {
  const auto now_ns = node().get_clock()->now().nanoseconds();
  for (const auto& status : message->status) {
    if (status.name != "fast_lio/estimator") continue;
    const bool tracking = diagnosticValue(status, "status") == "TRACKING";
    const bool healthy = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
                         tracking && diagnosticValueIsTrue(status, "navigation_valid") &&
                         diagnosticValueIsTrue(status, "corrected_estimate_valid") &&
                         diagnosticValueIsTrue(status, "translation_observability_valid");
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    lio_health_valid_ = healthy;
    last_lio_diagnostics_ns_ = now_ns;
    if (healthy) {
      lio_unhealthy_since_ns_ = 0;
    } else if (lio_unhealthy_since_ns_ == 0) {
      lio_unhealthy_since_ns_ = now_ns;
    }
    return;
  }
}

void NavigationMode::onPlannerHeartbeat(
    const std_msgs::msg::Empty::ConstSharedPtr& /*message*/) {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  last_planner_heartbeat_ns_ = node().get_clock()->now().nanoseconds();
}

void NavigationMode::logRuntimeMetrics(const rclcpp::Time& now) {
  const auto now_ns = now.nanoseconds();
  std::uint64_t odometry_callbacks;
  std::uint64_t trajectories_received;
  std::uint64_t trajectories_accepted;
  std::uint64_t trajectories_rejected;
  std::uint64_t setpoint_updates;
  std::uint64_t stale_state_failures;
  std::int64_t odometry_gap_us;
  std::int64_t setpoint_gap_us;
  double state_age_s;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (last_metrics_log_ns_ > 0 && now_ns - last_metrics_log_ns_ < 1'000'000'000LL) {
      return;
    }
    last_metrics_log_ns_ = now_ns;
    odometry_callbacks = odometry_callback_count_;
    trajectories_received = trajectory_received_count_;
    trajectories_accepted = trajectory_accepted_count_;
    trajectories_rejected = trajectory_rejected_count_;
    setpoint_updates = setpoint_update_count_;
    stale_state_failures = stale_state_failure_count_;
    odometry_gap_us = maximum_odometry_callback_gap_us_;
    setpoint_gap_us = maximum_setpoint_callback_gap_us_;
    state_age_s = last_state_age_s_;
  }
  RCLCPP_INFO(node().get_logger(),
              "external_mode_metrics odom_callbacks=%lu odom_max_gap_us=%ld "
              "trajectory_received=%lu trajectory_accepted=%lu trajectory_rejected=%lu "
              "setpoint_updates=%lu setpoint_max_gap_us=%ld last_state_age_s=%.6f "
              "stale_state_failures=%lu",
              static_cast<unsigned long>(odometry_callbacks),
              static_cast<long>(odometry_gap_us),
              static_cast<unsigned long>(trajectories_received),
              static_cast<unsigned long>(trajectories_accepted),
              static_cast<unsigned long>(trajectories_rejected),
              static_cast<unsigned long>(setpoint_updates),
              static_cast<long>(setpoint_gap_us), state_age_s,
              static_cast<unsigned long>(stale_state_failures));
}

void NavigationMode::failNavigation(const char* reason) {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (failure_reported_) return;
    failure_reported_ = true;
    handover_requested_ = true;
    trajectory_.reset();
    pending_trajectory_.reset();
    velocity_tracker_.reset();
    accepted_world_identity_valid_ = false;
    accepted_world_generation_ = 0U;
    accepted_world_revision_ = 0U;
  }
  if (mission_controller_) mission_controller_->deactivate();
  const auto status_reason = std::string_view(reason).find("odometry") != std::string_view::npos
                                 ? navigation_interfaces::msg::NavigationModeStatus::ODOMETRY_STALE
                                 : navigation_interfaces::msg::NavigationModeStatus::TRAJECTORY_INVALID;
  publishStatus(navigation_interfaces::msg::NavigationModeStatus::FAILED, status_reason);
  RCLCPP_ERROR(node().get_logger(), "%s; handing over to PX4 POSCTL", reason);
  if (position_control_handover_) {
    position_control_handover_();
  } else {
    completed(px4_ros2::Result::ModeFailureOther);
  }
}

void NavigationMode::updateSetpoint(float /*dt_s*/) {
  std::optional<navigation_interfaces::msg::PlannedTrajectory> trajectory;
  std::optional<nav_msgs::msg::Odometry> odometry;
  rclcpp::Time start_time;
  rclcpp::Time previous_setpoint_time;
  std::optional<navigation_interfaces::msg::PlannedTrajectory> promoted_trajectory;
  const auto now = node().get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (pending_trajectory_.has_value() &&
        rclcpp::Time(pending_trajectory_->valid_from).nanoseconds() <= now.nanoseconds()) {
      trajectory_ = *pending_trajectory_;
      trajectory_start_time_ = rclcpp::Time(pending_trajectory_->valid_from);
      promoted_trajectory = pending_trajectory_;
      pending_trajectory_.reset();
      ++trajectory_accepted_count_;
      accepted_world_identity_valid_ = true;
      accepted_world_generation_ = trajectory_->world_generation;
      accepted_world_revision_ = trajectory_->world_revision;
    }
    trajectory = trajectory_;
    odometry = odometry_;
    start_time = trajectory_start_time_;
    previous_setpoint_time = last_setpoint_time_;
  }

  if (promoted_trajectory.has_value() && mission_controller_) {
    mission_controller_->onTrajectory(true, promoted_trajectory->trajectory_role,
                                       promoted_trajectory->safety_plan_kind,
                                       now.seconds(), promoted_trajectory->duration_s);
  }
  const auto publishStationary = [&](const std::optional<Eigen::Vector3d>& position_enu) {
    px4_ros2::TrajectorySetpoint setpoint;
    if (output_mode_ == OutputMode::Velocity || !position_enu.has_value()) {
      setpoint.withVelocity(Eigen::Vector3f::Zero());
    } else {
      setpoint.withPosition(enuToNed(*position_enu)).withVelocity(Eigen::Vector3f::Zero());
      if (output_mode_ == OutputMode::PositionVelocityAcceleration) {
        setpoint.withAcceleration(Eigen::Vector3f::Zero());
      }
    }
    trajectory_setpoint_->update(setpoint);
  };
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const auto now_ns = now.nanoseconds();
    if (last_setpoint_update_ns_ > 0 && now_ns >= last_setpoint_update_ns_) {
      maximum_setpoint_callback_gap_us_ = std::max(
          maximum_setpoint_callback_gap_us_, (now_ns - last_setpoint_update_ns_) / 1000);
    }
    last_setpoint_update_ns_ = now_ns;
    ++setpoint_update_count_;
  }
  logRuntimeMetrics(now);
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (failure_reported_) {
      // Keep the PX4 setpoint stream valid and stationary while the mode
      // executor performs the handover after a terminal navigation failure.
      publishStationary(std::nullopt);
      last_setpoint_time_ = now;
      return;
    }
    if (mission_terminal_) {
      publishStationary(completion_position_);
      last_setpoint_time_ = now;
      return;
    }
    if (handover_requested_) {
      std::optional<Eigen::Vector3d> handover_position;
      if (mission_controller_) {
        handover_position = mission_controller_->activeWaypoint().position_enu;
      }
      publishStationary(handover_position);
      last_setpoint_time_ = now;
      return;
    }
  }
  const double since_activation_s = (now - activation_time_).seconds();
  if (mission_controller_ && mission_controller_->waitingForAirborne()) {
    publishStationary(std::nullopt);
    return;
  }
  if (mission_controller_ && mission_controller_->holding()) {
    const auto waypoint = mission_controller_->activeWaypoint();
    publishStationary(waypoint.position_enu);
    return;
  }

  bool lio_healthy = false;
  std::int64_t lio_diagnostics_age_ns = std::numeric_limits<std::int64_t>::max();
  std::int64_t lio_unhealthy_since_ns = 0;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    lio_healthy = lio_health_valid_;
    lio_unhealthy_since_ns = lio_unhealthy_since_ns_;
    if (last_lio_diagnostics_ns_ > 0 && now.nanoseconds() >= last_lio_diagnostics_ns_) {
      lio_diagnostics_age_ns = now.nanoseconds() - last_lio_diagnostics_ns_;
    }
  }
  // The first diagnostics sample can legitimately be in flight when PX4
  // hands control to the mode. Hold zero velocity for a short bounded
  // acquisition window; once a sample exists, any unhealthy value is an
  // immediate fail-closed event and a stale healthy sample uses the normal
  // finite health timeout.
  const bool diagnostics_missing = lio_diagnostics_age_ns == std::numeric_limits<std::int64_t>::max();
  const double diagnostics_wait_s = std::min(0.5, std::max(0.0, trajectory_wait_timeout_s_));
  if (diagnostics_missing && since_activation_s <= diagnostics_wait_s) {
    publishStationary(std::nullopt);
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    last_setpoint_time_ = now;
    return;
  }
  const bool lio_health_grace_active =
      !diagnostics_missing && !lio_healthy && lio_unhealthy_since_ns > 0 &&
      now.nanoseconds() >= lio_unhealthy_since_ns &&
      static_cast<double>(now.nanoseconds() - lio_unhealthy_since_ns) / 1e9 <=
          lio_health_grace_s_;
  if ((!diagnostics_missing && !lio_healthy && !lio_health_grace_active) ||
      lio_diagnostics_age_ns > static_cast<std::int64_t>(state_stale_after_s_ * 1e9) ||
      (diagnostics_missing && since_activation_s > diagnostics_wait_s)) {
    failNavigation("FAST-LIO navigation health invalid or stale");
    return;
  }

  if (!trajectory.has_value()) {
    if (mission_controller_) {
      const auto mission_state = mission_controller_->state();
      if (mission_state == MissionControllerState::WaitingForAirborne ||
          mission_state == MissionControllerState::ExecutingWaypoint ||
          mission_state == MissionControllerState::Holding) {
        publishStationary(std::nullopt);
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        last_setpoint_time_ = now;
        return;
      }
    }
    if (std::isfinite(since_activation_s) && since_activation_s <= trajectory_wait_timeout_s_) {
      publishStationary(std::nullopt);
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      last_setpoint_time_ = now;
      return;
    }
    failNavigation(kTrajectoryFailureReason);
    return;
  }

  if (!odometry.has_value()) {
    failNavigation("navigation odometry unavailable");
    return;
  }
  const auto& odometry_stamp = odometry->header.stamp;
  const double state_age_s = (now - rclcpp::Time(odometry_stamp)).seconds();
  // DDS delivery can put a freshly produced odometry sample a few
  // milliseconds ahead of the controller clock. Treat only this bounded
  // timestamp skew as fresh; a larger future stamp remains fail-closed.
  constexpr double kMaximumFutureStateSkewS = 0.05;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    last_state_age_s_ = std::max(0.0, state_age_s);
  }
  if (!std::isfinite(state_age_s) || state_age_s < -kMaximumFutureStateSkewS ||
      state_age_s > state_stale_after_s_) {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      ++stale_state_failure_count_;
    }
    RCLCPP_ERROR(node().get_logger(),
                 "navigation odometry unavailable or stale: age=%.6f s limit=%.6f s now=%ld stamp=%ld",
                 state_age_s, state_stale_after_s_, static_cast<long>(now.nanoseconds()),
                 static_cast<long>(rclcpp::Time(odometry_stamp).nanoseconds()));
    failNavigation("navigation odometry unavailable or stale");
    return;
  }

  const double elapsed_s = std::max(0.0, (now - start_time).seconds());
  const bool pass_through_braking = [&]() {
    if (!mission_controller_ ||
        mission_controller_->state() != MissionControllerState::Braking) {
      return false;
    }
    return mission_controller_->activeWaypoint().behavior ==
           MissionWaypoint::Behavior::PassThrough;
  }();
  bool planner_heartbeat_recent = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    planner_heartbeat_recent = last_planner_heartbeat_ns_ > 0 &&
                               now.nanoseconds() >= last_planner_heartbeat_ns_ &&
                               static_cast<double>(now.nanoseconds() -
                                                   last_planner_heartbeat_ns_) /
                                       1e9 <= trajectory_wait_timeout_s_;
  }
  const double freshness_limit_s = trajectory->duration_s + stale_after_s_;
  // A zero-duration braking stop is deliberately held while a pass-through
  // waypoint waits for the next map revision. The planner heartbeat is the
  // liveness proof in this bounded state; without this exception the PX4
  // adapter hands over after ``duration + stale_after`` before the rolling
  // planner can publish its replacement trajectory.
  if (!std::isfinite(elapsed_s) ||
      (elapsed_s > freshness_limit_s &&
       !(pass_through_braking && planner_heartbeat_recent))) {
    failNavigation(kTrajectoryFailureReason);
    return;
  }

  // Replanning restarts each trajectory at the timestamp carried by that
  // trajectory. All setpoint fields below come from the same verified sample.
  auto current_sample = sampleTrajectory(*trajectory, elapsed_s);
  const Eigen::Vector3d position_enu{odometry->pose.pose.position.x, odometry->pose.pose.position.y,
                                     odometry->pose.pose.position.z};
  const double dt_s = std::max(1e-3, (now - previous_setpoint_time).seconds());
  Eigen::Vector3f position_ned;
  Eigen::Vector3f velocity_ned;
  Eigen::Vector3f acceleration_ned;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    position_ned = enuToNed(current_sample.position_enu);
    if (output_mode_ == OutputMode::Velocity) {
      const auto preview_sample = sampleTrajectory(*trajectory, elapsed_s + trajectory_preview_s_);
      velocity_ned = enuToNed(
          velocity_tracker_.update(current_sample, preview_sample, position_enu, dt_s));
    } else {
      velocity_ned = enuToNed(current_sample.velocity_enu);
    }
    acceleration_ned = enuToNed(current_sample.acceleration_enu);
    last_setpoint_time_ = now;
  }
  px4_ros2::TrajectorySetpoint setpoint;
  if (output_mode_ == OutputMode::Velocity) {
    setpoint.withVelocity(velocity_ned);
  } else {
    setpoint.withPosition(position_ned).withVelocity(velocity_ned);
    if (output_mode_ == OutputMode::PositionVelocityAcceleration) {
      setpoint.withAcceleration(acceleration_ned);
    }
  }
  trajectory_setpoint_->update(setpoint);
}

NavigationModeExecutor::NavigationModeExecutor(px4_ros2::ModeBase& owned_mode)
    : ModeExecutorBase(px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
      node_(owned_mode.node()) {
  auto* navigation_mode = dynamic_cast<NavigationMode*>(&owned_mode);
  if (navigation_mode == nullptr) {
    throw std::invalid_argument("navigation mode executor received an incompatible owned mode");
  }
  navigation_mode->setPositionControlHandover([this]() {
    RCLCPP_WARN(node_.get_logger(), "External Mode requesting PX4 POSCTL handover");
    scheduleMode(px4_ros2::ModeBase::kModeIDPosctl, [this](px4_ros2::Result result) {
      RCLCPP_INFO(node_.get_logger(), "PX4 POSCTL handover completed with result=%s",
                  px4_ros2::resultToString(result));
    });
  });
}

void NavigationModeExecutor::onActivate() {
  RCLCPP_INFO(node_.get_logger(), "UAV Navigation External Mode activated");
  scheduleMode(ownedMode().id(), [this](px4_ros2::Result result) {
    onOwnedModeCompleted(result);
  });
}

void NavigationModeExecutor::onOwnedModeCompleted(px4_ros2::Result result) {
  if (result == px4_ros2::Result::Deactivated) {
    RCLCPP_DEBUG(node_.get_logger(), "Owned navigation mode was deactivated by mode handover");
    return;
  }
  if (result != px4_ros2::Result::Success) {
    RCLCPP_ERROR(node_.get_logger(), "Navigation mode completed with result=%s; handing over to POSCTL",
                 px4_ros2::resultToString(result));
    scheduleMode(px4_ros2::ModeBase::kModeIDPosctl, [this](px4_ros2::Result handover_result) {
      RCLCPP_INFO(node_.get_logger(), "PX4 POSCTL handover completed with result=%s",
                  px4_ros2::resultToString(handover_result));
    });
    return;
  }
  RCLCPP_INFO(node_.get_logger(), "Navigation mission completed; handing over to PX4 POSCTL");
  scheduleMode(px4_ros2::ModeBase::kModeIDPosctl, [this](px4_ros2::Result handover_result) {
    RCLCPP_INFO(node_.get_logger(), "PX4 POSCTL handover completed with result=%s",
                px4_ros2::resultToString(handover_result));
  });
}

void NavigationModeExecutor::onDeactivate(DeactivateReason reason) {
  RCLCPP_INFO(node_.get_logger(), "UAV Navigation External Mode deactivated (%s)",
              reason == DeactivateReason::FailsafeActivated ? "failsafe" : "mode_exit");
}

void NavigationModeExecutor::onFailsafeDeferred() {
  RCLCPP_WARN(node_.get_logger(), "PX4 requested a deferred failsafe; no failsafe is deferred");
}

}  // namespace px4_navigation_external_mode

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  using Node = px4_ros2::NodeWithModeExecutor<
      px4_navigation_external_mode::NavigationModeExecutor,
      px4_navigation_external_mode::NavigationMode>;
  rclcpp::spin(std::make_shared<Node>("px4_navigation_external_mode", true));
  rclcpp::shutdown();
  return 0;
}
