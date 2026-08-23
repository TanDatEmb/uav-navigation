#include "px4_navigation_external_mode/navigation_mode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>

#include <coordinate_conventions/frame_conventions.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

namespace px4_navigation_external_mode {
namespace {

constexpr char kModeName[] = "UAV Navigation";
constexpr char kTrajectoryFailureReason[] = "navigation trajectory unavailable or stale";

Eigen::Vector3f enuToNed(const Eigen::Vector3d& value_enu) {
  return coordinate_conventions::enuToNed(value_enu).cast<float>();
}

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

NavigationMode::OutputMode resolveOutputMode(const std::string& output,
                                             bool prefer_velocity_output) {
  if (output == "velocity") return NavigationMode::OutputMode::Velocity;
  if (output == "position_velocity") return NavigationMode::OutputMode::PositionVelocity;
  return prefer_velocity_output ? NavigationMode::OutputMode::Velocity
                                : NavigationMode::OutputMode::PositionVelocityAcceleration;
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
      node_(node),
      trajectory_setpoint_(std::make_shared<px4_ros2::TrajectorySetpointType>(*this)),
      super_command_topic_(node.declare_parameter<std::string>(
          "navigation.super_command_topic", "/navigation/super_command")),
      goal_topic_(node.declare_parameter<std::string>(
          "navigation.goal_topic", "/navigation/goal")),
      planning_frame_(node.declare_parameter<std::string>(
          "navigation.planning_frame", "lio_odom")),
      stale_after_s_(node.declare_parameter<double>(
          "navigation.trajectory_stale_after_s", 0.75)),
      command_anchor_max_error_m_(node.declare_parameter<double>(
          "navigation.command_anchor_max_error_m", 2.0)),
      state_stale_after_s_(node.declare_parameter<double>(
          "navigation.state_stale_after_s", 0.5)),
      trajectory_wait_timeout_s_(node.declare_parameter<double>(
          "navigation.trajectory_wait_timeout_s", 2.0)),
      prefer_velocity_output_(node.declare_parameter(
          "navigation.prefer_velocity_output", false)),
      lio_health_grace_s_(node.declare_parameter<double>(
          "navigation.lio_health_grace_s", 1.0)) {
  if (super_command_topic_.empty() || goal_topic_.empty() || planning_frame_.empty() ||
      !std::isfinite(stale_after_s_) || stale_after_s_ <= 0.0 ||
      !std::isfinite(command_anchor_max_error_m_) || command_anchor_max_error_m_ <= 0.0 ||
      !std::isfinite(state_stale_after_s_) || state_stale_after_s_ <= 0.0 ||
      !std::isfinite(trajectory_wait_timeout_s_) || trajectory_wait_timeout_s_ <= 0.0 ||
      !std::isfinite(lio_health_grace_s_) || lio_health_grace_s_ < 0.0) {
    throw std::invalid_argument("invalid PX4 navigation external mode parameters");
  }
  super_command_subscription_ = node.create_subscription<
      mars_quadrotor_msgs::msg::PositionCommand>(
      super_command_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const mars_quadrotor_msgs::msg::PositionCommand::ConstSharedPtr& message) {
        onSuperCommand(message);
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
  const auto mission_file = node.declare_parameter<std::string>("navigation.mission_file", "");
  if (!mission_file.empty()) {
    if (goal_topic_.empty()) {
      throw std::invalid_argument("navigation.goal_topic must not be empty for a mission");
    }
    mission_ = loadMission(mission_file, planning_frame_);
    output_mode_ = resolveOutputMode(mission_->control.output, prefer_velocity_output_);
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

std::optional<Eigen::Vector3d> NavigationMode::handoverPosition() {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (safety_hold_position_.has_value()) return safety_hold_position_;
  if (completion_position_.has_value()) return completion_position_;
  if (odometry_.has_value()) {
    return Eigen::Vector3d{odometry_->pose.pose.position.x,
                           odometry_->pose.pose.position.y,
                           odometry_->pose.pose.position.z};
  }
  return std::nullopt;
}

void NavigationMode::publishStatus(std::uint8_t state, std::uint8_t reason,
                                   const MissionControllerEvent* event) {
  if (!status_publisher_ || !mission_ || !mission_controller_) return;
  navigation_interfaces::msg::NavigationModeStatus status;
  status.header.stamp = node().get_clock()->now();
  status.header.frame_id = planning_frame_;
  status.mission_id = mission_->id;
  status.waypoint_index = static_cast<std::uint32_t>(mission_controller_->activeWaypointIndex());
  status.request_id = mission_controller_->activeRequestId();
  status.state = state;
  status.reason = reason;
  if (event != nullptr && event->waypoint_accepted) {
    status.waypoint_accepted = true;
    status.accepted_waypoint_index =
        static_cast<std::uint32_t>(event->accepted_waypoint_index);
    status.acceptance_position_error_m = event->acceptance_position_error_m;
    status.acceptance_speed_mps = event->acceptance_speed_mps;
  }
  status_publisher_->publish(status);
  last_status_state_ = state;
}

void NavigationMode::onActivate() {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    activation_time_ = node().get_clock()->now();
    last_setpoint_time_ = activation_time_;
    failure_reported_ = false;
    super_command_.reset();
    mission_complete_published_ = false;
    // Estimator and odometry freshness are process-level observations, not
    // per-activation state.  Clearing them here makes PX4 see an artificial
    // health gap in the first few hundred milliseconds after a mode
    // re-entry, which immediately activates the external-mode failsafe.
    // Keep the last samples and let the normal freshness checks decide
    // whether they are still usable.
    odometry_callback_count_ = 0U;
    trajectory_received_count_ = 0U;
    trajectory_accepted_count_ = 0U;
    trajectory_rejected_count_ = 0U;
    setpoint_update_count_ = 0U;
    stale_state_failure_count_ = 0U;
    last_goal_publish_ns_ = 0;
    last_command_receive_ns_ = 0;
    maximum_odometry_callback_gap_us_ = 0;
    last_setpoint_update_ns_ = 0;
    maximum_setpoint_callback_gap_us_ = 0;
    last_metrics_log_ns_ = 0;
    last_state_age_s_ = -1.0;
    mode_active_ = true;
    mission_terminal_ = false;
    handover_requested_ = false;
    completion_position_.reset();
    safety_hold_position_.reset();
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
    super_command_.reset();
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
  // goal yet, so the planner has no command to publish. Requiring command
  // freshness here makes PX4 fail the warm-up before arm/takeoff completes.
  // Once airborne, the normal command freshness gate is active.
  if (mission_ && !waiting_for_airborne &&
      stale(last_command_receive_ns_, trajectory_wait_timeout_s_)) {
    const double active_s = activation_time_.nanoseconds() > 0
                                ? static_cast<double>(now_ns - activation_time_.nanoseconds()) / 1e9
                                : 0.0;
    if (active_s > trajectory_wait_timeout_s_) {
      reporter.armingCheckFailureExt(
          px4_ros2::events::ID("uav_navigation_planner_command_stale"),
          px4_ros2::events::Log::Error, "Navigation planner command is stale");
    }
  }
}

void NavigationMode::onSuperCommand(
    const mars_quadrotor_msgs::msg::PositionCommand::ConstSharedPtr& message) {
  const auto finite = [](double value) { return std::isfinite(value); };
  const bool valid = message != nullptr && message->header.frame_id == planning_frame_ &&
                     (message->header.stamp.sec > 0 || message->header.stamp.nanosec > 0) &&
                     finite(message->position.x) && finite(message->position.y) &&
                     finite(message->position.z) && finite(message->velocity.x) &&
                     finite(message->velocity.y) && finite(message->velocity.z) &&
                     finite(message->acceleration.x) && finite(message->acceleration.y) &&
                     finite(message->acceleration.z) && finite(message->jerk.x) &&
                     finite(message->jerk.y) && finite(message->jerk.z) &&
                     finite(message->yaw) && finite(message->yaw_dot) &&
                     message->trajectory_id != 0U && message->trajectory_status !=
                         mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_EMPTY &&
                     (message->trajectory_flag ==
                          mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_MAIN ||
                      message->trajectory_flag ==
                          mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_BACKUP);
  if (!valid) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    ++trajectory_rejected_count_;
    super_command_.reset();
    return;
  }

  bool accepted = false;
  bool anchor_invalid = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (super_command_.has_value() &&
        message->trajectory_id <= super_command_->trajectory_id) {
      ++trajectory_rejected_count_;
      return;
    }
    const bool terminal_failure =
        message->trajectory_status ==
            mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_EMER ||
        message->trajectory_status ==
            mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_START ||
        message->trajectory_status ==
            mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_FINAL ||
        message->trajectory_status ==
            mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
    if (!terminal_failure && odometry_.has_value()) {
      const auto& point = odometry_->pose.pose.position;
      const Eigen::Vector3d measured{point.x, point.y, point.z};
      const Eigen::Vector3d command_position{message->position.x, message->position.y,
                                              message->position.z};
      anchor_invalid = measured.allFinite() && command_position.allFinite() &&
                       (command_position - measured).norm() > command_anchor_max_error_m_;
      if (anchor_invalid) {
        ++trajectory_rejected_count_;
      }
    }
    if (!anchor_invalid) {
      super_command_ = *message;
      ++trajectory_received_count_;
      ++trajectory_accepted_count_;
      last_command_receive_ns_ = node().get_clock()->now().nanoseconds();
      failure_reported_ = false;
      accepted = true;
    }
  }
  if (anchor_invalid) {
    safetyStopNavigation("SUPER PVA command anchor is not near vehicle");
    return;
  }
  if (accepted && mission_controller_) mission_controller_->onNativeTrajectoryReady();
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
      super_command_.reset();
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
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      last_goal_publish_ns_ = time.nanoseconds();
    }
    publishStatus(navigation_interfaces::msg::NavigationModeStatus::ACTIVE,
                  navigation_interfaces::msg::NavigationModeStatus::NONE, &event);
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
                  navigation_interfaces::msg::NavigationModeStatus::NONE, &event);
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
    RCLCPP_ERROR(node().get_logger(), "Mission '%s' failed at waypoint %zu",
                 mission_->id.c_str(), event.waypoint_index);
    (void)now_s;
    safetyStopNavigation("mission controller reported failure");
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
  Eigen::Vector3d velocity_command_enu;
  std::uint64_t forward_guard_count;
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
    velocity_command_enu = last_velocity_command_enu_;
    forward_guard_count = last_forward_guard_count_;
  }
  RCLCPP_INFO(node().get_logger(),
              "external_mode_metrics odom_callbacks=%lu odom_max_gap_us=%ld "
              "trajectory_received=%lu trajectory_accepted=%lu trajectory_rejected=%lu "
              "setpoint_updates=%lu setpoint_max_gap_us=%ld last_state_age_s=%.6f "
              "stale_state_failures=%lu velocity_command_enu=(%.3f,%.3f,%.3f) "
              "forward_guard_count=%lu",
              static_cast<unsigned long>(odometry_callbacks),
              static_cast<long>(odometry_gap_us),
              static_cast<unsigned long>(trajectories_received),
              static_cast<unsigned long>(trajectories_accepted),
              static_cast<unsigned long>(trajectories_rejected),
              static_cast<unsigned long>(setpoint_updates),
              static_cast<long>(setpoint_gap_us), state_age_s,
              static_cast<unsigned long>(stale_state_failures), velocity_command_enu.x(),
              velocity_command_enu.y(), velocity_command_enu.z(),
              static_cast<unsigned long>(forward_guard_count));
}

void NavigationMode::safetyStopNavigation(const char* reason) {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (failure_reported_) return;
    if (odometry_.has_value()) {
      const auto& point = odometry_->pose.pose.position;
      const Eigen::Vector3d measured{point.x, point.y, point.z};
      if (measured.allFinite()) safety_hold_position_ = measured;
    }
    failure_reported_ = true;
    handover_requested_ = true;
    super_command_.reset();
  }
  if (mission_controller_) mission_controller_->deactivate();
  publishStatus(navigation_interfaces::msg::NavigationModeStatus::PAUSED,
                navigation_interfaces::msg::NavigationModeStatus::SAFETY_STOP);
  RCLCPP_ERROR(node().get_logger(), "%s; safety hold then handover to PX4 POSCTL", reason);
  if (position_control_handover_) {
    position_control_handover_();
  } else {
    completed(px4_ros2::Result::ModeFailureOther);
  }
}

void NavigationMode::failNavigation(const char* reason) {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (failure_reported_) return;
    if (odometry_.has_value()) {
      const auto& point = odometry_->pose.pose.position;
      const Eigen::Vector3d measured{point.x, point.y, point.z};
      if (measured.allFinite()) safety_hold_position_ = measured;
    }
    failure_reported_ = true;
    handover_requested_ = true;
    super_command_.reset();
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
  std::optional<mars_quadrotor_msgs::msg::PositionCommand> super_command;
  std::optional<nav_msgs::msg::Odometry> odometry;
  const auto now = node().get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    super_command = super_command_;
    odometry = odometry_;
  }
  const auto publishStationary = [&](const std::optional<Eigen::Vector3d>& position_enu) {
    px4_ros2::TrajectorySetpoint setpoint;
    if (!position_enu.has_value()) {
      setpoint.withVelocity(Eigen::Vector3f::Zero());
    } else {
      setpoint.withPosition(enuToNed(*position_enu)).withVelocity(Eigen::Vector3f::Zero());
      if (output_mode_ == OutputMode::PositionVelocityAcceleration) {
        setpoint.withAcceleration(Eigen::Vector3f::Zero());
      }
    }
    trajectory_setpoint_->update(setpoint);
  };
  const auto publishPositionHold = [&](const Eigen::Vector3d& position_enu) {
    // Velocity-only is the normal flight output, but a terminal hold needs a
    // bounded position reference as well as zero velocity.  Sending only a
    // zero-velocity setpoint leaves PX4 free to drift in altitude and makes
    // the external velocity tracker the sole source of position correction;
    // use PX4's position controller for this short, latched terminal hold.
    px4_ros2::TrajectorySetpoint setpoint;
    setpoint.withPosition(enuToNed(position_enu)).withVelocity(Eigen::Vector3f::Zero());
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
      publishStationary(safety_hold_position_);
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
      if (safety_hold_position_.has_value()) {
        handover_position = safety_hold_position_;
      } else if (mission_controller_) {
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
    publishPositionHold(waypoint.position_enu);
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

  // Native SUPER command path.  The planner FSM already evaluated the
  // polynomial and selected main versus backup trajectory.  PX4 must receive
  // that PVA state directly; applying a second velocity controller here would
  // change SUPER's trajectory and reintroduce the old terminal oscillation.
  if (super_command.has_value()) {
    const auto receive_ns = [&]() {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      return last_command_receive_ns_;
    }();
    if (receive_ns > 0 && now.nanoseconds() >= receive_ns &&
        static_cast<double>(now.nanoseconds() - receive_ns) / 1e9 > stale_after_s_) {
      safetyStopNavigation("SUPER PVA command stale");
      return;
    }
    const auto& command = *super_command;
    const auto command_stamp_ns = rclcpp::Time(command.header.stamp).nanoseconds();
    if (command_stamp_ns <= 0 ||
        (now.nanoseconds() >= command_stamp_ns &&
         static_cast<double>(now.nanoseconds() - command_stamp_ns) / 1e9 > stale_after_s_) ||
        (command_stamp_ns > now.nanoseconds() &&
         static_cast<double>(command_stamp_ns - now.nanoseconds()) / 1e9 > stale_after_s_)) {
      safetyStopNavigation("SUPER PVA command timestamp invalid or stale");
      return;
    }
    const Eigen::Vector3d position_enu{command.position.x, command.position.y,
                                       command.position.z};
    const Eigen::Vector3d velocity_enu{command.velocity.x, command.velocity.y,
                                       command.velocity.z};
    const Eigen::Vector3d acceleration_enu{command.acceleration.x, command.acceleration.y,
                                            command.acceleration.z};
    if (command.trajectory_status ==
        mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_EMER ||
        command.trajectory_status ==
        mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_START ||
        command.trajectory_status ==
        mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_ILLEGAL_FINAL ||
        command.trajectory_status ==
        mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE) {
      safetyStopNavigation("SUPER planner failed without a valid backup trajectory");
      return;
    }
    if (command.trajectory_status ==
        mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_COMPLETED) {
      publishPositionHold(position_enu);
      if (command.trajectory_flag ==
          mars_quadrotor_msgs::msg::PositionCommand::TRAJECTORY_FLAG_BACKUP) {
        safetyStopNavigation("SUPER backup trajectory completed before planner recovery");
        return;
      }
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      last_setpoint_time_ = now;
      return;
    }
    px4_ros2::TrajectorySetpoint setpoint;
    setpoint.withPosition(enuToNed(position_enu))
        .withVelocity(enuToNed(velocity_enu))
        .withAcceleration(enuToNed(acceleration_enu));
    trajectory_setpoint_->update(setpoint);
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      last_velocity_command_enu_ = velocity_enu;
      last_setpoint_time_ = now;
    }
    return;
  }

  // A goal publication and its first PVA command are asynchronous. Hold the
  // current position during this bounded acquisition window so the mode does
  // not hand over to POSCTL on the same 50 Hz tick that starts planning. Once
  // the window expires, fail closed instead of reusing an older trajectory.
  std::int64_t goal_publish_ns = 0;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    goal_publish_ns = last_goal_publish_ns_;
  }
  if (goal_publish_ns > 0 && now.nanoseconds() >= goal_publish_ns &&
      static_cast<double>(now.nanoseconds() - goal_publish_ns) / 1e9 <=
          trajectory_wait_timeout_s_) {
    publishStationary(odometry.has_value()
                          ? std::optional<Eigen::Vector3d>{Eigen::Vector3d{
                                odometry->pose.pose.position.x,
                                odometry->pose.pose.position.y,
                                odometry->pose.pose.position.z}}
                          : std::nullopt);
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    last_setpoint_time_ = now;
    return;
  }
  safetyStopNavigation("SUPER PVA command unavailable");
}

NavigationHoldMode::NavigationHoldMode(rclcpp::Node& node)
    : ModeBase(node, Settings{"UAV Navigation Hold"}),
      trajectory_setpoint_(std::make_shared<px4_ros2::TrajectorySetpointType>(*this)) {
  setSetpointUpdateRate(50.0F);
}

void NavigationHoldMode::setHoldPosition(const Eigen::Vector3d& position_enu) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_enu_ = position_enu;
}

void NavigationHoldMode::onActivate() {
  RCLCPP_INFO(node().get_logger(), "UAV Navigation Hold activated");
}

void NavigationHoldMode::onDeactivate() {
  RCLCPP_INFO(node().get_logger(), "UAV Navigation Hold deactivated");
}

void NavigationHoldMode::updateSetpoint(float) {
  std::optional<Eigen::Vector3d> target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target = target_enu_;
  }
  if (!target.has_value()) {
    completed(px4_ros2::Result::ModeFailureOther);
    return;
  }
  px4_ros2::TrajectorySetpoint setpoint;
  setpoint.withPosition(enuToNed(*target))
      .withVelocity(Eigen::Vector3f::Zero())
      .withAcceleration(Eigen::Vector3f::Zero());
  trajectory_setpoint_->update(setpoint);
}

NavigationModeExecutor::NavigationModeExecutor(
    px4_ros2::ModeBase& owned_mode, NavigationHoldMode& hold_mode)
    : ModeExecutorBase(px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
      node_(owned_mode.node()),
      navigation_mode_(dynamic_cast<NavigationMode&>(owned_mode)),
      hold_mode_(hold_mode) {
  navigation_mode_.setPositionControlHandover([this]() {
    RCLCPP_WARN(node_.get_logger(), "External Mode requesting PX4 POSCTL handover");
    scheduleMode(px4_ros2::ModeBase::kModeIDPosctl, [this](px4_ros2::Result result) {
      onPositionControlHandoverCompleted(result, true);
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
      onPositionControlHandoverCompleted(handover_result, false);
    });
    return;
  }
  RCLCPP_INFO(node_.get_logger(), "Navigation mission completed; handing over to PX4 POSCTL");
  scheduleMode(px4_ros2::ModeBase::kModeIDPosctl, [this](px4_ros2::Result handover_result) {
    onPositionControlHandoverCompleted(handover_result, false);
  });
}

void NavigationModeExecutor::onPositionControlHandoverCompleted(
    px4_ros2::Result result, bool complete_navigation_failure) {
  if (result == px4_ros2::Result::Success || result == px4_ros2::Result::Deactivated) {
    RCLCPP_INFO(node_.get_logger(), "PX4 POSCTL handover completed with result=%s",
                px4_ros2::resultToString(result));
    return;
  }

  // POSCTL requires a manual-control source and PX4 LOITER requires a global
  // position on common headless LIO-only configurations.  Transition to the
  // registered position-hold mode when POSCTL is unavailable.
  RCLCPP_WARN(node_.get_logger(),
              "PX4 POSCTL handover failed with result=%s; requesting External Hold",
              px4_ros2::resultToString(result));
  scheduleExternalHold(complete_navigation_failure);
}

void NavigationModeExecutor::scheduleExternalHold(bool complete_navigation_failure) {
  const auto hold_position = navigation_mode_.handoverPosition();
  if (!hold_position.has_value()) {
    onHoldHandoverCompleted(px4_ros2::Result::Rejected, complete_navigation_failure);
    return;
  }
  hold_mode_.setHoldPosition(*hold_position);
  scheduleMode(hold_mode_.id(), [this, complete_navigation_failure](px4_ros2::Result hold_result) {
    onHoldHandoverCompleted(hold_result, complete_navigation_failure);
  });
}

void NavigationModeExecutor::onHoldHandoverCompleted(
    px4_ros2::Result result, bool complete_navigation_failure) {
  if (result == px4_ros2::Result::Success || result == px4_ros2::Result::Deactivated) {
    RCLCPP_INFO(node_.get_logger(), "External Hold handover completed with result=%s",
                px4_ros2::resultToString(result));
    return;
  }

  RCLCPP_ERROR(node_.get_logger(),
               "External Hold handover failed with result=%s; navigation cannot continue",
               px4_ros2::resultToString(result));
  if (complete_navigation_failure) {
    // scheduleMode() invokes this callback synchronously for Rejected/Timeout.
    // Without a terminal completion the executor keeps the external mode alive
    // while NavigationMode publishes a stationary setpoint indefinitely.
    ownedMode().completed(px4_ros2::Result::ModeFailureOther);
  }
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
      px4_navigation_external_mode::NavigationMode,
      px4_navigation_external_mode::NavigationHoldMode>;
  rclcpp::spin(std::make_shared<Node>("px4_navigation_external_mode", true));
  rclcpp::shutdown();
  return 0;
}
