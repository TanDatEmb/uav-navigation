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

Eigen::Vector3f enuToNed(const Eigen::Vector3d& value_enu) {
  // ROS uses ENU (+X east, +Y north, +Z up); PX4 setpoints use NED
  // (+X north, +Y east, +Z down).  Keep this identical to the odometry
  // bridge so position and velocity commands use the same world basis.
  return Eigen::Vector3f{static_cast<float>(value_enu.y()),
                         static_cast<float>(value_enu.x()),
                         static_cast<float>(-value_enu.z())};
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
      super_trajectory_topic_(node.declare_parameter<std::string>(
          "navigation.super_trajectory_topic", "/navigation/super_trajectory")),
      goal_topic_(node.declare_parameter<std::string>(
          "navigation.goal_topic", "/navigation/goal")),
      planner_heartbeat_topic_(node.declare_parameter<std::string>(
          "navigation.planner_heartbeat_topic", "/navigation/planner_heartbeat")),
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
      prefer_velocity_output_(node.declare_parameter(
          "navigation.prefer_velocity_output", true)),
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
        config.max_jerk_mps3 = node.declare_parameter<double>(
            "navigation.velocity_tracker.max_jerk_mps3", 6.0);
        config.max_position_error_m = node.declare_parameter<double>(
            "navigation.velocity_tracker.max_position_error_m", 2.0);
        return config;
      }()) {
  if (super_trajectory_topic_.empty() || goal_topic_.empty() ||
      planner_heartbeat_topic_.empty() || planning_frame_.empty() ||
      !std::isfinite(stale_after_s_) ||
      stale_after_s_ <= 0.0 || !std::isfinite(state_stale_after_s_) || state_stale_after_s_ <= 0.0 ||
      !std::isfinite(trajectory_wait_timeout_s_) || trajectory_wait_timeout_s_ <= 0.0 ||
      !std::isfinite(lio_health_grace_s_) || lio_health_grace_s_ < 0.0 ||
      !std::isfinite(trajectory_preview_s_) || trajectory_preview_s_ < 0.0) {
    throw std::invalid_argument("invalid PX4 navigation external mode parameters");
  }
  super_trajectory_subscription_ = node.create_subscription<
      mars_quadrotor_msgs::msg::PolynomialTrajectory>(
      super_trajectory_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const mars_quadrotor_msgs::msg::PolynomialTrajectory::ConstSharedPtr& message) {
        onSuperTrajectory(message);
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
      planner_heartbeat_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const std_msgs::msg::Empty::ConstSharedPtr& message) {
        onPlannerHeartbeat(message);
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
    velocity_tracker_.reset();
    failure_reported_ = false;
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
    super_trajectory_.reset();
    velocity_tracker_.reset();
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

void NavigationMode::onSuperTrajectory(
    const mars_quadrotor_msgs::msg::PolynomialTrajectory::ConstSharedPtr& message) {
  SuperPolynomialTrajectory candidate;
  std::string error;
  if (!candidate.assign(*message, &error)) {
    RCLCPP_WARN(node().get_logger(), "Rejecting SUPER polynomial trajectory: %s", error.c_str());
    bool active = false;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      ++trajectory_rejected_count_;
      super_trajectory_.reset();
      active = mode_active_;
    }
    if (active) failNavigation("SUPER polynomial trajectory invalid");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const std::uint32_t newest_id = super_trajectory_.has_value()
                                        ? super_trajectory_->trajectoryId()
                                        : 0U;
    if (candidate.trajectoryId() <= newest_id) {
      ++trajectory_rejected_count_;
      RCLCPP_WARN(node().get_logger(),
                  "Rejecting non-monotonic SUPER trajectory id=%u newest=%u",
                  candidate.trajectoryId(), newest_id);
      return;
    }
    super_trajectory_ = std::move(candidate);
    ++trajectory_received_count_;
    ++trajectory_accepted_count_;
    last_trajectory_receive_ns_ = node().get_clock()->now().nanoseconds();
    last_planner_heartbeat_ns_ = last_trajectory_receive_ns_;
    failure_reported_ = false;
  }
  if (mission_controller_) mission_controller_->onNativeTrajectoryReady();
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
      super_trajectory_.reset();
      velocity_tracker_.reset();
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
      super_trajectory_.reset();
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

void NavigationMode::failNavigation(const char* reason) {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (failure_reported_) return;
    failure_reported_ = true;
    handover_requested_ = true;
    super_trajectory_.reset();
    velocity_tracker_.reset();
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
  std::optional<SuperPolynomialTrajectory> super_trajectory;
  std::optional<nav_msgs::msg::Odometry> odometry;
  const auto now = node().get_clock()->now();
  rclcpp::Time previous_setpoint_time;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    super_trajectory = super_trajectory_;
    odometry = odometry_;
    previous_setpoint_time = last_setpoint_time_;
  }
  const auto velocityOnlyActive = [this]() {
    return output_mode_ == OutputMode::Velocity;
  };
  const auto publishStationary = [&](const std::optional<Eigen::Vector3d>& position_enu) {
    px4_ros2::TrajectorySetpoint setpoint;
    if (velocityOnlyActive() || !position_enu.has_value()) {
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

  // Native SUPER path. PX4 receives one polynomial and evaluates its
  // position/velocity/acceleration at the 50 Hz setpoint rate.
  if (super_trajectory.has_value()) {
    const auto receive_ns = [&]() {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      return last_trajectory_receive_ns_;
    }();
    const double trajectory_remaining_s =
        super_trajectory->startTimeSeconds() + super_trajectory->totalDurationSeconds() -
        now.seconds();
    if (receive_ns > 0 && now.nanoseconds() >= receive_ns &&
        static_cast<double>(now.nanoseconds() - receive_ns) / 1e9 > stale_after_s_ &&
        (!std::isfinite(trajectory_remaining_s) ||
         trajectory_remaining_s <= stale_after_s_)) {
      failNavigation("SUPER polynomial trajectory stale");
      return;
    }
    const auto state = super_trajectory->evaluate(now.seconds());
    if (state.finished || !state.position.allFinite() || !state.velocity.allFinite() ||
        !state.acceleration.allFinite()) {
      failNavigation(state.finished ? "SUPER polynomial trajectory expired"
                                    : "SUPER polynomial trajectory evaluation invalid");
      return;
    }
    if (!odometry.has_value()) {
      failNavigation("navigation odometry unavailable for SUPER tracking");
      return;
    }

    const Eigen::Vector3d measured_position_enu{
        odometry->pose.pose.position.x, odometry->pose.pose.position.y,
        odometry->pose.pose.position.z};
    if (!measured_position_enu.allFinite()) {
      failNavigation("navigation odometry invalid for SUPER tracking");
      return;
    }

    // Keep SUPER's polynomial P/V/A evaluation as the reference contract, but
    // use the bounded velocity tracker when
    // the mission resolves to velocity output.  Sending raw high-order
    // acceleration feed-forward directly to PX4 makes small polynomial or
    // estimator residuals turn into large lateral/vertical excursions in
    // SITL.  The tracker preserves the SUPER tangent while enforcing the
    // configured velocity, acceleration, deceleration and jerk limits.
    const auto preview_state = super_trajectory->evaluate(now.seconds() + trajectory_preview_s_);
    if (!preview_state.position.allFinite() || !preview_state.velocity.allFinite() ||
        !preview_state.acceleration.allFinite()) {
      failNavigation("SUPER polynomial preview evaluation invalid");
      return;
    }
    TrajectorySample current_reference;
    current_reference.position_enu = state.position;
    current_reference.velocity_enu = state.velocity;
    current_reference.acceleration_enu = state.acceleration;
    TrajectorySample preview_reference;
    preview_reference.position_enu = preview_state.position;
    preview_reference.velocity_enu = preview_state.velocity;
    preview_reference.acceleration_enu = preview_state.acceleration;
    const double dt_s = std::max(1e-3, (now - previous_setpoint_time).seconds());

    Eigen::Vector3d command_velocity_enu = state.velocity;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (velocityOnlyActive()) {
        command_velocity_enu = velocity_tracker_.update(
            current_reference, preview_reference, measured_position_enu, dt_s);
        last_forward_guard_count_ = velocity_tracker_.forwardGuardCount();
      }
      last_velocity_command_enu_ = command_velocity_enu;
      last_setpoint_time_ = now;
    }
    if (now.nanoseconds() - last_super_debug_log_ns_ >= 1000000000LL) {
      RCLCPP_INFO(node_.get_logger(),
                  "super_native_state ref_p=(%.2f,%.2f,%.2f) ref_v=(%.2f,%.2f,%.2f) "
                  "measured=(%.2f,%.2f,%.2f) command=(%.2f,%.2f,%.2f)",
                  state.position.x(), state.position.y(), state.position.z(),
                  state.velocity.x(), state.velocity.y(), state.velocity.z(),
                  measured_position_enu.x(), measured_position_enu.y(), measured_position_enu.z(),
                  command_velocity_enu.x(), command_velocity_enu.y(), command_velocity_enu.z());
      last_super_debug_log_ns_ = now.nanoseconds();
    }
    px4_ros2::TrajectorySetpoint setpoint;
    if (velocityOnlyActive()) {
      // Keep the planner's horizontal tangent as the high-speed command.  The
      // nominal polynomial remains the sole source of this command; no
      // alternate trajectory branch is introduced at the PX4 boundary.
      setpoint.withVelocity(enuToNed(command_velocity_enu));
    } else {
      setpoint.withPosition(enuToNed(state.position))
          .withVelocity(enuToNed(state.velocity));
      if (output_mode_ == OutputMode::PositionVelocityAcceleration) {
        setpoint.withAcceleration(enuToNed(state.acceleration));
      }
    }
    trajectory_setpoint_->update(setpoint);
    return;
  }

  // A goal publication and its first polynomial are asynchronous. Hold the
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
  failNavigation("SUPER polynomial trajectory unavailable");
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

  RCLCPP_ERROR(node_.get_logger(),
               "PX4 POSCTL handover failed with result=%s; navigation cannot continue",
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
      px4_navigation_external_mode::NavigationMode>;
  rclcpp::spin(std::make_shared<Node>("px4_navigation_external_mode", true));
  rclcpp::shutdown();
  return 0;
}
