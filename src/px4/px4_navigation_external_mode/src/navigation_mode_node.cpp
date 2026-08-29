#include "px4_navigation_external_mode/navigation_mode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>

#include <navigation_common/frame_conventions.hpp>
#include <navigation_common/time.hpp>
#include <navigation_contracts/command_safety_contract.hpp>
#include <navigation_contracts/navigation_command_contract.hpp>
#include <navigation_contracts/execution_state_freshness.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>

#include "px4_navigation_external_mode/tracking_envelope.hpp"
#include "px4_navigation_external_mode/navigation_input_validation.hpp"
#include "px4_navigation_external_mode/certified_command_handoff.hpp"
#include "px4_navigation_external_mode/reject_provenance.hpp"
#include "px4_navigation_external_mode/command_acceptance_gate.hpp"
#include "px4_navigation_external_mode/mission_command_identity.hpp"
#include "px4_navigation_external_mode/planner_recovery.hpp"
#include "px4_navigation_external_mode/runtime_metrics_policy.hpp"

namespace px4_navigation_external_mode {
namespace {

constexpr char kModeName[] = "Avoidance Mission";
constexpr char kTrajectoryFailureReason[] = "navigation trajectory unavailable or stale";

bool floatRepresentable(const double value) {
  return std::isfinite(value) &&
         std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}

std::optional<Eigen::Vector3f> checkedEnuToNed(const Eigen::Vector3d& value_enu) {
  if (!value_enu.allFinite()) return std::nullopt;
  const Eigen::Vector3d value_ned = navigation_common::enuToNed(value_enu);
  if (!value_ned.allFinite() ||
      (value_ned.cwiseAbs().array() > static_cast<double>(std::numeric_limits<float>::max()))
          .any()) {
    return std::nullopt;
  }
  return value_ned.cast<float>();
}

std::optional<std::int64_t> checkedTimestampAdd(const std::int64_t base_ns,
                                                 const std::int64_t delta_ns) {
  if ((delta_ns > 0 && base_ns > std::numeric_limits<std::int64_t>::max() - delta_ns) ||
      (delta_ns < 0 && base_ns < std::numeric_limits<std::int64_t>::min() - delta_ns)) {
    return std::nullopt;
  }
  return base_ns + delta_ns;
}

}  // namespace

NavigationMode::NavigationMode(rclcpp::Node& node)
    : ModeBase(node, Settings{kModeName}),
      node_(node),
      trajectory_setpoint_(std::make_shared<px4_ros2::TrajectorySetpointType>(*this)),
      navigation_command_topic_(node.declare_parameter<std::string>(
          "navigation.navigation_command_topic", "/navigation/navigation_command")),
      goal_topic_(node.declare_parameter<std::string>(
          "navigation.goal_topic", "/navigation/goal")),
      planning_frame_(node.declare_parameter<std::string>(
          "navigation.planning_frame", "lio_odom")),
      body_frame_(node.declare_parameter<std::string>(
          "navigation.body_frame", "base_link")),
      stale_after_s_(node.declare_parameter<double>(
          "navigation.trajectory_stale_after_s", 0.75)),
      state_stale_after_s_(node.declare_parameter<double>(
          "navigation.state_stale_after_s", 0.5)),
      trajectory_wait_timeout_s_(node.declare_parameter<double>(
          "navigation.trajectory_wait_timeout_s", 2.0)),
      planner_recovery_wait_timeout_s_(node.declare_parameter<double>(
          "navigation.planner_recovery_wait_timeout_s", 0.5)) {
  const auto stale_after_ns = navigation_common::secondsToNanoseconds(stale_after_s_);
  const auto state_stale_after_ns = navigation_common::secondsToNanoseconds(state_stale_after_s_);
  const auto planner_recovery_wait_timeout_ns =
      navigation_common::secondsToNanoseconds(planner_recovery_wait_timeout_s_);
  if (navigation_command_topic_.empty() || goal_topic_.empty() || planning_frame_.empty() ||
      body_frame_.empty() ||
      !std::isfinite(stale_after_s_) || stale_after_s_ <= 0.0 ||
      !std::isfinite(state_stale_after_s_) || state_stale_after_s_ <= 0.0 ||
      !std::isfinite(trajectory_wait_timeout_s_) || trajectory_wait_timeout_s_ <= 0.0 ||
      !std::isfinite(planner_recovery_wait_timeout_s_) ||
      planner_recovery_wait_timeout_s_ <= 0.0 ||
      planner_recovery_wait_timeout_s_ > trajectory_wait_timeout_s_ || !stale_after_ns ||
      !state_stale_after_ns || !planner_recovery_wait_timeout_ns || *stale_after_ns <= 0 ||
      *state_stale_after_ns <= 0 || *planner_recovery_wait_timeout_ns <= 0) {
    throw std::invalid_argument("invalid PX4 navigation external mode parameters");
  }
  stale_after_ns_ = *stale_after_ns;
  state_stale_after_ns_ = *state_stale_after_ns;
  planner_recovery_wait_timeout_ns_ = *planner_recovery_wait_timeout_ns;
  navigation_command_subscription_ = node.create_subscription<
      navigation_contracts::msg::NavigationCommand>(
      navigation_command_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const navigation_contracts::msg::NavigationCommand::ConstSharedPtr& message) {
        onNavigationCommand(message);
      });
  const auto state_topic = node.declare_parameter<std::string>(
      "navigation.state_topic", "/lio/odometry_propagated");
  if (state_topic.empty()) {
    throw std::invalid_argument("navigation.state_topic must not be empty");
  }
  odometry_subscription_ = node.create_subscription<
      navigation_contracts::msg::PropagatedOdometry>(
      // Propagated odometry is published reliably by FAST-LIO. This state is
      // a hard input to the velocity tracker, so do not downgrade it to
      // best-effort and allow a transient DDS drop to look like stale state.
      // The velocity tracker consumes the newest state. A one-sample queue
      // avoids applying a burst of stale odometry after DDS/executor jitter.
      state_topic, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const navigation_contracts::msg::PropagatedOdometry::ConstSharedPtr& message) {
        onOdometry(message);
      });
  estimator_health_subscription_ = node.create_subscription<
      navigation_contracts::msg::EstimatorHealth>(
      "/lio/health", rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort(),
      [this](const navigation_contracts::msg::EstimatorHealth::ConstSharedPtr& message) {
        onEstimatorHealth(message);
      });
  const auto mission_file = node.declare_parameter<std::string>("navigation.mission_file", "");
  if (!mission_file.empty()) {
    if (goal_topic_.empty()) {
      throw std::invalid_argument("navigation.goal_topic must not be empty for a mission");
    }
    mission_ = loadMission(mission_file, planning_frame_);
    RCLCPP_INFO(node.get_logger(), "External Mode command contract: planner backend PVA");
    mission_controller_ = std::make_unique<MissionController>(*mission_);
    goal_publisher_ = node.create_publisher<navigation_contracts::msg::NavigationGoal>(
        goal_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable());
    const auto status_topic = node.declare_parameter<std::string>(
        "navigation.status_topic", "/navigation/mode_status");
    if (status_topic.empty()) {
      throw std::invalid_argument("navigation.status_topic must not be empty for a mission");
    }
    status_publisher_ = node.create_publisher<navigation_contracts::msg::NavigationModeStatus>(
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

void NavigationMode::setPx4HoldHandover(std::function<void()> callback) {
  px4_hold_handover_ = std::move(callback);
}

void NavigationMode::publishStatus(std::uint8_t state, std::uint8_t reason,
                                   const MissionControllerEvent* event) {
  if (!status_publisher_ || !mission_ || !mission_controller_) return;
  navigation_contracts::msg::NavigationModeStatus status;
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
    navigation_command_ = transitionCertifiedCommand(
        navigation_command_, std::nullopt, CertifiedCommandTransition::kInvalidate);
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
    waypoint_handoff_retained_command_count_ = 0U;
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
    planner_recovery_pending_ = false;
    planner_recovery_deadline_ns_ = 0;
    last_completed_waypoint_index_ = 0U;
    last_completed_request_id_ = 0U;
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
    publishStatus(navigation_contracts::msg::NavigationModeStatus::ACTIVE,
                  navigation_contracts::msg::NavigationModeStatus::NONE);
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
    navigation_command_ = transitionCertifiedCommand(
        navigation_command_, std::nullopt, CertifiedCommandTransition::kInvalidate);
  }
  if (mission_controller_) mission_controller_->deactivate();
  if (last_status_state_ != navigation_contracts::msg::NavigationModeStatus::PAUSED &&
      last_status_state_ != navigation_contracts::msg::NavigationModeStatus::COMPLETE &&
      last_status_state_ != navigation_contracts::msg::NavigationModeStatus::FAILED) {
    publishStatus(navigation_contracts::msg::NavigationModeStatus::PAUSED,
                  navigation_contracts::msg::NavigationModeStatus::OPERATOR_TAKEOVER);
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
  if (diagnostics_stale || !lio_health_valid_) {
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

void NavigationMode::onNavigationCommand(
    const navigation_contracts::msg::NavigationCommand::ConstSharedPtr& message) {
  const bool valid = message != nullptr &&
                     navigation_contracts::commandContractValid(*message, planning_frame_) &&
                     navigation_contracts::commandValidAt(
                         *message, node().get_clock()->now().nanoseconds());
  if (!valid) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    ++trajectory_rejected_count_;
    // A malformed replacement does not revoke the independently certified
    // command already being executed. Its own validity/freshness and health
    // leases remain authoritative in updateSetpoint().
    navigation_command_ = transitionCertifiedCommand(
        navigation_command_, std::nullopt, CertifiedCommandTransition::kRetain);
    return;
  }

  bool accepted = false;
  bool anchor_invalid = false;
  bool odometry_stale = false;
  bool completed_command = false;
  bool terminal_backup_hold_inside_acceptance = false;
  bool terminal_main_hold_inside_acceptance = false;
  bool terminal_recovery_needed = false;
  bool recovery_deadline_invalid = false;
  std::optional<nav_msgs::msg::Odometry> completed_command_odometry;
  navigation_contracts::ExecutionStateFreshness odometry_freshness;
  TrackingEnvelopeResult tracking_envelope;
  std::optional<RejectProvenance> reject_provenance;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    // A command is executable only after a fresh, healthy typed-health sample
    // has established the current public estimator epoch.  Caching a command
    // before that handshake would let an untagged odometry stream become the
    // implicit epoch authority.
    const bool health_epoch_matches = navigation_contracts::estimatorHealthAllowsCommand(
        typed_health_seen_, lio_health_valid_, message->localization_epoch,
        lio_localization_epoch_);
    const bool mission_identity_matches = mission_ && mission_controller_ &&
        missionCommandIdentityMatches(
            *message, mission_->id,
            static_cast<std::uint32_t>(mission_controller_->activeWaypointIndex()),
            mission_controller_->activeRequestId(), mission_terminal_,
            last_completed_waypoint_index_, last_completed_request_id_);
    const bool command_identity_monotonic = !navigation_command_.has_value() ||
        navigation_contracts::commandWorldIdentityNonRegressing(
            *message, *navigation_command_);
    if (!health_epoch_matches || !mission_identity_matches || !command_identity_monotonic) {
      ++trajectory_rejected_count_;
      navigation_command_ = transitionCertifiedCommand(
          navigation_command_, std::nullopt, CertifiedCommandTransition::kRetain);
      return;
    }
    const auto odometry_source_ns = odometry_
        ? navigation_common::rosTimeToNanoseconds(odometry_->header.stamp).value_or(0) : 0;
    odometry_freshness = navigation_contracts::evaluateExecutionStateFreshness(
        node().get_clock()->now().nanoseconds(), odometry_source_ns,
        navigation_common::steadyClockNowNanoseconds(), last_odometry_receive_steady_ns_,
        state_stale_after_s_);
    const auto acceptance_gate = classifyCommandAcceptance(
        odometry_freshness, message->sample_id,
        navigation_command_ ? navigation_command_->sample_id : 0U);
    odometry_stale = acceptance_gate == CommandAcceptanceGate::kOdometryStale;
    if (odometry_stale) {
      ++trajectory_rejected_count_;
      if (!failure_reported_) ++stale_state_failure_count_;
    } else if (acceptance_gate == CommandAcceptanceGate::kNonIncreasingMessageId) {
      ++trajectory_rejected_count_;
      return;
    }
    const bool terminal_failure =
        message->status ==
            navigation_contracts::msg::NavigationCommand::STATUS_REJECTED;
    completed_command =
        message->status ==
        navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED;
    bool terminal_hold_inside_acceptance = false;
    // The final COMPLETED PVA can arrive after MissionController has already
    // advanced its checkpoint past the last waypoint.  Do not dereference
    // activeWaypoint() for that late terminal notification.
    if (completed_command && !mission_terminal_ && mission_controller_ &&
        odometry_.has_value()) {
      const auto waypoint = mission_controller_->activeWaypoint();
      if (!waypoint.has_value()) {
        completed_command = false;
      } else {
      const auto& point = odometry_->pose.pose.position;
      const Eigen::Vector3d measured{point.x, point.y, point.z};
      const Eigen::Vector3d command_position{message->position.x, message->position.y,
                                             message->position.z};
      const bool measured_finite = measured.allFinite();
      const bool command_finite = command_position.allFinite();
      const bool measured_inside_acceptance =
          measured_finite && (measured - waypoint->position_enu).norm() <=
              waypoint->acceptance_radius_m;
      const bool command_inside_acceptance =
          command_finite && (command_position - waypoint->position_enu).norm() <=
              waypoint->acceptance_radius_m;
      terminal_backup_hold_inside_acceptance =
          message->role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP &&
          measured_inside_acceptance &&
          backupEndpointHoldIsAnchored(
              command_inside_acceptance, measured_finite, command_finite,
              (measured - command_position).norm(),
              navigation_contracts::kCommandAnchorErrorLimitM);
      terminal_hold_inside_acceptance =
          terminal_backup_hold_inside_acceptance ||
          (waypoint->behavior == MissionWaypoint::Behavior::Stop &&
           measured_inside_acceptance && command_inside_acceptance);
      terminal_main_hold_inside_acceptance =
          message->role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN &&
          measured_inside_acceptance && command_inside_acceptance;
      terminal_recovery_needed =
          (message->role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP &&
           !terminal_backup_hold_inside_acceptance) ||
          (message->role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN &&
           !terminal_main_hold_inside_acceptance);
      }
    }
    if (!odometry_stale && !terminal_failure && !terminal_hold_inside_acceptance &&
        odometry_.has_value()) {
      const auto& point = odometry_->pose.pose.position;
      const Eigen::Vector3d measured{point.x, point.y, point.z};
      const Eigen::Vector3d command_position{message->position.x, message->position.y,
                                              message->position.z};
      const Eigen::Vector3d command_velocity{message->velocity.x, message->velocity.y,
                                             message->velocity.z};
      tracking_envelope = evaluateTrackingEnvelope(
          measured, command_position, command_velocity,
          navigation_contracts::kCommandAnchorErrorLimitM);
      anchor_invalid = !tracking_envelope.valid;
      if (anchor_invalid) {
        reject_provenance = buildRejectProvenance(
            node().get_clock()->now().nanoseconds(), last_odometry_receive_ns_,
            *odometry_, *message, navigation_command_);
        ++trajectory_rejected_count_;
      }
    }
    if (!anchor_invalid && !odometry_stale) {
      navigation_command_ = transitionCertifiedCommand(
          navigation_command_, *message, CertifiedCommandTransition::kCommit);
      ++trajectory_received_count_;
      ++trajectory_accepted_count_;
      last_command_receive_ns_ = node().get_clock()->now().nanoseconds();
      failure_reported_ = false;
      accepted = true;
      if (completed_command) completed_command_odometry = odometry_;
    }
  }
  if (odometry_stale) {
    RCLCPP_ERROR(node().get_logger(),
                 "Rejecting planner backend command because navigation odometry lease is stale: "
                 "reason=%s source_age_ms=%.3f receive_age_ms=%.3f generation=%lu "
                 "trajectory_time=%.6f",
                 navigation_contracts::executionStateFreshnessReasonName(
                     odometry_freshness.reason),
                 odometry_freshness.source_age_ms, odometry_freshness.receive_age_ms,
                 static_cast<unsigned long>(message->bundle_generation),
                 message->trajectory_time_s);
    failNavigation("navigation odometry stale at command acceptance");
    return;
  }
  if (anchor_invalid) {
    const char* role = "UNKNOWN";
    if (message->role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP) {
      role = "BACKUP";
    } else if (message->role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN) {
      role = "MAIN";
    } else if (message->role ==
               navigation_contracts::msg::NavigationCommand::ROLE_EMERGENCY) {
      role = "EMERGENCY";
    }
    const auto& provenance = *reject_provenance;
    RCLCPP_ERROR(node().get_logger(),
                 "planner backend tracking envelope exceeded: longitudinal=%.3f/%.3f m "
                 "reverse=%.3f/%.3f m lateral=%.3f/%.3f m "
                 "measured_enu=[%.3f,%.3f,%.3f] command_enu=[%.3f,%.3f,%.3f] "
                 "measured_velocity_body_frame=[%.3f,%.3f,%.3f] "
                 "command_velocity_enu=[%.3f,%.3f,%.3f] "
                 "command_acceleration_enu=[%.3f,%.3f,%.3f] "
                 "command_jerk_enu=[%.3f,%.3f,%.3f] "
                 "odom_header_age_ms=%.3f odom_receive_age_ms=%.3f message_id=%lu "
                 "generation=%lu role=%s trajectory_time=%.6f s status=%u "
                 "stamp=%d.%09u previous_message_id=%lu previous_generation=%lu "
                 "previous_trajectory_time=%.6f generation_changed=%d "
                 "generation_delta=%ld previous_valid=%d "
                 "previous_p=[%.3f,%.3f,%.3f] "
                 "previous_v=[%.3f,%.3f,%.3f] command_delta_p=%.6f "
                 "previous_a=[%.3f,%.3f,%.3f] previous_j=[%.3f,%.3f,%.3f] "
                 "command_delta_v=%.6f command_delta_a=%.6f command_delta_j=%.6f",
                 tracking_envelope.longitudinal_error_m,
                 tracking_envelope.longitudinal_limit_m,
                 tracking_envelope.reverse_error_m,
                 navigation_contracts::kCommandAnchorErrorLimitM,
                 tracking_envelope.lateral_error_m,
                 navigation_contracts::kCommandAnchorErrorLimitM,
                 provenance.measured_position.x(), provenance.measured_position.y(),
                 provenance.measured_position.z(),
                 message->position.x, message->position.y, message->position.z,
                 provenance.measured_velocity_body_frame.x(),
                 provenance.measured_velocity_body_frame.y(),
                 provenance.measured_velocity_body_frame.z(),
                 message->velocity.x, message->velocity.y, message->velocity.z,
                 message->acceleration.x, message->acceleration.y,
                 message->acceleration.z,
                 message->jerk.x, message->jerk.y, message->jerk.z,
                 provenance.odometry_header_age_ms, provenance.odometry_receive_age_ms,
                 static_cast<unsigned long>(message->sample_id),
                 static_cast<unsigned long>(message->bundle_generation), role,
                 message->trajectory_time_s,
                 static_cast<unsigned int>(message->status),
                 message->header.stamp.sec, message->header.stamp.nanosec,
                 provenance.previous_valid ? provenance.previous.sample_id : 0U,
                 static_cast<unsigned long>(provenance.previous_valid
                     ? provenance.previous.bundle_generation : 0U),
                 provenance.previous_valid ? provenance.previous.trajectory_time_s : 0.0,
                 provenance.generation_changed ? 1 : 0,
                 static_cast<long>(provenance.generation_delta),
                 provenance.previous_valid ? 1 : 0,
                 provenance.previous_position.x(), provenance.previous_position.y(),
                 provenance.previous_position.z(), provenance.previous_velocity.x(),
                 provenance.previous_velocity.y(), provenance.previous_velocity.z(),
                 provenance.command_delta_position_m,
                 provenance.previous_acceleration.x(), provenance.previous_acceleration.y(),
                 provenance.previous_acceleration.z(), provenance.previous_jerk.x(),
                 provenance.previous_jerk.y(), provenance.previous_jerk.z(),
                 provenance.command_delta_velocity_mps,
                 provenance.command_delta_acceleration_mps2,
                 provenance.command_delta_jerk_mps3);
    safetyStopNavigation("planner backend PVA command anchor is not near vehicle");
    return;
  }
  if (accepted && completed_command && terminal_recovery_needed) {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (!planner_recovery_pending_) {
        const auto now_ns = node().get_clock()->now().nanoseconds();
        const auto deadline = checkedTimestampAdd(now_ns, planner_recovery_wait_timeout_ns_);
        if (!deadline) {
          recovery_deadline_invalid = true;
        } else {
          planner_recovery_pending_ = true;
          planner_recovery_deadline_ns_ = *deadline;
          RCLCPP_WARN(node().get_logger(),
                      "planner backend terminal endpoint has not settled waypoint acceptance; "
                      "holding for bounded planner recovery window %.3f s",
                      planner_recovery_wait_timeout_s_);
        }
      }
    }
    if (mission_controller_) {
      mission_controller_->requestNativeTerminalRecovery(
          node().get_clock()->now().seconds());
    }
  } else if (accepted && !completed_command) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    planner_recovery_pending_ = false;
    planner_recovery_deadline_ns_ = 0;
  } else if (accepted && (terminal_backup_hold_inside_acceptance ||
                          terminal_main_hold_inside_acceptance)) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    planner_recovery_pending_ = false;
    planner_recovery_deadline_ns_ = 0;
  }
  if (accepted && mission_controller_ &&
      (!completed_command || terminal_backup_hold_inside_acceptance ||
       terminal_main_hold_inside_acceptance)) {
    mission_controller_->onNativeTrajectoryReady();
    if (terminal_main_hold_inside_acceptance) {
      mission_controller_->onNativeTerminalHoldObserved();
    }
  }
  if (recovery_deadline_invalid) {
    safetyStopNavigation("planner recovery deadline is not representable");
    return;
  }
  if (accepted && completed_command && mission_controller_) {
    const auto waypoint = mission_controller_->waypointAt(message->waypoint_index);
    const auto state = mission_controller_->state();
    const Eigen::Vector3d measured = completed_command_odometry
        ? Eigen::Vector3d{completed_command_odometry->pose.pose.position.x,
                          completed_command_odometry->pose.pose.position.y,
                          completed_command_odometry->pose.pose.position.z}
        : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    const Eigen::Vector3d command_position{message->position.x, message->position.y,
                                           message->position.z};
    const double measured_speed = completed_command_odometry
        ? Eigen::Vector3d{completed_command_odometry->twist.twist.linear.x,
                          completed_command_odometry->twist.twist.linear.y,
                          completed_command_odometry->twist.twist.linear.z}
              .norm()
        : -1.0;
    RCLCPP_INFO_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "Native completed command accepted: role=%u wp=%u request=%lu state=%u "
        "measured_error_m=%.3f command_error_m=%.3f measured_speed_mps=%.3f "
        "main_hold_inside=%s backup_hold_inside=%s trajectory_ready=%s "
        "terminal_hold_pending=%s",
        static_cast<unsigned>(message->role), static_cast<unsigned>(message->waypoint_index),
        static_cast<unsigned long>(message->request_id), static_cast<unsigned>(state),
        waypoint ? (measured - waypoint->position_enu).norm()
                 : std::numeric_limits<double>::quiet_NaN(),
        waypoint ? (command_position - waypoint->position_enu).norm()
                 : std::numeric_limits<double>::quiet_NaN(),
        measured_speed,
        terminal_main_hold_inside_acceptance ? "true" : "false",
        terminal_backup_hold_inside_acceptance ? "true" : "false",
        mission_controller_->nativeTrajectoryReady() ? "true" : "false",
        mission_controller_->terminalHoldPending() ? "true" : "false");
  }
}

void NavigationMode::updateMission() {
  if (!mission_controller_ || !goal_publisher_) return;
  bool recovery_expired = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (failure_reported_ || mission_terminal_ || handover_requested_) return;
    recovery_expired = plannerRecoveryWaitExpired(
        planner_recovery_pending_, node().get_clock()->now().nanoseconds(),
        planner_recovery_deadline_ns_);
    if (recovery_expired) {
      planner_recovery_pending_ = false;
      planner_recovery_deadline_ns_ = 0;
    }
  }
  if (recovery_expired) {
    safetyStopNavigation("planner backend backup trajectory completed before bounded planner recovery");
    return;
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
  const auto active_waypoint = mission_controller_->activeWaypoint();
  const auto state = mission_controller_->state();
  const auto native_ready = mission_controller_->nativeTrajectoryReady();
  const auto terminal_hold_pending = mission_controller_->terminalHoldPending();
  const double position_error = position.has_value() && active_waypoint.has_value()
                                    ? (*position - active_waypoint->position_enu).norm()
                                    : -1.0;
  const double speed = velocity.has_value() ? velocity->norm() : -1.0;
  RCLCPP_INFO_THROTTLE(
      node().get_logger(), *node().get_clock(), 1000,
      "Mission gate: wp=%zu request=%lu state=%u position_error_m=%.3f radius_m=%.3f "
      "speed_mps=%.3f acceptance_speed_mps=%.3f airborne=%s trajectory_ready=%s "
      "terminal_hold_pending=%s",
      mission_controller_->activeWaypointIndex(),
      static_cast<unsigned long>(mission_controller_->activeRequestId()),
      static_cast<unsigned>(state), position_error,
      active_waypoint.has_value() ? active_waypoint->acceptance_radius_m : -1.0, speed,
      mission_controller_->acceptanceSpeedMps(), airborne ? "true" : "false",
      native_ready ? "true" : "false", terminal_hold_pending ? "true" : "false");
  const auto event = mission_controller_->update(now_s, position, airborne, velocity);
  if (event.waypoint_accepted) {
    RCLCPP_INFO(node().get_logger(),
                "Mission waypoint accepted: wp=%zu position_error_m=%.3f speed_mps=%.3f "
                "next_wp=%zu next_request=%lu",
                event.accepted_waypoint_index, event.acceptance_position_error_m,
                event.acceptance_speed_mps, event.waypoint_index,
                static_cast<unsigned long>(event.request_id));
  }
  handleMissionEvent(event, now_s);
}

void NavigationMode::handleMissionEvent(const MissionControllerEvent& event, double now_s) {
  if (!mission_controller_ || event.type == MissionControllerEvent::Type::None) return;
  if (event.type == MissionControllerEvent::Type::PublishGoal) {
    const auto waypoint = mission_controller_->activeWaypoint();
    if (!waypoint.has_value()) {
      handover_requested_ = true;
      return;
    }
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      // Waypoint acceptance and planner publication run on independent
      // callbacks. Preserve the exact old certified command under its old
      // identity until a new command is committed atomically. Never relabel it
      // here; normal validity/freshness expiry remains fail-closed.
      if (navigation_command_.has_value()) {
        ++waypoint_handoff_retained_command_count_;
      }
      navigation_command_ = transitionCertifiedCommand(
          navigation_command_, std::nullopt, CertifiedCommandTransition::kRetain);
    }
    navigation_contracts::msg::NavigationGoal goal;
    goal.header.frame_id = planning_frame_;
    const auto time = node().get_clock()->now();
    goal.header.stamp = time;
    goal.mission_id = mission_->id;
    goal.waypoint_index = static_cast<std::uint32_t>(event.waypoint_index);
    goal.request_id = event.request_id;
    goal.target.x = waypoint->position_enu.x();
    goal.target.y = waypoint->position_enu.y();
    goal.target.z = waypoint->position_enu.z();
    goal.acceptance_radius_m = waypoint->acceptance_radius_m;
    goal.behavior = waypoint->behavior == MissionWaypoint::Behavior::Stop
                        ? navigation_contracts::msg::NavigationGoal::BEHAVIOR_STOP
                        : navigation_contracts::msg::NavigationGoal::BEHAVIOR_PASS_THROUGH;
    const auto next_waypoint = mission_controller_->nextWaypoint();
    if (next_waypoint.has_value()) {
      goal.has_next_target = true;
      goal.next_target.x = next_waypoint->position_enu.x();
      goal.next_target.y = next_waypoint->position_enu.y();
      goal.next_target.z = next_waypoint->position_enu.z();
    }
    const auto route = mission_controller_->routeSnapshot();
    if (!route.valid() || route.request_id != event.request_id ||
        route.active_waypoint_index != event.waypoint_index) {
      RCLCPP_ERROR(node().get_logger(),
                   "Refusing to publish goal without matching immutable route snapshot");
      handover_requested_ = true;
      publishStatus(navigation_contracts::msg::NavigationModeStatus::PAUSED,
                    navigation_contracts::msg::NavigationModeStatus::SAFETY_STOP,
                    &event);
      return;
    }
    goal.route.mission_id = route.mission_id;
    goal.route.frame_id = route.frame;
    goal.route.route_revision = route.route_revision;
    goal.route.request_id = route.request_id;
    goal.route.active_waypoint_index =
        static_cast<std::uint32_t>(route.active_waypoint_index);
    goal.route.measured_progress_valid = route.measured_progress.valid;
    goal.route.measured_segment_index = static_cast<std::uint32_t>(
        route.segments.empty() ? 0U
                               : route.measured_progress.projection.segment_index);
    goal.route.measured_progress_arc_m = route.measured_progress.progress_arc_m;
    goal.route.measured_projection_arc_m =
        route.measured_progress.projection.arc_length_m;
    goal.route.measured_lateral_error_m =
        route.measured_progress.projection.lateral_error_m;
    goal.route.waypoint_positions.reserve(route.waypoints.size());
    goal.route.waypoint_ids.reserve(route.waypoints.size());
    goal.route.waypoint_acceptance_radii_m.reserve(route.waypoints.size());
    goal.route.waypoint_behaviors.reserve(route.waypoints.size());
    for (const auto& route_waypoint : route.waypoints) {
      geometry_msgs::msg::Point point;
      point.x = route_waypoint.position_enu.x();
      point.y = route_waypoint.position_enu.y();
      point.z = route_waypoint.position_enu.z();
      goal.route.waypoint_positions.push_back(point);
      goal.route.waypoint_ids.push_back(route_waypoint.id);
      goal.route.waypoint_acceptance_radii_m.push_back(
          route_waypoint.acceptance_radius_m);
      goal.route.waypoint_behaviors.push_back(
          route_waypoint.behavior == MissionWaypoint::Behavior::Stop
              ? navigation_contracts::msg::RouteSnapshot::BEHAVIOR_STOP
              : navigation_contracts::msg::RouteSnapshot::BEHAVIOR_PASS_THROUGH);
    }
    goal_publisher_->publish(goal);
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      last_goal_publish_ns_ = time.nanoseconds();
    }
    publishStatus(navigation_contracts::msg::NavigationModeStatus::ACTIVE,
                  navigation_contracts::msg::NavigationModeStatus::NONE, &event);
    if (next_waypoint.has_value()) {
      RCLCPP_INFO(node().get_logger(),
                  "Published mission waypoint %zu (%s) behavior=%u next_target=(%.3f,%.3f,%.3f)",
                  event.waypoint_index, waypoint->id.c_str(),
                  static_cast<unsigned>(goal.behavior), next_waypoint->position_enu.x(),
                  next_waypoint->position_enu.y(), next_waypoint->position_enu.z());
    } else {
      RCLCPP_INFO(node().get_logger(),
                  "Published mission waypoint %zu (%s) behavior=%u terminal=true",
                  event.waypoint_index, waypoint->id.c_str(),
                  static_cast<unsigned>(goal.behavior));
    }
    return;
  }
  if (event.type == MissionControllerEvent::Type::Complete) {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      mission_terminal_ = true;
      handover_requested_ = true;
      last_completed_waypoint_index_ = static_cast<std::uint32_t>(event.waypoint_index);
      last_completed_request_id_ = event.request_id;
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
    publishStatus(navigation_contracts::msg::NavigationModeStatus::COMPLETE,
                  navigation_contracts::msg::NavigationModeStatus::NONE, &event);
    completed(px4_ros2::Result::Success);
    return;
  }
  if (event.type == MissionControllerEvent::Type::RequestPositionControl) {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      handover_requested_ = true;
    }
    publishStatus(navigation_contracts::msg::NavigationModeStatus::PAUSED,
                  navigation_contracts::msg::NavigationModeStatus::SAFETY_STOP);
    RCLCPP_WARN(node().get_logger(),
                "Safety stop completed at waypoint %zu; handing over to PX4 Hold",
                event.waypoint_index);
    if (px4_hold_handover_) {
      px4_hold_handover_();
    } else {
      failNavigation("PX4 Hold handover callback is unavailable");
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

void NavigationMode::onOdometry(
    const navigation_contracts::msg::PropagatedOdometry::ConstSharedPtr& message) {
  if (!message || message->localization_epoch == 0U || message->sequence == 0U) return;
  const auto& odometry = message->odometry;
  const auto& position = odometry.pose.pose.position;
  const auto& velocity = odometry.twist.twist.linear;
  if (odometry.header.frame_id != planning_frame_ ||
      odometry.child_frame_id != body_frame_ || odometry.header.stamp.sec < 0 ||
      !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z) ||
      !isNormalizableOdometryQuaternion(Eigen::Quaterniond(
          odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.x,
          odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z))) {
    RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000,
                         "Rejecting navigation odometry with invalid frame or values");
    return;
  }
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (!typed_health_seen_ || !lio_health_valid_ ||
      message->localization_epoch != lio_localization_epoch_ ||
      (last_propagated_state_sequence_ > 0U &&
       message->sequence <= last_propagated_state_sequence_)) {
    return;
  }
  const auto receive_ns = node().get_clock()->now().nanoseconds();
  const auto source_stamp_ns = navigation_common::rosTimeToNanoseconds(
      odometry.header.stamp).value_or(0);
  if (source_stamp_ns <= 0 ||
      (last_propagated_state_stamp_ns_ > 0 &&
       source_stamp_ns <= last_propagated_state_stamp_ns_)) {
    RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000,
                         "Rejecting non-increasing propagated odometry source timestamp");
    return;
  }
  if (last_odometry_receive_ns_ > 0 && receive_ns >= last_odometry_receive_ns_) {
    maximum_odometry_callback_gap_us_ = std::max(
        maximum_odometry_callback_gap_us_,
        (receive_ns - last_odometry_receive_ns_) / 1000);
  }
  last_odometry_receive_ns_ = receive_ns;
  last_odometry_receive_steady_ns_ = navigation_common::steadyClockNowNanoseconds();
  last_propagated_state_stamp_ns_ = source_stamp_ns;
  last_propagated_state_sequence_ = message->sequence;
  ++odometry_callback_count_;
  odometry_ = odometry;
}

void NavigationMode::onEstimatorHealth(
    const navigation_contracts::msg::EstimatorHealth::ConstSharedPtr& message) {
  if (!message || message->localization_epoch == 0U) return;
  const auto source_stamp_ns = navigation_common::rosTimeToNanoseconds(
      message->header.stamp).value_or(0);
  const bool healthy =
      message->state == navigation_contracts::msg::EstimatorHealth::TRACKING &&
      message->navigation_valid && message->covariance_valid &&
      message->observability_valid && message->correction_fresh &&
      message->propagation_valid;
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  typed_health_seen_ = true;
  last_health_state_ = message->state;
  last_health_navigation_valid_ = message->navigation_valid;
  last_health_covariance_valid_ = message->covariance_valid;
  last_health_observability_valid_ = message->observability_valid;
  last_health_correction_fresh_ = message->correction_fresh;
  last_health_propagation_valid_ = message->propagation_valid;
  last_health_source_stamp_ns_ = source_stamp_ns;
  if (source_stamp_ns <= 0 ||
      (last_lio_diagnostics_ns_ > 0 && source_stamp_ns <= last_lio_diagnostics_ns_)) {
    lio_health_valid_ = false;
    return;
  }
  if (lio_localization_epoch_ != 0U &&
      lio_localization_epoch_ != message->localization_epoch) {
    // Invalidate command exposure immediately when typed health announces a
    // new public frame; NavigationCommand carries the epoch and is checked at
    // the command boundary below.
    navigation_command_ = transitionCertifiedCommand(
        navigation_command_, std::nullopt, CertifiedCommandTransition::kInvalidate);
    last_command_receive_ns_ = 0;
    odometry_.reset();
    last_odometry_receive_ns_ = 0;
    last_odometry_receive_steady_ns_ = 0;
    last_propagated_state_stamp_ns_ = 0;
    last_propagated_state_sequence_ = 0U;
  }
  lio_localization_epoch_ = message->localization_epoch;
  lio_health_valid_ = healthy;
  last_lio_diagnostics_ns_ = source_stamp_ns;
}

void NavigationMode::logRuntimeMetrics(const rclcpp::Time& now) {
  const auto now_ns = now.nanoseconds();
  std::uint64_t odometry_callbacks;
  std::uint64_t trajectories_received;
  std::uint64_t trajectories_accepted;
  std::uint64_t trajectories_rejected;
  std::uint64_t waypoint_handoffs_retaining_command;
  std::uint64_t setpoint_updates;
  std::uint64_t stale_state_failures;
  std::int64_t odometry_gap_us;
  std::int64_t setpoint_gap_us;
  double state_age_s;
  Eigen::Vector3d velocity_command_enu;
  std::uint64_t forward_guard_count;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!runtimeMetricsLogDue(last_metrics_log_ns_, now_ns)) {
      return;
    }
    last_metrics_log_ns_ = now_ns;
    odometry_callbacks = odometry_callback_count_;
    trajectories_received = trajectory_received_count_;
    trajectories_accepted = trajectory_accepted_count_;
    trajectories_rejected = trajectory_rejected_count_;
    waypoint_handoffs_retaining_command = waypoint_handoff_retained_command_count_;
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
              "waypoint_handoffs_retaining_command=%lu "
              "setpoint_updates=%lu setpoint_max_gap_us=%ld last_state_age_s=%.6f "
              "stale_state_failures=%lu velocity_command_enu=(%.3f,%.3f,%.3f) "
              "forward_guard_count=%lu",
              static_cast<unsigned long>(odometry_callbacks),
              static_cast<long>(odometry_gap_us),
              static_cast<unsigned long>(trajectories_received),
              static_cast<unsigned long>(trajectories_accepted),
              static_cast<unsigned long>(trajectories_rejected),
              static_cast<unsigned long>(waypoint_handoffs_retaining_command),
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
    navigation_command_ = transitionCertifiedCommand(
        navigation_command_, std::nullopt, CertifiedCommandTransition::kInvalidate);
  }
  if (mission_controller_) mission_controller_->deactivate();
  publishStatus(navigation_contracts::msg::NavigationModeStatus::PAUSED,
                navigation_contracts::msg::NavigationModeStatus::SAFETY_STOP);
  RCLCPP_ERROR(node().get_logger(), "%s; safety hold then handover to PX4 Hold", reason);
  if (px4_hold_handover_) {
    px4_hold_handover_();
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
    navigation_command_ = transitionCertifiedCommand(
        navigation_command_, std::nullopt, CertifiedCommandTransition::kInvalidate);
  }
  if (mission_controller_) mission_controller_->deactivate();
  const auto status_reason = std::string_view(reason).find("odometry") != std::string_view::npos
                                 ? navigation_contracts::msg::NavigationModeStatus::ODOMETRY_STALE
                                 : navigation_contracts::msg::NavigationModeStatus::TRAJECTORY_INVALID;
  publishStatus(navigation_contracts::msg::NavigationModeStatus::FAILED, status_reason);
  RCLCPP_ERROR(node().get_logger(), "%s; handing over to PX4 Hold", reason);
  if (px4_hold_handover_) {
    px4_hold_handover_();
  } else {
    completed(px4_ros2::Result::ModeFailureOther);
  }
}

void NavigationMode::updateSetpoint(float /*dt_s*/) {
  std::optional<navigation_contracts::msg::NavigationCommand> navigation_command;
  std::optional<nav_msgs::msg::Odometry> odometry;
  std::int64_t odometry_receive_steady_ns = 0;
  const auto now = node().get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    navigation_command = navigation_command_;
    odometry = odometry_;
    odometry_receive_steady_ns = last_odometry_receive_steady_ns_;
  }
  bool stationary_position_unrepresentable = false;
  const auto publishStationary = [&](const std::optional<Eigen::Vector3d>& position_enu) {
    px4_ros2::TrajectorySetpoint setpoint;
    if (!position_enu.has_value()) {
      setpoint.withVelocity(Eigen::Vector3f::Zero());
    } else {
      if (const auto position_ned = checkedEnuToNed(*position_enu)) {
        setpoint.withPosition(*position_ned);
        setpoint.withAcceleration(Eigen::Vector3f::Zero());
      } else {
        stationary_position_unrepresentable = true;
      }
      setpoint.withVelocity(Eigen::Vector3f::Zero());
    }
    if (odometry.has_value()) {
      const auto& q = odometry->pose.pose.orientation;
      const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
      if (orientation.coeffs().allFinite() && std::isfinite(orientation.squaredNorm()) &&
          orientation.squaredNorm() > 1.0e-12) {
        const Eigen::Quaterniond normalized = orientation.normalized();
        const double yaw_enu = std::atan2(
            2.0 * (normalized.w() * normalized.z() + normalized.x() * normalized.y()),
            1.0 - 2.0 * (normalized.y() * normalized.y() + normalized.z() * normalized.z()));
        if (floatRepresentable(yaw_enu)) {
          setpoint.withYaw(px4_ros2::yawEnuToNed(static_cast<float>(yaw_enu)))
              .withYawRate(0.0F);
        }
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
    const auto position_ned = checkedEnuToNed(position_enu);
    if (!position_ned) return false;
    setpoint.withPosition(*position_ned);
    setpoint.withVelocity(Eigen::Vector3f::Zero());
    if (odometry.has_value()) {
      const auto& q = odometry->pose.pose.orientation;
      const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
      if (orientation.coeffs().allFinite() && std::isfinite(orientation.squaredNorm()) &&
          orientation.squaredNorm() > 1.0e-12) {
        const Eigen::Quaterniond normalized = orientation.normalized();
        const double yaw_enu = std::atan2(
            2.0 * (normalized.w() * normalized.z() + normalized.x() * normalized.y()),
            1.0 - 2.0 * (normalized.y() * normalized.y() + normalized.z() * normalized.z()));
        if (floatRepresentable(yaw_enu)) {
          setpoint.withYaw(px4_ros2::yawEnuToNed(static_cast<float>(yaw_enu)))
              .withYawRate(0.0F);
        }
      }
    }
    trajectory_setpoint_->update(setpoint);
    return true;
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
  bool terminal_stationary_setpoint = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (failure_reported_) {
      // Keep the PX4 setpoint stream valid and stationary while the mode
      // executor performs the handover after a terminal navigation failure.
      publishStationary(safety_hold_position_);
      last_setpoint_time_ = now;
      terminal_stationary_setpoint = true;
    }
    if (!terminal_stationary_setpoint && mission_terminal_) {
      publishStationary(completion_position_);
      last_setpoint_time_ = now;
      terminal_stationary_setpoint = true;
    }
    if (!terminal_stationary_setpoint && handover_requested_) {
      std::optional<Eigen::Vector3d> handover_position;
      if (safety_hold_position_.has_value()) {
        handover_position = safety_hold_position_;
      } else if (mission_controller_) {
        const auto waypoint = mission_controller_->activeWaypoint();
        if (waypoint.has_value()) handover_position = waypoint->position_enu;
      }
      publishStationary(handover_position);
      last_setpoint_time_ = now;
      terminal_stationary_setpoint = true;
    }
  }
  if (terminal_stationary_setpoint) {
    if (stationary_position_unrepresentable) {
      safetyStopNavigation("terminal hold position is not representable by PX4");
    }
    return;
  }
  const double since_activation_s = (now - activation_time_).seconds();
  if (mission_controller_ && mission_controller_->waitingForAirborne()) {
    publishStationary(std::nullopt);
    return;
  }
  {
    const auto odometry_source_ns = odometry
        ? navigation_common::rosTimeToNanoseconds(odometry->header.stamp).value_or(0) : 0;
    const auto odometry_freshness = navigation_contracts::evaluateExecutionStateFreshness(
        now.nanoseconds(), odometry_source_ns,
        navigation_common::steadyClockNowNanoseconds(),
        odometry_receive_steady_ns, state_stale_after_s_);
    if (!odometry_freshness.valid()) {
      {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (!failure_reported_) ++stale_state_failure_count_;
      }
      RCLCPP_ERROR(node().get_logger(),
                   "Navigation odometry lease failed before setpoint update: reason=%s "
                   "source_age_ms=%.3f receive_age_ms=%.3f",
                   navigation_contracts::executionStateFreshnessReasonName(
                       odometry_freshness.reason),
                   odometry_freshness.source_age_ms, odometry_freshness.receive_age_ms);
      failNavigation("navigation odometry stale before setpoint update");
      return;
    }
  }
  if (mission_controller_ && mission_controller_->holding()) {
    const auto waypoint = mission_controller_->activeWaypoint();
    if (!waypoint.has_value()) {
      safetyStopNavigation("mission hold has no active waypoint");
      return;
    }
    if (!publishPositionHold(waypoint->position_enu)) {
      safetyStopNavigation("mission hold position is not representable by PX4");
    }
    return;
  }

  bool lio_healthy = false;
  bool typed_health_seen = false;
  std::int64_t lio_diagnostics_age_ns = std::numeric_limits<std::int64_t>::max();
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    lio_healthy = lio_health_valid_;
    typed_health_seen = typed_health_seen_;
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
  if (diagnostics_missing && !typed_health_seen && since_activation_s <= diagnostics_wait_s) {
    publishStationary(std::nullopt);
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    last_setpoint_time_ = now;
    return;
  }
  if ((!diagnostics_missing && !lio_healthy) ||
      lio_diagnostics_age_ns > state_stale_after_ns_ ||
      (diagnostics_missing &&
       (typed_health_seen || since_activation_s > diagnostics_wait_s))) {
    std::uint8_t health_state = 0U;
    bool health_navigation_valid = false;
    bool health_covariance_valid = false;
    bool health_observability_valid = false;
    bool health_correction_fresh = false;
    bool health_propagation_valid = false;
    std::int64_t health_source_stamp_ns = 0;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      health_state = last_health_state_;
      health_navigation_valid = last_health_navigation_valid_;
      health_covariance_valid = last_health_covariance_valid_;
      health_observability_valid = last_health_observability_valid_;
      health_correction_fresh = last_health_correction_fresh_;
      health_propagation_valid = last_health_propagation_valid_;
      health_source_stamp_ns = last_health_source_stamp_ns_;
    }
    RCLCPP_ERROR(node().get_logger(),
                 "FAST-LIO health gate details: typed_seen=%s healthy=%s "
                 "diagnostics_missing=%s diagnostics_age_ms=%.3f state=%u "
                 "navigation_valid=%s covariance_valid=%s observability_valid=%s "
                 "correction_fresh=%s propagation_valid=%s health_stamp_ns=%ld",
                 typed_health_seen ? "true" : "false", lio_healthy ? "true" : "false",
                 diagnostics_missing ? "true" : "false",
                 diagnostics_missing
                     ? -1.0
                     : static_cast<double>(lio_diagnostics_age_ns) / 1e6,
                 static_cast<unsigned>(health_state),
                 health_navigation_valid ? "true" : "false",
                 health_covariance_valid ? "true" : "false",
                 health_observability_valid ? "true" : "false",
                 health_correction_fresh ? "true" : "false",
                 health_propagation_valid ? "true" : "false",
                 static_cast<long>(health_source_stamp_ns));
    failNavigation("FAST-LIO navigation health invalid or stale");
    return;
  }

  // Native planner backend command path.  The planner FSM already evaluated the
  // polynomial and selected main versus backup trajectory.  PX4 must receive
  // that PVA state directly; applying a second velocity controller here would
  // change planner backend's trajectory and reintroduce the old terminal oscillation.
  if (navigation_command.has_value()) {
    const auto receive_ns = [&]() {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      return last_command_receive_ns_;
    }();
    if (receive_ns > 0 && now.nanoseconds() >= receive_ns &&
        now.nanoseconds() - receive_ns > stale_after_ns_) {
      safetyStopNavigation("planner backend PVA command stale");
      return;
    }
    const auto& command = *navigation_command;
    const auto command_stamp_ns =
        navigation_common::rosTimeToNanoseconds(command.header.stamp).value_or(0);
    if (command_stamp_ns <= 0 ||
        (now.nanoseconds() >= command_stamp_ns &&
         now.nanoseconds() - command_stamp_ns > stale_after_ns_) ||
        (command_stamp_ns > now.nanoseconds() &&
         command_stamp_ns - now.nanoseconds() > stale_after_ns_)) {
      safetyStopNavigation("planner backend PVA command timestamp invalid or stale");
      return;
    }
    if (!navigation_contracts::commandValidAt(command, now.nanoseconds())) {
      safetyStopNavigation("planner backend command validity window expired");
      return;
    }
    const Eigen::Vector3d position_enu{command.position.x, command.position.y,
                                       command.position.z};
    const Eigen::Vector3d velocity_enu{command.velocity.x, command.velocity.y,
                                       command.velocity.z};
    const Eigen::Vector3d acceleration_enu{command.acceleration.x, command.acceleration.y,
                                            command.acceleration.z};
    bool terminal_hold_inside_acceptance = false;
    if (command.status ==
            navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED &&
        (command.role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP ||
         command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN) &&
        !mission_terminal_ && mission_controller_ && odometry.has_value()) {
      const auto waypoint = mission_controller_->activeWaypoint();
      if (!waypoint.has_value()) {
        safetyStopNavigation("completed command has no active waypoint");
        return;
      }
      const auto& point = odometry->pose.pose.position;
      const Eigen::Vector3d measured{point.x, point.y, point.z};
      const Eigen::Vector3d command_position{command.position.x, command.position.y,
                                             command.position.z};
      const bool command_inside_acceptance =
          command_position.allFinite() &&
          (command_position - waypoint->position_enu).norm() <=
              waypoint->acceptance_radius_m;
      const bool measured_inside_acceptance =
          measured.allFinite() &&
          (measured - waypoint->position_enu).norm() <=
              waypoint->acceptance_radius_m;
      if (command.role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP) {
        terminal_hold_inside_acceptance = measured_inside_acceptance &&
            backupEndpointHoldIsAnchored(
                command_inside_acceptance, measured.allFinite(),
                command_position.allFinite(), (measured - command_position).norm(),
                navigation_contracts::kCommandAnchorErrorLimitM);
      } else {
        terminal_hold_inside_acceptance =
            measured_inside_acceptance && command_inside_acceptance;
      }
    }
    if (command.status ==
        navigation_contracts::msg::NavigationCommand::STATUS_REJECTED) {
      safetyStopNavigation("planner backend planner failed without a valid backup trajectory");
      return;
    }
    if (command.status ==
        navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED) {
      if (!publishPositionHold(position_enu)) {
        safetyStopNavigation("completed command position is not representable by PX4");
        return;
      }
      if ((command.role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP ||
           command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN) &&
          !terminal_hold_inside_acceptance && !mission_terminal_) {
        // The command publisher and PX4 setpoint callback are independent
        // executor paths. A replacement PlanFromRest command can therefore
        // arrive immediately after this completed sample. Keep publishing
        // the exact endpoint hold for one bounded recovery window; the mission
        // timer performs the fail-closed handover if no replacement arrives.
        bool recovery_deadline_invalid = false;
        {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          if (!planner_recovery_pending_) {
            const auto deadline =
                checkedTimestampAdd(now.nanoseconds(), planner_recovery_wait_timeout_ns_);
            if (!deadline) {
              recovery_deadline_invalid = true;
            } else {
              planner_recovery_pending_ = true;
              planner_recovery_deadline_ns_ = *deadline;
              RCLCPP_WARN(node().get_logger(),
                          "planner backend terminal endpoint reached; holding for bounded "
                          "planner recovery window %.3f s",
                          planner_recovery_wait_timeout_s_);
            }
          }
        }
        if (recovery_deadline_invalid) {
          safetyStopNavigation("planner recovery deadline is not representable");
        }
        return;
      }
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      last_setpoint_time_ = now;
      return;
    }
    const auto position_ned = checkedEnuToNed(position_enu);
    const auto velocity_ned = checkedEnuToNed(velocity_enu);
    const auto acceleration_ned = checkedEnuToNed(acceleration_enu);
    if (!position_ned || !velocity_ned || !acceleration_ned ||
        !floatRepresentable(command.yaw) || !floatRepresentable(command.yaw_rate)) {
      safetyStopNavigation("planner backend PVA command is not representable by PX4");
      return;
    }
    px4_ros2::TrajectorySetpoint setpoint;
    setpoint.withPosition(*position_ned)
        .withVelocity(*velocity_ned)
        .withAcceleration(*acceleration_ned)
        .withYaw(px4_ros2::yawEnuToNed(static_cast<float>(command.yaw)))
        .withYawRate(px4_ros2::yawRateEnuToNed(static_cast<float>(command.yaw_rate)));
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
  // not hand over to PX4 Hold on the same 50 Hz tick that starts planning. Once
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
  safetyStopNavigation("planner backend PVA command unavailable");
}

NavigationModeExecutor::NavigationModeExecutor(px4_ros2::ModeBase& owned_mode)
    : ModeExecutorBase(px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
      node_(owned_mode.node()),
      navigation_mode_(dynamic_cast<NavigationMode&>(owned_mode)) {
  navigation_mode_.setPx4HoldHandover([this]() {
    RCLCPP_WARN(node_.get_logger(), "Avoidance Mission requesting PX4 Hold handover");
    schedulePx4Hold(true);
  });
}

void NavigationModeExecutor::onActivate() {
  RCLCPP_INFO(node_.get_logger(), "Avoidance Mission executor activated");
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
    RCLCPP_ERROR(node_.get_logger(), "Avoidance Mission completed with result=%s; handing over to PX4 Hold",
                 px4_ros2::resultToString(result));
    schedulePx4Hold(false);
    return;
  }
  RCLCPP_INFO(node_.get_logger(), "Avoidance Mission completed; handing over to PX4 Hold");
  schedulePx4Hold(false);
}

void NavigationModeExecutor::schedulePx4Hold(bool complete_navigation_failure) {
  scheduleMode(px4_ros2::ModeBase::kModeIDLoiter,
               [this, complete_navigation_failure](px4_ros2::Result hold_result) {
    onPx4HoldHandoverCompleted(hold_result, complete_navigation_failure);
  });
}

void NavigationModeExecutor::onPx4HoldHandoverCompleted(
    px4_ros2::Result result, bool complete_navigation_failure) {
  if (result == px4_ros2::Result::Success || result == px4_ros2::Result::Deactivated) {
    RCLCPP_INFO(node_.get_logger(), "PX4 Hold handover completed with result=%s",
                px4_ros2::resultToString(result));
    return;
  }

  RCLCPP_ERROR(node_.get_logger(),
               "PX4 Hold handover failed with result=%s; navigation cannot continue",
               px4_ros2::resultToString(result));
  if (complete_navigation_failure) {
    // scheduleMode() invokes this callback synchronously for Rejected/Timeout.
    // Without a terminal completion the executor keeps the external mode alive
    // while NavigationMode publishes a stationary setpoint indefinitely.
    ownedMode().completed(px4_ros2::Result::ModeFailureOther);
  }
}

void NavigationModeExecutor::onDeactivate(DeactivateReason reason) {
  RCLCPP_INFO(node_.get_logger(), "Avoidance Mission executor deactivated (%s)",
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
