#include "px4_navigation_external_mode/navigation_mode.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <navigation_common/frame_conventions.hpp>
#include <navigation_common/time.hpp>
#include <navigation_contracts/command_safety_contract.hpp>
#include <navigation_contracts/navigation_command_contract.hpp>
#include <navigation_contracts/execution_state_freshness.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "px4_navigation_external_mode/tracking_envelope.hpp"
#include "px4_navigation_external_mode/navigation_input_validation.hpp"
#include "px4_navigation_external_mode/certified_command_handoff.hpp"
#include "px4_navigation_external_mode/reject_provenance.hpp"
#include "px4_navigation_external_mode/command_acceptance_gate.hpp"
#include "px4_navigation_external_mode/command_admission_assessment.hpp"
#include "px4_navigation_external_mode/planner_recovery.hpp"
#include "px4_navigation_external_mode/runtime_metrics_policy.hpp"
#include "px4_navigation_external_mode/local_frame_alignment.hpp"
#include "px4_navigation_external_mode/paired_node_lifetime.hpp"

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

void logTrackingRejection(
    const rclcpp::Logger& logger,
    const navigation_contracts::msg::NavigationCommand& command,
    const TrackingEnvelopeResult& tracking_envelope,
    const RejectProvenance& provenance) {
  const char* role = "UNKNOWN";
  if (command.role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP) {
    role = "BACKUP";
  } else if (command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN) {
    role = "MAIN";
  } else if (command.role ==
             navigation_contracts::msg::NavigationCommand::ROLE_EMERGENCY) {
    role = "EMERGENCY";
  }
  RCLCPP_ERROR(logger,
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
               tracking_envelope.reverse_limit_m,
               tracking_envelope.lateral_error_m,
               tracking_envelope.lateral_limit_m,
               provenance.measured_position.x(), provenance.measured_position.y(),
               provenance.measured_position.z(),
               command.position.x, command.position.y, command.position.z,
               provenance.measured_velocity_body_frame.x(),
               provenance.measured_velocity_body_frame.y(),
               provenance.measured_velocity_body_frame.z(),
               command.velocity.x, command.velocity.y, command.velocity.z,
               command.acceleration.x, command.acceleration.y,
               command.acceleration.z,
               command.jerk.x, command.jerk.y, command.jerk.z,
               provenance.odometry_header_age_ms, provenance.odometry_receive_age_ms,
               static_cast<unsigned long>(command.sample_id),
               static_cast<unsigned long>(command.bundle_generation), role,
               command.trajectory_time_s,
               static_cast<unsigned int>(command.status),
               command.header.stamp.sec, command.header.stamp.nanosec,
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
}

}  // namespace

void NavigationMode::clearPlannerRecoveryEpisodeLocked() noexcept {
  planner_recovery_pending_ = false;
  planner_recovery_deadline_ns_ = 0;
  planner_recovery_mission_id_.clear();
  planner_recovery_waypoint_index_ = 0U;
  planner_recovery_request_id_ = 0U;
  planner_recovery_bundle_generation_ = 0U;
}

void NavigationMode::rememberPlannerRecoveryEpisodeLocked(
    const navigation_contracts::msg::NavigationCommand& command) {
  planner_recovery_mission_id_ = command.mission_id;
  planner_recovery_waypoint_index_ = command.waypoint_index;
  planner_recovery_request_id_ = command.request_id;
  planner_recovery_bundle_generation_ = command.bundle_generation;
}

bool NavigationMode::plannerRecoveryEpisodeMatchesLocked(
    const navigation_contracts::msg::NavigationCommand& command) const noexcept {
  return planner_recovery_pending_ &&
         command.mission_id == planner_recovery_mission_id_ &&
         command.waypoint_index == planner_recovery_waypoint_index_ &&
         command.request_id == planner_recovery_request_id_ &&
         command.bundle_generation == planner_recovery_bundle_generation_;
}

NavigationMode::NavigationMode(rclcpp::Node& node)
    : ModeBase(node, Settings{kModeName}),
      node_(node),
      trajectory_setpoint_(std::make_shared<px4_ros2::TrajectorySetpointType>(*this)),
      navigation_command_topic_(node.declare_parameter<std::string>(
          "navigation.navigation_command_topic", "/navigation/navigation_command")),
      state_topic_(node.declare_parameter<std::string>(
          "navigation.state_topic", "/lio/odometry_propagated")),
      planning_frame_(node.declare_parameter<std::string>(
          "navigation.planning_frame", "lio_odom")),
      body_frame_(node.declare_parameter<std::string>(
          "navigation.body_frame", "base_link")),
      stale_after_s_(node.declare_parameter<double>(
          "navigation.trajectory_stale_after_s", 0.10)),
      state_stale_after_s_(node.declare_parameter<double>(
          "navigation.state_stale_after_s", 0.20)),
      trajectory_wait_timeout_s_(node.declare_parameter<double>(
          "navigation.trajectory_wait_timeout_s", 5.0)),
      planner_recovery_wait_timeout_s_(node.declare_parameter<double>(
          "navigation.planner_recovery_wait_timeout_s", 5.0)) {
  tracking_experiment_ = navigation_contracts::loadTrackingExperimentPolicy(node);
  const bool state_transport_trace_enabled = node.declare_parameter<bool>(
      "diagnostics.state_transport_trace_enabled", false);
  if (state_transport_trace_enabled) {
    if (!node.get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("state transport trace is SITL/test only");
    }
    odometry_timing_publisher_ = node.create_publisher<
        navigation_contracts::msg::OdometryTransportTrace>(
        "/navigation/odometry_ingress_trace", rclcpp::QoS(256).best_effort());
  }
  RCLCPP_INFO(node.get_logger(),
      "RUNTIME_CONFIG_EFFECTIVE tracking_mode=%s enabled=%d suppress_braking=%d "
      "suppress_health=%d velocity_only=%d",
      node.get_parameter("tracking_experiment.mode").as_string().c_str(),
      tracking_experiment_.enabled, tracking_experiment_.suppress_braking,
      tracking_experiment_.suppress_estimator_health_response,
      tracking_experiment_.velocity_only_enabled);
  RCLCPP_INFO(node.get_logger(),
      "RUNTIME_CONFIG_EFFECTIVE tracking_bounds base=%.17g alpha=%.17g beta=%.17g "
      "velocity_gain=%.17g velocity_cap=%.17g velocity_accel=%.17g "
      "velocity_jerk=%.17g velocity_timing=%.17g velocity_reference_age=%.17g "
      "velocity_transport=%.17g velocity_px4_consume=%.17g",
      tracking_experiment_.base_m, tracking_experiment_.lateral_alpha_s,
      tracking_experiment_.longitudinal_beta_s,
      tracking_experiment_.velocity_only_gain_s_inv,
      tracking_experiment_.velocity_only_cap_mps,
      tracking_experiment_.velocity_only_max_acceleration_mps2,
      tracking_experiment_.velocity_only_max_jerk_mps3,
      tracking_experiment_.velocity_only_max_timing_bound_s,
      tracking_experiment_.velocity_only_max_reference_age_s,
      tracking_experiment_.velocity_only_output_transport_bound_s,
      tracking_experiment_.velocity_only_px4_consume_bound_s);
  if (tracking_experiment_.enabled) {
    RCLCPP_WARN(node.get_logger(),
        "SITL TRACKING EXPERIMENT: increased collision risk; base=%.3fm alpha=%.3fs beta=%.3fs "
        "suppress_geometric_tracking=%d suppress_fresh_typed_health_response=%d; "
        "not flight qualification",
        tracking_experiment_.base_m, tracking_experiment_.lateral_alpha_s,
        tracking_experiment_.longitudinal_beta_s, tracking_experiment_.suppress_braking,
        tracking_experiment_.suppress_estimator_health_response);
  }
  if (tracking_experiment_.velocity_only_enabled) {
    RCLCPP_WARN(node.get_logger(),
        "SITL VELOCITY-ONLY EXPERIMENT: LIO-owned path error, PX4 velocity/yaw boundary; "
        "gain=%.3f cap=%.3f accel=%.3f jerk=%.3f timing=%.3fs; not flight qualification",
        tracking_experiment_.velocity_only_gain_s_inv,
        tracking_experiment_.velocity_only_cap_mps,
        tracking_experiment_.velocity_only_max_acceleration_mps2,
        tracking_experiment_.velocity_only_max_jerk_mps3,
        tracking_experiment_.velocity_only_max_timing_bound_s);
  }
  const auto stale_after_ns = navigation_common::secondsToNanoseconds(stale_after_s_);
  const auto state_stale_after_ns = navigation_common::secondsToNanoseconds(state_stale_after_s_);
  const auto planner_recovery_wait_timeout_ns =
      navigation_common::secondsToNanoseconds(planner_recovery_wait_timeout_s_);
  if (navigation_command_topic_.empty() || state_topic_.empty() ||
      planning_frame_.empty() ||
      body_frame_.empty() ||
      !std::isfinite(stale_after_s_) || stale_after_s_ <= 0.0 ||
      !std::isfinite(state_stale_after_s_) || state_stale_after_s_ <= 0.0 ||
      !std::isfinite(trajectory_wait_timeout_s_) || trajectory_wait_timeout_s_ <= 0.0 ||
      !std::isfinite(planner_recovery_wait_timeout_s_) ||
      planner_recovery_wait_timeout_s_ <= 0.0 ||
      std::abs(stale_after_s_ - 0.10) > 1.0e-9 ||
      std::abs(state_stale_after_s_ - 0.20) > 1.0e-9 ||
      std::abs(trajectory_wait_timeout_s_ - 5.0) > 1.0e-9 ||
      std::abs(planner_recovery_wait_timeout_s_ - 5.0) > 1.0e-9 ||
      planner_recovery_wait_timeout_s_ > trajectory_wait_timeout_s_ || !stale_after_ns ||
      !state_stale_after_ns || !planner_recovery_wait_timeout_ns || *stale_after_ns <= 0 ||
      *state_stale_after_ns <= 0 || *planner_recovery_wait_timeout_ns <= 0) {
    throw std::invalid_argument("invalid PX4 navigation external mode parameters");
  }
  stale_after_ns_ = *stale_after_ns;
  state_stale_after_ns_ = *state_stale_after_ns;
  planner_recovery_wait_timeout_ns_ = *planner_recovery_wait_timeout_ns;
  RCLCPP_INFO(node.get_logger(),
              "External Mode timing contract: command_stream=%.2fs state_age=%.2fs "
              "initial_hold=%.1fs stopped_recovery=%.1fs",
              stale_after_s_, state_stale_after_s_, trajectory_wait_timeout_s_,
              planner_recovery_wait_timeout_s_);
  navigation_command_subscription_ = node.create_subscription<
      navigation_contracts::msg::NavigationCommand>(
      navigation_command_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const navigation_contracts::msg::NavigationCommand::ConstSharedPtr& message) {
        onNavigationCommand(message);
      });
  const auto status_topic = node.declare_parameter<std::string>(
      "navigation.status_topic", "/navigation/mode_status");
  if (status_topic.empty()) {
    throw std::invalid_argument("navigation.status_topic must not be empty");
  }
  status_publisher_ = node.create_publisher<navigation_contracts::msg::NavigationModeStatus>(
      status_topic, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local());
  command_admission_publisher_ = node.create_publisher<
      navigation_contracts::msg::NavigationCommandAdmission>(
      "/navigation/command_admission", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable());
  command_rejection_publisher_ = node.create_publisher<
      navigation_contracts::msg::NavigationCommandRejection>(
      "/navigation/command_rejection", rclcpp::QoS{rclcpp::KeepLast{20}}.best_effort());
  mission_progress_subscription_ = node.create_subscription<
      navigation_contracts::msg::NavigationMissionProgress>(
      "/navigation/mission_progress", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable(),
      [this](const navigation_contracts::msg::NavigationMissionProgress::ConstSharedPtr& message) {
        onMissionProgress(message);
      });
  px4_input_trace_publisher_ = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/navigation/diagnostics", rclcpp::QoS{rclcpp::KeepLast{50}}.reliable());
  boundary_timer_ = node.create_wall_timer(std::chrono::milliseconds{50},
                                           [this]() { updateBoundary(); });
  px4_input_trace_worker_ = std::thread([this]() {
    while (!px4_input_trace_worker_stop_.load(std::memory_order_acquire)) {
      drainPx4InputTrace();
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    drainPx4InputTrace();
  });
  setSetpointUpdateRate(50.0F);
}

NavigationMode::~NavigationMode() {
  px4_input_trace_worker_stop_.store(true, std::memory_order_release);
  if (px4_input_trace_worker_.joinable()) {
    px4_input_trace_worker_.join();
  }
}

void NavigationMode::setPx4HoldHandover(std::function<void()> callback) {
  px4_hold_handover_ = std::move(callback);
}

void NavigationMode::attachStateInputNode(rclcpp::Node& state_input_node) {
  if (odometry_subscription_ || estimator_health_subscription_) {
    throw std::logic_error("External Mode state input node may only be attached once");
  }
  // px4_ros2 requires its ModeBase subscriptions to remain on a single-thread
  // executor.  State/health are product-owned ROS inputs, so receive them on a
  // separate node instead of placing the PX4 node itself on a multi-threaded
  // executor.  The callbacks only update mutex-protected snapshots consumed by
  // the PX4 executor thread.
  odometry_subscription_ = state_input_node.create_subscription<
      navigation_contracts::msg::PropagatedOdometry>(
      state_topic_, rclcpp::QoS{rclcpp::KeepLast{1}}.reliable(),
      [this](const navigation_contracts::msg::PropagatedOdometry::ConstSharedPtr& message) {
        onOdometry(message);
      });
  estimator_health_subscription_ = state_input_node.create_subscription<
      navigation_contracts::msg::EstimatorHealth>(
      "/lio/health", rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort(),
      [this](const navigation_contracts::msg::EstimatorHealth::ConstSharedPtr& message) {
        onEstimatorHealth(message);
      });
  px4_local_position_subscription_ = state_input_node.create_subscription<
      px4_msgs::msg::VehicleLocalPosition>(
      std::string("/fmu/out/vehicle_local_position") +
          px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(),
      rclcpp::QoS{rclcpp::KeepLast{1}}.best_effort(),
      [this](const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr& message) {
        onPx4LocalPosition(message);
      });
  RCLCPP_INFO(node().get_logger(),
              "External Mode state inputs use an independent single-thread receiver node");
}

void NavigationMode::publishStatus(std::uint8_t state, std::uint8_t reason) {
  if (!status_publisher_) return;
  navigation_contracts::msg::NavigationModeStatus status;
  status.header.stamp = node().get_clock()->now();
  status.header.frame_id = planning_frame_;
  bool health_ready = false;
  bool command_present = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    status.activation_id = mode_activation_id_;
    if (navigation_command_) {
      command_present = true;
      status.mission_id = navigation_command_->mission_id;
      status.waypoint_index = navigation_command_->waypoint_index;
      status.request_id = navigation_command_->request_id;
    }
    if (state == navigation_contracts::msg::NavigationModeStatus::COMPLETE &&
        mission_completion_receipt_) {
      const auto& receipt = *mission_completion_receipt_;
      status.mission_id = receipt.mission_id;
      status.waypoint_index = receipt.waypoint_index;
      status.request_id = receipt.request_id;
      status.waypoint_accepted = receipt.waypoint_accepted;
      status.accepted_waypoint_index = receipt.accepted_waypoint_index;
      status.acceptance_position_error_m = receipt.acceptance_position_error_m;
      status.acceptance_speed_mps = receipt.acceptance_speed_mps;
    }
    if (odometry_) {
      status.airborne = isArmed() &&
          std::isfinite(odometry_->pose.pose.position.z) &&
          odometry_->pose.pose.position.z > 0.5;
    }
    health_ready = typed_health_seen_ &&
        (lio_health_valid_ || tracking_experiment_.suppress_estimator_health_response);
  }
  status.state = state;
  status.reason = reason;
  // This is an observational projection of the already-selected mode state.
  // Keep it derived here so the evidence stream records why External Mode is
  // waiting/holding without introducing a second control state machine.
  if (state == navigation_contracts::msg::NavigationModeStatus::COMPLETE) {
    status.external_mode_state =
        navigation_contracts::msg::NavigationModeStatus::COMPLETED_HOLD;
  } else if (state == navigation_contracts::msg::NavigationModeStatus::FAILED) {
    status.external_mode_state =
        navigation_contracts::msg::NavigationModeStatus::FAILSAFE_HOLD;
  } else if (state == navigation_contracts::msg::NavigationModeStatus::BRAKING) {
    status.external_mode_state =
        navigation_contracts::msg::NavigationModeStatus::RECOVERY_HOLD;
  } else if (state == navigation_contracts::msg::NavigationModeStatus::PAUSED) {
    status.external_mode_state = reason ==
            navigation_contracts::msg::NavigationModeStatus::OPERATOR_TAKEOVER
        ? navigation_contracts::msg::NavigationModeStatus::HANDOVER_HOLD
        : navigation_contracts::msg::NavigationModeStatus::RECOVERY_HOLD;
  } else {
    if (!status.airborne) {
      status.external_mode_state =
          navigation_contracts::msg::NavigationModeStatus::WAIT_AIRBORNE;
    } else if (!health_ready) {
      status.external_mode_state =
          navigation_contracts::msg::NavigationModeStatus::WAIT_HEALTH;
    } else if (!command_present) {
      status.external_mode_state =
          navigation_contracts::msg::NavigationModeStatus::WAIT_FIRST_COMMAND;
    } else {
      status.external_mode_state =
          navigation_contracts::msg::NavigationModeStatus::TRACK_TRAJECTORY;
    }
  }
  switch (reason) {
    case navigation_contracts::msg::NavigationModeStatus::SAFETY_STOP:
      status.external_mode_reason = "SAFETY_STOP";
      break;
    case navigation_contracts::msg::NavigationModeStatus::OPERATOR_TAKEOVER:
      status.external_mode_reason = "OPERATOR_TAKEOVER";
      break;
    case navigation_contracts::msg::NavigationModeStatus::ODOMETRY_STALE:
      status.external_mode_reason = "ODOMETRY_STALE";
      break;
    case navigation_contracts::msg::NavigationModeStatus::TRAJECTORY_INVALID:
      status.external_mode_reason = "TRAJECTORY_INVALID";
      break;
    default:
      status.external_mode_reason =
          status.external_mode_state ==
                  navigation_contracts::msg::NavigationModeStatus::WAIT_AIRBORNE
              ? "WAIT_AIRBORNE"
              : status.external_mode_state ==
                        navigation_contracts::msg::NavigationModeStatus::WAIT_HEALTH
                    ? "WAIT_HEALTH"
                    : status.external_mode_state ==
                              navigation_contracts::msg::NavigationModeStatus::WAIT_FIRST_COMMAND
                          ? "WAIT_FIRST_COMMAND"
                          : "NONE";
      break;
  }
  status_publisher_->publish(status);
  last_status_state_ = state;
}

Px4InputTraceRecord NavigationMode::makePx4InputTraceRecord(
    const navigation_contracts::msg::NavigationCommand* command,
    const std::optional<Eigen::Vector3f>& position_ned,
    const std::optional<Eigen::Vector3f>& velocity_ned,
    const std::optional<Eigen::Vector3f>& acceleration_ned,
    const float yaw_ned, const float yaw_rate_ned,
    const Px4InputTraceBoundary boundary,
    const std::int64_t update_start_ros_ns,
    const std::int64_t update_start_steady_ns,
    const std::string_view velocity_only_reason,
    const std::uint64_t velocity_only_limited_count,
    const Px4InputStateTrace& state_input_trace) {
  Px4InputTraceRecord record;
  record.trace_sequence = ++px4_input_trace_sequence_;
  record.boundary = boundary;
  record.update_start_ros_ns = update_start_ros_ns;
  record.update_start_steady_ns = update_start_steady_ns;
  record.velocity_only_limited_count = velocity_only_limited_count;
  record.state_input = state_input_trace;
  const auto reason_size = std::min(
      velocity_only_reason.size(), record.velocity_only_reason.size() - 1U);
  std::copy_n(velocity_only_reason.data(), reason_size,
              record.velocity_only_reason.data());
  const auto copy_vector = [](const std::optional<Eigen::Vector3f>& source,
                              float target[3], bool& present) {
    if (!source.has_value() || !source->allFinite()) return;
    target[0] = source->x();
    target[1] = source->y();
    target[2] = source->z();
    present = true;
  };
  copy_vector(position_ned, record.position_ned, record.position_present);
  copy_vector(velocity_ned, record.velocity_ned, record.velocity_present);
  copy_vector(acceleration_ned, record.acceleration_ned, record.acceleration_present);
  if (std::isfinite(yaw_ned)) {
    record.yaw_ned = yaw_ned;
    record.yaw_present = true;
  }
  if (std::isfinite(yaw_rate_ned)) {
    record.yaw_rate_ned = yaw_rate_ned;
    record.yaw_rate_present = true;
  }
  if (command != nullptr) {
    record.command_present = true;
    record.sample_id = command->sample_id;
    record.request_id = command->request_id;
    record.goal_epoch = command->goal_epoch;
    record.localization_epoch = command->localization_epoch;
    record.bundle_generation = command->bundle_generation;
    // Planner cycle provenance is on the observer-only diagnostics topic.
    // The adapter must not subscribe to it to determine command admission.
    record.causal_planning_cycle_id = 0U;
    record.world_generation = command->world_generation;
    record.world_revision = command->world_revision;
    record.world_observation_stamp_ns = navigation_common::rosTimeToNanoseconds(
        command->world_observation_stamp).value_or(0);
    record.waypoint_index = command->waypoint_index;
    record.role = command->role;
    const auto copy_size = std::min(
        command->mission_id.size(), record.mission_id.size() - 1U);
    std::copy_n(command->mission_id.data(), copy_size, record.mission_id.data());
  }
  return record;
}

void NavigationMode::enqueuePx4InputTrace(Px4InputTraceRecord record) {
  if (!px4_input_trace_publisher_) return;
  if (!px4_input_trace_queue_.tryPush(record)) {
    px4_input_trace_drop_count_.fetch_add(1U, std::memory_order_relaxed);
    return;
  }
  px4_input_trace_enqueued_count_.fetch_add(1U, std::memory_order_relaxed);
}

void NavigationMode::setVelocityOnlyLastReason(const std::string_view reason) {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  velocity_only_last_reason_ = reason;
}

void NavigationMode::drainPx4InputTrace() {
  Px4InputTraceRecord record;
  while (px4_input_trace_queue_.tryPop(record)) {
    try {
      publishPx4InputTrace(record);
    } catch (...) {
      // Evidence transport must not terminate or delay the control owner.
      // A later trace exposes the accumulated error; a missing terminal trace
      // remains incomplete evidence at the offline boundary.
      px4_input_trace_publish_error_count_.fetch_add(1U, std::memory_order_relaxed);
    }
  }
}

void NavigationMode::publishPx4InputTrace(const Px4InputTraceRecord& record) {
  if (!px4_input_trace_publisher_) return;
  diagnostic_msgs::msg::DiagnosticArray array;
  const auto now = node().get_clock()->now();
  array.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_external_mode/PX4_INPUT_SETPOINT";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "PX4_INPUT_SETPOINT";
  const auto add = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  };
  const auto add_i64 = [&add](const std::string& key, const std::int64_t value) {
    add(key, std::to_string(value));
  };
  const auto add_u64 = [&add](const std::string& key, const std::uint64_t value) {
    add(key, std::to_string(value));
  };
  const auto add_vector = [&add](const std::string& key, const float value[3],
                                 const bool present) {
    if (!present) {
      add(key, "NOT_RECORDED");
      return;
    }
    std::ostringstream stream;
    stream << std::setprecision(9) << '[' << value[0] << ',' << value[1] << ','
           << value[2] << ']';
    add(key, stream.str());
  };
  const auto boundary_name = [](const Px4InputTraceBoundary value) {
    switch (value) {
      case Px4InputTraceBoundary::kTracking: return "tracking";
      case Px4InputTraceBoundary::kVelocityOnly: return "velocity_only";
      case Px4InputTraceBoundary::kVelocityHold: return "velocity_hold";
      case Px4InputTraceBoundary::kPositionHold: return "position_hold";
    }
    return "unknown";
  };
  add_i64("trace_timestamp_ns", record.update_end_ros_ns > 0
      ? record.update_end_ros_ns : now.nanoseconds());
  add_i64("trace_steady_timestamp_ns", record.update_end_steady_ns);
  add_i64("update_start_ros_ns", record.update_start_ros_ns);
  add_i64("update_end_ros_ns", record.update_end_ros_ns);
  add_i64("update_start_steady_ns", record.update_start_steady_ns);
  add_i64("update_end_steady_ns", record.update_end_steady_ns);
  if (record.update_end_steady_ns >= record.update_start_steady_ns &&
      record.update_start_steady_ns > 0) {
    add_i64("setpoint_update_duration_ns",
            record.update_end_steady_ns - record.update_start_steady_ns);
  } else {
    add("setpoint_update_duration_ns", "NOT_RECORDED");
  }
  add_u64("trace_sequence", record.trace_sequence);
  const auto& state = record.state_input;
  const bool state_present = state.sequence > 0U && state.localization_epoch > 0U;
  add("state_input_present", state_present ? "true" : "false");
  const auto add_state_stamp = [&add, &add_i64, state_present](
      const std::string& key, const std::int64_t value) {
    if (state_present && value > 0) add_i64(key, value);
    else add(key, "NOT_RECORDED");
  };
  if (state_present) {
    add_u64("state_sequence", state.sequence);
    add_u64("state_localization_epoch", state.localization_epoch);
  } else {
    add("state_sequence", "NOT_RECORDED");
    add("state_localization_epoch", "NOT_RECORDED");
  }
  add_state_stamp("state_source_stamp_ros_ns", state.source_stamp_ros_ns);
  add_state_stamp("state_callback_enter_ros_ns", state.callback_enter_ros_ns);
  add_state_stamp("state_callback_enter_steady_ns", state.callback_enter_steady_ns);
  add_state_stamp("state_lock_requested_steady_ns", state.lock_requested_steady_ns);
  add_state_stamp("state_lock_acquired_steady_ns", state.lock_acquired_steady_ns);
  add_state_stamp("state_receive_steady_ns", state.receive_steady_ns);
  add_state_stamp("state_snapshot_ros_ns", state.snapshot_ros_ns);
  add_state_stamp("state_snapshot_steady_ns", state.snapshot_steady_ns);
  add("command_present", record.command_present ? "true" : "false");
  if (record.command_present) {
    add("mission_id", std::string(record.mission_id.data()));
    add_u64("waypoint_index", record.waypoint_index);
    add_u64("request_id", record.request_id);
    add_u64("goal_epoch", record.goal_epoch);
    add_u64("localization_epoch", record.localization_epoch);
    add_u64("bundle_generation", record.bundle_generation);
    add_u64("causal_planning_cycle_id", record.causal_planning_cycle_id);
    add_u64("world_generation", record.world_generation);
    add_u64("world_revision", record.world_revision);
    add_i64("world_observation_stamp_ns", record.world_observation_stamp_ns);
    add_u64("sample_id", record.sample_id);
    add("role", std::to_string(record.role));
  } else {
    add("mission_id", "NOT_RECORDED");
    add("waypoint_index", "NOT_RECORDED");
    add("request_id", "NOT_RECORDED");
    add("goal_epoch", "NOT_RECORDED");
    add("bundle_generation", "NOT_RECORDED");
    add("causal_planning_cycle_id", "NOT_RECORDED");
    add("world_generation", "NOT_RECORDED");
    add("world_revision", "NOT_RECORDED");
    add("world_observation_stamp_ns", "NOT_RECORDED");
    add("sample_id", "NOT_RECORDED");
    add("role", "NOT_RECORDED");
  }
  add("setpoint_boundary", boundary_name(record.boundary));
  add("velocity_only_gain_s_inv",
      std::to_string(tracking_experiment_.velocity_only_gain_s_inv));
  add("velocity_only_cap_mps",
      std::to_string(tracking_experiment_.velocity_only_cap_mps));
  add("velocity_only_max_acceleration_mps2",
      std::to_string(tracking_experiment_.velocity_only_max_acceleration_mps2));
  add("velocity_only_max_jerk_mps3",
      std::to_string(tracking_experiment_.velocity_only_max_jerk_mps3));
  add("velocity_only_limited_count",
      std::to_string(record.velocity_only_limited_count));
  add("velocity_only_reason", record.velocity_only_reason[0] == '\0'
          ? "NOT_RECORDED" : std::string(record.velocity_only_reason.data()));
  add_u64("trace_enqueued_count",
          px4_input_trace_enqueued_count_.load(std::memory_order_relaxed));
  add_u64("trace_published_before_count",
          px4_input_trace_published_count_.load(std::memory_order_relaxed));
  add_u64("trace_drop_count",
          px4_input_trace_drop_count_.load(std::memory_order_relaxed));
  add_u64("trace_publish_error_count",
          px4_input_trace_publish_error_count_.load(std::memory_order_relaxed));
  add_vector("position_ned", record.position_ned, record.position_present);
  add_vector("velocity_ned", record.velocity_ned, record.velocity_present);
  add_vector("acceleration_ned", record.acceleration_ned, record.acceleration_present);
  add("yaw_ned", record.yaw_present ? std::to_string(record.yaw_ned) : "NOT_RECORDED");
  add("yaw_rate_ned", record.yaw_rate_present
          ? std::to_string(record.yaw_rate_ned) : "NOT_RECORDED");
  array.status.push_back(std::move(status));
  px4_input_trace_publisher_->publish(std::move(array));
  px4_input_trace_published_count_.fetch_add(1U, std::memory_order_relaxed);
}

void NavigationMode::publishAlignmentLatchWitnessLocked() {
  if (!px4_input_trace_publisher_ || !odometry_.has_value() ||
      !px4_local_position_ned_.has_value() || !px4_local_velocity_ned_.has_value()) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  const auto latch_ros_time = node().get_clock()->now();
  array.header.stamp = latch_ros_time;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_external_mode/ALIGNMENT_LATCH_WITNESS";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "ALIGNMENT_LATCH_WITNESS";
  const auto add = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  };
  const auto add_i64 = [&add](const std::string& key, const std::int64_t value) {
    add(key, std::to_string(value));
  };
  const auto add_u64 = [&add](const std::string& key, const std::uint64_t value) {
    add(key, std::to_string(value));
  };
  const auto add_bool = [&add](const std::string& key, const bool value) {
    add(key, value ? "true" : "false");
  };
  const auto add_double = [&add](const std::string& key, const double value) {
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    add(key, stream.str());
  };
  const auto add_vector = [&add](const std::string& key,
                                 const Eigen::Vector3d& value) {
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::max_digits10)
           << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
    add(key, stream.str());
  };
  const auto add_covariance = [&add](const std::string& key,
                                     const std::array<double, 36>& covariance) {
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::max_digits10) << '[';
    for (std::size_t index = 0; index < covariance.size(); ++index) {
      if (index != 0U) stream << ',';
      stream << covariance[index];
    }
    stream << ']';
    add(key, stream.str());
  };
  const auto add_array4 = [&add](const std::string& key,
                                 const double x, const double y,
                                 const double z, const double w) {
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<double>::max_digits10)
           << '[' << x << ',' << y << ',' << z << ',' << w << ']';
    add(key, stream.str());
  };
  const auto px4_sample_ns = last_px4_local_position_timestamp_sample_us_ <=
          static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000)
      ? static_cast<std::int64_t>(last_px4_local_position_timestamp_sample_us_ * 1000U)
      : 0;
  const auto px4_receive_ns = last_px4_local_position_receive_ns_;
  const auto lio_source_ns = last_propagated_state_stamp_ns_;
  const auto source_skew_ns = (lio_source_ns > 0 && px4_sample_ns > 0)
      ? lio_source_ns - px4_sample_ns : 0;
  const auto receive_skew_ns = (last_odometry_receive_ns_ > 0 && px4_receive_ns > 0)
      ? last_odometry_receive_ns_ - px4_receive_ns : 0;
  const auto lio_velocity = Eigen::Vector3d{
      odometry_->twist.twist.linear.x, odometry_->twist.twist.linear.y,
      odometry_->twist.twist.linear.z};
  const auto lio_position = Eigen::Vector3d{
      odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
      odometry_->pose.pose.position.z};
  add_u64("alignment_generation", alignment_latch_generation_);
  add_i64("latch_ros_time_ns", latch_ros_time.nanoseconds());
  add_i64("latch_steady_time_ns", navigation_common::steadyClockNowNanoseconds());
  add("alignment_basis", "enu_to_ned_swap_xy_neg_z");
  add("alignment_basis_matrix_ned_from_enu", "[[0,1,0],[1,0,0],[0,0,-1]]");
  add_vector("t_align_ned", *lio_to_px4_local_translation_ned_);
  add("lio_frame_id", odometry_->header.frame_id);
  add("lio_child_frame_id", odometry_->child_frame_id);
  add_u64("lio_localization_epoch", lio_localization_epoch_);
  add_u64("lio_sequence", last_propagated_state_sequence_);
  add_i64("lio_source_stamp_ns", lio_source_ns);
  add_i64("lio_receive_stamp_ns", last_odometry_receive_ns_);
  add_vector("lio_position", lio_position);
  add_vector("lio_velocity", lio_velocity);
  add_array4("lio_orientation_xyzw", odometry_->pose.pose.orientation.x,
             odometry_->pose.pose.orientation.y, odometry_->pose.pose.orientation.z,
             odometry_->pose.pose.orientation.w);
  add_covariance("lio_pose_covariance", odometry_->pose.covariance);
  add_covariance("lio_twist_covariance", odometry_->twist.covariance);
  add_bool("lio_navigation_valid", last_health_navigation_valid_);
  add_bool("lio_covariance_valid", last_health_covariance_valid_);
  add_bool("lio_observability_valid", last_health_observability_valid_);
  add_bool("lio_correction_fresh", last_health_correction_fresh_);
  add_bool("lio_propagation_valid", last_health_propagation_valid_);
  add_i64("lio_health_source_stamp_ns", last_health_source_stamp_ns_);
  add_i64("lio_health_correction_stamp_ns", last_health_correction_stamp_ns_);
  add_i64("lio_health_propagated_stamp_ns", last_health_propagated_stamp_ns_);
  add_i64("lio_health_source_age_ns",
          last_health_source_stamp_ns_ > 0
              ? latch_ros_time.nanoseconds() - last_health_source_stamp_ns_ : 0);
  add_i64("lio_correction_age_ns",
          last_health_correction_stamp_ns_ > 0
              ? latch_ros_time.nanoseconds() - last_health_correction_stamp_ns_ : 0);
  add_bool("stationary_history_available", false);
  add_double("lio_speed_mps", lio_velocity.norm());
  add_double("px4_speed_mps", px4_local_velocity_ned_->norm());
  add_u64("px4_timestamp_us", last_px4_local_position_timestamp_us_);
  add_u64("px4_timestamp_sample_us", last_px4_local_position_timestamp_sample_us_);
  add_i64("px4_receive_stamp_ns", px4_receive_ns);
  add_i64("lio_px4_source_skew_ns", source_skew_ns);
  add_i64("lio_px4_receive_skew_ns", receive_skew_ns);
  add_vector("px4_position_ned", *px4_local_position_ned_);
  add_vector("px4_velocity_ned", *px4_local_velocity_ned_);
  add_bool("px4_xy_valid", last_px4_xy_valid_);
  add_bool("px4_z_valid", last_px4_z_valid_);
  add_bool("px4_vxy_valid", last_px4_vxy_valid_);
  add_bool("px4_vz_valid", last_px4_vz_valid_);
  add_bool("px4_dead_reckoning", last_px4_dead_reckoning_);
  add_u64("px4_xy_reset_counter", px4_xy_reset_counter_);
  add_u64("px4_z_reset_counter", px4_z_reset_counter_);
  add_u64("px4_vxy_reset_counter", px4_vxy_reset_counter_);
  add_u64("px4_vz_reset_counter", px4_vz_reset_counter_);
  add_u64("px4_heading_reset_counter", px4_heading_reset_counter_);
  add_double("px4_delta_xy_north_m", last_px4_delta_xy_north_m_);
  add_double("px4_delta_xy_east_m", last_px4_delta_xy_east_m_);
  add_double("px4_delta_z_m", last_px4_delta_z_m_);
  add_double("px4_delta_heading_rad", last_px4_delta_heading_rad_);
  array.status.push_back(std::move(status));
  px4_input_trace_publisher_->publish(std::move(array));
}

void NavigationMode::onActivate() {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    activation_time_ = node().get_clock()->now();
    last_setpoint_time_ = activation_time_;
    failure_reported_ = false;
    navigation_command_ = transitionCertifiedCommand(
        navigation_command_, std::nullopt, CertifiedCommandTransition::kInvalidate);
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
    admission_rejections_by_stage_.fill(0U);
    waypoint_handoff_retained_command_count_ = 0U;
    setpoint_update_count_ = 0U;
    stale_state_failure_count_ = 0U;
    last_command_receive_ns_ = 0;
    airborne_start_ns_ = 0;
    maximum_odometry_callback_gap_us_ = 0;
    last_setpoint_update_ns_ = 0;
    maximum_setpoint_callback_gap_us_ = 0;
    last_metrics_log_ns_ = 0;
    last_state_age_s_ = -1.0;
    mode_active_ = true;
    if (mode_activation_id_ < std::numeric_limits<std::uint64_t>::max()) {
      ++mode_activation_id_;
    } else {
      failure_reported_ = true;
    }
    mission_completion_receipt_.reset();
    handover_requested_ = false;
    clearPlannerRecoveryEpisodeLocked();
    // The estimator may still establish/re-anchor its public local frame during
    // the disarmed takeoff preparation.  Never carry a pre-activation frame
    // pair into this activation; capture it only after the airborne gate has
    // completed and both sources are stationary again.
    px4_local_frame_aligned_ = false;
    lio_to_px4_local_translation_ned_.reset();
    completion_position_.reset();
    safety_hold_position_.reset();
    velocity_only_previous_.reset();
    velocity_only_reset_counters_seen_ = false;
    velocity_only_last_reason_.clear();
    velocity_only_limited_count_ = 0U;
  }
  publishStatus(navigation_contracts::msg::NavigationModeStatus::ACTIVE,
                navigation_contracts::msg::NavigationModeStatus::NONE);
  updateBoundary();
}

void NavigationMode::onDeactivate() {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    mode_active_ = false;
    // Deactivation is a terminal boundary for the current mode activation.
    // Keep rejecting late trajectories until onActivate() explicitly starts
    // a new generation; otherwise the runtime can repopulate the cached
    // trajectory while the PX4 mode executor is already handing over.
    failure_reported_ = true;
    navigation_command_ = transitionCertifiedCommand(
        navigation_command_, std::nullopt, CertifiedCommandTransition::kInvalidate);
    velocity_only_previous_.reset();
    velocity_only_reset_counters_seen_ = false;
    velocity_only_last_reason_.clear();
    last_command_receive_ns_ = 0;
    airborne_start_ns_ = 0;
    px4_local_frame_aligned_ = false;
    lio_to_px4_local_translation_ned_.reset();
    completion_position_.reset();
    safety_hold_position_.reset();
    clearPlannerRecoveryEpisodeLocked();
  }
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
  // Core publishes the first mission intent after observing an airborne mode
  // activation. Freshness and estimator heartbeats remain local run conditions.
  if (!mode_active_ || mission_completion_receipt_ || handover_requested_) return;
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
  const auto health_freshness = navigation_contracts::evaluateExecutionStateFreshness(
      now_ns, last_health_source_stamp_ns_,
      navigation_common::steadyClockNowNanoseconds(),
      last_health_receive_steady_ns_, state_stale_after_s_);
  const bool diagnostics_stale = !health_freshness.valid();
  const bool fresh_typed_health_bypass =
      tracking_experiment_.suppress_estimator_health_response &&
      typed_health_seen_ && health_freshness.valid();
  if (diagnostics_stale || (!lio_health_valid_ && !fresh_typed_health_bypass)) {
    reporter.armingCheckFailureExt(px4_ros2::events::ID("uav_navigation_lio_unhealthy"),
                                   px4_ros2::events::Log::Error,
                                   "FAST-LIO health is stale or invalid");
  }
  const bool waiting_for_airborne =
      !isArmed() || !odometry_ || odometry_->pose.pose.position.z <= 0.5;
  // During the disarmed warm-up activation the mission deliberately has no
  // goal yet, so the planner has no command to publish. Requiring command
  // freshness here makes PX4 fail the warm-up before arm/takeoff completes.
  // Once airborne, the normal command freshness gate is active.
  if (!waiting_for_airborne && stale(last_command_receive_ns_, trajectory_wait_timeout_s_)) {
    const double active_s = airborne_start_ns_ > 0 && now_ns >= airborne_start_ns_
                                ? static_cast<double>(now_ns - airborne_start_ns_) / 1e9
                                : 0.0;
    if (active_s > trajectory_wait_timeout_s_) {
      reporter.armingCheckFailureExt(px4_ros2::events::ID("uav_navigation_planner_command_stale"),
                                     px4_ros2::events::Log::Error,
                                     "Navigation planner command is stale");
    }
  }
}

void NavigationMode::publishAdmissionRejection(
    const CommandAdmissionAssessment& assessment,
    const navigation_contracts::msg::NavigationCommand* command, std::int64_t callback_ros_ns,
    std::int64_t callback_steady_ns, double source_age_ms, double receive_age_ms,
    double longitudinal_error_m, double lateral_error_m) {
  if (assessment.accepted()) return;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const auto index = static_cast<std::size_t>(assessment.stage);
    if (index < admission_rejections_by_stage_.size()) {
      ++admission_rejections_by_stage_[index];
    }
  }
  if (!command_rejection_publisher_) return;
  navigation_contracts::msg::NavigationCommandRejection event;
  event.header.stamp = navigation_common::nanosecondsToRosTime(callback_ros_ns)
                           .value_or(builtin_interfaces::msg::Time{});
  event.header.frame_id = planning_frame_;
  event.callback_steady_ns =
      static_cast<std::uint64_t>(std::max<std::int64_t>(0, callback_steady_ns));
  event.command_present = command != nullptr;
  event.stage = static_cast<std::uint8_t>(assessment.stage);
  event.reason_code = admissionReasonCode(assessment.reason);
  event.disposition = static_cast<std::uint8_t>(assessment.disposition);
  event.source_age_ms = source_age_ms;
  event.receive_age_ms = receive_age_ms;
  event.tracking_longitudinal_error_m = longitudinal_error_m;
  event.tracking_lateral_error_m = lateral_error_m;
  if (command) {
    event.mode_activation_id = command->mode_activation_id;
    event.localization_epoch = command->localization_epoch;
    event.goal_epoch = command->goal_epoch;
    event.mission_id = command->mission_id;
    event.waypoint_index = command->waypoint_index;
    event.request_id = command->request_id;
    event.bundle_generation = command->bundle_generation;
    event.sample_id = command->sample_id;
    event.command_stamp = command->header.stamp;
    event.valid_until = command->valid_until;
  }
  try {
    command_rejection_publisher_->publish(event);
  } catch (const std::exception& error) {
    RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
                         "command rejection diagnostic publish failed: %s", error.what());
  }
}

TrackingEnvelopeResult NavigationMode::assessTrackingLocked(
    const navigation_contracts::msg::NavigationCommand& command,
    const nav_msgs::msg::Odometry& odometry, bool& anchor_invalid,
    std::optional<RejectProvenance>& reject_provenance) {
  TrackingEnvelopeResult tracking_envelope;
  const auto& point = odometry.pose.pose.position;
  const Eigen::Vector3d measured{point.x, point.y, point.z};
  const Eigen::Vector3d command_position{command.position.x, command.position.y,
                                         command.position.z};
  const Eigen::Vector3d command_velocity{command.velocity.x, command.velocity.y,
                                         command.velocity.z};
  // Use authoritative command/lifecycle fields, never diagnostic trace
  // flags. Runtime retains its stricter analytic and terminal checks.
  const bool main_phase_tracking =
      command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN &&
      command.status == navigation_contracts::msg::NavigationCommand::STATUS_READY &&
      !planner_recovery_pending_ && !mission_completion_receipt_;
  tracking_envelope = evaluateTrackingEnvelope(
      measured, command_position, command_velocity, navigation_contracts::kCommandAnchorErrorLimitM,
      main_phase_tracking ? navigation_contracts::kMainTrackingPhaseWindowS : 0.0);
  const bool geometric_tracking_support_valid = tracking_envelope.support_valid;
  anchor_invalid = !tracking_envelope.valid;
  if (tracking_experiment_.enabled && main_phase_tracking) {
    const auto& twist = odometry.twist.twist.linear;
    const auto& q = odometry.pose.pose.orientation;
    const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    const Eigen::Vector3d measured_velocity =
        isNormalizableOdometryQuaternion(orientation)
            ? (orientation.normalized() * Eigen::Vector3d(twist.x, twist.y, twist.z)).eval()
            : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    const auto adaptive = navigation_contracts::assessAdaptiveTracking(
        tracking_experiment_, measured, measured_velocity, command_position, command_velocity);
    const bool permitted =
        navigation_contracts::experimentPermitsTracking(tracking_experiment_, adaptive);
    if (permitted && tracking_experiment_.suppress_braking &&
        (anchor_invalid || !adaptive.within_limits)) {
      ++experimental_tracking_suppressed_count_;
      RCLCPP_WARN_THROTTLE(
          node().get_logger(), *node().get_clock(), 1000,
          "TRACKING_EXPERIMENT_BYPASS total=%lu lateral=%.3f/%.3fm longitudinal=%.3f/%.3fm",
          static_cast<unsigned long>(experimental_tracking_suppressed_count_),
          adaptive.lateral_error_m, adaptive.lateral_limit_m, adaptive.longitudinal_error_m,
          adaptive.longitudinal_limit_m);
    }
    anchor_invalid = !permitted;
    // Keep rejection logs in the same units as the actual experiment gate.
    tracking_envelope.valid = permitted;
    tracking_envelope.longitudinal_error_m = adaptive.longitudinal_error_m;
    tracking_envelope.longitudinal_limit_m = adaptive.longitudinal_limit_m;
    tracking_envelope.reverse_error_m = 0.0;
    tracking_envelope.reverse_limit_m = adaptive.longitudinal_limit_m;
    tracking_envelope.lateral_error_m = adaptive.lateral_error_m;
    tracking_envelope.lateral_limit_m = adaptive.lateral_limit_m;
  }
  // Explicit relaxed SITL comparator: suppress only the finite geometric
  // tracking response for every executable role so planner stability can
  // be observed without a later BACKUP/EMERGENCY consumer rejection
  // ending the run. Identity, freshness, lease, finite-input, map and
  // collision checks remain active. This mode is never qualification
  // eligible and is recorded in the runtime artifact.
  if (anchor_invalid && tracking_experiment_.enabled && tracking_experiment_.suppress_braking &&
      geometric_tracking_support_valid) {
    ++experimental_tracking_suppressed_count_;
    const char* bypass_role =
        command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN        ? "MAIN"
        : command.role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP    ? "BACKUP"
        : command.role == navigation_contracts::msg::NavigationCommand::ROLE_EMERGENCY ? "EMERGENCY"
                                                                                       : "UNKNOWN";
    RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
                         "TRACKING_EXPERIMENT_BYPASS total=%lu role=%s "
                         "longitudinal=%.3f/%.3fm reverse=%.3f/%.3fm lateral=%.3f/%.3fm",
                         static_cast<unsigned long>(experimental_tracking_suppressed_count_),
                         bypass_role, tracking_envelope.longitudinal_error_m,
                         tracking_envelope.longitudinal_limit_m, tracking_envelope.reverse_error_m,
                         tracking_envelope.reverse_limit_m, tracking_envelope.lateral_error_m,
                         tracking_envelope.lateral_limit_m);
    anchor_invalid = false;
    tracking_envelope.valid = true;
  }
  if (anchor_invalid) {
    reject_provenance =
        buildRejectProvenance(node().get_clock()->now().nanoseconds(), last_odometry_receive_ns_,
                              odometry, command, navigation_command_);
    ++trajectory_rejected_count_;
  }
  return tracking_envelope;
}

void NavigationMode::finishAcceptedCommand(
    const navigation_contracts::msg::NavigationCommand& command, bool completed_command,
    bool terminal_recovery_needed, bool terminal_backup_hold_inside_acceptance,
    bool terminal_main_hold_inside_acceptance) {
  bool recovery_deadline_invalid = false;
  if (command_admission_publisher_) {
    navigation_contracts::msg::NavigationCommandAdmission receipt;
    receipt.header.stamp = node().get_clock()->now();
    receipt.header.frame_id = planning_frame_;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      receipt.mode_activation_id = mode_activation_id_;
    }
    receipt.mission_id = command.mission_id;
    receipt.localization_epoch = command.localization_epoch;
    receipt.goal_epoch = command.goal_epoch;
    receipt.waypoint_index = command.waypoint_index;
    receipt.request_id = command.request_id;
    receipt.bundle_generation = command.bundle_generation;
    receipt.sample_id = command.sample_id;
    command_admission_publisher_->publish(receipt);
  }
  if (completed_command && terminal_recovery_needed) {
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
          rememberPlannerRecoveryEpisodeLocked(command);
          RCLCPP_WARN(node().get_logger(),
                      "planner backend terminal endpoint requires mission acknowledgement; "
                      "holding for bounded planner recovery window %.3f s",
                      planner_recovery_wait_timeout_s_);
        }
      }
    }
  } else if (!completed_command) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    clearPlannerRecoveryEpisodeLocked();
  } else if ((terminal_backup_hold_inside_acceptance || terminal_main_hold_inside_acceptance)) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    clearPlannerRecoveryEpisodeLocked();
  }
  if (recovery_deadline_invalid) {
    safetyStopNavigation("planner recovery deadline is not representable");
  }
}

void NavigationMode::onNavigationCommand(
    const navigation_contracts::msg::NavigationCommand::ConstSharedPtr& message) {
  const auto callback_now = node().get_clock()->now();
  const auto callback_steady_ns = navigation_common::steadyClockNowNanoseconds();
  CommandAdmissionAssessment assessment;
  if (!message) {
    assessment = {AdmissionStage::kPresence, AdmissionDisposition::kRejectRetainPrevious,
                  PresenceReason::kMessageMissing};
  } else if (const auto reason =
                 navigation_contracts::assessCommandContract(*message, planning_frame_);
             reason != navigation_contracts::CommandContractReason::kValid) {
    assessment = {AdmissionStage::kContract, AdmissionDisposition::kRejectRetainPrevious, reason};
  } else if (const auto reason = navigation_contracts::assessCommandTemporalLease(
                 *message, callback_now.nanoseconds());
             reason != navigation_contracts::CommandTemporalReason::kValid) {
    assessment = {AdmissionStage::kTemporalLease, AdmissionDisposition::kRejectRetainPrevious,
                  reason};
  }
  if (!assessment.accepted()) {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      ++trajectory_rejected_count_;
      // A malformed replacement does not revoke the independently accepted
      // command already being executed. Its own validity/freshness and health
      // leases remain authoritative in updateSetpoint().
      navigation_command_ = transitionCertifiedCommand(navigation_command_, std::nullopt,
                                                       CertifiedCommandTransition::kRetain);
    }
    publishAdmissionRejection(assessment, message.get(), callback_now.nanoseconds(),
                              callback_steady_ns);
    RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "planner command rejected stage=%s reason=%s reason_code=%u disposition=%s "
        "command_present=%d callback_ros_ns=%ld callback_steady_ns=%ld "
        "mission=%s localization_epoch=%lu goal_epoch=%lu request=%lu bundle=%lu sample=%lu "
        "header_ns=%ld valid_until_ns=%ld",
        admissionStageName(assessment.stage), admissionReasonName(assessment.reason),
        static_cast<unsigned int>(admissionReasonCode(assessment.reason)),
        admissionDispositionName(assessment.disposition), message ? 1 : 0,
        static_cast<long>(callback_now.nanoseconds()), static_cast<long>(callback_steady_ns),
        message ? message->mission_id.c_str() : "",
        static_cast<unsigned long>(message ? message->localization_epoch : 0U),
        static_cast<unsigned long>(message ? message->goal_epoch : 0U),
        static_cast<unsigned long>(message ? message->request_id : 0U),
        static_cast<unsigned long>(message ? message->bundle_generation : 0U),
        static_cast<unsigned long>(message ? message->sample_id : 0U),
        static_cast<long>(
            message ? navigation_contracts::commandStampNanoseconds(message->header.stamp) : 0),
        static_cast<long>(
            message ? navigation_contracts::commandStampNanoseconds(message->valid_until) : 0));
    return;
  }

  bool terminal_authority_closed = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    // Once control has entered a terminal/handover stream, later planner
    // samples cannot restore command ownership. Ignore them before running
    // identity/tracking gates so one terminal transition has one causal log.
    terminal_authority_closed =
        failure_reported_ || mission_completion_receipt_ || handover_requested_;
  }
  if (terminal_authority_closed) {
    publishAdmissionRejection(
        {AdmissionStage::kTerminalOwnership, AdmissionDisposition::kIgnoreAfterTerminal,
         TerminalReason::kAuthorityClosed},
        message.get(), callback_now.nanoseconds(), callback_steady_ns);
    return;
  }

  bool accepted = false;
  bool anchor_invalid = false;
  bool completed_command = false;
  bool terminal_backup_hold_inside_acceptance = false;
  bool terminal_main_hold_inside_acceptance = false;
  bool terminal_recovery_needed = false;
  navigation_contracts::ExecutionStateFreshness odometry_freshness;
  TrackingEnvelopeResult tracking_envelope;
  std::optional<RejectProvenance> reject_provenance;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    // A command is executable only after a fresh, healthy typed-health sample
    // has established the current public estimator epoch.  Caching a command
    // before that handshake would let an untagged odometry stream become the
    // implicit epoch authority.
    const auto session_reason = assessCommandSessionIdentity(
        *message, navigation_command_, typed_health_seen_,
        lio_health_valid_ || tracking_experiment_.suppress_estimator_health_response,
        lio_localization_epoch_, mode_activation_id_);
    // The accepted command itself is the authenticated Core execution identity.
    // The adapter checks a monotonic session and never reconstructs waypoint
    // policy from its own mission definition.
    if (session_reason != SessionIdentityReason::kValid) {
      ++trajectory_rejected_count_;
      navigation_command_ = transitionCertifiedCommand(navigation_command_, std::nullopt,
                                                       CertifiedCommandTransition::kRetain);
      assessment =
          CommandAdmissionAssessment{AdmissionStage::kSessionIdentity,
                                     AdmissionDisposition::kRejectRetainPrevious, session_reason};
    } else {
      const auto odometry_source_ns =
          odometry_ ? navigation_common::rosTimeToNanoseconds(odometry_->header.stamp).value_or(0)
                    : 0;
      odometry_freshness = navigation_contracts::evaluateExecutionStateFreshness(
          node().get_clock()->now().nanoseconds(), odometry_source_ns,
          navigation_common::steadyClockNowNanoseconds(), last_odometry_receive_steady_ns_,
          state_stale_after_s_);
      const auto acceptance_gate =
          classifyCommandAcceptance(odometry_freshness, message->sample_id,
                                    navigation_command_ ? navigation_command_->sample_id : 0U);
      if (acceptance_gate == CommandAcceptanceGate::kOdometryStale) {
        ++trajectory_rejected_count_;
        if (!failure_reported_) ++stale_state_failure_count_;
        assessment = CommandAdmissionAssessment{AdmissionStage::kOdometryFreshness,
                                                AdmissionDisposition::kRejectFailNavigation,
                                                odometry_freshness.reason};
      } else if (acceptance_gate == CommandAcceptanceGate::kNonIncreasingMessageId) {
        ++trajectory_rejected_count_;
        assessment = CommandAdmissionAssessment{AdmissionStage::kSampleOrdering,
                                                AdmissionDisposition::kRejectRetainPrevious,
                                                acceptance_gate};
      }
      if (assessment.accepted()) {
        const bool terminal_failure =
            message->status == navigation_contracts::msg::NavigationCommand::STATUS_REJECTED;
        completed_command =
            message->status == navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED;
        bool terminal_hold_inside_acceptance = false;
        if (completed_command && odometry_) {
          const auto& p = odometry_->pose.pose.position;
          const Eigen::Vector3d measured{p.x, p.y, p.z};
          const Eigen::Vector3d endpoint{message->position.x, message->position.y,
                                         message->position.z};
          terminal_hold_inside_acceptance =
              measured.allFinite() && endpoint.allFinite() &&
              (measured - endpoint).norm() <= navigation_contracts::kCommandAnchorErrorLimitM;
          terminal_backup_hold_inside_acceptance =
              terminal_hold_inside_acceptance &&
              message->role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP;
          terminal_main_hold_inside_acceptance =
              terminal_hold_inside_acceptance &&
              message->role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN;
          terminal_recovery_needed = !terminal_hold_inside_acceptance;
        }
        if (!terminal_failure && !terminal_hold_inside_acceptance && odometry_.has_value()) {
          tracking_envelope =
              assessTrackingLocked(*message, *odometry_, anchor_invalid, reject_provenance);
        }
        if (anchor_invalid) {
          assessment = CommandAdmissionAssessment{AdmissionStage::kTrackingEnvelope,
                                                  AdmissionDisposition::kRejectSafetyStop,
                                                  TrackingReason::kEnvelopeExceeded};
        }
        if (assessment.accepted()) {
          navigation_command_ = transitionCertifiedCommand(navigation_command_, *message,
                                                           CertifiedCommandTransition::kCommit);
          ++trajectory_received_count_;
          ++trajectory_accepted_count_;
          last_command_receive_ns_ = node().get_clock()->now().nanoseconds();
          failure_reported_ = false;
          accepted = true;
        }
      }
    }
  }
  if (assessment.disposition == AdmissionDisposition::kRejectRetainPrevious) {
    publishAdmissionRejection(assessment, message.get(), callback_now.nanoseconds(),
                              callback_steady_ns);
    RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "Core command rejected stage=%s reason=%s code=%u disposition=%s "
        "mission=%s wp=%u request=%lu goal_epoch=%lu sample=%lu",
        admissionStageName(assessment.stage), admissionReasonName(assessment.reason),
        static_cast<unsigned>(admissionReasonCode(assessment.reason)),
        admissionDispositionName(assessment.disposition), message->mission_id.c_str(),
        message->waypoint_index, static_cast<unsigned long>(message->request_id),
        static_cast<unsigned long>(message->goal_epoch),
        static_cast<unsigned long>(message->sample_id));
    return;
  }
  if (assessment.disposition == AdmissionDisposition::kRejectFailNavigation) {
    publishAdmissionRejection(assessment, message.get(), callback_now.nanoseconds(),
                              callback_steady_ns, odometry_freshness.source_age_ms,
                              odometry_freshness.receive_age_ms);
    RCLCPP_ERROR(node().get_logger(),
                 "Rejecting planner backend command because navigation odometry lease is stale: "
                 "reason=%s source_age_ms=%.3f receive_age_ms=%.3f generation=%lu "
                 "trajectory_time=%.6f",
                 navigation_contracts::executionStateFreshnessReasonName(odometry_freshness.reason),
                 odometry_freshness.source_age_ms, odometry_freshness.receive_age_ms,
                 static_cast<unsigned long>(message->bundle_generation),
                 message->trajectory_time_s);
    failNavigation("navigation odometry stale at command acceptance");
    return;
  }
  if (assessment.disposition == AdmissionDisposition::kRejectSafetyStop) {
    publishAdmissionRejection(
        assessment, message.get(), callback_now.nanoseconds(), callback_steady_ns,
        odometry_freshness.source_age_ms, odometry_freshness.receive_age_ms,
        tracking_envelope.longitudinal_error_m, tracking_envelope.lateral_error_m);
    logTrackingRejection(node().get_logger(), *message, tracking_envelope, *reject_provenance);
    safetyStopNavigation("planner backend PVA command anchor is not near vehicle");
    return;
  }
  if (accepted) {
    finishAcceptedCommand(*message, completed_command, terminal_recovery_needed,
                          terminal_backup_hold_inside_acceptance,
                          terminal_main_hold_inside_acceptance);
  }
}

void NavigationMode::updateBoundary() {
  bool recovery_expired = false;
  bool active = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    active = mode_active_ && !failure_reported_ && !handover_requested_ &&
        !mission_completion_receipt_.has_value();
    if (!active) return;
    recovery_expired = plannerRecoveryWaitExpired(
        planner_recovery_pending_, node().get_clock()->now().nanoseconds(),
        planner_recovery_deadline_ns_);
    if (recovery_expired) clearPlannerRecoveryEpisodeLocked();
  }
  if (recovery_expired) {
    safetyStopNavigation("planner backend terminal endpoint exceeded bounded recovery window");
    return;
  }
  publishStatus(navigation_contracts::msg::NavigationModeStatus::ACTIVE,
                navigation_contracts::msg::NavigationModeStatus::NONE);
}

void NavigationMode::onMissionProgress(
    const navigation_contracts::msg::NavigationMissionProgress::ConstSharedPtr& message) {
  if (!message ||
      message->event != navigation_contracts::msg::NavigationMissionProgress::COMPLETE ||
      !message->waypoint_accepted ||
      message->accepted_waypoint_index != message->waypoint_index ||
      message->mission_id.empty() || message->route_revision == 0U ||
      message->request_id == 0U) {
    return;
  }
  bool accepted = false;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    const auto receipt_stamp_ns =
        navigation_common::rosTimeToNanoseconds(message->header.stamp).value_or(0);
    if (!mode_active_ || failure_reported_ || handover_requested_ ||
        mission_completion_receipt_ ||
        message->mode_activation_id != mode_activation_id_ ||
        message->localization_epoch != lio_localization_epoch_ ||
        message->header.frame_id != planning_frame_ ||
        receipt_stamp_ns < activation_time_.nanoseconds() ||
        (navigation_command_ &&
         (navigation_command_->mission_id != message->mission_id ||
          navigation_command_->waypoint_index != message->waypoint_index ||
          navigation_command_->request_id != message->request_id))) {
      return;
    }
    mission_completion_receipt_ = *message;
    handover_requested_ = true;
    clearPlannerRecoveryEpisodeLocked();
    if (odometry_) {
      const auto& p = odometry_->pose.pose.position;
      completion_position_ = Eigen::Vector3d{p.x, p.y, p.z};
    } else if (navigation_command_) {
      completion_position_ = Eigen::Vector3d{
          navigation_command_->position.x, navigation_command_->position.y,
          navigation_command_->position.z};
    }
    accepted = true;
  }
  if (!accepted) return;
  RCLCPP_INFO(node().get_logger(),
              "Core mission complete receipt: mission=%s waypoint=%u request=%lu",
              message->mission_id.c_str(), message->waypoint_index,
              static_cast<unsigned long>(message->request_id));
  publishStatus(navigation_contracts::msg::NavigationModeStatus::COMPLETE,
                navigation_contracts::msg::NavigationModeStatus::NONE);
  completed(px4_ros2::Result::Success);
}

void NavigationMode::onOdometry(
    const navigation_contracts::msg::PropagatedOdometry::ConstSharedPtr& message) {
  const auto callback_enter_ros_ns = node().get_clock()->now().nanoseconds();
  const auto callback_enter_steady_ns = navigation_common::steadyClockNowNanoseconds();
  navigation_contracts::msg::OdometryTransportTrace timing;
  timing.phase = timing.ADAPTER_CALLBACK;
  timing.callback_enter_ros_ns = callback_enter_ros_ns;
  timing.callback_enter_steady_ns = callback_enter_steady_ns;
  const auto emit_timing = [this, &timing] {
    if (!odometry_timing_publisher_) return;
    try {
      odometry_timing_publisher_->publish(timing);
    } catch (...) {
      // Diagnostic transport cannot change adapter admission or Hold policy.
    }
  };
  if (!message || message->localization_epoch == 0U || message->sequence == 0U) {
    timing.disposition = timing.INVALID_MESSAGE;
    emit_timing();
    return;
  }
  timing.localization_epoch = message->localization_epoch;
  timing.sequence = message->sequence;
  const auto& odometry = message->odometry;
  timing.source_stamp_ros_ns = navigation_common::rosTimeToNanoseconds(
      odometry.header.stamp).value_or(0);
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
    timing.disposition = timing.INVALID_MESSAGE;
    emit_timing();
    return;
  }
  timing.lock_requested_steady_ns = navigation_common::steadyClockNowNanoseconds();
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    timing.lock_acquired_steady_ns = navigation_common::steadyClockNowNanoseconds();
    if (!typed_health_seen_) {
      timing.disposition = timing.HEALTH_NOT_READY;
    } else if (!lio_health_valid_ &&
               !tracking_experiment_.suppress_estimator_health_response) {
      timing.disposition = timing.HEALTH_INVALID;
    } else if (message->localization_epoch != lio_localization_epoch_) {
      timing.disposition = timing.LOCALIZATION_EPOCH_MISMATCH;
    } else if (last_propagated_state_sequence_ > 0U &&
               message->sequence <= last_propagated_state_sequence_) {
      timing.disposition = timing.SEQUENCE_NON_INCREASING;
    } else if (timing.source_stamp_ros_ns <= 0) {
      timing.disposition = timing.SOURCE_TIMESTAMP_INVALID;
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000,
                           "Rejecting non-increasing propagated odometry source timestamp");
    } else if (last_propagated_state_stamp_ns_ > 0 &&
               timing.source_stamp_ros_ns <= last_propagated_state_stamp_ns_) {
      timing.disposition = timing.SOURCE_TIMESTAMP_NON_INCREASING;
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 5000,
                           "Rejecting non-increasing propagated odometry source timestamp");
    } else {
      const auto receive_ns = node().get_clock()->now().nanoseconds();
      if (last_odometry_receive_ns_ > 0 && receive_ns >= last_odometry_receive_ns_) {
        maximum_odometry_callback_gap_us_ = std::max(
            maximum_odometry_callback_gap_us_,
            (receive_ns - last_odometry_receive_ns_) / 1000);
      }
      last_odometry_receive_ns_ = receive_ns;
      last_odometry_receive_steady_ns_ = navigation_common::steadyClockNowNanoseconds();
      odometry_input_trace_ = Px4InputStateTrace{
          message->localization_epoch, message->sequence, timing.source_stamp_ros_ns,
          callback_enter_ros_ns, callback_enter_steady_ns,
          timing.lock_requested_steady_ns, timing.lock_acquired_steady_ns,
          last_odometry_receive_steady_ns_, 0, 0};
      last_propagated_state_stamp_ns_ = timing.source_stamp_ros_ns;
      last_propagated_state_sequence_ = message->sequence;
      ++odometry_callback_count_;
      odometry_ = odometry;
      timing.disposition = timing.ACCEPTED;
      timing.accepted_receive_ros_ns = receive_ns;
      timing.accepted_receive_steady_ns = last_odometry_receive_steady_ns_;
      tryAlignPx4LocalFrameLocked();
    }
  }
  emit_timing();
}

void NavigationMode::tryAlignPx4LocalFrameLocked() {
  if (px4_local_frame_aligned_ || !mode_active_ ||
      !isArmed() || !odometry_ || odometry_->pose.pose.position.z <= 0.5 ||
      !odometry_.has_value() ||
      !px4_local_position_ned_.has_value() || !px4_local_velocity_ned_.has_value() ||
      !last_px4_xy_valid_ || !last_px4_z_valid_ || !last_px4_vxy_valid_ ||
      !last_px4_vz_valid_) {
    return;
  }
  const auto& velocity = odometry_->twist.twist.linear;
  if (Eigen::Vector3d{velocity.x, velocity.y, velocity.z}.norm() > 0.15 ||
      px4_local_velocity_ned_->norm() > 0.15) {
    return;
  }
  const auto& position = odometry_->pose.pose.position;
  const auto translation = localNedTranslationFromStationaryPair(
      Eigen::Vector3d{position.x, position.y, position.z}, *px4_local_position_ned_);
  if (!translation || translation->norm() > 2.0) return;
  lio_to_px4_local_translation_ned_ = *translation;
  px4_local_frame_aligned_ = true;
  ++alignment_latch_generation_;
  publishAlignmentLatchWitnessLocked();
  RCLCPP_INFO(node_.get_logger(),
              "PX4 local frame aligned to LIO: translation_ned=(%.3f,%.3f,%.3f)",
              translation->x(), translation->y(), translation->z());
}

void NavigationMode::onPx4LocalPosition(
    const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr& message) {
  if (!message || !std::isfinite(message->vx) || !std::isfinite(message->vy) ||
      !std::isfinite(message->vz) || !std::isfinite(message->heading) ||
      !std::isfinite(message->heading_var) || message->timestamp == 0U ||
      message->timestamp_sample == 0U) {
    return;
  }
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (px4_local_frame_aligned_ &&
      (message->xy_reset_counter != px4_xy_reset_counter_ ||
       message->z_reset_counter != px4_z_reset_counter_)) {
    px4_local_frame_aligned_ = false;
    lio_to_px4_local_translation_ned_.reset();
    last_px4_position_receive_steady_ns_ = 0;
    RCLCPP_WARN(node_.get_logger(),
                "PX4 local frame reset detected; waiting for stationary re-alignment");
  }
  px4_local_velocity_ned_ = Eigen::Vector3d{message->vx, message->vy, message->vz};
  const auto receive_steady_ns = navigation_common::steadyClockNowNanoseconds();
  const bool position_valid = message->xy_valid && message->z_valid &&
      std::isfinite(message->x) && std::isfinite(message->y) &&
      std::isfinite(message->z);
  if (position_valid) {
    px4_local_position_ned_ = Eigen::Vector3d{message->x, message->y, message->z};
    last_px4_position_receive_steady_ns_ = receive_steady_ns;
  } else {
    // Preserve the last alignment snapshot for the legacy position boundary,
    // but velocity-only control consumes the independent velocity/heading
    // witness below and does not use this position.
    if (!px4_local_frame_aligned_) {
      px4_local_position_ned_.reset();
    }
  }
  px4_xy_reset_counter_ = message->xy_reset_counter;
  px4_z_reset_counter_ = message->z_reset_counter;
  px4_vxy_reset_counter_ = message->vxy_reset_counter;
  px4_vz_reset_counter_ = message->vz_reset_counter;
  px4_heading_reset_counter_ = message->heading_reset_counter;
  last_px4_local_position_timestamp_us_ = message->timestamp;
  last_px4_local_position_timestamp_sample_us_ = message->timestamp_sample;
  last_px4_local_position_receive_ns_ = node().get_clock()->now().nanoseconds();
  last_px4_xy_valid_ = message->xy_valid;
  last_px4_z_valid_ = message->z_valid;
  last_px4_vxy_valid_ = message->v_xy_valid;
  last_px4_vz_valid_ = message->v_z_valid;
  last_px4_dead_reckoning_ = message->dead_reckoning;
  last_px4_heading_good_for_control_ = message->heading_good_for_control;
  last_px4_heading_valid_ = std::isfinite(message->heading);
  last_px4_heading_ned_ = message->heading;
  last_px4_heading_variance_rad2_ = message->heading_var;
  last_px4_delta_xy_north_m_ = message->delta_xy[0];
  last_px4_delta_xy_east_m_ = message->delta_xy[1];
  last_px4_delta_z_m_ = message->delta_z;
  last_px4_delta_heading_rad_ = message->delta_heading;
  // This lease covers the complete VehicleLocalPosition witness (including
  // velocity/heading). Position alignment has its own validity-qualified lease
  // above, so an XY/Z-invalid packet cannot keep a legacy PVA position alive.
  last_px4_local_position_receive_steady_ns_ = receive_steady_ns;
  tryAlignPx4LocalFrameLocked();
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
  if (source_stamp_ns <= 0) {
    lio_health_valid_ = false;
    last_health_receive_steady_ns_ = 0;
    return;
  }
  // Corrected-state and propagated-state health are emitted by independent
  // producer threads. A late correction sample must not invalidate a newer
  // propagated health assessment merely because it loses the delivery race.
  if (last_lio_diagnostics_ns_ > 0 &&
      source_stamp_ns <= last_lio_diagnostics_ns_) {
    return;
  }
  last_health_state_ = message->state;
  last_health_navigation_valid_ = message->navigation_valid;
  last_health_covariance_valid_ = message->covariance_valid;
  last_health_observability_valid_ = message->observability_valid;
  last_health_correction_fresh_ = message->correction_fresh;
  last_health_propagation_valid_ = message->propagation_valid;
  last_health_source_stamp_ns_ = source_stamp_ns;
  last_health_correction_stamp_ns_ = navigation_common::rosTimeToNanoseconds(
      message->last_correction_stamp).value_or(0);
  last_health_propagated_stamp_ns_ = navigation_common::rosTimeToNanoseconds(
      message->last_propagated_state_stamp).value_or(0);
  last_health_receive_steady_ns_ = navigation_common::steadyClockNowNanoseconds();
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
    odometry_input_trace_ = {};
    last_propagated_state_stamp_ns_ = 0;
    last_propagated_state_sequence_ = 0U;
    px4_local_frame_aligned_ = false;
    lio_to_px4_local_translation_ned_.reset();
    velocity_only_previous_.reset();
    velocity_only_reset_counters_seen_ = false;
  }
  lio_localization_epoch_ = message->localization_epoch;
  lio_health_valid_ = healthy;
  last_lio_diagnostics_ns_ = source_stamp_ns;
}

void NavigationMode::requestVelocityOnlyHold(const char* reason) {
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (odometry_.has_value()) {
      const auto& point = odometry_->pose.pose.position;
      const Eigen::Vector3d measured{point.x, point.y, point.z};
      if (measured.allFinite()) safety_hold_position_ = measured;
    }
    velocity_only_last_reason_ = reason;
    velocity_only_previous_.reset();
    velocity_only_reset_counters_seen_ = false;
    handover_requested_ = true;
    navigation_command_ = transitionCertifiedCommand(
        navigation_command_, std::nullopt, CertifiedCommandTransition::kInvalidate);
  }
  publishStatus(navigation_contracts::msg::NavigationModeStatus::PAUSED,
                navigation_contracts::msg::NavigationModeStatus::SAFETY_STOP);
  RCLCPP_WARN(node().get_logger(),
              "velocity-only experiment requires explicit PX4 Hold handover: %s", reason);
  if (px4_hold_handover_) {
    px4_hold_handover_();
  } else {
    completed(px4_ros2::Result::ModeFailureOther);
  }
}

bool NavigationMode::publishVelocityOnlySetpoint(
    const navigation_contracts::msg::NavigationCommand& command,
    const VelocityOnlySnapshot& snapshot, const rclcpp::Time& now) {
  using tracking_adapter::CommandRole;
  using tracking_adapter::Mode;
  using tracking_adapter::Policy;
  using tracking_adapter::Reference;
  using tracking_adapter::ReferenceFrame;
  using tracking_adapter::TimingWitness;
  using tracking_adapter::LioState;

  const bool certified_emergency =
      command.role == navigation_contracts::msg::NavigationCommand::ROLE_EMERGENCY &&
      command.status == navigation_contracts::msg::NavigationCommand::STATUS_BRAKING &&
      command.emergency_authorization_reason ==
          navigation_contracts::msg::NavigationCommand::EMERGENCY_AUTHORIZATION_ACTUAL_ANCHOR_CERTIFICATE_EXCEEDED &&
      command.emergency_candidate_commit_result == 1U;
  if (command.role != navigation_contracts::msg::NavigationCommand::ROLE_MAIN &&
      command.role != navigation_contracts::msg::NavigationCommand::ROLE_BACKUP &&
      !certified_emergency) {
    setVelocityOnlyLastReason("role_not_authorized_for_velocity_boundary");
    return false;
  }
  if (command.status != navigation_contracts::msg::NavigationCommand::STATUS_READY) {
    setVelocityOnlyLastReason("status_requires_native_px4_hold");
    return false;
  }

  const auto source_stamp_ns = navigation_common::rosTimeToNanoseconds(
      snapshot.odometry.header.stamp).value_or(0);
  // NavigationCommand.state_source_stamp identifies the measured LIO state;
  // the PVA reference itself was evaluated at the command header stamp.
  const auto reference_stamp_ns = navigation_contracts::commandStampNanoseconds(
      command.header.stamp);
  const auto lease_until_ns = navigation_contracts::commandStampNanoseconds(command.valid_until);
  const auto now_ns = now.nanoseconds();
  const auto now_steady_ns = navigation_common::steadyClockNowNanoseconds();
  if (source_stamp_ns <= 0 || reference_stamp_ns <= 0 || lease_until_ns <= 0 || now_ns <= 0 ||
      now_ns < source_stamp_ns || now_ns < reference_stamp_ns || now_steady_ns <= 0) {
    setVelocityOnlyLastReason("timestamp_contract_invalid");
    return false;
  }

  Eigen::Vector3d lio_position{snapshot.odometry.pose.pose.position.x,
                               snapshot.odometry.pose.pose.position.y,
                               snapshot.odometry.pose.pose.position.z};
  const Eigen::Quaterniond orientation(
      snapshot.odometry.pose.pose.orientation.w, snapshot.odometry.pose.pose.orientation.x,
      snapshot.odometry.pose.pose.orientation.y, snapshot.odometry.pose.pose.orientation.z);
  if (!isNormalizableOdometryQuaternion(orientation)) {
    setVelocityOnlyLastReason("lio_orientation_invalid");
    return false;
  }
  const auto normalized = orientation.normalized();
  // nav_msgs/Odometry expresses twist in child_frame_id (base_link/FLU).
  // The adapter contract is LIO world ENU, matching the planner reference.
  const Eigen::Vector3d lio_velocity = normalized * Eigen::Vector3d{
      snapshot.odometry.twist.twist.linear.x,
      snapshot.odometry.twist.twist.linear.y,
      snapshot.odometry.twist.twist.linear.z};
  const double lio_yaw = std::atan2(
      2.0 * (normalized.w() * normalized.z() + normalized.x() * normalized.y()),
      1.0 - 2.0 * (normalized.y() * normalized.y() + normalized.z() * normalized.z()));

  const auto& raw_px4 = snapshot.px4;
  const auto current_lio_epoch = snapshot.lio_localization_epoch;
  const auto current_lio_sequence = snapshot.lio_sequence;
  const auto current_lio_receive_steady_ns = snapshot.lio_receive_steady_ns;
  const auto px4_sample_ns = raw_px4.timestamp_sample_us <=
          static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000)
      ? static_cast<std::int64_t>(raw_px4.timestamp_sample_us * 1000U)
      : 0;
  if (px4_sample_ns <= 0 || now_ns < px4_sample_ns) {
    setVelocityOnlyLastReason("px4_timestamp_contract_invalid");
    return false;
  }
  if (raw_px4.reset_counters != snapshot.last_reset_counters &&
      snapshot.reset_counters_seen) {
    setVelocityOnlyLastReason("px4_reset_requires_new_velocity_epoch");
    return false;
  }

  Reference reference;
  reference.frame = ReferenceFrame::kLioEnu;
  reference.identity.localization_epoch = command.localization_epoch;
  reference.identity.mission_id = command.mission_id;
  reference.identity.waypoint_index = command.waypoint_index;
  reference.identity.goal_epoch = command.goal_epoch;
  reference.identity.request_id = command.request_id;
  reference.identity.bundle_generation = command.bundle_generation;
  reference.identity.sample_id = command.sample_id;
  reference.identity.role = command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN
      ? CommandRole::kMain
      : command.role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP
      ? CommandRole::kBackup
      : CommandRole::kEmergency;
  reference.identity.reference_sample_time_ns = reference_stamp_ns;
  reference.identity.lease_valid_until_ns = lease_until_ns;
  reference.position_enu = Eigen::Vector3d{command.position.x, command.position.y,
                                           command.position.z};
  reference.velocity_enu = Eigen::Vector3d{command.velocity.x, command.velocity.y,
                                           command.velocity.z};
  reference.acceleration_enu = Eigen::Vector3d{command.acceleration.x, command.acceleration.y,
                                               command.acceleration.z};
  reference.yaw_enu = command.yaw;
  reference.yaw_rate_enu_rad_s = command.yaw_rate;

  LioState lio;
  lio.position_enu = lio_position;
  lio.velocity_enu = lio_velocity;
  lio.yaw_enu = lio_yaw;
  lio.position_valid = lio_position.allFinite();
  lio.velocity_valid = lio_velocity.allFinite();
  lio.orientation_valid = orientation.coeffs().allFinite();
  lio.localization_epoch = current_lio_epoch;
  lio.sequence = current_lio_sequence;
  lio.source_stamp_ns = source_stamp_ns;
  lio.receive_steady_ns = current_lio_receive_steady_ns;
  lio.navigation_valid = snapshot.health_navigation_valid;
  lio.covariance_valid = snapshot.health_covariance_valid;
  lio.observability_valid = snapshot.health_observability_valid;
  lio.correction_fresh = snapshot.health_correction_fresh;
  lio.propagation_valid = snapshot.health_propagation_valid;
  lio.relative_heading_valid = raw_px4.heading_valid && raw_px4.heading_good_for_control;
  lio.tilt_valid = orientation.coeffs().allFinite();
  lio.extrinsic_valid = true;

  TimingWitness timing;
  timing.reference_sample_id = command.sample_id;
  timing.lio_localization_epoch = lio.localization_epoch;
  timing.lio_sequence = lio.sequence;
  timing.px4_timestamp_us = raw_px4.timestamp_us;
  timing.px4_timestamp_sample_us = raw_px4.timestamp_sample_us;
  timing.clock_mapping_generation = 1U;
  timing.conservative_bound_model = TimingWitness::kConservativeBoundModelV1;
  timing.common_time_contract_valid = source_stamp_ns <= now_ns &&
      reference_stamp_ns <= now_ns && px4_sample_ns <= now_ns;
  timing.expected_reference_use_time_ns = now_ns;
  timing.reference_age_s = static_cast<double>(now_ns - reference_stamp_ns) * 1.0e-9;
  timing.pair_skew_s = std::abs(static_cast<double>(source_stamp_ns - px4_sample_ns)) * 1.0e-9;
  timing.lio_source_age_s = static_cast<double>(now_ns - source_stamp_ns) * 1.0e-9;
  timing.lio_receive_age_s = current_lio_receive_steady_ns > 0 &&
      now_steady_ns >= current_lio_receive_steady_ns
      ? static_cast<double>(now_steady_ns - current_lio_receive_steady_ns) * 1.0e-9 : -1.0;
  timing.px4_source_age_s = static_cast<double>(now_ns - px4_sample_ns) * 1.0e-9;
  timing.px4_receive_age_s = raw_px4.receive_steady_ns > 0 &&
      now_steady_ns >= raw_px4.receive_steady_ns
      ? static_cast<double>(now_steady_ns - raw_px4.receive_steady_ns) * 1.0e-9 : -1.0;
  timing.predicted_anchor_age_s = std::max(timing.lio_source_age_s, timing.px4_source_age_s);
  timing.output_transport_age_s = tracking_experiment_.velocity_only_output_transport_bound_s;
  timing.px4_consume_age_s = tracking_experiment_.velocity_only_px4_consume_bound_s;
  const double overlap = std::max({timing.reference_age_s, timing.pair_skew_s,
      timing.lio_source_age_s, timing.lio_receive_age_s, timing.px4_source_age_s,
      timing.px4_receive_age_s, timing.predicted_anchor_age_s});
  timing.total_bound_s = overlap + timing.output_transport_age_s + timing.px4_consume_age_s;

  Policy adapter_policy;
  adapter_policy.mode = Mode::kLevelA;
  adapter_policy.boundary = tracking_adapter::SetpointBoundary::kVelocityOnly;
  adapter_policy.experiment_id = "velocity-only-lio-owned-v1";
  adapter_policy.lio_position_feedback_gain_s_inv = tracking_experiment_.velocity_only_gain_s_inv;
  adapter_policy.maximum_velocity_mps = tracking_experiment_.velocity_only_cap_mps;
  adapter_policy.maximum_timing_bound_s = tracking_experiment_.velocity_only_max_timing_bound_s;
  adapter_policy.maximum_reference_age_s = tracking_experiment_.velocity_only_max_reference_age_s;
  adapter_policy.expected_px4_reset_counters = raw_px4.reset_counters;
  adapter_policy.expected_lio_localization_epoch = lio.localization_epoch;

  const auto adapted = tracking_adapter::adapt(reference, lio, raw_px4, timing, adapter_policy);
  if (!adapted.success()) {
    setVelocityOnlyLastReason("adapter_rejected");
    return false;
  }
  velocity_only::Identity continuity_identity{
      command.mission_id, command.waypoint_index, command.request_id,
      command.bundle_generation, reference.identity.role == CommandRole::kMain
          ? velocity_only::Role::kMain
          : reference.identity.role == CommandRole::kBackup
          ? velocity_only::Role::kBackup : velocity_only::Role::kEmergency};
  velocity_only::Policy continuity_policy{
      tracking_experiment_.velocity_only_cap_mps,
      tracking_experiment_.velocity_only_max_acceleration_mps2,
      tracking_experiment_.velocity_only_max_jerk_mps3};
  const std::optional<velocity_only::Previous>& previous = snapshot.previous;
  const auto limited = velocity_only::limit(
      adapted.output->witness.velocity_command_lio_enu, now_ns, continuity_identity,
      continuity_policy, previous ? &*previous : nullptr);
  if (!limited.success()) {
    setVelocityOnlyLastReason(
        std::string("continuity_rejected:") + velocity_only::failureName(limited.failure));
    RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "velocity-only continuity rejected: failure=%s requested=(%.3f,%.3f,%.3f) "
        "previous_v=(%.3f,%.3f,%.3f) previous_a=(%.3f,%.3f,%.3f) dt=%.6f "
        "residual_v=%.6f residual_viability_v=%.6f residual_a=%.6f residual_jerk=%.6f "
        "projection_iterations=%u converged=%s",
        velocity_only::failureName(limited.failure), limited.requested_velocity_enu.x(),
        limited.requested_velocity_enu.y(), limited.requested_velocity_enu.z(),
        limited.previous_velocity_enu.x(), limited.previous_velocity_enu.y(),
        limited.previous_velocity_enu.z(), limited.previous_acceleration_enu.x(),
        limited.previous_acceleration_enu.y(), limited.previous_acceleration_enu.z(),
        limited.delta_s, limited.velocity_residual_mps,
        limited.velocity_viability_residual_mps, limited.acceleration_residual_mps2,
        limited.jerk_residual_mps3, limited.projection_iterations,
        limited.projection_converged ? "true" : "false");
    return false;
  }
  const Eigen::Vector3d velocity_ned_value =
      adapted.output->witness.rotation_lio_enu_to_px4_ned * limited.velocity_enu;
  std::optional<Eigen::Vector3f> velocity_ned;
  if (velocity_ned_value.allFinite() &&
      (velocity_ned_value.cwiseAbs().array() <=
       static_cast<double>(std::numeric_limits<float>::max())).all()) {
    velocity_ned = velocity_ned_value.cast<float>();
  }
  if (!velocity_ned || !floatRepresentable(adapted.output->yaw_ned) ||
      !floatRepresentable(adapted.output->yaw_rate_ned_rad_s)) {
    setVelocityOnlyLastReason("velocity_or_yaw_not_representable");
    return false;
  }

  px4_ros2::TrajectorySetpoint setpoint;
  setpoint.withVelocity(*velocity_ned)
      .withYaw(static_cast<float>(adapted.output->yaw_ned))
      .withYawRate(static_cast<float>(adapted.output->yaw_rate_ned_rad_s));
  std::string trace_velocity_only_reason;
  std::uint64_t trace_velocity_only_limited_count = 0U;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    velocity_only_previous_ = velocity_only::Previous{
        continuity_identity, limited.velocity_enu, limited.acceleration_enu, now_ns};
    velocity_only_last_reset_counters_ = raw_px4.reset_counters;
    velocity_only_reset_counters_seen_ = true;
    velocity_only_last_reason_ = limited.limited ? "continuity_limited" : "accepted";
    if (limited.limited) ++velocity_only_limited_count_;
    trace_velocity_only_reason = velocity_only_last_reason_;
    trace_velocity_only_limited_count = velocity_only_limited_count_;
    last_velocity_command_enu_ = limited.velocity_enu;
    last_setpoint_time_ = now;
  }
  auto trace = makePx4InputTraceRecord(
      &command,
      std::nullopt, velocity_ned, std::nullopt,
      setpoint.yaw_ned_rad.value_or(NAN), setpoint.yaw_rate_ned_rad_s.value_or(NAN),
      Px4InputTraceBoundary::kVelocityOnly, 0, 0,
      trace_velocity_only_reason, trace_velocity_only_limited_count,
      snapshot.state_input_trace);
  trace.update_start_ros_ns = node().get_clock()->now().nanoseconds();
  trace.update_start_steady_ns = navigation_common::steadyClockNowNanoseconds();
  trajectory_setpoint_->update(setpoint);
  trace.update_end_ros_ns = node().get_clock()->now().nanoseconds();
  trace.update_end_steady_ns = navigation_common::steadyClockNowNanoseconds();
  enqueuePx4InputTrace(trace);
  return true;
}

void NavigationMode::logRuntimeMetrics(const rclcpp::Time& now) {
  const auto now_ns = now.nanoseconds();
  std::uint64_t odometry_callbacks;
  std::uint64_t trajectories_received;
  std::uint64_t trajectories_accepted;
  std::uint64_t trajectories_rejected;
  std::array<std::uint64_t, 8> admission_rejections_by_stage;
  std::uint64_t experimental_tracking_suppressed;
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
    admission_rejections_by_stage = admission_rejections_by_stage_;
    experimental_tracking_suppressed = experimental_tracking_suppressed_count_;
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
              "forward_guard_count=%lu experimental_tracking_suppressed=%lu "
              "admission_reject_by_stage=[%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu]",
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
              static_cast<unsigned long>(forward_guard_count),
              static_cast<unsigned long>(experimental_tracking_suppressed),
              static_cast<unsigned long>(admission_rejections_by_stage[0]),
              static_cast<unsigned long>(admission_rejections_by_stage[1]),
              static_cast<unsigned long>(admission_rejections_by_stage[2]),
              static_cast<unsigned long>(admission_rejections_by_stage[3]),
              static_cast<unsigned long>(admission_rejections_by_stage[4]),
              static_cast<unsigned long>(admission_rejections_by_stage[5]),
              static_cast<unsigned long>(admission_rejections_by_stage[6]),
              static_cast<unsigned long>(admission_rejections_by_stage[7]));
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
  std::optional<VelocityOnlySnapshot> velocity_only_snapshot;
  std::string trace_velocity_only_reason;
  std::uint64_t trace_velocity_only_limited_count = 0U;
  std::int64_t odometry_receive_steady_ns = 0;
  Px4InputStateTrace state_input_trace;
  std::optional<Eigen::Vector3d> lio_to_px4_local_translation_ned;
  const auto now = node().get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    navigation_command = navigation_command_;
    odometry = odometry_;
    state_input_trace = odometry_input_trace_;
    state_input_trace.snapshot_ros_ns = node().get_clock()->now().nanoseconds();
    state_input_trace.snapshot_steady_ns = navigation_common::steadyClockNowNanoseconds();
    trace_velocity_only_reason = velocity_only_last_reason_;
    trace_velocity_only_limited_count = velocity_only_limited_count_;
    odometry_receive_steady_ns = last_odometry_receive_steady_ns_;
    if (tracking_experiment_.velocity_only_enabled && odometry_.has_value()) {
      VelocityOnlySnapshot snapshot;
      snapshot.odometry = *odometry_;
      snapshot.state_input_trace = state_input_trace;
      snapshot.px4.position_ned = px4_local_position_ned_.value_or(
          Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
      snapshot.px4.velocity_ned = px4_local_velocity_ned_.value_or(
          Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
      snapshot.px4.yaw_ned = last_px4_heading_ned_;
      snapshot.px4.position_valid = {last_px4_xy_valid_, last_px4_xy_valid_, last_px4_z_valid_};
      snapshot.px4.velocity_valid = {last_px4_vxy_valid_, last_px4_vxy_valid_, last_px4_vz_valid_};
      snapshot.px4.heading_valid = last_px4_heading_valid_;
      snapshot.px4.heading_good_for_control = last_px4_heading_good_for_control_;
      snapshot.px4.dead_reckoning = last_px4_dead_reckoning_;
      snapshot.px4.heading_variance_rad2 = last_px4_heading_variance_rad2_;
      snapshot.px4.timestamp_us = last_px4_local_position_timestamp_us_;
      snapshot.px4.timestamp_sample_us = last_px4_local_position_timestamp_sample_us_;
      snapshot.px4.receive_steady_ns = last_px4_local_position_receive_steady_ns_;
      snapshot.px4.reset_counters = tracking_adapter::ResetCounters{
          px4_xy_reset_counter_, px4_z_reset_counter_, px4_vxy_reset_counter_,
          px4_vz_reset_counter_, px4_heading_reset_counter_};
      snapshot.previous = velocity_only_previous_;
      snapshot.last_reset_counters = velocity_only_last_reset_counters_;
      snapshot.reset_counters_seen = velocity_only_reset_counters_seen_;
      snapshot.lio_localization_epoch = lio_localization_epoch_;
      snapshot.lio_sequence = last_propagated_state_sequence_;
      snapshot.lio_receive_steady_ns = last_odometry_receive_steady_ns_;
      snapshot.health_navigation_valid = last_health_navigation_valid_;
      snapshot.health_covariance_valid = last_health_covariance_valid_;
      snapshot.health_observability_valid = last_health_observability_valid_;
      snapshot.health_correction_fresh = last_health_correction_fresh_;
      snapshot.health_propagation_valid = last_health_propagation_valid_;
      velocity_only_snapshot = std::move(snapshot);
    }
    const auto now_steady_ns = navigation_common::steadyClockNowNanoseconds();
    if (px4_local_frame_aligned_ && lio_to_px4_local_translation_ned_.has_value() &&
        last_px4_position_receive_steady_ns_ > 0 &&
        now_steady_ns >= last_px4_position_receive_steady_ns_ &&
        now_steady_ns - last_px4_position_receive_steady_ns_ <=
            state_stale_after_ns_) {
      lio_to_px4_local_translation_ned = lio_to_px4_local_translation_ned_;
    }
  }
  std::optional<float> current_px4_yaw_ned;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (tracking_experiment_.velocity_only_enabled && last_px4_heading_valid_ &&
        last_px4_heading_good_for_control_ && floatRepresentable(last_px4_heading_ned_)) {
      current_px4_yaw_ned = static_cast<float>(last_px4_heading_ned_);
    }
  }
  const auto lioPositionToPx4Ned = [&](const Eigen::Vector3d& position_enu) {
    if (!lio_to_px4_local_translation_ned.has_value()) {
      return std::optional<Eigen::Vector3f>{};
    }
    const auto position_ned = lioPositionToLocalNed(
        position_enu, *lio_to_px4_local_translation_ned);
    if (!position_ned ||
        (position_ned->cwiseAbs().array() >
         static_cast<double>(std::numeric_limits<float>::max())).any()) {
      return std::optional<Eigen::Vector3f>{};
    }
    return std::optional<Eigen::Vector3f>{position_ned->cast<float>()};
  };
  bool stationary_position_unrepresentable = false;
  const auto publishStationary = [&](const std::optional<Eigen::Vector3d>& position_enu) {
    px4_ros2::TrajectorySetpoint setpoint;
    if (!position_enu.has_value()) {
      setpoint.withVelocity(Eigen::Vector3f::Zero());
    } else {
      if (const auto position_ned = lioPositionToPx4Ned(*position_enu)) {
        setpoint.withPosition(*position_ned);
        setpoint.withAcceleration(Eigen::Vector3f::Zero());
        setpoint.withVelocity(Eigen::Vector3f::Zero());
      } else {
        stationary_position_unrepresentable = true;
      }
    }
    setpoint.withVelocity(Eigen::Vector3f::Zero());
    if (tracking_experiment_.velocity_only_enabled) {
      // Velocity-only commands use the adapter's accepted PX4-relative yaw.
      // A stationary handover must retain that frame; if PX4 has no accepted
      // heading, omit yaw instead of injecting an ENU->NED pi/2 jump.
      if (current_px4_yaw_ned.has_value()) {
        setpoint.withYaw(*current_px4_yaw_ned).withYawRate(0.0F);
      }
    } else if (odometry.has_value()) {
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
    const std::optional<Eigen::Vector3f> trace_acceleration =
        tracking_experiment_.velocity_only_enabled
        ? std::nullopt
        : std::optional<Eigen::Vector3f>{Eigen::Vector3f::Zero()};
    auto trace = makePx4InputTraceRecord(
        navigation_command ? &*navigation_command : nullptr,
        std::nullopt, Eigen::Vector3f::Zero(), trace_acceleration,
        setpoint.yaw_ned_rad.value_or(NAN), setpoint.yaw_rate_ned_rad_s.value_or(NAN),
        Px4InputTraceBoundary::kVelocityHold, 0, 0,
        trace_velocity_only_reason, trace_velocity_only_limited_count, state_input_trace);
    trace.update_start_ros_ns = node().get_clock()->now().nanoseconds();
    trace.update_start_steady_ns = navigation_common::steadyClockNowNanoseconds();
    trajectory_setpoint_->update(setpoint);
    trace.update_end_ros_ns = node().get_clock()->now().nanoseconds();
    trace.update_end_steady_ns = navigation_common::steadyClockNowNanoseconds();
    enqueuePx4InputTrace(trace);
  };
  const auto publishPositionHold = [&](const Eigen::Vector3d& position_enu) {
    // Velocity-only is the normal flight output, but a terminal hold needs a
    // bounded position reference as well as zero velocity.  Sending only a
    // zero-velocity setpoint leaves PX4 free to drift in altitude and makes
    // the external velocity tracker the sole source of position correction;
    // use PX4's position controller for this short, latched terminal hold.
    px4_ros2::TrajectorySetpoint setpoint;
    const auto position_ned = lioPositionToPx4Ned(position_enu);
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
    auto trace = makePx4InputTraceRecord(
        navigation_command ? &*navigation_command : nullptr,
        position_ned, Eigen::Vector3f::Zero(), std::nullopt,
        setpoint.yaw_ned_rad.value_or(NAN), setpoint.yaw_rate_ned_rad_s.value_or(NAN),
        Px4InputTraceBoundary::kPositionHold, 0, 0,
        trace_velocity_only_reason, trace_velocity_only_limited_count, state_input_trace);
    trace.update_start_ros_ns = node().get_clock()->now().nanoseconds();
    trace.update_start_steady_ns = navigation_common::steadyClockNowNanoseconds();
    trajectory_setpoint_->update(setpoint);
    trace.update_end_ros_ns = node().get_clock()->now().nanoseconds();
    trace.update_end_steady_ns = navigation_common::steadyClockNowNanoseconds();
    enqueuePx4InputTrace(trace);
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
    if (!terminal_stationary_setpoint && mission_completion_receipt_) {
      publishStationary(completion_position_);
      last_setpoint_time_ = now;
      terminal_stationary_setpoint = true;
    }
    if (!terminal_stationary_setpoint && handover_requested_) {
      std::optional<Eigen::Vector3d> handover_position;
      if (safety_hold_position_.has_value()) {
        handover_position = safety_hold_position_;
      } else if (odometry) {
        const auto& p = odometry->pose.pose.position;
        handover_position = Eigen::Vector3d{p.x, p.y, p.z};
      }
      publishStationary(handover_position);
      last_setpoint_time_ = now;
      terminal_stationary_setpoint = true;
    }
  }
  if (terminal_stationary_setpoint) {
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      // Keep runtime metrics aligned with the actual zero-velocity safety
      // setpoint; do not report the last moving command while the mode is
      // already in its terminal handover stream.
      last_velocity_command_enu_.setZero();
    }
    if (stationary_position_unrepresentable) {
      safetyStopNavigation("terminal hold position is not representable by PX4");
    }
    return;
  }
  const double since_activation_s = (now - activation_time_).seconds();
  if (!isArmed() || !odometry || odometry->pose.pose.position.z <= 0.5) {
    publishStationary(std::nullopt);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (airborne_start_ns_ == 0) airborne_start_ns_ = now.nanoseconds();
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
  bool lio_healthy = false;
  bool typed_health_seen = false;
  std::int64_t health_source_stamp_ns = 0;
  std::int64_t health_receive_steady_ns = 0;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    lio_healthy = lio_health_valid_;
    typed_health_seen = typed_health_seen_;
    health_source_stamp_ns = last_health_source_stamp_ns_;
    health_receive_steady_ns = last_health_receive_steady_ns_;
  }
  const auto health_freshness = navigation_contracts::evaluateExecutionStateFreshness(
      now.nanoseconds(), health_source_stamp_ns,
      navigation_common::steadyClockNowNanoseconds(), health_receive_steady_ns,
      state_stale_after_s_);
  // The first diagnostics sample can legitimately be in flight when PX4
  // hands control to the mode. Hold zero velocity for a short bounded
  // acquisition window; once a sample exists, any unhealthy value is an
  // immediate fail-closed event and a stale healthy sample uses the normal
  // finite health timeout.
  const bool diagnostics_missing = !typed_health_seen || health_source_stamp_ns <= 0 ||
      health_receive_steady_ns <= 0;
  const double diagnostics_wait_s = std::min(0.5, std::max(0.0, trajectory_wait_timeout_s_));
  if (diagnostics_missing && !typed_health_seen && since_activation_s <= diagnostics_wait_s) {
    publishStationary(std::nullopt);
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    last_setpoint_time_ = now;
    return;
  }
  const bool suppress_fresh_typed_unhealthy =
      tracking_experiment_.suppress_estimator_health_response &&
      !diagnostics_missing && !lio_healthy && health_freshness.valid();
  if (((!diagnostics_missing && !lio_healthy) &&
       !suppress_fresh_typed_unhealthy) ||
      (!diagnostics_missing && !health_freshness.valid()) ||
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
                     : health_freshness.source_age_ms,
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
  if (suppress_fresh_typed_unhealthy) {
    RCLCPP_WARN_THROTTLE(
        node().get_logger(), *node().get_clock(), 1000,
        "TRACKING_EXPERIMENT_BYPASS role=FRESH_TYPED_FAST_LIO_HEALTH "
        "health_age_ms=%.3f; estimator and mapping remain active; "
        "stale/missing health and epoch mismatch remain fail-closed",
        health_freshness.source_age_ms);
  }

  // Planner command path. The upstream planner has already selected the
  // polynomial and MAIN/BACKUP role. PX4 receives that PVA state directly;
  // applying a second velocity controller here would change the planner
  // trajectory and reintroduce the old terminal oscillation.
  if (navigation_command.has_value()) {
    bool terminal_recovery_window_open = false;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      terminal_recovery_window_open = terminalRecoveryCommandMayBeHeld(
          navigation_command->status ==
              navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED,
          planner_recovery_pending_, now.nanoseconds(), planner_recovery_deadline_ns_);
      terminal_recovery_window_open = terminal_recovery_window_open &&
          plannerRecoveryEpisodeMatchesLocked(*navigation_command);
    }
    const auto receive_ns = [&]() {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      return last_command_receive_ns_;
    }();
    if (receive_ns > 0 && now.nanoseconds() >= receive_ns &&
        now.nanoseconds() - receive_ns > stale_after_ns_ &&
        !terminal_recovery_window_open) {
      safetyStopNavigation("planner backend PVA command stale");
      return;
    }
    const auto& command = *navigation_command;
    const auto command_stamp_ns =
        navigation_common::rosTimeToNanoseconds(command.header.stamp).value_or(0);
    if (command_stamp_ns <= 0 || command_stamp_ns > now.nanoseconds() ||
        (now.nanoseconds() >= command_stamp_ns &&
         now.nanoseconds() - command_stamp_ns > stale_after_ns_ &&
         !terminal_recovery_window_open)) {
      safetyStopNavigation("planner backend PVA command timestamp invalid or stale");
      return;
    }
    if (!terminal_recovery_window_open &&
        !navigation_contracts::commandValidAt(command, now.nanoseconds())) {
      safetyStopNavigation("planner backend command validity window expired");
      return;
    }
    const Eigen::Vector3d position_enu{command.position.x, command.position.y,
                                       command.position.z};
    const Eigen::Vector3d velocity_enu{command.velocity.x, command.velocity.y,
                                       command.velocity.z};
    const Eigen::Vector3d acceleration_enu{command.acceleration.x, command.acceleration.y,
                                            command.acceleration.z};
    bool terminal_endpoint_anchored = false;
    if (command.status ==
            navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED &&
        odometry.has_value()) {
      const auto& p = odometry->pose.pose.position;
      const Eigen::Vector3d measured{p.x, p.y, p.z};
      terminal_endpoint_anchored = measured.allFinite() && position_enu.allFinite() &&
          (measured - position_enu).norm() <= navigation_contracts::kCommandAnchorErrorLimitM;
    }
    if (command.status ==
        navigation_contracts::msg::NavigationCommand::STATUS_REJECTED) {
      safetyStopNavigation("planner backend planner failed without a valid backup trajectory");
      return;
    }
    if (command.status ==
        navigation_contracts::msg::NavigationCommand::STATUS_COMPLETED) {
      if (tracking_experiment_.velocity_only_enabled) {
        requestVelocityOnlyHold("terminal command requires native PX4 Hold");
        return;
      }
      if (!publishPositionHold(position_enu)) {
        safetyStopNavigation("completed command position is not representable by PX4");
        return;
      }
      if ((command.role == navigation_contracts::msg::NavigationCommand::ROLE_BACKUP ||
           command.role == navigation_contracts::msg::NavigationCommand::ROLE_MAIN) &&
          !terminal_endpoint_anchored && !mission_completion_receipt_) {
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
              rememberPlannerRecoveryEpisodeLocked(*navigation_command);
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
    if (tracking_experiment_.velocity_only_enabled) {
      if (!velocity_only_snapshot.has_value() ||
          !publishVelocityOnlySetpoint(command, *velocity_only_snapshot, now)) {
        std::string reason;
        {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          reason = velocity_only_last_reason_.empty()
              ? "velocity-only setpoint unavailable" : velocity_only_last_reason_;
        }
        requestVelocityOnlyHold(reason.c_str());
      }
      return;
    }
    const auto position_ned = lioPositionToPx4Ned(position_enu);
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
    auto trace = makePx4InputTraceRecord(
        navigation_command ? &*navigation_command : nullptr,
        position_ned, velocity_ned, acceleration_ned,
        setpoint.yaw_ned_rad.value_or(NAN), setpoint.yaw_rate_ned_rad_s.value_or(NAN),
        Px4InputTraceBoundary::kTracking, 0, 0,
        trace_velocity_only_reason, trace_velocity_only_limited_count, state_input_trace);
    trace.update_start_ros_ns = node().get_clock()->now().nanoseconds();
    trace.update_start_steady_ns = navigation_common::steadyClockNowNanoseconds();
    trajectory_setpoint_->update(setpoint);
    trace.update_end_ros_ns = node().get_clock()->now().nanoseconds();
    trace.update_end_steady_ns = navigation_common::steadyClockNowNanoseconds();
    enqueuePx4InputTrace(trace);
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      last_velocity_command_enu_ = velocity_enu;
      last_setpoint_time_ = now;
    }
    return;
  }

  // Airborne activation and the first Core command are asynchronous. The
  // acquisition window is measured from the PX4-local airborne observation,
  // not from a mission goal publication that the adapter no longer owns.
  std::int64_t airborne_start_ns = 0;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    airborne_start_ns = airborne_start_ns_;
  }
  if (airborne_start_ns > 0 && now.nanoseconds() >= airborne_start_ns &&
      static_cast<double>(now.nanoseconds() - airborne_start_ns) / 1e9 <=
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
  vehicle_status_subscription_ =
      SharedSubscription<px4_msgs::msg::VehicleStatus>::create(
          node_, owned_mode.topicNamespacePrefix() + "fmu/out/vehicle_status" +
              px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(),
          [this](const px4_msgs::msg::VehicleStatus::UniquePtr& message) {
            onVehicleStatus(message);
          });
  handover_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds{50}, [this]() { checkHoldHandover(); });
}

void NavigationModeExecutor::onVehicleStatus(
    const px4_msgs::msg::VehicleStatus::UniquePtr& message) {
  if (!message) return;
  px4_hold_confirmed_ =
      message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;
  if (px4_hold_confirmed_) {
    hold_handover_pending_ = false;
    hold_handover_in_flight_ = false;
  }
}

void NavigationModeExecutor::checkHoldHandover() {
  const auto now_steady_ns = navigation_common::steadyClockNowNanoseconds();
  if (hold_handover_pending_ && !px4_hold_confirmed_) {
    if (!hold_handover_in_flight_ && now_steady_ns >= hold_handover_next_retry_steady_ns_) {
      schedulePx4Hold(hold_handover_complete_navigation_failure_);
    }
  }
}

void NavigationModeExecutor::onActivate() {
  px4_hold_confirmed_ = false;
  hold_handover_pending_ = false;
  hold_handover_in_flight_ = false;
  hold_handover_complete_navigation_failure_ = false;
  hold_handover_attempts_ = 0U;
  hold_handover_next_retry_steady_ns_ = 0;
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
  hold_handover_pending_ = true;
  hold_handover_complete_navigation_failure_ =
      hold_handover_complete_navigation_failure_ || complete_navigation_failure;
  if (px4_hold_confirmed_ || hold_handover_in_flight_) return;
  hold_handover_in_flight_ = true;
  ++hold_handover_attempts_;
  scheduleMode(px4_ros2::ModeBase::kModeIDLoiter,
               [this](px4_ros2::Result hold_result) {
    onPx4HoldHandoverCompleted(hold_result, hold_handover_complete_navigation_failure_);
  });
}

void NavigationModeExecutor::onPx4HoldHandoverCompleted(
    px4_ros2::Result result, bool complete_navigation_failure) {
  if (result == px4_ros2::Result::Success || result == px4_ros2::Result::Deactivated) {
    hold_handover_in_flight_ = false;
    hold_handover_pending_ = false;
    RCLCPP_INFO(node_.get_logger(), "PX4 Hold handover completed with result=%s",
                px4_ros2::resultToString(result));
    return;
  }

  hold_handover_in_flight_ = false;
  hold_handover_pending_ = true;
  hold_handover_complete_navigation_failure_ =
      hold_handover_complete_navigation_failure_ || complete_navigation_failure;
  constexpr std::int64_t kHoldRetryPeriodNs = 250'000'000LL;
  hold_handover_next_retry_steady_ns_ =
      navigation_common::steadyClockNowNanoseconds() + kHoldRetryPeriodNs;
  RCLCPP_ERROR_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "PX4 Hold handover attempt=%u failed with result=%s; keeping the explicit "
      "stationary safety stream and retrying until AUTO_LOITER is confirmed",
      hold_handover_attempts_, px4_ros2::resultToString(result));
}

void NavigationModeExecutor::onDeactivate(DeactivateReason reason) {
  px4_hold_confirmed_ = false;
  hold_handover_pending_ = false;
  hold_handover_in_flight_ = false;
  hold_handover_complete_navigation_failure_ = false;
  hold_handover_attempts_ = 0U;
  hold_handover_next_retry_steady_ns_ = 0;
  RCLCPP_INFO(node_.get_logger(), "Avoidance Mission executor deactivated (%s)",
              reason == DeactivateReason::FailsafeActivated ? "failsafe" : "mode_exit");
}

void NavigationModeExecutor::onFailsafeDeferred() {
  RCLCPP_WARN(node_.get_logger(), "PX4 requested a deferred failsafe; no failsafe is deferred");
}

}  // namespace px4_navigation_external_mode
