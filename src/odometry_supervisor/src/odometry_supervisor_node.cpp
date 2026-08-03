#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <navigation_interfaces/msg/odometry_supervisor_status.hpp>
#include <navigation_interfaces/srv/sample_odometry_at_time.hpp>

#include "odometry_supervisor/diagnostic_adapter.hpp"
#include "odometry_supervisor/residual_calculator.hpp"
#include "odometry_supervisor/supervisor_state_machine.hpp"

namespace odometry_supervisor {
namespace {
using StatusMessage = navigation_interfaces::msg::OdometrySupervisorStatus;
using QueryService = navigation_interfaces::srv::SampleOdometryAtTime;

std::int64_t time_ns(const builtin_interfaces::msg::Time& time) {
  return static_cast<std::int64_t>(time.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(time.nanosec);
}

builtin_interfaces::msg::Time ros_time(std::int64_t timestamp_ns) {
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(timestamp_ns / 1'000'000'000LL);
  result.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1'000'000'000LL);
  return result;
}

OdometryState state_from_ros(const nav_msgs::msg::Odometry& message) {
  OdometryState state;
  state.timestamp_ns = time_ns(message.header.stamp);
  state.position_odom = Eigen::Vector3d(message.pose.pose.position.x, message.pose.pose.position.y,
                                        message.pose.pose.position.z);
  state.orientation_odom_base = Eigen::Quaterniond(
      message.pose.pose.orientation.w, message.pose.pose.orientation.x,
      message.pose.pose.orientation.y, message.pose.pose.orientation.z);
  state.velocity_base = Eigen::Vector3d(message.twist.twist.linear.x,
                                        message.twist.twist.linear.y,
                                        message.twist.twist.linear.z);
  state.frame_id = message.header.frame_id;
  state.child_frame_id = message.child_frame_id;
  state.valid = true;
  return state;
}

const char* health_name(HealthState state) {
  switch (state) {
    case HealthState::kStartup: return "STARTUP";
    case HealthState::kHealthy: return "HEALTHY";
    case HealthState::kSuspect: return "SUSPECT";
    case HealthState::kDegraded: return "DEGRADED";
    case HealthState::kDiverged: return "DIVERGED";
  }
  return "UNKNOWN";
}

}  // namespace

class OdometrySupervisorNode final : public rclcpp::Node {
 public:
  OdometrySupervisorNode() : Node("odometry_supervisor") {
    config_ = load_config();
    machine_ = SupervisorStateMachine(config_);
    status_ = create_publisher<StatusMessage>(
        "/navigation/odometry_supervisor/status", rclcpp::QoS(1).reliable().transient_local());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/navigation/odometry_supervisor/diagnostics", rclcpp::QoS(10).reliable());
    propagated_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/odometry_propagated", rclcpp::QoS(20).reliable(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          latest_propagated_ = state_from_ros(*message);
        });
    corrected_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/odometry_corrected", rclcpp::QoS(20).reliable(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          latest_corrected_ = state_from_ros(*message);
        });
    px4_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/px4/odometry_ros", rclcpp::QoS(20).reliable(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          latest_px4_ = state_from_ros(*message);
        });
    lio_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/lio/diagnostics", rclcpp::QoS(10).reliable(),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          lio_diagnostics_ = selectDiagnostic(*message, "fast_lio/estimator");
        });
    px4_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/px4/diagnostics", rclcpp::QoS(10).reliable(),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          px4_diagnostics_ = selectDiagnostic(*message, "px4_odometry_bridge");
        });
    query_client_ = create_client<QueryService>("/px4/sample_odometry_at_time");
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / config_.evaluation_rate_hz));
    evaluation_timer_ = create_wall_timer(std::max(period, std::chrono::milliseconds(1)),
                                          [this]() { evaluate_once(); });
  }

 private:
  SupervisorConfig load_config() {
    SupervisorConfig config;
    const auto reference = declare_parameter<std::string>("reference_mode", "independent");
    if (reference == "independent") {
      config.reference_mode = ReferenceMode::kIndependent;
    } else if (reference == "correlated") {
      config.reference_mode = ReferenceMode::kCorrelated;
    } else {
      throw std::invalid_argument("unknown reference_mode");
    }
    config.evaluation_rate_hz = declare_parameter("evaluation_rate_hz", config.evaluation_rate_hz);
    config.propagated_max_age_ns = declare_parameter("freshness.propagated_max_age_ns",
                                                      config.propagated_max_age_ns);
    config.corrected_max_age_ns = declare_parameter("freshness.corrected_max_age_ns",
                                                     config.corrected_max_age_ns);
    config.px4_max_age_ns = declare_parameter("freshness.px4_max_age_ns", config.px4_max_age_ns);
    config.diagnostics_max_age_ns = declare_parameter("freshness.diagnostics_max_age_ns",
                                                       config.diagnostics_max_age_ns);
    config.maximum_alignment_gap_ns = declare_parameter(
        "alignment.maximum_gap_ns", config.maximum_alignment_gap_ns);
    config.service_timeout_ns = declare_parameter("alignment.service_timeout_ns",
                                                  config.service_timeout_ns);
    config.clear_ratio = declare_parameter("hysteresis.clear_ratio", config.clear_ratio);
    config.suspect_enter_ns = declare_parameter("persistence.suspect_enter_ns",
                                                config.suspect_enter_ns);
    config.degraded_enter_ns = declare_parameter("persistence.degraded_enter_ns",
                                                 config.degraded_enter_ns);
    config.diverged_enter_ns = declare_parameter("persistence.diverged_enter_ns",
                                                 config.diverged_enter_ns);
    config.recovery_confirm_ns = declare_parameter("persistence.recovery_confirm_ns",
                                                   config.recovery_confirm_ns);
    config.reset_grace_ns = declare_parameter("persistence.reset_grace_ns", config.reset_grace_ns);
    config.suspect_speed_limit_m_s = declare_parameter(
        "actions.suspect_speed_limit_m_s", config.suspect_speed_limit_m_s);
    config.degraded_speed_limit_m_s = declare_parameter(
        "actions.degraded_speed_limit_m_s", config.degraded_speed_limit_m_s);
    config.suspect.position_m = declare_parameter("thresholds.suspect.position_m",
                                                  config.suspect.position_m);
    config.suspect.velocity_m_s = declare_parameter("thresholds.suspect.velocity_m_s",
                                                    config.suspect.velocity_m_s);
    config.suspect.orientation_rad = declare_parameter("thresholds.suspect.orientation_rad",
                                                       config.suspect.orientation_rad);
    config.suspect.yaw_rad = declare_parameter("thresholds.suspect.yaw_rad",
                                                config.suspect.yaw_rad);
    config.degraded.position_m = declare_parameter("thresholds.degraded.position_m",
                                                   config.degraded.position_m);
    config.degraded.velocity_m_s = declare_parameter("thresholds.degraded.velocity_m_s",
                                                     config.degraded.velocity_m_s);
    config.degraded.orientation_rad = declare_parameter("thresholds.degraded.orientation_rad",
                                                        config.degraded.orientation_rad);
    config.degraded.yaw_rad = declare_parameter("thresholds.degraded.yaw_rad",
                                                 config.degraded.yaw_rad);
    config.diverged.position_m = declare_parameter("thresholds.diverged.position_m",
                                                   config.diverged.position_m);
    config.diverged.velocity_m_s = declare_parameter("thresholds.diverged.velocity_m_s",
                                                     config.diverged.velocity_m_s);
    config.diverged.orientation_rad = declare_parameter("thresholds.diverged.orientation_rad",
                                                        config.diverged.orientation_rad);
    config.diverged.yaw_rad = declare_parameter("thresholds.diverged.yaw_rad",
                                                 config.diverged.yaw_rad);
    return config;
  }

  bool fresh(const std::optional<OdometryState>& state, std::int64_t max_age,
             std::int64_t now_ns, std::int64_t* age) const {
    if (!state || !ResidualCalculator::valid(*state) || state->timestamp_ns > now_ns) {
      if (age) *age = -1;
      return false;
    }
    *age = now_ns - state->timestamp_ns;
    return *age <= max_age;
  }

  void request_px4_at(const OdometryState& lio) {
    if (query_pending_ || !query_client_->service_is_ready()) return;
    auto request = std::make_shared<QueryService::Request>();
    request->sample_time = ros_time(lio.timestamp_ns);
    const auto sequence = ++query_sequence_;
    query_pending_ = true;
    pending_sequence_ = sequence;
    pending_started_ = std::chrono::steady_clock::now();
    query_client_->async_send_request(
        request, [this, sequence, requested_time = lio.timestamp_ns](
                     rclcpp::Client<QueryService>::SharedFuture future) {
          if (!query_pending_ || pending_sequence_ != sequence) return;
          query_pending_ = false;
          QueryService::Response::SharedPtr response;
          try {
            response = future.get();
          } catch (const std::exception&) {
            aligned_px4_.reset();
            aligned_lio_.reset();
            return;
          }
          if (!response || !response->success ||
              time_ns(response->odometry.header.stamp) != requested_time) {
            aligned_px4_.reset();
            aligned_lio_.reset();
            return;
          }
          const auto lio = latest_propagated_;
          if (!lio || lio->timestamp_ns != requested_time) return;
          aligned_lio_ = lio;
          aligned_px4_ = state_from_ros(response->odometry);
          const auto residual = ResidualCalculator::compare(*aligned_lio_, *aligned_px4_,
                                                             previous_residual_);
          if (residual) previous_residual_ = residual;
          aligned_residual_ = residual.value_or(Residual{});
        });
  }

  void evaluate_once() {
    const auto now_ns = now().nanoseconds();
    if (query_pending_) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - pending_started_).count();
      if (elapsed > config_.service_timeout_ns) {
        query_pending_ = false;
        pending_sequence_ = 0;
        aligned_px4_.reset();
        aligned_lio_.reset();
      }
    }
    std::int64_t propagated_age = -1;
    std::int64_t corrected_age = -1;
    std::int64_t px4_age = -1;
    const bool propagated_fresh = fresh(latest_propagated_, config_.propagated_max_age_ns,
                                        now_ns, &propagated_age);
    const bool corrected_fresh = fresh(latest_corrected_, config_.corrected_max_age_ns,
                                       now_ns, &corrected_age);
    const bool px4_fresh = fresh(latest_px4_, config_.px4_max_age_ns, now_ns, &px4_age);
    if (propagated_fresh && !query_pending_ &&
        (!aligned_lio_ || aligned_lio_->timestamp_ns != latest_propagated_->timestamp_ns)) {
      request_px4_at(*latest_propagated_);
    }

    EvaluationInput input;
    input.evaluation_time_ns = std::max(now_ns, latest_propagated_ ? latest_propagated_->timestamp_ns : 0);
    const auto lio_status = lio_diagnostics_.string("status");
    const auto lio_status_is = [&lio_status](const char* canonical, const char* legacy) {
      return lio_status == canonical || lio_status == legacy;
    };
    input.lio_valid = lio_diagnostics_.found &&
                      lio_status_is("TRACKING", "Tracking") &&
                      lio_diagnostics_.boolean("navigation_valid");
    input.lio_lost = lio_diagnostics_.found && lio_status_is("LOST", "Lost");
    input.lio_state_corruption =
        lio_diagnostics_.string("last_update_failure_class") == "StateCorruption";
    input.lio_degraded = lio_diagnostics_.found && lio_status_is("DEGRADED", "Degraded");
    input.lio_resetting = lio_diagnostics_.found && lio_status_is("RESETTING", "Resetting");
    input.propagated_fresh = propagated_fresh;
    input.corrected_fresh = corrected_fresh;
    input.px4_available = latest_px4_.has_value();
    input.px4_fresh = px4_fresh;
    input.px4_continuity_valid = px4_diagnostics_.boolean("continuity_valid");
    input.px4_post_reset_stable = px4_diagnostics_.boolean("post_reset_stable");
    const bool lio_diagnostics_fresh =
        lio_diagnostics_.found && lio_diagnostics_.stamp_ns > 0 && now_ns >= lio_diagnostics_.stamp_ns &&
        now_ns - lio_diagnostics_.stamp_ns <= config_.diagnostics_max_age_ns;
    const bool px4_diagnostics_fresh =
        px4_diagnostics_.found && px4_diagnostics_.stamp_ns > 0 && now_ns >= px4_diagnostics_.stamp_ns &&
        now_ns - px4_diagnostics_.stamp_ns <= config_.diagnostics_max_age_ns;
    input.diagnostics_valid = lio_diagnostics_fresh && px4_diagnostics_fresh &&
                              lio_diagnostics_.hasSchemaV1() && px4_diagnostics_.hasSchemaV1();
    const auto source = lio_diagnostics_.string("initial_prior_source");
    input.origin_aligned = source == "topic" &&
                           lio_diagnostics_.boolean("initial_prior_applied") &&
                           !lio_diagnostics_.boolean("initial_prior_fallback_applied") &&
                           lio_diagnostics_.string("initial_prior_reason") == "TOPIC_PRIOR_ACCEPTED";
    input.propagated_age_ns = propagated_age;
    input.corrected_age_ns = corrected_age;
    input.px4_age_ns = px4_age;
    input.px4_reset_generation = px4_diagnostics_.unsignedInteger("reset_generation");
    input.px4_time_generation = px4_diagnostics_.unsignedInteger("time_generation");
    input.alignment_gap_ns = aligned_px4_ && aligned_lio_
                                 ? std::llabs(aligned_px4_->timestamp_ns - aligned_lio_->timestamp_ns)
                                 : -1;
    input.residual = aligned_lio_ && aligned_px4_ && latest_propagated_ &&
                             aligned_lio_->timestamp_ns == latest_propagated_->timestamp_ns
                         ? aligned_residual_
                         : Residual{};
    input.time_generation_changed = false;
    if (px4_diagnostics_.found && px4_diagnostics_.hasSchemaV1()) {
      if (last_seen_px4_time_generation_ &&
          *last_seen_px4_time_generation_ != input.px4_time_generation) {
        input.time_generation_changed = true;
        query_pending_ = false;
        pending_sequence_ = 0;
        aligned_px4_.reset();
        aligned_lio_.reset();
      }
      last_seen_px4_time_generation_ = input.px4_time_generation;
    }
    const auto output = machine_.evaluate(input);
    publish(output);
  }

  void publish(const SupervisorOutput& output) {
    StatusMessage message;
    message.header.stamp = now();
    message.header.frame_id = "odom";
    message.health = static_cast<std::uint8_t>(output.health);
    message.reference_mode = static_cast<std::uint8_t>(output.reference_mode);
    message.reason_code = output.reason_code;
    message.reason = output.reason;
    message.monitoring_available = output.monitoring_available;
    message.comparison_valid = output.comparison_valid;
    message.lio_valid = output.lio_valid;
    message.px4_valid = output.px4_valid;
    message.time_aligned = output.time_aligned;
    message.external_odometry_allowed = output.external_odometry_allowed;
    message.reinitialization_requested = output.reinitialization_requested;
    message.reinitialization_request_sequence = output.reinitialization_request_sequence;
    message.planner_speed_limit_active = output.planner_speed_limit_active;
    message.planner_speed_limit_m_s = output.planner_speed_limit_m_s;
    message.hover_or_failsafe_requested = output.hover_or_failsafe_requested;
    message.evaluation_time = ros_time(output.evaluation_time_ns);
    message.lio_propagated_age_ns = output.lio_propagated_age_ns;
    message.lio_corrected_age_ns = output.lio_corrected_age_ns;
    message.px4_age_ns = output.px4_age_ns;
    message.alignment_gap_ns = output.alignment_gap_ns;
    message.position_error_m = output.residual.position_error_m;
    message.velocity_error_m_s = output.residual.velocity_error_m_s;
    message.orientation_error_rad = output.residual.orientation_error_rad;
    message.yaw_error_rad = output.residual.yaw_error_rad;
    message.position_error_growth_m_s = output.residual.position_error_growth_m_s;
    message.px4_reset_generation = output.px4_reset_generation;
    message.px4_time_generation = output.px4_time_generation;
    message.state_transition_count = output.state_transition_count;
    message.evaluation_count = output.evaluation_count;
    message.alignment_failure_count = output.alignment_failure_count;
    status_->publish(message);

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = message.header.stamp;
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = "odometry_supervisor";
    diagnostic.level = output.health == HealthState::kHealthy
                           ? diagnostic_msgs::msg::DiagnosticStatus::OK
                           : (output.health == HealthState::kDiverged
                                  ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                  : diagnostic_msgs::msg::DiagnosticStatus::WARN);
    diagnostic.message = health_name(output.health) + std::string(": ") + output.reason;
    const auto add_value = [&diagnostic](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      diagnostic.values.push_back(std::move(item));
    };
    add_value("diagnostic_schema_version", "1");
    add_value("health", health_name(output.health));
    add_value("reference_mode", output.reference_mode == ReferenceMode::kIndependent ? "independent" : "correlated");
    add_value("comparison_valid", output.comparison_valid ? "true" : "false");
    add_value("external_odometry_allowed", output.external_odometry_allowed ? "true" : "false");
    add_value("reinitialization_requested", output.reinitialization_requested ? "true" : "false");
    add_value("position_error_m", std::to_string(output.residual.position_error_m));
    add_value("velocity_error_m_s", std::to_string(output.residual.velocity_error_m_s));
    add_value("orientation_error_rad", std::to_string(output.residual.orientation_error_rad));
    add_value("yaw_error_rad", std::to_string(output.residual.yaw_error_rad));
    add_value("position_error_growth_m_s", std::to_string(output.residual.position_error_growth_m_s));
    array.status.push_back(std::move(diagnostic));
    diagnostics_->publish(std::move(array));
  }

  SupervisorConfig config_;
  SupervisorStateMachine machine_;
  rclcpp::Publisher<StatusMessage>::SharedPtr status_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr propagated_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr corrected_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr px4_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr lio_diagnostics_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr px4_diagnostics_sub_;
  rclcpp::Client<QueryService>::SharedPtr query_client_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
  std::optional<OdometryState> latest_propagated_;
  std::optional<OdometryState> latest_corrected_;
  std::optional<OdometryState> latest_px4_;
  std::optional<OdometryState> aligned_lio_;
  std::optional<OdometryState> aligned_px4_;
  Residual aligned_residual_;
  std::optional<Residual> previous_residual_;
  DiagnosticSnapshot lio_diagnostics_;
  DiagnosticSnapshot px4_diagnostics_;
  bool query_pending_{false};
  std::uint64_t query_sequence_{0};
  std::uint64_t pending_sequence_{0};
  std::chrono::steady_clock::time_point pending_started_{};
  std::optional<std::uint64_t> last_seen_px4_time_generation_;
};

}  // namespace odometry_supervisor

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odometry_supervisor::OdometrySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
