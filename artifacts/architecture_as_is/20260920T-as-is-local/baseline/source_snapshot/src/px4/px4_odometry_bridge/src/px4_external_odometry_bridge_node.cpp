#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <navigation_common/time.hpp>
#include <navigation_contracts/msg/propagated_odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "px4_odometry_bridge/external_odometry_conversion.hpp"
#include "px4_odometry_bridge/external_odometry_gate.hpp"
#include "px4_odometry_bridge/frame_generation_policy.hpp"
#include "px4_odometry_bridge/geometric_jump_continuity.hpp"
#include "px4_odometry_bridge/geometric_jump_latch.hpp"
#include "px4_odometry_bridge/timestamp_conversion.hpp"
#include "px4_odometry_bridge/topic_version.hpp"

namespace px4_odometry_bridge {

namespace {
constexpr char kLioPropagatedOdometryTopic[] = "/lio/odometry_propagated";

std::int64_t requireDurationNanoseconds(const double seconds, const bool allow_zero) {
  const auto nanoseconds = navigation_common::secondsToNanoseconds(seconds);
  if (!nanoseconds.has_value() || (!allow_zero && *nanoseconds <= 0)) {
    throw std::invalid_argument("duration parameter must be finite and in range");
  }
  return *nanoseconds;
}
}  // namespace

class Px4ExternalOdometryBridgeNode final : public rclcpp::Node {
 public:
  using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
  static constexpr std::uint64_t kTransportTimestampMappingGeneration = 1;

  Px4ExternalOdometryBridgeNode() : Node("px4_external_odometry_bridge") {
    const double max_age_s = declare_parameter<double>(
        "external_odometry.maximum_age_s", 0.5);
    const double diagnostics_max_age_s = declare_parameter<double>(
        "external_odometry.diagnostics_maximum_age_s", 2.0);
    position_jump_m_ = declare_parameter<double>(
        "external_odometry.position_jump_m", 0.75);
    orientation_jump_rad_ = declare_parameter<double>(
        "external_odometry.orientation_jump_rad", 0.35);
    maximum_expected_speed_mps_ = declare_parameter<double>(
      "external_odometry.maximum_expected_speed_mps", 10.0);
    maximum_expected_angular_rate_rad_s_ = declare_parameter<double>(
      "external_odometry.maximum_expected_angular_rate_rad_s", 6.0);
    minimum_continuity_dt_s_ = declare_parameter<double>(
      "external_odometry.minimum_continuity_dt_s", 1e-4);
    maximum_continuity_dt_s_ = declare_parameter<double>(
      "external_odometry.maximum_continuity_dt_s", 0.5);
    if (!std::isfinite(max_age_s) || max_age_s <= 0.0 ||
        !std::isfinite(diagnostics_max_age_s) || diagnostics_max_age_s <= 0.0 ||
        !std::isfinite(position_jump_m_) || position_jump_m_ <= 0.0 ||
      !std::isfinite(orientation_jump_rad_) || orientation_jump_rad_ <= 0.0 ||
      !std::isfinite(maximum_expected_speed_mps_) ||
        maximum_expected_speed_mps_ <= 0.0 ||
      !std::isfinite(maximum_expected_angular_rate_rad_s_) ||
        maximum_expected_angular_rate_rad_s_ <= 0.0 ||
      !std::isfinite(minimum_continuity_dt_s_) || minimum_continuity_dt_s_ <= 0.0 ||
      !std::isfinite(maximum_continuity_dt_s_) ||
      maximum_continuity_dt_s_ < minimum_continuity_dt_s_) {
      throw std::invalid_argument("invalid external odometry bridge gate parameters");
    }
    max_age_ns_ = requireDurationNanoseconds(max_age_s, false);
    diagnostics_max_age_ns_ = requireDurationNanoseconds(diagnostics_max_age_s, false);
    const auto input_clock_domain = declare_parameter<std::string>(
        "timing.clock_domain", "ros_time");
    timestamp_mapping_mode_ = timestampMappingModeFor(
        get_parameter("use_sim_time").as_bool(), input_clock_domain);
    if (timestamp_mapping_mode_ == TimestampMappingMode::kUnresolved) {
      throw std::invalid_argument(
          "timing.clock_domain must be simulation_time with use_sim_time=true, "
          "or ros_time/system_time with use_sim_time=false");
    }
    jump_continuity_config_.position_jump_margin_m = position_jump_m_;
    jump_continuity_config_.orientation_jump_margin_rad = orientation_jump_rad_;
    jump_continuity_config_.maximum_expected_speed_mps = maximum_expected_speed_mps_;
    jump_continuity_config_.maximum_expected_angular_rate_rad_s =
      maximum_expected_angular_rate_rad_s_;
    jump_continuity_config_.minimum_continuity_dt_s = minimum_continuity_dt_s_;
    jump_continuity_config_.maximum_continuity_dt_s = maximum_continuity_dt_s_;
    timestamp_converter_ = std::make_unique<TimestampConverter>(max_age_ns_);
    output_ = create_publisher<VehicleOdometry>(
        versioned_topic<VehicleOdometry>("/fmu/in/vehicle_visual_odometry"),
        rclcpp::QoS(10).best_effort());
    // This is deliberately the only odometry input. Never substitute
    // simulator ground truth here: it is evaluation-only.
    lio_sub_ = create_subscription<navigation_contracts::msg::PropagatedOdometry>(
        kLioPropagatedOdometryTopic, rclcpp::QoS(20).reliable(),
        [this](navigation_contracts::msg::PropagatedOdometry::ConstSharedPtr message) {
          if (message) on_lio(*message);
        });
    lio_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/lio/diagnostics", rclcpp::QoS(20).best_effort(),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          if (message) on_lio_diagnostics(*message);
        });
    node_ready_ = true;
  }

 private:
  static bool fits_px4_float(const ExternalOdometryFrame& frame) {
    const auto finite_float = [](const double value) {
      return std::isfinite(static_cast<float>(value));
    };
    for (const double value : frame.position_ned) {
      if (!finite_float(value)) return false;
    }
    for (const double value : frame.orientation_ned.coeffs()) {
      if (!finite_float(value)) return false;
    }
    for (const double value : frame.velocity_ned) {
      if (!finite_float(value)) return false;
    }
    for (const double value : frame.angular_velocity_body_frd) {
      if (!finite_float(value)) return false;
    }
    return true;
  }

  static bool value_is_true(const diagnostic_msgs::msg::DiagnosticStatus& status,
                            const std::string& key) {
    for (const auto& item : status.values) {
      if (item.key == key) return item.value == "true";
    }
    return false;
  }

  static std::uint64_t value_uint(const diagnostic_msgs::msg::DiagnosticStatus& status,
                                  const std::string& key) {
    for (const auto& item : status.values) {
      if (item.key == key) {
        try {
          return static_cast<std::uint64_t>(std::stoull(item.value));
        } catch (const std::exception&) {
          return 0U;
        }
      }
    }
    return 0U;
  }

  void on_lio_diagnostics(const diagnostic_msgs::msg::DiagnosticArray& message) {
    for (const auto& status : message.status) {
      if (status.name == "fast_lio/estimator") {
        const auto diagnostic_stamp = navigation_common::rosTimeToNanoseconds(
            message.header.stamp);
        if (!diagnostic_stamp || *diagnostic_stamp <= 0 ||
            (last_lio_diagnostics_stamp_ns_ != 0 &&
             *diagnostic_stamp <= last_lio_diagnostics_stamp_ns_)) {
          // Diagnostics are a state stream. A delayed or malformed heartbeat
          // must not move the public generation backwards or reopen a latch.
          lio_valid_ = false;
          lio_covariance_valid_ = false;
          return;
        }
        last_lio_diagnostics_stamp_ns_ = *diagnostic_stamp;
        const auto generation = value_uint(status, "lio_public_frame_generation");
        const bool generation_valid = value_is_true(status, "lio_public_frame_generation_valid");
        const bool previous_latched = jump_latch_.latched();
        const bool generation_changed = jump_latch_.observePublicFrameGeneration(
            generation_valid, generation);
        if (generation_changed) {
          RCLCPP_INFO(
              get_logger(),
              "public generation changed: previous=%llu current=%llu generation_valid=%s "
              "latch_cleared=%s published_count=%llu gated_count=%llu",
              static_cast<unsigned long long>(lio_public_frame_generation_),
              static_cast<unsigned long long>(generation),
              generation_valid ? "true" : "false",
              (previous_latched && !jump_latch_.latched()) ? "true" : "false",
              static_cast<unsigned long long>(published_count_),
              static_cast<unsigned long long>(gated_count_));
          last_published_.reset();
          jump_continuity_state_.continuity_trusted = false;
        }
        lio_public_frame_generation_ = generation;
        lio_public_frame_generation_valid_ = generation_valid;
        lio_valid_ = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
                     value_is_true(status, "navigation_valid") &&
                     value_is_true(status, "corrected_estimate_valid") &&
                     std::any_of(status.values.begin(), status.values.end(),
                                 [](const auto& item) {
                                   return item.key == "status" && item.value == "TRACKING";
                                 });
        lio_covariance_valid_ = value_is_true(status, "pose_covariance_available") &&
                                value_is_true(status, "twist_covariance_available");
        last_lio_diagnostics_ns_ = *diagnostic_stamp;
      }
    }
  }

  bool lio_diagnostics_fresh(std::int64_t now_ns) const {
    // Heartbeat liveness is based on the producer's source timestamp, not
    // callback arrival time. A delayed old heartbeat must not reopen the gate.
    if (last_lio_diagnostics_ns_ <= 0 || now_ns <= 0) return false;
    const auto age = navigation_common::checkedDifference(
        now_ns, last_lio_diagnostics_ns_);
    return age.has_value() && *age >= 0 && *age <= diagnostics_max_age_ns_;
  }

  void on_lio(const navigation_contracts::msg::PropagatedOdometry& message) {
    const bool previous_publication_active = publication_active_;
    if (!strictly_newer_source_identity(highest_received_lio_epoch_,
                                        highest_received_lio_sequence_,
                                        message.localization_epoch,
                                        message.sequence)) {
      ++rejected_count_;
      last_frame_valid_ = false;
      last_rejection_reason_ = "PROPAGATED_STATE_IDENTITY_REJECTED";
      publication_ready_ = false;
      publication_active_ = false;
      return;
    }
    const bool epoch_changed = highest_received_lio_epoch_ != 0U &&
                               message.localization_epoch != highest_received_lio_epoch_;
    // Advance the source high-water mark before conversion and publication
    // gates. A newer sample that is temporarily gated must still prevent a
    // delayed older sample from being published after the gate recovers.
    highest_received_lio_epoch_ = message.localization_epoch;
    highest_received_lio_sequence_ = message.sequence;
    if (epoch_changed) {
      // The typed state identity is the authoritative reset boundary. Clear
      // continuity baselines before considering the first sample of the new
      // epoch; diagnostics still has to certify estimator health before PX4
      // publication is enabled again.
      last_published_.reset();
      last_received_.reset();
      jump_continuity_state_ = GeometricJumpContinuityState{};
      (void)jump_latch_.observePublicFrameGeneration(true, message.localization_epoch);
    }
    const auto frame = convert_ros_lio_odometry(message.odometry);
    if (!frame) {
      ++rejected_count_;
      last_frame_valid_ = false;
      last_rejection_reason_ = "EXTERNAL_ODOMETRY_CONVERSION_REJECTED";
      last_gate_ = ExternalOdometryGateResult{};
      jump_continuity_state_.continuity_trusted = false;
      publication_ready_ = false;
      publication_active_ = false;
      return;
    }
    if (!fits_px4_float(*frame)) {
      ++rejected_count_;
      last_frame_valid_ = false;
      last_rejection_reason_ = "EXTERNAL_ODOMETRY_NOT_FLOAT_REPRESENTABLE";
      last_gate_ = ExternalOdometryGateResult{};
      jump_continuity_state_.continuity_trusted = false;
      publication_ready_ = false;
      publication_active_ = false;
      return;
    }
    const auto position_variance_float =
        positive_variances_to_px4_float(frame->position_variance);
    const auto orientation_variance_float =
        positive_variances_to_px4_float(frame->orientation_variance);
    const auto velocity_variance_float =
        positive_variances_to_px4_float(frame->velocity_variance);
    if (!position_variance_float || !orientation_variance_float ||
        !velocity_variance_float) {
      ++rejected_count_;
      ++covariance_rejected_count_;
      last_frame_valid_ = false;
      last_rejection_reason_ = "COVARIANCE_NOT_FLOAT_REPRESENTABLE";
      last_gate_ = ExternalOdometryGateResult{};
      jump_continuity_state_.continuity_trusted = false;
      publication_ready_ = false;
      publication_active_ = false;
      return;
    }
    last_frame_valid_ = true;
    last_rejection_reason_ = "NONE";
    const auto now_ns = now().nanoseconds();
    last_sample_timestamp_ns_ = frame->timestamp_ns;
    last_transport_timestamp_ns_ = now_ns;
    const std::uint64_t public_generation =
        lio_public_frame_generation_valid_ ? lio_public_frame_generation_ : 0U;
    last_timestamp_result_ = timestamp_converter_->convert(
        frame->timestamp_ns, now_ns, timestamp_mapping_mode_,
        kTransportTimestampMappingGeneration);
    const bool timestamp_ready = last_timestamp_result_.valid;
    const bool transport_ready = output_->get_subscription_count() > 0;
    // The callback carries the high-rate propagated odometry and its timestamp
    // is checked above. A slow propagated-diagnostics heartbeat may report a
    // transient stale correction while this stream is already current; it is
    // therefore not an independent publication gate.
    const bool lio_fresh = lio_diagnostics_fresh(now_ns);
    const bool source_continuity_valid =
      lio_valid_ && frame->frame_valid && lio_public_frame_generation_valid_ &&
      public_generation > 0U;
    const bool previous_latched = jump_latch_.latched();
    jump_continuity_state_.last_received = last_received_;
    last_jump_observation_ = observe_geometric_jump_continuity(
      *frame, source_continuity_valid,
      lio_public_frame_generation_valid_ && public_generation > 0U,
      public_generation, jump_continuity_config_, jump_continuity_state_);
    last_received_ = jump_continuity_state_.last_received;
    const bool jump_set_now = jump_latch_.observeGeometricJump(
      last_jump_observation_.jumped);

    ExternalOdometryGateInput gate_input;
    gate_input.node_ready = node_ready_;
    gate_input.transport_ready = transport_ready;
    gate_input.timestamp_ready = timestamp_ready;
    gate_input.covariance_ready = frame->covariance_valid && lio_covariance_valid_;
    gate_input.public_frame_generation_valid = lio_public_frame_generation_valid_ &&
                                               public_generation > 0;
    gate_input.lio_valid = lio_valid_;
    gate_input.lio_fresh = lio_fresh;
    gate_input.frame_valid = frame->frame_valid;
    gate_input.geometric_continuity_trusted =
        jump_continuity_state_.continuity_trusted;
    gate_input.geometric_jump_latched = jump_latch_.latched();
    const auto previous_gate_reason = last_gate_.reason;
    last_gate_ = evaluate_external_odometry_gate(gate_input);
    publication_ready_ = last_gate_.publication_ready;
    publication_active_ = publication_ready_;

    maybe_log_runtime_transitions(previous_gate_reason, previous_publication_active,
                                  previous_latched, jump_set_now, now_ns, *frame);

    if (!publication_ready_) {
      ++gated_count_;
      publication_active_ = false;
      return;
    }

    VehicleOdometry output;
    output.timestamp = last_timestamp_result_.publication_time_us;
    output.timestamp_sample = last_timestamp_result_.measurement_time_us;
    // VehicleOdometry contract: position/q and velocity below are both NED;
    // q is body-FRD -> world-NED.  angular_velocity remains body-FRD.
    output.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    output.velocity_frame = VehicleOdometry::VELOCITY_FRAME_NED;
    output.position = {static_cast<float>(frame->position_ned.x()),
                       static_cast<float>(frame->position_ned.y()),
                       static_cast<float>(frame->position_ned.z())};
    output.q = {static_cast<float>(frame->orientation_ned.w()),
                static_cast<float>(frame->orientation_ned.x()),
                static_cast<float>(frame->orientation_ned.y()),
                static_cast<float>(frame->orientation_ned.z())};
    output.velocity = {static_cast<float>(frame->velocity_ned.x()),
                       static_cast<float>(frame->velocity_ned.y()),
                       static_cast<float>(frame->velocity_ned.z())};
    output.angular_velocity = {static_cast<float>(frame->angular_velocity_body_frd.x()),
                               static_cast<float>(frame->angular_velocity_body_frd.y()),
                               static_cast<float>(frame->angular_velocity_body_frd.z())};
    output.position_variance = *position_variance_float;
    output.orientation_variance = *orientation_variance_float;
    output.velocity_variance = *velocity_variance_float;
    output.reset_counter =
        public_frame_generation_to_reset_counter(public_generation);
    output.quality = lio_valid_ ? 100 : 50;
    output_->publish(output);
    last_published_ = *frame;
    last_published_time_ns_ = frame->timestamp_ns;
    last_reset_counter_ = output.reset_counter;
    ++published_count_;
    publication_active_ = true;
  }

  rclcpp::Publisher<VehicleOdometry>::SharedPtr output_;
  rclcpp::Subscription<navigation_contracts::msg::PropagatedOdometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr lio_diagnostics_sub_;
  std::optional<ExternalOdometryFrame> last_published_;
  std::optional<ExternalOdometryFrame> last_received_;
  GeometricJumpContinuityState jump_continuity_state_;
  std::unique_ptr<TimestampConverter> timestamp_converter_;
  TimestampMappingMode timestamp_mapping_mode_{TimestampMappingMode::kUnresolved};
  GeometricJumpContinuityConfig jump_continuity_config_;
  GeometricJumpContinuityObservation last_jump_observation_;
  TimestampConversionResult last_timestamp_result_;
  ExternalOdometryGateResult last_gate_;
  std::int64_t max_age_ns_{500'000'000};
  std::int64_t diagnostics_max_age_ns_{2'000'000'000};
  double position_jump_m_{0.75};
  double orientation_jump_rad_{0.35};
  double maximum_expected_speed_mps_{10.0};
  double maximum_expected_angular_rate_rad_s_{6.0};
  double minimum_continuity_dt_s_{1e-4};
  double maximum_continuity_dt_s_{0.5};
  std::int64_t last_published_time_ns_{0};
  std::int64_t last_sample_timestamp_ns_{0};
  std::int64_t last_transport_timestamp_ns_{0};
  std::uint64_t published_count_{0};
  std::uint64_t gated_count_{0};
  std::uint64_t rejected_count_{0};
  std::uint64_t covariance_rejected_count_{0};
  std::uint8_t last_reset_counter_{0};
  GeometricJumpLatch jump_latch_;
  std::uint64_t lio_public_frame_generation_{0};
  std::uint64_t highest_received_lio_epoch_{0};
  std::uint64_t highest_received_lio_sequence_{0};
  std::int64_t last_lio_diagnostics_ns_{0};
  std::int64_t last_lio_diagnostics_stamp_ns_{0};
  bool node_ready_{false};
  bool lio_valid_{false};
  bool lio_covariance_valid_{false};
  bool lio_public_frame_generation_valid_{false};
  bool last_frame_valid_{false};
  bool publication_ready_{false};
  bool publication_active_{false};
  bool transition_timestamp_known_{false};
  bool transition_diagnostics_known_{false};
  bool transition_transport_known_{false};
  bool last_timestamp_ready_{false};
  bool last_diagnostics_fresh_{false};
  bool last_transport_ready_{false};
  std::string last_rejection_reason_{"NONE"};

  [[nodiscard]] std::string log_gate_context(
      const std::string& previous_reason,
      const ExternalOdometryFrame& frame,
      const std::int64_t now_ns) const {
    std::ostringstream stream;
    const double age_ms = now_ns >= frame.timestamp_ns
                              ? static_cast<double>(now_ns - frame.timestamp_ns) * 1e-6
                              : -1.0;
    stream << "previous_gate_reason=" << previous_reason
           << " current_gate_reason=" << last_gate_.reason
           << " publication_active=" << (publication_active_ ? "true" : "false")
           << " lio_valid=" << (last_gate_.lio_valid ? "true" : "false")
           << " lio_status_fresh=" << (last_gate_.lio_fresh ? "true" : "false")
           << " odometry_sample_age_ms=" << age_ms
           << " transport_ready=" << (last_gate_.transport_ready ? "true" : "false")
           << " timestamp_ready=" << (last_gate_.timestamp_ready ? "true" : "false")
           << " covariance_ready=" << (last_gate_.covariance_ready ? "true" : "false")
           << " frame_valid=" << (last_gate_.frame_valid ? "true" : "false")
           << " public_generation="
           << static_cast<unsigned long long>(lio_public_frame_generation_)
           << " generation_valid="
           << (lio_public_frame_generation_valid_ ? "true" : "false")
           << " geometric_jump_latched="
           << (last_gate_.geometric_jump_latched ? "true" : "false")
           << " jump_reason=" << to_string(last_jump_observation_.reason);
    if (last_jump_observation_.evaluated) {
      stream << " dt_ms=" << last_jump_observation_.dt_s * 1e3
             << " position_delta_m=" << last_jump_observation_.position_delta_m
             << " allowed_position_delta_m="
             << last_jump_observation_.allowed_position_delta_m
             << " orientation_delta_rad="
             << last_jump_observation_.orientation_delta_rad
             << " allowed_orientation_delta_rad="
             << last_jump_observation_.allowed_orientation_delta_rad;
    }
    stream << " published_count=" << static_cast<unsigned long long>(published_count_)
           << " gated_count=" << static_cast<unsigned long long>(gated_count_);
    return stream.str();
  }

  void maybe_log_runtime_transitions(const std::string& previous_gate_reason,
                                     const bool previous_publication_active,
                                     const bool previous_latched,
                                     const bool jump_set_now,
                                     const std::int64_t now_ns,
                                     const ExternalOdometryFrame& frame) {
    const auto context = log_gate_context(previous_gate_reason, frame, now_ns);
    if (previous_gate_reason != last_gate_.reason) {
      if (last_gate_.publication_ready) {
        RCLCPP_INFO(get_logger(), "external odometry gate transitioned to READY: %s",
                    context.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "external odometry gate transitioned closed: %s",
                    context.c_str());
      }
    }
    if (previous_publication_active != publication_active_) {
      if (publication_active_) {
        RCLCPP_INFO(get_logger(), "external odometry publication resumed: %s",
                    context.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "external odometry publication stopped: %s",
                    context.c_str());
      }
    }
    if (!previous_latched && jump_latch_.latched() && jump_set_now) {
      RCLCPP_WARN(get_logger(), "geometric jump latch set: %s", context.c_str());
    }
    if (previous_latched && !jump_latch_.latched()) {
      RCLCPP_INFO(get_logger(), "geometric jump latch cleared: %s", context.c_str());
    }

    if (!transition_timestamp_known_) {
      transition_timestamp_known_ = true;
      last_timestamp_ready_ = last_gate_.timestamp_ready;
    } else if (last_timestamp_ready_ != last_gate_.timestamp_ready) {
      if (last_gate_.timestamp_ready) {
        RCLCPP_INFO(get_logger(), "timestamp conversion recovered: %s", context.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "timestamp conversion failed: %s", context.c_str());
      }
      last_timestamp_ready_ = last_gate_.timestamp_ready;
    }

    if (!transition_diagnostics_known_) {
      transition_diagnostics_known_ = true;
      last_diagnostics_fresh_ = last_gate_.lio_fresh;
    } else if (last_diagnostics_fresh_ != last_gate_.lio_fresh) {
      if (last_gate_.lio_fresh) {
        RCLCPP_INFO(get_logger(), "lio diagnostics freshness recovered: %s", context.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "lio diagnostics freshness failed: %s", context.c_str());
      }
      last_diagnostics_fresh_ = last_gate_.lio_fresh;
    }

    if (!transition_transport_known_) {
      transition_transport_known_ = true;
      last_transport_ready_ = last_gate_.transport_ready;
    } else if (last_transport_ready_ != last_gate_.transport_ready) {
      if (last_gate_.transport_ready) {
        RCLCPP_INFO(get_logger(), "px4 subscription count transitioned to >0: %s",
                    context.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "px4 subscription count transitioned to 0: %s",
                    context.c_str());
      }
      last_transport_ready_ = last_gate_.transport_ready;
    }
  }
};

}  // namespace px4_odometry_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_odometry_bridge::Px4ExternalOdometryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
