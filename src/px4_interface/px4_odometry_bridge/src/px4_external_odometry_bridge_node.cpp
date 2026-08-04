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
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <navigation_interfaces/msg/odometry_supervisor_status.hpp>

#include "px4_odometry_bridge/external_odometry_conversion.hpp"
#include "px4_odometry_bridge/external_odometry_gate.hpp"
#include "px4_odometry_bridge/geometric_jump_latch.hpp"
#include "px4_odometry_bridge/timestamp_conversion.hpp"
#include "px4_odometry_bridge/topic_version.hpp"

namespace px4_odometry_bridge {

class Px4ExternalOdometryBridgeNode final : public rclcpp::Node {
 public:
  using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
  using SupervisorStatus = navigation_interfaces::msg::OdometrySupervisorStatus;
  static constexpr std::uint64_t kSimulationTimestampMappingGeneration = 1;

  Px4ExternalOdometryBridgeNode() : Node("px4_external_odometry_bridge") {
    max_age_ns_ = declare_parameter<std::int64_t>(
        "external_odometry.maximum_age_ns", 150'000'000);
    position_jump_m_ = declare_parameter<double>(
        "external_odometry.position_jump_m", 0.75);
    orientation_jump_rad_ = declare_parameter<double>(
        "external_odometry.orientation_jump_rad", 0.35);
    if (max_age_ns_ <= 0 || !std::isfinite(position_jump_m_) || position_jump_m_ <= 0.0 ||
        !std::isfinite(orientation_jump_rad_) || orientation_jump_rad_ <= 0.0) {
      throw std::invalid_argument("invalid external odometry bridge gate parameters");
    }
    timestamp_converter_ = std::make_unique<TimestampConverter>(max_age_ns_);
    output_ = create_publisher<VehicleOdometry>(
        versioned_topic<VehicleOdometry>("/fmu/in/vehicle_visual_odometry"),
        rclcpp::QoS(10).best_effort());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/px4/external_odometry_diagnostics", rclcpp::QoS(1).transient_local());
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/lio/odometry_propagated", rclcpp::QoS(20).reliable(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { on_lio(*message); });
    supervisor_sub_ = create_subscription<SupervisorStatus>(
        "/navigation/odometry_supervisor/status",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](SupervisorStatus::ConstSharedPtr message) { on_supervisor(*message); });
    diagnostics_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                            [this] { publish_diagnostics(); });
    node_ready_ = true;
  }

 private:
  static std::int64_t time_ns(const builtin_interfaces::msg::Time& time) {
    return static_cast<std::int64_t>(time.sec) * 1'000'000'000LL +
           static_cast<std::int64_t>(time.nanosec);
  }

  static bool fits_px4_float(const ExternalOdometryFrame& frame) {
    const auto finite_float = [](const double value) {
      return std::isfinite(static_cast<float>(value));
    };
    for (const double value : frame.position_frd) {
      if (!finite_float(value)) return false;
    }
    for (const double value : frame.orientation_frd.coeffs()) {
      if (!finite_float(value)) return false;
    }
    for (const double value : frame.velocity_body_frd) {
      if (!finite_float(value)) return false;
    }
    for (const double value : frame.angular_velocity_body_frd) {
      if (!finite_float(value)) return false;
    }
    return true;
  }

  void on_supervisor(const SupervisorStatus& message) {
    const bool generation_changed = jump_latch_.observePublicFrameGeneration(
        message.lio_public_frame_generation_valid,
        message.lio_public_frame_generation);
    const bool operator_reset = jump_latch_.observeOperatorReset(
        message.reinitialization_requested,
        message.reinitialization_request_sequence);
    // Only an authoritative LIO public generation or an explicit operator
    // reset can recover a geometric-jump latch. PX4 frame/reset generations
    // are observed for diagnostics and never control this state.
    if (generation_changed || operator_reset) {
      last_published_.reset();
    }
    supervisor_ = message;
  }

  void on_lio(const nav_msgs::msg::Odometry& message) {
    const auto frame = convert_ros_lio_odometry(message);
    if (!frame) {
      ++rejected_count_;
      last_frame_valid_ = false;
      last_rejection_reason_ = "EXTERNAL_ODOMETRY_CONVERSION_REJECTED";
      last_gate_ = ExternalOdometryGateResult{};
      publication_ready_ = false;
      publication_active_ = false;
      return;
    }
    if (!fits_px4_float(*frame)) {
      ++rejected_count_;
      last_frame_valid_ = false;
      last_rejection_reason_ = "EXTERNAL_ODOMETRY_NOT_FLOAT_REPRESENTABLE";
      last_gate_ = ExternalOdometryGateResult{};
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
      publication_ready_ = false;
      publication_active_ = false;
      return;
    }
    last_frame_valid_ = true;
    last_rejection_reason_ = "NONE";
    const auto now_ns = now().nanoseconds();
    last_sample_timestamp_ns_ = frame->timestamp_ns;
    last_transport_timestamp_ns_ = now_ns;
    const bool use_sim_time = get_parameter("use_sim_time").as_bool();
    const std::uint64_t public_generation =
        supervisor_.has_value() && supervisor_->lio_public_frame_generation_valid
            ? supervisor_->lio_public_frame_generation
            : 0U;
    last_timestamp_result_ = timestamp_converter_->convert(
        frame->timestamp_ns, now_ns, use_sim_time,
        kSimulationTimestampMappingGeneration);
    const bool timestamp_ready = last_timestamp_result_.valid;
    const bool transport_ready = output_->get_subscription_count() > 0;
    const bool supervisor_fresh = supervisor_.has_value() &&
                                 time_ns(supervisor_->evaluation_time) > 0 &&
                                 now_ns >= time_ns(supervisor_->evaluation_time) &&
                                 now_ns - time_ns(supervisor_->evaluation_time) <= max_age_ns_;
    const bool lio_fresh = supervisor_.has_value() &&
                           supervisor_->lio_propagated_age_ns >= 0 &&
                           supervisor_->lio_propagated_age_ns <= max_age_ns_ &&
                           supervisor_->lio_corrected_age_ns >= 0 &&
                           supervisor_->lio_corrected_age_ns <= max_age_ns_;
    const bool supervisor_authorized =
        supervisor_.has_value() && supervisor_->external_measurement_authorized;

    bool frame_jump = false;
    if (last_published_.has_value()) {
      frame_jump = (frame->position_frd - last_published_->position_frd).norm() >
                       position_jump_m_ ||
                   last_published_->orientation_frd.angularDistance(
                       frame->orientation_frd) > orientation_jump_rad_;
    }
    (void)jump_latch_.observeGeometricJump(frame_jump);

    ExternalOdometryGateInput gate_input;
    gate_input.node_ready = node_ready_;
    gate_input.transport_ready = transport_ready;
    gate_input.timestamp_ready = timestamp_ready;
    gate_input.covariance_ready = frame->covariance_valid && supervisor_.has_value() &&
                                  supervisor_->covariance_valid;
    gate_input.supervisor_authorized = supervisor_authorized;
    gate_input.public_frame_generation_valid =
        supervisor_.has_value() && supervisor_->lio_public_frame_generation_valid &&
        public_generation > 0;
    gate_input.corrected_propagated_fresh = lio_fresh;
    gate_input.supervisor_fresh = supervisor_fresh;
    gate_input.frame_valid = frame->frame_valid;
    gate_input.geometric_jump_latched = jump_latch_.latched();
    last_gate_ = evaluate_external_odometry_gate(gate_input);
    publication_ready_ = last_gate_.publication_ready;
    if (!publication_ready_) {
      ++gated_count_;
      publication_active_ = false;
      return;
    }

    VehicleOdometry output;
    output.timestamp = last_timestamp_result_.publication_time_us;
    output.timestamp_sample = last_timestamp_result_.measurement_time_us;
    output.pose_frame = VehicleOdometry::POSE_FRAME_FRD;
    output.velocity_frame = VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    output.position = {static_cast<float>(frame->position_frd.x()),
                       static_cast<float>(frame->position_frd.y()),
                       static_cast<float>(frame->position_frd.z())};
    output.q = {static_cast<float>(frame->orientation_frd.w()),
                static_cast<float>(frame->orientation_frd.x()),
                static_cast<float>(frame->orientation_frd.y()),
                static_cast<float>(frame->orientation_frd.z())};
    output.velocity = {static_cast<float>(frame->velocity_body_frd.x()),
                       static_cast<float>(frame->velocity_body_frd.y()),
                       static_cast<float>(frame->velocity_body_frd.z())};
    output.angular_velocity = {static_cast<float>(frame->angular_velocity_body_frd.x()),
                               static_cast<float>(frame->angular_velocity_body_frd.y()),
                               static_cast<float>(frame->angular_velocity_body_frd.z())};
    output.position_variance = *position_variance_float;
    output.orientation_variance = *orientation_variance_float;
    output.velocity_variance = *velocity_variance_float;
    output.reset_counter =
        public_frame_generation_to_reset_counter(public_generation);
    output.quality = supervisor_->health == SupervisorStatus::HEALTHY ? 100 : 50;
    output_->publish(output);
    last_published_ = *frame;
    last_published_time_ns_ = frame->timestamp_ns;
    last_reset_counter_ = output.reset_counter;
    ++published_count_;
    publication_active_ = true;
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "px4_external_odometry_bridge";
    const bool transport_ready = output_->get_subscription_count() > 0;
    const auto now_ns = now().nanoseconds();
    const bool supervisor_fresh = supervisor_.has_value() &&
                                  time_ns(supervisor_->evaluation_time) > 0 &&
                                  now_ns >= time_ns(supervisor_->evaluation_time) &&
                                  now_ns - time_ns(supervisor_->evaluation_time) <= max_age_ns_;
    const bool timestamp_ready = last_timestamp_result_.valid;
    const bool infrastructure_ready = node_ready_ && transport_ready && timestamp_ready;
    status.level = infrastructure_ready ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                         : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = publication_ready_ ? "external odometry publication is active"
                       : "external odometry publication gate is closed";
    const auto add = [&status](std::string key, std::string value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = std::move(key);
      item.value = std::move(value);
      status.values.push_back(std::move(item));
    };
    const auto timestamp_diagnostics = timestamp_converter_->diagnostics();
    const bool publisher_ready = node_ready_ && transport_ready;
    add("diagnostic_schema_version", "2");
    add("ready", infrastructure_ready ? "true" : "false");
    add("publisher_ready", publisher_ready ? "true" : "false");
    add("node_ready", node_ready_ ? "true" : "false");
    add("transport_ready", transport_ready ? "true" : "false");
    add("timestamp_ready", timestamp_ready ? "true" : "false");
    add("covariance_ready", last_gate_.covariance_ready ? "true" : "false");
    add("supervisor_authorized",
        last_gate_.supervisor_authorized ? "true" : "false");
    add("public_frame_generation_valid",
        last_gate_.public_frame_generation_valid ? "true" : "false");
    add("corrected_propagated_fresh",
        last_gate_.corrected_propagated_fresh ? "true" : "false");
    add("supervisor_fresh", supervisor_fresh ? "true" : "false");
    add("frame_valid", last_frame_valid_ ? "true" : "false");
    add("geometric_jump_latched", jump_latch_.latched() ? "true" : "false");
    add("publication_ready", publication_ready_ ? "true" : "false");
    add("publication_active",
        (publication_ready_ && publication_active_ && last_published_time_ns_ > 0 &&
         now_ns >= last_published_time_ns_ &&
         now_ns - last_published_time_ns_ <= max_age_ns_)
            ? "true"
            : "false");
    add("publication_authorized", last_gate_.supervisor_authorized ? "true" : "false");
    add("publisher_subscription_count", std::to_string(output_->get_subscription_count()));
    add("publisher_topic", versioned_topic<VehicleOdometry>(
                                "/fmu/in/vehicle_visual_odometry"));
    add("pose_frame", "POSE_FRAME_FRD");
    add("velocity_frame", "VELOCITY_FRAME_BODY_FRD");
    add("timestamp_source_domain", last_timestamp_result_.source_domain);
    add("timestamp_target_domain", last_timestamp_result_.target_domain);
    add("timestamp_conversion_valid", last_timestamp_result_.valid ? "true" : "false");
    add("timestamp_conversion_generation",
        std::to_string(last_timestamp_result_.timestamp_mapping_generation));
    add("timestamp_mapping_generation",
        std::to_string(timestamp_diagnostics.timestamp_mapping_generation));
    add("timestamp_mapping_generation_change_count",
        std::to_string(timestamp_diagnostics.timestamp_mapping_generation_change_count));
    add("timestamp_sample_us",
        std::to_string(last_timestamp_result_.measurement_time_us));
    add("publication_timestamp_us",
        std::to_string(last_timestamp_result_.publication_time_us));
    add("timestamp_age_ns", std::to_string(last_timestamp_result_.timestamp_age_ns));
    add("timestamp_regression_count",
        std::to_string(timestamp_diagnostics.regression_count));
    add("timestamp_sample_regression_count",
        std::to_string(timestamp_diagnostics.timestamp_sample_regression_count));
    add("publication_timestamp_regression_count",
        std::to_string(timestamp_diagnostics.publication_timestamp_regression_count));
    add("duplicate_measurement_suppressed_count",
        std::to_string(timestamp_diagnostics.duplicate_measurement_suppressed_count));
    add("timestamp_conversion_failure_count",
        std::to_string(timestamp_diagnostics.conversion_failure_count));
    add("timestamp_failure_reason", timestamp_diagnostics.failure_reason);
    add("timestamp_reason", last_timestamp_result_.reason);
    add("published_count", std::to_string(published_count_));
    add("gated_count", std::to_string(gated_count_));
    add("rejected_count", std::to_string(rejected_count_));
    add("covariance_rejected_count", std::to_string(covariance_rejected_count_));
    add("covariance_rejection_reason", last_rejection_reason_ ==
                                            "COVARIANCE_NOT_FLOAT_REPRESENTABLE"
                                        ? last_rejection_reason_
                                        : "NONE");
    add("last_rejection_reason", last_rejection_reason_);
    add("lio_public_frame_generation",
        std::to_string(supervisor_.has_value()
                           ? supervisor_->lio_public_frame_generation
                           : 0U));
    add("reset_counter", std::to_string(last_reset_counter_));
    add("geometric_jump_count", std::to_string(jump_latch_.count()));
    add("last_sample_timestamp_ns", std::to_string(last_sample_timestamp_ns_));
    add("last_transport_timestamp_ns", std::to_string(last_transport_timestamp_ns_));
    add("last_published_time_ns", std::to_string(last_published_time_ns_));
    add("supervisor_gate", last_gate_.supervisor_authorized ? "true" : "false");
    add("gate_reason", last_gate_.reason);
    add("px4_frame_generation_observed",
        std::to_string(supervisor_.has_value() ? supervisor_->px4_frame_generation : 0U));
    array.status.push_back(std::move(status));
    diagnostics_->publish(std::move(array));
  }

  rclcpp::Publisher<VehicleOdometry>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<SupervisorStatus>::SharedPtr supervisor_sub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::optional<SupervisorStatus> supervisor_;
  std::optional<ExternalOdometryFrame> last_published_;
  std::unique_ptr<TimestampConverter> timestamp_converter_;
  TimestampConversionResult last_timestamp_result_;
  ExternalOdometryGateResult last_gate_;
  std::int64_t max_age_ns_{150'000'000};
  double position_jump_m_{0.75};
  double orientation_jump_rad_{0.35};
  std::int64_t last_published_time_ns_{0};
  std::int64_t last_sample_timestamp_ns_{0};
  std::int64_t last_transport_timestamp_ns_{0};
  std::uint64_t published_count_{0};
  std::uint64_t gated_count_{0};
  std::uint64_t rejected_count_{0};
  std::uint64_t covariance_rejected_count_{0};
  std::uint8_t last_reset_counter_{0};
  GeometricJumpLatch jump_latch_;
  bool node_ready_{false};
  bool last_frame_valid_{false};
  bool publication_ready_{false};
  bool publication_active_{false};
  std::string last_rejection_reason_{"NONE"};
};

}  // namespace px4_odometry_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_odometry_bridge::Px4ExternalOdometryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
