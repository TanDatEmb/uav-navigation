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

#include "px4_odometry_bridge/external_odometry_conversion.hpp"
#include "px4_odometry_bridge/external_odometry_gate.hpp"
#include "px4_odometry_bridge/geometric_jump_continuity.hpp"
#include "px4_odometry_bridge/geometric_jump_latch.hpp"
#include "px4_odometry_bridge/timestamp_conversion.hpp"
#include "px4_odometry_bridge/topic_version.hpp"

namespace px4_odometry_bridge {

namespace {
constexpr char kLioPropagatedOdometryTopic[] = "/lio/odometry_propagated";
}  // namespace

class Px4ExternalOdometryBridgeNode final : public rclcpp::Node {
 public:
  using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
  static constexpr std::uint64_t kSimulationTimestampMappingGeneration = 1;

  Px4ExternalOdometryBridgeNode() : Node("px4_external_odometry_bridge") {
    max_age_ns_ = declare_parameter<std::int64_t>(
        "external_odometry.maximum_age_ns", 500'000'000);
    diagnostics_max_age_ns_ = declare_parameter<std::int64_t>(
        "external_odometry.diagnostics_maximum_age_ns", 2'000'000'000);
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
    if (max_age_ns_ <= 0 || diagnostics_max_age_ns_ <= 0 ||
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
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        kLioPropagatedOdometryTopic, rclcpp::QoS(20).reliable(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { on_lio(*message); });
    lio_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/lio/diagnostics", rclcpp::QoS(20).best_effort(),
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
          on_lio_diagnostics(*message);
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
    const auto now_ns = now().nanoseconds();
    for (const auto& status : message.status) {
      if (status.name == "fast_lio/estimator") {
        const auto generation = value_uint(status, "lio_public_frame_generation");
        const bool generation_valid = value_is_true(status, "lio_public_frame_generation_valid");
        const bool generation_changed = jump_latch_.observePublicFrameGeneration(
            generation_valid, generation);
        if (generation_changed) {
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
        last_lio_diagnostics_ns_ = now_ns;
      }
    }
  }

  bool lio_diagnostics_fresh(std::int64_t now_ns) const {
    // The propagated odometry callback is the authoritative high-rate
    // freshness signal.  Its diagnostic heartbeat is intentionally slower
    // and may be delivered late while the executor is busy with a motion
    // transition; requiring both clocks here can close an otherwise healthy
    // external-odometry stream for a full heartbeat interval.
    return last_lio_diagnostics_ns_ > 0 && now_ns >= last_lio_diagnostics_ns_ &&
           now_ns - last_lio_diagnostics_ns_ <= diagnostics_max_age_ns_;
  }

  void on_lio(const nav_msgs::msg::Odometry& message) {
    const auto frame = convert_ros_lio_odometry(message);
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
    const bool use_sim_time = get_parameter("use_sim_time").as_bool();
    const std::uint64_t public_generation =
        lio_public_frame_generation_valid_ ? lio_public_frame_generation_ : 0U;
    last_timestamp_result_ = timestamp_converter_->convert(
        frame->timestamp_ns, now_ns, use_sim_time,
        kSimulationTimestampMappingGeneration);
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
    jump_continuity_state_.last_received = last_received_;
    last_jump_observation_ = observe_geometric_jump_continuity(
      *frame, source_continuity_valid,
      lio_public_frame_generation_valid_ && public_generation > 0U,
      public_generation, jump_continuity_config_, jump_continuity_state_);
    last_received_ = jump_continuity_state_.last_received;
    (void)jump_latch_.observeGeometricJump(last_jump_observation_.jumped);

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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr lio_diagnostics_sub_;
  std::optional<ExternalOdometryFrame> last_published_;
  std::optional<ExternalOdometryFrame> last_received_;
  GeometricJumpContinuityState jump_continuity_state_;
  std::unique_ptr<TimestampConverter> timestamp_converter_;
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
  std::int64_t last_lio_diagnostics_ns_{0};
  bool node_ready_{false};
  bool lio_valid_{false};
  bool lio_covariance_valid_{false};
  bool lio_public_frame_generation_valid_{false};
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
