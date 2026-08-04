#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <navigation_interfaces/msg/odometry_supervisor_status.hpp>

#include "px4_odometry_bridge/external_odometry_conversion.hpp"
#include "px4_odometry_bridge/topic_version.hpp"

namespace px4_odometry_bridge {

class Px4ExternalOdometryBridgeNode final : public rclcpp::Node {
 public:
  using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
  using SupervisorStatus = navigation_interfaces::msg::OdometrySupervisorStatus;

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
        [this](SupervisorStatus::ConstSharedPtr message) {
          if (!last_supervisor_frame_generation_.has_value() ||
              *last_supervisor_frame_generation_ != message->px4_frame_generation) {
            jump_latched_ = false;
          }
          last_supervisor_frame_generation_ = message->px4_frame_generation;
          supervisor_ = *message;
        });
    diagnostics_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                           [this] { publish_diagnostics(); });
    node_ready_ = true;
  }

 private:
  static std::int64_t time_ns(const builtin_interfaces::msg::Time& time) {
    return static_cast<std::int64_t>(time.sec) * 1'000'000'000LL +
           static_cast<std::int64_t>(time.nanosec);
  }

  void on_lio(const nav_msgs::msg::Odometry& message) {
    const auto frame = convert_ros_lio_odometry(message);
    if (!frame) {
      ++rejected_count_;
      return;
    }
    const auto now_ns = now().nanoseconds();
    last_sample_timestamp_ns_ = frame->timestamp_ns;
    last_transport_timestamp_ns_ = now_ns;
    const bool use_sim_time = get_parameter("use_sim_time").as_bool();
    const bool timestamp_conversion_valid = use_sim_time && now_ns > 0 &&
                                            frame->timestamp_ns > 0 &&
                                            std::llabs(now_ns - frame->timestamp_ns) <= max_age_ns_;
    const bool transport_ready = output_->get_subscription_count() > 0;
    if (!supervisor_.has_value() || !supervisor_->external_measurement_authorized ||
        !node_ready_ || !transport_ready || !timestamp_conversion_valid ||
        time_ns(supervisor_->evaluation_time) <= 0 ||
        now_ns < time_ns(supervisor_->evaluation_time) ||
        now_ns - time_ns(supervisor_->evaluation_time) > max_age_ns_) {
      ++gated_count_;
      return;
    }

    bool frame_jump = false;
    if (last_published_.has_value()) {
      frame_jump = (frame->position_frd - last_published_->position_frd).norm() > position_jump_m_ ||
                   last_published_->orientation_frd.angularDistance(frame->orientation_frd) >
                       orientation_jump_rad_;
    }
    if (frame_jump && !jump_latched_) {
      ++geometric_jump_count_;
      jump_latched_ = true;
    }
    if (frame_jump || jump_latched_) {
      ++gated_count_;
      return;
    }

    VehicleOdometry output;
    output.timestamp = static_cast<std::uint64_t>(now_ns / 1'000);
    output.timestamp_sample = static_cast<std::uint64_t>(frame->timestamp_ns / 1'000);
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
    output.position_variance = {static_cast<float>(frame->position_variance.x()),
                                static_cast<float>(frame->position_variance.y()),
                                static_cast<float>(frame->position_variance.z())};
    output.orientation_variance = {static_cast<float>(frame->orientation_variance.x()),
                                   static_cast<float>(frame->orientation_variance.y()),
                                   static_cast<float>(frame->orientation_variance.z())};
    output.velocity_variance = {static_cast<float>(frame->velocity_variance.x()),
                                static_cast<float>(frame->velocity_variance.y()),
                                static_cast<float>(frame->velocity_variance.z())};
    output.reset_counter = static_cast<std::uint8_t>(
        supervisor_->px4_frame_generation & 0xffU);
    output.quality = supervisor_->health == SupervisorStatus::HEALTHY ? 100 : 50;
    output_->publish(output);
    last_published_ = *frame;
    last_published_time_ns_ = frame->timestamp_ns;
    ++published_count_;
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "px4_external_odometry_bridge";
    const bool transport_ready = output_->get_subscription_count() > 0;
    const auto now_ns = now().nanoseconds();
    const bool use_sim_time = get_parameter("use_sim_time").as_bool();
    const bool timestamp_conversion_valid = use_sim_time && now_ns > 0 &&
                                            last_sample_timestamp_ns_ > 0 &&
                                            std::llabs(now_ns - last_sample_timestamp_ns_) <= max_age_ns_;
    const bool time_sync_ready = timestamp_conversion_valid;
    const bool infrastructure_ready = node_ready_ && transport_ready && time_sync_ready;
    status.level = infrastructure_ready ? diagnostic_msgs::msg::DiagnosticStatus::OK
                          : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = !infrastructure_ready ? "external odometry infrastructure is not ready"
                         : supervisor_.has_value() && supervisor_->external_measurement_authorized
                         ? "external odometry publishing is enabled"
                         : "external odometry gate is closed";
    const auto add = [&status](std::string key, std::string value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = std::move(key);
      item.value = std::move(value);
      status.values.push_back(std::move(item));
    };
    add("diagnostic_schema_version", "1");
    add("ready", infrastructure_ready ? "true" : "false");
    add("node_ready", node_ready_ ? "true" : "false");
    add("transport_ready", transport_ready ? "true" : "false");
    add("time_sync_ready", time_sync_ready ? "true" : "false");
    add("publication_authorized", supervisor_.has_value() &&
                                      supervisor_->external_measurement_authorized
                                  ? "true" : "false");
    add("publisher_subscription_count",
        std::to_string(output_->get_subscription_count()));
    add("timestamp_domain", use_sim_time ? "ros_sim_time" : "TIME_DOMAIN_UNRESOLVED");
    add("timestamp_conversion_valid", timestamp_conversion_valid ? "true" : "false");
    add("publisher_topic", versioned_topic<VehicleOdometry>("/fmu/in/vehicle_visual_odometry"));
    add("pose_frame", "POSE_FRAME_FRD");
    add("velocity_frame", "VELOCITY_FRAME_BODY_FRD");
    add("timestamp_source", "measurement_header_stamp_for_timestamp_sample");
    add("published_count", std::to_string(published_count_));
    add("gated_count", std::to_string(gated_count_));
    add("rejected_count", std::to_string(rejected_count_));
    add("reset_counter", std::to_string(supervisor_.has_value()
                                            ? supervisor_->px4_frame_generation
                                            : 0));
    add("frame_generation", std::to_string(supervisor_.has_value()
                                                ? supervisor_->px4_frame_generation
                                                : 0));
    add("geometric_jump_count", std::to_string(geometric_jump_count_));
    add("geometric_jump_gate_closed", jump_latched_ ? "true" : "false");
    add("last_sample_timestamp_ns", std::to_string(last_sample_timestamp_ns_));
    add("last_transport_timestamp_ns", std::to_string(last_transport_timestamp_ns_));
    add("last_published_time_ns", std::to_string(last_published_time_ns_));
    add("supervisor_gate", supervisor_.has_value() &&
                                   supervisor_->external_measurement_authorized
                               ? "true" : "false");
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
  std::int64_t max_age_ns_{150'000'000};
  double position_jump_m_{0.75};
  double orientation_jump_rad_{0.35};
  std::int64_t last_published_time_ns_{0};
  std::int64_t last_sample_timestamp_ns_{0};
  std::int64_t last_transport_timestamp_ns_{0};
  std::uint64_t published_count_{0};
  std::uint64_t gated_count_{0};
  std::uint64_t rejected_count_{0};
  std::uint64_t geometric_jump_count_{0};
  bool jump_latched_{false};
  bool node_ready_{false};
  std::optional<std::uint64_t> last_supervisor_frame_generation_;
};

}  // namespace px4_odometry_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_odometry_bridge::Px4ExternalOdometryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
