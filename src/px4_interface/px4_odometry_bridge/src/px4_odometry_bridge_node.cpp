#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <navigation_interfaces/srv/sample_odometry_at_time.hpp>

#include "px4_odometry_bridge/frame_converter.hpp"
#include "px4_odometry_bridge/odometry_ring_buffer.hpp"
#include "px4_odometry_bridge/reset_compensator.hpp"
#include "px4_odometry_bridge/time_validator.hpp"
#include "px4_odometry_bridge/topic_version.hpp"

namespace px4_odometry_bridge {

class Px4OdometryBridgeNode final : public rclcpp::Node {
 public:
  using SampleService = navigation_interfaces::srv::SampleOdometryAtTime;
  using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
  using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
  using VehicleAttitude = px4_msgs::msg::VehicleAttitude;
  using TimesyncStatus = px4_msgs::msg::TimesyncStatus;

  Px4OdometryBridgeNode() : Node("px4_odometry_bridge") {
    simulation_clock_ = declare_parameter<bool>("simulation_clock", false);
    xrce_synchronized_ = declare_parameter<bool>("xrce_synchronized", false);
    output_ = create_publisher<nav_msgs::msg::Odometry>("/px4/odometry_ros", 10);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/px4/diagnostics", rclcpp::QoS(1).transient_local());
    const auto px4_output_qos = rclcpp::QoS(1).best_effort();
    odometry_sub_ = create_subscription<VehicleOdometry>(
        versioned_topic<VehicleOdometry>("/fmu/out/vehicle_odometry"), px4_output_qos,
        [this](VehicleOdometry::ConstSharedPtr message) { on_odometry(*message); });
    local_position_sub_ = create_subscription<VehicleLocalPosition>(
        versioned_topic<VehicleLocalPosition>("/fmu/out/vehicle_local_position"), px4_output_qos,
        [this](VehicleLocalPosition::ConstSharedPtr message) {
          latest_detail_timestamp_us_ = message->timestamp_sample;
        });
    attitude_sub_ = create_subscription<VehicleAttitude>(
        versioned_topic<VehicleAttitude>("/fmu/out/vehicle_attitude"), px4_output_qos,
        [this](VehicleAttitude::ConstSharedPtr message) {
          latest_detail_timestamp_us_ = message->timestamp_sample;
        });
    timesync_sub_ = create_subscription<TimesyncStatus>(
        "/fmu/out/timesync_status", px4_output_qos, [this](TimesyncStatus::ConstSharedPtr message) {
          last_timesync_timestamp_us_ = message->timestamp;
          timesync_seen_ = true;
        });
    service_ = create_service<SampleService>(
        "/px4/sample_odometry_at_time",
        [this](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<SampleService::Request> request,
               std::shared_ptr<SampleService::Response> response) {
          const auto query = checked_microseconds_to_nanoseconds(
              static_cast<std::uint64_t>(request->sample_time.sec) * 1'000'000ULL +
              static_cast<std::uint64_t>(request->sample_time.nanosec / 1000U));
          if (!query) {
            response->success = false;
            response->reason = "requested time is invalid";
            return;
          }
          const auto result = history_.sample(*query);
          if (!result) {
            response->success = false;
            response->reason = "time is outside buffered history or crosses a generation";
            return;
          }
          response->success = true;
          response->reason = "";
          response->odometry = to_ros(result->value);
          response->interpolated = result->interpolated;
          response->reset_generation = result->value.reset_generation;
          response->time_generation = result->value.time_generation;
          response->component_validity_mask = 0x7U;
          response->covariance_availability_mask = 0x7U;
        });
  }

 private:
  void on_odometry(const VehicleOdometry &message) {
    const auto timestamp_ns = checked_microseconds_to_nanoseconds(message.timestamp_sample);
    if (!timestamp_ns) return;
    const auto time_result = time_validator_.observe(message.timestamp_sample, *timestamp_ns);
    if (!time_result.accepted) return;

    Px4OdometrySample sample;
    sample.timestamp_ns = *timestamp_ns;
    sample.pose_frame = static_cast<PoseFrame>(message.pose_frame);
    sample.velocity_frame = static_cast<VelocityFrame>(message.velocity_frame);
    sample.position = Eigen::Vector3d(message.position[0], message.position[1], message.position[2]);
    sample.orientation = Eigen::Quaterniond(message.q[0], message.q[1], message.q[2], message.q[3]);
    sample.velocity = Eigen::Vector3d(message.velocity[0], message.velocity[1], message.velocity[2]);
    sample.angular_velocity = Eigen::Vector3d(message.angular_velocity[0], message.angular_velocity[1],
                                              message.angular_velocity[2]);
    sample.position_variance = Eigen::Vector3d(message.position_variance[0], message.position_variance[1],
                                               message.position_variance[2]);
    sample.velocity_variance = Eigen::Vector3d(message.velocity_variance[0], message.velocity_variance[1],
                                               message.velocity_variance[2]);
    sample.orientation_variance = Eigen::Vector3d(message.orientation_variance[0],
                                                  message.orientation_variance[1],
                                                  message.orientation_variance[2]);
    sample.reset_counter = message.reset_counter;
    sample.angular_velocity_valid = sample.angular_velocity.allFinite();
    auto converted = converter_.convert(sample);
    if (!converted) return;
    const bool detailed = latest_detail_timestamp_us_.has_value() &&
                          std::llabs(static_cast<long long>(*latest_detail_timestamp_us_) -
                                     static_cast<long long>(message.timestamp_sample)) <= 50'000;
    auto continuous = reset_compensator_.observe(*converted.value, {.available = detailed});
    if (!continuous) return;
    continuous->time_generation = time_result.generation;
    history_.push(*continuous);
    output_->publish(to_ros(*continuous));
    publish_diagnostics("running", "VehicleOdometry converted and published");
  }

  void publish_diagnostics(const std::string &state, const std::string &message) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "px4_odometry_bridge";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = message;
    const auto add_value = [&status](std::string key, std::string value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = std::move(key);
      item.value = std::move(value);
      status.values.push_back(std::move(item));
    };
    add_value("state", state);
    add_value("source", "VehicleOdometry");
    add_value("output_frame", "odom");
    add_value("output_child_frame", "base_link");
    add_value("simulation_clock", simulation_clock_ ? "true" : "false");
    add_value("xrce_synchronized", xrce_synchronized_ ? "true" : "false");
    add_value("timesync_seen", timesync_seen_ ? "true" : "false");
    add_value("buffer_size", std::to_string(history_.size()));
    array.status.push_back(status);
    diagnostics_->publish(array);
  }

  static nav_msgs::msg::Odometry to_ros(const ConvertedOdometry &sample) {
    nav_msgs::msg::Odometry message;
    message.header.stamp = rclcpp::Time(sample.timestamp_ns);
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = sample.position.x();
    message.pose.pose.position.y = sample.position.y();
    message.pose.pose.position.z = sample.position.z();
    message.pose.pose.orientation.w = sample.orientation.w();
    message.pose.pose.orientation.x = sample.orientation.x();
    message.pose.pose.orientation.y = sample.orientation.y();
    message.pose.pose.orientation.z = sample.orientation.z();
    message.twist.twist.linear.x = sample.velocity_body.x();
    message.twist.twist.linear.y = sample.velocity_body.y();
    message.twist.twist.linear.z = sample.velocity_body.z();
    message.twist.twist.angular.x = sample.angular_velocity_body.x();
    message.twist.twist.angular.y = sample.angular_velocity_body.y();
    message.twist.twist.angular.z = sample.angular_velocity_body.z();
    for (int i = 0; i < 3; ++i) {
      message.pose.covariance[static_cast<std::size_t>(i * 6 + i)] = sample.position_variance[i];
      message.twist.covariance[static_cast<std::size_t>(i * 6 + i)] = sample.velocity_variance[i];
    }
    return message;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Service<SampleService>::SharedPtr service_;
  FrameConverter converter_;
  ResetCompensator reset_compensator_;
  TimestampValidator time_validator_;
  OdometryRingBuffer history_;
  std::optional<std::uint64_t> latest_detail_timestamp_us_;
  std::optional<std::uint64_t> last_timesync_timestamp_us_;
  bool simulation_clock_{false};
  bool xrce_synchronized_{false};
  bool timesync_seen_{false};
};

}  // namespace px4_odometry_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_odometry_bridge::Px4OdometryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
