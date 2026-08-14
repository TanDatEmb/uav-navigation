#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
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

#include <navigation_interfaces/odometry_validity.hpp>
#include <navigation_interfaces/srv/sample_odometry_at_time.hpp>

#include "px4_odometry_bridge/frame_converter.hpp"
#include "px4_odometry_bridge/odometry_ring_buffer.hpp"
#include "px4_odometry_bridge/frame_generation_policy.hpp"
#include "px4_odometry_bridge/reset_compensator.hpp"
#include "px4_odometry_bridge/time_validator.hpp"
#include "px4_odometry_bridge/topic_version.hpp"

namespace px4_odometry_bridge {

namespace {
constexpr char kOutputTopic[] = "/px4/estimator_odometry";
// The message has already crossed the PX4 NED/FRD -> ROS ENU/FLU boundary.
// The frame name identifies the PX4-originated local origin; it must not be
// interpreted as raw PX4 NED coordinates by the initial-prior consumer.
constexpr char kOutputFrame[] = "px4_odom";

const char *world_convention_name(WorldConvention convention) {
  switch (convention) {
    case WorldConvention::kRosEnu: return "ROS_ENU";
    case WorldConvention::kPx4FrdLocal: return "PX4_FRD_LOCAL_ZUP";
    default: return "UNKNOWN";
  }
}
}  // namespace

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
    maximum_metadata_age_ns_ = declare_parameter<std::int64_t>(
        "reset.maximum_metadata_age_ns", 100'000'000);
    maximum_metadata_association_gap_ns_ = declare_parameter<std::int64_t>(
        "reset.maximum_event_association_gap_ns", 5'000'000);
    stable_samples_after_reset_ = declare_parameter<std::int64_t>(
        "reset.stable_samples_after_reset", 3);
    if (maximum_metadata_age_ns_ <= 0 || maximum_metadata_association_gap_ns_ < 0 ||
        stable_samples_after_reset_ <= 0) {
      throw std::invalid_argument("reset metadata/stable-sample configuration must be positive");
    }
    history_.setStableSamples(static_cast<std::size_t>(stable_samples_after_reset_));
    output_ = create_publisher<nav_msgs::msg::Odometry>(kOutputTopic, 10);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/px4/diagnostics", rclcpp::QoS(1).transient_local());
    const auto px4_output_qos = rclcpp::QoS(1).best_effort();
    odometry_sub_ = create_subscription<VehicleOdometry>(
        versioned_topic<VehicleOdometry>("/fmu/out/vehicle_odometry"), px4_output_qos,
        [this](VehicleOdometry::ConstSharedPtr message) { on_odometry(*message); });
    local_position_sub_ = create_subscription<VehicleLocalPosition>(
        versioned_topic<VehicleLocalPosition>("/fmu/out/vehicle_local_position"), px4_output_qos,
        [this](VehicleLocalPosition::ConstSharedPtr message) {
          record_local_reset_metadata(*message);
        });
    attitude_sub_ = create_subscription<VehicleAttitude>(
        versioned_topic<VehicleAttitude>("/fmu/out/vehicle_attitude"), px4_output_qos,
        [this](VehicleAttitude::ConstSharedPtr message) {
          record_attitude_reset_metadata(*message);
        });
    timesync_sub_ = create_subscription<TimesyncStatus>(
        "/fmu/out/timesync_status", px4_output_qos, [this](TimesyncStatus::ConstSharedPtr message) {
          static_cast<void>(message);
          timesync_seen_ = true;
        });
    service_ = create_service<SampleService>(
        "/px4/sample_odometry_at_time",
        [this](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<SampleService::Request> request,
               std::shared_ptr<SampleService::Response> response) {
          const auto query = checked_ros_time_to_nanoseconds(
              request->sample_time.sec, request->sample_time.nanosec);
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
          response->reset_generation = reset_event_generation_;
          response->reset_event_generation = reset_event_generation_;
          response->frame_generation = result->value.frame_generation;
          response->time_generation = result->value.time_generation;
          response->component_validity_mask = component_validity_mask(result->value);
          response->covariance_availability_mask = covariance_availability_mask(result->value);
        });
    diagnostics_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      const auto now_ns = now().nanoseconds();
      if (output_valid_ && continuity_valid_ && last_valid_sample_time_ns_ > 0 &&
          now_ns >= last_valid_sample_time_ns_ &&
          now_ns - last_valid_sample_time_ns_ <= time_validator_max_stale_ns_) {
        publish_diagnostics("running", "periodic bridge health snapshot");
      } else if (!output_valid_) {
        publish_diagnostics("waiting", "periodic bridge health snapshot");
      } else {
        publish_diagnostics("stale", "PX4 odometry output is stale");
      }
    });
  }

 private:
  static constexpr std::size_t kResetHistoryCapacity = 64;

  static std::uint8_t counter_delta(std::uint8_t before, std::uint8_t after) {
    return static_cast<std::uint8_t>(after - before);
  }

  void insert_metadata(std::deque<DetailedResetMetadata>& history,
                       DetailedResetMetadata metadata) {
    const auto position = std::upper_bound(
        history.begin(), history.end(), metadata.timestamp_ns,
        [](std::int64_t timestamp, const DetailedResetMetadata& entry) {
          return timestamp < entry.timestamp_ns;
        });
    history.insert(position, std::move(metadata));
    while (history.size() > kResetHistoryCapacity) history.pop_front();
  }

  void record_local_reset_metadata(const VehicleLocalPosition& message) {
    DetailedResetMetadata metadata;
    const auto timestamp = checked_microseconds_to_nanoseconds(message.timestamp_sample);
    if (!timestamp) return;
    metadata.timestamp_ns = *timestamp;
    metadata.matched_odometry_timestamp_ns = *timestamp;
    metadata.position_delta_source =
        Eigen::Vector3d(message.delta_xy[0], message.delta_xy[1], message.delta_z);
    metadata.velocity_delta_source =
        Eigen::Vector3d(message.delta_vxy[0], message.delta_vxy[1], message.delta_vz);
    metadata.heading_delta_rad = message.delta_heading;
    metadata.xy_reset_counter = message.xy_reset_counter;
    metadata.z_reset_counter = message.z_reset_counter;
    metadata.vxy_reset_counter = message.vxy_reset_counter;
    metadata.vz_reset_counter = message.vz_reset_counter;
    metadata.heading_reset_counter = message.heading_reset_counter;
    insert_metadata(local_reset_history_, std::move(metadata));
  }

  void record_attitude_reset_metadata(const VehicleAttitude& message) {
    DetailedResetMetadata metadata;
    const auto timestamp = checked_microseconds_to_nanoseconds(message.timestamp_sample);
    if (!timestamp) return;
    metadata.timestamp_ns = *timestamp;
    metadata.matched_odometry_timestamp_ns = *timestamp;
    metadata.attitude_delta = Eigen::Quaterniond(
        message.delta_q_reset[0], message.delta_q_reset[1], message.delta_q_reset[2],
        message.delta_q_reset[3]);
    metadata.attitude_reset_counter = message.quat_reset_counter;
    insert_metadata(attitude_reset_history_, std::move(metadata));
  }

  std::optional<DetailedResetMetadata> metadata_at(
      const std::deque<DetailedResetMetadata>& history, std::int64_t timestamp_ns) const {
    if (history.empty()) return std::nullopt;
    const auto position = std::upper_bound(
        history.begin(), history.end(), timestamp_ns,
        [](std::int64_t timestamp, const DetailedResetMetadata& entry) {
          return timestamp < entry.timestamp_ns;
        });
    if (position == history.begin()) return std::nullopt;
    const auto current = std::prev(position);
    if (timestamp_ns < current->timestamp_ns ||
        timestamp_ns - current->timestamp_ns > maximum_metadata_age_ns_) {
      return std::nullopt;
    }
    DetailedResetMetadata result = *current;
    result.matched_odometry_timestamp_ns = timestamp_ns;
    if (current == history.begin()) return result;
    const auto previous = std::prev(current);
    const auto local_delta = counter_delta(previous->xy_reset_counter, current->xy_reset_counter);
    const auto z_delta = counter_delta(previous->z_reset_counter, current->z_reset_counter);
    const auto vxy_delta = counter_delta(previous->vxy_reset_counter, current->vxy_reset_counter);
    const auto vz_delta = counter_delta(previous->vz_reset_counter, current->vz_reset_counter);
    const auto heading_delta = counter_delta(previous->heading_reset_counter,
                                             current->heading_reset_counter);
    const auto attitude_delta = counter_delta(previous->attitude_reset_counter,
                                              current->attitude_reset_counter);
    if (local_delta > 1 || z_delta > 1 || vxy_delta > 1 || vz_delta > 1 ||
        heading_delta > 1 || attitude_delta > 1) {
      result.association_invalid = true;
      result.available = false;
      return result;
    }
    result.position_xy_reset = local_delta == 1;
    result.position_z_reset = z_delta == 1;
    result.velocity_xy_reset = vxy_delta == 1;
    result.velocity_z_reset = vz_delta == 1;
    result.heading_reset = heading_delta == 1;
    result.attitude_reset = attitude_delta == 1;
    result.available = !result.association_invalid && result.hasReset();
    return result;
  }

  std::optional<DetailedResetMetadata> detailed_metadata_for(std::int64_t timestamp_ns) const {
    const auto local = metadata_at(local_reset_history_, timestamp_ns);
    const auto attitude = metadata_at(attitude_reset_history_, timestamp_ns);
    if (!local && !attitude) return std::nullopt;
    DetailedResetMetadata result;
    result.matched_odometry_timestamp_ns = timestamp_ns;
    result.association_invalid =
        (local && local->association_invalid) ||
        (attitude && attitude->association_invalid);
    if (local && attitude && local->hasReset() && attitude->hasReset() &&
        std::llabs(local->timestamp_ns - attitude->timestamp_ns) >
            maximum_metadata_association_gap_ns_) {
      result.association_invalid = true;
    }
    if (local) {
      result.position_xy_reset = local->position_xy_reset;
      result.position_z_reset = local->position_z_reset;
      result.velocity_xy_reset = local->velocity_xy_reset;
      result.velocity_z_reset = local->velocity_z_reset;
      result.heading_reset = local->heading_reset;
      result.position_delta_source = local->position_delta_source;
      result.velocity_delta_source = local->velocity_delta_source;
      result.heading_delta_rad = local->heading_delta_rad;
      result.xy_reset_counter = local->xy_reset_counter;
      result.z_reset_counter = local->z_reset_counter;
      result.vxy_reset_counter = local->vxy_reset_counter;
      result.vz_reset_counter = local->vz_reset_counter;
      result.heading_reset_counter = local->heading_reset_counter;
    }
    if (attitude) {
      result.attitude_reset = attitude->attitude_reset;
      result.attitude_delta = attitude->attitude_delta;
      result.attitude_reset_counter = attitude->attitude_reset_counter;
    }
    result.available = !result.association_invalid && result.hasReset();
    return result;
  }

  void on_odometry(const VehicleOdometry &message) {
    const auto timestamp_ns = checked_microseconds_to_nanoseconds(message.timestamp_sample);
    if (!timestamp_ns) {
      ++timestamp_rejected_count_;
      publish_diagnostics("rejected", "invalid PX4 timestamp");
      return;
    }
    const auto time_result = time_validator_.observe(message.timestamp_sample, now().nanoseconds());
    if (!time_result.accepted) {
      ++timestamp_rejected_count_;
      publish_diagnostics("rejected", time_result.reason);
      return;
    }
    if (time_result.event == TimestampEvent::kProbableSourceRestart) {
      frame_generation_ = frame_generation_after_source_restart(frame_generation_, output_valid_);
      history_.clear();
      local_reset_history_.clear();
      attitude_reset_history_.clear();
      reset_compensator_.clear();
      converter_.reset_sign_continuity();
      startup_origin_rebased_ = false;
      output_valid_ = false;
      continuity_valid_ = false;
      last_valid_sample_time_ns_ = 0;
      last_time_generation_ = time_result.generation;
      last_px4_timestamp_sample_ns_ = 0;
      last_px4_ros_output_stamp_ns_ = 0;
      publish_diagnostics("restarting_time_generation", time_result.reason);
    }

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
    if (!converted) {
      ++conversion_rejected_count_;
      publish_diagnostics("rejected", "PX4 frame conversion rejected sample");
      return;
    }
    const auto metadata = detailed_metadata_for(*timestamp_ns);
    const DetailedResetMetadata associated_metadata = metadata.value_or(DetailedResetMetadata{});
    const bool reset_counter_changed = last_input_reset_counter_.has_value() &&
                                       *last_input_reset_counter_ != sample.reset_counter;
    auto observation = reset_compensator_.observe(*converted.value, associated_metadata);
    last_reset_observation_status_ = toString(observation.status);
    if (observation.status == ResetObservationStatus::kResetTransitionSuppressed) {
      ++reset_event_generation_;
    }
    if (reset_counter_changed && output_valid_ &&
        (observation.status == ResetObservationStatus::kInvalidMetadata ||
         observation.status == ResetObservationStatus::kCounterDiscontinuity ||
         observation.status == ResetObservationStatus::kInvalidResetRotation)) {
      // The reset cannot be compensated, so the old public PX4 frame ends.
      // This is distinct from the reset event counter and from the geometric
      // jump guard in the external publisher.
      ++frame_generation_;
      history_.clear();
      reset_compensator_.clear();
      converter_.reset_sign_continuity();
      startup_origin_rebased_ = false;
      continuity_valid_ = false;
      observation = reset_compensator_.observe(*converted.value);
      last_reset_observation_status_ = toString(observation.status);
    }
    if (!observation.accepted() && !output_valid_ &&
        observation.status == ResetObservationStatus::kMetadataPending &&
        !associated_metadata.association_invalid) {
      // A bridge can receive VehicleOdometry before the first
      // VehicleLocalPosition/VehicleAttitude reset metadata sample. There is
      // no downstream odometry frame yet, so establish the startup baseline
      // from this current sample instead of suppressing forever. Once an
      // output has been published, the normal metadata requirement remains
      // strict and a reset cannot silently change the LIO world frame.
      history_.clear();
      reset_compensator_.clear();
      converter_.reset_sign_continuity();
      startup_origin_rebased_ = false;
      continuity_valid_ = false;
      last_valid_sample_time_ns_ = 0;
      observation = reset_compensator_.observe(*converted.value);
      last_reset_observation_status_ = toString(observation.status);
      ++startup_rebaseline_count_;
      ++startup_rebaseline_generation_;
      publish_diagnostics("stabilizing", "PX4 startup baseline reinitialized before reset metadata");
    }
    if (!observation.accepted()) {
      ++reset_suppressed_count_;
      continuity_valid_ = false;
      publish_diagnostics("suppressed", std::string("PX4 reset observation: ") +
                                      toString(observation.status));
      return;
    }
    auto continuous = std::move(observation.sample);
    continuous->frame_generation = frame_generation_;
    continuous->reset_generation = reset_event_generation_;
    continuous->reset_event_generation = reset_event_generation_;
    if (!startup_origin_rebased_) {
      const auto startup_origin = reset_compensator_.rebasePositionAtCurrentOutput();
      if (!startup_origin.has_value()) {
        continuity_valid_ = false;
        publish_diagnostics("suppressed", "PX4 startup origin was not finite");
        return;
      }
      continuous->position -= *startup_origin;
      startup_origin_rebased_ = true;
    }
    continuous->time_generation = time_result.generation;
    if (!history_.push(*continuous)) {
      continuity_valid_ = false;
      publish_diagnostics("stabilizing", "PX4 post-reset stable sample gate active");
      return;
    }
    const auto output = to_ros(*continuous);
    continuity_valid_ = true;
    output_valid_ = true;
    last_world_convention_ = converted.value->world_convention;
    last_valid_sample_time_ns_ = continuous->timestamp_ns;
    last_time_generation_ = time_result.generation;
    last_input_reset_counter_ = sample.reset_counter;
    last_px4_timestamp_sample_ns_ = continuous->timestamp_ns;
    last_px4_ros_output_stamp_ns_ =
        static_cast<std::int64_t>(output.header.stamp.sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(output.header.stamp.nanosec);
    output_->publish(output);
    publish_diagnostics("running", "VehicleOdometry converted and published");
  }

  void publish_diagnostics(const std::string &state, const std::string &message) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "px4_odometry_bridge";
    status.level = state == "running" ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                       : (state == "waiting" || state == "stabilizing"
                                              ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                              : diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    status.message = message;
    const auto add_value = [&status](std::string key, std::string value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = std::move(key);
      item.value = std::move(value);
      status.values.push_back(std::move(item));
    };
    add_value("state", state);
    add_value("diagnostic_schema_version", "1");
    add_value("source", "VehicleOdometry");
    add_value("output_valid", output_valid_ ? "true" : "false");
    add_value("continuity_valid", continuity_valid_ ? "true" : "false");
    add_value("last_valid_sample_time_ns", std::to_string(last_valid_sample_time_ns_));
    add_value("reset_generation", std::to_string(reset_event_generation_));
    add_value("reset_event_generation", std::to_string(reset_event_generation_));
    add_value("frame_generation", std::to_string(frame_generation_));
    add_value("time_generation", std::to_string(last_time_generation_));
    add_value("post_reset_stable", history_.postResetStable() ? "true" : "false");
    add_value("post_reset_stable_sample_count",
              std::to_string(history_.stableSampleCount()));
    add_value("post_reset_stable_samples_required",
              std::to_string(history_.stableSamplesRequired()));
    add_value("startup_origin_rebased", startup_origin_rebased_ ? "true" : "false");
    add_value("output_topic", kOutputTopic);
    add_value("output_frame", kOutputFrame);
    add_value("output_child_frame", "base_link");
    add_value("world_convention", world_convention_name(last_world_convention_));
    add_value("simulation_clock", simulation_clock_ ? "true" : "false");
    add_value("bridge_use_sim_time", get_parameter("use_sim_time").as_bool() ? "true" : "false");
    add_value("xrce_synchronized", xrce_synchronized_ ? "true" : "false");
    add_value("timesync_seen", timesync_seen_ ? "true" : "false");
    add_value("px4_timestamp_sample_ns", std::to_string(last_px4_timestamp_sample_ns_));
    add_value("px4_ros_output_stamp_ns", std::to_string(last_px4_ros_output_stamp_ns_));
    add_value("bridge_node_now_ns", std::to_string(now().nanoseconds()));
    add_value("buffer_size", std::to_string(history_.size()));
    add_value("timestamp_rejected_count", std::to_string(timestamp_rejected_count_));
    add_value("conversion_rejected_count", std::to_string(conversion_rejected_count_));
    add_value("reset_suppressed_count", std::to_string(reset_suppressed_count_));
    add_value("startup_rebaseline_count", std::to_string(startup_rebaseline_count_));
    add_value("startup_rebaseline_generation",
              std::to_string(startup_rebaseline_generation_));
    add_value("reset_observation_status", last_reset_observation_status_);
    array.status.push_back(status);
    diagnostics_->publish(array);
  }

  static nav_msgs::msg::Odometry to_ros(const ConvertedOdometry &sample) {
    // ConvertedOdometry is ROS ENU/FLU here: position/attitude are expressed
    // in the converted local ENU world and twist is expressed in base_link
    // FLU, exactly as required by nav_msgs/Odometry.
    nav_msgs::msg::Odometry message;
    message.header.stamp = rclcpp::Time(sample.timestamp_ns);
    message.header.frame_id = kOutputFrame;
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
    message.pose.covariance.fill(0.0);
    message.twist.covariance.fill(0.0);
    for (int i = 0; i < 3; ++i) {
      if (sample.position_covariance_available) {
        message.pose.covariance[static_cast<std::size_t>(i * 6 + i)] = sample.position_variance[i];
      }
      if (sample.orientation_covariance_available) {
        message.pose.covariance[static_cast<std::size_t>((i + 3) * 6 + (i + 3))] =
            sample.orientation_variance[i];
      }
      if (sample.velocity_covariance_available) {
        message.twist.covariance[static_cast<std::size_t>(i * 6 + i)] = sample.velocity_variance[i];
      }
    }
    message.twist.covariance[21] = -1.0;
    message.twist.covariance[28] = -1.0;
    message.twist.covariance[35] = -1.0;
    return message;
  }

  static std::uint32_t component_validity_mask(const ConvertedOdometry& sample) {
    std::uint32_t mask = 0;
    if (sample.position.allFinite()) mask |= navigation_interfaces::kPositionValid;
    if (sample.orientation.coeffs().allFinite() && sample.orientation.norm() > 1e-9) {
      mask |= navigation_interfaces::kOrientationValid;
    }
    if (sample.velocity_body.allFinite()) mask |= navigation_interfaces::kLinearVelocityValid;
    if (sample.angular_velocity_body.allFinite() && sample.angular_velocity_valid) {
      mask |= navigation_interfaces::kAngularVelocityValid;
    }
    return mask;
  }

  static std::uint32_t covariance_availability_mask(const ConvertedOdometry& sample) {
    std::uint32_t mask = 0;
    if (sample.position_covariance_available) {
      mask |= navigation_interfaces::kPositionCovarianceAvailable;
    }
    if (sample.orientation_covariance_available) {
      mask |= navigation_interfaces::kOrientationCovarianceAvailable;
    }
    if (sample.velocity_covariance_available) {
      mask |= navigation_interfaces::kLinearVelocityCovarianceAvailable;
    }
    return mask;
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<TimesyncStatus>::SharedPtr timesync_sub_;
  rclcpp::Service<SampleService>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  FrameConverter converter_;
  ResetCompensator reset_compensator_;
  TimestampValidator time_validator_;
  OdometryRingBuffer history_;
  std::deque<DetailedResetMetadata> local_reset_history_;
  std::deque<DetailedResetMetadata> attitude_reset_history_;
  std::int64_t maximum_metadata_age_ns_{100'000'000};
  std::int64_t maximum_metadata_association_gap_ns_{5'000'000};
  std::int64_t stable_samples_after_reset_{3};
  static constexpr std::int64_t time_validator_max_stale_ns_{200'000'000};
  bool simulation_clock_{false};
  bool xrce_synchronized_{false};
  bool timesync_seen_{false};
  std::int64_t last_px4_timestamp_sample_ns_{0};
  std::int64_t last_px4_ros_output_stamp_ns_{0};
  std::uint64_t timestamp_rejected_count_{0};
  std::uint64_t conversion_rejected_count_{0};
  std::uint64_t reset_suppressed_count_{0};
  std::uint64_t startup_rebaseline_count_{0};
  std::uint64_t startup_rebaseline_generation_{0};
  std::string last_reset_observation_status_{"accepted"};
  bool output_valid_{false};
  bool continuity_valid_{true};
  bool startup_origin_rebased_{false};
  std::int64_t last_valid_sample_time_ns_{0};
  std::uint64_t last_time_generation_{0};
  std::uint64_t frame_generation_{1};
  std::uint64_t reset_event_generation_{0};
  std::optional<std::uint8_t> last_input_reset_counter_;
  WorldConvention last_world_convention_{WorldConvention::kUnknown};
};

}  // namespace px4_odometry_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_odometry_bridge::Px4OdometryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
