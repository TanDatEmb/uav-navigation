#include "fast_lio_ros/ros_mapping_observation_publisher.hpp"

#include <chrono>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "fast_lio_ros/qos_profiles.hpp"
#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {

RosMappingObservationPublisher::RosMappingObservationPublisher(
    rclcpp::Node& node, RosParameters parameters,
    std::shared_ptr<LioPublicFrameGeneration> public_frame_generation)
    : parameters_(std::move(parameters)),
      public_frame_generation_(std::move(public_frame_generation)) {
  observation_stream_id_ = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count()) ^
      static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(this));
  if (observation_stream_id_ == 0) {
    observation_stream_id_ = 1;
  }
  if (!parameters_.mapping_observation_enabled) {
    return;
  }
  publisher_ = node.create_publisher<navigation_interfaces::msg::LidarMappingObservation>(
      "/lio/mapping_observation", QosProfiles::mappingObservation());
}

void RosMappingObservationPublisher::publish(const ProcessResult& result) {
  if (!publisher_) {
    return;
  }
  // Mapping publication gate (P1 section 6): tracking + corrected estimate +
  // navigation-valid + a finite corrected sensor pose, exactly mirroring the
  // corrected-odometry usability gate in RosOutputPublisher. No propagated
  // state ever reaches this path.
  if (!result.hasMappingObservationOutput() || !result.scan_time.has_value()) {
    ++skipped_not_ready_count_;
    return;
  }
  const auto public_frame = public_frame_generation_
                                ? public_frame_generation_->snapshot()
                                : LioPublicFrameGenerationSnapshot{0U, false, 0U,
                                                                   "OWNER_UNAVAILABLE"};
  if (!public_frame.valid) {
    ++skipped_public_frame_invalid_count_;
    return;
  }

  navigation_interfaces::msg::LidarMappingObservation message;
  const auto stamp = RosTimeConverter::toRos(*result.scan_time);
  message.header.stamp = stamp;
  message.header.frame_id = parameters_.odom_frame;

  const RigidTransform& T_odom_lidar = *result.sensor_pose_odom_lidar;
  message.sensor_pose.position.x = T_odom_lidar.translation().x();
  message.sensor_pose.position.y = T_odom_lidar.translation().y();
  message.sensor_pose.position.z = T_odom_lidar.translation().z();
  message.sensor_pose.orientation.x = T_odom_lidar.rotation().x();
  message.sensor_pose.orientation.y = T_odom_lidar.rotation().y();
  message.sensor_pose.orientation.z = T_odom_lidar.rotation().z();
  message.sensor_pose.orientation.w = T_odom_lidar.rotation().w();

  auto& cloud = message.points;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = parameters_.lidar_frame;
  cloud.height = 1;
  cloud.width =
      static_cast<std::uint32_t>(result.mapping_candidate_points_lidar_m.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(result.mapping_candidate_points_lidar_m.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto& point : result.mapping_candidate_points_lidar_m) {
    *x = static_cast<float>(point.x());
    *y = static_cast<float>(point.y());
    *z = static_cast<float>(point.z());
    ++x;
    ++y;
    ++z;
  }

  message.public_frame_generation = public_frame.generation;
  message.observation_stream_id = observation_stream_id_;
  const std::uint64_t sequence = last_published_sequence_.load(std::memory_order_relaxed) + 1U;
  message.observation_sequence = sequence;
  publisher_->publish(message);
  last_published_sequence_.store(sequence, std::memory_order_relaxed);
  ++published_count_;
}

}  // namespace uav::nav::lio
