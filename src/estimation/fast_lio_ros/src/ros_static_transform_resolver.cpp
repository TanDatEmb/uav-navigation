#include "fast_lio_ros/ros_static_transform_resolver.hpp"

#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.hpp>

namespace uav::nav::lio {

RosStaticTransformResolver::RosStaticTransformResolver(rclcpp::Node& node)
    : buffer_(node.get_clock()), listener_(buffer_, true) {}

Result<RigidTransform> RosStaticTransformResolver::resolve(
    const std::string& target_frame, const std::string& source_frame,
    const std::size_t attempts, const std::chrono::milliseconds retry_period) {
  for (std::size_t attempt = 0U; attempt < attempts; ++attempt) {
    if (buffer_.canTransform(target_frame, source_frame, tf2::TimePointZero,
                             tf2::durationFromSec(0.05))) {
      try {
        const auto transform =
            buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        const Eigen::Quaterniond rotation(
            transform.transform.rotation.w, transform.transform.rotation.x,
            transform.transform.rotation.y, transform.transform.rotation.z);
        const Eigen::Vector3d translation(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);
        return RigidTransform::Create(FrameId(target_frame), FrameId(source_frame),
                                      rotation, translation);
      } catch (const tf2::TransformException&) {
        // The static publisher may be between discovery and cache insertion.
      }
    }
    if (attempt + 1U < attempts) {
      std::this_thread::sleep_for(retry_period);
    }
  }
  return Status(StatusCode::kNotReady,
                "static TF was not available for " + target_frame + " -> " +
                    source_frame);
}

}  // namespace uav::nav::lio
