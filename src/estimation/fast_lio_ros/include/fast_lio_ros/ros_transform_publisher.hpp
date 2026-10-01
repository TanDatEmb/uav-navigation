#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/node.hpp>

#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_core/navigation/base_link_state_converter.hpp"

namespace uav::nav::lio {

class RosTransformPublisher {
 public:
  struct Diagnostics {
    std::size_t publication_count{0U};
    std::size_t timestamp_suppressed_count{0U};
    std::size_t conversion_failure_count{0U};
  };

  explicit RosTransformPublisher(rclcpp::Node& node);
  void setBaseLinkConverter(std::shared_ptr<const BaseLinkStateConverter> converter);
  void publishPropagated(const KinematicStateEstimate& estimate);
  [[nodiscard]] Diagnostics diagnostics() const;

 private:
  void publishKinematic(const KinematicStateEstimate& estimate);
  tf2_ros::TransformBroadcaster broadcaster_;
  std::shared_ptr<const BaseLinkStateConverter> base_link_converter_;
  mutable std::mutex mutex_;
  std::optional<Timestamp> last_published_time_;
  Diagnostics diagnostics_;
};

}  // namespace uav::nav::lio
