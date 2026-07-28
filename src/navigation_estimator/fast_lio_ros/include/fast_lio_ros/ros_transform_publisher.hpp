#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/node.hpp>

#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_ros/parameter_loader.hpp"

namespace uav::nav::lio {

class RosTransformPublisher {
 public:
  RosTransformPublisher(rclcpp::Node& node, RosParameters parameters);
  void publish(const ProcessResult& result);

 private:
  RosParameters parameters_;
  tf2_ros::TransformBroadcaster broadcaster_;
};

}  // namespace uav::nav::lio
