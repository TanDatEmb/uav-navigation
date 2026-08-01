#pragma once

#include <cstdint>
#include <optional>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_ros/parameter_loader.hpp"

namespace uav::nav::lio {

class RosPropagatedOdometryPublisher {
 public:
  RosPropagatedOdometryPublisher(rclcpp::Node& node,
                                 const RosParameters& parameters);

  void publish(const StateEstimate& estimate);

 private:
  RosParameters parameters_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

}  // namespace uav::nav::lio
