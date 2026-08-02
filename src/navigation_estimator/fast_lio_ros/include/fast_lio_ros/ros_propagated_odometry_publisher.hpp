#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_core/navigation/base_link_covariance_projector.hpp"
#include "fast_lio_core/navigation/base_link_state_converter.hpp"
#include "fast_lio_ros/parameter_loader.hpp"
#include "fast_lio_ros/runtime_diagnostics.hpp"

namespace uav::nav::lio {

class RosPropagatedOdometryPublisher {
 public:
  RosPropagatedOdometryPublisher(rclcpp::Node& node,
                                 const RosParameters& parameters,
                                 std::shared_ptr<CovarianceProjectionRuntime>
                                     covariance_runtime);
  void setBaseLinkConverter(std::shared_ptr<const BaseLinkStateConverter> converter);

  void publish(const KinematicStateEstimate& estimate);

 private:
  RosParameters parameters_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::shared_ptr<const BaseLinkStateConverter> base_link_converter_;
  std::optional<BaseLinkCovarianceProjector> covariance_projector_;
  std::shared_ptr<CovarianceProjectionRuntime> covariance_runtime_;
};

}  // namespace uav::nav::lio
