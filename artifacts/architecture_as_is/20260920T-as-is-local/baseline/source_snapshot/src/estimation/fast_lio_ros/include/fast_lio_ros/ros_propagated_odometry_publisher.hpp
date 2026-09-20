#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <nav_msgs/msg/odometry.hpp>
#include <navigation_contracts/msg/propagated_odometry.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_core/navigation/base_link_covariance_projector.hpp"
#include "fast_lio_core/navigation/base_link_state_converter.hpp"
#include "fast_lio_ros/parameter_loader.hpp"
#include "fast_lio_ros/lio_public_frame_generation.hpp"
#include "fast_lio_ros/runtime_diagnostics.hpp"

namespace uav::nav::lio {

class RosPropagatedOdometryPublisher {
 public:
  RosPropagatedOdometryPublisher(rclcpp::Node& node,
                                 const RosParameters& parameters,
                                 std::shared_ptr<CovarianceProjectionRuntime>
                                     covariance_runtime,
                                 std::shared_ptr<LioPublicFrameGeneration>
                                     public_frame_generation);
  void setBaseLinkConverter(std::shared_ptr<const BaseLinkStateConverter> converter);

  void publish(const KinematicStateEstimate& estimate);

 private:
  RosParameters parameters_;
  rclcpp::Publisher<navigation_contracts::msg::PropagatedOdometry>::SharedPtr
      publisher_;
  std::shared_ptr<const BaseLinkStateConverter> base_link_converter_;
  std::optional<BaseLinkCovarianceProjector> covariance_projector_;
  std::shared_ptr<CovarianceProjectionRuntime> covariance_runtime_;
  std::shared_ptr<LioPublicFrameGeneration> public_frame_generation_;
  std::uint64_t publication_sequence_{0U};
};

}  // namespace uav::nav::lio
