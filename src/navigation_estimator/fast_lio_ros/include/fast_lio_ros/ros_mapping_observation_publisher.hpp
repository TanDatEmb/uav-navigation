#pragma once

#include <cstddef>
#include <memory>

#include <navigation_interfaces/msg/lidar_mapping_observation.hpp>
#include <rclcpp/node.hpp>

#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_ros/lio_public_frame_generation.hpp"
#include "fast_lio_ros/parameter_loader.hpp"

namespace uav::nav::lio {

// Publishes the atomic LidarMappingObservation contract consumed by
// navigation_mapping. This is the single P1 extraction point where FAST-LIO
// exposes a mapping-grade observation; see
// docs/architecture/navigation_layers.md for the full contract. FAST-LIO has
// no dependency on rog_map_vendor or navigation_mapping; this class only
// depends on the contract-only navigation_interfaces package.
class RosMappingObservationPublisher {
 public:
  RosMappingObservationPublisher(
      rclcpp::Node& node, RosParameters parameters,
      std::shared_ptr<LioPublicFrameGeneration> public_frame_generation);

  // No-op unless output.mapping_observation.enabled is set. Publishes only
  // when result.hasMappingObservationOutput() and the public frame generation
  // is valid; see the P1 mapping publication gate contract.
  void publish(const ProcessResult& result);

  [[nodiscard]] std::size_t publishedCount() const noexcept { return published_count_; }
  [[nodiscard]] std::size_t skippedNotReadyCount() const noexcept {
    return skipped_not_ready_count_;
  }
  [[nodiscard]] std::size_t skippedPublicFrameInvalidCount() const noexcept {
    return skipped_public_frame_invalid_count_;
  }

 private:
  RosParameters parameters_;
  std::shared_ptr<LioPublicFrameGeneration> public_frame_generation_;
  rclcpp::Publisher<navigation_interfaces::msg::LidarMappingObservation>::SharedPtr publisher_;
  std::size_t published_count_{0};
  std::size_t skipped_not_ready_count_{0};
  std::size_t skipped_public_frame_invalid_count_{0};
};

}  // namespace uav::nav::lio
