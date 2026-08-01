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

  void onImuEstimate(const std::optional<StateEstimate>& estimate);
  [[nodiscard]] std::uint64_t publicationCount() const noexcept;
  [[nodiscard]] std::uint64_t publicationSkipCount() const noexcept;
  [[nodiscard]] std::optional<Timestamp> lastPublishedTime() const noexcept;
  [[nodiscard]] std::optional<Timestamp> nextPublishDeadline() const noexcept;

 private:
  RosParameters parameters_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::int64_t publish_period_ns_{0};
  std::optional<Timestamp> last_published_time_;
  std::optional<Timestamp> next_publish_deadline_;
  std::uint64_t publication_count_{0U};
  std::uint64_t publication_skip_count_{0U};
};

}  // namespace uav::nav::lio
