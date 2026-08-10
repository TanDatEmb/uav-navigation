#pragma once

#include <memory>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio_core/pipeline/process_result.hpp"
#include "fast_lio_core/navigation/base_link_state_converter.hpp"
#include "fast_lio_core/navigation/base_link_covariance_projector.hpp"
#include "fast_lio_ros/parameter_loader.hpp"
#include "fast_lio_ros/lio_public_frame_generation.hpp"
#include "fast_lio_ros/propagated_odometry_worker.hpp"
#include "fast_lio_ros/runtime_diagnostics.hpp"

namespace uav::nav::lio {

class RosOutputPublisher {
 public:
  RosOutputPublisher(rclcpp::Node& node, RosParameters parameters,
                     std::shared_ptr<LioPublicFrameGeneration>
                         public_frame_generation);
  ~RosOutputPublisher();
  void setBaseLinkConverter(std::shared_ptr<const BaseLinkStateConverter> converter);
  [[nodiscard]] std::shared_ptr<CovarianceProjectionRuntime>
  covarianceProjectionRuntime() const noexcept {
    return covariance_runtime_;
  }
  void publish(const ProcessResult& result);
  void publishTransportSnapshot(
      const SensorDiagnostics& sensor,
      const ProcessingStatistics& processing,
      const RuntimeDiagnostics& runtime);
  void publishPropagatedOdometryDiagnostics(
      const PropagatedOdometryWorkerDiagnostics& propagated,
      std::uint64_t publication_count,
      std::uint64_t publication_skip_count,
      std::optional<Timestamp> last_published_time,
      std::optional<Timestamp> next_publish_deadline);

 private:
  [[nodiscard]] sensor_msgs::msg::PointCloud2 makeCloud(
      const std::vector<Eigen::Vector3d>& points, const builtin_interfaces::msg::Time& stamp) const;
  [[nodiscard]] sensor_msgs::msg::PointCloud2 makeDeskewedCloud(
      const DeskewedObservation& observation) const;
  void deskewedPublisherLoop();
  void publishDiagnostics(const ProcessResult& result, const builtin_interfaces::msg::Time& stamp);

  RosParameters parameters_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  std::shared_ptr<const BaseLinkStateConverter> base_link_converter_;
  std::optional<BaseLinkCovarianceProjector> covariance_projector_;
  std::shared_ptr<CovarianceProjectionRuntime> covariance_runtime_;
  std::shared_ptr<LioPublicFrameGeneration> public_frame_generation_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_points_;
  std::mutex deskewed_mutex_;
  std::condition_variable deskewed_ready_;
  std::shared_ptr<const DeskewedObservation> pending_deskewed_observation_;
  bool deskewed_stopping_{false};
  std::thread deskewed_publisher_thread_;
};

}  // namespace uav::nav::lio
