#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <navigation_contracts/msg/estimator_health.hpp>
#include <navigation_contracts/msg/registered_scan.hpp>
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
  void setBaseLinkConverter(std::shared_ptr<const BaseLinkStateConverter> converter);
  void setVisibilityCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
  [[nodiscard]] std::shared_ptr<CovarianceProjectionRuntime>
  covarianceProjectionRuntime() const noexcept {
    return covariance_runtime_;
  }
  void publish(const ProcessResult& result, std::uint64_t scan_sequence);
  // Publishes the latest health snapshot at the runtime diagnostic cadence.
  // The estimator path only replaces this latest-only snapshot.
  void publishDiagnosticsSnapshot();
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
  struct EstimatorHealthSnapshot {
    EstimatorStatus status{EstimatorStatus::kWaitingForSensors};
    LidarUpdateFailureClass failure_class{LidarUpdateFailureClass::kNone};
    bool corrected_output{false};
    bool navigation_valid{false};
    bool corrected_estimate_valid{false};
    std::string failure_reason;
    std::int64_t output_time_ns{0};
    std::int64_t last_lidar_correction_time_ns{0};
    std::uint64_t lio_generation{0};
    std::size_t imu_received_count{0};
    std::size_t lidar_received_count{0};
    std::size_t lidar_processed_count{0};
    std::size_t imu_drop_count{0};
    std::size_t lidar_drop_count{0};
    std::size_t timestamp_regression_count{0};
    std::size_t queue_maximum{0};
    std::size_t correction_accepted_count{0};
    std::size_t correction_rejected_count{0};
    std::size_t recovery_covariance_clamp_count{0};
    double recovery_covariance_maximum_eigenvalue_before_clamp{0.0};
    double recovery_covariance_maximum_eigenvalue_after_clamp{0.0};
    std::size_t map_point_count{0};
    std::size_t valid_point_count_busy_count{0};
    std::size_t measurement_callback_count{0};
    std::size_t observability_rejection_count{0};
    double translation_observability_min_eigenvalue{0.0};
    double translation_observability_max_eigenvalue{0.0};
    double translation_observability_ratio{0.0};
    bool translation_observability_valid{false};
    std::int64_t measurement_model_us{0};
    std::int64_t ikfom_solver_only_us{0};
    std::size_t map_size_after_insert{0};
    std::size_t map_size_after_maintenance{0};
    bool crop_performed{false};
    bool absolute_guard_triggered{false};
    bool absolute_guard_recovery_failed{false};
    bool map_insertion_frozen{false};
    std::int64_t map_maintenance_us{0};
  };

  [[nodiscard]] sensor_msgs::msg::PointCloud2 makeCloud(
      const std::vector<Eigen::Vector3d>& points, const builtin_interfaces::msg::Time& stamp) const;
  [[nodiscard]] sensor_msgs::msg::PointCloud2 makeFreeSpaceCloud(
      const sensor_msgs::msg::PointCloud2& cloud,
      const nav_msgs::msg::Odometry& corrected_odometry,
      const builtin_interfaces::msg::Time& stamp) const;
  void publishTypedHealth(const EstimatorHealthSnapshot& health,
                          const ProcessResult& result,
                          const builtin_interfaces::msg::Time& stamp);
  void publishDiagnostics(const EstimatorHealthSnapshot& health,
                          const builtin_interfaces::msg::Time& stamp);

  RosParameters parameters_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_points_;
  rclcpp::Publisher<navigation_contracts::msg::RegisteredScan>::SharedPtr
      registered_scan_;
  rclcpp::Publisher<navigation_contracts::msg::EstimatorHealth>::SharedPtr
      typed_health_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  std::shared_ptr<const BaseLinkStateConverter> base_link_converter_;
  std::optional<BaseLinkCovarianceProjector> covariance_projector_;
  std::shared_ptr<CovarianceProjectionRuntime> covariance_runtime_;
  std::shared_ptr<LioPublicFrameGeneration> public_frame_generation_;
  std::atomic_bool propagation_valid_{false};
  std::atomic<std::int64_t> last_propagated_state_stamp_ns_{0};
  mutable std::mutex visibility_cloud_mutex_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> visibility_clouds_;
  mutable std::mutex diagnostics_mutex_;
  std::optional<EstimatorHealthSnapshot> latest_diagnostic_health_;
};

}  // namespace uav::nav::lio
