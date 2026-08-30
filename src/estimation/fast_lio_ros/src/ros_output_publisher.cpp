#include "fast_lio_ros/ros_output_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <chrono>
#include <Eigen/Geometry>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <utility>

#include "fast_lio_ros/qos_profiles.hpp"
#include "fast_lio_ros/ros_odometry_serializer.hpp"
#include "fast_lio_ros/ros_time_converter.hpp"

namespace uav::nav::lio {
namespace {

constexpr std::size_t kMaximumVisibilityCloudPoints = 262144U;
constexpr std::size_t kVisibilityAssociationCapacity = 32U;
constexpr std::int64_t kVisibilityAssociationMaximumAgeNs = 500'000'000LL;
constexpr auto kVisibilityAssociationWait = std::chrono::milliseconds(10);

std::optional<std::int64_t> stampNanoseconds(
    const builtin_interfaces::msg::Time& stamp) noexcept {
  if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) return std::nullopt;
  constexpr std::int64_t kBillion = 1'000'000'000LL;
  if (static_cast<std::int64_t>(stamp.sec) >
      (std::numeric_limits<std::int64_t>::max() - stamp.nanosec) / kBillion) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(stamp.sec) * kBillion + stamp.nanosec;
}

bool boundedVisibilityCloud(const sensor_msgs::msg::PointCloud2& cloud) {
  if (cloud.is_bigendian || cloud.height == 0U ||
      cloud.point_step < sizeof(float) * 3U) {
    return false;
  }
  const auto point_count = static_cast<std::uint64_t>(cloud.width) *
                           static_cast<std::uint64_t>(cloud.height);
  const auto row_bytes = static_cast<std::uint64_t>(cloud.point_step) *
                         static_cast<std::uint64_t>(cloud.width);
  const auto storage_bytes = static_cast<std::uint64_t>(cloud.row_step) *
                             static_cast<std::uint64_t>(cloud.height);
  if (point_count > kMaximumVisibilityCloudPoints ||
      static_cast<std::uint64_t>(cloud.row_step) < row_bytes ||
      storage_bytes > cloud.data.size()) {
    return false;
  }
  bool x_found = false;
  bool y_found = false;
  bool z_found = false;
  for (const auto& field : cloud.fields) {
    if (field.name == "x") {
      x_found = field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                field.count == 1U &&
                static_cast<std::size_t>(field.offset) + sizeof(float) <=
                    cloud.point_step;
    } else if (field.name == "y") {
      y_found = field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                field.count == 1U &&
                static_cast<std::size_t>(field.offset) + sizeof(float) <=
                    cloud.point_step;
    } else if (field.name == "z") {
      z_found = field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                field.count == 1U &&
                static_cast<std::size_t>(field.offset) + sizeof(float) <=
                    cloud.point_step;
    }
  }
  return x_found && y_found && z_found;
}

std::uint32_t visibilitySourceRayCount(
    const sensor_msgs::msg::PointCloud2& cloud) noexcept {
  const auto metadata = std::find_if(
      cloud.fields.begin(), cloud.fields.end(), [](const auto& field) {
        return field.name == "visibility_source_ray_count";
      });
  if (metadata != cloud.fields.end() && metadata->count >= cloud.width * cloud.height) {
    return metadata->count;
  }
  return cloud.width * cloud.height;
}

diagnostic_msgs::msg::KeyValue keyValue(std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

}  // namespace

RosOutputPublisher::RosOutputPublisher(
    rclcpp::Node& node, RosParameters parameters,
    std::shared_ptr<LioPublicFrameGeneration> public_frame_generation)
    : parameters_(std::move(parameters)),
      clock_(node.get_clock()),
      covariance_runtime_(std::make_shared<CovarianceProjectionRuntime>()),
      public_frame_generation_(std::move(public_frame_generation)) {
  odometry_ = node.create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odometry_corrected", QosProfiles::estimatorOutput());
  if (parameters_.publish_registered_points) {
    registered_points_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/registered_points", QosProfiles::estimatorOutput());
  }
  registered_scan_ = node.create_publisher<navigation_contracts::msg::RegisteredScan>(
      "/lio/mapping_observation", QosProfiles::estimatorOutput());
  typed_health_ = node.create_publisher<navigation_contracts::msg::EstimatorHealth>(
      "/lio/health", QosProfiles::estimatorOutput());
  diagnostics_ = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/lio/diagnostics", QosProfiles::estimatorOutput());
}

void RosOutputPublisher::setBaseLinkConverter(
    std::shared_ptr<const BaseLinkStateConverter> converter) {
  std::lock_guard lock(converter_mutex_);
  if (converter) {
    covariance_projector_.emplace(converter->baseToImu());
  } else {
    covariance_projector_.reset();
  }
  base_link_converter_ = std::move(converter);
}

bool RosOutputPublisher::setVisibilityCloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
  if (!cloud || !boundedVisibilityCloud(*cloud) ||
      !stampNanoseconds(cloud->header.stamp)) return false;
  const auto incoming_stamp_ns = *stampNanoseconds(cloud->header.stamp);
  std::lock_guard lock(visibility_cloud_mutex_);
  if (std::any_of(visibility_clouds_.begin(), visibility_clouds_.end(),
                  [&](const auto& queued) {
                    return queued->header.stamp == cloud->header.stamp;
                  })) {
    return false;
  }
  const auto newest_stamp_ns = visibility_clouds_.empty()
      ? incoming_stamp_ns
      : stampNanoseconds(visibility_clouds_.back()->header.stamp).value_or(
            incoming_stamp_ns);
  if (incoming_stamp_ns < newest_stamp_ns -
          kVisibilityAssociationMaximumAgeNs) {
    return false;
  }
  const auto insertion = std::upper_bound(
      visibility_clouds_.begin(), visibility_clouds_.end(), incoming_stamp_ns,
      [](const std::int64_t stamp_ns, const auto& queued) {
        return stamp_ns < stampNanoseconds(queued->header.stamp).value_or(0);
      });
  visibility_clouds_.insert(insertion, std::move(cloud));
  const auto pruning_reference_ns = std::max(newest_stamp_ns, incoming_stamp_ns);
  while (!visibility_clouds_.empty()) {
    const auto oldest_stamp_ns = stampNanoseconds(
        visibility_clouds_.front()->header.stamp).value_or(0);
    if (visibility_clouds_.size() <= kVisibilityAssociationCapacity &&
        pruning_reference_ns - oldest_stamp_ns <=
            kVisibilityAssociationMaximumAgeNs) {
      break;
    }
    visibility_clouds_.pop_front();
  }
  visibility_cloud_cv_.notify_all();
  return true;
}

sensor_msgs::msg::PointCloud2 RosOutputPublisher::makeCloud(
    const std::vector<Eigen::Vector3d>& points, const builtin_interfaces::msg::Time& stamp) const {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = parameters_.odom_frame;
  cloud.height = 1;
  if (points.size() > std::numeric_limits<std::uint32_t>::max()) {
    return cloud;
  }
  const auto representable = [](const Eigen::Vector3d& point) {
    return point.allFinite() &&
           std::isfinite(static_cast<float>(point.x())) &&
           std::isfinite(static_cast<float>(point.y())) &&
           std::isfinite(static_cast<float>(point.z()));
  };
  std::size_t valid_count = 0U;
  for (const auto& point : points) {
    if (representable(point)) ++valid_count;
  }
  cloud.width = static_cast<std::uint32_t>(valid_count);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(valid_count);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto& point : points) {
    if (!representable(point)) continue;
    *x = static_cast<float>(point.x());
    *y = static_cast<float>(point.y());
    *z = static_cast<float>(point.z());
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 RosOutputPublisher::makeFreeSpaceCloud(
    const sensor_msgs::msg::PointCloud2& cloud,
    const nav_msgs::msg::Odometry& corrected_odometry,
    const builtin_interfaces::msg::Time& stamp) const {
  std::shared_ptr<const BaseLinkStateConverter> base_link_converter;
  {
    std::lock_guard lock(converter_mutex_);
    base_link_converter = base_link_converter_;
  }
  std::vector<Eigen::Vector3d> endpoints;
  if (cloud.header.frame_id != parameters_.lidar_frame ||
      cloud.header.stamp != stamp || cloud.height == 0U ||
      cloud.width == 0U || cloud.point_step < sizeof(float) * 3U ||
      !boundedVisibilityCloud(cloud) ||
      static_cast<std::size_t>(cloud.row_step) <
          static_cast<std::size_t>(cloud.point_step) * cloud.width ||
      cloud.data.size() <
          static_cast<std::size_t>(cloud.row_step) * cloud.height ||
      cloud.width * static_cast<std::size_t>(cloud.height) > 262144U ||
      !base_link_converter) {
    return makeCloud(endpoints, stamp);
  }

  std::uint32_t x_offset = 0U;
  std::uint32_t y_offset = 0U;
  std::uint32_t z_offset = 0U;
  bool x_found = false;
  bool y_found = false;
  bool z_found = false;
  for (const auto& field : cloud.fields) {
    if (field.name == "x") {
      x_offset = field.offset;
      x_found = field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    } else if (field.name == "y") {
      y_offset = field.offset;
      y_found = field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    } else if (field.name == "z") {
      z_offset = field.offset;
      z_found = field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    }
  }
  if (!x_found || !y_found || !z_found) {
    return makeCloud(endpoints, stamp);
  }

  const Eigen::Quaterniond q_odom_base(
      corrected_odometry.pose.pose.orientation.w,
      corrected_odometry.pose.pose.orientation.x,
      corrected_odometry.pose.pose.orientation.y,
      corrected_odometry.pose.pose.orientation.z);
  const Eigen::Quaterniond q_imu_lidar(
      parameters_.rotation_imu_lidar_xyzw[3],
      parameters_.rotation_imu_lidar_xyzw[0],
      parameters_.rotation_imu_lidar_xyzw[1],
      parameters_.rotation_imu_lidar_xyzw[2]);
  const double q_scale = q_odom_base.coeffs().cwiseAbs().maxCoeff();
  const double lidar_q_scale = q_imu_lidar.coeffs().cwiseAbs().maxCoeff();
  if (!q_odom_base.coeffs().allFinite() || !std::isfinite(q_scale) ||
      q_scale <= 1.0e-9 || !q_imu_lidar.coeffs().allFinite() ||
      !std::isfinite(lidar_q_scale) || lidar_q_scale <= 1.0e-9) {
    return makeCloud(endpoints, stamp);
  }

  const Eigen::Quaterniond normalized_odom_base(
      q_odom_base.w() / q_scale, q_odom_base.x() / q_scale,
      q_odom_base.y() / q_scale, q_odom_base.z() / q_scale);
  const Eigen::Quaterniond normalized_imu_lidar(
      q_imu_lidar.w() / lidar_q_scale, q_imu_lidar.x() / lidar_q_scale,
      q_imu_lidar.y() / lidar_q_scale, q_imu_lidar.z() / lidar_q_scale);
  const Eigen::Matrix3d R_odom_base = normalized_odom_base.normalized().toRotationMatrix();
  const Eigen::Matrix3d R_base_imu =
      base_link_converter->baseToImu().rotation().toRotationMatrix();
  const Eigen::Matrix3d R_base_lidar =
      R_base_imu * normalized_imu_lidar.normalized().toRotationMatrix();
  const Eigen::Vector3d t_base_lidar =
      R_base_imu * Eigen::Vector3d(
          parameters_.translation_imu_lidar_m[0],
          parameters_.translation_imu_lidar_m[1],
          parameters_.translation_imu_lidar_m[2]) +
      base_link_converter->baseToImu().translation();
  const Eigen::Vector3d p_odom_base(
      corrected_odometry.pose.pose.position.x,
      corrected_odometry.pose.pose.position.y,
      corrected_odometry.pose.pose.position.z);
  if (!p_odom_base.allFinite() || !t_base_lidar.allFinite()) {
    return makeCloud(endpoints, stamp);
  }

  const Eigen::Vector3d p_odom_lidar =
      p_odom_base + R_odom_base * t_base_lidar;
  const Eigen::Matrix3d R_odom_lidar = R_odom_base * R_base_lidar;
  const std::size_t point_count =
      static_cast<std::size_t>(cloud.width) * cloud.height;
  endpoints.reserve(point_count);
  for (std::size_t row = 0U; row < cloud.height; ++row) {
    for (std::size_t column = 0U; column < cloud.width; ++column) {
      const std::size_t point_offset = row * cloud.row_step +
          column * cloud.point_step;
      float xyz[3]{};
      std::memcpy(&xyz[0], cloud.data.data() + point_offset + x_offset, sizeof(float));
      std::memcpy(&xyz[1], cloud.data.data() + point_offset + y_offset, sizeof(float));
      std::memcpy(&xyz[2], cloud.data.data() + point_offset + z_offset, sizeof(float));
      const Eigen::Vector3d p_lidar(xyz[0], xyz[1], xyz[2]);
      if (!p_lidar.allFinite()) continue;
      const Eigen::Vector3d p_odom = p_odom_lidar + R_odom_lidar * p_lidar;
      if (p_odom.allFinite()) endpoints.push_back(p_odom);
    }
  }
  return makeCloud(endpoints, stamp);
}

void RosOutputPublisher::publish(const ProcessResult& result,
                                 const std::uint64_t scan_sequence) {
  EstimatorHealthSnapshot health;
  health.status = result.status_after;
  health.failure_class = result.diagnostics.last_update_failure_class;
  health.corrected_output = result.hasCorrectedOutput();
  health.navigation_valid = result.diagnostics.navigation_valid;
  health.corrected_estimate_valid =
      result.diagnostics.output.corrected_estimate_valid;
  health.failure_reason = result.rejection_reason.empty()
                              ? result.diagnostics.reason
                              : result.rejection_reason;
  health.output_time_ns = result.diagnostics.output.output_time_ns;
  health.last_lidar_correction_time_ns =
      result.diagnostics.output.last_lidar_correction_time_ns;
  health.lio_generation = result.diagnostics.lio_generation;
  health.imu_received_count = result.diagnostics.sensor.ros_received_imu_count;
  health.lidar_received_count = result.diagnostics.sensor.ros_received_lidar_count;
  health.lidar_processed_count = result.diagnostics.sensor.core_accepted_lidar_count;
  health.imu_drop_count = result.diagnostics.sensor.imu_drop_count;
  health.lidar_drop_count = result.diagnostics.sensor.lidar_drop_count;
  health.timestamp_regression_count =
      result.diagnostics.sensor.timestamp_regression_count;
  health.queue_maximum =
      result.diagnostics.sensor.processing_queue_high_water_mark;
  health.correction_accepted_count =
      result.diagnostics.processing.correction_success_count;
  health.correction_rejected_count =
      result.diagnostics.processing.correction_failure_count;
  health.recovery_covariance_clamp_count =
      result.diagnostics.recovery_covariance_clamp_count;
  health.recovery_covariance_maximum_eigenvalue_before_clamp =
      result.diagnostics
          .recovery_covariance_maximum_eigenvalue_before_clamp;
  health.recovery_covariance_maximum_eigenvalue_after_clamp =
      result.diagnostics.recovery_covariance_maximum_eigenvalue_after_clamp;
  health.map_point_count = result.diagnostics.map.map_point_count;
  health.valid_point_count_busy_count =
      result.diagnostics.map.valid_point_count_busy_count;
  health.measurement_callback_count =
      result.diagnostics.registration.measurement_callback_count;
  health.observability_rejection_count =
      result.diagnostics.registration.observability_rejection_count;
  health.translation_observability_min_eigenvalue =
      result.diagnostics.registration.translation_observability_min_eigenvalue;
  health.translation_observability_max_eigenvalue =
      result.diagnostics.registration.translation_observability_max_eigenvalue;
  health.translation_observability_ratio =
      result.diagnostics.registration.translation_observability_ratio;
  health.translation_observability_valid =
      result.diagnostics.registration.translation_observability_valid;
  health.measurement_model_us = result.diagnostics.timing.measurement_model_us;
  health.ikfom_solver_only_us = result.diagnostics.timing.ikfom_solver_only_us;
  health.map_size_after_insert = result.diagnostics.map.map_size_after_insert;
  health.map_size_after_maintenance =
      result.diagnostics.map.map_size_after_maintenance;
  health.crop_performed = result.diagnostics.map.crop_performed;
  health.absolute_guard_triggered =
      result.diagnostics.map.absolute_guard_triggered;
  health.absolute_guard_recovery_failed =
      result.diagnostics.map.absolute_guard_recovery_failed;
  health.map_insertion_frozen = result.diagnostics.map.map_insertion_frozen;
  health.map_maintenance_us = result.diagnostics.map.map_maintenance_us;
  const auto is_bridge_usable = [](const EstimatorHealthSnapshot& snapshot) {
    return snapshot.status == EstimatorStatus::kTracking &&
           snapshot.corrected_output && snapshot.navigation_valid &&
           snapshot.corrected_estimate_valid;
  };
  std::optional<EstimatorHealthSnapshot> transition_health;
  {
    std::scoped_lock lock(diagnostics_mutex_);
    if (!latest_diagnostic_health_.has_value() ||
        is_bridge_usable(*latest_diagnostic_health_) !=
            is_bridge_usable(health)) {
      // The periodic 2 Hz snapshot keeps serialization off the correction hot
      // path. Validity edges are different: the PX4 bridge consumes this
      // status as a safety gate, so delaying a recovery edge until the next
      // timer tick would turn one rejected 10 Hz scan into a 500 ms EV outage.
      transition_health = health;
    }
    latest_diagnostic_health_ = std::move(health);
  }

  const auto diagnostics_stamp =
      result.scan_time.has_value() ? RosTimeConverter::toRos(*result.scan_time)
                                   : static_cast<builtin_interfaces::msg::Time>(clock_->now());
  const bool transitioned_to_usable =
      transition_health.has_value() && is_bridge_usable(*transition_health);
  if (transition_health.has_value() && !transitioned_to_usable) {
    publishDiagnostics(*transition_health, diagnostics_stamp);
  }
  std::shared_ptr<const BaseLinkStateConverter> base_link_converter;
  std::optional<BaseLinkCovarianceProjector> covariance_projector;
  {
    std::lock_guard lock(converter_mutex_);
    base_link_converter = base_link_converter_;
    covariance_projector = covariance_projector_;
  }
  if (!result.hasCorrectedOutput() ||
      !result.corrected_kinematic_estimate.has_value() ||
      !base_link_converter) {
    publishTypedHealth(health, result, diagnostics_stamp);
    return;
  }
  const auto converted = base_link_converter->convert(
      result.corrected_kinematic_estimate->estimate,
      result.corrected_kinematic_estimate->angular_velocity_imu_rad_s);
  if (!converted.ok()) {
    publishTypedHealth(health, result, diagnostics_stamp);
    return;
  }
  std::optional<builtin_interfaces::msg::Time> odometry_stamp;
  std::optional<nav_msgs::msg::Odometry> corrected_odometry;
  if (covariance_projector.has_value() && covariance_runtime_) {
    BaseLinkCovarianceProjectionDiagnostics projection_diagnostics;
    const auto projection_started = std::chrono::steady_clock::now();
    const auto covariance = covariance_projector->project(
        *result.corrected_kinematic_estimate, converted.value(),
        &projection_diagnostics);
    const auto projection_elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - projection_started)
            .count();
    covariance_runtime_->record(projection_diagnostics, projection_elapsed);
    if (covariance.ok()) {
      const auto odometry = RosOdometrySerializer::serialize(
          converted.value(), covariance.value(), parameters_);
      if (odometry.ok()) {
        odometry_stamp = odometry.value().header.stamp;
        corrected_odometry = odometry.value();
        odometry_->publish(*corrected_odometry);
      }
    }
  }
  const auto stamp = odometry_stamp.value_or(diagnostics_stamp);
  if (result.hasRegisteredScanOutput() && corrected_odometry.has_value()) {
    // Build the registered cloud once. The raw cloud topic is an optional
    // visualization output; the typed observation carries the same immutable
    // message into the mapping runtime.
    auto registered_cloud = makeCloud(result.registered_points_odom_m, stamp);
    if (parameters_.publish_registered_points) {
      registered_points_->publish(registered_cloud);
    }
    navigation_contracts::msg::RegisteredScan observation;
    observation.header = corrected_odometry->header;
    observation.localization_epoch = public_frame_generation_
                                         ? public_frame_generation_->snapshot().generation
                                         : 0U;
    observation.scan_sequence = scan_sequence;
    observation.body_frame_id = corrected_odometry->child_frame_id;
    observation.corrected_pose = corrected_odometry->pose;
    observation.points = std::move(registered_cloud);
    sensor_msgs::msg::PointCloud2::ConstSharedPtr matching_visibility_cloud;
    {
      std::unique_lock lock(visibility_cloud_mutex_);
      const auto find_match = [&] {
        return std::find_if(
            visibility_clouds_.begin(), visibility_clouds_.end(),
            [&](const auto& queued) { return queued->header.stamp == stamp; });
      };
      (void)visibility_cloud_cv_.wait_for(
          lock, kVisibilityAssociationWait,
          [&] { return find_match() != visibility_clouds_.end(); });
      const auto match = find_match();
      if (match != visibility_clouds_.end()) {
        matching_visibility_cloud = *match;
        visibility_clouds_.erase(match);
      }
    }
    if (matching_visibility_cloud) {
      observation.free_space_endpoints = makeFreeSpaceCloud(
          *matching_visibility_cloud, *corrected_odometry, stamp);
      observation.visibility_observation_present = true;
      observation.visibility_source_ray_count =
          visibilitySourceRayCount(*matching_visibility_cloud);
      observation.visibility_no_return_count =
          observation.free_space_endpoints.width *
          observation.free_space_endpoints.height;
      observation.visibility_stamp_skew_ns = 0;
    }
    registered_scan_->publish(std::move(observation));
  }
  publishTypedHealth(health, result, diagnostics_stamp);
  if (transitioned_to_usable) {
    // Publish the recovery edge only after covariance projection has updated
    // its availability snapshot and corrected odometry has been emitted.
    publishDiagnostics(*transition_health, diagnostics_stamp);
  }
}

void RosOutputPublisher::publishTypedHealth(
    const EstimatorHealthSnapshot& health, const ProcessResult& result,
    const builtin_interfaces::msg::Time& stamp) {
  navigation_contracts::msg::EstimatorHealth message;
  message.header.stamp = stamp;
  const auto public_frame = public_frame_generation_
                                ? public_frame_generation_->snapshot()
                                : LioPublicFrameGenerationSnapshot{0U, false, 0U,
                                                                   "OWNER_UNAVAILABLE"};
  message.localization_epoch = public_frame.generation;
  switch (health.status) {
    case EstimatorStatus::kWaitingForSensors:
      message.state = navigation_contracts::msg::EstimatorHealth::WAITING_FOR_SENSORS;
      break;
    case EstimatorStatus::kCollectingImu:
    case EstimatorStatus::kInitializingImu:
    case EstimatorStatus::kInitializingMap:
      message.state = navigation_contracts::msg::EstimatorHealth::INITIALIZING;
      break;
    case EstimatorStatus::kTracking:
      message.state = navigation_contracts::msg::EstimatorHealth::TRACKING;
      break;
    case EstimatorStatus::kDegraded:
      message.state = navigation_contracts::msg::EstimatorHealth::DEGRADED;
      break;
    case EstimatorStatus::kLost:
      message.state = navigation_contracts::msg::EstimatorHealth::LOST;
      break;
    case EstimatorStatus::kResetting:
      message.state = navigation_contracts::msg::EstimatorHealth::RESETTING;
      break;
  }
  const auto covariance = covariance_runtime_->snapshot();
  message.navigation_valid = health.navigation_valid;
  message.covariance_valid = covariance.pose_covariance_available &&
                             covariance.twist_covariance_available;
  message.observability_valid = health.translation_observability_valid;
  message.correction_fresh = result.hasCorrectedOutput();
  message.propagation_valid =
      propagation_valid_.load(std::memory_order_acquire);
  if (health.last_lidar_correction_time_ns > 0) {
    message.last_correction_stamp.sec =
        static_cast<std::int32_t>(health.last_lidar_correction_time_ns / 1'000'000'000LL);
    message.last_correction_stamp.nanosec = static_cast<std::uint32_t>(
        health.last_lidar_correction_time_ns % 1'000'000'000LL);
  }
  const auto last_propagated_state_stamp_ns =
      last_propagated_state_stamp_ns_.load(std::memory_order_acquire);
  if (last_propagated_state_stamp_ns > 0) {
    message.last_propagated_state_stamp.sec = static_cast<std::int32_t>(
        last_propagated_state_stamp_ns / 1'000'000'000LL);
    message.last_propagated_state_stamp.nanosec = static_cast<std::uint32_t>(
        last_propagated_state_stamp_ns % 1'000'000'000LL);
  }
  message.reason_code = static_cast<std::uint16_t>(health.failure_class);
  typed_health_->publish(std::move(message));
}

void RosOutputPublisher::publishDiagnosticsSnapshot() {
  std::optional<EstimatorHealthSnapshot> health;
  {
    std::scoped_lock lock(diagnostics_mutex_);
    health = latest_diagnostic_health_;
  }
  if (health.has_value()) {
    publishDiagnostics(*health,
                       static_cast<builtin_interfaces::msg::Time>(clock_->now()));
  }
}

void RosOutputPublisher::publishDiagnostics(const EstimatorHealthSnapshot& health,
                                            const builtin_interfaces::msg::Time& stamp) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/estimator";
  status.hardware_id = "lidar_imu";
  status.level =
      health.status == EstimatorStatus::kLost ||
              health.failure_class == LidarUpdateFailureClass::kStateCorruption
          ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
          : (health.status == EstimatorStatus::kTracking && health.corrected_output
                 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status.message = health.failure_reason;
  const auto public_frame = public_frame_generation_
                                ? public_frame_generation_->snapshot()
                                : LioPublicFrameGenerationSnapshot{0U, false, 0U,
                                                                   "OWNER_UNAVAILABLE"};
  // Keep the public diagnostics contract small.  Detailed counters remain
  // available in the report artifacts, while this topic is the health gate
  // consumed by the runtime monitor and PX4 bridge.
  const auto covariance = covariance_runtime_->snapshot();
  status.values = {
      keyValue("state", toString(health.status)),
      keyValue("status", toString(health.status)),
      keyValue("navigation_valid", health.navigation_valid ? "true" : "false"),
      keyValue("corrected_estimate_valid",
               health.corrected_estimate_valid ? "true" : "false"),
      keyValue("translation_observability_valid",
               health.translation_observability_valid ? "true" : "false"),
      keyValue("translation_observability_min_eigenvalue",
               std::to_string(health.translation_observability_min_eigenvalue)),
      keyValue("translation_observability_max_eigenvalue",
               std::to_string(health.translation_observability_max_eigenvalue)),
      keyValue("translation_observability_ratio",
               std::to_string(health.translation_observability_ratio)),
      keyValue("observability_rejection_count",
               std::to_string(health.observability_rejection_count)),
      keyValue("last_failure_code", toString(health.failure_class)),
      keyValue("last_failure_reason", status.message),
      keyValue("output_time_ns", std::to_string(health.output_time_ns)),
      keyValue("last_lidar_correction_time_ns",
               std::to_string(health.last_lidar_correction_time_ns)),
      keyValue("lio_generation", std::to_string(health.lio_generation)),
      keyValue("lio_public_frame_generation", std::to_string(public_frame.generation)),
      keyValue("lio_public_frame_generation_valid", public_frame.valid ? "true" : "false"),
      keyValue("imu_received_count", std::to_string(health.imu_received_count)),
      keyValue("lidar_received_count", std::to_string(health.lidar_received_count)),
      keyValue("lidar_processed_count", std::to_string(health.lidar_processed_count)),
      keyValue("imu_drop_count", std::to_string(health.imu_drop_count)),
      keyValue("lidar_drop_count", std::to_string(health.lidar_drop_count)),
      keyValue("timestamp_regression_count",
               std::to_string(health.timestamp_regression_count)),
      keyValue("queue_maximum", std::to_string(health.queue_maximum)),
      keyValue("correction_accepted_count",
               std::to_string(health.correction_accepted_count)),
      keyValue("correction_rejected_count",
               std::to_string(health.correction_rejected_count)),
      keyValue("recovery_covariance_clamp_count",
               std::to_string(health.recovery_covariance_clamp_count)),
      keyValue("recovery_covariance_maximum_eigenvalue_before_clamp",
               std::to_string(
                   health.recovery_covariance_maximum_eigenvalue_before_clamp)),
      keyValue("recovery_covariance_maximum_eigenvalue_after_clamp",
               std::to_string(
                   health.recovery_covariance_maximum_eigenvalue_after_clamp)),
      keyValue("map_point_count", std::to_string(health.map_point_count)),
      keyValue("pose_covariance_available",
               covariance.pose_covariance_available ? "true" : "false"),
      keyValue("twist_covariance_available",
               covariance.twist_covariance_available ? "true" : "false"),
      keyValue("measurement_callback_count",
               std::to_string(health.measurement_callback_count)),
      keyValue("measurement_model_us",
               std::to_string(health.measurement_model_us)),
      keyValue("ikfom_solver_only_us",
               std::to_string(health.ikfom_solver_only_us)),
      // Small, per-correction map-maintenance contract. These values make a
      // guard/crop event auditable without publishing the full internal
      // diagnostic payload at LiDAR rate.
      keyValue("map_size_after_insert",
               std::to_string(health.map_size_after_insert)),
      keyValue("map_size_after_maintenance",
               std::to_string(health.map_size_after_maintenance)),
      keyValue("valid_point_count_busy_count",
               std::to_string(health.valid_point_count_busy_count)),
      keyValue("crop_performed", health.crop_performed ? "true" : "false"),
      keyValue("absolute_guard_triggered",
               health.absolute_guard_triggered ? "true" : "false"),
      keyValue("absolute_guard_recovery_failed",
               health.absolute_guard_recovery_failed ? "true" : "false"),
      keyValue("map_insertion_frozen",
               health.map_insertion_frozen ? "true" : "false"),
      keyValue("map_maintenance_us",
               std::to_string(health.map_maintenance_us)),
  };
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

void RosOutputPublisher::publishTransportSnapshot(
    const SensorDiagnostics& sensor,
    const ProcessingStatistics& processing,
    const RuntimeDiagnostics& runtime) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = clock_->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/transport";
  status.hardware_id = "lidar_imu";
  status.level = runtime.overflow_detected || runtime.processing_lag_exceeded
                     ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                     : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = runtime.overflow_detected
                       ? "INPUT_QUEUE_OVERFLOW"
                       : (runtime.processing_lag_exceeded
                              ? "PROCESSING_LAG_LIMIT_EXCEEDED"
                              : "TRANSPORT_COUNTER_SNAPSHOT");
  status.values = {
      keyValue("transport_ok", status.level == diagnostic_msgs::msg::DiagnosticStatus::OK ? "true" : "false"),
      keyValue("imu_received_count", std::to_string(sensor.ros_received_imu_count)),
      keyValue("lidar_received_count", std::to_string(sensor.ros_received_lidar_count)),
      keyValue("imu_drop_count", std::to_string(runtime.imu_drop_count)),
      keyValue("lidar_drop_count", std::to_string(runtime.lidar_drop_count)),
      keyValue("timestamp_regression_count",
               std::to_string(runtime.reject_reason_timestamp_regression)),
      keyValue("queue_depth", std::to_string(runtime.current_input_queue_depth)),
      keyValue("queue_maximum", std::to_string(runtime.maximum_queue_depth)),
      keyValue("processing_lag_ns", std::to_string(runtime.processing_lag_ns)),
      keyValue("processing_worker_failed",
               runtime.processing_worker_failed ? "true" : "false"),
      keyValue("processing_worker_failure_message",
               runtime.processing_worker_failure_message),
      keyValue("scan_processing_p95_us", std::to_string(runtime.p95_scan_processing_us)),
      keyValue("correction_accepted_count",
               std::to_string(processing.correction_success_count)),
      keyValue("correction_rejected_count",
               std::to_string(processing.correction_failure_count)),
      keyValue("pose_covariance_available",
               runtime.covariance_projection.pose_covariance_available ? "true" : "false"),
      keyValue("twist_covariance_available",
               runtime.covariance_projection.twist_covariance_available ? "true" : "false"),
  };
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

void RosOutputPublisher::publishPropagatedOdometryDiagnostics(
    const PropagatedOdometryWorkerDiagnostics& propagated,
    std::uint64_t publication_count,
    std::uint64_t publication_skip_count,
    std::optional<Timestamp> last_published_time,
    std::optional<Timestamp> next_publish_deadline) {
  propagation_valid_.store(
      propagated.propagator.status == PropagatedOdometryStatus::kReady &&
          propagated.navigation_valid,
      std::memory_order_release);
  if (propagated.propagator.propagated_time.has_value()) {
    const auto propagated_stamp_ns =
        propagated.propagator.propagated_time->nanoseconds();
    auto previous_stamp_ns =
        last_propagated_state_stamp_ns_.load(std::memory_order_acquire);
    while (propagated_stamp_ns > previous_stamp_ns &&
           !last_propagated_state_stamp_ns_.compare_exchange_weak(
               previous_stamp_ns, propagated_stamp_ns,
               std::memory_order_release, std::memory_order_acquire)) {
    }
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = propagated.propagator.latest_imu_time.has_value()
                           ? RosTimeConverter::toRos(
                                 *propagated.propagator.latest_imu_time)
                           : static_cast<builtin_interfaces::msg::Time>(
                                 clock_->now());
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/propagated_odometry";
  status.hardware_id = "lidar_imu";
  status.level = propagated.worker_failed
                     ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                     : (propagated.propagator.status == PropagatedOdometryStatus::kReady
                            ? diagnostic_msgs::msg::DiagnosticStatus::OK
                            : diagnostic_msgs::msg::DiagnosticStatus::WARN);
  status.message = toString(propagated.propagator.status);
  const auto timeNs = [](const std::optional<Timestamp>& time) {
    return std::to_string(time.has_value() ? time->nanoseconds() : 0);
  };
  const auto& core = propagated.propagator;
  status.values = {
      keyValue("status", toString(core.status)),
      keyValue("navigation_valid", propagated.navigation_valid ? "true" : "false"),
      keyValue("latest_imu_time_ns", timeNs(core.latest_imu_time)),
      keyValue("propagated_time_ns", timeNs(core.propagated_time)),
      keyValue("last_correction_time_ns", timeNs(propagated.last_applied_correction_time)),
      keyValue("timestamp_regression_count", std::to_string(core.timestamp_regression_count)),
      keyValue("queue_overflow_count", std::to_string(propagated.queue_overflow_count)),
      keyValue("publication_count", std::to_string(publication_count)),
      keyValue("publication_skip_count", std::to_string(publication_skip_count)),
      keyValue("last_published_time_ns", timeNs(last_published_time)),
      keyValue("next_publish_deadline_ns", timeNs(next_publish_deadline)),
      // These 1 Hz counters explain a propagated-odometry pause without
      // inflating the high-rate estimator health status.
      keyValue("last_received_correction_time_ns", timeNs(propagated.last_correction_time)),
      keyValue("last_applied_correction_time_ns", timeNs(propagated.last_applied_correction_time)),
      keyValue("reanchor_count", std::to_string(core.reanchor_count)),
      keyValue("replay_count", std::to_string(core.replay_count)),
      keyValue("last_replay_sample_count", std::to_string(core.last_replay_sample_count)),
      keyValue("last_replay_runtime_us", std::to_string(propagated.last_replay_runtime_us)),
      keyValue("maximum_replay_runtime_us", std::to_string(propagated.maximum_replay_runtime_us)),
      keyValue("replay_in_progress", propagated.replay_in_progress ? "true" : "false"),
      keyValue("requires_reanchor", core.requires_reanchor ? "true" : "false"),
      keyValue("load_shedding_count", std::to_string(propagated.load_shedding_count)),
      keyValue("maximum_imu_batch_size", std::to_string(propagated.maximum_imu_batch_size)),
      keyValue("stale_stop_count", std::to_string(propagated.stale_stop_count)),
      keyValue("worker_failed", propagated.worker_failed ? "true" : "false"),
      keyValue("worker_failure_message", propagated.worker_failure_message),
  };
  array.status.push_back(std::move(status));
  diagnostics_->publish(array);
}

}  // namespace uav::nav::lio
