#include "navigation_mapping/navigation_mapping_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace navigation_mapping {
namespace {

diagnostic_msgs::msg::KeyValue keyValue(std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

builtin_interfaces::msg::Time rosTimeFromNanoseconds(std::int64_t nanoseconds) {
  builtin_interfaces::msg::Time stamp;
  const std::int64_t seconds = nanoseconds / 1'000'000'000;
  const std::int64_t remainder = nanoseconds % 1'000'000'000;
  stamp.sec = static_cast<std::int32_t>(seconds);
  stamp.nanosec = static_cast<std::uint32_t>(remainder);
  if (remainder < 0) {
    --stamp.sec;
    stamp.nanosec = static_cast<std::uint32_t>(remainder + 1'000'000'000);
  }
  return stamp;
}

bool cloudHasXyzFloatFields(const sensor_msgs::msg::PointCloud2& cloud) {
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto& field : cloud.fields) {
    if (field.name == "x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) has_x = true;
    if (field.name == "y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) has_y = true;
    if (field.name == "z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) has_z = true;
  }
  return has_x && has_y && has_z;
}

}  // namespace

NavigationMappingNode::NavigationMappingNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("navigation_mapping_node", options) {
  const bool mapping_enabled = declare_parameter("mapping.enabled", true);

  MappingPipelineConfig pipeline_config;
  pipeline_config.contract.odom_frame_id = declare_parameter("mapping.frames.odom", std::string("lio_odom"));
  pipeline_config.contract.lidar_frame_id =
      declare_parameter("mapping.frames.lidar", std::string("livox_frame"));

  pipeline_config.point_filter.voxel_size_m =
      declare_parameter("mapping.input.voxel_size_m", 0.20);
  pipeline_config.point_filter.minimum_range_m =
      declare_parameter("mapping.input.minimum_range_m", 0.0);
  pipeline_config.point_filter.maximum_range_m =
      declare_parameter("mapping.input.maximum_range_m", 0.0);

  pipeline_config.rog.resolution_m = declare_parameter("mapping.rog.resolution_m", 0.20);
  const auto local_map_size = declare_parameter<std::vector<double>>(
      "mapping.rog.local_map_size_m", std::vector<double>{30.0, 30.0, 12.0});
  if (local_map_size.size() == 3) {
    pipeline_config.rog.local_map_size_m = {local_map_size[0], local_map_size[1],
                                            local_map_size[2]};
  } else {
    RCLCPP_WARN(get_logger(),
                "mapping.rog.local_map_size_m must have exactly 3 elements; using default");
  }
  pipeline_config.rog.raycasting_enabled =
      declare_parameter("mapping.rog.raycasting_enabled", true);
  pipeline_config.rog.sliding_enabled = declare_parameter("mapping.rog.sliding_enabled", true);
  pipeline_config.rog.occupied_inflation_enabled =
      declare_parameter("mapping.rog.occupied_inflation_enabled", true);
  pipeline_config.rog.unknown_inflation_enabled =
      declare_parameter("mapping.rog.unknown_inflation_enabled", false);
  pipeline_config.rog.esdf_enabled = declare_parameter("mapping.rog.esdf_enabled", false);
  pipeline_config.rog.frontier_enabled = declare_parameter("mapping.rog.frontier_enabled", false);
  pipeline_config.rog.inflation_step =
      static_cast<int>(declare_parameter<std::int64_t>("mapping.rog.inflation_step", 1));
  pipeline_config.rog.point_filt_num =
      static_cast<int>(declare_parameter<std::int64_t>("mapping.rog.point_filt_num", 1));
  pipeline_config.rog.ray_range_min_m = declare_parameter("mapping.rog.ray_range_min_m", 0.3);
  pipeline_config.rog.ray_range_max_m = declare_parameter("mapping.rog.ray_range_max_m", 15.0);

  visualization_enabled_ = declare_parameter("mapping.visualization.enabled", false);
  publish_unknown_ = declare_parameter("mapping.visualization.publish_unknown", false);
  publish_frontier_ = declare_parameter("mapping.visualization.publish_frontier", false);
  const auto visualization_range = declare_parameter<std::vector<double>>(
      "mapping.visualization.range_m", std::vector<double>{15.0, 15.0, 6.0});
  if (visualization_range.size() == 3 &&
      std::all_of(visualization_range.begin(), visualization_range.end(),
                  [](double value) { return std::isfinite(value) && value > 0.0; })) {
    visualization_range_x_m_ = visualization_range[0];
    visualization_range_y_m_ = visualization_range[1];
    visualization_range_z_m_ = visualization_range[2];
  } else {
    RCLCPP_WARN(get_logger(),
                "mapping.visualization.range_m must contain three positive finite values; "
                "using [15, 15, 6] m");
  }
  const auto visualization_max_points = declare_parameter<std::int64_t>(
      "mapping.visualization.max_points", 150000);
  visualization_max_points_ = visualization_max_points > 0
                                  ? static_cast<std::size_t>(visualization_max_points)
                                  : 150000U;
  visualization_frame_id_ = declare_parameter(
      "mapping.visualization.frame_id", pipeline_config.contract.odom_frame_id);
  const double visualization_rate_hz = declare_parameter(
      "mapping.visualization.publish_rate_hz", 2.0);
  const std::string qos_reliability =
      declare_parameter("mapping.qos.reliability", std::string("best_effort"));

  const std::string generated_config_directory =
      declare_parameter("mapping.generated_config_directory",
                        std::string("/tmp/navigation_mapping"));
  std::filesystem::create_directories(generated_config_directory);

  pipeline_ = std::make_unique<MappingPipeline>(
      pipeline_config, [this]() { return get_clock()->now().seconds(); },
      generated_config_directory);

  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/navigation_mapping/diagnostics", rclcpp::QoS{rclcpp::KeepLast{10}}.reliable());

  if (!mapping_enabled) {
    RCLCPP_INFO(get_logger(),
                "mapping.enabled is false; navigation_mapping_node running idle "
                "(no subscription, no map mutation)");
    return;
  }

  // P1 section 17: all map-touching callbacks are serialized in one
  // MutuallyExclusiveCallbackGroup. There is exactly one such callback today
  // (onObservation), but this makes the serialization requirement explicit
  // and future-proof rather than implicit in "there happens to be one
  // subscription".
  mapping_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = mapping_callback_group_;

  auto qos = rclcpp::QoS{rclcpp::KeepLast{1}}.durability_volatile();
  if (qos_reliability == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  observation_subscription_ =
      create_subscription<navigation_interfaces::msg::LidarMappingObservation>(
          "/lio/mapping_observation", qos,
          [this](const navigation_interfaces::msg::LidarMappingObservation::ConstSharedPtr&
                     message) { onObservation(message); },
          subscription_options);

  if (visualization_enabled_) {
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    occupied_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/rog_map/occ", qos);
    inflated_occupied_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/rog_map/inf_occ", qos);
    if (publish_unknown_) {
      unknown_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/rog_map/unk", qos);
    }
    if (publish_frontier_) {
      frontier_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "/rog_map/frontier", qos);
    }
    const double period_s = std::isfinite(visualization_rate_hz) && visualization_rate_hz > 0.0
                                ? 1.0 / visualization_rate_hz
                                : 0.5;
    visualization_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(period_s)),
        [this]() { publishMapVisualization(); }, mapping_callback_group_);
    RCLCPP_INFO(get_logger(),
                "ROG-Map visualization enabled: occ=/rog_map/occ inf_occ=/rog_map/inf_occ "
                "unk=%s frontier=%s rate=%.2f Hz max_points=%zu",
                publish_unknown_ ? "on" : "off", publish_frontier_ ? "on" : "off",
                1.0 / period_s, visualization_max_points_);
  }
}

void NavigationMappingNode::onObservation(
    const navigation_interfaces::msg::LidarMappingObservation::ConstSharedPtr& message) {
  const auto callback_started = std::chrono::steady_clock::now();
  const auto callback_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  auto& diagnostics = pipeline_->diagnostics();
  if (diagnostics.first_callback_wall_ns == 0) {
    diagnostics.first_callback_wall_ns = callback_wall_ns;
  }
  diagnostics.last_callback_wall_ns = callback_wall_ns;
  ++diagnostics.mapping_observation_receive_count;
  if (!cloudHasXyzFloatFields(message->points)) {
    ++invalid_cloud_count_;
    ++diagnostics.mapping_observation_rejection_count;
    publishDiagnostics();
    return;
  }

  ObservationInput input;
  input.header_frame_id = message->header.frame_id;
  input.header_stamp = message->header.stamp;
  input.points_frame_id = message->points.header.frame_id;
  input.points_stamp = message->points.header.stamp;
  input.sensor_pose = message->sensor_pose;
  input.public_frame_generation = message->public_frame_generation;

  const auto decode_started = std::chrono::steady_clock::now();
  const std::size_t point_count =
      static_cast<std::size_t>(message->points.width) * message->points.height;
  input.points_lidar_m.reserve(point_count);
  sensor_msgs::PointCloud2ConstIterator<float> x(message->points, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y(message->points, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z(message->points, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    input.points_lidar_m.emplace_back(static_cast<double>(*x), static_cast<double>(*y),
                                      static_cast<double>(*z));
  }
  diagnostics.ros_pointcloud_decode_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - decode_started).count();

  try {
    pipeline_->process(input);
  } catch (const std::exception& error) {
    ++diagnostics.processing_exception_count;
    ++diagnostics.mapping_observation_rejection_count;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "ROG-Map update failed; keeping node alive: %s", error.what());
  }
  diagnostics.mapping_callback_total_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - callback_started).count();
  publishDiagnostics();
}

sensor_msgs::msg::PointCloud2 NavigationMappingNode::makePointCloud(
    const rog_map::vec_E<rog_map::Vec3f>& points,
    const builtin_interfaces::msg::Time& stamp) const {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = visualization_frame_id_;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  const std::size_t stride = points.size() > visualization_max_points_
                                 ? (points.size() + visualization_max_points_ - 1U) /
                                       visualization_max_points_
                                 : 1U;
  const std::size_t output_count =
      points.empty() ? 0U : (points.size() + stride - 1U) / stride;
  cloud.width = static_cast<std::uint32_t>(std::min<std::size_t>(
      output_count, std::numeric_limits<std::uint32_t>::max()));
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(cloud.width);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (std::size_t index = 0; index < points.size() && x != x.end(); index += stride) {
    *x = static_cast<float>(points[index].x());
    *y = static_cast<float>(points[index].y());
    *z = static_cast<float>(points[index].z());
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

void NavigationMappingNode::publishMapVisualization() {
  if (!visualization_enabled_ || !pipeline_->adapter().isInitialized()) {
    return;
  }

  const bool have_occupied_subscriber = occupied_publisher_ &&
      occupied_publisher_->get_subscription_count() > 0;
  const bool have_inflated_subscriber = inflated_occupied_publisher_ &&
      inflated_occupied_publisher_->get_subscription_count() > 0;
  const bool have_unknown_subscriber = unknown_publisher_ &&
      unknown_publisher_->get_subscription_count() > 0;
  const bool have_frontier_subscriber = frontier_publisher_ &&
      frontier_publisher_->get_subscription_count() > 0;
  pipeline_->diagnostics().visualization_subscriber_count =
      static_cast<std::uint64_t>(occupied_publisher_->get_subscription_count() +
                                 inflated_occupied_publisher_->get_subscription_count() +
                                 (unknown_publisher_ ? unknown_publisher_->get_subscription_count() : 0) +
                                 (frontier_publisher_ ? frontier_publisher_->get_subscription_count() : 0));
  if (!have_occupied_subscriber && !have_inflated_subscriber &&
      !have_unknown_subscriber && !have_frontier_subscriber) {
    return;
  }

  try {
    const auto visualization_started = std::chrono::steady_clock::now();
    auto& map = pipeline_->adapter().map();
    const auto center = map.getLocalMapOrigin();
    const rog_map::Vec3f half_range(
        visualization_range_x_m_ * 0.5, visualization_range_y_m_ * 0.5,
        visualization_range_z_m_ * 0.5);
    const rog_map::Vec3f box_min = center - half_range;
    const rog_map::Vec3f box_max = center + half_range;
    rog_map::vec_E<rog_map::Vec3f> occupied;
    rog_map::vec_E<rog_map::Vec3f> inflated_occupied;
    rog_map::vec_E<rog_map::Vec3f> unknown;
    rog_map::vec_E<rog_map::Vec3f> frontier;
    const auto occ_started = std::chrono::steady_clock::now();
    if (have_occupied_subscriber) {
      map.boxSearch(box_min, box_max, super_utils::OCCUPIED, occupied);
    }
    pipeline_->diagnostics().visualization_occ_query_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - occ_started).count();
    const auto inf_started = std::chrono::steady_clock::now();
    if (have_inflated_subscriber) {
      map.boxSearchInflate(box_min, box_max, super_utils::OCCUPIED, inflated_occupied);
    }
    pipeline_->diagnostics().visualization_inf_occ_query_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - inf_started).count();
    const auto unknown_started = std::chrono::steady_clock::now();
    if (have_unknown_subscriber && publish_unknown_) {
      map.boxSearch(box_min, box_max, super_utils::UNKNOWN, unknown);
    }
    pipeline_->diagnostics().visualization_unknown_query_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - unknown_started).count();
    const auto frontier_started = std::chrono::steady_clock::now();
    if (have_frontier_subscriber && publish_frontier_) {
      map.boxSearch(box_min, box_max, super_utils::FRONTIER, frontier);
    }
    pipeline_->diagnostics().visualization_frontier_query_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - frontier_started).count();

    const auto& diagnostics = pipeline_->diagnostics();
    const auto stamp = diagnostics.last_successful_update_stamp_ns > 0
                           ? rosTimeFromNanoseconds(diagnostics.last_successful_update_stamp_ns)
                           : rosTimeFromNanoseconds(get_clock()->now().nanoseconds());
    const auto build_started = std::chrono::steady_clock::now();
    auto occupied_cloud = have_occupied_subscriber ? makePointCloud(occupied, stamp)
                                                   : sensor_msgs::msg::PointCloud2{};
    auto inflated_cloud = have_inflated_subscriber ? makePointCloud(inflated_occupied, stamp)
                                                   : sensor_msgs::msg::PointCloud2{};
    auto unknown_cloud = have_unknown_subscriber && publish_unknown_
                             ? makePointCloud(unknown, stamp)
                             : sensor_msgs::msg::PointCloud2{};
    auto frontier_cloud = have_frontier_subscriber && publish_frontier_
                              ? makePointCloud(frontier, stamp)
                              : sensor_msgs::msg::PointCloud2{};
    pipeline_->diagnostics().visualization_pointcloud_build_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - build_started).count();
    const auto publish_started = std::chrono::steady_clock::now();
    if (have_occupied_subscriber) occupied_publisher_->publish(occupied_cloud);
    if (have_inflated_subscriber) inflated_occupied_publisher_->publish(inflated_cloud);
    if (have_unknown_subscriber && publish_unknown_) {
      unknown_publisher_->publish(unknown_cloud);
    }
    if (have_frontier_subscriber && publish_frontier_) {
      frontier_publisher_->publish(frontier_cloud);
    }
    pipeline_->diagnostics().visualization_publish_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - publish_started).count();
    auto& mutable_diagnostics = pipeline_->diagnostics();
    ++mutable_diagnostics.visualization_publish_count;
    mutable_diagnostics.visualization_occupied_point_count = occupied.size();
    mutable_diagnostics.visualization_inflated_occupied_point_count = inflated_occupied.size();
    mutable_diagnostics.visualization_unknown_point_count = unknown.size();
    mutable_diagnostics.visualization_frontier_point_count = frontier.size();
    mutable_diagnostics.map_updates_since_last_visualization =
        diagnostics.accepted_observation_count - last_visualization_update_count_;
    last_visualization_update_count_ = diagnostics.accepted_observation_count;
    mutable_diagnostics.visualization_source_age_ms =
        std::max<std::int64_t>(0, get_clock()->now().nanoseconds() -
          diagnostics.last_successful_update_stamp_ns) / 1'000'000;
    mutable_diagnostics.visualization_total_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - visualization_started).count();
  } catch (const std::exception& error) {
    ++pipeline_->diagnostics().visualization_exception_count;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "ROG-Map visualization failed; keeping node alive: %s", error.what());
  }
}

void NavigationMappingNode::publishDiagnostics() {
  const auto& diagnostics = pipeline_->diagnostics();
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_mapping/world_model";
  status.hardware_id = "rog_map";
  status.level = diagnostics.processing_exception_count > 0 ||
                         diagnostics.visualization_exception_count > 0
                     ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                     : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = pipeline_->adapter().isInitialized() ? "map initialized" : "waiting for first generation";
  status.values = {
      keyValue("received_observation_count", std::to_string(diagnostics.received_observation_count)),
      keyValue("accepted_observation_count", std::to_string(diagnostics.accepted_observation_count)),
      keyValue("generation", std::to_string(diagnostics.generation)),
      keyValue("generation_reset_count", std::to_string(diagnostics.generation_reset_count)),
      keyValue("old_generation_drop_count", std::to_string(diagnostics.old_generation_drop_count)),
      keyValue("invalid_stamp_count", std::to_string(diagnostics.invalid_stamp_count)),
      keyValue("invalid_frame_count", std::to_string(diagnostics.invalid_frame_count)),
      keyValue("invalid_pose_count", std::to_string(diagnostics.invalid_pose_count)),
      keyValue("nonfinite_point_count", std::to_string(diagnostics.nonfinite_point_count)),
      keyValue("post_filter_nonfinite_point_count",
               std::to_string(diagnostics.post_filter_nonfinite_point_count)),
      keyValue("transform_nonfinite_point_count",
               std::to_string(diagnostics.transform_nonfinite_point_count)),
      keyValue("invalid_cloud_count", std::to_string(invalid_cloud_count_)),
      keyValue("input_point_count", std::to_string(diagnostics.input_point_count)),
      keyValue("filtered_point_count", std::to_string(diagnostics.filtered_point_count)),
      keyValue("mapping_point_count", std::to_string(diagnostics.mapping_point_count)),
      keyValue("last_input_stamp_ns", std::to_string(diagnostics.last_input_stamp_ns)),
      keyValue("first_input_stamp_ns", std::to_string(diagnostics.first_input_stamp_ns)),
      keyValue("first_callback_wall_ns", std::to_string(diagnostics.first_callback_wall_ns)),
      keyValue("last_callback_wall_ns", std::to_string(diagnostics.last_callback_wall_ns)),
      keyValue("last_successful_update_stamp_ns",
               std::to_string(diagnostics.last_successful_update_stamp_ns)),
      keyValue("map_update_us", std::to_string(diagnostics.map_update_us)),
      keyValue("mapping_callback_total_us", std::to_string(diagnostics.mapping_callback_total_us)),
      keyValue("ros_pointcloud_decode_us", std::to_string(diagnostics.ros_pointcloud_decode_us)),
      keyValue("mapping_filter_us", std::to_string(diagnostics.mapping_filter_us)),
      keyValue("transform_to_odom_us", std::to_string(diagnostics.transform_to_odom_us)),
      keyValue("rog_update_us", std::to_string(diagnostics.rog_update_us)),
      keyValue("rog_total_update_us", std::to_string(diagnostics.rog_total_update_us)),
      keyValue("rog_raycast_us", std::to_string(diagnostics.rog_raycast_us)),
      keyValue("rog_probability_update_us", std::to_string(diagnostics.rog_probability_update_us)),
      keyValue("rog_inflation_us", std::to_string(diagnostics.rog_inflation_us)),
      keyValue("rog_slide_us", std::to_string(diagnostics.rog_slide_us)),
      keyValue("mapping_filter_input_point_count", std::to_string(diagnostics.mapping_filter_input_point_count)),
      keyValue("mapping_filter_output_point_count", std::to_string(diagnostics.mapping_filter_output_point_count)),
      keyValue("rog_endpoint_count", std::to_string(diagnostics.rog_endpoint_count)),
      keyValue("rog_ray_attempt_count", std::to_string(diagnostics.rog_ray_attempt_count)),
      keyValue("rog_ray_processed_count", std::to_string(diagnostics.rog_ray_processed_count)),
      keyValue("rog_ray_clipped_count", std::to_string(diagnostics.rog_ray_clipped_count)),
      keyValue("rog_ray_skipped_count", std::to_string(diagnostics.rog_ray_skipped_count)),
      keyValue("rog_skip_nonfinite", std::to_string(diagnostics.rog_skip_nonfinite)),
      keyValue("rog_skip_intensity", std::to_string(diagnostics.rog_skip_intensity)),
      keyValue("rog_skip_point_filter", std::to_string(diagnostics.rog_skip_point_filter)),
      keyValue("rog_skip_below_raycast_min_range", std::to_string(diagnostics.rog_skip_below_raycast_min_range)),
      keyValue("rog_skip_endpoint_outside_local_map", std::to_string(diagnostics.rog_skip_endpoint_outside_local_map)),
      keyValue("rog_clipped_virtual_ground_or_ceiling", std::to_string(diagnostics.rog_clipped_virtual_ground_or_ceiling)),
      keyValue("rog_clipped_raycast_max_range", std::to_string(diagnostics.rog_clipped_raycast_max_range)),
      keyValue("rog_clipped_local_update_box", std::to_string(diagnostics.rog_clipped_local_update_box)),
      keyValue("rog_ray_outside_local_map_step", std::to_string(diagnostics.rog_ray_outside_local_map_step)),
      keyValue("rog_voxel_traversal_count", std::to_string(diagnostics.rog_voxel_traversal_count)),
      keyValue("rog_hit_candidate_count", std::to_string(diagnostics.rog_hit_candidate_count)),
      keyValue("rog_miss_candidate_count", std::to_string(diagnostics.rog_miss_candidate_count)),
      keyValue("rog_update_cache_entry_count", std::to_string(diagnostics.rog_update_cache_entry_count)),
      keyValue("map_slide_count", std::to_string(diagnostics.map_slide_count)),
      keyValue("map_slide_cells_cleared", std::to_string(diagnostics.map_slide_cells_cleared)),
      keyValue("rog_allocated_voxel_count", std::to_string(diagnostics.rog_allocated_voxel_count)),
      keyValue("sensor_origin_grid_type", diagnostics.sensor_origin_grid_type),
      keyValue("mapping_observation_receive_count", std::to_string(diagnostics.mapping_observation_receive_count)),
      keyValue("mapping_observation_rejection_count", std::to_string(diagnostics.mapping_observation_rejection_count)),
      keyValue("processing_exception_count",
               std::to_string(diagnostics.processing_exception_count)),
      keyValue("visualization_publish_count",
               std::to_string(diagnostics.visualization_publish_count)),
      keyValue("visualization_subscriber_count",
               std::to_string(diagnostics.visualization_subscriber_count)),
      keyValue("visualization_exception_count",
               std::to_string(diagnostics.visualization_exception_count)),
      keyValue("visualization_occupied_point_count",
               std::to_string(diagnostics.visualization_occupied_point_count)),
      keyValue("visualization_inflated_occupied_point_count",
               std::to_string(diagnostics.visualization_inflated_occupied_point_count)),
      keyValue("visualization_unknown_point_count",
               std::to_string(diagnostics.visualization_unknown_point_count)),
      keyValue("visualization_frontier_point_count",
               std::to_string(diagnostics.visualization_frontier_point_count)),
      keyValue("visualization_total_us", std::to_string(diagnostics.visualization_total_us)),
      keyValue("visualization_source_age_ms", std::to_string(diagnostics.visualization_source_age_ms)),
      keyValue("map_updates_since_last_visualization",
               std::to_string(diagnostics.map_updates_since_last_visualization)),
  };
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

}  // namespace navigation_mapping
