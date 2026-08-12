#include "navigation_mapping/navigation_mapping_node.hpp"

#include <filesystem>

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

  const bool visualization_enabled = declare_parameter("mapping.visualization.enabled", false);
  if (visualization_enabled) {
    RCLCPP_WARN(get_logger(),
                "mapping.visualization.enabled is set but P1 does not implement "
                "visualization; ignoring");
  }
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
}

void NavigationMappingNode::onObservation(
    const navigation_interfaces::msg::LidarMappingObservation::ConstSharedPtr& message) {
  if (!cloudHasXyzFloatFields(message->points)) {
    ++invalid_cloud_count_;
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

  pipeline_->process(input);
  publishDiagnostics();
}

void NavigationMappingNode::publishDiagnostics() {
  const auto& diagnostics = pipeline_->diagnostics();
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "navigation_mapping/world_model";
  status.hardware_id = "rog_map";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
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
      keyValue("invalid_cloud_count", std::to_string(invalid_cloud_count_)),
      keyValue("input_point_count", std::to_string(diagnostics.input_point_count)),
      keyValue("filtered_point_count", std::to_string(diagnostics.filtered_point_count)),
      keyValue("mapping_point_count", std::to_string(diagnostics.mapping_point_count)),
      keyValue("last_input_stamp_ns", std::to_string(diagnostics.last_input_stamp_ns)),
      keyValue("last_successful_update_stamp_ns",
               std::to_string(diagnostics.last_successful_update_stamp_ns)),
      keyValue("map_update_us", std::to_string(diagnostics.map_update_us)),
  };
  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

}  // namespace navigation_mapping
