#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <rclcpp/rclcpp.hpp>

#include "fast_lio_ros/parameter_loader.hpp"

namespace uav::nav::lio {

class ParameterLoaderTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

std::filesystem::path canonicalConfigWithout(std::string_view line,
                                             std::string_view suffix) {
  std::ifstream source(FAST_LIO_ROS_SOURCE_DIR "/config/mid360-real.yaml");
  std::string contents((std::istreambuf_iterator<char>(source)),
                       std::istreambuf_iterator<char>());
  const auto position = contents.find(line);
  if (position == std::string::npos) {
    throw std::runtime_error("test fixture line was not found");
  }
  contents.erase(position, line.size());
  const auto path = std::filesystem::temp_directory_path() /
                    ("fast_lio_missing_" + std::string(suffix) + ".yaml");
  std::ofstream(path) << contents;
  return path;
}

TEST_F(ParameterLoaderTest, LoadsAndValidatesDefaultProductionSchema) {
  rclcpp::Node node{"parameter_loader_test"};
  const auto parameters = ParameterLoader::declareAndLoad(node);
  EXPECT_EQ(parameters.odom_frame, "odom");
  EXPECT_EQ(parameters.lidar_timing_mode, "simultaneous_scan");
  EXPECT_EQ(parameters.input_clock_domain, "ros_time");
  EXPECT_EQ(parameters.livox_timestamp_policy, "require_header_match");
  EXPECT_FALSE(parameters.estimate_extrinsic_online);
  EXPECT_EQ(parameters.minimum_imu_samples, 200);
  EXPECT_EQ(parameters.imu_queue_capacity, 4096);
  EXPECT_EQ(parameters.lidar_queue_capacity, 8);
  EXPECT_EQ(parameters.overload_policy, "fail");
}

TEST_F(ParameterLoaderTest, RejectsAutoTimingForProductionNode) {
  rclcpp::Node node{"parameter_loader_auto_test"};
  auto parameters = ParameterLoader::declareAndLoad(node);
  parameters.lidar_timing_mode = "auto";
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
}

TEST_F(ParameterLoaderTest, RejectsInvalidRuntimeQueuePolicy) {
  rclcpp::Node node{"parameter_loader_runtime_test"};
  auto parameters = ParameterLoader::declareAndLoad(node);
  parameters.lidar_queue_capacity = 0;
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
  parameters.lidar_queue_capacity = 8;
  parameters.overload_policy = "drop";
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
}

TEST_F(ParameterLoaderTest, AcceptsPinnedLivoxCustomBoundary) {
  rclcpp::Node node{"parameter_loader_livox_test"};
  auto parameters = ParameterLoader::declareAndLoad(node);
  parameters.lidar_topic = "/livox/lidar";
  parameters.imu_topic = "/livox/imu";
  parameters.lidar_message_type = "livox_custom";
  parameters.lidar_timing_mode = "per_point";
  parameters.input_clock_domain = "sensor_time";
  parameters.livox_timestamp_policy = "require_header_match";
  EXPECT_NO_THROW(ParameterLoader::validate(parameters));
}

TEST_F(ParameterLoaderTest, RejectsLivoxCustomWithoutPerPointTiming) {
  rclcpp::Node node{"parameter_loader_bad_livox_test"};
  auto parameters = ParameterLoader::declareAndLoad(node);
  parameters.lidar_message_type = "livox_custom";
  parameters.lidar_timing_mode = "simultaneous_scan";
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
}

void expectEstimatorConfigsEqual(const EstimatorConfig& direct,
                                 const EstimatorConfig& ros) {
  EXPECT_EQ(direct.synchronization.maximum_imu_gap_ns,
            ros.synchronization.maximum_imu_gap_ns);
  EXPECT_EQ(direct.deskew.mode, ros.deskew.mode);
  EXPECT_DOUBLE_EQ(direct.preprocessing.point_filter.minimum_range_m,
                   ros.preprocessing.point_filter.minimum_range_m);
  EXPECT_DOUBLE_EQ(direct.preprocessing.point_filter.maximum_range_m,
                   ros.preprocessing.point_filter.maximum_range_m);
  EXPECT_DOUBLE_EQ(direct.preprocessing.voxel_filter.voxel_size_m,
                   ros.preprocessing.voxel_filter.voxel_size_m);
  EXPECT_EQ(direct.initialization.minimum_imu_samples,
            ros.initialization.minimum_imu_samples);
  EXPECT_EQ(direct.initialization.require_stationary,
            ros.initialization.require_stationary);
  EXPECT_EQ(direct.ikfom.maximum_iterations, ros.ikfom.maximum_iterations);
  EXPECT_EQ(direct.extrinsic.estimate_online, ros.extrinsic.estimate_online);
  EXPECT_LT((direct.extrinsic.translation_imu_lidar_m -
             ros.extrinsic.translation_imu_lidar_m)
                .norm(),
            1e-15);
  EXPECT_LT(direct.extrinsic.rotation_imu_lidar.angularDistance(
                ros.extrinsic.rotation_imu_lidar),
            1e-15);
  EXPECT_DOUBLE_EQ(
      direct.residual_builder.point_measurement_standard_deviation_m,
      ros.residual_builder.point_measurement_standard_deviation_m);
  EXPECT_DOUBLE_EQ(
      direct.residual_builder.residual_gate.maximum_absolute_distance_m,
      ros.residual_builder.residual_gate.maximum_absolute_distance_m);
  EXPECT_EQ(direct.residual_builder.correspondence_search.neighbor_count,
            ros.residual_builder.correspondence_search.neighbor_count);
  EXPECT_EQ(direct.registration_map.voxel_size_m,
            ros.registration_map.voxel_size_m);
  EXPECT_TRUE(direct.local_map.half_extent_m.isApprox(
      ros.local_map.half_extent_m, 0.0));
}

TEST_F(ParameterLoaderTest, CanonicalConfigParsesAndRecordsSha) {
  const std::string path =
      FAST_LIO_ROS_SOURCE_DIR
      "/config/mid360-real.yaml";
  const auto profile = loadCanonicalEstimatorProfile(path);
  EXPECT_EQ(profile.config_sha256, sha256File(path));
  EXPECT_EQ(profile.config_sha256.size(), 64U);
  EXPECT_EQ(profile.estimator.ikfom.maximum_iterations, 10U);
  EXPECT_DOUBLE_EQ(
      profile.estimator.preprocessing.voxel_filter.voxel_size_m, 0.2);
  EXPECT_DOUBLE_EQ(profile.estimator.registration_map.voxel_size_m, 0.2);
}

TEST_F(ParameterLoaderTest, AistUsesIndependentScanAndRegistrationMapVoxels) {
  const auto profile = loadCanonicalEstimatorProfile(
      FAST_LIO_ROS_SOURCE_DIR "/config/aist.yaml");
  EXPECT_DOUBLE_EQ(
      profile.estimator.preprocessing.voxel_filter.voxel_size_m, 0.9);
  EXPECT_DOUBLE_EQ(profile.estimator.registration_map.voxel_size_m, 0.2);
  EXPECT_TRUE(
      profile.estimator.lifecycle.enable_periodic_local_map_snapshot);
}

TEST_F(ParameterLoaderTest, PublishLocalMapControlsPeriodicCoreSnapshot) {
  rclcpp::Node node{"parameter_loader_publish_map_test"};
  auto parameters = ParameterLoader::declareAndLoad(node);
  parameters.publish_local_map = false;
  const auto profile = makeEstimatorProfile(parameters);
  EXPECT_FALSE(
      profile.estimator.lifecycle.enable_periodic_local_map_snapshot);
}

TEST_F(ParameterLoaderTest, CanonicalConfigRequiresBothVoxelFields) {
  for (const auto& [line, suffix] :
       std::array<std::pair<std::string_view, std::string_view>, 2>{
           std::pair{"      scan_voxel_size_m: 0.2\n", "scan_voxel"},
           std::pair{"    mapping:\n"
                     "      registration_map: {voxel_size_m: 0.2}\n",
                     "registration_map_voxel"}}) {
    const auto path = canonicalConfigWithout(line, suffix);
    EXPECT_THROW(loadCanonicalEstimatorProfile(path.string()),
                 std::invalid_argument);
    std::filesystem::remove(path);
  }
}

TEST_F(ParameterLoaderTest, DirectAndRosConfigEquivalentFieldByField) {
  const std::string path =
      FAST_LIO_ROS_SOURCE_DIR
      "/config/mid360-real.yaml";
  const auto direct = loadCanonicalEstimatorProfile(path);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", path});
  rclcpp::Node ros_node{"fast_lio", options};
  const auto ros =
      makeEstimatorProfile(ParameterLoader::declareAndLoad(ros_node));
  expectEstimatorConfigsEqual(direct.estimator, ros.estimator);
  EXPECT_EQ(direct.config_sha256, ros.config_sha256);
  EXPECT_EQ(direct.lidar_topic, ros.lidar_topic);
  EXPECT_EQ(direct.imu_topic, ros.imu_topic);
  EXPECT_EQ(direct.lidar_input_frame, ros.lidar_input_frame);
  EXPECT_EQ(direct.imu_input_frame, ros.imu_input_frame);
  EXPECT_EQ(direct.clock_domain, ros.clock_domain);
  EXPECT_EQ(direct.timestamp_policy, ros.timestamp_policy);
}

TEST_F(ParameterLoaderTest, InvalidConfigIsRejectedWithoutDefaults) {
  const auto path =
      std::filesystem::temp_directory_path() /
      "m1_d2_missing_required_estimator_config.yaml";
  {
    std::ofstream stream(path);
    stream << "fast_lio:\n  ros__parameters:\n"
              "    frames: {odom: odom}\n";
  }
  EXPECT_THROW(loadCanonicalEstimatorProfile(path.string()),
               std::invalid_argument);
  std::filesystem::remove(path);

  rclcpp::Node node{"parameter_loader_invalid_numeric_test"};
  auto parameters = ParameterLoader::declareAndLoad(node);
  parameters.scan_voxel_size_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
  parameters.scan_voxel_size_m = 0.0;
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
  parameters.scan_voxel_size_m = 0.2;
  parameters.registration_map_voxel_size_m = 0.0;
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
  parameters.registration_map_voxel_size_m = -0.2;
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
  parameters.registration_map_voxel_size_m =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
  parameters.registration_map_voxel_size_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
  parameters.registration_map_voxel_size_m = 0.2;
  parameters.scan_voxel_size_m = -0.2;
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
  parameters.scan_voxel_size_m = std::numeric_limits<double>::infinity();
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
  parameters.scan_voxel_size_m = 0.2;
  parameters.rotation_imu_lidar_xyzw = {0.0, 0.0, 0.0, 2.0};
  EXPECT_THROW(ParameterLoader::validate(parameters), std::invalid_argument);
}

}  // namespace uav::nav::lio
