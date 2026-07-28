#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "fast_lio_core/pipeline/fast_lio_pipeline.hpp"
#include "fast_lio_ros/parameter_loader.hpp"
#include "fast_lio_ros/ros_imu_adapter.hpp"
#include "fast_lio_ros/ros_livox_custom_adapter.hpp"

namespace uav::nav::lio {
namespace {

struct Counters {
  std::size_t raw_imu{};
  std::size_t raw_lidar{};
  std::size_t accepted_imu{};
  std::size_t accepted_lidar{};
  std::size_t rejected_imu{};
  std::size_t rejected_lidar{};
  std::size_t process_results{};
  std::size_t corrections{};
  std::size_t failed_corrections{};
  std::int64_t maximum_imu_gap_ns{};
  std::int64_t previous_imu_ns{-1};
};

template <class Message>
Message deserialize(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message) {
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  rclcpp::Serialization<Message> serializer;
  Message message;
  serializer.deserialize_message(&serialized, &message);
  return message;
}

void writePcd(const std::filesystem::path& path,
              const std::vector<Eigen::Vector3d>& points) {
  std::ofstream stream(path);
  stream << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
            "TYPE F F F\nCOUNT 1 1 1\nWIDTH "
         << points.size()
         << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size()
         << "\nDATA ascii\n";
  stream << std::setprecision(9);
  for (const auto& point : points) {
    stream << point.x() << ' ' << point.y() << ' ' << point.z() << '\n';
  }
}

int run(const std::filesystem::path& bag_path,
        const std::filesystem::path& config_path,
        const std::filesystem::path& output_path,
        std::size_t maximum_lidar_messages) {
  std::filesystem::create_directories(output_path);
  rosbag2_cpp::Reader reader;
  reader.open(bag_path.string());

  const EstimatorProfile profile =
      loadCanonicalEstimatorProfile(config_path.string());
  FastLioPipeline pipeline(profile.estimator);
  RosImuAdapter imu_adapter(profile.imu_input_frame, profile.clock_domain);
  RosLivoxCustomAdapter lidar_adapter(
      profile.lidar_input_frame, profile.clock_domain,
      profile.timestamp_policy);
  Counters counters;
  std::ofstream diagnostics(output_path / "diagnostics.csv");
  std::ofstream trajectory(output_path / "trajectory.csv");
  std::ofstream corrections(output_path / "corrections.csv");
  diagnostics << "record_index,reason,status,imu_samples,imu_gap_ns,"
                 "input_points,filtered_points,queries,accepted_residuals,"
                 "residual_rms,iterations,final_increment_norm,map_points,"
                 "prediction_us,deskew_us,preprocessing_us,residual_build_us,"
                 "ikfom_update_us,map_insert_crop_us,snapshot_us,"
                 "total_processing_us\n";
  trajectory << "time_ns,x,y,z,qx,qy,qz,qw\n";
  corrections << "time_ns,status,iterations,residual_rms,map_points\n";
  std::size_t record_index = 0;
  const auto wall_start = std::chrono::steady_clock::now();

  while (reader.has_next()) {
    const auto record = reader.read_next();
    ++record_index;
    if (record->topic_name == profile.imu_topic) {
      ++counters.raw_imu;
      const auto message = deserialize<sensor_msgs::msg::Imu>(record);
      const auto sample = imu_adapter.convert(message);
      if (counters.previous_imu_ns >= 0) {
        counters.maximum_imu_gap_ns =
            std::max(counters.maximum_imu_gap_ns,
                     sample.time.nanoseconds() - counters.previous_imu_ns);
      }
      counters.previous_imu_ns = sample.time.nanoseconds();
      const auto status = pipeline.pushImu(sample);
      status.ok() ? ++counters.accepted_imu : ++counters.rejected_imu;
    } else if (record->topic_name == profile.lidar_topic) {
      ++counters.raw_lidar;
      try {
        const auto message =
            deserialize<livox_ros_driver2::msg::CustomMsg>(record);
        const auto status = pipeline.pushLidar(lidar_adapter.convert(message));
        status.ok() ? ++counters.accepted_lidar : ++counters.rejected_lidar;
      } catch (const std::exception& error) {
        ++counters.rejected_lidar;
        diagnostics << record_index << ",adapter rejection: " << error.what()
                    << ",ADAPTER_REJECTED,0,0,0,0,0,0,0,0,0,0,"
                       "0,0,0,0,0,0,0,0\n";
      }
    } else {
      continue;
    }

    while (const auto result = pipeline.processNext()) {
      ++counters.process_results;
      const auto& diagnostic = result->diagnostics;
      diagnostics << record_index << ',' << result->rejection_reason << ','
                  << toString(diagnostic.status) << ','
                  << diagnostic.synchronization.imu_samples_per_scan << ','
                  << diagnostic.synchronization.imu_gap_max_ns << ','
                  << diagnostic.registration.input_point_count << ','
                  << diagnostic.registration.filtered_point_count << ','
                  << diagnostic.registration.query_count << ','
                  << diagnostic.registration.accepted_residual_count << ','
                  << diagnostic.registration.residual_rms_m << ','
                  << diagnostic.registration.iteration_count << ','
                  << diagnostic.registration.final_increment_norm << ','
                  << diagnostic.map.map_point_count << ','
                  << diagnostic.timing.imu_prediction_us << ','
                  << diagnostic.timing.deskew_us << ','
                  << diagnostic.timing.preprocessing_us << ','
                  << diagnostic.timing.residual_build_us << ','
                  << diagnostic.timing.ikfom_update_us << ','
                  << diagnostic.timing.map_insert_crop_us << ','
                  << diagnostic.timing.snapshot_us << ','
                  << diagnostic.timing.total_processing_us << '\n';
      if (result->hasCorrectedOutput()) {
        ++counters.corrections;
        const auto& estimate = *result->corrected_estimate;
        const auto& state = estimate.state;
        const auto& q = state.orientation_odom_imu();
        trajectory << estimate.time.nanoseconds() << ','
                   << state.position_odom_imu_m().transpose().format(
                          Eigen::IOFormat(Eigen::StreamPrecision,
                                          Eigen::DontAlignCols, ",", ","))
                   << ',' << q.x() << ',' << q.y() << ',' << q.z() << ','
                   << q.w() << '\n';
        corrections << estimate.time.nanoseconds() << ",SUCCEEDED,"
                    << diagnostic.registration.iteration_count << ','
                    << diagnostic.registration.residual_rms_m << ','
                    << diagnostic.map.map_point_count << '\n';
      } else if (result->lidar_update_status == LidarUpdateStatus::kRejected) {
        ++counters.failed_corrections;
      }
    }
    if (maximum_lidar_messages > 0 &&
        counters.raw_lidar >= maximum_lidar_messages) {
      break;
    }
  }

  const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              std::chrono::steady_clock::now() - wall_start)
                              .count();
  const auto map = pipeline.registrationMapSnapshot();
  writePcd(output_path / "map_full.pcd", map);
  writePcd(output_path / "map_final_local.pcd", map);
  const auto write_run_metadata =
      [&](const std::filesystem::path& path, bool include_map_frame) {
    std::ofstream summary(path);
    summary << "{\n"
          << "  \"config_path\": \"" << profile.config_path << "\",\n"
          << "  \"config_SHA256\": \"" << profile.config_sha256 << "\",\n"
          << "  \"maximum_imu_gap_ns\": "
          << profile.estimator.synchronization.maximum_imu_gap_ns << ",\n"
          << "  \"deskew_mode\": \"" << profile.lidar_timing_mode << "\",\n"
          << "  \"minimum_range_m\": "
          << profile.estimator.preprocessing.point_filter.minimum_range_m
          << ",\n"
          << "  \"maximum_range_m\": "
          << profile.estimator.preprocessing.point_filter.maximum_range_m
          << ",\n"
          << "  \"voxel_size_m\": "
          << profile.estimator.preprocessing.voxel_filter.voxel_size_m << ",\n"
          << "  \"minimum_imu_samples\": "
          << profile.estimator.initialization.minimum_imu_samples << ",\n"
          << "  \"maximum_ikfom_iterations\": "
          << profile.estimator.ikfom.maximum_iterations << ",\n"
          << "  \"extrinsic_translation_imu_lidar_m\": ["
          << profile.estimator.extrinsic.translation_imu_lidar_m.x() << ", "
          << profile.estimator.extrinsic.translation_imu_lidar_m.y() << ", "
          << profile.estimator.extrinsic.translation_imu_lidar_m.z() << "],\n"
          << "  \"extrinsic_rotation_imu_lidar_xyzw\": ["
          << profile.estimator.extrinsic.rotation_imu_lidar.x() << ", "
          << profile.estimator.extrinsic.rotation_imu_lidar.y() << ", "
          << profile.estimator.extrinsic.rotation_imu_lidar.z() << ", "
          << profile.estimator.extrinsic.rotation_imu_lidar.w() << "],\n";
    if (include_map_frame) {
      summary << "  \"frame\": \"odom\",\n"
              << "  \"point_unit\": \"meter\",\n";
    }
    summary
          << "  \"raw_dataset_imu_count\": " << counters.raw_imu << ",\n"
          << "  \"raw_dataset_lidar_count\": " << counters.raw_lidar << ",\n"
          << "  \"core_accepted_imu_count\": " << counters.accepted_imu << ",\n"
          << "  \"core_accepted_lidar_count\": " << counters.accepted_lidar << ",\n"
          << "  \"core_rejected_imu_count\": " << counters.rejected_imu << ",\n"
          << "  \"core_rejected_lidar_count\": " << counters.rejected_lidar << ",\n"
          << "  \"source_maximum_imu_gap_ns\": "
          << counters.maximum_imu_gap_ns << ",\n"
          << "  \"process_result_count\": " << counters.process_results << ",\n"
          << "  \"successful_correction_count\": " << counters.corrections
          << ",\n"
          << "  \"failed_correction_count\": " << counters.failed_corrections
          << ",\n"
          << "  \"map_point_count\": " << map.size() << ",\n"
          << "  \"wall_runtime_us\": " << elapsed_us << "\n"
          << "}\n";
  };
  write_run_metadata(output_path / "run_summary.json", false);
  write_run_metadata(output_path / "run_manifest.json", false);
  write_run_metadata(output_path / "map_metadata.json", true);
  if (maximum_lidar_messages == 0 &&
      (counters.raw_imu != 8000 || counters.raw_lidar != 1384)) {
    std::cerr << "record-count gate failed\n";
    return 2;
  }
  return 0;
}

}  // namespace
}  // namespace uav::nav::lio

int main(int argc, char** argv) {
  if (argc != 4 && argc != 5) {
    std::cerr << "usage: mid360_dataset_runner BAG_DIRECTORY CONFIG_YAML "
                 "OUTPUT_DIRECTORY [MAX_LIDAR]\n";
    return 64;
  }
  try {
    rclcpp::init(argc, argv);
    const std::size_t maximum_lidar =
        argc == 5 ? static_cast<std::size_t>(std::stoull(argv[4])) : 0U;
    const int result =
        uav::nav::lio::run(argv[1], argv[2], argv[3], maximum_lidar);
    rclcpp::shutdown();
    return result;
  } catch (const std::exception& error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << error.what() << '\n';
    return 1;
  }
}
