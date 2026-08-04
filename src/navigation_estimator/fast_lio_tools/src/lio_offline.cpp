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
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fast_lio_core/pipeline/fast_lio_pipeline.hpp"
#include "fast_lio_ros/parameter_loader.hpp"
#include "fast_lio_ros/ros_imu_adapter.hpp"
#include "fast_lio_ros/ros_lidar_adapter.hpp"
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
  std::size_t deskew_applied_count{};
  std::int64_t maximum_imu_gap_ns{};
  std::int64_t previous_imu_ns{-1};
  std::int64_t first_lidar_start_ns{-1};
  std::int64_t last_lidar_end_ns{-1};
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
  RosLivoxCustomAdapter livox_adapter(
      profile.lidar_input_frame, profile.clock_domain,
      profile.timestamp_policy);
  RosLidarAdapter pointcloud_adapter(
      profile.lidar_input_frame,
      profile.lidar_timing_mode == "per_point" ? LidarTimingMode::kPerPoint
                                                : LidarTimingMode::kSimultaneousScan,
      profile.clock_domain, profile.point_time);
  Counters counters;
  std::ofstream diagnostics(output_path / "diagnostics.csv");
  std::ofstream trajectory(output_path / "trajectory.csv");
  std::ofstream corrections(output_path / "corrections.csv");
  diagnostics << "record_index,reason,status,synchronized,deskew_attempted,"
                 "deskew_applied,correction_attempted,correction_succeeded,"
                 "map_update_performed,scan_start_ns,scan_end_ns,scan_duration_ns,"
                 "imu_samples,imu_gap_ns,"
                 "input_points,filtered_points,queries,accepted_residuals,"
                 "residual_rms,iterations,final_increment_norm,map_points,"
                 "prediction_us,deskew_us,preprocessing_us,residual_build_us,"
                 "ikfom_update_us,map_insert_crop_us,map_maintenance_us,snapshot_us,"
                 "total_processing_us,map_size_before_insert,map_candidate_count,"
                 "map_inserted_count,map_size_after_insert,crop_performed,"
                 "crop_removed_count,crop_triggered_by_motion,"
                 "crop_triggered_by_point_threshold,map_size_before_maintenance,"
                 "confidence_pruned_count,distance_pruned_count,"
                 "redundancy_pruned_count,map_size_after_maintenance,"
                 "local_map_center_x,local_map_center_y,local_map_center_z,"
                 "local_map_half_extent_x,local_map_half_extent_y,"
                 "local_map_half_extent_z,snapshot_point_count,"
                 "dynamic_filter_enabled,dynamic_evidence_voxel_count,"
                 "dynamic_candidate_count\n";
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
        LidarScan scan;
        if (profile.lidar_message_type == "pointcloud2") {
          scan = pointcloud_adapter.convert(
              deserialize<sensor_msgs::msg::PointCloud2>(record));
        } else if (profile.lidar_message_type == "livox_custom") {
          scan = livox_adapter.convert(
              deserialize<livox_ros_driver2::msg::CustomMsg>(record));
        } else {
          throw std::invalid_argument("unsupported configured LiDAR message type");
        }
        if (counters.first_lidar_start_ns < 0) {
          counters.first_lidar_start_ns = scan.start_time.nanoseconds();
        }
        counters.last_lidar_end_ns = scan.end_time.nanoseconds();
        const auto status = pipeline.pushLidar(std::move(scan));
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
                  << (diagnostic.synchronization.synchronized ? 1 : 0) << ','
                  << (diagnostic.deskew.deskew_attempted ? 1 : 0) << ','
                  << (diagnostic.deskew.deskew_applied ? 1 : 0) << ','
                  << (diagnostic.registration.correction_attempted ? 1 : 0) << ','
                  << (diagnostic.registration.correction_succeeded ? 1 : 0) << ','
                  << (diagnostic.map.map_update_performed ? 1 : 0) << ','
                  << diagnostic.synchronization.scan_start_ns << ','
                  << diagnostic.synchronization.scan_end_ns << ','
                  << diagnostic.synchronization.scan_duration_ns << ','
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
                  << diagnostic.timing.map_maintenance_us << ','
                  << diagnostic.timing.snapshot_us << ','
                  << diagnostic.timing.total_processing_us << ','
                  << diagnostic.map.map_size_before_insert << ','
                  << diagnostic.map.map_candidate_count << ','
                  << diagnostic.map.map_inserted_count << ','
                  << diagnostic.map.map_size_after_insert << ','
                  << (diagnostic.map.crop_performed ? 1 : 0) << ','
                  << diagnostic.map.crop_removed_count << ','
                  << (diagnostic.map.crop_triggered_by_motion ? 1 : 0) << ','
                  << (diagnostic.map.crop_triggered_by_point_threshold ? 1 : 0) << ','
                  << diagnostic.map.map_size_before_maintenance << ','
                  << diagnostic.map.confidence_pruned_count << ','
                  << diagnostic.map.distance_pruned_count << ','
                  << diagnostic.map.redundancy_pruned_count << ','
                  << diagnostic.map.map_size_after_maintenance << ','
                  << diagnostic.map.local_map_center_odom_m.x() << ','
                  << diagnostic.map.local_map_center_odom_m.y() << ','
                  << diagnostic.map.local_map_center_odom_m.z() << ','
                  << diagnostic.map.local_map_half_extent_m.x() << ','
                  << diagnostic.map.local_map_half_extent_m.y() << ','
                  << diagnostic.map.local_map_half_extent_m.z() << ','
                  << diagnostic.map.snapshot_point_count << ','
                  << (diagnostic.map.dynamic_filter_enabled ? 1 : 0) << ','
                  << diagnostic.map.dynamic_evidence_voxel_count << ','
                  << diagnostic.map.dynamic_candidate_count << '\n';
      if (diagnostic.deskew.deskew_applied) {
        ++counters.deskew_applied_count;
      }
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
  const auto final_diagnostics = pipeline.diagnostics();
  const auto processing = final_diagnostics.processing;
  const auto normalization =
      pointcloud_adapter.normalizationStatistics();
  const double dataset_duration_seconds =
      counters.first_lidar_start_ns >= 0 &&
              counters.last_lidar_end_ns > counters.first_lidar_start_ns
          ? static_cast<double>(counters.last_lidar_end_ns -
                                counters.first_lidar_start_ns) *
                1e-9
          : 0.0;
  const double effective_corrected_output_rate_hz =
      dataset_duration_seconds > 0.0
          ? static_cast<double>(processing.correction_success_count) /
                dataset_duration_seconds
          : 0.0;
  writePcd(output_path / "local_map.pcd", map);
  const auto write_run_metadata =
      [&](const std::filesystem::path& path) {
    std::ofstream summary(path);
    summary << "{\n"
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
    summary
          << "  \"raw_dataset_imu_count\": " << counters.raw_imu << ",\n"
          << "  \"raw_dataset_lidar_count\": " << counters.raw_lidar << ",\n"
          << "  \"core_accepted_imu_count\": " << counters.accepted_imu << ",\n"
          << "  \"core_accepted_lidar_count\": " << counters.accepted_lidar << ",\n"
          << "  \"core_rejected_imu_count\": " << counters.rejected_imu << ",\n"
          << "  \"core_rejected_lidar_count\": " << counters.rejected_lidar << ",\n"
          << "  \"normalization_input_point_count\": "
          << normalization.input_point_count << ",\n"
          << "  \"normalization_emitted_point_count\": "
          << normalization.emitted_point_count << ",\n"
          << "  \"normalization_dropped_overlapping_point_count\": "
          << normalization.dropped_overlapping_point_count << ",\n"
          << "  \"normalization_dropped_ratio\": "
          << (normalization.input_point_count == 0
                  ? 0.0
                  : static_cast<double>(
                        normalization.dropped_overlapping_point_count) /
                        static_cast<double>(
                            normalization.input_point_count))
          << ",\n"
          << "  \"source_maximum_imu_gap_ns\": "
          << counters.maximum_imu_gap_ns << ",\n"
          << "  \"process_result_count\": " << counters.process_results << ",\n"
          << "  \"successful_correction_count\": " << counters.corrections
          << ",\n"
          << "  \"failed_correction_count\": " << counters.failed_corrections
          << ",\n"
          << "  \"deskew_applied\": "
          << (counters.deskew_applied_count > 0 ? "true" : "false") << ",\n"
          << "  \"deskew_applied_count\": " << counters.deskew_applied_count
          << ",\n"
          << "  \"overlap_rejected_count\": "
          << processing.overlap_rejected_count << ",\n"
          << "  \"missing_bracket_rejected_count\": "
          << processing.missing_bracket_rejected_count << ",\n"
          << "  \"invalid_timestamp_rejected_count\": "
          << processing.invalid_timestamp_rejected_count << ",\n"
          << "  \"synchronized_group_count\": "
          << processing.synchronized_group_count << ",\n"
          << "  \"correction_attempt_count\": "
          << processing.correction_attempt_count << ",\n"
          << "  \"buffer_acceptance_ratio\": "
          << processing.bufferAcceptanceRatio() << ",\n"
          << "  \"synchronization_ratio\": "
          << processing.synchronizationRatio() << ",\n"
          << "  \"correction_success_ratio_among_attempts\": "
          << processing.correctionSuccessRatio() << ",\n"
          << "  \"dataset_duration_seconds\": "
          << dataset_duration_seconds << ",\n"
          << "  \"effective_corrected_output_rate_hz\": "
          << effective_corrected_output_rate_hz << ",\n"
          << "  \"map_point_count\": " << map.size() << ",\n"
          << "  \"map_size_before_insert\": "
          << final_diagnostics.map.map_size_before_insert << ",\n"
          << "  \"map_candidate_count\": "
          << final_diagnostics.map.map_candidate_count << ",\n"
          << "  \"map_inserted_count\": "
          << final_diagnostics.map.map_inserted_count << ",\n"
          << "  \"map_size_after_insert\": "
          << final_diagnostics.map.map_size_after_insert << ",\n"
          << "  \"map_size_before_maintenance\": "
          << final_diagnostics.map.map_size_before_maintenance << ",\n"
          << "  \"map_size_after_maintenance\": "
          << final_diagnostics.map.map_size_after_maintenance << ",\n"
          << "  \"map_maintenance_us\": "
          << final_diagnostics.map.map_maintenance_us << ",\n"
          << "  \"snapshot_point_count\": "
          << final_diagnostics.map.snapshot_point_count << ",\n"
          << "  \"wall_runtime_us\": " << elapsed_us << "\n"
          << "}\n";
  };
  write_run_metadata(output_path / "summary.json");
  if (!std::filesystem::exists(output_path / "run.json")) {
    write_run_metadata(output_path / "run.json");
  }
  return 0;
}

}  // namespace
}  // namespace uav::nav::lio

int main(int argc, char** argv) {
  if (argc != 4 && argc != 5) {
    std::cerr << "usage: lio_offline BAG_DIRECTORY CONFIG_YAML "
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
