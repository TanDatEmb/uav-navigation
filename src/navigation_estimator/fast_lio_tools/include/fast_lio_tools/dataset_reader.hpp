#pragma once

#include <filesystem>
#include <functional>

#include "fast_lio_core/sensor/imu_sample.hpp"
#include "fast_lio_core/sensor/lidar_scan.hpp"

namespace uav::nav::lio {

// M1's dependency-light adapter format is documented in
// docs/verification/offline_evaluation.md. rosbag2 support can be added at this
// boundary without introducing ROS into fast_lio_core.
class DatasetReader {
 public:
  using ImuCallback = std::function<void(const ImuSample&)>;
  using LidarCallback = std::function<void(const LidarScan&)>;

  explicit DatasetReader(std::filesystem::path directory);
  void replay(const ImuCallback& on_imu, const LidarCallback& on_lidar) const;

 private:
  std::filesystem::path directory_;
};

}  // namespace uav::nav::lio
