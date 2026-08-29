#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>

#include "fast_lio_core/common/status.hpp"
#include "fast_lio_core/sensor/imu_sample.hpp"
#include "fast_lio_core/sensor/lidar_scan.hpp"
#include "fast_lio_core/time/timestamp_validator.hpp"

namespace uav::nav::lio {

struct MeasurementBufferConfig {
  std::size_t maximum_lidar_scans{32};
  std::size_t maximum_imu_samples{8192};
  bool reject_timestamp_regression{true};
};

struct MeasurementBufferStats {
  std::size_t accepted_lidar_scans{0};
  std::size_t accepted_imu_samples{0};
  std::size_t rejected_lidar_scans{0};
  std::size_t rejected_imu_samples{0};
  std::size_t timestamp_regressions{0};
  std::size_t clock_domain_mismatches{0};
  std::size_t buffer_full_rejections{0};
};

class MeasurementSynchronizer;

class MeasurementBuffer {
 public:
  explicit MeasurementBuffer(MeasurementBufferConfig config = {});

  [[nodiscard]] Status pushLidar(LidarScan scan);
  [[nodiscard]] Status pushImu(ImuSample sample);

  [[nodiscard]] std::size_t lidarSize() const;
  [[nodiscard]] std::optional<Timestamp> nextLidarStartTime() const;
  [[nodiscard]] std::size_t imuSize() const;
  [[nodiscard]] bool empty() const;
  [[nodiscard]] MeasurementBufferStats stats() const;
  void clear();

 private:
  friend class MeasurementSynchronizer;

  MeasurementBufferConfig config_;
  mutable std::mutex mutex_;
  std::deque<LidarScan> lidar_scans_;
  std::deque<ImuSample> imu_samples_;
  TimestampValidator lidar_start_validator_;
  TimestampValidator lidar_end_validator_;
  TimestampValidator imu_validator_;
  MeasurementBufferStats stats_;
};

}  // namespace uav::nav::lio
