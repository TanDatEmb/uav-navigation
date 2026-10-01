#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace livox_ros {

struct FrameIdContract {
  std::string lidar;
  std::string imu;
};

inline FrameIdContract makeFrameIdContract(std::string lidar, std::string imu) {
  if (lidar.empty() || imu.empty()) {
    throw std::invalid_argument("Livox lidar and IMU frame IDs must not be empty");
  }
  if (lidar == imu) {
    throw std::invalid_argument("Livox lidar and IMU frame IDs must be distinct");
  }
  return {std::move(lidar), std::move(imu)};
}

template <typename MessageT>
inline void assignLidarFrameId(MessageT& message, const FrameIdContract& frames) {
  message.header.frame_id = frames.lidar;
}

template <typename MessageT>
inline void assignImuFrameId(MessageT& message, const FrameIdContract& frames) {
  message.header.frame_id = frames.imu;
}

}  // namespace livox_ros
