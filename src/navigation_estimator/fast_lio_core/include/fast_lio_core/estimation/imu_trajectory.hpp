#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <vector>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/geometry/frame.hpp"
#include "fast_lio_core/geometry/frame_ids.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

struct ImuTrajectoryState {
  Timestamp time;
  Eigen::Quaterniond orientation_odom_imu{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_odom_imu_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_odom_imu_m_s{Eigen::Vector3d::Zero()};

  [[nodiscard]] bool allFinite() const noexcept;
};

class ImuTrajectory {
 public:
  explicit ImuTrajectory(FrameId odom_frame = odomFrame(), FrameId imu_frame = imuFrame());

  [[nodiscard]] Status addState(ImuTrajectoryState state);
  [[nodiscard]] Result<ImuTrajectoryState> interpolate(const Timestamp& time) const;
  [[nodiscard]] Result<RigidTransform> pose(const Timestamp& time) const;

  [[nodiscard]] const std::vector<ImuTrajectoryState>& states() const noexcept;
  [[nodiscard]] const FrameId& odomFrameId() const noexcept;
  [[nodiscard]] const FrameId& imuFrameId() const noexcept;
  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] const Timestamp* startTime() const noexcept;
  [[nodiscard]] const Timestamp* endTime() const noexcept;

 private:
  FrameId odom_frame_;
  FrameId imu_frame_;
  std::vector<ImuTrajectoryState> states_;
};

}  // namespace uav::nav::lio
