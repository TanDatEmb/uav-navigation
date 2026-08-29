#include "fast_lio_core/deskew/scan_deskewer.hpp"

#include <algorithm>
#include <string>

namespace uav::nav::lio {

ScanDeskewer::ScanDeskewer(ScanDeskewerConfig config) : config_(config) {}

Result<DeskewResult> ScanDeskewer::deskew(const LidarScan& scan,
                                          const ImuTrajectory& imu_trajectory,
                                          const RigidTransform& T_imu_lidar) const {
  const Status scan_status = scan.validate();
  if (!scan_status.ok()) {
    return scan_status;
  }

  const DeskewMode effective_mode =
      config_.mode == DeskewMode::kAuto
          ? (scan.has_per_point_time ? DeskewMode::kPerPoint : DeskewMode::kSimultaneousScan)
          : config_.mode;
  const Timestamp reference_time =
      config_.reference == DeskewReference::kScanStart ? scan.start_time : scan.end_time;

  DeskewResult output;
  output.scan = scan;
  output.reference_time = reference_time;
  const auto relative_time_bounds = std::minmax_element(
      scan.points.begin(), scan.points.end(), [](const LidarPoint& lhs, const LidarPoint& rhs) {
        return lhs.relative_time_ns < rhs.relative_time_ns;
      });
  output.point_time_min_ns = relative_time_bounds.first->relative_time_ns;
  output.point_time_max_ns = relative_time_bounds.second->relative_time_ns;

  if (effective_mode == DeskewMode::kSimultaneousScan) {
    if (scan.has_per_point_time) {
      return Status(StatusCode::kDeskewRejected,
                    "Simultaneous-scan mode requires absent per-point timing");
    }
    output.status = DeskewStatus::kBypassedSimultaneousScan;
    output.deskew_applied = false;
    return output;
  }
  if (!scan.has_per_point_time) {
    return Status(StatusCode::kDeskewRejected, "Per-point deskew requires point relative time");
  }
  if (T_imu_lidar.targetFrame() != imu_trajectory.imuFrameId()) {
    return Status(StatusCode::kFrameMismatch,
                  "T_imu_lidar target frame '" + std::string(T_imu_lidar.targetFrame().name()) +
                      "' does not match trajectory IMU frame '" +
                      std::string(imu_trajectory.imuFrameId().name()) + "'");
  }

  const auto T_odom_imu_reference = imu_trajectory.pose(reference_time);
  if (!T_odom_imu_reference.ok()) {
    return T_odom_imu_reference.status();
  }
  const auto T_odom_lidar_reference = T_odom_imu_reference.value().compose(T_imu_lidar);
  if (!T_odom_lidar_reference.ok()) {
    return T_odom_lidar_reference.status();
  }
  const auto T_lidar_reference_odom_result =
      T_odom_lidar_reference.value().inverse();
  if (!T_lidar_reference_odom_result.ok()) {
    return T_lidar_reference_odom_result.status();
  }
  const RigidTransform& T_lidar_reference_odom =
      T_lidar_reference_odom_result.value();

  for (auto& point : output.scan.points) {
    const auto point_time = scan.pointTime(point);
    if (!point_time.ok()) {
      ++output.interpolation_failure_count;
      return point_time.status();
    }
    const auto T_odom_imu_point = imu_trajectory.pose(point_time.value());
    if (!T_odom_imu_point.ok()) {
      ++output.interpolation_failure_count;
      return T_odom_imu_point.status();
    }
    const auto T_odom_lidar_point = T_odom_imu_point.value().compose(T_imu_lidar);
    if (!T_odom_lidar_point.ok()) {
      return T_odom_lidar_point.status();
    }
    const auto T_lidar_reference_lidar_point =
        T_lidar_reference_odom.compose(T_odom_lidar_point.value());
    if (!T_lidar_reference_lidar_point.ok()) {
      return T_lidar_reference_lidar_point.status();
    }
    point.position_lidar_m = T_lidar_reference_lidar_point.value().apply(point.position_lidar_m);
  }
  output.status = DeskewStatus::kApplied;
  output.deskew_applied = true;
  return output;
}

}  // namespace uav::nav::lio
