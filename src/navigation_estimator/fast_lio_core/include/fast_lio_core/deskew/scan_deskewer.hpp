#pragma once

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/deskew/deskew_mode.hpp"
#include "fast_lio_core/deskew/deskew_result.hpp"
#include "fast_lio_core/estimation/imu_trajectory.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/sensor/lidar_scan.hpp"

namespace uav::nav::lio {

struct ScanDeskewerConfig {
  DeskewMode mode{DeskewMode::kPerPoint};
  DeskewReference reference{DeskewReference::kScanEnd};
};

class ScanDeskewer {
 public:
  explicit ScanDeskewer(ScanDeskewerConfig config = {});

  // T_imu_lidar must represent ^I T_L. Applied output points remain in
  // lidar_link, but all are expressed at the selected common reference time.
  [[nodiscard]] Result<DeskewResult> deskew(const LidarScan& scan,
                                            const ImuTrajectory& imu_trajectory,
                                            const RigidTransform& T_imu_lidar) const;

 private:
  ScanDeskewerConfig config_;
};

}  // namespace uav::nav::lio
