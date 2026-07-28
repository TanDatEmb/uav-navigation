#pragma once

#include <vector>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/sensor/lidar_point.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

struct LidarScan {
  Timestamp start_time;
  Timestamp end_time;
  std::vector<LidarPoint> points;
  bool has_per_point_time{false};

  [[nodiscard]] Status validate() const;
  [[nodiscard]] Result<Timestamp> pointTime(const LidarPoint& point) const;
};

}  // namespace uav::nav::lio
