#pragma once

#include <vector>

#include "fast_lio_core/sensor/lidar_point.hpp"

namespace uav::nav::lio {

struct PointFilterConfig {
  double minimum_range_m{0.1};
  double maximum_range_m{40.0};
};

class PointFilter {
 public:
  explicit PointFilter(PointFilterConfig config = {});

  [[nodiscard]] bool accepts(const LidarPoint& point) const noexcept;
  [[nodiscard]] std::vector<LidarPoint> filter(const std::vector<LidarPoint>& points) const;

 private:
  PointFilterConfig config_;
};

}  // namespace uav::nav::lio
