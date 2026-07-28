#include "fast_lio_core/preprocessing/point_filter.hpp"

#include <cmath>

namespace uav::nav::lio {

PointFilter::PointFilter(PointFilterConfig config) : config_(config) {}

bool PointFilter::accepts(const LidarPoint& point) const noexcept {
  if (!point.allFinite() || !std::isfinite(config_.minimum_range_m) ||
      !std::isfinite(config_.maximum_range_m) || config_.minimum_range_m < 0.0 ||
      config_.maximum_range_m < config_.minimum_range_m) {
    return false;
  }
  const double squared_range = point.position_lidar_m.cast<double>().squaredNorm();
  return squared_range >= config_.minimum_range_m * config_.minimum_range_m &&
         squared_range <= config_.maximum_range_m * config_.maximum_range_m;
}

std::vector<LidarPoint> PointFilter::filter(const std::vector<LidarPoint>& points) const {
  std::vector<LidarPoint> output;
  output.reserve(points.size());
  for (const auto& point : points) {
    if (accepts(point)) {
      output.push_back(point);
    }
  }
  return output;
}

}  // namespace uav::nav::lio
