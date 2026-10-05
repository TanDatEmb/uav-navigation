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
  const Eigen::Vector3d position = point.position_lidar_m.cast<double>();
  const double scale = position.cwiseAbs().maxCoeff();
  if (!std::isfinite(scale)) return false;
  const double range = scale == 0.0 ? 0.0 : scale * (position / scale).norm();
  if (!std::isfinite(range)) return false;
  return range >= config_.minimum_range_m && range <= config_.maximum_range_m;
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
