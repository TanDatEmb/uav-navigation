#include "fast_lio_core/geometry/transform_utils.hpp"

namespace uav::nav::lio {

std::vector<Eigen::Vector3f> transformPoints(const RigidTransform& T_target_source,
                                             const std::vector<Eigen::Vector3f>& points_source) {
  std::vector<Eigen::Vector3f> points_target;
  points_target.reserve(points_source.size());
  for (const auto& point : points_source) {
    points_target.push_back(T_target_source.apply(point));
  }
  return points_target;
}

}  // namespace uav::nav::lio
