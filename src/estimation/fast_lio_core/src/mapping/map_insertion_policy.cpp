#include "fast_lio_core/mapping/map_insertion_policy.hpp"

#include <stdexcept>

namespace uav::nav::lio {

MapInsertionPolicy::MapInsertionPolicy(MapInsertionPolicyConfig config) : config_(config) {
  if (config_.minimum_point_count == 0U) {
    throw std::invalid_argument("map insertion minimum point count must be positive");
  }
}

bool MapInsertionPolicy::permits(const MapInsertionContext& context) const noexcept {
  return context.estimator_tracking && context.lidar_update_successful && context.correction_usable &&
         context.transform_finite && context.filtered_point_count >= config_.minimum_point_count;
}

}  // namespace uav::nav::lio
