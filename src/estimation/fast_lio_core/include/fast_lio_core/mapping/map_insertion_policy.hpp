#pragma once

#include <cstddef>

namespace uav::nav::lio {

struct MapInsertionContext {
  bool estimator_tracking{false};
  bool lidar_update_successful{false};
  bool correction_usable{false};
  bool transform_finite{false};
  std::size_t filtered_point_count{0};
};

struct MapInsertionPolicyConfig {
  std::size_t minimum_point_count{20};
};

class MapInsertionPolicy {
 public:
  explicit MapInsertionPolicy(MapInsertionPolicyConfig config = {});

  [[nodiscard]] bool permits(const MapInsertionContext& context) const noexcept;

 private:
  MapInsertionPolicyConfig config_;
};

}  // namespace uav::nav::lio
