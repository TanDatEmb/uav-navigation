#pragma once

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

struct PoseStamped {
  Timestamp time;
  RigidTransform pose;
};

class PoseInterpolator {
 public:
  [[nodiscard]] static Result<RigidTransform> interpolate(const PoseStamped& lower,
                                                          const PoseStamped& upper,
                                                          const Timestamp& query_time);
};

}  // namespace uav::nav::lio
