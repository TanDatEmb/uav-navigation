#pragma once

#include <Eigen/Core>
#include <vector>

#include "fast_lio_core/geometry/rigid_transform.hpp"

namespace uav::nav::lio {

[[nodiscard]] std::vector<Eigen::Vector3f> transformPoints(
    const RigidTransform& T_target_source, const std::vector<Eigen::Vector3f>& points_source);

}  // namespace uav::nav::lio
