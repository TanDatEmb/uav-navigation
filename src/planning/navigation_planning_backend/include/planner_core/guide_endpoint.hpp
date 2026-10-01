#pragma once

#include <cmath>

#include <utils/header/type_utils.hpp>

namespace navigation_planning_backend {

struct GuideEndpoint {
    navigation_math::Vec3f position;
    bool goal_connected{false};
};

inline GuideEndpoint resolveGuideEndpoint(const navigation_math::Vec3f &guide_endpoint,
                                          const navigation_math::Vec3f &goal,
                                          const double connection_tolerance_m) {
    GuideEndpoint result{guide_endpoint, false};
    if (!guide_endpoint.allFinite() || !goal.allFinite() ||
        !std::isfinite(connection_tolerance_m) || connection_tolerance_m < 0.0) {
        return result;
    }
    // The goal contract is inclusive: a point exactly at the allowed
    // completion radius is connected everywhere in the planner/runtime.
    if ((guide_endpoint - goal).norm() <= connection_tolerance_m) {
        result.position = goal;
        result.goal_connected = true;
    }
    return result;
}

}  // namespace navigation_planning_backend
