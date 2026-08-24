#pragma once

#include <cmath>

#include <utils/header/type_utils.hpp>

namespace super_planner {

struct GuideEndpoint {
    super_utils::Vec3f position;
    bool goal_connected{false};
};

inline GuideEndpoint resolveGuideEndpoint(const super_utils::Vec3f &guide_endpoint,
                                          const super_utils::Vec3f &goal,
                                          const double connection_tolerance_m) {
    GuideEndpoint result{guide_endpoint, false};
    if (!guide_endpoint.allFinite() || !goal.allFinite() ||
        !std::isfinite(connection_tolerance_m) || connection_tolerance_m < 0.0) {
        return result;
    }
    if ((guide_endpoint - goal).norm() < connection_tolerance_m) {
        result.position = goal;
        result.goal_connected = true;
    }
    return result;
}

}  // namespace super_planner
