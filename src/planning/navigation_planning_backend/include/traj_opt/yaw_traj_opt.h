/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#pragma once

#include "utils/geometry/geometry_utils.h"
#include <data_structure/base/trajectory.h>
#include <utils/header/type_utils.hpp>

namespace traj_opt {
    using namespace geometry_utils;
    class YawTrajOpt {
    private:
        double yaw_rate_max_rad_s_{10};
        double yaw_acceleration_max_rad_s2_{10};

    public:

        explicit YawTrajOpt(const double &max_yaw_rate_rad_s,
                            const double &max_yaw_acceleration_rad_s2 = 10.0);

        typedef std::shared_ptr<YawTrajOpt> Ptr;

        // Generate a semantic heading trajectory that is independent of the
        // position-trajectory shape. If the requested excursion cannot fit
        // the rate/acceleration envelope in the available duration, execute
        // the largest certified partial turn instead of changing duration.
        bool optimizeToTarget(const Vec4f &initial_state,
                              double target_yaw_rad,
                              const Trajectory &position_trajectory,
                              Trajectory &output_trajectory);

    };


}
