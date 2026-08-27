/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

#pragma once

#include <iostream>
#include <vector>

#include "utils/geometry/geometry_utils.h"
#include "traj_opt/config.hpp"
#include <utils/optimization/minco.h>

#include <utils/header/type_utils.hpp>
#include <navigation_math/scope_timer.hpp>

namespace traj_opt {
    using namespace geometry_utils;
    using namespace optimization_utils;
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class YawTrajOpt {
    private:
        bool free_goal_{false};
        double yaw_rate_max_rad_s_{10};

    public:

        explicit YawTrajOpt(const double &max_yaw_rate_rad_s);

        typedef std::shared_ptr<YawTrajOpt> Ptr;

        void getYawTimeAllocation(const double &duration, VecDf &times) const ;

        static void getYawWaypointAllocation(const Vec4f &init_state, Vec4f &goal_state, VecDf &way_pts, VecDf &times,
                                      const Trajectory &pos_traj) ;

        bool optimize(const Vec4f &istate_in,
                      const Vec4f &gstate_in,
                      const Trajectory &pos_traj,
                      Trajectory &out_traj,
                      const int & order = 3,
                      const bool &free_start = false,
                      const bool &free_goal = true);

    };


}
