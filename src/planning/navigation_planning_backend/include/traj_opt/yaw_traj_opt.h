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
    enum class YawOptimizationFailure : std::uint8_t {
        kNone = 0,
        kInvalidInput = 1,
        kNoFeasibleHold = 2,
    };

    struct YawOptimizationDiagnostics {
        YawOptimizationFailure failure{YawOptimizationFailure::kNone};
        navigation_math::Vec4f initial_state{navigation_math::Vec4f::Zero()};
        double target_yaw_rad{0.0};
        double duration_s{0.0};
        double requested_delta_rad{0.0};
        double full_turn_max_rate_rad_s{0.0};
        double full_turn_max_acceleration_rad_s2{0.0};
        double hold_max_rate_rad_s{0.0};
        double hold_max_acceleration_rad_s2{0.0};
        double stopping_displacement_rad{0.0};
        double stopping_max_rate_rad_s{0.0};
        double stopping_max_acceleration_rad_s2{0.0};
        bool used_stopping_displacement{false};
    };

    using namespace geometry_utils;
    class YawTrajOpt {
    private:
        double yaw_rate_max_rad_s_{10};
        double yaw_acceleration_max_rad_s2_{10};
        YawOptimizationDiagnostics last_diagnostics_{};

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

        [[nodiscard]] const YawOptimizationDiagnostics& lastDiagnostics() const noexcept {
            return last_diagnostics_;
        }

    };


}
