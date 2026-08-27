/*
 * Product-owned navigation implementation.
 * Algorithmic provenance and external attributions are documented in the
 * package documentation; they are not part of the runtime API or behaviour.
 */

/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>

#include <data_structure/base/polytope.h>
#include <data_structure/base/ellipsoid.h>

#include <utils/optimization/sdlp.h>
#include <utils/geometry/geometry_utils.h>
#include <utils/optimization/optimization_utils.h>
#include <utils/optimization/mvie.h>
#include <utils/header/type_utils.hpp>

#include <planner_runtime_context/planner_runtime_context.hpp>
#include <planner_core/absolute_deadline.hpp>

namespace navigation_planning_backend {
    using navigation_math::RET_CODE;
    using geometry_utils::Ellipsoid;
    using geometry_utils::Polytope;

#ifdef NAVIGATION_PLANNING_BACKEND_TESTING
    struct CiriGeometryTestAccess;
#endif

    class CIRI {
#ifdef NAVIGATION_PLANNING_BACKEND_TESTING
        friend struct CiriGeometryTestAccess;
#endif
        navigation_planner_context::PlannerRuntimeContext::Ptr planner_context_;
        double robot_r_{0};
        int iter_num_{1};
        bool debug_en{false};

        Ellipsoid sphere_template_;
        Polytope optimized_polytope_;

        std::ofstream failed_log;

/**
 * @brief findEllipsoid: find maximum ellipsoid with RILS
 * @param pc the obstacle points
 * @param a the start point of the line segment seed
 * @param b the end point of the line segment seed
 * @param out_ell the output ellipsoid
 * @param r_robot the robot_size, decide if the polytope need to be shrink
 * @param _fix_p decide if the ellipsoid center need to be optimized
 * @param iterations number of the alternating optimization
 */
        void findEllipsoid(
                const Eigen::Matrix3Xd &pc,
                const Eigen::Vector3d &a,
                const Eigen::Vector3d &b,
                Ellipsoid &out_ell);

        static bool findTangentPlaneOfSphere(const Eigen::Vector3d &center, const double &r,
                                             const Eigen::Vector3d &pass_point,
                                             const Eigen::Vector3d &seed_p,
                                             Eigen::Vector4d &outter_plane);

        static double distancePointToSegment(const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& B) {
            if (!P.allFinite() || !A.allFinite() || !B.allFinite()) {
                // A non-finite obstacle must never make the clearance
                // precheck fail open through NaN comparison semantics.
                return 0.0;
            }
            const Eigen::Vector3d AB = B - A;
            const Eigen::Vector3d AP = P - A;

            const double AB_AB = AB.squaredNorm();
            const double scale = std::max({1.0, AB.squaredNorm(), AP.squaredNorm()});
            const double degeneracy_limit =
                64.0 * std::numeric_limits<double>::epsilon() * scale;
            if (!std::isfinite(AB_AB) || AB_AB <= degeneracy_limit) {
                return AP.norm();
            }

            const double t = AP.dot(AB) / AB_AB;

            // 判断 t 是否在线段范围内
            if (t < 0.0) {
                // t 小于 0，最近点是 A
                return (P - A).norm();
            } else if (t > 1.0) {
                // t 大于 1，最近点是 B
                return (P - B).norm();
            } else {
                // t 在 0 到 1 之间，最近点是 Q(t)
                Eigen::Vector3d Q = A + t * AB;
                return (P - Q).norm();
            }
        }

    public:
        CIRI() = default;

        CIRI(const navigation_planner_context::PlannerRuntimeContext::Ptr & planner_context):planner_context_(planner_context){
            debug_en = true;
            // failed_log.open(failed_log_path, std::ios::out | std::ios::trunc);
        }

        ~CIRI() = default;

        typedef std::shared_ptr<CIRI> Ptr;

        void setupParams(double robot_r, int iter_num);

        RET_CODE comvexDecomposition(const Eigen::MatrixX4d &bd,
                                     const Eigen::Matrix3Xd &pc,
                                     const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b,
                                     const AbsoluteDeadline* deadline = nullptr);

        void getPolytope(Polytope &optimized_poly);
    };
}
