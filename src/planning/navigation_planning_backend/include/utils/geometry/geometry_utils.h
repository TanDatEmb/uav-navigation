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
#include <limits>

#include <utils/header/type_utils.hpp>
#include <utils/geometry/quickhull.h>
#include <Eigen/Eigen>


namespace geometry_utils {
    using navigation_math::Mat3f;
    using navigation_math::Vec3f;
    using navigation_math::Vec4f;
    using navigation_math::vec_Vec3f;
    using navigation_math::Mat3Df;
    using navigation_math::MatD4f;
    using navigation_math::vec_E;
    using navigation_math::PolyhedronH;

    ///============ 2023-06-30: add by yunfan ============///
    static void simplePMTimeAllocator(const double &a_max, const double &v_max,
                                const double &v0,
                                const double &total_dis,
                                const double &cur_dis, double &t, double &vel) {
        // Helper lambda functions
        auto calc_dis = [](double a, double t) { return 0.5 * a * t * t; };
        auto calc_time = [](double a, double cur_dis) { return sqrt(2 * cur_dis / a); };
        auto solve_quadratic = [](double a, double b, double c) {
            double delta = b * b - 4 * a * c;
            if (!std::isfinite(delta) || !std::isfinite(a) ||
                !std::isfinite(b) || !std::isfinite(c) || a == 0.0) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            // The switching-distance boundary can produce a tiny negative
            // discriminant from floating-point cancellation even though the
            // analytic root is exactly zero.  Clamp only that round-off band;
            // a materially negative discriminant remains invalid.
            const double scale = std::max({1.0, std::abs(b * b), std::abs(4 * a * c)});
            if (delta < 0.0 && delta > -1.0e-12 * scale) {
                delta = 0.0;
            }
            if (delta < 0.0) {
                return std::numeric_limits<double>::quiet_NaN();
            }
            return (-b + sqrt(delta)) / (2 * a);
        };

        // Precompute reusable values
        const double t_to_v_max = v_max / a_max;
        const double dis_to_v_max = calc_dis(a_max, t_to_v_max);

        const double t_to_v0 = v0 / a_max;
        const double dis_to_v0 = calc_dis(a_max, t_to_v0);

        const double dec_time = (v_max - v0) / a_max;
        const double dec_dis = 0.5 * (v_max + v0) * dec_time;

        // Case 1: Only acceleration to v0
        if (total_dis <= dis_to_v0) {
            // The vehicle is already moving faster than the distance can
            // support under the declared acceleration envelope.  Decelerate
            // from v0; the old zero-speed formula could assign a duration
            // and terminal speed that did not satisfy the initial boundary.
            const double discriminant = v0 * v0 - 2.0 * a_max * cur_dis;
            const double scale = std::max({1.0, std::abs(v0 * v0),
                                           std::abs(2.0 * a_max * cur_dis)});
            const double bounded_discriminant =
                discriminant < 0.0 && discriminant > -1.0e-12 * scale
                    ? 0.0 : discriminant;
            if (!std::isfinite(bounded_discriminant) || bounded_discriminant < 0.0) {
                t = std::numeric_limits<double>::quiet_NaN();
                vel = std::numeric_limits<double>::quiet_NaN();
                return;
            }
            t = (v0 - std::sqrt(bounded_discriminant)) / a_max;
            vel = v0 - a_max * t;
            return;
        }

        // Case 2: Acceleration to v_max, then deceleration
        if (total_dis <= dis_to_v_max + dec_dis) {
            const double a = 2 * a_max;
            const double b = -(a_max - v0);
            const double c = -(v0 * v0 / a_max + 2 * total_dis);
            const double t_acc = solve_quadratic(a, b, c);
            const double dis_acc = calc_dis(a_max, t_acc);
            const double cur_v_max = a_max * t_acc;

            if (cur_dis <= dis_acc) {
                t = calc_time(a_max, cur_dis);
                vel = a_max * t;
            } else {
                const double remaining_dis = cur_dis - dis_acc;
                const double t2 = solve_quadratic(-a_max, 2 * cur_v_max, -2 * remaining_dis);
                t = t_acc + t2;
                vel = cur_v_max - t2 * a_max;
            }
            return;
        }

        // Case 3: Acceleration + constant speed + deceleration
        if (cur_dis < dis_to_v_max) {  // Case 3.1: During acceleration phase
            t = calc_time(a_max, cur_dis);
            vel = a_max * t;
            return;
        }

        if (cur_dis < total_dis - dec_dis) {  // Case 3.2: During constant speed phase
            const double remaining_dis = cur_dis - dis_to_v_max;
            const double t_const = remaining_dis / v_max;
            t = t_to_v_max + t_const;
            vel = v_max;
            return;
        }

        // Case 3.3: During deceleration phase
        const double const_phase_dis = total_dis - dec_dis - dis_to_v_max;
        const double remaining_dis = cur_dis - dis_to_v_max - const_phase_dis;
        const double t_dec = solve_quadratic(-a_max, 2 * v_max, -2 * remaining_dis);
        t = t_to_v_max + const_phase_dis / v_max + t_dec;
        vel = v_max - t_dec * a_max;
    }

    struct GuideTimeAllocation {
        vec_Vec3f points;
        std::vector<double> elapsed_s;
        double path_length_m{0.0};
        double terminal_velocity_mps{0.0};
    };

    // Corridor generation has a bounded line-seed contract.  A* may return a
    // sparse path whose final direct edge is longer than that contract even
    // though the complete polyline is collision-free.  Preserve every source
    // waypoint while inserting deterministic interior samples for long edges.
    inline bool subdividePathByMaximumSegmentLength(
        const vec_Vec3f &path, const double max_segment_length,
        vec_Vec3f &subdivided_path) {
        subdivided_path.clear();
        if (path.empty() || !std::isfinite(max_segment_length) ||
            max_segment_length <= 0.0) {
            return false;
        }

        constexpr std::size_t kMaximumSubsegmentsPerEdge = 4096U;
        subdivided_path.reserve(path.size());
        subdivided_path.emplace_back(path.front());
        for (std::size_t index = 1U; index < path.size(); ++index) {
            const Vec3f &start = path[index - 1U];
            const Vec3f &end = path[index];
            if (!start.allFinite() || !end.allFinite()) {
                subdivided_path.clear();
                return false;
            }
            const double segment_length = (end - start).norm();
            if (!std::isfinite(segment_length)) {
                subdivided_path.clear();
                return false;
            }
            const double segment_ratio = segment_length / max_segment_length;
            if (!std::isfinite(segment_ratio) ||
                segment_ratio > static_cast<double>(kMaximumSubsegmentsPerEdge)) {
                subdivided_path.clear();
                return false;
            }
            const std::size_t segment_count = std::max<std::size_t>(
                1U, static_cast<std::size_t>(std::ceil(segment_ratio)));
            for (std::size_t segment = 1U; segment <= segment_count; ++segment) {
                if (segment == segment_count) {
                    // Keep the caller's exact endpoint rather than accumulating
                    // interpolation roundoff across a long guide.
                    subdivided_path.emplace_back(end);
                    continue;
                }
                const double fraction = static_cast<double>(segment) /
                    static_cast<double>(segment_count);
                subdivided_path.emplace_back(
                    start + static_cast<float>(fraction) * (end - start));
            }
        }
        return true;
    }

    // Keep a long A* route as a bounded receding prefix. The mission route
    // remains owned by the controller, while the trajectory solver receives
    // only the currently certifiable spatial horizon. The caller still
    // rechecks the resulting prefix against its authoritative world oracle.
    inline bool truncatePathAtDistance(
            const vec_Vec3f &path, const double maximum_length,
            vec_Vec3f &prefix, bool &truncated) {
        prefix.clear();
        truncated = false;
        if (path.size() < 2U || !std::isfinite(maximum_length) ||
            maximum_length <= 0.0) {
            return false;
        }
        for (const auto &point : path) {
            if (!point.allFinite()) return false;
        }
        prefix.reserve(path.size());
        prefix.emplace_back(path.front());
        constexpr double duplicate_distance_m = 1.0e-9;
        double accumulated_length = 0.0;
        for (std::size_t index = 1U; index < path.size(); ++index) {
            const Vec3f &start = path[index - 1U];
            const Vec3f &end = path[index];
            const double segment_length = (end - start).norm();
            if (!std::isfinite(segment_length)) {
                prefix.clear();
                return false;
            }
            if (segment_length <= duplicate_distance_m) continue;
            const double remaining_length = maximum_length - accumulated_length;
            if (!std::isfinite(remaining_length) || remaining_length < 0.0) {
                truncated = true;
                break;
            }
            if (remaining_length + duplicate_distance_m >= segment_length) {
                prefix.emplace_back(end);
                accumulated_length += segment_length;
                continue;
            }
            if (remaining_length <= duplicate_distance_m) {
                truncated = true;
                break;
            }
            const double fraction = std::clamp(
                    remaining_length / segment_length, 0.0, 1.0);
            prefix.emplace_back(start + static_cast<float>(fraction) * (end - start));
            truncated = true;
            break;
        }
        if (!truncated && prefix.size() < 2U) {
            prefix.clear();
            return false;
        }
        return prefix.size() >= 2U && prefix.back().allFinite();
    }

    inline bool allocateGuideElapsedTimes(const double a_max,
                                           const double v_max,
                                           const double initial_speed,
                                           const Vec3f &start,
                                           const vec_Vec3f &path,
                                           GuideTimeAllocation &allocation) {
        allocation = GuideTimeAllocation{};
        if (!std::isfinite(a_max) || a_max <= 0.0 ||
            !std::isfinite(v_max) || v_max <= 0.0 ||
            !std::isfinite(initial_speed) || initial_speed < 0.0 ||
            !start.allFinite() || path.empty()) {
            return false;
        }

        constexpr double duplicate_distance_m = 1.0e-6;
        Vec3f previous = start;
        std::vector<double> cumulative_distance_m;
        cumulative_distance_m.reserve(path.size());
        allocation.points.reserve(path.size());
        allocation.elapsed_s.reserve(path.size());
        for (const auto &point : path) {
            if (!point.allFinite()) {
                return false;
            }
            const double segment_length_m = (point - previous).norm();
            if (!std::isfinite(segment_length_m)) {
                return false;
            }
            if (segment_length_m <= duplicate_distance_m) {
                previous = point;
                continue;
            }
            allocation.path_length_m += segment_length_m;
            allocation.points.push_back(point);
            cumulative_distance_m.push_back(allocation.path_length_m);
            previous = point;
        }
        if (allocation.points.empty() ||
            !std::isfinite(allocation.path_length_m) ||
            allocation.path_length_m <= duplicate_distance_m) {
            return false;
        }

        double previous_elapsed_s = 0.0;
        for (const double distance_m : cumulative_distance_m) {
            double elapsed_s = std::numeric_limits<double>::quiet_NaN();
            double velocity_mps = std::numeric_limits<double>::quiet_NaN();
            simplePMTimeAllocator(a_max, v_max, initial_speed,
                                  allocation.path_length_m, distance_m,
                                  elapsed_s, velocity_mps);
            if (!std::isfinite(elapsed_s) || !std::isfinite(velocity_mps) ||
                elapsed_s <= previous_elapsed_s) {
                return false;
            }
            allocation.elapsed_s.push_back(elapsed_s);
            previous_elapsed_s = elapsed_s;
        }
        if (allocation.elapsed_s.size() != allocation.points.size()) return false;

        double terminal_elapsed_s = std::numeric_limits<double>::quiet_NaN();
        double terminal_velocity_mps = std::numeric_limits<double>::quiet_NaN();
        simplePMTimeAllocator(a_max, v_max, initial_speed,
                              allocation.path_length_m, allocation.path_length_m,
                              terminal_elapsed_s, terminal_velocity_mps);
        if (!std::isfinite(terminal_elapsed_s) ||
            !std::isfinite(terminal_velocity_mps) || terminal_velocity_mps < 0.0) {
            return false;
        }
        allocation.terminal_velocity_mps = terminal_velocity_mps;
        return true;
    }

    ///============ 2023-06-30: add by yunfan ============///
    double DistancePointEllipse(double e0, double e1, double y0, double y1, double& x0, double& x1);

    double
    DistancePointEllipsoid(double e0, double e1, double e2, double y0, double y1, double y2, double& x0, double& x1,
                           double& x2);


    ///============ 2023-06-13: add by Gene ============///
    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_yrp(const Eigen::Quaternion<Scalar_t>& q_);

    ///============ 2023-06-13: add by Yunfan ============///
    Vec4f translatePlane(const Vec4f& plane, const Vec3f& translation);

    ///============ 2023-05-23: add by Yunfan ============///
    void normalizeNextYaw(const double& last_yaw, double& yaw);

    ///============ 2023-3-12: add by Yunfan ============///
    void convertFlatOutputToAttAndOmg(const Vec3f& p,
                                      const Vec3f& v,
                                      const Vec3f& a,
                                      const Vec3f& j,
                                      const double& yaw,
                                      const double& yaw_dot,
                                      Vec3f& rpy,
                                      Vec3f& omg,
                                      double& aT
    );


    ///============ 2022-12-10: add by Yunfan ============///
    bool pointInsidePolytope(const Vec3f& point, const PolyhedronH& polytope,
                             double margin = 1e-6);

    ///============ 2022-12-5: add by Yunfan ============///
    double pointLineSegmentDistance(const Vec3f& p, const Vec3f& a, const Vec3f& b);

    double computePathLength(const vec_E<Vec3f>& path);

    ///============ 2022-11-18 ====================================================================================
    int inline GetIntersection(float fDst1, float fDst2, Vec3f P1, Vec3f P2, Vec3f& Hit);

    int inline InBox(Vec3f Hit, Vec3f B1, Vec3f B2, const int Axis);

    //The box in this article is Axis-Aligned and so can be defined by only two 3D points:
    // B1 - the smallest values of X, Y, Z
    //        B2 - the largest values of X, Y, Z
    // returns true if line (L1, L2) intersects with the box (B1, B2)
    // returns intersection point in Hit
    int lineIntersectBox(Vec3f L1, Vec3f L2, Vec3f B1, Vec3f B2, Vec3f& Hit);

    Vec3f lineBoxIntersectPoint(const Vec3f& pt, const Vec3f& pos,
                                const Vec3f& box_min, const Vec3f& box_max);

    ///================================================================================================


    Eigen::Matrix3d RotationFromVec3(const Eigen::Vector3d& v);

    // 通过三点获得一个平面
    void FromPointsToPlane(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3,
                           Eigen::Vector4d& hPoly);

    void getFovCheckPlane(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d& fov_planes,
                          std::vector<Eigen::Matrix3d>& fov_pts);

    void GetFovPlanes(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d& fov_planes,
                      std::vector<Eigen::Matrix3d>& fov_pts);


    double findInteriorDist(const Eigen::MatrixX4d& hPoly,
                            Eigen::Vector3d& interior);

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    bool findInterior(const Eigen::MatrixX4d& hPoly,
                      Eigen::Vector3d& interior);

    bool overlap(const Eigen::MatrixX4d& hPoly0,
                 const Eigen::MatrixX4d& hPoly1,
                 const double eps = 1.0e-6);

    struct filterLess {
        bool operator()(const Eigen::Vector3d& l,
                        const Eigen::Vector3d& r) const {
            return l(0) < r(0) ||
            (l(0) == r(0) &&
                (l(1) < r(1) ||
                    (l(1) == r(1) &&
                        l(2) < r(2))));
        }
    };

    void filterVs(const Eigen::Matrix3Xd& rV,
                  const double& epsilon,
                  Eigen::Matrix3Xd& fV);

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    void enumerateVs(const Eigen::MatrixX4d& hPoly,
                     const Eigen::Vector3d& inner,
                     Eigen::Matrix3Xd& vPoly,
                     const double epsilon = 1.0e-6);

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    bool enumerateVs(const Eigen::MatrixX4d& hPoly,
                     Eigen::Matrix3Xd& vPoly,
                     const double epsilon = 1.0e-6);


    template <typename Scalar_t>
    Scalar_t toRad(const Scalar_t& x);

    template <typename Scalar_t>
    Scalar_t toDeg(const Scalar_t& x);

    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 3> rotx(Scalar_t t);

    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 3> roty(Scalar_t t);

    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 3> rotz(Scalar_t t);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr_to_R(const Eigen::DenseBase<Derived>& ypr);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3>
    vec_to_R(const Eigen::MatrixBase<Derived>& v1, const Eigen::MatrixBase<Derived>& v2);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 1> R_to_ypr(const Eigen::DenseBase<Derived>& R);

    template <typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> ypr_to_quaternion(const Eigen::DenseBase<Derived>& ypr);

    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_);

    template <typename Scalar_t>
    Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t>& q);

    template <typename Scalar_t>
    Eigen::Quaternion<Scalar_t> yaw_to_quaternion(Scalar_t yaw);

    template <typename Scalar_t>
    Scalar_t normalize_angle(Scalar_t a);

    template <typename Scalar_t>
    Scalar_t angle_add(Scalar_t a, Scalar_t b);

    template <typename Scalar_t>
    Scalar_t yaw_add(Scalar_t a, Scalar_t b);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> get_skew_symmetric(const Eigen::DenseBase<Derived>& v);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 1> from_skew_symmetric(const Eigen::DenseBase<Derived>& M);
}
