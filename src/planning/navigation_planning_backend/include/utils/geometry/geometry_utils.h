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
#include <array>
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

    static void simplePMTimeAllocator(const double &max_acceleration_mps2,
                                      const double &max_velocity_mps,
                                      const double &initial_velocity_mps,
                                      const double &total_distance_m,
                                      const double &current_distance_m,
                                      double &elapsed_s,
                                      double &velocity_mps) {
        if (!std::isfinite(max_acceleration_mps2) ||
            max_acceleration_mps2 <= 0.0 ||
            !std::isfinite(max_velocity_mps) || max_velocity_mps <= 0.0 ||
            !std::isfinite(initial_velocity_mps) || initial_velocity_mps < 0.0 ||
            initial_velocity_mps > max_velocity_mps ||
            !std::isfinite(total_distance_m) || total_distance_m <= 0.0 ||
            !std::isfinite(current_distance_m) || current_distance_m < 0.0 ||
            current_distance_m > total_distance_m) {
            elapsed_s = std::numeric_limits<double>::quiet_NaN();
            velocity_mps = std::numeric_limits<double>::quiet_NaN();
            return;
        }
        const double acceleration = max_acceleration_mps2;
        const double initial_speed_squared =
            initial_velocity_mps * initial_velocity_mps;
        const double maximum_speed_squared =
            max_velocity_mps * max_velocity_mps;
        const double stopping_distance =
            initial_speed_squared / (2.0 * acceleration);
        const double acceleration_distance_to_maximum =
            (maximum_speed_squared - initial_speed_squared) /
            (2.0 * acceleration);
        const double deceleration_distance_from_maximum =
            maximum_speed_squared / (2.0 * acceleration);
        if (!std::isfinite(stopping_distance) ||
            !std::isfinite(acceleration_distance_to_maximum) ||
            !std::isfinite(deceleration_distance_from_maximum) ||
            total_distance_m + 1.0e-12 * std::max(1.0, total_distance_m) <
                stopping_distance) {
            elapsed_s = std::numeric_limits<double>::quiet_NaN();
            velocity_mps = std::numeric_limits<double>::quiet_NaN();
            return;
        }

        const bool reaches_maximum_speed =
            total_distance_m >= acceleration_distance_to_maximum +
                                    deceleration_distance_from_maximum;
        const double peak_speed = reaches_maximum_speed
            ? max_velocity_mps
            : std::sqrt(acceleration * total_distance_m +
                        0.5 * initial_speed_squared);
        if (!std::isfinite(peak_speed) || peak_speed < initial_velocity_mps ||
            peak_speed > max_velocity_mps) {
            elapsed_s = std::numeric_limits<double>::quiet_NaN();
            velocity_mps = std::numeric_limits<double>::quiet_NaN();
            return;
        }
        const double acceleration_duration =
            (peak_speed - initial_velocity_mps) / acceleration;
        const double acceleration_distance =
            (peak_speed * peak_speed - initial_speed_squared) /
            (2.0 * acceleration);
        const double deceleration_distance =
            peak_speed * peak_speed / (2.0 * acceleration);
        const double cruise_distance = std::max(
            0.0, total_distance_m - acceleration_distance -
                     deceleration_distance);
        const double cruise_duration = cruise_distance / peak_speed;
        if (!std::isfinite(acceleration_duration) ||
            !std::isfinite(acceleration_distance) ||
            !std::isfinite(deceleration_distance) ||
            !std::isfinite(cruise_duration)) {
            elapsed_s = std::numeric_limits<double>::quiet_NaN();
            velocity_mps = std::numeric_limits<double>::quiet_NaN();
            return;
        }

        if (current_distance_m <= acceleration_distance) {
            velocity_mps = std::sqrt(
                initial_speed_squared + 2.0 * acceleration * current_distance_m);
            elapsed_s = (velocity_mps - initial_velocity_mps) / acceleration;
            return;
        }
        if (current_distance_m <= acceleration_distance + cruise_distance) {
            velocity_mps = peak_speed;
            elapsed_s = acceleration_duration +
                (current_distance_m - acceleration_distance) / peak_speed;
            return;
        }
        const double deceleration_progress =
            current_distance_m - acceleration_distance - cruise_distance;
        const double remaining_speed_squared = std::max(
            0.0, peak_speed * peak_speed -
                     2.0 * acceleration * deceleration_progress);
        velocity_mps = std::sqrt(remaining_speed_squared);
        elapsed_s = acceleration_duration + cruise_duration +
            (peak_speed - velocity_mps) / acceleration;
    }

    struct GuideTimeAllocation {
        vec_Vec3f points;
        std::vector<double> elapsed_s;
        double path_length_m{0.0};
        double terminal_velocity_mps{0.0};
    };

    struct JerkLimitedContinuationProfile {
        double initial_velocity_mps{0.0};
        double terminal_velocity_mps{0.0};
        double maximum_jerk_mps3{0.0};
        double jerk_phase_s{0.0};
        double constant_acceleration_phase_s{0.0};
        double acceleration_duration_s{0.0};
        double acceleration_distance_m{0.0};
        double cruise_duration_s{0.0};
        double total_duration_s{0.0};

        double distanceAtTime(const double time_s) const noexcept {
            if (!std::isfinite(time_s) || time_s <= 0.0) return 0.0;
            const double t = std::min(time_s, total_duration_s);
            const double jerk = maximum_jerk_mps3;
            const double t_jerk = jerk_phase_s;
            const double peak_acceleration = jerk * t_jerk;
            if (t <= t_jerk) {
                return initial_velocity_mps * t + jerk * t * t * t / 6.0;
            }
            const double first_distance = initial_velocity_mps * t_jerk +
                jerk * t_jerk * t_jerk * t_jerk / 6.0;
            const double first_velocity = initial_velocity_mps +
                0.5 * jerk * t_jerk * t_jerk;
            if (t <= t_jerk + constant_acceleration_phase_s) {
                const double dt = t - t_jerk;
                return first_distance + first_velocity * dt +
                    0.5 * peak_acceleration * dt * dt;
            }
            const double second_distance = first_distance +
                first_velocity * constant_acceleration_phase_s +
                0.5 * peak_acceleration * constant_acceleration_phase_s *
                    constant_acceleration_phase_s;
            const double second_velocity = first_velocity +
                peak_acceleration * constant_acceleration_phase_s;
            if (t <= acceleration_duration_s) {
                const double dt = t - t_jerk - constant_acceleration_phase_s;
                return second_distance + second_velocity * dt +
                    0.5 * peak_acceleration * dt * dt -
                    jerk * dt * dt * dt / 6.0;
            }
            return acceleration_distance_m +
                terminal_velocity_mps * (t - acceleration_duration_s);
        }
    };

    inline bool makeJerkLimitedContinuationProfile(
            const double path_length_m,
            const double initial_velocity_mps,
            const double maximum_velocity_mps,
            const double maximum_acceleration_mps2,
            const double maximum_jerk_mps3,
            JerkLimitedContinuationProfile &profile) noexcept {
        profile = JerkLimitedContinuationProfile{};
        if (!std::isfinite(path_length_m) || path_length_m <= 0.0 ||
            !std::isfinite(initial_velocity_mps) || initial_velocity_mps < 0.0 ||
            !std::isfinite(maximum_velocity_mps) || maximum_velocity_mps <= 0.0 ||
            initial_velocity_mps > maximum_velocity_mps + 1.0e-9 ||
            !std::isfinite(maximum_acceleration_mps2) ||
            maximum_acceleration_mps2 <= 0.0 ||
            !std::isfinite(maximum_jerk_mps3) || maximum_jerk_mps3 <= 0.0) {
            return false;
        }
        const double bounded_initial_velocity =
            std::min(initial_velocity_mps, maximum_velocity_mps);
        const auto acceleration_envelope = [&](const double terminal_velocity) {
            const double delta_velocity =
                std::max(0.0, terminal_velocity - bounded_initial_velocity);
            const double full_acceleration_delta =
                maximum_acceleration_mps2 * maximum_acceleration_mps2 /
                maximum_jerk_mps3;
            const double jerk_phase = delta_velocity <= full_acceleration_delta
                ? std::sqrt(delta_velocity / maximum_jerk_mps3)
                : maximum_acceleration_mps2 / maximum_jerk_mps3;
            const double constant_acceleration_phase =
                delta_velocity <= full_acceleration_delta
                ? 0.0
                : delta_velocity / maximum_acceleration_mps2 - jerk_phase;
            const double duration = 2.0 * jerk_phase + constant_acceleration_phase;
            const double distance =
                0.5 * (bounded_initial_velocity + terminal_velocity) * duration;
            return std::array<double, 4>{
                jerk_phase, constant_acceleration_phase, duration, distance};
        };

        double terminal_velocity = maximum_velocity_mps;
        auto envelope = acceleration_envelope(terminal_velocity);
        if (!std::all_of(envelope.begin(), envelope.end(),
                         [](const double value) { return std::isfinite(value); })) {
            return false;
        }
        if (envelope[3] > path_length_m) {
            double lower = bounded_initial_velocity;
            double upper = maximum_velocity_mps;
            for (int iteration = 0; iteration < 80; ++iteration) {
                const double middle = 0.5 * (lower + upper);
                const auto middle_envelope = acceleration_envelope(middle);
                if (!std::isfinite(middle_envelope[3])) return false;
                if (middle_envelope[3] <= path_length_m) {
                    lower = middle;
                } else {
                    upper = middle;
                }
            }
            terminal_velocity = lower;
            envelope = acceleration_envelope(terminal_velocity);
        }
        if (!std::isfinite(terminal_velocity) || terminal_velocity <= 0.0 ||
            !std::isfinite(envelope[3]) || envelope[3] > path_length_m + 1.0e-9) {
            return false;
        }
        const double cruise_distance = std::max(0.0, path_length_m - envelope[3]);
        const double cruise_duration = cruise_distance / terminal_velocity;
        const double total_duration = envelope[2] + cruise_duration;
        if (!std::isfinite(cruise_duration) || cruise_duration < 0.0 ||
            !std::isfinite(total_duration) || total_duration <= 0.0) {
            return false;
        }
        profile.initial_velocity_mps = bounded_initial_velocity;
        profile.terminal_velocity_mps = terminal_velocity;
        profile.maximum_jerk_mps3 = maximum_jerk_mps3;
        profile.jerk_phase_s = envelope[0];
        profile.constant_acceleration_phase_s = envelope[1];
        profile.acceleration_duration_s = envelope[2];
        profile.acceleration_distance_m = envelope[3];
        profile.cruise_duration_s = cruise_duration;
        profile.total_duration_s = total_duration;
        const double represented_distance = profile.distanceAtTime(total_duration);
        return std::isfinite(represented_distance) &&
            std::abs(represented_distance - path_length_m) <=
                1.0e-9 * std::max(1.0, path_length_m);
    }

    // Allocate a non-stopping frontier profile. Acceleration starts and ends
    // at zero, is ramped by the declared jerk limit, and may continue at the
    // declared cruise speed. This is distinct from allocateGuideElapsedTimes,
    // whose symmetric accelerate/decelerate profile is reserved for terminal
    // mission goals.
    inline bool allocateGuideContinuationElapsedTimes(
            const double max_acceleration_mps2,
            const double max_jerk_mps3,
            const double max_velocity_mps,
            const double initial_velocity_mps,
            const Vec3f &start,
            const vec_Vec3f &path,
            GuideTimeAllocation &allocation) {
        allocation = GuideTimeAllocation{};
        if (!start.allFinite() || path.empty()) return false;
        constexpr double duplicate_distance_m = 1.0e-6;
        Vec3f previous = start;
        std::vector<double> cumulative_distance_m;
        cumulative_distance_m.reserve(path.size());
        allocation.points.reserve(path.size());
        allocation.elapsed_s.reserve(path.size());
        for (const auto &point : path) {
            if (!point.allFinite()) return false;
            const double segment_length_m = (point - previous).norm();
            if (!std::isfinite(segment_length_m)) return false;
            if (segment_length_m > duplicate_distance_m) {
                allocation.path_length_m += segment_length_m;
                if (!std::isfinite(allocation.path_length_m)) return false;
                allocation.points.push_back(point);
                cumulative_distance_m.push_back(allocation.path_length_m);
            }
            previous = point;
        }
        if (allocation.points.empty() ||
            allocation.path_length_m <= duplicate_distance_m) {
            return false;
        }
        JerkLimitedContinuationProfile profile;
        if (!makeJerkLimitedContinuationProfile(
                allocation.path_length_m, initial_velocity_mps,
                max_velocity_mps, max_acceleration_mps2,
                max_jerk_mps3, profile)) {
            return false;
        }
        double previous_elapsed_s = 0.0;
        for (const double distance_m : cumulative_distance_m) {
            double elapsed_s = profile.total_duration_s;
            if (distance_m < allocation.path_length_m) {
                double lower = previous_elapsed_s;
                double upper = profile.total_duration_s;
                for (int iteration = 0; iteration < 80; ++iteration) {
                    const double middle = 0.5 * (lower + upper);
                    if (profile.distanceAtTime(middle) < distance_m) {
                        lower = middle;
                    } else {
                        upper = middle;
                    }
                }
                elapsed_s = 0.5 * (lower + upper);
            }
            if (!std::isfinite(elapsed_s) || elapsed_s <= previous_elapsed_s) {
                return false;
            }
            allocation.elapsed_s.push_back(elapsed_s);
            previous_elapsed_s = elapsed_s;
        }
        allocation.terminal_velocity_mps = profile.terminal_velocity_mps;
        return allocation.elapsed_s.size() == allocation.points.size();
    }

    // A remote mission goal is a direction/progress contract, not a request
    // to expand A* beyond the route that can be certified and executed in the
    // current visibility window. Bound graph expansion before search; the
    // returned path is still truncated and re-certified by the caller.
    inline double localRouteSearchHorizon(
            const double remaining_planning_horizon,
            const double visibility_horizon) {
        if (!std::isfinite(remaining_planning_horizon) ||
            !std::isfinite(visibility_horizon) ||
            remaining_planning_horizon <= 0.0 || visibility_horizon <= 0.0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return std::min(remaining_planning_horizon, visibility_horizon);
    }

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

    inline bool allocateGuideElapsedTimes(const double max_acceleration_mps2,
                                           const double max_velocity_mps,
                                           const double initial_velocity_mps,
                                           const Vec3f &start,
                                           const vec_Vec3f &path,
                                           GuideTimeAllocation &allocation) {
        allocation = GuideTimeAllocation{};
        if (!std::isfinite(max_acceleration_mps2) || max_acceleration_mps2 <= 0.0 ||
            !std::isfinite(max_velocity_mps) || max_velocity_mps <= 0.0 ||
            !std::isfinite(initial_velocity_mps) || initial_velocity_mps < 0.0 ||
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
            simplePMTimeAllocator(max_acceleration_mps2, max_velocity_mps,
                                  initial_velocity_mps,
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
        simplePMTimeAllocator(max_acceleration_mps2, max_velocity_mps,
                              initial_velocity_mps,
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
