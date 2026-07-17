// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_IMU_TRAJECTORY_HPP_
#define FAST_LIO_IMU_TRAJECTORY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "fast_lio/commons.hpp"  // SO3d, SE3d, V3D

namespace fast_lio {

/// @brief A bracketed IMU motion segment used for deskew interpolation.
///
/// Each segment covers [t0, t1]. The control inputs (omega_mid_I,
/// acceleration_mid_W) are the midpoint IMU measurement of the two bracket
/// samples, matching the FAST-LIO2 propagation sequence.
///
/// REF-LIO-PRED-001: process model and segment interpolation follow HKUST
/// FAST-LIO2 / IKFoM. Any change requires deskew synthetic tests.
struct ImuMotionSegment {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Segment start time (seconds).
    double t0_s = 0.0;

    /// Segment end time (seconds).
    double t1_s = 0.0;

    /// Body-to-world rotation at t0.
    SO3d R_W_I_t0;

    /// Position in world at t0.
    Eigen::Vector3d p_W_I_t0 = Eigen::Vector3d::Zero();

    /// Velocity in world at t0.
    Eigen::Vector3d v_W_I_t0 = Eigen::Vector3d::Zero();

    /// Bias-corrected angular velocity in IMU body frame at the interval
    /// midpoint (rad/s). Used for rotation interpolation within the segment.
    Eigen::Vector3d omega_mid_I = Eigen::Vector3d::Zero();

    /// World-frame acceleration including gravity at the interval midpoint
    /// (m/s²). Used for position interpolation within the segment.
    Eigen::Vector3d acceleration_mid_W = Eigen::Vector3d::Zero();
};

/// @brief Forward-propagated IMU trajectory spanning one LiDAR scan.
///
/// Contains bracketed motion segments from scan_start to scan_end.
/// Used by LidarDeskewer to compensate point motion distortion.
///
/// Contract:
///   - segments are sorted by ascending t0
///   - segments.front().t0_s <= scan_start_time_s
///   - segments.back().t1_s >= scan_end_time_s
///   - All fields finite
class ImuTrajectory {
   public:
    ImuTrajectory() = default;

    /// Add a segment. Must be called in ascending t0 order.
    void append(const ImuMotionSegment& segment) { segments_.push_back(segment); }

    /// Number of segments.
    std::size_t size() const { return segments_.size(); }

    /// Access segment by index.
    const ImuMotionSegment& operator[](std::size_t i) const { return segments_[i]; }

    /// First segment.
    const ImuMotionSegment& front() const { return segments_.front(); }

    /// Last segment.
    const ImuMotionSegment& back() const { return segments_.back(); }

    /// Check if trajectory covers a given timestamp.
    bool covers(double timestamp_s) const {
        constexpr double kEps = 1e-6;
        return !segments_.empty() &&
               segments_.front().t0_s <= timestamp_s + kEps &&
               segments_.back().t1_s >= timestamp_s - kEps;
    }

    /// Find the segment index containing the given timestamp.
    /// Returns index k such that segments_[k].t0_s <= t < segments_[k].t1_s.
    /// If t < segments_[0].t0_s, returns 0.
    /// If t >= segments_.back().t1_s, returns index of last segment.
    std::size_t findSegment(double timestamp_s) const {
        if (segments_.size() < 2) return 0;
        std::size_t k = 0;
        while (k + 1 < segments_.size() && segments_[k + 1].t0_s <= timestamp_s) {
            ++k;
        }
        return k;
    }

    /// Interpolate IMU pose at a given timestamp.
    /// Uses: R(t) = R_t0 * Exp(omega_mid * dt)
    ///       p(t) = p_t0 + v_t0 * dt + 0.5 * a_mid * dt²
    /// where dt = t - segment.t0_s (clamped to the segment interval).
    ///
    /// @param timestamp_s Query time
    /// @return T_W_I (IMU pose in world at time t)
    SE3d interpolateImuPose(double timestamp_s) const {
        if (segments_.empty()) {
            return SE3d();
        }

        const std::size_t k = findSegment(timestamp_s);
        const auto& seg = segments_[k];
        const double clamped_t = std::max(seg.t0_s, std::min(timestamp_s, seg.t1_s));
        const double dt = clamped_t - seg.t0_s;
        const double interval = seg.t1_s - seg.t0_s;
        if (interval <= 0.0) {
            return SE3d(seg.R_W_I_t0, seg.p_W_I_t0);
        }

        // Rotation: R(t) = R_t0 * Exp(omega_mid * dt)
        const SO3d R_W_I_t = seg.R_W_I_t0 * SO3d::exp(seg.omega_mid_I * dt);

        // Position: p(t) = p_t0 + v_t0 * dt + 0.5 * a_mid * dt²
        const Eigen::Vector3d p_W_I_t =
            seg.p_W_I_t0 + seg.v_W_I_t0 * dt + 0.5 * seg.acceleration_mid_W * dt * dt;

        return SE3d(R_W_I_t, p_W_I_t);
    }

   private:
    std::vector<ImuMotionSegment, Eigen::aligned_allocator<ImuMotionSegment>> segments_;
};

}  // namespace fast_lio

#endif  // FAST_LIO_IMU_TRAJECTORY_HPP_