// Copyright (c) 2026, LeTanDat
// SPDX-License-Identifier: BSD-3-Clause

#ifndef FAST_LIO_IMU_TRAJECTORY_HPP_
#define FAST_LIO_IMU_TRAJECTORY_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "fast_lio/commons.hpp"  // SO3d, SE3d, V3D

namespace fast_lio {

/// @brief A single IMU pose knot in the forward-propagated trajectory.
///
/// Stored at each IMU timestamp during forward propagation. Used by
/// LidarDeskewer to interpolate pose at arbitrary point timestamps.
///
/// All fields are in the LIO world frame (z-up, gravity = [0, 0, -g]).
struct ImuTrajectoryKnot {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Absolute timestamp (seconds, same domain as scan_start/end).
    double timestamp_s = 0.0;

    /// Body-to-world rotation at this timestamp.
    SO3d R_W_I;

    /// Position in world at this timestamp.
    Eigen::Vector3d p_W_I = Eigen::Vector3d::Zero();

    /// Velocity in world at this timestamp.
    Eigen::Vector3d v_W_I = Eigen::Vector3d::Zero();

    /// Bias-corrected angular velocity in IMU body frame (rad/s).
    /// Used for rotation interpolation within the interval.
    Eigen::Vector3d omega_I = Eigen::Vector3d::Zero();

    /// World-frame acceleration including gravity (m/s²).
    /// a_W = R_W_I * (f_measured - b_a) + gravity_W
    /// Used for position interpolation within the interval.
    Eigen::Vector3d a_W = Eigen::Vector3d::Zero();
};

/// @brief Forward-propagated IMU trajectory spanning one LiDAR scan.
///
/// Contains pose knots at each IMU timestamp from scan_start to scan_end.
/// Used by LidarDeskewer to compensate point motion distortion.
///
/// Contract:
///   - knots are sorted by ascending timestamp
///   - knots.front().timestamp_s <= scan_start_time_s
///   - knots.back().timestamp_s >= scan_end_time_s
///   - All fields finite
class ImuTrajectory {
   public:
    ImuTrajectory() = default;

    /// Add a knot. Must be called in ascending timestamp order.
    void append(const ImuTrajectoryKnot& knot) { knots_.push_back(knot); }

    /// Number of knots.
    std::size_t size() const { return knots_.size(); }

    /// Access knot by index.
    const ImuTrajectoryKnot& operator[](std::size_t i) const { return knots_[i]; }

    /// First knot.
    const ImuTrajectoryKnot& front() const { return knots_.front(); }

    /// Last knot.
    const ImuTrajectoryKnot& back() const { return knots_.back(); }

    /// Check if trajectory covers a given timestamp.
    bool covers(double timestamp_s) const {
        constexpr double kEps = 1e-6;
        return !knots_.empty() &&
               knots_.front().timestamp_s <= timestamp_s + kEps &&
               knots_.back().timestamp_s >= timestamp_s - kEps;
    }

    /// Find the interval index [k, k+1] containing the given timestamp.
    /// Returns index k such that knots_[k].timestamp_s <= t < knots_[k+1].timestamp_s.
    /// If t < knots_[0], returns 0 (before first knot).
    /// If t >= knots_.back(), returns size()-2 (after last knot, may be invalid).
    std::size_t findInterval(double timestamp_s) const {
        if (knots_.size() < 2) return 0;
        std::size_t k = 0;
        while (k + 1 < knots_.size() && knots_[k + 1].timestamp_s < timestamp_s) {
            ++k;
        }
        return k;
    }

    /// Interpolate IMU pose at a given timestamp.
    /// Uses: R(t) = R_k * Exp(omega_k * dt)
    ///       p(t) = p_k + v_k * dt + 0.5 * a_k * dt²
    /// where dt = t - knot[k].timestamp_s.
    ///
    /// @param timestamp_s Query time
    /// @return T_W_I (IMU pose in world at time t)
    SE3d interpolateImuPose(double timestamp_s) const {
        if (knots_.empty()) {
            return SE3d();
        }

        std::size_t k = findInterval(timestamp_s);
        if (k >= knots_.size() - 1) {
            // At or after last knot — use last knot pose
            const auto& last = knots_.back();
            return SE3d(last.R_W_I, last.p_W_I);
        }

        const auto& head = knots_[k];
        const auto& tail = knots_[k + 1];

        const double dt = timestamp_s - head.timestamp_s;
        const double interval = tail.timestamp_s - head.timestamp_s;
        if (interval <= 0.0) {
            return SE3d(head.R_W_I, head.p_W_I);
        }

        // Rotation: R(t) = R_k * Exp(omega * dt)
        const SO3d R_W_I_t = head.R_W_I * SO3d::exp(head.omega_I * dt);

        // Position: p(t) = p_k + v_k * dt + 0.5 * a_k * dt²
        const Eigen::Vector3d p_W_I_t =
            head.p_W_I + head.v_W_I * dt + 0.5 * head.a_W * dt * dt;

        return SE3d(R_W_I_t, p_W_I_t);
    }

   private:
    std::vector<ImuTrajectoryKnot, Eigen::aligned_allocator<ImuTrajectoryKnot>> knots_;
};

}  // namespace fast_lio

#endif  // FAST_LIO_IMU_TRAJECTORY_HPP_