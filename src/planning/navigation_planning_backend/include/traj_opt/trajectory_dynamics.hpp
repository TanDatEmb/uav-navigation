#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <data_structure/base/trajectory.h>
#include <traj_opt/config.hpp>

namespace traj_opt {

struct TrajectoryDynamicReport {
    bool finite{true};
    double first_nonfinite_time_s{std::numeric_limits<double>::quiet_NaN()};
    std::uint32_t nonfinite_mask{0U};
    double maximum_body_rate_rad_s{0.0};
    double minimum_thrust_n{std::numeric_limits<double>::infinity()};
    double maximum_thrust_n{0.0};
};

// planner backend's optimizer penalties are sampled soft costs. This independent final
// gate evaluates the generated polynomial through the same quadrotor flatness
// model and prevents a low aggregate penalty from authorizing a trajectory
// outside the vehicle body-rate or thrust envelope.
inline TrajectoryDynamicReport evaluateTrajectoryDynamics(
        const geometry_utils::Trajectory &trajectory,
        const Config &config,
        const double maximum_sample_period_s = 0.01,
        const geometry_utils::Trajectory *yaw_trajectory = nullptr) {
    TrajectoryDynamicReport report;
    const double duration = trajectory.getTotalDuration();
    if (trajectory.empty() || !std::isfinite(duration) || duration <= 0.0 ||
        !std::isfinite(maximum_sample_period_s) || maximum_sample_period_s <= 0.0) {
        report.finite = false;
        return report;
    }
    if (yaw_trajectory != nullptr &&
        (yaw_trajectory->empty() ||
         yaw_trajectory->getTotalDuration() + 1.0e-6 < duration)) {
        report.finite = false;
        return report;
    }

    const std::size_t intervals = std::max<std::size_t>(
            1U, static_cast<std::size_t>(std::ceil(duration / maximum_sample_period_s)));
    flatness::FlatnessMap flatness = config.quadrotot_flatness;
    const auto evaluate_sample = [&](const double time) {
        if (!std::isfinite(time) || time < 0.0 || time > duration) return false;
        const Eigen::Vector3d velocity = trajectory.getVel(time);
        const Eigen::Vector3d acceleration = trajectory.getAcc(time);
        const Eigen::Vector3d jerk = trajectory.getJer(time);
        double thrust = std::numeric_limits<double>::quiet_NaN();
        Eigen::Vector4d quaternion;
        Eigen::Vector3d body_rate;
        const double yaw = yaw_trajectory == nullptr
                               ? 0.0
                               : yaw_trajectory->getPos(time).x();
        const double yaw_rate = yaw_trajectory == nullptr
                                    ? 0.0
                                    : yaw_trajectory->getVel(time).x();
        flatness.forward(velocity, acceleration, jerk, yaw, yaw_rate,
                         thrust, quaternion, body_rate);
        std::uint32_t nonfinite_mask = 0U;
        if (!velocity.allFinite()) nonfinite_mask |= 1U << 0U;
        if (!acceleration.allFinite()) nonfinite_mask |= 1U << 1U;
        if (!jerk.allFinite()) nonfinite_mask |= 1U << 2U;
        if (!std::isfinite(yaw)) nonfinite_mask |= 1U << 3U;
        if (!std::isfinite(yaw_rate)) nonfinite_mask |= 1U << 4U;
        if (!std::isfinite(thrust)) nonfinite_mask |= 1U << 5U;
        if (!quaternion.allFinite()) nonfinite_mask |= 1U << 6U;
        if (!body_rate.allFinite()) nonfinite_mask |= 1U << 7U;
        if (nonfinite_mask != 0U) {
            report.finite = false;
            if (!std::isfinite(report.first_nonfinite_time_s)) {
                report.first_nonfinite_time_s = time;
                report.nonfinite_mask = nonfinite_mask;
            }
            return false;
        }
        report.maximum_body_rate_rad_s =
                std::max(report.maximum_body_rate_rad_s, body_rate.norm());
        report.minimum_thrust_n = std::min(report.minimum_thrust_n, thrust);
        report.maximum_thrust_n = std::max(report.maximum_thrust_n, thrust);
        return true;
    };
    for (std::size_t sample = 0; sample <= intervals; ++sample) {
        const double time = duration * static_cast<double>(sample) /
                            static_cast<double>(intervals);
        if (!evaluate_sample(time)) {
            report.finite = false;
            return report;
        }
    }
    // Uniform samples need not land on internal MINCO piece boundaries. Add
    // every exact junction explicitly because flatness can peak there even
    // when both neighboring uniform samples are below the limit.
    double boundary_time = 0.0;
    for (int piece = 0; piece + 1 < trajectory.getPieceNum(); ++piece) {
        const double piece_duration = trajectory[piece].getDuration();
        if (!std::isfinite(piece_duration) || piece_duration <= 0.0) {
            report.finite = false;
            return report;
        }
        boundary_time += piece_duration;
        if (!evaluate_sample(boundary_time)) {
            report.finite = false;
            return report;
        }
    }
    return report;
}

inline bool trajectorySatisfiesFlatnessEnvelope(
        const geometry_utils::Trajectory &trajectory,
        const Config &config,
        TrajectoryDynamicReport *output = nullptr,
        const double maximum_sample_period_s = 0.01,
        const geometry_utils::Trajectory *yaw_trajectory = nullptr) {
    const TrajectoryDynamicReport report =
            evaluateTrajectoryDynamics(
                trajectory, config, maximum_sample_period_s, yaw_trajectory);
    if (output != nullptr) {
        *output = report;
    }
    // The dynamic envelope is a hard physical certificate. Objective slack
    // must never widen the limits that authorize a command.
    const double minimum_thrust_n = config.min_acc_thr * config.mass;
    const double maximum_thrust_n = config.max_acc_thr * config.mass;
    return report.finite &&
           report.maximum_body_rate_rad_s <= config.max_omg &&
           report.minimum_thrust_n >= minimum_thrust_n &&
           report.maximum_thrust_n <= maximum_thrust_n;
}

}  // namespace traj_opt
