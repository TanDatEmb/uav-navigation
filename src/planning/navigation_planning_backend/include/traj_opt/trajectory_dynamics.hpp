#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <data_structure/base/trajectory.h>
#include <traj_opt/config.hpp>

namespace traj_opt {

struct TrajectoryDynamicReport {
    bool finite{true};
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
    for (std::size_t sample = 0; sample <= intervals; ++sample) {
        const double time = duration * static_cast<double>(sample) /
                            static_cast<double>(intervals);
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
        if (!velocity.allFinite() || !acceleration.allFinite() || !jerk.allFinite() ||
            !std::isfinite(yaw) || !std::isfinite(yaw_rate) ||
            !std::isfinite(thrust) || !quaternion.allFinite() || !body_rate.allFinite()) {
            report.finite = false;
            return report;
        }
        report.maximum_body_rate_rad_s =
                std::max(report.maximum_body_rate_rad_s, body_rate.norm());
        report.minimum_thrust_n = std::min(report.minimum_thrust_n, thrust);
        report.maximum_thrust_n = std::max(report.maximum_thrust_n, thrust);
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
    const double margin = 1.0 + config.penna_margin;
    const double minimum_thrust_n = config.min_acc_thr * config.mass / margin;
    const double maximum_thrust_n = config.max_acc_thr * config.mass * margin;
    return report.finite &&
           report.maximum_body_rate_rad_s <= config.max_omg * margin &&
           report.minimum_thrust_n >= minimum_thrust_n &&
           report.maximum_thrust_n <= maximum_thrust_n;
}

}  // namespace traj_opt
