#pragma once

#include <cmath>
#include <numbers>

// Angle helpers shared by the estimator, the PX4 bridge and the nodes (no Eigen, no ROS).
namespace uavnav::angles {

/// Shortest-arc representative of `a` in [-pi, pi]: std::remainder(a, 2*pi), which is exact for every
/// finite input. NaN stays NaN and so do +-infinity (std::remainder(inf, y) is NaN), so a caller that
/// must refuse garbage checks std::isfinite on the result. The boundary is a tie (a = pi + 2*pi*k)
/// that std::remainder resolves to the even multiple: pi and -pi stay, 3*pi maps to -pi and -3*pi to pi.
constexpr double wrap_pi(double a) noexcept { return std::remainder(a, 2.0 * std::numbers::pi); }

/// Yaw of a rotation matrix about the down/up axis, from its first column: atan2(R(1,0), R(0,0)).
/// Result in [-pi, pi]; NaN in, NaN out; (0, 0) gives 0.
inline double yaw_from_rotation(double r10, double r00) noexcept { return std::atan2(r10, r00); }

}  // namespace uavnav::angles
