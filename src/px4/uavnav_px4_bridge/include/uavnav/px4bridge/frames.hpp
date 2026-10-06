#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

// FLU -> FRD conversions for the PX4 bridge (SYSTEM_DESIGN §4.1, F34).
//
// lio_odom is a gravity-aligned FLU world (x forward, y left, z up) with an arbitrary
// heading. PX4's local frame in FRD form (x forward, y right, z down, constant arbitrary
// heading offset from north) is obtained by the proper rotation F = Rx(pi) = diag(1,-1,-1).
// The same F maps the body frame from FLU to FRD.
//
//   vector   v_frd = F v_flu                  -> (x, -y, -z)
//   rotation R_frd = F R_flu F^-1             (world FLU->FRD on the left, body FLU->FRD on the right)
//   quaternion (w, x, y, z) -> (w, x, -y, -z) (F is a rotation, so conjugation by F maps the
//                                              axis n to F n and keeps the angle and w)
//   covariance S_frd = F S F^T
//
// Pure functions, no state, no ROS. The quaternion conversion does NOT change the
// hemisphere (w keeps its sign) and does not normalise; the EV encoder canonicalises.
namespace uavnav::px4bridge {

/// (x, y, z) in FLU -> (x, -y, -z) in FRD. Also valid for FRD -> FLU (F is its own inverse).
inline Eigen::Vector3d flu_to_frd(const Eigen::Vector3d& v) { return {v.x(), -v.y(), -v.z()}; }

/// q_world_body in FLU (body->world, Hamilton) -> the same rotation expressed with FRD world
/// and FRD body axes: (w, x, y, z) -> (w, x, -y, -z). Example: yaw +30 deg about FLU z becomes
/// yaw -30 deg about FRD z. Self-inverse.
inline Eigen::Quaterniond flu_to_frd(const Eigen::Quaterniond& q_world_body_flu) {
  return {q_world_body_flu.w(), q_world_body_flu.x(), -q_world_body_flu.y(), -q_world_body_flu.z()};
}

/// F S F^T with F = diag(1,-1,-1): flips the sign of the (x,y) and (x,z) cross terms, keeps the
/// (y,z) cross term and every diagonal entry. For an orientation covariance this is the
/// small-rotation covariance in body FRD axes. The input is not assumed symmetric.
inline Eigen::Matrix3d flu_to_frd_cov(const Eigen::Matrix3d& cov) {
  const Eigen::Vector3d d{1.0, -1.0, -1.0};
  return d.asDiagonal() * cov * d.asDiagonal();
}

}  // namespace uavnav::px4bridge
