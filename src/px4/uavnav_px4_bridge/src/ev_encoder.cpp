#include "uavnav/px4bridge/ev_encoder.hpp"

#include <cmath>
#include <expected>
#include <limits>

#include "uavnav/px4bridge/frames.hpp"

namespace uavnav::px4bridge {
namespace {

constexpr std::uint8_t kMaxQuality = 100;
constexpr double kFloatMax = static_cast<double>(std::numeric_limits<float>::max());
/// |norm - 1| above this means the quaternion is not a rotation (zero, runaway or garbage).
constexpr double kQuaternionNormTolerance = 1e-3;

/// True when the double is finite and fits a float32 without overflow (the cast is then defined).
bool fits_float(double x) { return std::isfinite(x) && std::fabs(x) <= kFloatMax; }

template <class Vec>
bool all_fit_float(const Vec& v) {
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (!fits_float(v[i])) return false;
  }
  return true;
}

std::array<float, 3> to_f3(const Eigen::Vector3d& v) {
  return {static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())};
}

/// Variances taken from the diagonal of a covariance block. The FLU->FRD flip D S D^T (D = diag(1,-1,-1))
/// only changes the signs of off-diagonal entries, so a diagonal is the same in FLU and FRD axes and no
/// explicit flip is applied (flu_to_frd_cov is not needed here).
/// Returns false unless every entry is finite, representable and > 0 as a float32.
bool diagonal_variances(const Eigen::Vector3d& diag, std::array<float, 3>& out) {
  for (int i = 0; i < 3; ++i) {
    if (!fits_float(diag[i]) || !(diag[i] > 0.0)) return false;
    out[static_cast<std::size_t>(i)] = static_cast<float>(diag[i]);
    if (!(out[static_cast<std::size_t>(i)] > 0.0F)) return false;  // underflowed to zero in float32
  }
  return true;
}

}  // namespace

Result<EvSample, EvReason> encode_ev(const EvInput& in, ClockMode mode) {
  if (!in.tracking) return std::unexpected(EvReason::kNotTracking);

  const auto px4_time = to_px4(in.t, mode);
  if (!px4_time) return std::unexpected(EvReason::kTime);

  // Position, velocity, quaternion components must all be finite and representable.
  const Eigen::Vector4d qv{in.q.w(), in.q.x(), in.q.y(), in.q.z()};
  if (!all_fit_float(in.p_m) || !all_fit_float(in.v_mps) || !all_fit_float(qv)) {
    return std::unexpected(EvReason::kNonFiniteState);
  }
  const double qn = qv.norm();
  if (!(std::fabs(qn - 1.0) <= kQuaternionNormTolerance)) return std::unexpected(EvReason::kNonFiniteState);

  const Eigen::Quaterniond q_flu{qv[0] / qn, qv[1] / qn, qv[2] / qn, qv[3] / qn};

  // Orientation: the producer's rotation block is a small-rotation covariance in lio_odom (world) axes,
  // VehicleOdometry.orientation_variance is in body axes: S_body = R^T S_world R, R = body->world.
  // The rotation needs the off-diagonals, so the whole block must be finite. It is symmetrised first so a
  // slightly asymmetric input gives a deterministic result.
  const Eigen::Matrix3d orient_world = in.pose_cov.block<3, 3>(3, 3);
  if (!orient_world.allFinite()) return std::unexpected(EvReason::kBadCovariance);
  const Eigen::Matrix3d orient_sym = 0.5 * (orient_world + orient_world.transpose());
  const Eigen::Matrix3d r_world_body = q_flu.toRotationMatrix();
  const Eigen::Matrix3d orient_body = r_world_body.transpose() * orient_sym * r_world_body;

  EvSample s{};
  if (!diagonal_variances(in.pose_cov.diagonal().head<3>(), s.position_variance) ||
      !diagonal_variances(orient_body.diagonal(), s.orientation_variance) ||
      !diagonal_variances(in.vel_cov.diagonal(), s.velocity_variance)) {
    return std::unexpected(EvReason::kBadCovariance);
  }

  Eigen::Quaterniond q = flu_to_frd(q_flu);
  if (q.w() < 0.0) q.coeffs() *= -1.0;  // canonical hemisphere: same rotation, w >= 0

  s.timestamp_sample_us = to_px4_us(*px4_time);
  s.pose_frame = kPoseFrameFrd;
  s.position = to_f3(flu_to_frd(in.p_m));
  s.q_wxyz = {static_cast<float>(q.w()), static_cast<float>(q.x()), static_cast<float>(q.y()),
              static_cast<float>(q.z())};
  s.velocity_frame = kVelocityFrameFrd;
  s.velocity = to_f3(flu_to_frd(in.v_mps));
  s.reset_counter = static_cast<std::uint8_t>(in.epoch % 256U);
  // D28 range is 0..100; anything above is out of contract and becomes 0, "invalid" in VehicleOdometry.
  s.quality = static_cast<std::int8_t>(in.quality <= kMaxQuality ? in.quality : 0);
  return s;
}

}  // namespace uavnav::px4bridge
