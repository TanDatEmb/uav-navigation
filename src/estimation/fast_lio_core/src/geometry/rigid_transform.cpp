#include "fast_lio_core/geometry/rigid_transform.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace uav::nav::lio {
namespace {

[[nodiscard]] bool quaternionFinite(const Eigen::Quaterniond& quaternion) {
  return quaternion.coeffs().allFinite();
}

[[nodiscard]] Status validateTransform(const FrameId& target_frame, const FrameId& source_frame,
                                       const Eigen::Quaterniond& q_target_source,
                                       const Eigen::Vector3d& t_target_source) {
  if (target_frame.empty() || source_frame.empty()) {
    return Status(StatusCode::kInvalidArgument, "RigidTransform frame ids must not be empty");
  }
  if (!quaternionFinite(q_target_source) || !t_target_source.allFinite()) {
    return Status(StatusCode::kInvalidArgument, "RigidTransform coefficients must be finite");
  }
  if (!std::isfinite(q_target_source.squaredNorm()) ||
      q_target_source.squaredNorm() <= 1e-24) {
    return Status(StatusCode::kInvalidArgument,
                  "RigidTransform quaternion must have non-zero norm");
  }
  return Status::Ok();
}

}  // namespace

RigidTransform::RigidTransform(FrameId target_frame, FrameId source_frame,
                               Eigen::Quaterniond q_target_source, Eigen::Vector3d t_target_source)
    : target_frame_(std::move(target_frame)),
      source_frame_(std::move(source_frame)),
      q_target_source_(std::move(q_target_source)),
      t_target_source_(std::move(t_target_source)) {
  const Status status =
      validateTransform(target_frame_, source_frame_, q_target_source_, t_target_source_);
  if (!status.ok()) {
    throw std::invalid_argument(status.message());
  }
  q_target_source_.normalize();
}

Result<RigidTransform> RigidTransform::Create(FrameId target_frame, FrameId source_frame,
                                              const Eigen::Quaterniond& q_target_source,
                                              const Eigen::Vector3d& t_target_source) {
  const Status status =
      validateTransform(target_frame, source_frame, q_target_source, t_target_source);
  if (!status.ok()) {
    return status;
  }
  return RigidTransform(std::move(target_frame), std::move(source_frame), q_target_source,
                        t_target_source);
}

RigidTransform RigidTransform::Identity(const FrameId& frame) {
  return RigidTransform(frame, frame, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
}

const FrameId& RigidTransform::targetFrame() const noexcept { return target_frame_; }

const FrameId& RigidTransform::sourceFrame() const noexcept { return source_frame_; }

const Eigen::Quaterniond& RigidTransform::rotation() const noexcept { return q_target_source_; }

const Eigen::Vector3d& RigidTransform::translation() const noexcept { return t_target_source_; }

bool RigidTransform::allFinite() const noexcept {
  return quaternionFinite(q_target_source_) &&
         std::isfinite(q_target_source_.squaredNorm()) &&
         q_target_source_.squaredNorm() > 1e-24 && t_target_source_.allFinite();
}

Result<RigidTransform> RigidTransform::inverse() const {
  const Eigen::Quaterniond q_source_target = q_target_source_.conjugate();
  const Eigen::Vector3d t_source_target = -(q_source_target * t_target_source_);
  if (!q_source_target.coeffs().allFinite() || !t_source_target.allFinite()) {
    return Status(StatusCode::kNumericalFailure,
                  "RigidTransform inverse is not finite");
  }
  return Create(source_frame_, target_frame_, q_source_target, t_source_target);
}

Result<RigidTransform> RigidTransform::compose(const RigidTransform& rhs) const {
  if (source_frame_ != rhs.target_frame_) {
    return Status(StatusCode::kFrameMismatch, "Cannot compose transforms: left source frame '" +
                                                  std::string(source_frame_.name()) +
                                                  "' does not match right target frame '" +
                                                  std::string(rhs.target_frame_.name()) + "'");
  }
  const Eigen::Quaterniond rotation = q_target_source_ * rhs.q_target_source_;
  const Eigen::Vector3d translation =
      q_target_source_ * rhs.t_target_source_ + t_target_source_;
  if (!rotation.coeffs().allFinite() || !translation.allFinite()) {
    return Status(StatusCode::kNumericalFailure,
                  "RigidTransform composition is not finite");
  }
  return Create(target_frame_, rhs.source_frame_, rotation, translation);
}

}  // namespace uav::nav::lio
