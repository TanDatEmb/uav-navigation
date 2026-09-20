#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/geometry/frame.hpp"

namespace uav::nav::lio {

// RigidTransform(target, source, q_target_source, t_target_source) represents
// ^target T_source: it converts point coordinates from source into target.
class RigidTransform {
 public:
  RigidTransform(FrameId target_frame, FrameId source_frame, Eigen::Quaterniond q_target_source,
                 Eigen::Vector3d t_target_source);

  [[nodiscard]] static Result<RigidTransform> Create(FrameId target_frame, FrameId source_frame,
                                                     const Eigen::Quaterniond& q_target_source,
                                                     const Eigen::Vector3d& t_target_source);

  [[nodiscard]] static RigidTransform Identity(const FrameId& frame);

  [[nodiscard]] const FrameId& targetFrame() const noexcept;
  [[nodiscard]] const FrameId& sourceFrame() const noexcept;
  [[nodiscard]] const Eigen::Quaterniond& rotation() const noexcept;
  [[nodiscard]] const Eigen::Vector3d& translation() const noexcept;
  [[nodiscard]] bool allFinite() const noexcept;

  template <typename Derived>
  [[nodiscard]] Eigen::Matrix<typename Derived::Scalar, 3, 1> apply(
      const Eigen::MatrixBase<Derived>& point_source) const noexcept {
    using Scalar = typename Derived::Scalar;
    return q_target_source_.toRotationMatrix().template cast<Scalar>() * point_source +
           t_target_source_.template cast<Scalar>();
  }
  [[nodiscard]] Result<RigidTransform> inverse() const;

  // For this = ^A T_B and rhs = ^B T_C, returns ^A T_C.
  [[nodiscard]] Result<RigidTransform> compose(const RigidTransform& rhs) const;

 private:
  FrameId target_frame_;
  FrameId source_frame_;
  Eigen::Quaterniond q_target_source_;
  Eigen::Vector3d t_target_source_;
};

}  // namespace uav::nav::lio
