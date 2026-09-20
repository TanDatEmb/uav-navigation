#pragma once

#include "fast_lio_core/common/status.hpp"
#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/geometry/rigid_transform.hpp"
#include "fast_lio_core/initialization/initial_state_prior.hpp"

namespace uav::nav::lio {

class InitialStatePriorApplicator {
 public:
  explicit InitialStatePriorApplicator(RigidTransform base_to_imu);

  [[nodiscard]] Status setGeometry(RigidTransform base_to_imu);
  [[nodiscard]] Status apply(const InitialStatePrior& prior,
                             const ManifoldState& imu_initialized_state,
                             double maximum_full_attitude_tilt_disagreement_rad,
                             ManifoldState& output) const;

 private:
  RigidTransform base_to_imu_;
};

}  // namespace uav::nav::lio
