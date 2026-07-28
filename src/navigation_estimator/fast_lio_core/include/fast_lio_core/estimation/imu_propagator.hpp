#pragma once

#include <cstdint>
#include <span>

#include "fast_lio_core/common/result.hpp"
#include "fast_lio_core/estimation/imu_trajectory.hpp"
#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/estimation/process_model.hpp"
#include "fast_lio_core/sensor/imu_sample.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

struct ImuPropagatorConfig {
  std::int64_t maximum_integration_step_ns{20'000'000};
  ImuNoise noise;
};

class ImuPropagator {
 public:
  explicit ImuPropagator(ImuPropagatorConfig config = {});

  // Samples must bracket [start_time, end_time]. The nominal state and its
  // 23-DoF covariance are advanced in-place; returned trajectory includes both
  // interval boundaries and is suitable for per-point deskew.
  [[nodiscard]] Result<ImuTrajectory> propagate(ManifoldState& state,
                                                ManifoldState::Covariance& covariance,
                                                std::span<const ImuSample> samples,
                                                const Timestamp& start_time,
                                                const Timestamp& end_time) const;

 private:
  ImuPropagatorConfig config_;
};

}  // namespace uav::nav::lio
