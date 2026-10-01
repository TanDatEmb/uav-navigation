#pragma once

#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/time/timestamp.hpp"

namespace uav::nav::lio {

struct StateEstimate {
  Timestamp time;
  ManifoldState state;
  ManifoldState::Covariance covariance{ManifoldState::Covariance::Identity()};

  [[nodiscard]] bool allFinite() const noexcept {
    return state.allFinite() && covariance.allFinite();
  }
};

}  // namespace uav::nav::lio
