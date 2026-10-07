#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <optional>
#include <vector>

#include "fast_lio_core/estimation/manifold_state.hpp"
#include "fast_lio_core/estimation/state_estimate.hpp"
#include "fast_lio_core/initialization/initial_state_prior_applicator.hpp"
#include "fast_lio_core/navigation/base_link_covariance_projector.hpp"
#include "fast_lio_core/navigation/base_link_state_converter.hpp"
#include "fast_lio_core/sensor/imu_sample.hpp"
#include "fast_lio_core/sensor/lidar_scan.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/types.hpp"

// Private to backend.cpp (not installed): the stateless fast_lio_core adapters of LioBackend, moved from the
// S1a facade (estimator.cpp). No events, no members: LioBackend::Impl owns the state and decides.
namespace uavnav::lio::backend_math {

namespace fl = uav::nav::lio;

inline fl::Timestamp stamp(time::SensorTime t) { return fl::Timestamp(t.ns); }

EstimatorSnapshot snapshot(const fl::ManifoldState& s, time::SensorTime t);

/// LioBackend::create's input checks: finite transforms (kNotFinite), and imu_T_lidar's translation equal to
/// the config's extrinsic_imu_lidar_*_m within 1e-6 m (kOutOfRange naming the key), so the extrinsic has one
/// source.
Result<void, config::ConfigError> check_frames(const LioConfig& config, const Eigen::Isometry3d& base_T_imu,
                                               const Eigen::Isometry3d& imu_T_lidar);

/// The rebased covariance after a failed prediction or a math exception: symmetrised, multiplied by
/// limits::kRebaseCovarianceInflation, each eigenvalue capped at limits::kRebaseMaxCovarianceEigenvalue.
fl::ManifoldState::Covariance inflated_for_rebase(const fl::ManifoldState::Covariance& p);

/// The history samples of one prediction: the newest sample at or before `start`, every sample after it
/// up to the first one at or after `end`. Empty when the history does not cover [start, end].
std::vector<fl::ImuSample> prediction_span(const std::deque<fl::ImuSample>& history, time::SensorTime start,
                                           time::SensorTime end);

/// Points stamped before `eskf_t` cannot be deskewed (no trajectory there): drops them and re-references the
/// rest to `eskf_t`. Contiguous scans (start == previous end) lose nothing.
void drop_points_before(fl::LidarScan& scan, time::SensorTime scan_start, time::SensorTime eskf_t);

/// The ESKF estimate converted to base_link (fast_lio_core navigation converters), with its covariance;
/// nullopt when a conversion fails or a value is not finite. May throw (reused math): the caller catches.
std::optional<BaseOdometry> base_odometry(const fl::BaseLinkStateConverter& converter,
                                          const fl::BaseLinkCovarianceProjector& projector,
                                          const fl::StateEstimate& est, const Eigen::Vector3d& omega);

/// Restart seed (§3.4) at c.t: `carried` (biases, gravity, extrinsics of the old epoch) with its velocity
/// rotated from the old world into the seed's, then the seed pose applied by InitialStatePriorApplicator
/// (in-flight context). nullopt when the applicator refuses. May throw (reused math): the caller catches.
std::optional<fl::ManifoldState> seed_state(const fl::InitialStatePriorApplicator& applicator,
                                            fl::ManifoldState carried, const RestartCommand& c,
                                            const Eigen::Quaterniond& q_base_imu);

}  // namespace uavnav::lio::backend_math
