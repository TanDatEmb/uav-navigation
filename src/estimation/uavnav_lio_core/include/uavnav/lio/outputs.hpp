#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <vector>

#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/lifecycle.hpp"
#include "uavnav/lio/output_predictor.hpp"
#include "uavnav/lio/types.hpp"

// What the LIO frontend (ingest thread) publishes, SYSTEM_DESIGN §3.3/§3.5: /lio/state,
// /lio/odometry and /lio/health, plus the answer to a restart. Built only by the frontend (and the
// synchronous LioPipeline driver, which only merges); the backend never includes this header (it carries a LioState).
namespace uavnav::lio {

/// One per accepted IMU sample once the output predictor is aligned (S1b decimates to 100 Hz).
/// `sample` is the predictor output of the IMU origin (q_world_imu, p/v of the IMU), see
/// output_predictor.hpp.
struct StateOutput {
  OutputSample sample;
  std::uint32_t epoch;
  LioState state;
};

/// Corrected ESKF state at the scan END time, converted to base_link (fast_lio_core navigation
/// converters).
struct OdometryOutput {
  time::SensorTime t;
  std::uint32_t epoch;
  /// OutputPredictor::reset_counter(). Invariant: reset_counter == epoch - 1 (also for
  /// StateOutput::sample): an applied restart switches the epoch and resets the predictor exactly
  /// once, a refused answer (backend rejection or stale, D33/O16) does neither.
  std::uint32_t reset_counter;
  Eigen::Vector3d p_world_m;
  Eigen::Quaterniond q_world_base;
  Eigen::Vector3d v_world_mps;           ///< base_link origin velocity in the world
  Eigen::Matrix<double, 6, 6> pose_cov;  ///< (position world, rotation world) of base_link
  Eigen::Matrix3d vel_cov;               ///< base_link linear velocity, world frame
  std::uint8_t quality;                  ///< 0..100 from the degeneracy report of this scan
};

struct HealthOutput {
  time::SensorTime
      t;  ///< newest IMU time seen (the scan end before any IMU): monotonic across outputs
  LioState state;
  std::uint32_t epoch;
  LioReason reason;  ///< reason of the latest lifecycle transition (kNone before the first)
  /// Of the latest scan's degeneracy report; 0 when the latest scan had no report (empty, failed
  /// prediction).
  double translation_min_eigenvalue, rotation_min_eigenvalue;
  /// Newest IMU time - end time of the latest GOOD scan (kScanGood whose correction the output
  /// predictor applied; empty, degenerate and rejected-correction scans do not count; the S1a
  /// estimator counted every kScanGood). 0 before the first good scan.
  double correction_age_s;
  Eigen::Vector3d output_tracking_error;  ///< OutputPredictor::tracking_error()
};

/// One call's outputs. When one call applies several backend results (possible on the two-thread
/// runner), `odometry`, `health` and `restart` hold the LAST one (last wins); every state
/// transition is still recorded by its own StateTransition event.
struct StepOutputs {
  std::vector<StateOutput> states;
  std::optional<OdometryOutput> odometry;
  std::optional<HealthOutput>
      health;  ///< set on every transition and every limits::kHealthPeriod of IMU time
  /// Set by the call that applied the backend's answer to a restart (LioFrontend): the ResetDelta
  /// of the output predictor, kNotFinite when the backend rejected the seed, or kStaleAnswer when
  /// the output predictor no longer covers the command time (D33/O16: refused, no rewind). Set in
  /// the restart call itself when driven through LioPipeline.
  std::optional<Result<ResetDelta, EstimatorReason>> restart;
};

}  // namespace uavnav::lio
