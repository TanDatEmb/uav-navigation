#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

#include "fast_lio_core/sensor/lidar_point.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/degeneracy.hpp"

// Data shared by the LIO frontend (ingest thread) and backend (estimator thread), SYSTEM_DESIGN §3.5,
// D30/O14: the inputs, the estimator snapshot, the reasons, the queue messages and the one event catalogue.
// Plain immutable data: no lifecycle state lives here (the frontend is its single writer), so nothing in
// this header may include the lifecycle or the output-predictor headers.
//
// Event catalogue (component "lio"; event name -> emitting block -> reason enum):
//   "LioScan"              backend   EstimatorEventReason: one per scan request: kScanGood, kScanDegenerate,
//                                    kScanEmpty, kMapBootstrap, kPredictionFailed, kDeskewFailed,
//                                    kMathException, kBeforeImuInit, kBeforeEstimatorTime, kStaleEpoch (values
//                                    sensor_time_s, input_points, points, translation/rotation min eigenvalue,
//                                    quality, degenerate); kMathException also when a map insert threw
//   "EskfRebased"          backend   kEskfRebased: after a failed prediction (IMU gap, history dropped) or a
//                                    math exception the ESKF moves to the scan end with an inflated covariance
//                                    (values skipped_s, position_sigma_m)
//   "ImuInitialized"       backend   kImuInitialized (values gyro/accel bias norms)
//   "ImuInitRejected"      backend   kImuInitRejected (once), kMathException
//   "OdometryInvalid"      backend   kOdometryInvalid: the base_link conversion of a good scan failed
//   "AllocationFailed"     backend   kAllocationFailed, value site 0 (IMU history);
//                          frontend  site 1 (state output)
//   "LioQueueOverflow"     frontend  kRequestQueueFull (rate-limited);  backend  kResultQueueFull
//   "ImuGap" / "ImuDuplicate" / "ImuRateTooHigh"   frontend   kImuGap / kImuDuplicate (rate-limited) /
//                                    kImuRateTooHigh (once per epoch)
//   "LioInputRejected"     frontend  EstimatorReason (rate-limited, value count), incl. kBackendBusy
//   "LioResultDiscarded"   frontend  kStaleEpoch, kUnexpectedSequence
//   "LioRestartRejected"   frontend  kRestartRejected (the backend's answer; the backend emits no event of its own)
//   "StateTransition" / "LioRestart"   frontend   the lifecycle's reason enum (LioReason)
//   "PredictorCorrectionRejected"      frontend   PredictorReason
// Rate-limited events are emitted on the 1st, kEventSummaryEvery-th, ... occurrence per reason and epoch,
// and once more at the end of the epoch when occurrences happened since the last one.
namespace uavnav::lio {

struct ImuInput {
  time::SensorTime t;
  Eigen::Vector3d gyro_rad_s;
  Eigen::Vector3d accel_mps2;  ///< already scaled to m/s^2 (specific force, IMU frame)
};

struct ScanInput {
  time::SensorTime start, end;
  std::vector<uav::nav::lio::LidarPoint> points;  ///< LiDAR frame
  bool per_point_time;  ///< points carry relative_time_ns from `start`; false: one instant (no deskew)
};

/// T^-1 * PX4 pose (§3.4): the base_link pose in the LIO world of the new epoch.
struct SeedPose {
  Eigen::Vector3d p_world_m;
  Eigen::Quaterniond q_world_base;
};

/// The ESKF state at its own (scan) time s.t. Frames and signs: q_world_imu rotates IMU-frame vectors into
/// the world (lio_odom); gravity_world is the gravity VECTOR in the world, e.g. (0, 0, -9.81) for a z-up world.
struct EstimatorSnapshot {
  time::SensorTime t;
  Eigen::Quaterniond q_world_imu;
  Eigen::Vector3d v_world_mps;
  Eigen::Vector3d p_world_m;
  Eigen::Vector3d gyro_bias;      ///< rad/s, subtracted from delta_angle (bias * dt)
  Eigen::Vector3d accel_bias;     ///< m/s^2, subtracted from delta_velocity (bias * dt)
  Eigen::Vector3d gravity_world;  ///< the world gravity vector (see the sign convention above)
};

/// Why an input was rejected. A rejected input changes nothing (no state, no history, no lifecycle event).
enum class EstimatorReason : std::uint8_t {
  kAccepted,
  kOutOfOrder,
  kTooManyPoints,
  kNotFinite,
  kWrongState,
  kScanAheadOfImu,
  kBackendBusy  ///< limits::kMaxScansInFlight scans in flight, or the request queue is full
};

constexpr std::string_view to_string(EstimatorReason r) {
  switch (r) {
    case EstimatorReason::kAccepted:
      return "ACCEPTED";
    case EstimatorReason::kOutOfOrder:
      return "OUT_OF_ORDER";
    case EstimatorReason::kTooManyPoints:
      return "TOO_MANY_POINTS";
    case EstimatorReason::kNotFinite:
      return "NOT_FINITE";
    case EstimatorReason::kWrongState:
      return "WRONG_STATE";
    case EstimatorReason::kScanAheadOfImu:
      return "SCAN_AHEAD_OF_IMU";
    case EstimatorReason::kBackendBusy:
      return "BACKEND_BUSY";
  }
  return "";
}

static_assert(ReasonEnum<EstimatorReason>);

/// Reason of the LIO decision events that are not lifecycle transitions (see the catalogue above).
enum class EstimatorEventReason : std::uint8_t {
  kScanGood,
  kScanDegenerate,
  kScanEmpty,
  kMapBootstrap,
  kPredictionFailed,
  kDeskewFailed,
  kMathException,
  kBeforeImuInit,
  kBeforeEstimatorTime,
  kImuInitialized,
  kImuInitRejected,
  kImuGap,
  kImuDuplicate,
  kImuRateTooHigh,
  kOdometryInvalid,
  kEskfRebased,
  kAllocationFailed,
  kRequestQueueFull,
  kResultQueueFull,
  kStaleEpoch,
  kUnexpectedSequence,
  kRestartRejected
};

constexpr std::string_view to_string(EstimatorEventReason r) {
  switch (r) {
    case EstimatorEventReason::kScanGood:
      return "SCAN_GOOD";
    case EstimatorEventReason::kScanDegenerate:
      return "SCAN_DEGENERATE";
    case EstimatorEventReason::kScanEmpty:
      return "SCAN_EMPTY";
    case EstimatorEventReason::kMapBootstrap:
      return "MAP_BOOTSTRAP";
    case EstimatorEventReason::kPredictionFailed:
      return "PREDICTION_FAILED";
    case EstimatorEventReason::kDeskewFailed:
      return "DESKEW_FAILED";
    case EstimatorEventReason::kMathException:
      return "MATH_EXCEPTION";
    case EstimatorEventReason::kBeforeImuInit:
      return "BEFORE_IMU_INIT";
    case EstimatorEventReason::kBeforeEstimatorTime:
      return "BEFORE_ESTIMATOR_TIME";
    case EstimatorEventReason::kImuInitialized:
      return "IMU_INITIALIZED";
    case EstimatorEventReason::kImuInitRejected:
      return "IMU_INIT_REJECTED";
    case EstimatorEventReason::kImuGap:
      return "IMU_GAP";
    case EstimatorEventReason::kImuDuplicate:
      return "IMU_DUPLICATE";
    case EstimatorEventReason::kImuRateTooHigh:
      return "IMU_RATE_TOO_HIGH";
    case EstimatorEventReason::kOdometryInvalid:
      return "ODOMETRY_INVALID";
    case EstimatorEventReason::kEskfRebased:
      return "ESKF_REBASED";
    case EstimatorEventReason::kAllocationFailed:
      return "ALLOCATION_FAILED";
    case EstimatorEventReason::kRequestQueueFull:
      return "REQUEST_QUEUE_FULL";
    case EstimatorEventReason::kResultQueueFull:
      return "RESULT_QUEUE_FULL";
    case EstimatorEventReason::kStaleEpoch:
      return "STALE_EPOCH";
    case EstimatorEventReason::kUnexpectedSequence:
      return "UNEXPECTED_SEQUENCE";
    case EstimatorEventReason::kRestartRejected:
      return "RESTART_REJECTED";
  }
  return "";
}

static_assert(ReasonEnum<EstimatorEventReason>);

// --- frontend -> backend --------------------------------------------------------------------------------

/// Decided by the frontend at admission from the lifecycle state then (kInsert in INITIALIZING, TRACKING and
/// DEGRADED; kFrozen in LOST and RESTARTING). The backend never reads the lifecycle.
enum class MapPolicy : std::uint8_t { kInsert, kFrozen };

/// One admitted scan. `seq` increases by one per admitted scan; `epoch` is the frontend's requested epoch.
struct ScanJob {
  std::uint64_t seq;
  std::uint32_t epoch;
  MapPolicy map_policy;
  ScanInput scan;  ///< the points live in a std::vector: moving a job is a pointer swap
};

/// Restart (§3.4): seed the ESKF at `t` (the frontend's newest IMU time) in epoch `new_epoch`. The old
/// world's attitude and velocity at `t` (the frontend's newest output) give the carried velocity.
struct RestartCommand {
  std::uint32_t new_epoch;
  time::SensorTime t;
  SeedPose seed;
  Eigen::Quaterniond q_world_imu_old;
  Eigen::Vector3d v_world_mps_old;
};

/// One FIFO carries every request, so the IMU copies a scan or restart needs are always queued ahead of it.
using BackendRequest = std::variant<ImuInput, ScanJob, RestartCommand>;

// --- backend -> frontend --------------------------------------------------------------------------------

/// The first five kinds feed the lifecycle (via the frontend); the last three do not (D31).
enum class BackendResultKind : std::uint8_t {
  kScanGood,
  kScanDegenerate,
  kScanEmpty,
  kMapReady,
  kRestartSeeded,
  kImuInitialized,    ///< the snapshot aligns the frontend's predictor
  kScanNotProcessed,  ///< stale epoch, before the IMU initialisation or before the ESKF time: accounting only
  kRestartRejected    ///< the seed was refused; the backend's epoch is unchanged
};

constexpr std::string_view to_string(BackendResultKind k) {
  switch (k) {
    case BackendResultKind::kScanGood:
      return "SCAN_GOOD";
    case BackendResultKind::kScanDegenerate:
      return "SCAN_DEGENERATE";
    case BackendResultKind::kScanEmpty:
      return "SCAN_EMPTY";
    case BackendResultKind::kMapReady:
      return "MAP_READY";
    case BackendResultKind::kRestartSeeded:
      return "RESTART_SEEDED";
    case BackendResultKind::kImuInitialized:
      return "IMU_INITIALIZED";
    case BackendResultKind::kScanNotProcessed:
      return "SCAN_NOT_PROCESSED";
    case BackendResultKind::kRestartRejected:
      return "RESTART_REJECTED";
  }
  return "";
}

static_assert(ReasonEnum<BackendResultKind>);

/// Corrected ESKF state at the scan end, converted to base_link (fast_lio_core navigation converters).
struct BaseOdometry {
  Eigen::Vector3d p_world_m;
  Eigen::Quaterniond q_world_base;
  Eigen::Vector3d v_world_mps;           ///< base_link origin velocity in the world
  Eigen::Matrix<double, 6, 6> pose_cov;  ///< (position world, rotation world) of base_link
  Eigen::Matrix3d vel_cov;               ///< base_link linear velocity, world frame
};

/// The answer to one request. Carries estimator data only, never a lifecycle state.
struct BackendResult {
  BackendResultKind kind;
  std::uint32_t epoch;  ///< scan results: the ScanJob's epoch; restart results: the command's new_epoch
  std::uint64_t seq;    ///< seq of the ScanJob; 0 for non-scan results
  time::SensorTime t;   ///< scan end, initialising IMU sample, or restart time
  /// The ESKF state at `t`: set for kImuInitialized, kRestartSeeded and every processed scan (the ESKF
  /// time is the scan end afterwards); unset for kScanNotProcessed and kRestartRejected. The frontend
  /// corrects its predictor only from a kScanGood snapshot.
  std::optional<EstimatorSnapshot> snapshot;
  std::optional<DegeneracyReport> report;  ///< set when a correction was attempted
  double position_sigma_m;                 ///< after this request; NaN fails closed in the lifecycle
  std::optional<BaseOdometry> odometry;    ///< kScanGood only (OdometryInvalid event when the conversion fails)
};

static_assert(std::is_nothrow_move_constructible_v<BackendRequest>);
static_assert(std::is_nothrow_move_constructible_v<BackendResult>);

}  // namespace uavnav::lio
