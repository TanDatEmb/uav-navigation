#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "fast_lio_core/sensor/lidar_point.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/lifecycle.hpp"
#include "uavnav/lio/output_predictor.hpp"

// LioEstimator: the ROS-free facade the S1b `lio` node calls (SYSTEM_DESIGN §3, D20). It wires the reused
// FAST-LIO math (fast_lio_core: IMU initialiser, IKFoM ESKF, deskew, preprocessing, ikd-tree map) to the
// lifecycle (lifecycle.hpp), the degeneracy check (degeneracy.hpp) and the output predictor
// (output_predictor.hpp).
//
// Threading: single-threaded; the caller serialises all calls (push_imu, push_scan, restart and the
// accessors). There are no locks and the facade starts no thread of its own: the ikd-tree is built with
// its asynchronous rebuild thread disabled. The reused residual builder parallelises one correction over
// points with OpenMP (fork/join inside push_scan, deterministic per point; fast_lio_core default of 3
// threads). The event recorder is called only from inside these calls and never under a lock.
//
// Time: every input stamp is sensor time and must come from the range-checked converters
// (time::from_stamp<SensorTag>), as in lifecycle.hpp. The TimeSnapshot `now` passed to each call stamps the
// events of that call (one snapshot per cycle, AGENTS.md §2.5); nothing here reads a clock.
//
// Never throws: inputs are checked at the boundary (EstimatorReason), and an exception escaping the reused
// math is caught, emitted as a "LioScan" event with reason kMathException and treated as a degenerate scan.
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

/// One per accepted IMU sample once the output predictor is aligned (S1b decimates to 100 Hz).
/// `sample` is the predictor output of the IMU origin (q_world_imu, p/v of the IMU), see output_predictor.hpp.
struct StateOutput {
  OutputSample sample;
  std::uint32_t epoch;
  LioState state;
};

/// Corrected ESKF state at the scan END time, converted to base_link (fast_lio_core navigation converters).
struct OdometryOutput {
  time::SensorTime t;
  std::uint32_t epoch;
  std::uint32_t reset_counter;
  Eigen::Vector3d p_world_m;
  Eigen::Quaterniond q_world_base;
  Eigen::Vector3d v_world_mps;           ///< base_link origin velocity in the world
  Eigen::Matrix<double, 6, 6> pose_cov;  ///< (position world, rotation world) of base_link
  Eigen::Matrix3d vel_cov;               ///< base_link linear velocity, world frame
  std::uint8_t quality;                  ///< 0..100 from the degeneracy report of this scan
};

struct HealthOutput {
  time::SensorTime t;
  LioState state;
  std::uint32_t epoch;
  LioReason reason;  ///< reason of the latest lifecycle transition (kNone before the first)
  double translation_min_eigenvalue, rotation_min_eigenvalue;  ///< of the latest degeneracy report
  double correction_age_s;                ///< newest IMU time - latest scan end time (0 before the first scan)
  Eigen::Vector3d output_tracking_error;  ///< OutputPredictor::tracking_error()
};

struct StepOutputs {
  std::vector<StateOutput> states;
  std::optional<OdometryOutput> odometry;
  std::optional<HealthOutput> health;  ///< set on every transition and every limits::kHealthPeriod of IMU time
};

/// Why an input was rejected. A rejected input changes nothing (no state, no history, no lifecycle event).
enum class EstimatorReason : std::uint8_t { kAccepted, kOutOfOrder, kTooManyPoints, kNotFinite, kWrongState };

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
  }
  return "";
}

static_assert(ReasonEnum<EstimatorReason>);

/// Reason of the facade's own decision events (not lifecycle transitions). Event names:
///   "LioScan"            one per accepted scan: kScanGood, kScanDegenerate, kScanEmpty, kMapBootstrap,
///                        kPredictionFailed, kDeskewFailed, kMathException, kBeforeImuInit,
///                        kBeforeEstimatorTime (values: translation/rotation min eigenvalue, quality, points)
///   "ImuInitialized" / "ImuInitRejected" (once per epoch) / "ImuGap" / "ImuDuplicate" / "ImuRateTooHigh"
///   "OdometryInvalid"    the base_link conversion of a TRACKING scan failed (no odometry)
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
  kOdometryInvalid
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
  }
  return "";
}

static_assert(ReasonEnum<EstimatorEventReason>);

class LioEstimator {
 public:
  /// base_T_imu: ^base_link T_imu. imu_T_lidar: ^imu T_lidar; its translation must equal the config's
  /// extrinsic_imu_lidar_*_m (1e-6 m), so the extrinsic has one source (kOutOfRange naming the key otherwise).
  /// Fails (kOutOfRange / kNotFinite) on a non-finite transform or a config the reused math refuses. `events`
  /// must outlive the estimator.
  static Result<std::unique_ptr<LioEstimator>, config::ConfigError> create(const LioConfig& config,
                                                                           const Eigen::Isometry3d& base_T_imu,
                                                                           const Eigen::Isometry3d& imu_T_lidar,
                                                                           events::EventRecorder& events);

  ~LioEstimator();
  LioEstimator(const LioEstimator&) = delete;
  LioEstimator& operator=(const LioEstimator&) = delete;

  /// IMU sample: initialiser (until gravity and bias are known), predictor step, lifecycle kImuTick (the
  /// IMU-only LiDAR gap check, F22) and periodic health. Rejects (changing nothing) a non-finite sample
  /// (kNotFinite) or a stamp before the previous one (kOutOfOrder); an equal stamp is accepted as a
  /// duplicate (event, tick, no integration). Before the predictor is aligned `states` is empty.
  Result<StepOutputs, EstimatorReason> push_imu(const ImuInput& imu, const time::TimeSnapshot& now);

  /// Scan: predict -> deskew -> preprocess -> correct -> degeneracy -> lifecycle -> map insert/crop ->
  /// predictor correction. Rejects (changing nothing) more than limits::kMaxScanPoints points
  /// (kTooManyPoints), a non-finite point (kNotFinite), end < start or end before the previous scan's end
  /// (kOutOfOrder). An empty scan (before or after preprocessing) is the lifecycle event kScanEmpty and an
  /// Ok result. `odometry` is set only when the scan was corrected and leaves the state in TRACKING.
  Result<StepOutputs, EstimatorReason> push_scan(ScanInput&& scan, const time::TimeSnapshot& now);

  /// Restart (§3.4), only in RESTARTING (kWrongState otherwise; kNotFinite for a non-finite seed): clears
  /// the map, applies the seed through InitialStatePriorApplicator at the newest IMU time, epoch + 1,
  /// predictor reset_to (reset_counter + 1), lifecycle kRestartSeeded, event "LioRestart".
  Result<void, EstimatorReason> restart(const SeedPose& seed, const time::TimeSnapshot& now);

  LioState state() const noexcept;
  std::uint32_t epoch() const noexcept;

 private:
  struct Impl;
  explicit LioEstimator(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uavnav::lio
