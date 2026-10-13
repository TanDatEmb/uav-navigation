#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "uavnav/core/config.hpp"
#include "uavnav/core/event_builder.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/outputs.hpp"
#include "uavnav/lio/runner.hpp"
#include "uavnav/lio/types.hpp"
#include "uavnav/lio_node/node_config.hpp"

// LioIngest: the ROS-free ingest layer of the lio node (SYSTEM_DESIGN §3.3, §3.5 "Node lio", §6.4; D31, D33/O20,
// O21). It is the ONLY caller of LioRunner, the product two-thread assembly; nothing in uavnav_lio uses
// LioPipeline (D31). The node (Task 8) converts messages and publishes; every decision is here.
//
// Threads (S1b runtime design). The node's executor thread is the ingest thread: on_imu, on_scan and runner() are
// called from it. A second, overlapping call (a misconfigured executor) is REFUSED, never raced: each call first
// tries to enter with one atomic compare-exchange (no lock, no wait); a call that finds another inside returns
// kReentered, emits a rate-limited "LioIngestReentered" and touches nothing else (it reads no member but the flag
// and the atomic counter, and it does not even store the ROS time). The guard's release on exit and acquire on
// entry order every member write of one call before the next call, on whatever thread it runs. The estimator
// thread (LioRunner's) reads only SharedRosTime, through the backend clock. stats() is any-thread (atomics).
// stop() and the destructor run on the owner's thread when no call can run any more (after the executor stopped).
//
// Pending scan (O20). A scan whose end is after the newest IMU stamp the runner accepted would be refused by the
// core (kScanAheadOfImu, F22). It waits instead, without blocking: ONE scan is held, and every call (on_imu and
// on_scan) first drops it once it has waited MORE than limits::kScanImuWaitMax of ROS time (now.ros - arrival ros;
// exactly kScanImuWaitMax still waits), then pushes it as soon as the newest accepted IMU stamp is >= its end.
// A newer scan arriving while one waits drops the waiting one (kSuperseded). Either drop is a "LioScanDropped"
// event. The expiry is checked before the IMU sample of the same call is pushed, so a scan that expired is never
// pushed even by the sample that would have covered it. A ROS clock that jumps back cannot expire a scan (the
// wait reads as negative); the next scan supersedes it. A runner refusal of a pushed waiting scan drops it (the
// core emits its own "LioInputRejected"); the call that pushed it is still Ok.
//
// Stamps. A stamp later than now.ros + limits::kMaxStampAhead is refused kStampInFuture before it reaches the
// core (rate-limited "LioInputRejected", reason STAMP_IN_FUTURE). Sensor and ROS time are compared directly:
// in SITL sensor time is ROS (sim) time (D29). For a scan the end stamp is checked (start > end is the core's
// kOutOfOrder).
//
// Outputs (O21). Each runner output passes through OutputCollector: an Ok restart answer latches last_reset; each
// state passes StateDecimator(limits::kStatePeriod) (one per 10 ms sensor-time cell) and is mapped to base_link
// with the raw gyro of the newest accepted IMU sample; odometry and health are "last wins" within one call,
// and an odometry replaced that way is counted (odometry_overwritten). A call that is refused after some
// outputs were collected (on_scan: a waiting scan pushed, then the new scan refused) keeps them for the next
// successful call; no output is lost.
//
// Event catalogue of uavnav_lio (component "lio"; the core's catalogue in uavnav/lio/types.hpp points here):
//   "LioIngestReentered"  IngestReason::kReentered: an overlapping call was refused (rate-limited: the 1st and every
//                         lio::limits::kEventSummaryEvery-th; value count). Emitted by the refused thread with
//                         its own `now`; no epoch (the runner is not read).
//   "LioInputRejected"    IngestReason::kStampInFuture (rate-limited as above; values stream 0 IMU / 1 scan,
//                         sensor_time_s, ros_time_s, count). The core emits the same event name with its
//                         EstimatorReason for its own refusals.
//   "LioScanDropped"      IngestReason::kImuCoverageTimeout or kSuperseded (values scan_end_s, waited_s).
//   "IngestSummary"       IngestReason::kAccepted, once, at stop(): IngestStats, the frontend's scan/IMU-copy
//                         counters and both runner queue stats (pushed, popped, rejected). This is the SITL
//                         measurement of kBackendRequestCapacity and kMaxScansInFlight.
namespace uavnav::lio_node {

enum class IngestReason : std::uint8_t {
  kAccepted,
  kReentered,
  kStampInFuture,
  kImuCoverageTimeout,
  kSuperseded,
  kRunnerRejected
};

constexpr std::string_view to_string(IngestReason r) {
  switch (r) {
    case IngestReason::kAccepted:
      return "ACCEPTED";
    case IngestReason::kReentered:
      return "REENTERED";
    case IngestReason::kStampInFuture:
      return "STAMP_IN_FUTURE";
    case IngestReason::kImuCoverageTimeout:
      return "IMU_COVERAGE_TIMEOUT";
    case IngestReason::kSuperseded:
      return "SUPERSEDED";
    case IngestReason::kRunnerRejected:
      return "RUNNER_REJECTED";
  }
  return "";
}

static_assert(ReasonEnum<IngestReason>);

/// One sample per `period` cell of SENSOR time (O21). Cell index = floor(t / period), absolute, so a gap or a
/// jump never shifts the phase. due(t) is true for the first sample of a cell later than the last due one; a
/// sample in that cell or an earlier one (non-monotonic time) is false and changes nothing. Precondition:
/// period > 0.
class StateDecimator {
 public:
  explicit StateDecimator(time::Duration period) noexcept;
  bool due(time::SensorTime t) noexcept;

 private:
  std::int64_t period_ns_;
  std::optional<std::int64_t> last_cell_;
};

/// One /lio/state sample (O21): the base_link pose in the LIO world, the base_link velocity in base_link, the raw
/// gyro rotated into base_link, and the latched reset delta. Covariances are a message concern (NaN, Task 8).
struct LioStateOut {
  time::SensorTime t;
  std::uint32_t epoch, reset_counter;
  lio::LioState state;
  Eigen::Vector3d p_world_base;
  Eigen::Quaterniond q_world_base;
  Eigen::Vector3d v_base_mps, w_base_rad_s;
  lio::ResetDelta last_reset;  ///< of the last applied restart; zero before the first reset
};

struct IngestOutputs {
  std::vector<LioStateOut> states;
  std::optional<lio::OdometryOutput> odometry;
  std::optional<lio::HealthOutput> health;
};

struct IngestStats {
  std::uint64_t scans_waited;           ///< scans held for IMU coverage
  std::uint64_t scans_dropped_timeout;  ///< waiting scans dropped after kScanImuWaitMax
  std::uint64_t scans_superseded;       ///< waiting scans dropped for a newer scan
  std::uint64_t odometry_overwritten;   ///< odometry replaced by a later one within one call (last wins)
  std::uint64_t reentered;              ///< overlapping calls refused
};

/// The ingest-thread output path of LioIngest, public so that tests can feed it runner outputs that LioIngest
/// cannot produce yet (a restart answer: the restart caller is S3/S4, D33/O25).
///
/// State mapping (O21). base_T_imu = (R_bi, t_bi) maps IMU-frame vectors into base_link; t_bi is the IMU origin
/// in base_link. The runner's state is the IMU's: q_world_imu (R_wi), p_world_imu, v_world_imu (output_predictor.hpp).
///   R_wb = R_wi * R_bi^T                               world <- base
///   p_wb = p_wi - R_wb * t_bi                          base origin = IMU origin - lever arm
///   w_b  = R_bi * gyro                                 the raw gyro (IMU frame) in base_link
///   v_b  = R_wb^T * v_wi + w_b x r,   r = -t_bi        base_link velocity in base_link
/// Derivation of the lever-arm sign: p_wi = p_wb + R_wb t_bi, so v_wi = v_wb + R_wb (w_b x t_bi) (d/dt R_wb =
/// R_wb [w_b]x), hence v_wb = v_wi - R_wb (w_b x t_bi) and in base_link v_b = R_wb^T v_wi - w_b x t_bi
/// = R_wb^T v_wi + w_b x r, with r the vector FROM the IMU TO the base origin. Example: IMU still, 0.25 m ahead
/// of the base (t_bi = (0.25, 0, 0)), yawing at +1 rad/s: v_b = (0, 0, 1) x (-0.25, 0, 0) = (0, -0.25, 0).
class OutputCollector {
 public:
  OutputCollector(const Eigen::Isometry3d& base_T_imu, time::Duration state_period) noexcept;

  /// Merges one runner output into `into`: first an Ok restart answer becomes last_reset (a refused one changes
  /// nothing), then each due state is mapped with `gyro_imu_rad_s` and appended, then odometry and health replace
  /// any earlier ones in `into` (an odometry replaced is counted). The restart goes first because a StepOutputs
  /// holds states only from the IMU step that follows the results it applied (frontend.hpp: push_imu applies the
  /// results, then integrates), so every state of a call that carries a restart answer is in the new epoch.
  void add(lio::StepOutputs&& from, const Eigen::Vector3d& gyro_imu_rad_s, IngestOutputs& into);

  /// The mapping above, with the current last_reset.
  LioStateOut map(const lio::StateOutput& s, const Eigen::Vector3d& gyro_imu_rad_s) const noexcept;

  std::uint64_t odometry_overwritten() const noexcept { return odometry_overwritten_; }

 private:
  Eigen::Matrix3d r_bi_;
  Eigen::Quaterniond q_ib_;  // R_bi^T as a quaternion
  Eigen::Vector3d t_bi_;
  StateDecimator decimator_;
  lio::ResetDelta last_reset_;
  std::uint64_t odometry_overwritten_{0};
};

/// The latest ROS time, shared lock-free between the ingest thread (store) and the estimator thread (the backend
/// clock: LioClock = [&shared] { return shared.snapshot(); }), so the backend never touches the node.
class SharedRosTime {
 public:
  void store(time::RosTime t) noexcept { ros_ns_.store(t.ns, std::memory_order_relaxed); }
  /// {time::steady_now(), the latest stored ROS time} (RosTime{0} before the first store). The value carries no
  /// other data, so relaxed ordering is enough. The steady read stamps the backend's events only.
  time::TimeSnapshot snapshot() const noexcept;

 private:
  std::atomic<std::int64_t> ros_ns_{0};
  static_assert(std::atomic<std::int64_t>::is_always_lock_free);
};

class LioIngest {
 public:
  /// Creates the LioRunner (starting its estimator thread) with base_T_imu = compose_base_T_imu(node, config),
  /// imu_T_lidar from config, and the backend clock reading `ros_time`. LioRunner::create's errors are returned
  /// as is. `events` and `ros_time` must outlive the ingest.
  static Result<std::unique_ptr<LioIngest>, config::ConfigError> create(const lio::LioConfig& config,
                                                                        const LioNodeConfig& node,
                                                                        events::EventRecorder& events,
                                                                        SharedRosTime& ros_time);
  ~LioIngest();  ///< stop(), then the runner (joined backend) is destroyed
  LioIngest(const LioIngest&) = delete;
  LioIngest& operator=(const LioIngest&) = delete;

  /// One IMU sample; `now` is the ingest thread's snapshot of this callback. Stores now.ros into the shared ROS
  /// time, expires the waiting scan, refuses a future stamp, pushes the sample (a runner refusal is
  /// kRunnerRejected), then pushes the waiting scan if this sample covers it.
  Result<IngestOutputs, IngestReason> on_imu(const lio::ImuInput& imu, const time::TimeSnapshot& now);

  /// One scan: as on_imu up to the stamp check; then the waiting scan (if any, still uncovered) is superseded,
  /// and this scan is pushed when covered (a runner refusal is kRunnerRejected) or held (Ok, no outputs).
  Result<IngestOutputs, IngestReason> on_scan(lio::ScanInput&& scan, const time::TimeSnapshot& now);

  /// LioRunner::stop() (joins the estimator thread), then one "IngestSummary". Idempotent; never throws. After it
  /// the calls keep working on the frontend alone (LioRunner).
  void stop() noexcept;

  /// Any thread.
  IngestStats stats() const noexcept;

  /// The runner, read-only: frontend(), state(), epoch() from the ingest thread (or when no call can run); the
  /// queue stats from any thread. For stats and tests.
  const lio::LioRunner& runner() const noexcept { return *runner_; }

 private:
  struct Pending {
    lio::ScanInput scan;
    time::RosTime arrived;
  };

  LioIngest(std::unique_ptr<lio::LioRunner> runner, const Eigen::Isometry3d& base_T_imu,
            events::EventRecorder& events, SharedRosTime& ros_time) noexcept;

  Result<IngestOutputs, IngestReason> refuse_reentered(const time::TimeSnapshot& now) noexcept;
  Result<IngestOutputs, IngestReason> refuse_future(int stream, time::SensorTime t, const time::TimeSnapshot& now);
  void expire_pending(const time::TimeSnapshot& now) noexcept;
  void push_pending_if_covered(const time::TimeSnapshot& now);
  void drop_pending(IngestReason why, const time::TimeSnapshot& now) noexcept;
  bool covered(time::SensorTime end) const noexcept { return newest_imu_ && *newest_imu_ >= end; }
  void collect(lio::StepOutputs&& out);
  IngestOutputs take_outputs() noexcept { return std::exchange(outputs_, IngestOutputs{}); }
  events::EventBuilder event(std::string_view name, const time::TimeSnapshot& now) const noexcept;
  void emit(const events::EventBuilder& b) noexcept;
  static void bump(std::atomic<std::uint64_t>& c) noexcept {
    c.store(c.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);  // single writer
  }

  events::EventRecorder& events_;
  SharedRosTime& ros_time_;
  std::atomic<bool> busy_{false};      // the try-enter flag (see "Threads")
  std::atomic<bool> stopped_{false};   // stop() ran
  std::atomic<std::uint64_t> reentered_{0};
  // Below: written only inside an entered call (or by stop()); atomics only so that stats() is any-thread.
  std::atomic<std::uint64_t> scans_waited_{0}, scans_dropped_timeout_{0}, scans_superseded_{0},
      odometry_overwritten_{0}, stamps_in_future_{0};
  OutputCollector collector_;
  IngestOutputs outputs_;                     // collected, returned by the next successful call
  std::optional<Pending> pending_;            // the one scan waiting for IMU coverage
  std::optional<time::SensorTime> newest_imu_;  // newest IMU stamp the runner accepted
  Eigen::Vector3d last_gyro_{Eigen::Vector3d::Zero()};  // raw gyro of that sample
  std::unique_ptr<lio::LioRunner> runner_;    // LAST, so destroyed first; its estimator thread reads ros_time_ only
};

}  // namespace uavnav::lio_node
