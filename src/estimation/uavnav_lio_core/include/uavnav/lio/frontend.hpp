#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "uavnav/core/event_builder.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/channels.hpp"
#include "uavnav/lio/frontend_config.hpp"
#include "uavnav/lio/lifecycle.hpp"
#include "uavnav/lio/output_predictor.hpp"
#include "uavnav/lio/outputs.hpp"
#include "uavnav/lio/types.hpp"

// LioFrontend: the ingest-thread block of the LIO split (SYSTEM_DESIGN §3.5, D30/O14, D31, D32). It
// owns the OutputPredictor and the LioLifecycle (the ONLY writer of the LIO state), runs the
// LiDAR-gap check on every IMU sample, applies the backend's results and builds /lio/state,
// /lio/odometry and /lio/health. It never runs the registration and never touches the map: it talks
// to LioBackend only through LioChannels (pushes `requests`, pops `results`) and includes
// frontend_config.hpp, never the merged config or the backend's.
//
// Threading: single-threaded. Every call (push_imu, push_scan, poll_results and the
// accessors) comes from ONE thread, the ingest thread; the backend thread only touches the queues.
// No call blocks: a full request queue or a busy backend is an event with a reason (AGENTS.md §2),
// never a wait.
//
// Time: every input stamp is sensor time from the range-checked converters
// (time::from_stamp<SensorTag>), as in lifecycle.hpp. The TimeSnapshot `now` stamps the events of
// that call; nothing here reads a clock.
//
// Gap semantics (F22, D31, D40, AR4): the lifecycle's last-scan time advances only when a scan
// RESULT is applied; its time is the scan end. Admitted, refused, unprocessed and discarded scans
// do not count, and neither does a kScanGood result that is a rejected correction (fed as
// kScanCorrectionRejected, no odometry, "PredictorCorrectionRejected"): one the output predictor
// cannot place (OutputPredictor::on_correction: older than its 300 ms buffer, newer than the output,
// not finite; or no snapshot). That typed result is the only refusal of a correction (D40): an age
// never refuses one. An applied kScanGood always corrects the output and moves the gap reference; the
// lifecycle counts it toward confirm_scans only when (newest ticked IMU time - scan end) +
// limits::kScanPeriod <= gap_degraded (the freshness guard, lifecycle.hpp). The newest ticked IMU
// time is the newest accepted sample already TICKED: push_imu and push_scan apply the results once
// their input passed its checks (before anything else; poll_results at once), so the sample of the
// current push_imu call does not count yet. A result older than gap_degraded can only arrive outside TRACKING: its
// scan end is not before the gap reference, so the tick that made it that old already fired the
// TRACKING -> DEGRADED gap rule. A saturated backend whose results stay placeable (age <= 300 ms)
// and are older than gap_degraded - kScanPeriod therefore stays DEGRADED with a corrected output and
// no odometry, without flapping; a stalled one, or one whose results cannot be placed, reads as a
// LiDAR gap: DEGRADED, then LOST, on IMU time alone.
//
// LOST and RESTARTING (D41, AR4, review T2 M1): the map is frozen and the IMU prior has drifted, so the registration
// of a scan against it says nothing (critique §1). The frontend applies NO correction from a scan result there: a
// kScanGood is fed to the lifecycle as a non-empty scan only (no on_correction, no PredictorCorrectionRejected), and
// the output stays IMU-only until the new epoch.
//
// Restart (§3.4, D41, D47): LIO restarts itself; there is no restart call. The lifecycle decides (LOST ->
// RESTARTING after restart_dwell of stable data); the frontend's effect follows by level: at the end of every
// accepted push_imu, after the lifecycle tick, if the state is RESTARTING and no restart is pending it queues
// RestartCommand{epoch + 1, newest IMU time, seed, predictor's newest q/v}. The seed is the predictor's newest
// output (position, full attitude) moved to base_link with base_T_imu (D47: a seed for a fresh map only picks the
// gauge; PX4's information reaches the system through T's INIT). Refused before the queue (rate-limited
// "LioInputRejected", stream kRestartStream): kNotFinite for a non-finite predictor output, kBackendBusy when the
// request queue is full; the next push_imu tries again. The epoch switches only when the backend confirms
// (kRestartSeeded): that call's StepOutputs::restart holds the ResetDelta (about zero: the seed is the output
// itself; kNotFinite when the backend rejected the seed). A seed the backend refused (kRestartRejected) is fed to the
// lifecycle: RESTARTING -> LOST (RESTART_REJECTED), so the next command waits a full LOST dwell instead of following
// at the IMU rate (review T3 I2). A kRestartSeeded without a snapshot (a wiring error) is refused like a stale answer
// (kNotFinite, stays RESTARTING, re-issued at the next push_imu): the backend already switched epoch (review T3b n1).
//
// Late answer (D33/O16): a kRestartSeeded whose time the output predictor no longer covers
// (OutputPredictor::covers: older than kPredictorBufferSpan, or not matched) is refused, never applied by a rewinding
// reset_to: "LioRestartRejected" (kRestartAnswerStale), restart error kStaleAnswer, the state stays RESTARTING, epoch
// and output untouched, no restart pending: the next push_imu re-issues with the newest pose; the retry reuses the
// epoch number (the backend already switched to it). While a restart is pending only its answer is applied: the new
// epoch's scan results follow the answer (one FIFO), so an earlier one carrying that number belongs to the refused
// answer's seed and is discarded (kStaleEpoch).
//
// Never throws: inputs are checked at the boundary (EstimatorReason); an allocation failure of a
// state output drops that output with an "AllocationFailed" event.
namespace uavnav::lio {

struct FrontendStats {
  std::uint64_t imu_accepted;        ///< push_imu calls accepted (duplicates included)
  std::uint64_t scans_admitted;      ///< ScanJobs queued to the backend
  std::uint64_t scans_refused_busy;  ///< push_scan refused with kBackendBusy
  std::uint64_t imu_copies_dropped;  ///< IMU copies refused by a full request queue
  std::uint64_t
      requests_rejected;  ///< every kFull from requests.try_push (IMU copies, scans, restarts)
  std::uint64_t results_applied;    ///< backend results applied (any kind)
  std::uint64_t results_discarded;  ///< LioResultDiscarded (kStaleEpoch, kUnexpectedSequence)
  std::uint32_t
      scans_in_flight;  ///< admitted scans without a result yet, <= limits::kMaxScansInFlight
};

class LioFrontend {
 public:
  /// `channels` and `events` must outlive the frontend. Allocates nothing. base_T_imu: ^base_link T_imu, the same
  /// transform the backend gets (finite; LioRunner / LioPipeline::create check it through LioBackend::create); it
  /// moves the predictor's IMU pose to base_link for the restart seed.
  LioFrontend(const FrontendConfig& config, const Eigen::Isometry3d& base_T_imu, LioChannels& channels,
              events::EventRecorder& events) noexcept;
  LioFrontend(const LioFrontend&) = delete;
  LioFrontend& operator=(const LioFrontend&) = delete;

  /// IMU sample. Rejects (changing nothing) a non-finite sample (kNotFinite) or a stamp before the
  /// previous one (kOutOfOrder). Otherwise: applies the available backend results, then the S1a IMU
  /// path (an equal stamp is a duplicate: rate-limited "ImuDuplicate", no integration; "ImuGap"
  /// above imu_max_gap; "ImuRateTooHigh" once per epoch; predictor step; lifecycle kImuTick, the
  /// LiDAR-gap check; the restart effect when RESTARTING, see "Restart"; health on the sensor-time
  /// grid), and forwards a copy of a non-duplicate sample to the backend. A full request queue drops only that copy (rate-limited
  /// "LioQueueOverflow", kRequestQueueFull) and the call still returns Ok. Before the predictor is
  /// aligned (kImuInitialized) `states` is empty, afterwards it holds exactly one state per
  /// non-duplicate sample.
  Result<StepOutputs, EstimatorReason> push_imu(const ImuInput& imu, const time::TimeSnapshot& now);

  /// Scan. Checks in this order, each refusal changing nothing (rate-limited "LioInputRejected"):
  /// kTooManyPoints (> limits::kMaxScanPoints), kNotFinite (a point), kOutOfOrder (end < start, or
  /// end before the previous admitted scan's end), kScanAheadOfImu (end after the newest IMU
  /// sample, or no IMU yet: F22, a future stamp can never reach the gap reference), kBackendBusy
  /// (limits::kMaxScansInFlight scans without a result, or the request queue full; results still
  /// queued count as in flight until applied). Then applies the available results and queues
  /// ScanJob{seq, requested epoch, map policy}: kInsert in INITIALIZING/TRACKING/DEGRADED, kFrozen
  /// in LOST/RESTARTING. Outputs come only from the applied results.
  Result<StepOutputs, EstimatorReason> push_scan(ScanInput&& scan, const time::TimeSnapshot& now);

  /// The "stream" value of the restart effect's events (LioInputRejected / LioQueueOverflow).
  static constexpr int kRestartStream = 2;

  /// Applies the available backend results, in order (at most limits::kBackendResultCapacity per
  /// call).
  StepOutputs poll_results(const time::TimeSnapshot& now) {
    StepOutputs out;
    apply_results(out, now);
    return out;
  }

  LioState state() const noexcept { return lifecycle_.state(); }
  std::uint32_t epoch() const noexcept { return epoch_; }
  FrontendStats stats() const noexcept { return stats_; }

 private:
  /// Rate-limited per-sample events: one slot per EstimatorReason, plus ImuDuplicate and
  /// LioQueueOverflow.
  static constexpr std::size_t kDuplicateSlot = 8;
  static constexpr std::size_t kQueueFullSlot = 9;
  static constexpr std::size_t kSpamSlots = 10;
  static_assert(static_cast<std::size_t>(EstimatorReason::kStaleAnswer) + 1 == kDuplicateSlot,
                "one spam slot per EstimatorReason");
  struct Spam {
    std::uint64_t count{0};     // occurrences this epoch
    std::uint64_t reported{0};  // count at the last emitted event
  };

  events::EventBuilder event(std::string_view name, const time::TimeSnapshot& now) const noexcept;
  void emit(const events::EventBuilder& b) noexcept;
  bool spam_due(std::size_t slot) noexcept;
  void flush_spam(const time::TimeSnapshot& now) noexcept;
  EstimatorReason reject(EstimatorReason r, int stream, time::SensorTime t,
                         const time::TimeSnapshot& now) noexcept;
  void request_queue_full(int stream, time::SensorTime t, const time::TimeSnapshot& now) noexcept;
  HealthOutput health(time::SensorTime t) const noexcept;
  void on_transition(const Transition& tr, time::SensorTime t, const time::TimeSnapshot& now,
                     StepOutputs& out, const std::optional<DegeneracyReport>& report) noexcept;
  void track_rate(time::Duration dt, const time::TimeSnapshot& now) noexcept;
  void apply_results(StepOutputs& out, const time::TimeSnapshot& now) noexcept;
  void apply(const BackendResult& r, StepOutputs& out, const time::TimeSnapshot& now) noexcept;
  void apply_scan(const BackendResult& r, StepOutputs& out, const time::TimeSnapshot& now) noexcept;
  void apply_restart(const BackendResult& r, StepOutputs& out,
                     const time::TimeSnapshot& now) noexcept;
  void discard(const BackendResult& r, EstimatorEventReason why,
               const time::TimeSnapshot& now) noexcept;
  /// The restart effect (see "Restart" above): end of every accepted push_imu, after the lifecycle tick.
  void issue_restart(const time::TimeSnapshot& now) noexcept;

  FrontendConfig cfg_;
  LioChannels& channels_;
  events::EventRecorder& events_;
  LioLifecycle lifecycle_;
  OutputPredictor predictor_;
  Eigen::Quaterniond q_imu_base_;  // R_bi^T: base_link attitude = IMU attitude * q_imu_base_ (restart seed)
  Eigen::Vector3d t_base_imu_;     // the IMU origin in base_link (restart seed)

  std::optional<ImuInput> last_imu_;               // newest accepted IMU sample
  std::optional<time::SensorTime> last_scan_end_;  // newest admitted scan end (ordering check only)
  std::optional<time::SensorTime> next_health_t_;  // next point of the health grid
  std::optional<time::SensorTime>
      last_good_scan_t_;                  // end of the newest applied kScanGood (correction age, measured)
  std::optional<SeedPose> pending_seed_;  // seed of the pending restart (LioRestart event)
  std::array<Spam, kSpamSlots> spam_{};
  DegeneracyReport last_report_{0.0, 0.0, true, 0};
  LioReason last_reason_{LioReason::kNone};
  double position_sigma_m_{0.0};  // of the newest applied result; NaN fails closed in the lifecycle
  double imu_period_s_{0.0};
  std::size_t imu_period_samples_{0};
  bool rate_reported_{false};         // ImuRateTooHigh latch, reset per epoch
  std::uint32_t epoch_{1};            // the epoch the backend confirmed
  std::uint32_t requested_epoch_{1};  // != epoch_ <=> a restart is pending (data, not a flag)
  std::uint64_t seq_{
      0};  // seq of the newest admitted scan; in flight: seq_ - in_flight + 1 .. seq_
  FrontendStats stats_{0, 0, 0, 0, 0, 0, 0, 0};
};

}  // namespace uavnav::lio
