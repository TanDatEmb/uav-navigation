#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <functional>
#include <memory>
#include <stop_token>
#include <thread>

#include "uavnav/core/config.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/spsc_queue.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/backend.hpp"
#include "uavnav/lio/channels.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/frontend.hpp"
#include "uavnav/lio/outputs.hpp"
#include "uavnav/lio/types.hpp"

// LioRunner: the PRODUCT two-thread assembly of the LIO split (SYSTEM_DESIGN §3.5 "the threaded execution is
// product code", D30/O14, D31, D32). It owns the LioChannels, the LioBackend and the LioFrontend and runs
// LioBackend::run on its own std::jthread (the estimator thread). The S1b node uses this class and never wires
// the two threads itself; LioPipeline (synchronous) is for tests and offline replay only.
//
// Threads. Exactly one thread is started (by create()); the runner takes no lock.
//   - Ingest thread (the caller): push_imu, push_scan, restart, poll_results, state, epoch, frontend() must
//     all come from ONE thread at a time (the node's ingest thread): they run the LioFrontend, which is
//     single-threaded. None of them blocks or waits on the backend: a busy backend is kBackendBusy, a full
//     request queue drops the IMU copy with an event (frontend.hpp).
//   - Estimator thread: LioBackend::run, which calls the clock once before every request (see LioClock).
//   - request_queue_stats() / result_queue_stats(): any thread.
//   - stop(), request_stop() and the destructor: the owner's thread, never the estimator thread (the clock).
// The two threads share only the two SPSC queues (and the event recorder, which is thread-safe).
//
// Outputs. Every frontend result is applied on the ingest thread. A call returns the outputs of the results it
// applied, merged "last wins" (outputs.hpp: states appended in order; a later odometry/health/restart
// replaces an earlier one). push_scan polls the results BEFORE handing the scan to the frontend, because the
// frontend's busy check precedes its own application of waiting results: a scan is never refused busy while
// the answers that free its slot are already waiting. If the scan is then refused, the outputs of the results
// that poll applied are kept and returned by the next successful call (push_imu, push_scan or poll_results):
// no output is lost. The ingest thread should call poll_results (or push_imu, which applies results too) at
// least at the IMU rate.
//
// Shutdown (design table "Shutdown"). stop() requests stop on the jthread and joins it: LioBackend::run returns
// after the request in progress and leaves the rest of the queue unprocessed. The destructor calls stop(), then
// destroys the backend, the frontend and the channels, in that order. The jthread is declared LAST, so it is
// started only after every member it uses is built, and (were stop() ever skipped) destroyed, i.e. joined,
// before any of them. After stop() the ingest-thread API keeps working on the frontend alone: IMU samples are
// processed (gap check, states), scans are admitted until limits::kMaxScansInFlight are in flight and then
// refused kBackendBusy, and no result arrives any more.
//
// Failure paths of the estimator thread (AGENTS.md §2.6: a worker never dies silently).
//   - The clock throws: caught; the last good snapshot (a zero snapshot before the first) stamps that request
//     and a rate-limited "LioClockFailed" event (kClockFailed, value count) is emitted. The backend goes on.
//   - LioBackend::step does not throw (it catches the reused math, backend.hpp). Anything that still escaped
//     LioBackend::run would end in std::terminate (std::jthread): loud, never silent.
//   - A result that never arrives would leave its scan in flight forever: FrontendStats::scans_in_flight stays
//     up, every later scan is refused kBackendBusy and the gap rule takes the system to LOST (fail closed,
//     until a restart). This cannot happen by construction: the result queue cannot fill (static_assert in
//     limits.hpp) and the backend answers every request it pops; only stop() leaves requests unanswered.
//   - A slow or stalled backend reads as a LiDAR gap on IMU time: DEGRADED, then LOST (frontend.hpp, F22).
//
// Late restart answer. A kRestartSeeded that is applied more than limits::kPredictorBufferSpan (300 ms of IMU
// time) after its command (a backend stalled while the restart was pending) cannot be matched in the output
// predictor's buffer: OutputPredictor::reset_to restarts the predictor at the command time cmd.t, i.e. the output
// is rewound to cmd.t and the next IMU sample continues from there (the IMU motion between cmd.t and the answer
// is not integrated into the new epoch's output). The state becomes INITIALIZING as for a timely answer, and the
// "LioRestart" event's seed_age_s (newest IMU time - cmd.t) exposes the age; the confirm scans of the new epoch
// then correct the output. Pinned by LioRunner.LateRestartAnswerRestartsThePredictorAndRecovers.
//
// OpenMP: the backend's ICP forks limits::kIcpThreads OpenMP threads inside one correction call, on the
// estimator thread only (1 in the TSan build).
namespace uavnav::lio {

/// The backend clock: called on the estimator thread once before every request; its snapshot stamps the events
/// of that request. Supplied by the caller (the cores read no clock). It may throw (handled, see above); a clock
/// that blocks stalls the backend, which reads as a LiDAR gap (the tests use it to stall the backend).
using LioClock = std::function<time::TimeSnapshot()>;

class LioRunner {
 public:
  /// Builds the channels (on the heap), the backend and the frontend, then starts the backend thread.
  /// base_T_imu / imu_T_lidar as LioBackend::create (its errors are returned as is: kOutOfRange naming the
  /// extrinsic key, kNotFinite). An empty `backend_clock` is kMissingKey with key "backend_clock".
  /// `events` must outlive the runner. Throws std::system_error only if the thread cannot be started.
  static Result<std::unique_ptr<LioRunner>, config::ConfigError> create(const LioConfig& config,
                                                                        const Eigen::Isometry3d& base_T_imu,
                                                                        const Eigen::Isometry3d& imu_T_lidar,
                                                                        events::EventRecorder& events,
                                                                        LioClock backend_clock);

  ~LioRunner();
  LioRunner(const LioRunner&) = delete;
  LioRunner& operator=(const LioRunner&) = delete;
  LioRunner(LioRunner&&) = delete;
  LioRunner& operator=(LioRunner&&) = delete;

  // --- ingest-thread API: ONE caller thread; same results and reasons as LioFrontend ---------------------

  /// LioFrontend::push_imu (applies the waiting results), merged after any outputs carried from a refused scan.
  Result<StepOutputs, EstimatorReason> push_imu(const ImuInput& imu, const time::TimeSnapshot& now);

  /// Applies the waiting results first (see "Outputs"), then LioFrontend::push_scan. On a refusal the applied
  /// results' outputs are returned by the next successful call.
  Result<StepOutputs, EstimatorReason> push_scan(ScanInput&& scan, const time::TimeSnapshot& now);

  /// LioFrontend::restart: queues the command. The answer (ResetDelta, or kNotFinite when the backend refused
  /// the seed) is StepOutputs::restart of the later call that applies it.
  Result<void, EstimatorReason> restart(const SeedPose& seed, const time::TimeSnapshot& now);

  /// Applies the waiting results (and returns any outputs carried from a refused scan).
  StepOutputs poll_results(const time::TimeSnapshot& now);

  LioState state() const noexcept { return frontend_->state(); }
  std::uint32_t epoch() const noexcept { return frontend_->epoch(); }

  /// For stats and tests. frontend(): ingest thread. backend(): only after stop() (its stats are written by the
  /// estimator thread while it runs).
  const LioFrontend& frontend() const noexcept { return *frontend_; }
  const LioBackend& backend() const noexcept { return *backend_; }

  /// Queue counters (SpscQueue::stats): any thread.
  concurrency::QueueStats request_queue_stats() const noexcept { return channels_->requests.stats(); }
  concurrency::QueueStats result_queue_stats() const noexcept { return channels_->results.stats(); }

  // --- owner ------------------------------------------------------------------------------------------

  /// Asks the estimator thread to return after the request in progress; does not wait (stop() joins).
  void request_stop() noexcept { (void)thread_.request_stop(); }

  /// request_stop(), then joins the estimator thread. Idempotent; never throws.
  void stop() noexcept;

 private:
  LioRunner(std::unique_ptr<LioChannels> channels, std::unique_ptr<LioBackend> backend,
            std::unique_ptr<LioFrontend> frontend, events::EventRecorder& events, LioClock clock);

  /// Estimator-thread body: LioBackend::run with the guarded clock.
  void run_backend(std::stop_token stop) noexcept;

  /// Ingest thread: carried outputs, then `later` (last wins).
  StepOutputs take_carried(StepOutputs&& later);

  // Declaration order is the lifetime order (see "Shutdown"); the jthread is LAST.
  std::unique_ptr<LioChannels> channels_;
  std::unique_ptr<LioBackend> backend_;
  std::unique_ptr<LioFrontend> frontend_;
  events::EventRecorder& events_;
  LioClock clock_;        // called on the estimator thread only
  StepOutputs carried_;   // ingest thread only: outputs of results applied by a refused push_scan
  std::jthread thread_;   // LAST: started after, and joined before, everything above
};

}  // namespace uavnav::lio
