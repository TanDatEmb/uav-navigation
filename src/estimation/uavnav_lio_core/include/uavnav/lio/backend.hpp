#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stop_token>

#include "uavnav/core/config.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/channels.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/types.hpp"

// LioBackend: the estimator-thread block of the LIO split (SYSTEM_DESIGN §3.5, D30/O14). It owns all the
// reused FAST-LIO math (fast_lio_core: IMU initialiser, IKFoM ESKF, deskew, preprocessing, ICP, ikd-tree
// map) and the degeneracy check (degeneracy.hpp). It pops BackendRequests from `channels.requests`, in
// FIFO order, and pushes one BackendResult per scan job, per restart command and per IMU initialisation.
//
// It never sees the lifecycle and publishes nothing: it classifies each scan (BackendResultKind) and the
// frontend, the single writer of the lifecycle, decides what the result means. The map policy of a scan is
// decided by the frontend at admission (ScanJob::map_policy); the map bootstrap is the first processed scan
// of an epoch (empty map).
//
// Per scan (S1a call order): predict -> deskew -> preprocess -> (map bootstrap | correct -> degeneracy) ->
// map insert/crop -> base_link odometry (kScanGood only).
//
// Threading: single consumer of `requests`, single producer of `results`. step() and run() are called from
// one thread (the estimator thread, or the caller of the synchronous driver); stats() from that thread or
// after run() returned. The ikd-tree is built without its asynchronous rebuild thread; one ICP correction
// forks limits::kIcpThreads OpenMP threads and joins them inside step(). The event recorder is called only
// from inside step() and never under a lock.
//
// Time: every stamp is sensor time and must come from the range-checked converters (time::from_stamp). The
// TimeSnapshot passed to step() stamps the events of that request; nothing here reads a clock (run() takes
// the clock callable from its caller).
//
// Never throws out of step(): an exception escaping the reused math is caught, emitted as a "LioScan" event
// with reason kMathException and treated as a degenerate scan (the ESKF is restored and rebased).
namespace uavnav::lio {

struct BackendStats {
  std::uint64_t imu_samples;      ///< ImuInput requests processed
  std::uint64_t scans;            ///< ScanJob requests processed (any result kind)
  std::uint64_t restarts;         ///< RestartCommands applied (kRestartSeeded)
  std::uint64_t results_dropped;  ///< results refused by a full result queue (LioQueueOverflow)
  std::size_t map_points;         ///< points in the ikd-tree map now
};

class LioBackend {
 public:
  /// base_T_imu: ^base_link T_imu. imu_T_lidar: ^imu T_lidar; its translation must equal the config's
  /// extrinsic_imu_lidar_*_m (1e-6 m), so the extrinsic has one source (kOutOfRange naming the key otherwise).
  /// Fails (kOutOfRange / kNotFinite) on a non-finite transform or a config the reused math refuses.
  /// `channels` and `events` must outlive the backend.
  static Result<std::unique_ptr<LioBackend>, config::ConfigError> create(const LioConfig& config,
                                                                         const Eigen::Isometry3d& base_T_imu,
                                                                         const Eigen::Isometry3d& imu_T_lidar,
                                                                         LioChannels& channels,
                                                                         events::EventRecorder& events);

  ~LioBackend();
  LioBackend(const LioBackend&) = delete;
  LioBackend& operator=(const LioBackend&) = delete;

  /// Processes at most one request. Returns false (doing nothing) when the request queue was empty.
  ///   ImuInput        appended to the IMU history; feeds the initialiser until it succeeds (kImuInitialized).
  ///   ScanJob         kScanNotProcessed for another epoch (kStaleEpoch), before the IMU initialisation
  ///                   (kBeforeImuInit) or ending at/before the ESKF time (kBeforeEstimatorTime); otherwise
  ///                   kMapReady (empty map: the scan becomes the map), kScanGood (with odometry),
  ///                   kScanDegenerate or kScanEmpty. One "LioScan" event per scan.
  ///   RestartCommand  clears the map, seeds the ESKF at cmd.t, switches to cmd.new_epoch: kRestartSeeded;
  ///                   kRestartRejected (epoch unchanged) when not initialised or the seed is refused.
  /// A full result queue drops the result with a LioQueueOverflow event (kResultQueueFull).
  bool step(const time::TimeSnapshot& now);

  /// Estimator-thread body: waits for a request, then step(clock()), until `stop` is requested. Checks the
  /// stop token after every wait and before every request, so it returns after the request in progress and
  /// leaves the rest of the queue unprocessed.
  void run(std::stop_token stop, const std::function<time::TimeSnapshot()>& clock);

  BackendStats stats() const noexcept;

 private:
  struct Impl;
  explicit LioBackend(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uavnav::lio
