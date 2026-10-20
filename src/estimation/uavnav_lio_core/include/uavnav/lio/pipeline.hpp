#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>

#include "uavnav/core/config.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/lio/backend.hpp"
#include "uavnav/lio/channels.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/frontend.hpp"
#include "uavnav/lio/outputs.hpp"
#include "uavnav/lio/types.hpp"

// LioPipeline: the synchronous single-thread driver over LioFrontend and LioBackend (SYSTEM_DESIGN §3.5,
// D30/O14, D31, D32). It keeps the call surface of the S1a LioEstimator facade, for tests and offline replay
// only; the product node uses LioRunner (SYSTEM_DESIGN §3.5), which runs the backend on its own thread.
//
// Every call is: the frontend call (an error is returned as is), then LioBackend::step(now) until the
// request queue is empty, then LioFrontend::poll_results(now), merged into the outputs (states appended; a
// later non-empty optional replaces an earlier one, the "last wins" rule of outputs.hpp). The backend work
// therefore completes inside the call that queued it, so the results of a scan come back in the same call, and so
// does the answer to a restart the frontend issued itself (D41) in that push_imu: one call can then carry two
// transitions (LOST -> RESTARTING on the tick, RESTARTING -> INITIALIZING on the answer).
//
// Threading: single-threaded; the caller serialises all calls (the frontend's ingest contract), and the
// pipeline starts no thread. The reused residual builder still forks OpenMP workers inside a scan
// (limits::kIcpThreads) and joins them before the call returns.
//
// Ownership: the pipeline owns the LioChannels (on the heap), the backend and the frontend, declared in
// that order so that they are destroyed frontend, backend, channels.
namespace uavnav::lio {

class LioPipeline {
 public:
  /// base_T_imu: ^base_link T_imu. imu_T_lidar: ^imu T_lidar; its translation must equal the config's
  /// extrinsic_imu_lidar_*_m (1e-6 m), so the extrinsic has one source (kOutOfRange naming the key
  /// otherwise). Fails (kOutOfRange / kNotFinite) on a non-finite transform or a config the reused math
  /// refuses. `events` must outlive the pipeline.
  static Result<std::unique_ptr<LioPipeline>, config::ConfigError> create(const LioConfig& config,
                                                                          const Eigen::Isometry3d& base_T_imu,
                                                                          const Eigen::Isometry3d& imu_T_lidar,
                                                                          events::EventRecorder& events);

  LioPipeline(const LioPipeline&) = delete;
  LioPipeline& operator=(const LioPipeline&) = delete;

  /// LioFrontend::push_imu, then the backend drains its queue and the frontend applies the results.
  Result<StepOutputs, EstimatorReason> push_imu(const ImuInput& imu, const time::TimeSnapshot& now);

  /// LioFrontend::push_scan (refusals returned as is), then the backend processes the queue and the frontend
  /// applies the results: the scan's odometry, health and state outputs come back in this call.
  Result<StepOutputs, EstimatorReason> push_scan(ScanInput&& scan, const time::TimeSnapshot& now);

  LioState state() const noexcept { return frontend_->state(); }
  std::uint32_t epoch() const noexcept { return frontend_->epoch(); }
  const LioFrontend& frontend() const noexcept { return *frontend_; }
  const LioBackend& backend() const noexcept { return *backend_; }

 private:
  LioPipeline(std::unique_ptr<LioChannels> channels, std::unique_ptr<LioBackend> backend,
              std::unique_ptr<LioFrontend> frontend) noexcept;

  /// Backend steps until the request queue is empty, then the frontend's poll, merged into `out`.
  void drive(StepOutputs& out, const time::TimeSnapshot& now);

  std::unique_ptr<LioChannels> channels_;
  std::unique_ptr<LioBackend> backend_;
  std::unique_ptr<LioFrontend> frontend_;
};

}  // namespace uavnav::lio
