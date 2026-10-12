#include "uavnav/lio/frontend.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <variant>

#include "uavnav/lio/limits.hpp"

// Extracted from the S1a facade (estimator.cpp): ingest checks, IMU path, health/transition/event
// helpers, predictor side of restart. It decides what a backend result means (§3.5, D30/O14, D31).
namespace uavnav::lio {

namespace {

using events::EventBuilder;

/// Weight of the newest dt in the IMU period average used for the ImuRateTooHigh check.
constexpr double kImuPeriodEmaWeight = 0.1;
/// The report of a scan without a correction (empty, map bootstrap): no information.
constexpr DegeneracyReport kNoReport{0.0, 0.0, true, 0};
/// LioInputRejected / LioQueueOverflow "stream" value.
enum Stream : int { kStreamImu = 0, kStreamScan = 1, kStreamRestart = LioFrontend::kRestartStream };

double seconds(time::SensorTime t) { return static_cast<double>(t.ns) / 1e9; }

double yaw_of(const Eigen::Matrix3d& r) { return std::atan2(r(1, 0), r(0, 0)); }

bool usable(const Eigen::Quaterniond& q) { return q.coeffs().allFinite() && q.norm() > 1e-9; }

bool is_scan_result(BackendResultKind k) {
  return k == BackendResultKind::kScanGood || k == BackendResultKind::kScanDegenerate ||
         k == BackendResultKind::kScanEmpty || k == BackendResultKind::kMapReady ||
         k == BackendResultKind::kScanNotProcessed;
}

}  // namespace

LioFrontend::LioFrontend(const FrontendConfig& config, LioChannels& channels,
                         events::EventRecorder& events) noexcept
    : cfg_(config),
      channels_(channels),
      events_(events),
      lifecycle_(config.lifecycle),
      predictor_(config.predictor) {}

// --- events --------------------------------------------------------------------------------------

EventBuilder LioFrontend::event(std::string_view name,
                                const time::TimeSnapshot& now) const noexcept {
  events::EventIdentity id;
  id.lio_epoch = epoch_;
  EventBuilder b(events::Component::kLio, name, now);
  b.identity(id);
  return b;
}

void LioFrontend::emit(const EventBuilder& b) noexcept { (void)events_.emit(b.build()); }

/// Counts one occurrence; true on the 1st and every kEventSummaryEvery-th of the epoch (emit it).
bool LioFrontend::spam_due(std::size_t slot) noexcept {
  Spam& c = spam_[slot];
  ++c.count;
  if (c.count != 1 && c.count % limits::kEventSummaryEvery != 0) return false;
  c.reported = c.count;
  return true;
}

/// End of an epoch: one summary per slot whose latest occurrences were not emitted, then reset.
void LioFrontend::flush_spam(const time::TimeSnapshot& now) noexcept {
  for (std::size_t slot = 0; slot < kSpamSlots; ++slot) {
    const Spam& c = spam_[slot];
    if (c.count <= c.reported) continue;
    EventBuilder b = event(slot == kDuplicateSlot   ? "ImuDuplicate"
                           : slot == kQueueFullSlot ? "LioQueueOverflow"
                                                    : "LioInputRejected",
                           now);
    if (slot == kDuplicateSlot) b.reason(EstimatorEventReason::kImuDuplicate);
    if (slot == kQueueFullSlot) b.reason(EstimatorEventReason::kRequestQueueFull);
    if (slot < kDuplicateSlot) b.reason(static_cast<EstimatorReason>(slot));
    emit(b.value("count", static_cast<double>(c.count)));
  }
  spam_ = {};
}

EstimatorReason LioFrontend::reject(EstimatorReason r, int stream, time::SensorTime t,
                                    const time::TimeSnapshot& now) noexcept {
  const std::size_t slot = static_cast<std::size_t>(r);
  if (spam_due(slot)) {
    emit(event("LioInputRejected", now)
             .reason(r)
             .value("stream", stream)
             .value("sensor_time_s", seconds(t))
             .value("count", static_cast<double>(spam_[slot].count)));
  }
  return r;
}

void LioFrontend::request_queue_full(int stream, time::SensorTime t,
                                     const time::TimeSnapshot& now) noexcept {
  ++stats_.requests_rejected;
  if (!spam_due(kQueueFullSlot)) return;
  emit(event("LioQueueOverflow", now)
           .reason(EstimatorEventReason::kRequestQueueFull)
           .value("stream", stream)
           .value("sensor_time_s", seconds(t))
           .value("count", static_cast<double>(spam_[kQueueFullSlot].count)));
}

// --- health and transitions ----------------------------------------------------------------------

HealthOutput LioFrontend::health(time::SensorTime t) const noexcept {
  double age = 0.0;
  if (last_good_scan_t_ && last_imu_)
    age = std::max(0.0, time::to_seconds(last_imu_->t - *last_good_scan_t_));
  const DegeneracyReport& d = last_report_;
  // clang-format off
  return {t, lifecycle_.state(), epoch_, last_reason_, d.translation_min_eigenvalue,
          d.rotation_min_eigenvalue, age, predictor_.tracking_error()};
  // clang-format on
}

/// One StateTransition event per change, and the health output of that call.
void LioFrontend::on_transition(const Transition& tr, time::SensorTime t,
                                const time::TimeSnapshot& now, StepOutputs& out,
                                const std::optional<DegeneracyReport>& report) noexcept {
  if (!tr.changed) return;
  last_reason_ = tr.reason;
  EventBuilder b = event("StateTransition", now);
  b.states(tr.before, tr.after).reason(tr.reason).value("sensor_time_s", seconds(t));
  b.value("position_sigma_m", position_sigma_m_);
  if (report) {
    b.value("quality", report->quality)
        .value("translation_min_eigenvalue", report->translation_min_eigenvalue)
        .value("rotation_min_eigenvalue", report->rotation_min_eigenvalue);
  }
  emit(b);
  // Stamped with the newest IMU time (scan ends are older), so health stamps never decrease.
  out.health = health(last_imu_ ? std::max(last_imu_->t, t) : t);
}

void LioFrontend::track_rate(time::Duration dt, const time::TimeSnapshot& now) noexcept {
  if (dt.ns <= 0 || dt > limits::kPredictorBufferSpan) return;
  const double dt_s = time::to_seconds(dt);
  imu_period_s_ = imu_period_samples_ == 0
                      ? dt_s
                      : imu_period_s_ + kImuPeriodEmaWeight * (dt_s - imu_period_s_);
  ++imu_period_samples_;
  if (rate_reported_ || imu_period_samples_ < limits::kPredictorBufferCapacity) return;
  // The predictor buffer must span kPredictorBufferSpan (§3.3: up to 300 ms scan delay).
  if (static_cast<double>(limits::kPredictorBufferCapacity) * imu_period_s_ <
      time::to_seconds(limits::kPredictorBufferSpan)) {
    rate_reported_ = true;  // once per epoch
    emit(event("ImuRateTooHigh", now)
             .reason(EstimatorEventReason::kImuRateTooHigh)
             .value("imu_period_s", imu_period_s_));
  }
}

// --- results -------------------------------------------------------------------------------------

void LioFrontend::apply_results(StepOutputs& out, const time::TimeSnapshot& now) noexcept {
  for (std::size_t i = 0; i < ResultQueue::capacity(); ++i) {
    std::optional<BackendResult> r = channels_.results.try_pop();
    if (!r) return;
    apply(*r, out, now);
  }
}

void LioFrontend::discard(const BackendResult& r, EstimatorEventReason why,
                          const time::TimeSnapshot& now) noexcept {
  ++stats_.results_discarded;
  emit(event("LioResultDiscarded", now)
           .reason(why)
           .value("kind", static_cast<double>(r.kind))
           .value("seq", static_cast<double>(r.seq))
           .value("result_epoch", r.epoch)
           .value("sensor_time_s", seconds(r.t)));
}

void LioFrontend::apply(const BackendResult& r, StepOutputs& out,
                        const time::TimeSnapshot& now) noexcept {
  if (is_scan_result(r.kind)) {
    // Results come back in admission order, so a scan result answers the oldest scan in flight.
    const std::uint64_t oldest = seq_ - stats_.scans_in_flight + 1;
    assert(stats_.scans_in_flight > 0 && r.seq == oldest && "backend answered a scan out of order");
    if (stats_.scans_in_flight == 0 || r.seq != oldest)
      return discard(r, EstimatorEventReason::kUnexpectedSequence, now);
    --stats_.scans_in_flight;  // a stale result still ends its scan's flight
  }
  const bool restart_result =
      r.kind == BackendResultKind::kRestartSeeded || r.kind == BackendResultKind::kRestartRejected;
  // The newest requested epoch only; while a restart is pending, only its answer (frontend.hpp).
  if (r.epoch != requested_epoch_ || restart_result != (requested_epoch_ != epoch_)) {
    return discard(r, EstimatorEventReason::kStaleEpoch, now);
  }
  ++stats_.results_applied;
  switch (r.kind) {
    case BackendResultKind::kImuInitialized:
      position_sigma_m_ = r.position_sigma_m;
      if (r.snapshot && !predictor_.latest()) predictor_.align(*r.snapshot);
      return;
    case BackendResultKind::kRestartSeeded:
    case BackendResultKind::kRestartRejected:
      return apply_restart(r, out, now);
    case BackendResultKind::kScanNotProcessed:  // accounting only (the backend's LioScan event)
      return;
    case BackendResultKind::kScanGood:
    case BackendResultKind::kScanDegenerate:
    case BackendResultKind::kScanEmpty:
    case BackendResultKind::kMapReady:
      return apply_scan(r, out, now);
  }
}

void LioFrontend::apply_scan(const BackendResult& r, StepOutputs& out,
                             const time::TimeSnapshot& now) noexcept {
  position_sigma_m_ = r.position_sigma_m;
  last_report_ = r.report ? *r.report : kNoReport;  // no correction: no information
  using K = BackendResultKind;
  LioEvent::Kind kind = r.kind == K::kScanGood    ? LioEvent::Kind::kScanGood
                        : r.kind == K::kScanEmpty ? LioEvent::Kind::kScanEmpty
                        : r.kind == K::kMapReady  ? LioEvent::Kind::kMapReady
                                                  : LioEvent::Kind::kScanDegenerate;
  bool applied = false;
  // D33/O15 age, see "Gap semantics" in frontend.hpp. No IMU yet is unreachable (F22 admission).
  assert(last_imu_ && "scan result before any IMU sample");
  const time::Duration age = last_imu_ ? last_imu_->t - r.t : time::Duration{0};
  if (r.kind == BackendResultKind::kScanGood && age > cfg_.lifecycle.gap_degraded) {
    // Too old: a rejected correction, never offered to the predictor (fail closed).
    emit(event("ScanResultTooOld", now)
             .reason(EstimatorEventReason::kResultTooOld)
             .value("sensor_time_s", seconds(r.t))
             .value("age_s", time::to_seconds(age))
             .value("gap_degraded_s", time::to_seconds(cfg_.lifecycle.gap_degraded)));
    kind = LioEvent::Kind::kScanCorrectionRejected;
  } else if (r.kind == BackendResultKind::kScanGood) {
    // D31: a scan counts as corrected only if the output predictor applied its snapshot (matched by
    // stamp). A good result without a snapshot is a wiring error and fails closed the same way.
    const Result<void, PredictorReason> c =
        r.snapshot ? predictor_.on_correction(*r.snapshot)
                   : Result<void, PredictorReason>(std::unexpected(PredictorReason::kNonFinite));
    applied = c.has_value();
    if (applied) {
      last_good_scan_t_ = r.t;
    } else {
      emit(event("PredictorCorrectionRejected", now)
               .reason(c.error())
               .value("sensor_time_s", seconds(r.t)));
      kind = LioEvent::Kind::kScanCorrectionRejected;  // like degenerate; gap reference stays
    }
  }
  const Transition tr = lifecycle_.on(LioEvent{kind, r.t, position_sigma_m_});
  on_transition(tr, r.t, now, out, r.report);
  // Odometry only for an applied good scan leaving TRACKING (D28 beta: no downstream quality gate).
  if (applied && lifecycle_.state() == LioState::kTracking && r.odometry && r.report) {
    const BaseOdometry& o = *r.odometry;
    out.odometry = OdometryOutput{r.t,         epoch_,         predictor_.reset_counter(),
                                  o.p_world_m, o.q_world_base, o.v_world_mps,
                                  o.pose_cov,  o.vel_cov,      r.report->quality};
  }
}

void LioFrontend::apply_restart(const BackendResult& r, StepOutputs& out,
                                const time::TimeSnapshot& now) noexcept {
  const SeedPose seed =
      pending_seed_.value_or(SeedPose{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()});
  pending_seed_.reset();
  assert((r.kind == BackendResultKind::kRestartRejected || r.snapshot) &&
         "kRestartSeeded without snapshot");
  const double seed_age_s = last_imu_ ? time::to_seconds(last_imu_->t - r.t) : 0.0;
  // D33/O16: reset_to would rewind the output to an answer the predictor no longer covers: refused.
  const bool stale = r.snapshot && !predictor_.covers(r.snapshot->t);
  if (r.kind == BackendResultKind::kRestartRejected || !r.snapshot || stale) {
    requested_epoch_ = epoch_;  // no restart pending any more; the state stays RESTARTING
    EventBuilder b = event("LioRestartRejected", now);
    b.reason(stale ? EstimatorEventReason::kRestartAnswerStale
                   : EstimatorEventReason::kRestartRejected);
    b.value("new_epoch", r.epoch).value("sensor_time_s", seconds(r.t));
    emit(stale ? b.value("seed_age_s", seed_age_s) : b);
    out.restart =
        std::unexpected(stale ? EstimatorReason::kStaleAnswer : EstimatorReason::kNotFinite);
    return;
  }
  const ResetDelta delta = predictor_.reset_to(*r.snapshot);
  flush_spam(now);  // end of the old epoch: summaries, under the old epoch id
  const std::uint32_t old_epoch = epoch_;
  epoch_ = r.epoch;
  last_report_ = kNoReport;
  rate_reported_ = false;
  imu_period_samples_ = 0;
  position_sigma_m_ = r.position_sigma_m;
  const Transition tr =
      lifecycle_.on(LioEvent{LioEvent::Kind::kRestartSeeded, r.t, position_sigma_m_});
  emit(event("LioRestart", now)
           .reason(LioReason::kRestartSeeded)
           .value("old_epoch", old_epoch)
           .value("new_epoch", epoch_)
           .value("seed_x_m", seed.p_world_m.x())
           .value("seed_y_m", seed.p_world_m.y())
           .value("seed_z_m", seed.p_world_m.z())
           .value("seed_yaw_rad", yaw_of(seed.q_world_base.toRotationMatrix()))
           .value("delta_x_m", delta.position_m.x())
           .value("delta_y_m", delta.position_m.y())
           .value("delta_z_m", delta.position_m.z())
           .value("delta_yaw_rad", delta.yaw_rad)
           .value("seed_age_s", seed_age_s));
  on_transition(tr, r.t, now, out, std::nullopt);
  out.restart = delta;
}

// --- public --------------------------------------------------------------------------------------

Result<StepOutputs, EstimatorReason> LioFrontend::push_imu(const ImuInput& imu,
                                                           const time::TimeSnapshot& now) {
  if (!imu.gyro_rad_s.allFinite() || !imu.accel_mps2.allFinite()) {
    return std::unexpected(reject(EstimatorReason::kNotFinite, kStreamImu, imu.t, now));
  }
  if (last_imu_ && imu.t < last_imu_->t)
    return std::unexpected(reject(EstimatorReason::kOutOfOrder, kStreamImu, imu.t, now));
  ++stats_.imu_accepted;

  StepOutputs out;
  apply_results(out, now);
  std::optional<OutputSample> produced;
  if (last_imu_ && imu.t == last_imu_->t) {
    if (spam_due(kDuplicateSlot)) {
      emit(event("ImuDuplicate", now)
               .reason(EstimatorEventReason::kImuDuplicate)
               .value("sensor_time_s", seconds(imu.t))
               .value("count", static_cast<double>(spam_[kDuplicateSlot].count)));
    }
  } else {
    if (last_imu_) {
      const time::Duration dt = imu.t - last_imu_->t;
      if (dt > cfg_.imu_max_gap) {
        // Reported only; the LiDAR-gap rule (F22) is the lifecycle's kImuTick below.
        emit(event("ImuGap", now)
                 .reason(EstimatorEventReason::kImuGap)
                 .value("gap_s", time::to_seconds(dt)));
      }
      track_rate(dt, now);
      const double dt_s = time::to_seconds(dt);
      const ImuDelta delta{imu.t, 0.5 * (last_imu_->gyro_rad_s + imu.gyro_rad_s) * dt_s,
                           0.5 * (last_imu_->accel_mps2 + imu.accel_mps2) * dt_s, dt_s};
      produced = predictor_.on_imu(delta);  // nullopt (no change) before the predictor is aligned
    }
    last_imu_ = imu;
    // The backend's copy (time-ordered, de-duplicated, finite). A full queue drops only the copy:
    // the backend sees an IMU gap, that scan's prediction fails (fail closed); ingest goes on.
    BackendRequest copy{imu};
    if (!channels_.requests.try_push(std::move(copy))) {
      ++stats_.imu_copies_dropped;
      request_queue_full(kStreamImu, imu.t, now);
    }
  }

  const Transition tr = lifecycle_.on(LioEvent{LioEvent::Kind::kImuTick, imu.t, position_sigma_m_});
  on_transition(tr, imu.t, now, out, std::nullopt);
  if (produced) {
    try {
      out.states.push_back(StateOutput{*produced, epoch_, lifecycle_.state()});
    } catch (...) {  // never throws: this output is dropped
      emit(event("AllocationFailed", now)
               .reason(EstimatorEventReason::kAllocationFailed)
               .value("site", 1));
    }
  }
  // Health on the grid next_health_t + k * kHealthPeriod, at most one per sample; the grid skips a
  // stamp jump in O(1). No overflow: range-checked stamps give 0 <= imu.t - next < 9.3e18.
  if (!next_health_t_ || imu.t >= *next_health_t_) {
    out.health = health(imu.t);
    if (!next_health_t_) next_health_t_ = imu.t;
    const std::int64_t periods = (imu.t - *next_health_t_).ns / limits::kHealthPeriod.ns + 1;
    next_health_t_ = *next_health_t_ + time::Duration{periods * limits::kHealthPeriod.ns};
  }
  return out;
}

Result<StepOutputs, EstimatorReason> LioFrontend::push_scan(ScanInput&& in,
                                                            const time::TimeSnapshot& now) {
  if (in.points.size() > limits::kMaxScanPoints)
    return std::unexpected(reject(EstimatorReason::kTooManyPoints, kStreamScan, in.end, now));
  if (!std::all_of(in.points.begin(), in.points.end(), [](const auto& p) { return p.allFinite(); }))
    return std::unexpected(reject(EstimatorReason::kNotFinite, kStreamScan, in.end, now));
  if (in.end < in.start || (last_scan_end_ && in.end < *last_scan_end_))
    return std::unexpected(reject(EstimatorReason::kOutOfOrder, kStreamScan, in.end, now));
  // F22: no admitted scan ends after the newest IMU sample, so no gap reference runs ahead of it.
  if (!last_imu_ || in.end > last_imu_->t)
    return std::unexpected(reject(EstimatorReason::kScanAheadOfImu, kStreamScan, in.end, now));
  // Drop-newest (§3.5). Only this thread pushes and `popped` only lags: a free slot stays free.
  const concurrency::QueueStats q = channels_.requests.stats();
  if (stats_.scans_in_flight >= limits::kMaxScansInFlight ||
      q.pushed - q.popped >= RequestQueue::capacity()) {
    ++stats_.scans_refused_busy;
    return std::unexpected(reject(EstimatorReason::kBackendBusy, kStreamScan, in.end, now));
  }

  StepOutputs out;
  apply_results(out, now);
  const LioState s = lifecycle_.state();
  const MapPolicy policy = (s == LioState::kLost || s == LioState::kRestarting)
                               ? MapPolicy::kFrozen
                               : MapPolicy::kInsert;
  const time::SensorTime end = in.end;
  BackendRequest job{ScanJob{seq_ + 1, requested_epoch_, policy, std::move(in)}};
  if (!channels_.requests.try_push(std::move(job))) {  // unreachable (slot checked): fail closed
    assert(false && "request queue full although a free slot was checked");
    ++stats_.scans_refused_busy;
    request_queue_full(kStreamScan, end, now);
    return out;
  }
  ++seq_;
  ++stats_.scans_in_flight;
  ++stats_.scans_admitted;
  last_scan_end_ = end;
  return out;
}

Result<void, EstimatorReason> LioFrontend::restart(const SeedPose& seed,
                                                   const time::TimeSnapshot& now) {
  const time::SensorTime t = last_imu_ ? last_imu_->t : time::SensorTime{};
  const std::optional<OutputSample> latest = predictor_.latest();
  if (lifecycle_.state() != LioState::kRestarting || !latest || !last_imu_ ||
      requested_epoch_ != epoch_) {
    return std::unexpected(reject(EstimatorReason::kWrongState, kStreamRestart, t, now));
  }
  if (!seed.p_world_m.allFinite() || !usable(seed.q_world_base)) {
    return std::unexpected(reject(EstimatorReason::kNotFinite, kStreamRestart, t, now));
  }
  const SeedPose normalized{seed.p_world_m, seed.q_world_base.normalized()};
  // The old world's attitude and velocity at t: the predictor's newest output, including the last
  // applied correction (latest(), not the value on_imu returned).
  BackendRequest cmd{
      RestartCommand{epoch_ + 1, t, normalized, latest->q_world_imu, latest->v_world_mps}};
  if (!channels_.requests.try_push(std::move(cmd))) {  // one decision, one event: LioInputRejected
    ++stats_.requests_rejected;
    return std::unexpected(reject(EstimatorReason::kBackendBusy, kStreamRestart, t, now));
  }
  requested_epoch_ = epoch_ + 1;
  pending_seed_ = normalized;
  return {};
}

}  // namespace uavnav::lio
