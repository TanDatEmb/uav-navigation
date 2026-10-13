#include "uavnav/lio_node/ingest.hpp"

#include <cassert>
#include <utility>

#include "uavnav/lio/limits.hpp"
#include "uavnav/lio_node/limits.hpp"

namespace uavnav::lio_node {

namespace {

using events::Component;
using events::EventBuilder;

/// "stream" value of LioInputRejected (the frontend's numbering: 0 IMU, 1 scan).
constexpr int kStreamImu = 0;
constexpr int kStreamScan = 1;

/// Rate limit of the per-sample events: the 1st and every kEventSummaryEvery-th occurrence (as the frontend's).
constexpr bool spam_due(std::uint64_t count) noexcept {
  return count == 1 || count % lio::limits::kEventSummaryEvery == 0;
}

/// The atomic try-enter of one call (see ingest.hpp "Threads"). One compare-exchange false -> true: the winner
/// enters (acquire: it sees every write of the previous call), a loser leaves the flag untouched and is refused.
/// The destructor of the winner releases it (release: its writes are visible to the next winner). No lock, no
/// wait, no retry.
class EnterGuard {
 public:
  explicit EnterGuard(std::atomic<bool>& busy) noexcept : busy_(busy) {
    bool expected = false;
    entered_ = busy_.compare_exchange_strong(expected, true, std::memory_order_acquire, std::memory_order_relaxed);
  }
  ~EnterGuard() {
    if (entered_) busy_.store(false, std::memory_order_release);
  }
  EnterGuard(const EnterGuard&) = delete;
  EnterGuard& operator=(const EnterGuard&) = delete;
  explicit operator bool() const noexcept { return entered_; }

 private:
  std::atomic<bool>& busy_;
  bool entered_;
};

/// A stamp later than now.ros + kMaxStampAhead. Sensor and ROS time are one clock in SITL (D29), so the two int64
/// counts are compared directly. Both come from the range-checked converters (<= 9e18 + 1e9 ns), so adding 0.5 s
/// cannot overflow.
bool in_future(time::SensorTime t, const time::TimeSnapshot& now) noexcept {
  return t.ns > now.ros.ns + limits::kMaxStampAhead.ns;
}

}  // namespace

// --- StateDecimator ----------------------------------------------------------------------------------------

StateDecimator::StateDecimator(time::Duration period) noexcept : period_ns_(period.ns) {
  assert(period.ns > 0 && "StateDecimator needs a positive period");
}

bool StateDecimator::due(time::SensorTime t) noexcept {
  std::int64_t cell = t.ns / period_ns_;
  if (t.ns % period_ns_ != 0 && t.ns < 0) --cell;  // floor, also before the epoch of the clock
  if (last_cell_ && cell <= *last_cell_) return false;
  last_cell_ = cell;
  return true;
}

// --- OutputCollector ---------------------------------------------------------------------------------------

OutputCollector::OutputCollector(const Eigen::Isometry3d& base_T_imu, time::Duration state_period) noexcept
    : r_bi_(base_T_imu.linear()),
      q_ib_(Eigen::Quaterniond(base_T_imu.linear()).conjugate().normalized()),
      t_bi_(base_T_imu.translation()),
      decimator_(state_period),
      last_reset_{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0} {}

LioStateOut OutputCollector::map(const lio::StateOutput& s, const Eigen::Vector3d& gyro_imu_rad_s) const noexcept {
  // ingest.hpp "State mapping": R_wb = R_wi R_bi^T; p_wb = p_wi - R_wb t_bi; w_b = R_bi gyro;
  // v_b = R_wb^T v_wi + w_b x r with r = -t_bi (IMU -> base origin).
  const Eigen::Quaterniond q_wb = (s.sample.q_world_imu.normalized() * q_ib_).normalized();
  const Eigen::Matrix3d r_wb = q_wb.toRotationMatrix();
  const Eigen::Vector3d w_b = r_bi_ * gyro_imu_rad_s;
  const Eigen::Vector3d r = -t_bi_;
  LioStateOut out;
  out.t = s.sample.t;
  out.epoch = s.epoch;
  out.reset_counter = s.sample.reset_counter;
  out.state = s.state;
  out.p_world_base = s.sample.p_world_m - r_wb * t_bi_;
  out.q_world_base = q_wb;
  out.v_base_mps = r_wb.transpose() * s.sample.v_world_mps + w_b.cross(r);
  out.w_base_rad_s = w_b;
  out.last_reset = last_reset_;
  return out;
}

void OutputCollector::add(lio::StepOutputs&& from, const Eigen::Vector3d& gyro_imu_rad_s, IngestOutputs& into) {
  if (from.restart && from.restart->has_value()) last_reset_ = **from.restart;
  for (const lio::StateOutput& s : from.states) {
    if (decimator_.due(s.sample.t)) into.states.push_back(map(s, gyro_imu_rad_s));
  }
  if (from.odometry) {
    if (into.odometry) ++odometry_overwritten_;
    into.odometry = std::move(from.odometry);
  }
  if (from.health) into.health = std::move(from.health);
}

// --- SharedRosTime -----------------------------------------------------------------------------------------

time::TimeSnapshot SharedRosTime::snapshot() const noexcept {
  return time::TimeSnapshot{time::steady_now(), time::RosTime{ros_ns_.load(std::memory_order_relaxed)}};
}

// --- LioIngest ---------------------------------------------------------------------------------------------

Result<std::unique_ptr<LioIngest>, config::ConfigError> LioIngest::create(const lio::LioConfig& config,
                                                                          const LioNodeConfig& node,
                                                                          events::EventRecorder& events,
                                                                          SharedRosTime& ros_time) {
  const Eigen::Isometry3d base_T_imu = compose_base_T_imu(node, config);
  auto runner = lio::LioRunner::create(config, base_T_imu, imu_T_lidar(config), events,
                                       [&ros_time] { return ros_time.snapshot(); });
  if (!runner) return std::unexpected(runner.error());
  return std::unique_ptr<LioIngest>(new LioIngest(std::move(*runner), base_T_imu, events, ros_time));
}

LioIngest::LioIngest(std::unique_ptr<lio::LioRunner> runner, const Eigen::Isometry3d& base_T_imu,
                     events::EventRecorder& events, SharedRosTime& ros_time) noexcept
    : events_(events),
      ros_time_(ros_time),
      collector_(base_T_imu, limits::kStatePeriod),
      runner_(std::move(runner)) {}

LioIngest::~LioIngest() { stop(); }

Result<IngestOutputs, IngestReason> LioIngest::on_imu(const lio::ImuInput& imu, const time::TimeSnapshot& now) {
  const EnterGuard entered(busy_);
  if (!entered) return refuse_reentered(now);
  ros_time_.store(now.ros);
  expire_pending(now);  // before this sample can cover it
  if (in_future(imu.t, now)) return refuse_future(kStreamImu, imu.t, now);
  auto pushed = runner_->push_imu(imu, now);
  if (!pushed) return std::unexpected(IngestReason::kRunnerRejected);  // the core emitted its event
  newest_imu_ = imu.t;  // accepted: not before the previous accepted stamp
  last_gyro_ = imu.gyro_rad_s;
  collect(std::move(*pushed));
  push_pending_if_covered(now);
  return take_outputs();
}

Result<IngestOutputs, IngestReason> LioIngest::on_scan(lio::ScanInput&& scan, const time::TimeSnapshot& now) {
  const EnterGuard entered(busy_);
  if (!entered) return refuse_reentered(now);
  ros_time_.store(now.ros);
  expire_pending(now);
  if (in_future(scan.end, now)) return refuse_future(kStreamScan, scan.end, now);
  // The rule of every call. Coverage changes only in on_imu, which already retried, so this pushes nothing in
  // practice; a scan still waiting here is uncovered and the newer one supersedes it.
  push_pending_if_covered(now);
  if (pending_) drop_pending(IngestReason::kSuperseded, now);
  if (!covered(scan.end)) {
    pending_.emplace(Pending{std::move(scan), now.ros});
    bump(scans_waited_);
    return take_outputs();
  }
  auto pushed = runner_->push_scan(std::move(scan), now);
  if (!pushed) return std::unexpected(IngestReason::kRunnerRejected);  // outputs_ waits for the next call
  collect(std::move(*pushed));
  return take_outputs();
}

void LioIngest::stop() noexcept {
  if (stopped_.exchange(true, std::memory_order_acq_rel)) return;
  runner_->stop();
  const lio::FrontendStats f = runner_->frontend().stats();
  const concurrency::QueueStats rq = runner_->request_queue_stats();
  const concurrency::QueueStats rs = runner_->result_queue_stats();
  const IngestStats s = stats();
  const auto d = [](std::uint64_t v) { return static_cast<double>(v); };
  emit(event("IngestSummary", ros_time_.snapshot())
           .reason(IngestReason::kAccepted)
           .value("scans_waited", d(s.scans_waited))
           .value("scans_dropped_timeout", d(s.scans_dropped_timeout))
           .value("scans_superseded", d(s.scans_superseded))
           .value("odometry_overwritten", d(s.odometry_overwritten))
           .value("reentered", d(s.reentered))
           .value("stamps_in_future", d(stamps_in_future_.load(std::memory_order_relaxed)))
           .value("scans_admitted", d(f.scans_admitted))
           .value("scans_refused_busy", d(f.scans_refused_busy))
           .value("imu_copies_dropped", d(f.imu_copies_dropped))
           .value("request_pushed", d(rq.pushed))
           .value("request_popped", d(rq.popped))
           .value("request_rejected", d(rq.rejected))
           .value("result_pushed", d(rs.pushed))
           .value("result_popped", d(rs.popped))
           .value("result_rejected", d(rs.rejected)));
}

IngestStats LioIngest::stats() const noexcept {
  constexpr auto kRelaxed = std::memory_order_relaxed;
  return IngestStats{scans_waited_.load(kRelaxed), scans_dropped_timeout_.load(kRelaxed),
                     scans_superseded_.load(kRelaxed), odometry_overwritten_.load(kRelaxed),
                     reentered_.load(kRelaxed)};
}

// --- private -----------------------------------------------------------------------------------------------

Result<IngestOutputs, IngestReason> LioIngest::refuse_reentered(const time::TimeSnapshot& now) noexcept {
  // Not entered: only the atomic counter and the thread-safe recorder; no epoch (the runner is not read).
  const std::uint64_t n = reentered_.fetch_add(1, std::memory_order_relaxed) + 1;
  if (spam_due(n)) {
    emit(EventBuilder(Component::kLio, "LioIngestReentered", now)
             .reason(IngestReason::kReentered)
             .value("count", static_cast<double>(n)));
  }
  return std::unexpected(IngestReason::kReentered);
}

Result<IngestOutputs, IngestReason> LioIngest::refuse_future(int stream, time::SensorTime t,
                                                             const time::TimeSnapshot& now) {
  bump(stamps_in_future_);
  const std::uint64_t n = stamps_in_future_.load(std::memory_order_relaxed);
  if (spam_due(n)) {
    emit(event("LioInputRejected", now)
             .reason(IngestReason::kStampInFuture)
             .value("stream", stream)
             .value("sensor_time_s", time::to_seconds(t))
             .value("ros_time_s", time::to_seconds(now.ros))
             .value("count", static_cast<double>(n)));
  }
  return std::unexpected(IngestReason::kStampInFuture);
}

void LioIngest::expire_pending(const time::TimeSnapshot& now) noexcept {
  if (pending_ && now.ros - pending_->arrived > limits::kScanImuWaitMax) {
    drop_pending(IngestReason::kImuCoverageTimeout, now);
  }
}

void LioIngest::push_pending_if_covered(const time::TimeSnapshot& now) {
  if (!pending_ || !covered(pending_->scan.end)) return;
  lio::ScanInput scan = std::move(pending_->scan);
  pending_.reset();
  auto pushed = runner_->push_scan(std::move(scan), now);
  if (pushed) collect(std::move(*pushed));  // a refusal drops the scan; the core emitted its event
}

void LioIngest::drop_pending(IngestReason why, const time::TimeSnapshot& now) noexcept {
  emit(event("LioScanDropped", now)
           .reason(why)
           .value("scan_end_s", time::to_seconds(pending_->scan.end))
           .value("waited_s", time::to_seconds(now.ros - pending_->arrived)));
  bump(why == IngestReason::kSuperseded ? scans_superseded_ : scans_dropped_timeout_);
  pending_.reset();
}

void LioIngest::collect(lio::StepOutputs&& out) {
  collector_.add(std::move(out), last_gyro_, outputs_);
  odometry_overwritten_.store(collector_.odometry_overwritten(), std::memory_order_relaxed);
}

events::EventBuilder LioIngest::event(std::string_view name, const time::TimeSnapshot& now) const noexcept {
  events::EventIdentity id;
  id.lio_epoch = runner_->epoch();
  return EventBuilder(Component::kLio, name, now).identity(id);
}

void LioIngest::emit(const events::EventBuilder& b) noexcept { (void)events_.emit(b.build()); }

}  // namespace uavnav::lio_node
