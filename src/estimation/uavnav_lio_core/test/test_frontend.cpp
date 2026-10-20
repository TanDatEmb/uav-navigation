// LioFrontend (SYSTEM_DESIGN §3.5, D30/O14, D31): the ingest-thread half of the LIO split. A
// test-side FakeBackend pops `channels.requests` and answers with crafted BackendResults; no reused
// math runs here.
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "synthetic_scene.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/lio/channels.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/frontend.hpp"
#include "uavnav/lio/limits.hpp"

using namespace uavnav;
using namespace uavnav::lio;

namespace {

// ---------------------------------------------------------------------------------------------------
// Event capture: the recorder writes on its own thread; tests flush() and then read the store.

struct Store {
  std::mutex mutex;
  std::vector<events::EventRecord> records;
};

class FakeSink final : public events::EventSink {
 public:
  explicit FakeSink(std::shared_ptr<Store> store) : store_(std::move(store)) {}
  Result<void, events::SinkError> write(std::span<const events::EventRecord> batch) override {
    const std::scoped_lock lock(store_->mutex);
    store_->records.insert(store_->records.end(), batch.begin(), batch.end());
    return {};
  }

 private:
  std::shared_ptr<Store> store_;
};

LioConfig DefaultConfig() {
  const auto values = config::load_params_file(UAVNAV_LIO_DEFAULT_YAML, kLioSpecs);
  EXPECT_TRUE(values.has_value());
  const auto cfg = load_lio_config(*values);
  EXPECT_TRUE(cfg.has_value());
  return *cfg;
}

/// Beta thresholds except a wider LiDAR gap (1.0 s / 1.5 s, inside the kLioSpecs bounds). With the
/// beta 0.25 s a correction older than the 300 ms predictor buffer can only arrive after the gap
/// rule already left TRACKING, so the D31 run of rejected corrections inside TRACKING needs a gap
/// threshold above the buffer span.
FrontendConfig WideGapConfig() {
  FrontendConfig c = frontend_config(DefaultConfig());
  c.lifecycle.gap_degraded = time::seconds(1);
  c.lifecycle.gap_lost = time::milliseconds(1500);
  return c;
}

constexpr time::Duration kImuPeriod = time::milliseconds(5);     // 200 Hz
constexpr time::Duration kScanPeriod = time::milliseconds(100);  // 10 Hz

constexpr time::SensorTime Ms(std::int64_t ms) { return time::SensorTime{ms * 1'000'000}; }

time::TimeSnapshot Snap(time::SensorTime t) {
  return {time::SteadyTime{3 * t.ns}, time::RosTime{t.ns + 7'000'000'000}};
}

double Value(const events::EventRecord& r, std::string_view key) {
  for (std::uint8_t i = 0; i < r.value_count; ++i) {
    if (r.values[i].key == key) return r.values[i].value;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  EXPECT_TRUE(in.good()) << path;
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

/// The stationary scene's ESKF state at t (level IMU at the world origin, no biases), moved by
/// `dp`.
EstimatorSnapshot Still(time::SensorTime t, const Eigen::Vector3d& dp = Eigen::Vector3d::Zero()) {
  return EstimatorSnapshot{t,
                           Eigen::Quaterniond::Identity(),
                           Eigen::Vector3d::Zero(),
                           dp,
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(),
                           Eigen::Vector3d(0.0, 0.0, -scene::kGravity)};
}

constexpr DegeneracyReport kGoodReport{2.0e5, 5.0e6, false, 80};
constexpr DegeneracyReport kDegenerateReport{1.0e3, 5.0e6, true, 1};

// ---------------------------------------------------------------------------------------------------
// Test-side backend: pops the requests (when asked) and answers with crafted results.

class FakeBackend {
 public:
  explicit FakeBackend(LioChannels& channels) : channels_(channels) {}

  /// Pops every queued request.
  void Drain() {
    while (auto r = channels_.requests.try_pop()) {
      if (std::holds_alternative<ImuInput>(*r)) imu_.push_back(std::get<ImuInput>(*r).t);
      if (auto* job = std::get_if<ScanJob>(&*r))
        jobs_.push_back(Job{job->seq, job->epoch, job->map_policy, job->scan.end});
      if (auto* cmd = std::get_if<RestartCommand>(&*r)) restarts_.push_back(*cmd);
    }
  }

  struct Job {
    std::uint64_t seq;
    std::uint32_t epoch;
    MapPolicy map_policy;
    time::SensorTime end;
  };

  /// The oldest admitted scan not answered yet.
  Job NextJob() {
    Drain();
    EXPECT_LT(answered_, jobs_.size());
    if (answered_ >= jobs_.size()) return Job{0, 0, MapPolicy::kInsert, {}};
    return jobs_[answered_++];
  }

  void Push(BackendResult&& r) {
    EXPECT_TRUE(channels_.results.try_push(std::move(r)).has_value());
  }

  /// Answers the oldest unanswered scan. kScanGood carries a snapshot at the scan end (or
  /// `snapshot`), a good report and odometry; kScanDegenerate a degenerate report; kMapReady and
  /// kScanEmpty no report.
  Job AnswerScan(BackendResultKind kind, std::optional<EstimatorSnapshot> snapshot = std::nullopt) {
    const Job job = NextJob();
    BackendResult r{kind,         job.epoch,    job.seq, job.end,
                    std::nullopt, std::nullopt, 0.03,    std::nullopt};
    if (kind != BackendResultKind::kScanNotProcessed)
      r.snapshot = snapshot ? *snapshot : Still(job.end);
    if (kind == BackendResultKind::kScanGood) {
      r.report = kGoodReport;
      r.odometry =
          BaseOdometry{Eigen::Vector3d(-0.10, 0.0, -0.05), Eigen::Quaterniond::Identity(),
                       Eigen::Vector3d::Zero(), Eigen::Matrix<double, 6, 6>::Identity() * 1e-4,
                       Eigen::Matrix3d::Identity() * 1e-4};
    }
    if (kind == BackendResultKind::kScanDegenerate) r.report = kDegenerateReport;
    Push(std::move(r));
    return job;
  }

  void AnswerImuInitialized(time::SensorTime t) {
    Push(BackendResult{BackendResultKind::kImuInitialized, 1, 0, t, Still(t), std::nullopt, 0.03,
                       std::nullopt});
  }

  const std::vector<time::SensorTime>& imu() const { return imu_; }
  const std::vector<Job>& jobs() const { return jobs_; }
  const std::vector<RestartCommand>& restarts() const { return restarts_; }

 private:
  LioChannels& channels_;
  std::vector<time::SensorTime> imu_;
  std::vector<Job> jobs_;
  std::size_t answered_{0};
  std::vector<RestartCommand> restarts_;
};

// ---------------------------------------------------------------------------------------------------

class Rig {
 public:
  explicit Rig(const FrontendConfig& cfg = frontend_config(DefaultConfig()))
      : store_(std::make_shared<Store>()),
        channels_(std::make_unique<LioChannels>()),
        backend_(*channels_) {
    auto recorder = events::EventRecorder::create(std::make_unique<FakeSink>(store_));
    EXPECT_TRUE(recorder.has_value());
    recorder_ = std::move(*recorder);
    frontend_ = std::make_unique<LioFrontend>(cfg, scene::base_T_imu(), *channels_, *recorder_);
  }

  LioFrontend& fe() { return *frontend_; }
  FakeBackend& backend() { return backend_; }
  LioChannels& channels() { return *channels_; }
  time::SensorTime now() const { return newest_; }

  Result<StepOutputs, EstimatorReason> Imu(time::SensorTime t) {
    auto r = frontend_->push_imu(scene::make_imu({}, t), Snap(t));
    if (r) {
      outputs_.push_back(*r);
      newest_ = t;
    }
    return r;
  }

  /// IMU samples every 5 ms after the newest one, up to and including `end` (the first at `end`
  /// when none yet).
  void ImuUntil(time::SensorTime end) {
    time::SensorTime t = started_ ? newest_ + kImuPeriod : end;
    started_ = true;
    for (; t <= end; t = t + kImuPeriod) EXPECT_TRUE(Imu(t).has_value()) << t.ns;
  }

  Result<StepOutputs, EstimatorReason> Scan(time::SensorTime start, time::SensorTime end) {
    ScanInput scan = scene::make_scan(scene::Pose{}, start, end);
    scan.points.resize(10);  // the frontend never looks at the geometry
    auto r = frontend_->push_scan(std::move(scan), Snap(end));
    if (r) outputs_.push_back(*r);
    return r;
  }

  StepOutputs Poll() {
    StepOutputs out = frontend_->poll_results(Snap(newest_));
    outputs_.push_back(out);
    return out;
  }

  /// IMU from `t` on; the backend reports the IMU initialisation at `t`.
  void Initialise(time::SensorTime t) {
    ImuUntil(t);
    backend_.Drain();
    backend_.AnswerImuInitialized(t);
    Poll();
  }

  /// One scan period: IMU up to `end`, the scan (end - 100 ms, end], the backend's answer, poll.
  StepOutputs ScanAnswered(time::SensorTime end, BackendResultKind kind,
                           std::optional<EstimatorSnapshot> snapshot = std::nullopt) {
    ImuUntil(end);
    const auto admitted = Scan(end - kScanPeriod, end);
    EXPECT_TRUE(admitted.has_value()) << end.ns;
    backend_.AnswerScan(kind, snapshot);
    return Poll();
  }

  /// IMU from 9.0 s, map ready at 9.5 s, five good scans: TRACKING with the last applied scan end
  /// at 10.0 s.
  void ToTracking() {
    Initialise(Ms(9'000));
    ScanAnswered(Ms(9'500), BackendResultKind::kMapReady);
    for (std::int64_t end = 9'600; end <= 10'000; end += 100)
      ScanAnswered(Ms(end), BackendResultKind::kScanGood);
    ASSERT_EQ(frontend_->state(), LioState::kTracking);
    ASSERT_EQ(newest_, Ms(10'000));
  }

  /// From LOST: a non-empty scan every 100 ms (answered kScanGood on the frozen map, not applied) until the IMU tick
  /// restart_dwell (2 s) after the first one: RESTARTING, and that push_imu queued the frontend's own restart command
  /// (not drained). No scan is admitted after the tick.
  void DwellUntilRestarting() {
    ASSERT_EQ(frontend_->state(), LioState::kLost);
    for (int i = 0; i < 30 && frontend_->state() == LioState::kLost; ++i) {
      const time::SensorTime end = newest_ + kScanPeriod;
      ImuUntil(end);
      if (frontend_->state() != LioState::kLost) break;
      ASSERT_TRUE(Scan(end - kScanPeriod, end).has_value());
      backend_.AnswerScan(BackendResultKind::kScanGood);
      Poll();
    }
    ASSERT_EQ(frontend_->state(), LioState::kRestarting);
  }

  /// TRACKING, IMU only until LOST (10.505 s), then non-empty scans from 10.7 s: RESTARTING at the IMU tick 12.7 s,
  /// with the frontend's own restart command queued at 12.7 s. No command before RESTARTING.
  void ToRestarting() {
    ToTracking();
    ImuUntil(Ms(10'600));
    ASSERT_EQ(frontend_->state(), LioState::kLost);
    DwellUntilRestarting();
    ASSERT_EQ(newest_, Ms(12'700));
    backend_.Drain();
    ASSERT_EQ(backend_.restarts().size(), 1U) << "exactly one command, issued at RESTARTING";
    ASSERT_EQ(backend_.restarts()[0].t, Ms(12'700));
  }

  std::vector<events::EventRecord> Events(std::string_view name) {
    recorder_->flush();
    const std::scoped_lock lock(store_->mutex);
    std::vector<events::EventRecord> out;
    for (const auto& r : store_->records) {
      if (r.event == name) out.push_back(r);
    }
    return out;
  }

  const std::vector<StepOutputs>& outputs() const { return outputs_; }

  std::optional<HealthOutput> LastHealth() const {
    for (auto it = outputs_.rbegin(); it != outputs_.rend(); ++it) {
      if (it->health) return it->health;
    }
    return std::nullopt;
  }

  std::optional<StateOutput> LastState() const {
    for (auto it = outputs_.rbegin(); it != outputs_.rend(); ++it) {
      if (!it->states.empty()) return it->states.back();
    }
    return std::nullopt;
  }

 private:
  // Declaration order = destruction in reverse: the frontend goes first, then the channels and the
  // recorder.
  std::shared_ptr<Store> store_;
  std::unique_ptr<events::EventRecorder> recorder_;
  std::unique_ptr<LioChannels> channels_;  // heap: the rings hold their slots inline (channels.hpp)
  FakeBackend backend_;
  std::unique_ptr<LioFrontend> frontend_;
  time::SensorTime newest_{};
  bool started_{false};
  std::vector<StepOutputs> outputs_;
};

/// IMU every 5 ms up to `end`; returns the first IMU stamp at which the state became `target` (or
/// nullopt), and checks one StateOutput per sample.
std::optional<time::SensorTime> ImuUntilState(Rig& h, time::SensorTime end, LioState target,
                                              std::optional<LioReason> reason = std::nullopt) {
  std::optional<time::SensorTime> reached;
  for (time::SensorTime t = h.now() + kImuPeriod; t <= end; t = t + kImuPeriod) {
    const LioState before = h.fe().state();
    const auto r = h.Imu(t);
    EXPECT_TRUE(r.has_value());
    if (!r) continue;
    EXPECT_EQ(r->states.size(), 1U) << t.ns;
    if (!reached && before != target && h.fe().state() == target) {
      reached = t;
      EXPECT_TRUE(r->health.has_value());
      if (reason && r->health) {
        EXPECT_EQ(r->health->reason, *reason) << to_string(r->health->reason);
      }
    }
  }
  return reached;
}

/// The newest restart command the frontend queued (drains the request queue).
RestartCommand IssuedCommand(Rig& h) {
  h.backend().Drain();
  EXPECT_FALSE(h.backend().restarts().empty());
  return h.backend().restarts().empty() ? RestartCommand{} : h.backend().restarts().back();
}

/// The backend's kRestartSeeded for `cmd`, as LioBackend answers: the IMU state of the seed at cmd.t (level, still).
void AnswerSeeded(Rig& h, const RestartCommand& cmd) {
  const Eigen::Vector3d p_imu = cmd.seed.p_world_m + cmd.seed.q_world_base * scene::base_T_imu().translation();
  h.backend().Push(BackendResult{BackendResultKind::kRestartSeeded, cmd.new_epoch, 0, cmd.t, Still(cmd.t, p_imu),
                                 std::nullopt, 0.03, std::nullopt});
}

/// The base_link pose of a predictor output (D47 seed): R_wb = R_wi R_bi^T, p_wb = p_wi - R_wb t_bi.
SeedPose BasePose(const OutputSample& s) {
  const Eigen::Quaterniond q_wb =
      (s.q_world_imu * Eigen::Quaterniond(scene::base_T_imu().linear()).conjugate()).normalized();
  return SeedPose{s.p_world_m - q_wb * scene::base_T_imu().translation(), q_wb};
}

}  // namespace

// ---------------------------------------------------------------------------------------------------

TEST(LioFrontend, EveryAcceptedImuSampleYieldsOneStateOnceAligned) {
  Rig h;
  h.ImuUntil(Ms(9'000));
  h.ImuUntil(Ms(9'100));
  for (const StepOutputs& o : h.outputs()) EXPECT_TRUE(o.states.empty());
  h.backend().Drain();
  EXPECT_EQ(h.backend().imu().size(), 21U);  // every accepted sample is forwarded
  h.backend().AnswerImuInitialized(Ms(9'100));
  EXPECT_TRUE(h.Poll().states.empty());
  for (int i = 1; i <= 40; ++i) {
    const time::SensorTime t = Ms(9'100) + time::Duration{i * kImuPeriod.ns};
    const auto r = h.Imu(t);
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r->states.size(), 1U) << i;
    EXPECT_EQ(r->states[0].sample.t, t);
    EXPECT_EQ(r->states[0].epoch, 1U);
    EXPECT_EQ(r->states[0].state, LioState::kInitializing);
    EXPECT_LT(r->states[0].sample.p_world_m.norm(), 1e-9);
  }
  // A duplicate stamp is accepted (event, tick) but integrates nothing and is not forwarded
  // (de-duplicated FIFO).
  const auto dup = h.Imu(h.now());
  ASSERT_TRUE(dup.has_value());
  EXPECT_TRUE(dup->states.empty());
  h.backend().Drain();
  EXPECT_EQ(h.backend().imu().size(), 61U);
  EXPECT_EQ(h.Events("ImuDuplicate").size(), 1U);
  EXPECT_EQ(h.fe().stats().imu_accepted, 62U);
}

// Review Focus 2 (F22): a backend that never answers reads as a LiDAR gap on IMU time.
TEST(LioFrontend, StalledBackendStillReachesLostOnImuTime) {
  Rig h;
  h.ToTracking();  // last applied scan end 10.0 s
  std::optional<time::SensorTime> degraded;
  std::optional<time::SensorTime> lost;
  int scans = 0;
  for (time::SensorTime t = Ms(10'005); t <= Ms(10'600); t = t + kImuPeriod) {
    const LioState before = h.fe().state();
    const auto r = h.Imu(t);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->states.size(), 1U) << t.ns;
    if (h.fe().state() != before) {
      ASSERT_TRUE(r->health.has_value());
      if (h.fe().state() == LioState::kDegraded) {
        degraded = t;
        EXPECT_EQ(r->health->reason, LioReason::kLidarGapDegraded);
      }
      if (h.fe().state() == LioState::kLost) {
        lost = t;
        EXPECT_EQ(r->health->reason, LioReason::kLidarGapLost);
      }
    }
    if ((t.ns - Ms(10'000).ns) % kScanPeriod.ns == 0) {  // a scan every 100 ms, never answered
      const auto s = h.Scan(t - kScanPeriod, t);
      ++scans;
      if (scans <= 2) {
        EXPECT_TRUE(s.has_value()) << scans;
      } else {
        ASSERT_FALSE(s.has_value()) << scans;
        EXPECT_EQ(s.error(), EstimatorReason::kBackendBusy);
      }
    }
  }
  EXPECT_EQ(degraded, std::optional<time::SensorTime>(Ms(10'255)));  // first IMU t > 10.25 s
  EXPECT_EQ(lost, std::optional<time::SensorTime>(Ms(10'505)));      // first IMU t > 10.5 s
  EXPECT_EQ(h.fe().stats().scans_in_flight, 2U);
  EXPECT_EQ(h.fe().stats().scans_refused_busy, 4U);  // 10.3 .. 10.6 s
}

TEST(LioFrontend, ScanAheadOfImuIsRefusedBeforeTheQueue) {
  Rig h;
  // No IMU at all yet.
  auto r = h.Scan(Ms(9'000), Ms(9'100));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), EstimatorReason::kScanAheadOfImu);
  EXPECT_EQ(h.channels().requests.stats().pushed, 0U);

  h.Initialise(Ms(9'000));
  h.ImuUntil(Ms(9'500));
  const std::uint64_t pushed = h.channels().requests.stats().pushed;
  for (const time::Duration ahead :
       {time::nanoseconds(1), time::milliseconds(100), time::seconds(10)}) {
    r = h.Scan(Ms(9'400), Ms(9'500) + ahead);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), EstimatorReason::kScanAheadOfImu);
  }
  EXPECT_EQ(h.channels().requests.stats().pushed, pushed);
  EXPECT_EQ(h.fe().stats().scans_admitted, 0U);
  EXPECT_EQ(h.fe().stats().scans_in_flight, 0U);
  const auto rejected = h.Events("LioInputRejected");
  ASSERT_EQ(rejected.size(), 1U);  // rate-limited: the first occurrence only
  EXPECT_EQ(rejected[0].reason, "SCAN_AHEAD_OF_IMU");
  // The refused scans did not move the ordering reference: a scan ending at the newest IMU is
  // admitted.
  ASSERT_TRUE(h.Scan(Ms(9'400), Ms(9'500)).has_value());
  EXPECT_EQ(h.channels().requests.stats().pushed, pushed + 1);
}

TEST(LioFrontend, BackendBusyRefusesTheNewestScanAndRecovers) {
  Rig h;
  h.Initialise(Ms(9'000));
  h.ImuUntil(Ms(9'500));
  ASSERT_TRUE(h.Scan(Ms(9'400), Ms(9'500)).has_value());
  h.ImuUntil(Ms(9'600));
  ASSERT_TRUE(h.Scan(Ms(9'500), Ms(9'600)).has_value());
  h.ImuUntil(Ms(9'700));
  const std::uint64_t pushed = h.channels().requests.stats().pushed;
  const auto busy = h.Scan(Ms(9'600), Ms(9'700));
  ASSERT_FALSE(busy.has_value());
  EXPECT_EQ(busy.error(), EstimatorReason::kBackendBusy);
  EXPECT_EQ(h.channels().requests.stats().pushed, pushed);
  EXPECT_EQ(h.fe().stats().scans_in_flight, 2U);
  EXPECT_EQ(h.fe().stats().scans_refused_busy, 1U);
  const auto rejected = h.Events("LioInputRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "BACKEND_BUSY");
  EXPECT_EQ(Value(rejected[0], "stream"), 1.0);

  h.backend().AnswerScan(BackendResultKind::kMapReady);  // one result frees one slot
  h.Poll();
  EXPECT_EQ(h.fe().stats().scans_in_flight, 1U);
  ASSERT_TRUE(h.Scan(Ms(9'600), Ms(9'700)).has_value());
  h.backend().Drain();
  ASSERT_EQ(h.backend().jobs().size(), 3U);
  EXPECT_EQ(h.backend().jobs()[2].seq, 3U);  // the refused scan took no sequence number
  EXPECT_EQ(h.fe().stats().scans_admitted, 3U);
  EXPECT_EQ(h.fe().stats().results_applied, 2U);  // IMU initialisation and the map-ready scan
}

// Review Focus 4: nobody pops the request queue; ingest never blocks and the drop is one
// rate-limited event.
TEST(LioFrontend, FullRequestQueueIsAnEventAndNeverBlocks) {
  Rig h;
  auto run = std::async(std::launch::async, [&h] {
    bool all_ok = true;
    for (int i = 0; i < 600; ++i)
      all_ok = h.Imu(Ms(9'000) + time::Duration{i * kImuPeriod.ns}).has_value() && all_ok;
    return all_ok;
  });
  ASSERT_EQ(run.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  EXPECT_TRUE(run.get());
  const FrontendStats s = h.fe().stats();
  EXPECT_EQ(s.imu_accepted, 600U);
  EXPECT_EQ(s.imu_copies_dropped, 600U - lio::limits::kBackendRequestCapacity);
  EXPECT_EQ(s.requests_rejected, 600U - lio::limits::kBackendRequestCapacity);
  EXPECT_EQ(h.channels().requests.stats().pushed, lio::limits::kBackendRequestCapacity);
  const auto overflow = h.Events("LioQueueOverflow");
  ASSERT_EQ(overflow.size(), 1U);
  EXPECT_EQ(overflow[0].reason, "REQUEST_QUEUE_FULL");
  EXPECT_EQ(Value(overflow[0], "count"), 1.0);
}

// Review Focus 3: once the frontend has issued its restart, results of the old epoch are discarded and only end
// their scan's flight.
TEST(LioFrontend, ResultsOfTheOldEpochAreDiscardedAfterRestart) {
  Rig h;
  h.ToTracking();
  h.ImuUntil(Ms(10'600));  // LOST
  for (std::int64_t end = 10'700; end <= 12'500; end += 100) h.ScanAnswered(Ms(end), BackendResultKind::kScanGood);
  h.ImuUntil(Ms(12'600));
  ASSERT_TRUE(h.Scan(Ms(12'500), Ms(12'600)).has_value());  // epoch 1, in flight
  ASSERT_EQ(h.fe().state(), LioState::kLost);
  h.ImuUntil(Ms(12'700));  // the dwell from 10.7 s ends: RESTARTING, the frontend's own command (epoch 2)
  EXPECT_EQ(h.fe().state(), LioState::kRestarting);
  EXPECT_EQ(h.fe().epoch(), 1U);
  ASSERT_TRUE(h.LastHealth().has_value());
  const Eigen::Vector3d tracking_error = h.LastHealth()->output_tracking_error;

  // The old epoch's scan answers kScanGood with a snapshot 1 m away: discarded, nothing corrected.
  const auto job = h.backend().AnswerScan(BackendResultKind::kScanGood,
                                          Still(Ms(12'600), Eigen::Vector3d(1, 0, 0)));
  EXPECT_EQ(job.epoch, 1U);
  const StepOutputs out = h.Poll();
  EXPECT_FALSE(out.odometry.has_value());
  EXPECT_FALSE(out.restart.has_value());
  EXPECT_EQ(h.fe().state(), LioState::kRestarting);
  EXPECT_EQ(h.fe().stats().scans_in_flight, 0U);
  EXPECT_EQ(h.fe().stats().results_discarded, 1U);
  const auto discarded = h.Events("LioResultDiscarded");
  ASSERT_EQ(discarded.size(), 1U);
  EXPECT_EQ(discarded[0].reason, "STALE_EPOCH");
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());
  h.ImuUntil(Ms(12'800));  // the next health grid point
  ASSERT_TRUE(h.LastHealth().has_value());
  EXPECT_EQ(h.LastHealth()->t, Ms(12'800));
  EXPECT_EQ(h.LastHealth()->output_tracking_error, tracking_error);
  EXPECT_LT(h.LastState()->sample.p_world_m.norm(), 1e-6);

  // The backend confirms the restart: epoch 2, INITIALIZING.
  const RestartCommand cmd = IssuedCommand(h);
  ASSERT_EQ(h.backend().restarts().size(), 1U) << "no second command while one is pending";
  EXPECT_EQ(cmd.new_epoch, 2U);
  EXPECT_EQ(cmd.t, Ms(12'700));
  AnswerSeeded(h, cmd);
  const StepOutputs seeded = h.Poll();
  ASSERT_TRUE(seeded.restart.has_value());
  EXPECT_TRUE(seeded.restart->has_value());
  EXPECT_EQ(h.fe().state(), LioState::kInitializing);
  EXPECT_EQ(h.fe().epoch(), 2U);
}

// D47: the frontend seeds its own restart with the predictor's newest output (at base_link), its attitude and
// velocity; the answer's ResetDelta is applied.
TEST(LioFrontend, RestartAppliesResetDeltaWhenTheBackendConfirms) {
  Rig h;
  h.ToRestarting();
  const std::optional<StateOutput> old_state = h.LastState();
  ASSERT_TRUE(old_state.has_value());
  ASSERT_EQ(old_state->sample.t, h.now());
  EXPECT_EQ(old_state->sample.reset_counter, 0U);
  EXPECT_EQ(h.fe().state(), LioState::kRestarting);
  EXPECT_EQ(h.fe().epoch(), 1U);

  const RestartCommand cmd = IssuedCommand(h);
  EXPECT_EQ(cmd.new_epoch, 2U);
  EXPECT_EQ(cmd.t, old_state->sample.t);
  const SeedPose expected = BasePose(old_state->sample);
  EXPECT_LT((cmd.seed.p_world_m - expected.p_world_m).norm(), 1e-12);
  EXPECT_LT(cmd.seed.q_world_base.angularDistance(expected.q_world_base), 1e-12);
  EXPECT_LT(cmd.q_world_imu_old.angularDistance(old_state->sample.q_world_imu), 1e-12);
  EXPECT_LT((cmd.v_world_mps_old - old_state->sample.v_world_mps).norm(), 1e-12);

  // Three more IMU samples before the answer: no second command while one is pending, and reset_to matches cmd.t
  // in the buffer (no rewind).
  h.ImuUntil(h.now() + time::milliseconds(15));
  h.backend().Drain();
  EXPECT_EQ(h.backend().restarts().size(), 1U);
  EstimatorSnapshot s = Still(cmd.t);
  s.q_world_imu = old_state->sample.q_world_imu;
  s.v_world_mps = old_state->sample.v_world_mps;
  s.p_world_m = old_state->sample.p_world_m + Eigen::Vector3d(1.0, 0.0, 0.0);
  h.backend().Push(BackendResult{BackendResultKind::kRestartSeeded, 2, 0, cmd.t, s, std::nullopt,
                                 0.03, std::nullopt});
  const StepOutputs out = h.Poll();
  ASSERT_TRUE(out.restart.has_value());
  ASSERT_TRUE(out.restart->has_value());
  EXPECT_LT((out.restart->value().position_m - Eigen::Vector3d(1.0, 0.0, 0.0)).norm(), 1e-9);
  EXPECT_EQ(h.fe().epoch(), 2U);
  EXPECT_EQ(h.fe().state(), LioState::kInitializing);
  ASSERT_TRUE(out.health.has_value());
  EXPECT_EQ(out.health->reason, LioReason::kRestartSeeded);

  const auto next = h.Imu(h.now() + kImuPeriod);
  ASSERT_TRUE(next.has_value());
  ASSERT_EQ(next->states.size(), 1U);
  EXPECT_EQ(next->states[0].sample.reset_counter, 1U);
  EXPECT_EQ(next->states[0].epoch, 2U);

  const auto restarts = h.Events("LioRestart");
  ASSERT_EQ(restarts.size(), 1U);
  EXPECT_EQ(restarts[0].reason, "STABLE_DATA_DWELL");
  EXPECT_EQ(Value(restarts[0], "old_epoch"), 1.0);
  EXPECT_EQ(Value(restarts[0], "new_epoch"), 2.0);
  EXPECT_EQ(restarts[0].identity.lio_epoch, 2U);
  EXPECT_DOUBLE_EQ(Value(restarts[0], "seed_x_m"), expected.p_world_m.x());
  EXPECT_DOUBLE_EQ(Value(restarts[0], "seed_z_m"), expected.p_world_m.z());
  EXPECT_NEAR(Value(restarts[0], "seed_yaw_rad"), 0.0, 1e-12);
  EXPECT_NEAR(Value(restarts[0], "delta_x_m"), 1.0, 1e-9);
  EXPECT_NEAR(Value(restarts[0], "seed_age_s"), 0.015, 1e-9);
}

// Review T3 I2: a seed the backend refuses (kRestartRejected) returns the lifecycle to LOST (RESTART_REJECTED): no
// command until the LOST wait has run a full dwell again.
TEST(LioFrontend, RestartRejectedByTheBackendReturnsToLost) {
  Rig h;
  h.ToRestarting();
  const RestartCommand cmd = IssuedCommand(h);
  h.backend().Push(BackendResult{BackendResultKind::kRestartRejected, cmd.new_epoch, 0, cmd.t,
                                 std::nullopt, std::nullopt, 0.03, std::nullopt});
  const StepOutputs out = h.Poll();
  ASSERT_TRUE(out.restart.has_value());
  ASSERT_FALSE(out.restart->has_value());
  EXPECT_EQ(out.restart->error(), EstimatorReason::kNotFinite);
  EXPECT_EQ(h.fe().state(), LioState::kLost);
  EXPECT_EQ(h.fe().epoch(), 1U);
  ASSERT_TRUE(out.health.has_value());
  EXPECT_EQ(out.health->reason, LioReason::kRestartRejected);
  const auto rejected = h.Events("LioRestartRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "RESTART_REJECTED");
  const auto transitions = h.Events("StateTransition");
  ASSERT_FALSE(transitions.empty());
  EXPECT_EQ(transitions.back().state_before, "RESTARTING");
  EXPECT_EQ(transitions.back().state_after, "LOST");
  EXPECT_EQ(transitions.back().reason, "RESTART_REJECTED");
  EXPECT_TRUE(h.Events("LioRestart").empty());

  // No new command at the next IMU samples: LOST waits for stable data first.
  h.ImuUntil(h.now() + time::milliseconds(50));
  h.backend().Drain();
  EXPECT_EQ(h.backend().restarts().size(), 1U);
}

// Review T3b n1: a kRestartSeeded without a snapshot (a wiring error; asserts in a debug build) is not a refused
// seed: the backend has already switched to the new epoch, so going LOST would leave the frontend one epoch behind
// for a whole dwell. It is refused like a stale answer: RESTARTING, nothing pending, the next IMU sample re-issues.
TEST(LioFrontend, SeededAnswerWithoutSnapshotIsReissuedLikeAStaleOne) {
#ifndef NDEBUG
  GTEST_SKIP() << "a wiring error asserts in a debug build";
#else
  Rig h;
  h.ToRestarting();
  const RestartCommand cmd = IssuedCommand(h);
  h.backend().Push(BackendResult{BackendResultKind::kRestartSeeded, cmd.new_epoch, 0, cmd.t, std::nullopt,
                                 std::nullopt, 0.03, std::nullopt});
  const StepOutputs out = h.Poll();
  ASSERT_TRUE(out.restart.has_value());
  ASSERT_FALSE(out.restart->has_value());
  EXPECT_EQ(h.fe().state(), LioState::kRestarting);
  EXPECT_EQ(h.fe().epoch(), 1U);
  EXPECT_EQ(h.Events("LioRestartRejected").size(), 1U);
  const auto next = h.Imu(h.now() + kImuPeriod);
  ASSERT_TRUE(next.has_value());
  h.backend().Drain();
  ASSERT_EQ(h.backend().restarts().size(), 2U) << "re-issued at the next IMU sample";
  EXPECT_EQ(h.backend().restarts()[1].new_epoch, 2U);
  EXPECT_EQ(h.backend().restarts()[1].t, h.now());
#endif
}

// Review T3 I2: a backend that refuses every seed (e.g. its own ESKF state went non-finite) gets one command per
// restart dwell, not one per IMU sample: 20 s of non-empty scans give 10 commands, each at least restart_dwell after
// the previous one, each with one LioRestartRejected; LIO keeps trying (no give-up).
TEST(LioFrontend, BackendRefusingEverySeedGetsOneCommandPerDwell) {
  Rig h;
  h.ToRestarting();  // the first command at 12.7 s
  std::size_t answered = 0;
  for (std::int64_t end = 12'800; end <= 32'700; end += 100) {
    h.ImuUntil(Ms(end));
    h.backend().Drain();
    while (answered < h.backend().restarts().size()) {
      const RestartCommand c = h.backend().restarts()[answered++];
      h.backend().Push(BackendResult{BackendResultKind::kRestartRejected, c.new_epoch, 0, c.t, std::nullopt,
                                     std::nullopt, 0.03, std::nullopt});
    }
    h.Poll();
    ASSERT_TRUE(h.Scan(Ms(end - 100), Ms(end)).has_value()) << end;
    h.backend().AnswerScan(BackendResultKind::kScanDegenerate);
    h.Poll();
  }
  const std::vector<RestartCommand>& commands = h.backend().restarts();
  ASSERT_EQ(commands.size(), 10U);  // 12.7 s, then 14.8 .. 30.8 s
  for (std::size_t i = 1; i < commands.size(); ++i) {
    EXPECT_GE(commands[i].t - commands[i - 1].t, time::seconds(2)) << i;
    EXPECT_EQ(commands[i].new_epoch, 2U) << i;
  }
  EXPECT_EQ(h.Events("LioRestartRejected").size(), commands.size()) << "one event per refused command";
  EXPECT_TRUE(h.Events("LioInputRejected").empty());
  EXPECT_EQ(h.fe().epoch(), 1U);
}

// Review T2 M1, D41, AR4: in LOST (and RESTARTING) the registration ran against the frozen map with a drifted prior,
// so its result is never applied: no correction of the output, no PredictorCorrectionRejected, no odometry; the
// scan still counts as a non-empty scan for the LOST wait.
TEST(LioFrontend, FrozenMapResultsAreNotAppliedInLostOrRestarting) {
  Rig h;
  h.ToTracking();
  h.ImuUntil(Ms(10'600));
  ASSERT_EQ(h.fe().state(), LioState::kLost);
  ASSERT_TRUE(h.LastState().has_value());
  const Eigen::Vector3d p0 = h.LastState()->sample.p_world_m;
  for (std::int64_t end = 10'700; end <= 11'000; end += 100) {
    const StepOutputs out =
        h.ScanAnswered(Ms(end), BackendResultKind::kScanGood, Still(Ms(end), Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_FALSE(out.odometry.has_value()) << end;
    EXPECT_EQ(h.fe().state(), LioState::kLost) << end;
  }
  h.ImuUntil(Ms(11'100));  // a health grid point
  EXPECT_LT((h.LastState()->sample.p_world_m - p0).norm(), 1e-9) << "a frozen-map result moved the output";
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());
  ASSERT_TRUE(h.LastHealth().has_value());
  EXPECT_NEAR(h.LastHealth()->correction_age_s, 1.1, 1e-9) << "the last applied correction is still the 10.0 s scan";
  // The wait still runs from 10.7 s: RESTARTING at 12.7 s.
  h.DwellUntilRestarting();
  EXPECT_EQ(h.now(), Ms(12'700));

  // RESTARTING with no restart pending (after a stale answer was refused): a frozen-map result is not applied either.
  const RestartCommand cmd = IssuedCommand(h);
  h.ImuUntil(cmd.t + time::milliseconds(400));
  AnswerSeeded(h, cmd);
  ASSERT_TRUE(h.Poll().restart.has_value());
  ASSERT_EQ(h.fe().state(), LioState::kRestarting);
  ASSERT_TRUE(h.Scan(Ms(12'600), h.now()).has_value());  // admitted before the next IMU sample re-issues
  h.backend().AnswerScan(BackendResultKind::kScanGood, Still(h.now(), Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_FALSE(h.Poll().odometry.has_value());
  h.ImuUntil(h.now() + kImuPeriod);
  EXPECT_LT((h.LastState()->sample.p_world_m - p0).norm(), 1e-9);
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());
}

TEST(LioFrontend, OnlyGoodScansInTrackingPublishOdometry) {
  Rig h;
  h.Initialise(Ms(9'000));
  EXPECT_FALSE(h.ScanAnswered(Ms(9'500), BackendResultKind::kMapReady).odometry.has_value());
  for (std::int64_t end = 9'600; end <= 9'900; end += 100) {
    EXPECT_FALSE(h.ScanAnswered(Ms(end), BackendResultKind::kScanGood).odometry.has_value()) << end;
    EXPECT_EQ(h.fe().state(), LioState::kInitializing);
  }
  h.ScanAnswered(Ms(10'000), BackendResultKind::kScanGood);
  ASSERT_EQ(h.fe().state(), LioState::kTracking);
  const StepOutputs good = h.ScanAnswered(Ms(10'100), BackendResultKind::kScanGood);
  ASSERT_TRUE(good.odometry.has_value());
  EXPECT_EQ(good.odometry->t, Ms(10'100));
  EXPECT_EQ(good.odometry->epoch, 1U);
  EXPECT_EQ(good.odometry->reset_counter, 0U);
  EXPECT_EQ(good.odometry->quality, kGoodReport.quality);
  EXPECT_LT((good.odometry->p_world_m - Eigen::Vector3d(-0.10, 0.0, -0.05)).norm(), 1e-12);
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  EXPECT_EQ(good.odometry->pose_cov, Matrix6d(Matrix6d::Identity() * 1e-4));
  const StepOutputs degenerate = h.ScanAnswered(Ms(10'200), BackendResultKind::kScanDegenerate);
  EXPECT_EQ(h.fe().state(), LioState::kTracking);
  EXPECT_FALSE(degenerate.odometry.has_value());
  EXPECT_FALSE(h.ScanAnswered(Ms(10'300), BackendResultKind::kScanEmpty).odometry.has_value());
}

// D31: a kScanGood result the output predictor rejects is not a scan with a correction. It must not
// hold TRACKING, publish odometry or move the LiDAR-gap reference.
TEST(LioFrontend, RejectedCorrectionDoesNotHoldTracking) {
  Rig h(WideGapConfig());
  h.ToTracking();  // last applied scan end 10.0 s
  // A slow backend: each answer arrives 0.5 s after its scan end, older than the 300 ms predictor
  // buffer.
  for (int k = 1; k <= 3; ++k) {
    const time::SensorTime end = Ms(10'000 + 100 * k);
    EXPECT_FALSE(ImuUntilState(h, end + time::milliseconds(500), LioState::kDegraded).has_value());
    ASSERT_TRUE(h.Scan(end - kScanPeriod, end).has_value());
    h.backend().AnswerScan(BackendResultKind::kScanGood);
    const StepOutputs out = h.Poll();
    EXPECT_FALSE(out.odometry.has_value()) << k;
    const auto rejected = h.Events("PredictorCorrectionRejected");
    ASSERT_EQ(rejected.size(), static_cast<std::size_t>(k));
    EXPECT_EQ(rejected.back().reason, "OLDER_THAN_BUFFER");
    if (k < 3) {
      EXPECT_EQ(h.fe().state(), LioState::kTracking) << k;
    } else {
      EXPECT_EQ(h.fe().state(), LioState::kDegraded);
      ASSERT_TRUE(out.health.has_value());
      EXPECT_EQ(out.health->reason, LioReason::kCorrectionRejected);
    }
  }
  // The gap reference is still 10.0 s: LOST at the first IMU t > 10.0 + 1.5 s, and never TRACKING
  // meanwhile.
  const auto lost = ImuUntilState(h, Ms(11'600), LioState::kLost, LioReason::kLidarGapLost);
  EXPECT_EQ(lost, std::optional<time::SensorTime>(Ms(11'505)));
  for (const StepOutputs& o : h.outputs()) EXPECT_FALSE(o.odometry && o.odometry->t > Ms(10'000));
}

// D31 at the beta thresholds: a good result the predictor cannot place, arriving while DEGRADED,
// does not delay LOST. D40: 0.31 s is older than the 300 ms predictor buffer, so the predictor
// refuses it (OLDER_THAN_BUFFER), the one technical refusal left; its age alone refuses nothing.
TEST(LioFrontend, RejectedCorrectionDoesNotMoveTheGapReference) {
  Rig h;
  h.ToTracking();  // last applied scan end 10.0 s
  EXPECT_EQ(ImuUntilState(h, Ms(10'350), LioState::kDegraded, LioReason::kLidarGapDegraded),
            std::optional<time::SensorTime>(Ms(10'255)));
  ASSERT_TRUE(
      h.Scan(Ms(9'940), Ms(10'040)).has_value());  // 0.31 s old: older than the predictor buffer
  h.backend().AnswerScan(BackendResultKind::kScanGood);
  h.Poll();
  const auto rejected = h.Events("PredictorCorrectionRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "OLDER_THAN_BUFFER");
  EXPECT_EQ(
      ImuUntilState(h, Ms(10'600), LioState::kLost, LioReason::kLidarGapLost),
      std::optional<time::SensorTime>(Ms(10'505)));  // from 10.0 s, not from the rejected 10.04 s
}

TEST(LioFrontend, AppliedCorrectionAfterRejectionsRestoresTracking) {
  Rig h(WideGapConfig());
  h.ToTracking();
  for (int k = 1; k <= 3; ++k) {
    const time::SensorTime end = Ms(10'000 + 100 * k);
    h.ImuUntil(end + time::milliseconds(500));
    ASSERT_TRUE(h.Scan(end - kScanPeriod, end).has_value());
    h.backend().AnswerScan(BackendResultKind::kScanGood);
    h.Poll();
  }
  ASSERT_EQ(h.fe().state(), LioState::kDegraded);  // at 10.8 s, by rejected corrections
  // Fresh results the predictor applies: confirm_scans (5) of them -> TRACKING.
  std::optional<OdometryOutput> odometry;
  for (std::int64_t end = 10'900; end <= 11'300; end += 100) {
    const StepOutputs out = h.ScanAnswered(Ms(end), BackendResultKind::kScanGood);
    if (out.odometry) odometry = out.odometry;
  }
  EXPECT_EQ(h.fe().state(), LioState::kTracking);
  ASSERT_TRUE(odometry.has_value());
  EXPECT_EQ(odometry->t, Ms(11'300));
  ASSERT_TRUE(h.LastHealth().has_value());
  EXPECT_EQ(h.LastHealth()->reason, LioReason::kConfirmationReached);
  // The last-scan time advanced to 11.3 s: the gap rule degrades at 11.3 + 1.0 s, not at 10.0 + 1.0
  // s.
  EXPECT_EQ(ImuUntilState(h, Ms(12'400), LioState::kDegraded, LioReason::kLidarGapDegraded),
            std::optional<time::SensorTime>(Ms(12'305)));
}

TEST(LioFrontend, NotProcessedScansEndTheirFlightAndChangeNothing) {
  Rig h;
  h.ToTracking();
  ASSERT_EQ(h.fe().stats().scans_in_flight, 0U);
  h.ImuUntil(Ms(10'100));
  ASSERT_TRUE(h.Scan(Ms(10'000), Ms(10'100)).has_value());
  EXPECT_EQ(h.fe().stats().scans_in_flight, 1U);
  h.backend().AnswerScan(BackendResultKind::kScanNotProcessed);
  const StepOutputs out = h.Poll();
  EXPECT_FALSE(out.odometry.has_value());
  EXPECT_FALSE(out.health.has_value());
  EXPECT_EQ(h.fe().stats().scans_in_flight, 0U);
  EXPECT_EQ(h.fe().state(), LioState::kTracking);
  // Not a scan for the gap rule: DEGRADED at 10.0 + 0.25 s.
  EXPECT_EQ(ImuUntilState(h, Ms(10'300), LioState::kDegraded),
            std::optional<time::SensorTime>(Ms(10'255)));
}

TEST(LioFrontend, MapPolicyFollowsTheLifecycleAtAdmission) {
  Rig h;
  h.ToRestarting();
  h.ImuUntil(Ms(12'800));
  ASSERT_TRUE(h.Scan(Ms(12'700), Ms(12'800)).has_value());
  h.backend().Drain();
  ASSERT_FALSE(h.backend().jobs().empty());
  EXPECT_EQ(h.backend().jobs().back().map_policy, MapPolicy::kFrozen);  // RESTARTING
  for (const FakeBackend::Job& job : h.backend().jobs()) {
    if (job.end >= Ms(10'700)) {
      EXPECT_EQ(job.map_policy, MapPolicy::kFrozen) << job.end.ns;  // LOST, RESTARTING
    }
  }
  EXPECT_EQ(h.backend().jobs().front().map_policy,
            MapPolicy::kInsert);  // INITIALIZING (the map-ready scan)
}

TEST(LioFrontend, UnexpectedSequenceIsDiscarded) {
#ifndef NDEBUG
  GTEST_SKIP() << "a wiring error asserts in a debug build";
#else
  Rig h;
  h.Initialise(Ms(9'000));
  h.ImuUntil(Ms(9'500));
  ASSERT_TRUE(h.Scan(Ms(9'400), Ms(9'500)).has_value());
  h.backend().Push(BackendResult{BackendResultKind::kMapReady, 1, 7, Ms(9'500), Still(Ms(9'500)),
                                 std::nullopt, 0.03, std::nullopt});
  h.Poll();
  const auto discarded = h.Events("LioResultDiscarded");
  ASSERT_EQ(discarded.size(), 1U);
  EXPECT_EQ(discarded[0].reason, "UNEXPECTED_SEQUENCE");
  EXPECT_EQ(h.fe().stats().scans_in_flight, 1U);
  EXPECT_EQ(h.fe().stats().results_discarded, 1U);
#endif
}

TEST(LioFrontend, IdenticalRunsProduceIdenticalOutputs) {
  const auto scenario = [](Rig& h) {
    h.ToTracking();
    h.ScanAnswered(Ms(10'100), BackendResultKind::kScanDegenerate);
    h.ScanAnswered(Ms(10'200), BackendResultKind::kScanGood,
                   Still(Ms(10'200), Eigen::Vector3d(0.02, -0.01, 0.0)));
    h.ImuUntil(Ms(10'900));
  };
  Rig a;
  Rig b;
  scenario(a);
  scenario(b);
  ASSERT_EQ(a.outputs().size(), b.outputs().size());
  for (std::size_t i = 0; i < a.outputs().size(); ++i) {
    const StepOutputs& x = a.outputs()[i];
    const StepOutputs& y = b.outputs()[i];
    ASSERT_EQ(x.states.size(), y.states.size()) << i;
    for (std::size_t k = 0; k < x.states.size(); ++k) {
      EXPECT_EQ(x.states[k].sample.p_world_m, y.states[k].sample.p_world_m) << i;
      EXPECT_EQ(x.states[k].sample.q_world_imu.coeffs(), y.states[k].sample.q_world_imu.coeffs())
          << i;
      EXPECT_EQ(x.states[k].state, y.states[k].state) << i;
    }
    ASSERT_EQ(x.odometry.has_value(), y.odometry.has_value()) << i;
    ASSERT_EQ(x.health.has_value(), y.health.has_value()) << i;
    if (x.health) {
      EXPECT_EQ(x.health->t, y.health->t) << i;
      EXPECT_EQ(x.health->reason, y.health->reason) << i;
      EXPECT_EQ(x.health->output_tracking_error, y.health->output_tracking_error) << i;
    }
  }
  EXPECT_EQ(a.fe().state(), LioState::kLost);
}

// Review fix I1 (D31 consequence): IMU stamps jittered by up to 0.4 ms, scan ends off the IMU grid
// and results arriving 20 ms after their scan end. Every correction is bracketed by buffered
// samples, so none is rejected and TRACKING holds for 3 s.
TEST(LioFrontend, JitteredImuAndOffGridScanEndsKeepTracking) {
  Rig h;
  h.ToTracking();  // newest IMU 10.0 s
  // Sample n after 10.0 s, jittered deterministically by up to 0.4 ms (the IMU and LiDAR share a
  // clock).
  const auto stamp = [](std::int64_t n) {
    return Ms(10'000) + time::Duration{n * kImuPeriod.ns + ((n * 7) % 9 - 4) * 100'000};
  };
  std::int64_t n = 0;
  time::SensorTime last_end = Ms(10'000);
  for (std::int64_t k = 1; k <= 30; ++k) {
    // Scan end inside an IMU interval: at the sample, mid-way, or 1 ns before the next sample (the
    // stamps the S1a rule rejected when the newest dt was shorter than the bracketing interval).
    // Odd k: the first long (5.7 ms) interval after sample 20k, scan end 1 ns before its next
    // sample.
    std::int64_t m = 20 * k;
    while (k % 2 == 1 && (stamp(m + 1) - stamp(m)).ns < 5'500'000) ++m;
    const time::Duration interval = stamp(m + 1) - stamp(m);
    const std::int64_t pick = k % 2 == 1 ? 2 : (k / 2) % 2;
    const time::SensorTime end = stamp(m) + time::Duration{pick == 0   ? 0
                                                           : pick == 1 ? interval.ns / 2
                                                                       : interval.ns - 1};
    // The result arrives about 20 ms later, after a short (4.8 ms) IMU step: the newest dt is then
    // shorter than the bracketing interval.
    while (h.now() < end + time::milliseconds(20) || (stamp(n) - stamp(n - 1)).ns > 5'000'000) {
      ASSERT_TRUE(h.Imu(stamp(++n)).has_value());
    }
    ASSERT_TRUE(h.Scan(last_end, end).has_value()) << k;
    last_end = end;
    h.backend().AnswerScan(BackendResultKind::kScanGood);
    h.Poll();
    ASSERT_EQ(h.fe().state(), LioState::kTracking) << k;
  }
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());
  EXPECT_GE(h.now(), Ms(13'000));
}

// issue_restart fails closed on a non-finite predictor output (review T3 m7): no command, one rate-limited
// LioInputRejected NOT_FINITE on the restart stream; LIO stays RESTARTING. The output is driven non-finite by two
// applied corrections whose difference overflows (+1.7e308 then -1.7e308).
TEST(LioFrontend, NonFinitePredictorOutputRefusesTheRestartWithAnEvent) {
  Rig h;
  h.ToTracking();
  h.ScanAnswered(Ms(10'100), BackendResultKind::kScanGood, Still(Ms(10'100), Eigen::Vector3d(1.7e308, 0.0, 0.0)));
  h.ScanAnswered(Ms(10'200), BackendResultKind::kScanGood, Still(Ms(10'200), Eigen::Vector3d(-1.7e308, 0.0, 0.0)));
  ASSERT_TRUE(h.LastHealth().has_value());
  ASSERT_FALSE(h.LastHealth()->output_tracking_error.allFinite()) << "the overflow did not happen";
  h.ImuUntil(Ms(10'800));  // LiDAR gap: LOST at 10.705 s
  ASSERT_EQ(h.fe().state(), LioState::kLost);
  h.DwellUntilRestarting();  // RESTARTING: the effect finds no finite seed
  h.ImuUntil(h.now() + time::milliseconds(20));
  h.backend().Drain();
  EXPECT_TRUE(h.backend().restarts().empty());
  EXPECT_EQ(h.fe().state(), LioState::kRestarting);
  const auto rejected = h.Events("LioInputRejected");
  ASSERT_EQ(rejected.size(), 1U) << "rate-limited: the first occurrence";
  EXPECT_EQ(rejected[0].reason, "NOT_FINITE");
  EXPECT_EQ(Value(rejected[0], "stream"), 2.0);
}

// A restart refused by a full request queue is one decision and one event (LioInputRejected BACKEND_BUSY, stream
// 2); the next IMU sample issues it.
TEST(LioFrontend, RestartWithAFullRequestQueueIsBackendBusy) {
  Rig h;
  h.ToTracking();
  h.ImuUntil(Ms(10'600));  // LOST
  for (std::int64_t end = 10'700; end <= 12'600; end += 100) h.ScanAnswered(Ms(end), BackendResultKind::kScanGood);
  ASSERT_EQ(h.fe().state(), LioState::kLost);
  h.backend().Drain();
  // Nobody pops any more: fill the request queue with IMU copies, 0.1 ms apart, before the dwell ends (12.7 s).
  time::SensorTime t = h.now();
  while (h.channels().requests.stats().pushed - h.channels().requests.stats().popped < RequestQueue::capacity()) {
    t = t + time::nanoseconds(100'000);
    ASSERT_TRUE(h.Imu(t).has_value());
  }
  ASSERT_LT(t, Ms(12'700));
  ASSERT_EQ(h.fe().state(), LioState::kLost);
  const std::uint64_t rejected_before = h.fe().stats().requests_rejected;
  ASSERT_TRUE(h.Imu(Ms(12'700)).has_value());  // RESTARTING; its IMU copy and the restart command both find it full
  ASSERT_EQ(h.fe().state(), LioState::kRestarting);
  EXPECT_EQ(h.fe().stats().requests_rejected, rejected_before + 2);
  const auto busy = h.Events("LioInputRejected");
  ASSERT_EQ(busy.size(), 1U);
  EXPECT_EQ(busy[0].reason, "BACKEND_BUSY");
  EXPECT_EQ(Value(busy[0], "stream"), 2.0);
  h.backend().Drain();
  EXPECT_TRUE(h.backend().restarts().empty());
  // Nothing is pending: once the backend drains, the next IMU sample issues the command.
  ASSERT_TRUE(h.Imu(Ms(12'705)).has_value());
  h.backend().Drain();
  ASSERT_EQ(h.backend().restarts().size(), 1U);
  EXPECT_EQ(h.backend().restarts()[0].t, Ms(12'705));
  EXPECT_EQ(h.backend().restarts()[0].new_epoch, 2U);
  EXPECT_EQ(h.fe().epoch(), 1U);
}

// --- D33/O16: a restart answer the output predictor can no longer match -------------------------

namespace {

/// Bitwise equal state outputs, call by call.
void ExpectSameStates(const Rig& a, const Rig& b) {
  ASSERT_EQ(a.outputs().size(), b.outputs().size());
  for (std::size_t i = 0; i < a.outputs().size(); ++i) {
    const std::vector<StateOutput>& x = a.outputs()[i].states;
    const std::vector<StateOutput>& y = b.outputs()[i].states;
    ASSERT_EQ(x.size(), y.size()) << i;
    for (std::size_t k = 0; k < x.size(); ++k) {
      EXPECT_EQ(x[k].sample.t, y[k].sample.t) << i;
      EXPECT_EQ(x[k].sample.p_world_m, y[k].sample.p_world_m) << i;
      EXPECT_EQ(x[k].sample.v_world_mps, y[k].sample.v_world_mps) << i;
      EXPECT_EQ(x[k].sample.q_world_imu.coeffs(), y[k].sample.q_world_imu.coeffs()) << i;
      EXPECT_EQ(x[k].sample.reset_counter, y[k].sample.reset_counter) << i;
      EXPECT_EQ(x[k].epoch, y[k].epoch) << i;
      EXPECT_EQ(x[k].state, y[k].state) << i;
    }
  }
}

}  // namespace

// The answer arrives 400 ms (> kPredictorBufferSpan) after its command: refused, never a rewind. A
// twin fed the same inputs but never answered must publish bitwise the same states. The frontend then issues the
// next command itself (D41), at the next IMU sample, seeded with the newest output.
TEST(LioFrontend, LateRestartAnswerIsRefusedAndStaysRestarting) {
  Rig late;
  Rig twin;
  late.ToRestarting();
  twin.ToRestarting();
  const RestartCommand cmd = IssuedCommand(late);
  (void)IssuedCommand(twin);
  EXPECT_EQ(cmd.new_epoch, 2U);
  late.ImuUntil(cmd.t + time::milliseconds(400));
  twin.ImuUntil(cmd.t + time::milliseconds(400));
  const std::optional<StateOutput> previous = late.LastState();
  ASSERT_TRUE(previous.has_value());

  AnswerSeeded(late, cmd);
  const StepOutputs out = late.Poll();
  twin.Poll();
  ASSERT_TRUE(out.restart.has_value());
  ASSERT_FALSE(out.restart->has_value());
  EXPECT_EQ(out.restart->error(), EstimatorReason::kStaleAnswer);
  EXPECT_FALSE(out.health.has_value()) << "no transition";
  EXPECT_EQ(late.fe().state(), LioState::kRestarting);
  EXPECT_EQ(late.fe().epoch(), 1U);
  const auto rejected = late.Events("LioRestartRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "RESTART_ANSWER_STALE");
  EXPECT_EQ(rejected[0].identity.lio_epoch, 1U);
  EXPECT_EQ(Value(rejected[0], "new_epoch"), 2.0);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), static_cast<double>(cmd.t.ns) / 1e9);
  EXPECT_NEAR(Value(rejected[0], "seed_age_s"), 0.4, 1e-9);
  EXPECT_TRUE(late.Events("LioRestart").empty());

  // No rewind: the next state is newer than the previous one, and every state equals the twin's (whose command is
  // still pending, so it issues none).
  late.ImuUntil(late.now() + time::milliseconds(100));
  twin.ImuUntil(twin.now() + time::milliseconds(100));
  const auto& outs = late.outputs();
  const auto first_after = std::find_if(outs.end() - 21, outs.end(),
                                        [](const StepOutputs& o) { return !o.states.empty(); });
  ASSERT_NE(first_after, outs.end());
  EXPECT_GT(first_after->states[0].sample.t, previous->sample.t);
  EXPECT_EQ(first_after->states[0].sample.reset_counter, 0U);
  ExpectSameStates(late, twin);

  // No restart was pending after the refusal: the first IMU sample after it issued a new command for epoch 2 again
  // (the backend has already switched to 2 and seeds again at the newer time), seeded with the output of that
  // sample; its timely answer is applied.
  late.backend().Drain();
  ASSERT_EQ(late.backend().restarts().size(), 2U);
  const RestartCommand retry = late.backend().restarts()[1];
  EXPECT_EQ(retry.new_epoch, 2U);
  EXPECT_EQ(retry.t, cmd.t + time::milliseconds(405));
  EXPECT_LT((retry.seed.p_world_m - BasePose(first_after->states[0].sample).p_world_m).norm(), 1e-12);
  twin.backend().Drain();
  EXPECT_EQ(twin.backend().restarts().size(), 1U);
  AnswerSeeded(late, retry);
  const StepOutputs seeded = late.Poll();
  ASSERT_TRUE(seeded.restart.has_value());
  EXPECT_TRUE(seeded.restart->has_value());
  EXPECT_EQ(late.fe().state(), LioState::kInitializing);
  EXPECT_EQ(late.fe().epoch(), 2U);
  EXPECT_EQ(late.Events("LioRestartRejected").size(), 1U);
}

// The boundary of the match rule: an answer exactly kPredictorBufferSpan old is applied without a
// rewind; one IMU sample later it is refused.
TEST(LioFrontend, TimelyRestartAnswerAtTheBufferBoundaryIsApplied) {
  Rig h;
  h.ToRestarting();
  const RestartCommand cmd = IssuedCommand(h);
  h.ImuUntil(cmd.t + lio::limits::kPredictorBufferSpan);
  const time::SensorTime newest = h.now();
  AnswerSeeded(h, cmd);
  const StepOutputs out = h.Poll();
  ASSERT_TRUE(out.restart.has_value());
  ASSERT_TRUE(out.restart->has_value());
  EXPECT_LT(out.restart->value().position_m.norm(), 1e-6) << "the seed is the output itself";
  EXPECT_EQ(h.fe().state(), LioState::kInitializing);
  EXPECT_EQ(h.fe().epoch(), 2U);
  EXPECT_TRUE(h.Events("LioRestartRejected").empty());
  const auto restarts = h.Events("LioRestart");
  ASSERT_EQ(restarts.size(), 1U);
  EXPECT_NEAR(Value(restarts[0], "seed_age_s"), 0.3, 1e-9);
  const auto next = h.Imu(newest + kImuPeriod);  // the time axis is kept
  ASSERT_TRUE(next.has_value());
  ASSERT_EQ(next->states.size(), 1U);
  EXPECT_EQ(next->states[0].sample.t, newest + kImuPeriod);
  EXPECT_EQ(next->states[0].epoch, 2U);

  Rig g;
  g.ToRestarting();
  const RestartCommand late = IssuedCommand(g);
  g.ImuUntil(late.t + lio::limits::kPredictorBufferSpan + kImuPeriod);
  AnswerSeeded(g, late);
  const StepOutputs refused = g.Poll();
  ASSERT_TRUE(refused.restart.has_value());
  ASSERT_FALSE(refused.restart->has_value());
  EXPECT_EQ(refused.restart->error(), EstimatorReason::kStaleAnswer);
  EXPECT_EQ(g.fe().epoch(), 1U);
}

// The retry reuses the refused answer's epoch number, so a scan admitted while the refused restart
// was pending carries that number too: its result (computed in the refused seed's world, answered
// before the retry's own answer) must not be taken for a result of the retry.
TEST(LioFrontend, ScanResultsOfARefusedAnswerAreDiscardedAfterTheRetry) {
  Rig h;
  h.ToRestarting();
  const RestartCommand cmd = IssuedCommand(h);
  h.ImuUntil(cmd.t + time::milliseconds(350));
  ASSERT_TRUE(h.Scan(h.now() - kScanPeriod, h.now()).has_value());
  const time::SensorTime scan_end = h.now();
  h.ImuUntil(cmd.t + time::milliseconds(400));
  AnswerSeeded(h, cmd);
  const StepOutputs refused = h.Poll();
  ASSERT_TRUE(refused.restart.has_value());
  ASSERT_FALSE(refused.restart->has_value());
  ASSERT_TRUE(h.Imu(h.now() + kImuPeriod).has_value());  // the frontend's own retry

  const auto job = h.backend().AnswerScan(BackendResultKind::kScanGood,
                                          Still(scan_end, Eigen::Vector3d(1.0, 0.0, 0.0)));
  EXPECT_EQ(job.epoch, 2U);
  const std::uint64_t discarded = h.fe().stats().results_discarded;
  const StepOutputs out = h.Poll();
  EXPECT_FALSE(out.odometry.has_value());
  EXPECT_EQ(h.fe().stats().results_discarded, discarded + 1);
  EXPECT_EQ(h.fe().stats().scans_in_flight, 0U);
  const auto events = h.Events("LioResultDiscarded");
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.back().reason, "STALE_EPOCH");
  EXPECT_EQ(Value(events.back(), "result_epoch"), 2.0);
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());
  h.ImuUntil(h.now() + kImuPeriod);
  EXPECT_LT(h.LastState()->sample.p_world_m.norm(), 1e-6) << "the predictor was not corrected";

  h.backend().Drain();
  ASSERT_EQ(h.backend().restarts().size(), 2U);
  AnswerSeeded(h, h.backend().restarts()[1]);
  const StepOutputs seeded = h.Poll();
  ASSERT_TRUE(seeded.restart.has_value());
  EXPECT_TRUE(seeded.restart->has_value());
  EXPECT_EQ(h.fe().state(), LioState::kInitializing);
  EXPECT_EQ(h.fe().epoch(), 2U);
}

// outputs.hpp: every output carries reset_counter == epoch - 1 (an applied restart switches the
// epoch and resets the predictor once; a refused answer does neither).
TEST(LioFrontend, ResetCounterIsEpochMinusOne) {
  Rig h;
  h.ToRestarting();
  const auto to_tracking = [&h] {
    const time::SensorTime start = h.now();
    h.ScanAnswered(start + kScanPeriod, BackendResultKind::kMapReady);
    for (int k = 2; k <= 6; ++k) {
      const time::SensorTime end = start + time::Duration{k * kScanPeriod.ns};
      h.ScanAnswered(end, BackendResultKind::kScanGood);
    }
    ASSERT_EQ(h.fe().state(), LioState::kTracking);
  };
  AnswerSeeded(h, IssuedCommand(h));  // applied: epoch 2
  h.Poll();
  ASSERT_EQ(h.fe().epoch(), 2U);
  to_tracking();
  h.ImuUntil(h.now() + time::milliseconds(600));  // LiDAR gap: LOST
  ASSERT_EQ(h.fe().state(), LioState::kLost);
  h.DwellUntilRestarting();
  const RestartCommand refused = IssuedCommand(h);
  h.ImuUntil(refused.t + time::milliseconds(400));
  AnswerSeeded(h, refused);  // refused: still epoch 2
  h.Poll();
  ASSERT_EQ(h.fe().epoch(), 2U);
  ASSERT_TRUE(h.Imu(h.now() + kImuPeriod).has_value());  // the frontend's own retry
  const RestartCommand retry = IssuedCommand(h);
  EXPECT_EQ(retry.new_epoch, 3U);
  AnswerSeeded(h, retry);  // applied: epoch 3
  h.Poll();
  ASSERT_EQ(h.fe().epoch(), 3U);
  to_tracking();

  std::array<int, 4> odometry{};
  std::size_t states = 0;
  for (const StepOutputs& o : h.outputs()) {
    for (const StateOutput& s : o.states) {
      EXPECT_EQ(s.sample.reset_counter, s.epoch - 1) << s.sample.t.ns;
      ++states;
    }
    if (!o.odometry) continue;
    EXPECT_EQ(o.odometry->reset_counter, o.odometry->epoch - 1) << o.odometry->t.ns;
    if (o.odometry->epoch < odometry.size()) ++odometry[o.odometry->epoch];
  }
  EXPECT_GT(states, 400U);
  EXPECT_GE(odometry[1], 1);
  EXPECT_GE(odometry[2], 1);
  EXPECT_GE(odometry[3], 1);
}

/// Answers the oldest scan with kScanGood carrying `snapshot` (nullopt: none), a good report and
/// odometry.
void AnswerGood(Rig& h, std::optional<EstimatorSnapshot> snapshot) {
  const FakeBackend::Job job = h.backend().NextJob();
  h.backend().Push(BackendResult{
      BackendResultKind::kScanGood, job.epoch, job.seq, job.end, snapshot, kGoodReport, 0.03,
      BaseOdometry{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
                   Eigen::Matrix<double, 6, 6>::Identity() * 1e-4,
                   Eigen::Matrix3d::Identity() * 1e-4}});
}

// Review fix (e): a kScanGood without a snapshot (a wiring error) and a snapshot newer than the
// newest output (kNewerThanOutput) are rejected corrections: event, kScanCorrectionRejected, no
// odometry.
TEST(LioFrontend, GoodResultsThePredictorCannotUseAreRejectedCorrections) {
  for (const bool without_snapshot : {true, false}) {
    Rig h(WideGapConfig());  // three rejections inside TRACKING need a gap threshold above 0.3 s
    h.ToTracking();
    for (int k = 1; k <= 3; ++k) {
      const time::SensorTime end = Ms(10'000 + 100 * k);
      h.ImuUntil(end);
      ASSERT_TRUE(h.Scan(end - kScanPeriod, end).has_value());
      AnswerGood(h, without_snapshot
                        ? std::nullopt
                        : std::optional<EstimatorSnapshot>(Still(end + time::milliseconds(1))));
      const StepOutputs out = h.Poll();
      EXPECT_FALSE(out.odometry.has_value()) << k;
      const auto rejected = h.Events("PredictorCorrectionRejected");
      ASSERT_EQ(rejected.size(), static_cast<std::size_t>(k));
      EXPECT_EQ(rejected.back().reason, without_snapshot ? "NON_FINITE" : "NEWER_THAN_OUTPUT");
    }
    EXPECT_EQ(h.fe().state(), LioState::kDegraded);
    ASSERT_TRUE(h.LastHealth().has_value());
    EXPECT_EQ(h.LastHealth()->reason, LioReason::kCorrectionRejected);
    // The gap reference is still 10.0 s: LOST at the first IMU t > 10.0 + 1.5 s.
    EXPECT_EQ(ImuUntilState(h, Ms(11'600), LioState::kLost, LioReason::kLidarGapLost),
              std::optional<time::SensorTime>(Ms(11'505)));
  }
}

// --- D40, AR4: no age rule; the freshness guard on confirmation ---------------------------------
//
// A kScanGood result is refused only when the output predictor cannot place it (on_correction's
// Result<void, PredictorReason>: older than the 300 ms buffer, newer than the output, not finite).
// Its age is never a reason to refuse it: an applied correction moves the output and the gap
// reference. It counts toward confirm_scans only while (newest ticked IMU time - scan end) <=
// gap_degraded (LioLifecycle, the guard of D40/AR4); a stale one also breaks the confirmation run.
// Results come back in admission order, so a result older than gap_degraded can only arrive after
// the gap rule already left TRACKING: most tests run in DEGRADED.

/// TRACKING (last applied scan end 10.0 s), DEGRADED by the LiDAR gap at 10.255 s, then four fresh
/// good scans ending 10.3 .. 10.6 s (applied at age 0): one more fresh good scan confirms TRACKING.
void DegradedOneScanFromConfirmation(Rig& h) {
  h.ToTracking();
  ASSERT_EQ(ImuUntilState(h, Ms(10'300), LioState::kDegraded, LioReason::kLidarGapDegraded),
            std::optional<time::SensorTime>(Ms(10'255)));
  for (std::int64_t end = 10'300; end <= 10'600; end += 100) {
    h.ScanAnswered(Ms(end), BackendResultKind::kScanGood);
    ASSERT_EQ(h.fe().state(), LioState::kDegraded) << end;
  }
}

TEST(LioFrontend, ResultOlderThanGapDegradedIsAppliedToTheOutput) {
  Rig h;
  DegradedOneScanFromConfirmation(h);  // last applied scan end 10.6 s
  h.ImuUntil(Ms(10'900));              // a health grid point
  const std::optional<HealthOutput> before = h.LastHealth();
  ASSERT_TRUE(before.has_value());
  ASSERT_EQ(before->t, Ms(10'900));
  EXPECT_NEAR(before->correction_age_s, 0.3, 1e-9);

  // Scan (10.6, 10.7] answered when the newest IMU is 10.97 s: 0.27 s old, inside the 300 ms
  // predictor buffer. The registration puts the vehicle 1 m from the output (the synthetic truth).
  h.ImuUntil(Ms(10'970));
  const Eigen::Vector3d truth(1.0, 0.0, 0.0);
  ASSERT_TRUE(h.LastState().has_value());
  const double error_before = (h.LastState()->sample.p_world_m - truth).norm();
  EXPECT_NEAR(error_before, 1.0, 1e-6);
  ASSERT_TRUE(h.Scan(Ms(10'600), Ms(10'700)).has_value());
  h.backend().AnswerScan(BackendResultKind::kScanGood, Still(Ms(10'700), truth));
  const StepOutputs out = h.Poll();
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());  // applied
  // Stale for confirmation: it would have been the fifth good scan, but DEGRADED stays, no odometry.
  EXPECT_EQ(h.fe().state(), LioState::kDegraded);
  EXPECT_FALSE(out.odometry.has_value());

  // The correction reached the output: the error shrinks by one PI step (no jump), and the
  // correction age now runs from 10.7 s.
  h.ImuUntil(Ms(11'000));
  const std::optional<HealthOutput> after = h.LastHealth();
  ASSERT_TRUE(after.has_value());
  ASSERT_EQ(after->t, Ms(11'000));
  EXPECT_NEAR(after->correction_age_s, 0.3, 1e-9);
  EXPECT_NEAR(after->output_tracking_error.z(), 1.0, 1e-6);  // position error seen at 10.7 s
  ASSERT_TRUE(h.LastState().has_value());
  const double error_after = (h.LastState()->sample.p_world_m - truth).norm();
  EXPECT_LT(error_after, error_before - 0.01);
  EXPECT_GT(error_after, 0.5);  // a small step towards the snapshot, never to it

  // It broke the confirmation run: five more fresh good scans are needed.
  for (std::int64_t end = 11'000; end <= 11'300; end += 100) {
    EXPECT_FALSE(h.ScanAnswered(Ms(end), BackendResultKind::kScanGood, Still(Ms(end), truth))
                     .odometry.has_value())
        << end;
    EXPECT_EQ(h.fe().state(), LioState::kDegraded) << end;
  }
  const StepOutputs confirmed =
      h.ScanAnswered(Ms(11'400), BackendResultKind::kScanGood, Still(Ms(11'400), truth));
  EXPECT_EQ(h.fe().state(), LioState::kTracking);
  ASSERT_TRUE(confirmed.odometry.has_value());
  EXPECT_EQ(confirmed.odometry->t, Ms(11'400));
}

// Review Focus 1 (O15, C4), results older than gap_degraded: every scan answered 0.27 s after its
// end. S1a/D30 applied and counted them: TRACKING with 0.27 s old odometry, then DEGRADED on the
// next tick (flapping). S1b refused them by age: LOST, a new epoch, a CPU load turned into a fault of
// four layers. Now they are applied but never confirm: DEGRADED for the whole saturation, the output
// corrected, no odometry (no EV), no LOST. The DV1 band (younger than gap_degraded, but too old for
// the next scan period) is SaturatedBackendWithTwoScansInFlightNeverFlaps.
TEST(LioFrontend, SaturatedBackendStaysDegradedWithACorrectedOutput) {
  Rig h;
  h.ToTracking();  // last applied scan end 10.0 s, output at the origin
  ASSERT_EQ(ImuUntilState(h, Ms(10'300), LioState::kDegraded, LioReason::kLidarGapDegraded),
            std::optional<time::SensorTime>(Ms(10'255)));
  // The synthetic truth: the registration puts the (still) vehicle 0.3 m from the output.
  const Eigen::Vector3d truth(0.3, 0.0, 0.0);
  bool tracking = false;
  bool lost = false;
  int results = 0;
  double max_error_last_second = 0.0;
  for (time::SensorTime t = Ms(10'305); t <= Ms(13'300); t = t + kImuPeriod) {
    const auto r = h.Imu(t);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r->states.size(), 1U) << t.ns;
    if ((t.ns - Ms(10'000).ns) % kScanPeriod.ns == 0) {  // every 100 ms, a result 0.27 s old
      const time::SensorTime end = t - time::milliseconds(270);
      ASSERT_TRUE(h.Scan(end - kScanPeriod, end).has_value()) << t.ns;
      h.backend().AnswerScan(BackendResultKind::kScanGood, Still(end, truth));
      EXPECT_FALSE(h.Poll().odometry.has_value()) << t.ns;
      ++results;
    }
    tracking = tracking || h.fe().state() == LioState::kTracking;
    lost = lost || h.fe().state() == LioState::kLost;
    if (t >= Ms(12'300) && !r->states.empty())
      max_error_last_second =
          std::max(max_error_last_second, (r->states.back().sample.p_world_m - truth).norm());
  }
  EXPECT_FALSE(tracking);
  EXPECT_FALSE(lost);
  EXPECT_EQ(h.fe().state(), LioState::kDegraded);
  EXPECT_EQ(results, 30);  // 10.4 .. 13.3 s
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());
  const auto transitions = h.Events("StateTransition");
  ASSERT_FALSE(transitions.empty());
  EXPECT_EQ(transitions.back().reason, "LIDAR_GAP_DEGRADED");  // the only change since TRACKING
  for (const StepOutputs& o : h.outputs()) EXPECT_FALSE(o.odometry && o.odometry->t > Ms(10'000));
  // The output follows the registration: < 0.05 m from the truth over the last second.
  EXPECT_LT(max_error_last_second, 0.05);
  ASSERT_TRUE(h.LastHealth().has_value());
  EXPECT_LE(h.LastHealth()->correction_age_s, 0.37 + 1e-9);  // the newest applied scan end
}

// The guard uses the newest IMU sample already ticked: push_imu applies the queued results before it
// ticks its own sample. A result 0.15 s old at 10.85 s (gap_degraded - kScanPeriod: the boundary,
// counted) is counted by push_imu(10.855 s) although it is 0.155 s old at that new sample. Because
// the guard leaves one scan period of margin, the tick of 10.855 s (gap 0.155 s) keeps TRACKING.
TEST(LioFrontend, ResultAgeUsesTheNewestTickedImuSample) {
  Rig h;
  DegradedOneScanFromConfirmation(h);
  h.ImuUntil(Ms(10'850));
  ASSERT_TRUE(h.Scan(Ms(10'600), Ms(10'700)).has_value());
  h.backend().AnswerScan(BackendResultKind::kScanGood);  // queued: the next push_imu applies it
  const auto r = h.Imu(Ms(10'855));
  ASSERT_TRUE(r.has_value());
  ASSERT_TRUE(r->odometry.has_value());  // counted: the fifth good scan confirmed TRACKING
  EXPECT_EQ(r->odometry->t, Ms(10'700));
  const auto transitions = h.Events("StateTransition");
  ASSERT_FALSE(transitions.empty());
  EXPECT_EQ(transitions.back().reason, "CONFIRMATION_REACHED");
  EXPECT_EQ(h.fe().state(), LioState::kTracking);
  ASSERT_TRUE(r->health.has_value());
  // The confirmation's health is stamped at the newest ticked sample (10.85 s), as is its age.
  EXPECT_EQ(r->health->t, Ms(10'850));
  EXPECT_NEAR(r->health->correction_age_s, 0.15, 1e-9);
}

// Review Focus 1, the realistic saturation (review T2 I1, the DV1 band): ICP 135 ms per scan, one
// at a time, at most limits::kMaxScansInFlight (2) admitted scans without a result. Every fourth scan
// is refused BACKEND_BUSY and the results come back 0.17-0.26 s after their scan end: most are younger
// than gap_degraded, but every one is too old for the gap rule to hold one scan period later. The
// S1c Task 2 guard (age <= gap_degraded) confirmed TRACKING on five of them and degraded on the next
// tick, again and again. The guard with the scan-period margin never confirms: one TRACKING ->
// DEGRADED by the gap rule, then DEGRADED for the whole saturation, no LOST, no odometry.
TEST(LioFrontend, SaturatedBackendWithTwoScansInFlightNeverFlaps) {
  Rig h;
  h.ToTracking();  // last applied scan end 10.0 s; one StateTransition (CONFIRMATION_REACHED)
  constexpr time::Duration kIcp = time::milliseconds(135);
  std::deque<time::SensorTime> finish;  // when the backend finishes each admitted scan, in order
  std::vector<double> ages;             // age of each result when applied (newest ticked IMU - end)
  int refused = 0;
  std::optional<time::SensorTime> degraded_at;
  for (time::SensorTime t = Ms(10'005); t <= Ms(13'000); t = t + kImuPeriod) {
    ASSERT_TRUE(h.Imu(t).has_value());
    while (!finish.empty() && finish.front() <= t) {  // the answer is ready: this poll applies it
      const FakeBackend::Job job = h.backend().AnswerScan(BackendResultKind::kScanGood);
      finish.pop_front();
      h.Poll();
      ages.push_back(time::to_seconds(t - job.end));
    }
    if ((t.ns - Ms(10'000).ns) % kScanPeriod.ns == 0) {
      const auto admitted = h.Scan(t - kScanPeriod, t);
      if (admitted) {
        finish.push_back((finish.empty() ? t : std::max(t, finish.back())) + kIcp);
      } else {
        EXPECT_EQ(admitted.error(), EstimatorReason::kBackendBusy) << t.ns;
        ++refused;
      }
    }
    if (!degraded_at && h.fe().state() == LioState::kDegraded) degraded_at = t;
    EXPECT_NE(h.fe().state(), LioState::kLost) << t.ns;
  }
  ASSERT_EQ(degraded_at, std::optional<time::SensorTime>(Ms(10'355)));  // gap 0.255 s from 10.1 s
  EXPECT_EQ(h.fe().state(), LioState::kDegraded);
  const auto transitions = h.Events("StateTransition");
  ASSERT_EQ(transitions.size(), 2U) << "no TRACKING <-> DEGRADED flapping";
  EXPECT_EQ(transitions[1].reason, "LIDAR_GAP_DEGRADED");
  EXPECT_GE(refused, 5);  // the two-in-flight limit binds
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());  // every result was placeable
  // The pattern is the DV1 band: after the first result, every age is above gap_degraded - kScanPeriod,
  // and there are runs of >= 5 consecutive results at or below gap_degraded.
  ASSERT_GE(ages.size(), 20U);
  int run = 0, longest = 0;
  for (std::size_t i = 1; i < ages.size(); ++i) {
    EXPECT_GT(ages[i], 0.15 + 1e-9) << i;
    EXPECT_LT(ages[i], 0.3) << i;
    run = ages[i] <= 0.25 + 1e-9 ? run + 1 : 0;
    longest = std::max(longest, run);
  }
  EXPECT_GE(longest, 5);
  for (const StepOutputs& o : h.outputs()) EXPECT_FALSE(o.odometry && o.odometry->t > Ms(10'100));
}

// INITIALIZING (no gap rule): a stale good result after the map is ready is applied but breaks the
// confirmation run.
TEST(LioFrontend, ResultOlderThanGapDegradedBreaksTheInitialConfirmation) {
  Rig h;
  h.Initialise(Ms(9'000));
  h.ScanAnswered(Ms(9'500), BackendResultKind::kMapReady);
  for (std::int64_t end = 9'600; end <= 9'900; end += 100) {
    h.ScanAnswered(Ms(end), BackendResultKind::kScanGood);
    ASSERT_EQ(h.fe().state(), LioState::kInitializing) << end;
  }
  h.ImuUntil(Ms(10'200));
  ASSERT_TRUE(h.Scan(Ms(9'900), Ms(9'930)).has_value());  // 0.27 s old at 10.2 s
  h.backend().AnswerScan(BackendResultKind::kScanGood);
  const StepOutputs out = h.Poll();
  EXPECT_EQ(h.fe().state(), LioState::kInitializing);  // not the fifth confirming scan
  EXPECT_FALSE(out.odometry.has_value());
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());  // applied to the output
  h.ImuUntil(Ms(10'300));
  ASSERT_TRUE(h.LastHealth().has_value());
  EXPECT_NEAR(h.LastHealth()->correction_age_s, 0.37, 1e-9);  // from 9.93 s, the stale scan end
  for (std::int64_t end = 10'300; end <= 10'600; end += 100) {
    h.ScanAnswered(Ms(end), BackendResultKind::kScanGood);
    EXPECT_EQ(h.fe().state(), LioState::kInitializing) << end;
  }
  h.ScanAnswered(Ms(10'700), BackendResultKind::kScanGood);
  EXPECT_EQ(h.fe().state(), LioState::kTracking);
}

// The ImuGap event threshold comes from the one key imu_max_gap_s (frontend_config copies it, D32).
TEST(LioFrontend, ImuGapIsReportedAboveTheConfiguredStep) {
  const LioConfig full = DefaultConfig();
  EXPECT_EQ(frontend_config(full).imu_max_gap, full.math.imu_max_gap);
  Rig h;
  h.ImuUntil(Ms(9'000));
  ASSERT_TRUE(h.Imu(Ms(9'000) + full.math.imu_max_gap).has_value());  // exactly the limit: no event
  EXPECT_TRUE(h.Events("ImuGap").empty());
  ASSERT_TRUE(h.Imu(h.now() + full.math.imu_max_gap + time::nanoseconds(1)).has_value());
  const auto gaps = h.Events("ImuGap");
  ASSERT_EQ(gaps.size(), 1U);
  EXPECT_EQ(gaps[0].reason, "IMU_GAP");
}

TEST(LioSplit, FrontendSourcesNeverReferenceTheMathLibrary) {
  const std::string dir = UAVNAV_LIO_CORE_SOURCE_DIR;
  for (const std::string file : {"/include/uavnav/lio/frontend.hpp", "/src/frontend.cpp"}) {
    const std::string text = ReadFile(dir + file);
    ASSERT_FALSE(text.empty()) << file;
    for (const std::string_view token :
         {"fast_lio_core/", "IkfomEstimator", "IkdTree", "ScanDeskewer", "evaluate_degeneracy",
          "uavnav/lio/config.hpp", "backend_config.hpp"}) {
      EXPECT_EQ(text.find(token), std::string::npos) << file << " names " << token;
    }
  }
}
