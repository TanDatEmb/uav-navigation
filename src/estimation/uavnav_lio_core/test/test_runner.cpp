// LioRunner: the PRODUCT two-thread assembly (SYSTEM_DESIGN §3.5, D30/O14, D31). These tests run the real
// backend jthread; `make sanitize` runs them under ThreadSanitizer (D28).
//
// Conventions:
// - ONE producer at a time calls the ingest-thread API. A phase that runs on a std::async thread hands the
//   runner back through future::get (happens-before), so the calls stay serialised.
// - The test's LioClock (called by LioBackend::run before every request) blocks on a Gate while it is closed:
//   this is how a test stalls the backend thread.
// - No wall-clock bound below 1 s. Every step that could hang runs under Bounded() (std::async +
//   wait_for(10 s)); on timeout it opens the gate and sets the abort flag before the future is destroyed, so a
//   failing test fails instead of hanging the suite. Polling loops (Eventually) give up after 10 s.
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "synthetic_scene.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/limits.hpp"
#include "uavnav/lio/pipeline.hpp"
#include "uavnav/lio/runner.hpp"

using namespace uavnav;
using namespace uavnav::lio;

namespace {

constexpr auto kGuard = std::chrono::seconds(10);

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

constexpr time::Duration kImuPeriod = time::milliseconds(5);     // 200 Hz
constexpr time::Duration kScanPeriod = time::milliseconds(100);  // 10 Hz
constexpr time::SensorTime kT0{1'000'000'000};                   // sensor time must be > 0

double Value(const events::EventRecord& r, std::string_view key) {
  for (std::uint8_t i = 0; i < r.value_count; ++i) {
    if (r.values[i].key == key) return r.values[i].value;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

bool Finite(const StepOutputs& out) {
  for (const StateOutput& s : out.states) {
    if (!s.sample.q_world_imu.coeffs().allFinite() || !s.sample.v_world_mps.allFinite() ||
        !s.sample.p_world_m.allFinite()) {
      return false;
    }
  }
  if (out.odometry && (!out.odometry->p_world_m.allFinite() || !out.odometry->pose_cov.allFinite())) return false;
  if (out.health && (!std::isfinite(out.health->correction_age_s) || !out.health->output_tracking_error.allFinite()))
    return false;
  return true;
}

/// Polls `pred` until it holds; false after kGuard (never hangs).
template <class Pred>
bool Eventually(Pred pred, const std::atomic<bool>* abort = nullptr) {
  const auto deadline = std::chrono::steady_clock::now() + kGuard;
  while (!pred()) {
    if (std::chrono::steady_clock::now() > deadline || (abort && abort->load())) return pred();
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  return true;
}

/// Number of threads of this process (Linux).
int ThreadCount() {
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (line.rfind("Threads:", 0) == 0) return std::stoi(line.substr(8));
  }
  return -1;
}

// ---------------------------------------------------------------------------------------------------
// The backend clock of the tests.

/// Closed: the backend thread blocks inside its clock call, i.e. before popping its next request.
class Gate {
 public:
  void close() noexcept { closed_.store(true, std::memory_order_release); }
  void open() noexcept {
    closed_.store(false, std::memory_order_release);
    closed_.notify_all();
  }
  /// Backend thread (the clock).
  void pass() noexcept {
    if (!closed_.load(std::memory_order_acquire)) return;
    blocked_.fetch_add(1, std::memory_order_acq_rel);
    while (closed_.load(std::memory_order_acquire)) closed_.wait(true, std::memory_order_acquire);
    blocked_.fetch_sub(1, std::memory_order_acq_rel);
  }
  int blocked() const noexcept { return blocked_.load(std::memory_order_acquire); }

 private:
  std::atomic<bool> closed_{false};
  std::atomic<int> blocked_{0};
};

/// Shared between the test and the backend thread (atomics only).
struct ClockControl {
  Gate gate;
  std::atomic<std::int64_t> now_ns{0};  // newest IMU stamp pushed (stamps the backend's events)
  std::atomic<int> throws_left{0};      // the clock throws while > 0
  std::atomic<std::uint64_t> calls{0};
  std::atomic<std::thread::id> thread{};  // the first caller
  std::atomic<bool> other_thread{false};  // a call from a second thread
};

time::TimeSnapshot Snap(time::SensorTime t) {
  return {time::SteadyTime{3 * t.ns}, time::RosTime{t.ns + 7'000'000'000}};
}

LioClock MakeClock(std::shared_ptr<ClockControl> c) {
  return [c]() -> time::TimeSnapshot {
    c->calls.fetch_add(1);
    std::thread::id expected{};
    if (!c->thread.compare_exchange_strong(expected, std::this_thread::get_id()) &&
        expected != std::this_thread::get_id()) {
      c->other_thread.store(true);
    }
    c->gate.pass();
    if (c->throws_left.load() > 0) {
      c->throws_left.fetch_sub(1);
      throw std::runtime_error("test clock failure");
    }
    return Snap(time::SensorTime{c->now_ns.load()});
  };
}

enum class ScanKind { kRoom, kPlane, kEmpty };
enum class Mode { kRunner, kPipeline };

// ---------------------------------------------------------------------------------------------------
// Drives a LioRunner (or, for the equivalence test, a LioPipeline) with the synthetic scene.

class Rig {
 public:
  explicit Rig(Mode mode = Mode::kRunner, const LioConfig& cfg = DefaultConfig())
      : store_(std::make_shared<Store>()), clock_(std::make_shared<ClockControl>()) {
    auto recorder = events::EventRecorder::create(std::make_unique<FakeSink>(store_));
    EXPECT_TRUE(recorder.has_value());
    recorder_ = std::move(*recorder);
    if (mode == Mode::kRunner) {
      auto r = LioRunner::create(cfg, scene::base_T_imu(), scene::imu_T_lidar(), *recorder_, MakeClock(clock_));
      EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().detail);
      if (r) runner_ = std::move(*r);
    } else {
      auto p = LioPipeline::create(cfg, scene::base_T_imu(), scene::imu_T_lidar(), *recorder_);
      EXPECT_TRUE(p.has_value());
      if (p) pipeline_ = std::move(*p);
    }
    traj_.t0 = kT0;
  }

  // The backend thread may be blocked in the gate: open it before the runner joins it.
  ~Rig() {
    Abort();
    runner_.reset();
  }

  Rig(const Rig&) = delete;
  Rig& operator=(const Rig&) = delete;

  /// Unblocks everything a failing test could wait on (the gate, the polling loops).
  void Abort() noexcept {
    abort_.store(true);
    clock_->gate.open();
  }
  const std::atomic<bool>& aborted() const { return abort_; }

  LioRunner& runner() { return *runner_; }
  bool has_runner() const { return runner_ != nullptr; }
  ClockControl& clock() { return *clock_; }
  events::EventRecorder& recorder() { return *recorder_; }
  time::SensorTime now() const { return next_imu_ - kImuPeriod; }
  /// The next Run() scan starts at `t` (the LiDAR resumes after an IMU-only stretch).
  void SkipScansTo(time::SensorTime t) { scan_end_ = t; }
  time::SensorTime scan_end() const { return scan_end_; }

  LioState state() const { return runner_ ? runner_->state() : pipeline_->state(); }
  std::uint32_t epoch() const { return runner_ ? runner_->epoch() : pipeline_->epoch(); }
  std::uint32_t in_flight() const { return runner_ ? runner_->frontend().stats().scans_in_flight : 0U; }

  Result<StepOutputs, EstimatorReason> Imu(const ImuInput& imu) {
    clock_->now_ns.store(imu.t.ns);
    auto r = runner_ ? runner_->push_imu(imu, Snap(imu.t)) : pipeline_->push_imu(imu, Snap(imu.t));
    Record(r);
    return r;
  }

  Result<StepOutputs, EstimatorReason> Scan(ScanInput&& scan) {
    const time::SensorTime t = scan.end;
    auto r = runner_ ? runner_->push_scan(std::move(scan), Snap(t)) : pipeline_->push_scan(std::move(scan), Snap(t));
    Record(r);
    return r;
  }

  StepOutputs Poll() {
    StepOutputs out = runner_->poll_results(Snap(now()));
    Record(Result<StepOutputs, EstimatorReason>(out));
    return out;
  }

  /// Runner: polls until every admitted scan has been answered and applied (the real-time equivalent of
  /// ICP finishing within a scan period). False on timeout or abort.
  bool Drain() {
    if (!runner_) return true;
    return Eventually(
        [&] {
          Poll();
          return in_flight() == 0;
        },
        &abort_);
  }

  /// Pushes `imu` (its stamp must be the next one) and advances the IMU schedule past it.
  void ImuUntilWith(const ImuInput& imu) {
    const auto r = Imu(imu);
    EXPECT_TRUE(r.has_value());
    next_imu_ = imu.t + kImuPeriod;
  }

  void ImuUntil(time::SensorTime end) {
    while (next_imu_ <= end) {
      const auto r = Imu(scene::make_imu(traj_, next_imu_));
      EXPECT_TRUE(r.has_value());
      next_imu_ = next_imu_ + kImuPeriod;
    }
  }

  ScanInput MakeScan(ScanKind kind, time::SensorTime start, time::SensorTime end) const {
    const scene::Pose pose = traj_.at(end);
    switch (kind) {
      case ScanKind::kRoom:
        return scene::make_scan(pose, start, end);
      case ScanKind::kPlane:
        return scene::make_plane_only_scan(pose, start, end);
      case ScanKind::kEmpty:
        return ScanInput{start, end, {}, false};
    }
    return {};
  }

  /// `n` scan periods: IMU up to each scan end, then the scan (when `kind` is set); paced: then Drain().
  void Run(int n, std::optional<ScanKind> kind = ScanKind::kRoom, bool paced = true) {
    for (int i = 0; i < n; ++i) {
      const time::SensorTime end = scan_end_ + kScanPeriod;
      ImuUntil(end);
      if (kind) {
        const auto r = Scan(MakeScan(*kind, scan_end_, end));
        EXPECT_TRUE(r.has_value() || (!paced && r.error() == EstimatorReason::kBackendBusy));
        if (paced) {
          EXPECT_TRUE(Drain()) << "the backend did not answer within 10 s";
        }
      }
      scan_end_ = end;
    }
  }

  void RunUntilTracking(int max_scans = 40) {
    for (int i = 0; i < max_scans && state() != LioState::kTracking; ++i) Run(1);
    ASSERT_EQ(state(), LioState::kTracking);
  }

  /// Every StateTransition as "BEFORE>AFTER:REASON", in order.
  std::vector<std::string> Transitions() {
    std::vector<std::string> out;
    for (const auto& e : Events("StateTransition")) {
      out.push_back(std::string(e.state_before) + ">" + std::string(e.state_after) + ":" + std::string(e.reason));
    }
    return out;
  }

  /// The state output stamped `t` (nullopt when none).
  std::optional<StateOutput> StateAt(time::SensorTime t) const {
    for (const StepOutputs& o : outputs_) {
      for (const StateOutput& s : o.states) {
        if (s.sample.t == t) return s;
      }
    }
    return std::nullopt;
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
  /// The LIO state after every call, consecutive repeats removed.
  const std::vector<LioState>& states() const { return states_; }
  bool all_finite() const { return all_finite_; }

 private:
  void Record(const Result<StepOutputs, EstimatorReason>& r) {
    const LioState s = state();
    if (states_.empty() || states_.back() != s) states_.push_back(s);
    if (!r) return;
    if (!Finite(*r)) all_finite_ = false;
    outputs_.push_back(*r);
  }

  std::shared_ptr<Store> store_;
  std::unique_ptr<events::EventRecorder> recorder_;
  std::shared_ptr<ClockControl> clock_;
  std::atomic<bool> abort_{false};
  std::unique_ptr<LioPipeline> pipeline_;
  std::unique_ptr<LioRunner> runner_;  // after the recorder and the clock: destroyed (joined) first
  scene::Trajectory traj_;
  time::SensorTime next_imu_{kT0};
  time::SensorTime scan_end_{kT0};
  std::vector<StepOutputs> outputs_;
  std::vector<LioState> states_;
  bool all_finite_{true};
};

/// Runs `fn` on a std::async thread, bounded by kGuard. On timeout the rig is aborted (gate opened, polling
/// loops stopped) BEFORE the future is waited on, so a stalled step fails the test instead of hanging.
template <class F>
auto Bounded(Rig& rig, F&& fn) -> std::pair<bool, decltype(fn())> {
  auto future = std::async(std::launch::async, std::forward<F>(fn));
  const bool ready = future.wait_for(kGuard) == std::future_status::ready;
  if (!ready) rig.Abort();
  return {ready, future.get()};
}

/// The base_link pose of a predictor output (the IMU's), as the frontend builds its restart seed (D47):
/// R_wb = R_wi R_bi^T, p_wb = p_wi - R_wb t_bi.
SeedPose BasePose(const StateOutput& s) {
  const Eigen::Isometry3d base_T_imu = scene::base_T_imu();
  const Eigen::Quaterniond q_wb =
      (s.sample.q_world_imu * Eigen::Quaterniond(base_T_imu.linear()).conjugate()).normalized();
  return SeedPose{s.sample.p_world_m - q_wb * base_T_imu.translation(), q_wb};
}

double Yaw(const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d r = q.toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}

/// The applied restart answer of the rig's calls, polling until one arrives (nullopt after kGuard).
std::optional<Result<ResetDelta, EstimatorReason>> AnswerOf(Rig& rig) {
  std::optional<Result<ResetDelta, EstimatorReason>> answer;
  const bool got = Eventually(
      [&] {
        StepOutputs o = rig.Poll();
        if (o.restart) answer = o.restart;
        return answer.has_value();
      },
      &rig.aborted());
  EXPECT_TRUE(got) << "no restart answer";
  return answer;
}

/// TRACKING, a 1 s LiDAR gap (LOST), then 20 paced room scans (1.9 s, still LOST: the dwell runs from the first of
/// them). The backend gate is then closed and the IMU pushed to the end of the dwell: the frontend reaches RESTARTING
/// and queues its own restart command, which the held backend does not answer. Returns the command time.
time::SensorTime ToRestartingHeld(Rig& rig) {
  rig.RunUntilTracking();
  rig.Run(10, std::nullopt);  // LiDAR gap -> LOST
  EXPECT_EQ(rig.state(), LioState::kLost);
  const time::SensorTime first = rig.scan_end() + kScanPeriod;
  rig.Run(20);
  EXPECT_EQ(rig.state(), LioState::kLost) << "1.9 s of scans: inside the dwell";
  rig.clock().gate.close();
  rig.ImuUntil(first + time::seconds(2));
  EXPECT_EQ(rig.state(), LioState::kRestarting);
  EXPECT_EQ(rig.epoch(), 1U);
  return rig.now();
}

}  // namespace

// ---------------------------------------------------------------------------------------------------
// First in the file: no earlier test of this binary left OpenMP pool threads behind that could exit meanwhile.

TEST(LioRunner, CreateStartsOneBackendThreadAndDestructorJoinsIt) {
  // The recorder's writer thread is running before `base` is read; no thread was joined just before.
  auto store = std::make_shared<Store>();
  auto made = events::EventRecorder::create(std::make_unique<FakeSink>(store));
  ASSERT_TRUE(made.has_value());
  std::unique_ptr<events::EventRecorder> recorder = std::move(*made);
  const int base = ThreadCount();
  // The kernel's count can lag a create or a join slightly: every check polls.
  const auto threads_are = [](int n) { return Eventually([n] { return ThreadCount() == n; }); };

  auto clock = std::make_shared<ClockControl>();
  auto created =
      LioRunner::create(DefaultConfig(), scene::base_T_imu(), scene::imu_T_lidar(), *recorder, MakeClock(clock));
  ASSERT_TRUE(created.has_value());
  std::unique_ptr<LioRunner> runner = std::move(*created);
  EXPECT_TRUE(threads_are(base + 1)) << "create() starts exactly one thread";

  scene::Trajectory traj;
  traj.t0 = kT0;
  for (int i = 0; i < 5; ++i) {
    const time::SensorTime t = kT0 + time::Duration{i * kImuPeriod.ns};
    ASSERT_TRUE(runner->push_imu(scene::make_imu(traj, t), Snap(t)).has_value());
  }
  ASSERT_TRUE(Eventually([&] { return clock->calls.load() == 5; })) << "one clock call per request";
  EXPECT_NE(clock->thread.load(), std::this_thread::get_id()) << "the backend runs on its own thread";
  EXPECT_FALSE(clock->other_thread.load()) << "every request is processed on the same thread";

  // stop() is idempotent; after it the frontend still works without a backend.
  runner->stop();
  runner->stop();
  EXPECT_TRUE(threads_are(base)) << "stop() joined the backend thread";
  time::SensorTime t = kT0 + time::Duration{5 * kImuPeriod.ns};
  for (int i = 0; i < 40; ++i, t = t + kImuPeriod) {
    ASSERT_TRUE(runner->push_imu(scene::make_imu(traj, t), Snap(t)).has_value());
  }
  const time::SensorTime last_imu = t - kImuPeriod;
  ASSERT_TRUE(runner->push_scan(scene::make_scan(traj.at(last_imu), kT0, kT0 + kScanPeriod), Snap(last_imu)));
  ASSERT_TRUE(runner->push_scan(scene::make_scan(traj.at(last_imu), kT0 + kScanPeriod, last_imu), Snap(last_imu)));
  const auto busy = runner->push_scan(scene::make_scan(traj.at(last_imu), last_imu, last_imu), Snap(last_imu));
  ASSERT_FALSE(busy.has_value());
  EXPECT_EQ(busy.error(), EstimatorReason::kBackendBusy) << "nobody answers the 2 scans in flight";
  EXPECT_EQ(runner->frontend().stats().imu_accepted, 45U);
  EXPECT_EQ(clock->calls.load(), 5U) << "no request was processed after stop()";

  // The destructor (after stop) returns; a second runner is destroyed without stop() and joins as well.
  // get() also joins the std::async thread, so it no longer counts below.
  auto destroyed = std::async(std::launch::async, [&] { runner.reset(); });
  ASSERT_EQ(destroyed.wait_for(kGuard), std::future_status::ready);
  destroyed.get();
  EXPECT_TRUE(threads_are(base));
  auto second = LioRunner::create(DefaultConfig(), scene::base_T_imu(), scene::imu_T_lidar(), *recorder,
                                  MakeClock(std::make_shared<ClockControl>()));
  ASSERT_TRUE(second.has_value());
  EXPECT_TRUE(threads_are(base + 1));
  auto joined = std::async(std::launch::async, [r = std::move(*second)]() mutable { r.reset(); });
  ASSERT_EQ(joined.wait_for(kGuard), std::future_status::ready);
  joined.get();
  EXPECT_TRUE(threads_are(base)) << "the destructor joined the backend thread";

  // The recorder outlives the runners: nothing reaches its ring after they are gone.
  recorder->flush();
  const std::uint64_t emitted = recorder->stats().emitted;
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_EQ(recorder->stats().emitted, emitted);
}

TEST(LioRunner, CreateRejectsAnEmptyClock) {
  auto store = std::make_shared<Store>();
  auto recorder = events::EventRecorder::create(std::make_unique<FakeSink>(store));
  ASSERT_TRUE(recorder.has_value());
  const auto r = LioRunner::create(DefaultConfig(), scene::base_T_imu(), scene::imu_T_lidar(), **recorder, LioClock{});
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, config::ConfigError::Kind::kMissingKey);
  EXPECT_EQ(r.error().key, "backend_clock");

  // The extrinsic check of the backend still applies.
  Eigen::Isometry3d imu_T_lidar = scene::imu_T_lidar();
  imu_T_lidar.translation().x() += 0.01;
  const auto bad = LioRunner::create(DefaultConfig(), scene::base_T_imu(), imu_T_lidar, **recorder,
                                     MakeClock(std::make_shared<ClockControl>()));
  ASSERT_FALSE(bad.has_value());
  EXPECT_EQ(bad.error().key, "extrinsic_imu_lidar_x_m");
}

// Review Focus 2 (F22 after the split): a stalled backend never delays the ingest thread; it reads as a LiDAR gap.
TEST(LioRunner, BlockedBackendDoesNotDelayGapCheckOrStateOutput) {
  Rig rig;
  rig.RunUntilTracking();
  // Paced: the last scan (end = newest IMU sample) has been answered and applied.
  const time::SensorTime last = rig.now();
  ASSERT_EQ(last, rig.scan_end());
  const std::uint64_t popped_before = rig.runner().request_queue_stats().popped;

  rig.clock().gate.close();
  struct Phase {
    int imu_calls{0};
    int one_state_each{0};
    int scans_ok{0};
    int scans_busy{0};
    std::optional<time::SensorTime> degraded_at, lost_at;
  };
  const auto [ready, phase] = Bounded(rig, [&] {
    Phase p;
    scene::Trajectory traj;
    traj.t0 = kT0;
    time::SensorTime scan_start = last;
    for (int i = 1; i <= 400; ++i) {  // 2 s of 200 Hz IMU
      const time::SensorTime t = last + time::Duration{i * kImuPeriod.ns};
      rig.clock().now_ns.store(t.ns);
      const auto r = rig.runner().push_imu(scene::make_imu(traj, t), Snap(t));
      ++p.imu_calls;
      if (r && r->states.size() == 1U) ++p.one_state_each;
      if (!p.degraded_at && rig.runner().state() == LioState::kDegraded) p.degraded_at = t;
      if (!p.lost_at && rig.runner().state() == LioState::kLost) p.lost_at = t;
      if (i % 20 == 0) {  // a scan every 100 ms
        const auto s = rig.runner().push_scan(scene::make_scan(traj.at(t), scan_start, t), Snap(t));
        if (s) ++p.scans_ok;
        if (!s && s.error() == EstimatorReason::kBackendBusy) ++p.scans_busy;
        scan_start = t;
      }
    }
    return p;
  });
  ASSERT_TRUE(ready) << "the ingest thread waited on the blocked backend";
  EXPECT_TRUE(Eventually([&] { return rig.clock().gate.blocked() == 1; })) << "the backend is held in the gate";
  EXPECT_EQ(rig.runner().request_queue_stats().popped, popped_before) << "the backend processed nothing";

  EXPECT_EQ(phase.imu_calls, 400);
  EXPECT_EQ(phase.one_state_each, 400) << "every IMU sample produced exactly one state output";
  EXPECT_EQ(phase.scans_ok, 2) << "kMaxScansInFlight scans admitted";
  EXPECT_EQ(phase.scans_busy, 18) << "the rest refused (drop newest), never waited for";
  ASSERT_TRUE(phase.degraded_at.has_value());
  ASSERT_TRUE(phase.lost_at.has_value());
  EXPECT_EQ(*phase.degraded_at, last + time::milliseconds(255)) << "first IMU t > last applied scan end + 0.25 s";
  EXPECT_EQ(*phase.lost_at, last + time::milliseconds(505)) << "first IMU t > last applied scan end + 0.5 s";

  rig.clock().gate.open();
  const auto [drained, ok] = Bounded(rig, [&] { return rig.Drain(); });
  ASSERT_TRUE(drained);
  EXPECT_TRUE(ok) << "the queued scans are answered once the backend runs again";
  EXPECT_EQ(rig.runner().frontend().stats().scans_in_flight, 0U);
}

// Review Focus 4: unpaced producer against the running backend; every scan is accounted for (TSan target).
TEST(LioRunner, ConcurrentIngestAndBackendAccountForEveryScan) {
  Rig rig;
  struct Counts {
    std::uint64_t pushed{0}, ok{0}, busy{0}, other{0};
  };
  const auto [ready, counts] = Bounded(rig, [&] {
    Counts c;
    scene::Trajectory traj;
    traj.t0 = kT0;
    time::SensorTime t = kT0;
    time::SensorTime scan_start = kT0;
    for (int scan = 1; scan <= 300; ++scan) {  // 30 s: 200 Hz IMU, 10 Hz scans, as fast as possible
      for (int k = 0; k < 20; ++k, t = t + kImuPeriod) {
        rig.clock().now_ns.store(t.ns);
        if (!rig.runner().push_imu(scene::make_imu(traj, t), Snap(t))) ++c.other;
      }
      const time::SensorTime end = t - kImuPeriod;
      const auto r = rig.runner().push_scan(scene::make_scan(traj.at(end), scan_start, end), Snap(end));
      ++c.pushed;
      if (r) ++c.ok;
      else if (r.error() == EstimatorReason::kBackendBusy) ++c.busy;
      else ++c.other;
      scan_start = end;
    }
    return c;
  });
  ASSERT_TRUE(ready);
  EXPECT_EQ(counts.other, 0U) << "only Ok or kBackendBusy";

  const auto [drained, ok] = Bounded(rig, [&] {
    return Eventually(
        [&] {
          (void)rig.runner().poll_results(Snap(rig.now()));
          const auto q = rig.runner().request_queue_stats();
          return rig.runner().frontend().stats().scans_in_flight == 0 && q.pushed == q.popped;
        },
        &rig.aborted());
  });
  ASSERT_TRUE(drained);
  ASSERT_TRUE(ok);
  rig.runner().stop();
  (void)rig.runner().poll_results(Snap(rig.now()));  // a result pushed by the last request

  const FrontendStats f = rig.runner().frontend().stats();
  const auto req = rig.runner().request_queue_stats();
  const auto res = rig.runner().result_queue_stats();
  EXPECT_EQ(counts.pushed, 300U);
  EXPECT_EQ(counts.ok, f.scans_admitted);
  EXPECT_EQ(counts.busy, f.scans_refused_busy);
  EXPECT_EQ(counts.pushed, f.scans_admitted + f.scans_refused_busy);
  EXPECT_EQ(f.scans_in_flight, 0U);
  // The unpaced producer can outrun the backend into a LiDAR gap: LOST, then LIO restarts itself (D41). Each of its
  // restart commands is one request and one answer.
  const std::size_t answers = rig.Events("LioRestart").size() + rig.Events("LioRestartRejected").size();
  RecordProperty("restart_answers", std::to_string(answers));
  EXPECT_EQ(f.results_applied + f.results_discarded, f.scans_admitted + 1U + answers) << "+1: the IMU initialisation";
  EXPECT_EQ(req.pushed, req.popped);
  EXPECT_EQ(req.rejected, f.requests_rejected);
  EXPECT_GE(f.requests_rejected, f.imu_copies_dropped);
  EXPECT_EQ(res.rejected, 0U) << "the result queue cannot fill (limits.hpp)";
  EXPECT_EQ(res.pushed, res.popped);
  // backend().stats() is valid after stop(): every queued request was processed.
  const BackendStats b = rig.runner().backend().stats();
  EXPECT_EQ(b.scans, f.scans_admitted);
  EXPECT_EQ(b.imu_samples + b.scans + answers, req.popped);
  EXPECT_EQ(b.results_dropped, 0U);
  RecordProperty("scans_admitted", std::to_string(f.scans_admitted));
  RecordProperty("imu_copies_dropped", std::to_string(f.imu_copies_dropped));
}

// The product assembly behaves like the synchronous driver when the backend keeps up (paced).
TEST(LioRunner, ResultsAndStatesMatchThePipelineWhenPaced) {
  const auto scenario = [](Rig& r) {
    r.RunUntilTracking();
    r.Run(2, ScanKind::kPlane);
    r.Run(4);
    r.Run(10, std::nullopt);  // LiDAR gap -> LOST
    // Room scans until LIO has restarted itself (dwell 2 s): the paced runner applies the answer, at the latest,
    // with the result of the scan admitted after it (one FIFO), so both drivers leave the loop at the same scan.
    for (int i = 0; i < 30 && r.epoch() == 1U; ++i) r.Run(1);
    ASSERT_EQ(r.epoch(), 2U);
    r.RunUntilTracking();
    r.Run(3);
  };
  Rig pipeline(Mode::kPipeline);
  scenario(pipeline);
  Rig runner;
  const auto [ready, unused] = Bounded(runner, [&] {
    scenario(runner);
    return true;
  });
  ASSERT_TRUE(ready);

  // The synchronous driver answers the restart inside the call that issued it (no RESTARTING after a call), the
  // runner a call later: compare the transitions, not the per-call states.
  EXPECT_EQ(runner.Transitions(), pipeline.Transitions());
  EXPECT_EQ(runner.epoch(), pipeline.epoch());
  EXPECT_EQ(runner.epoch(), 2U);
  const auto count = [](const Rig& r) {
    return std::count_if(r.outputs().begin(), r.outputs().end(), [](const StepOutputs& o) { return o.odometry.has_value(); });
  };
  EXPECT_EQ(count(runner), count(pipeline));
  EXPECT_GE(count(runner), 5);
  const auto delta = [](const Rig& r) {
    std::optional<Result<ResetDelta, EstimatorReason>> d;
    for (const StepOutputs& o : r.outputs()) {
      if (o.restart) d = o.restart;
    }
    return d;
  };
  const auto a = delta(runner);
  const auto b = delta(pipeline);
  ASSERT_TRUE(a && *a);
  ASSERT_TRUE(b && *b);
  EXPECT_LT(((*a)->position_m - (*b)->position_m).norm(), 1e-9);
  EXPECT_LT(((*a)->velocity_mps - (*b)->velocity_mps).norm(), 1e-9);
  EXPECT_NEAR((*a)->yaw_rad, (*b)->yaw_rad, 1e-12);
  EXPECT_TRUE(runner.all_finite());
}

// Shutdown (design table): request_stop leaves the queue unprocessed; run() returns after the request in progress.
TEST(LioRunner, ShutdownWithQueuedRequestsReturns) {
  Rig rig;
  rig.clock().gate.close();
  // Pushes against the closed gate never wait; bounded so that a regression fails instead of hanging.
  const auto [pushed, admitted] = Bounded(rig, [&] {
    rig.ImuUntil(kT0 + time::Duration{299 * kImuPeriod.ns});  // 300 IMU copies
    const time::SensorTime last = rig.now();
    const bool a = rig.Scan(rig.MakeScan(ScanKind::kRoom, last - kScanPeriod - kScanPeriod, last - kScanPeriod))
                       .has_value();
    const bool b = rig.Scan(rig.MakeScan(ScanKind::kRoom, last - kScanPeriod, last)).has_value();
    return a && b;
  });
  ASSERT_TRUE(pushed) << "the ingest thread waited on the blocked backend";
  ASSERT_TRUE(admitted);
  ASSERT_TRUE(Eventually([&] { return rig.clock().gate.blocked() == 1; })) << "the backend waits in the gate";
  auto q = rig.runner().request_queue_stats();
  ASSERT_EQ(q.pushed, 302U);
  ASSERT_EQ(q.popped, 0U);

  rig.runner().request_stop();
  rig.clock().gate.open();
  const auto [ready, unused] = Bounded(rig, [&] {
    rig.runner().stop();
    return true;
  });
  ASSERT_TRUE(ready) << "stop() returned with 301 requests still queued";
  q = rig.runner().request_queue_stats();
  EXPECT_EQ(q.popped, 1U) << "only the request in progress was processed";
  EXPECT_EQ(rig.runner().result_queue_stats().pushed, 0U) << "no result after run() returned";
  EXPECT_EQ(rig.runner().frontend().stats().scans_in_flight, 2U);
}

// Review note (1): the frontend checks busy before applying waiting results, so the runner polls first.
TEST(LioRunner, ScanIsNotRefusedBusyWhileItsAnswersWait) {
  Rig rig;
  rig.RunUntilTracking();
  // Three scans inside 0.2 s of IMU, so the gap rule (0.25 s) keeps TRACKING while the answers wait.
  const time::SensorTime s0 = rig.scan_end();
  const time::SensorTime e1 = s0 + kScanPeriod;
  const time::SensorTime e2 = e1 + time::milliseconds(50);
  const time::SensorTime e3 = e2 + time::milliseconds(50);
  rig.ImuUntil(e3);
  ASSERT_TRUE(rig.Scan(rig.MakeScan(ScanKind::kRoom, s0, e1)));
  ASSERT_TRUE(rig.Scan(rig.MakeScan(ScanKind::kRoom, e1, e2)));
  ASSERT_TRUE(Eventually([&] {
    const auto r = rig.runner().result_queue_stats();
    return r.pushed - r.popped == 2U;
  })) << "both answers wait in the result queue";
  ASSERT_EQ(rig.in_flight(), 2U);

  const auto third = rig.Scan(rig.MakeScan(ScanKind::kRoom, e2, e3));
  ASSERT_TRUE(third.has_value()) << "refused " << to_string(third.error()) << " although both answers were waiting";
  ASSERT_TRUE(third->odometry.has_value());
  EXPECT_EQ(third->odometry->t, e2) << "last wins: the newer of the two applied results";
  EXPECT_EQ(rig.in_flight(), 1U);
}

// The outputs of results applied by a push_scan that is then refused are returned by the next call.
TEST(LioRunner, OutputsOfResultsAppliedByARefusedScanAreNotLost) {
  Rig rig;
  rig.RunUntilTracking();
  const time::SensorTime s0 = rig.scan_end();
  const time::SensorTime e1 = s0 + kScanPeriod;
  const time::SensorTime e2 = e1 + kScanPeriod;
  rig.ImuUntil(e2);
  ASSERT_TRUE(rig.Scan(rig.MakeScan(ScanKind::kRoom, s0, e1)));
  ASSERT_TRUE(rig.Scan(rig.MakeScan(ScanKind::kRoom, e1, e2)));
  ASSERT_TRUE(Eventually([&] {
    const auto r = rig.runner().result_queue_stats();
    return r.pushed - r.popped == 2U;
  }));
  const auto refused = rig.Scan(rig.MakeScan(ScanKind::kRoom, s0, e1));  // ends before the last admitted scan
  ASSERT_FALSE(refused.has_value());
  EXPECT_EQ(refused.error(), EstimatorReason::kOutOfOrder);
  EXPECT_EQ(rig.in_flight(), 0U) << "the poll before the push applied both answers";

  const auto next = rig.Imu(scene::make_imu({}, e2 + kImuPeriod));
  ASSERT_TRUE(next.has_value());
  ASSERT_TRUE(next->odometry.has_value()) << "the refused call's odometry is carried to the next call";
  EXPECT_EQ(next->odometry->t, e2);
  EXPECT_EQ(next->states.size(), 1U);
  EXPECT_FALSE(rig.Poll().odometry.has_value()) << "returned once";
}

// D33/O16 (replaces LateRestartAnswerRestartsThePredictorAndRecovers): a kRestartSeeded applied more than
// kPredictorBufferSpan after its command is refused (RESTART_ANSWER_STALE, kStaleAnswer), the state stays
// RESTARTING and the output is not rewound; the frontend's own retry (D41: at the next IMU sample, no caller) reuses
// epoch 2 and recovers. A scan admitted while the refused restart was pending (processed by the real backend in the
// refused seed's world) is discarded.
TEST(LioRunner, LateRestartAnswerIsRefusedAndARetrySucceeds) {
  Rig rig;
  const time::SensorTime cmd_t = ToRestartingHeld(rig);
  const std::uint64_t discarded = rig.runner().frontend().stats().results_discarded;
  const auto [ready, ok] = Bounded(rig, [&] {
    rig.ImuUntil(cmd_t + time::milliseconds(50));
    const bool admitted = rig.Scan(rig.MakeScan(ScanKind::kRoom, rig.scan_end(), rig.now())).has_value();
    rig.ImuUntil(cmd_t + time::milliseconds(400));  // > kPredictorBufferSpan of IMU while the answer is held
    return admitted && Eventually([&] { return rig.clock().gate.blocked() == 1; }, &rig.aborted());
  });
  ASSERT_TRUE(ready);
  ASSERT_TRUE(ok) << "the scan was admitted and the backend held the restart command";
  EXPECT_EQ(rig.state(), LioState::kRestarting);
  EXPECT_EQ(rig.epoch(), 1U);

  rig.clock().gate.open();
  const auto stale = AnswerOf(rig);
  ASSERT_TRUE(stale.has_value());
  ASSERT_FALSE(stale->has_value()) << "the late answer was applied";
  EXPECT_EQ(stale->error(), EstimatorReason::kStaleAnswer);
  EXPECT_EQ(rig.state(), LioState::kRestarting);
  EXPECT_EQ(rig.epoch(), 1U);
  const auto rejected = rig.Events("LioRestartRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "RESTART_ANSWER_STALE");
  EXPECT_NEAR(Value(rejected[0], "seed_age_s"), 0.4, 1e-9);
  EXPECT_TRUE(rig.Events("LioRestart").empty());

  // The retry: the next IMU sample issues a fresh command at its time, epoch 2 again; its answer is timely.
  rig.ImuUntil(rig.now() + kImuPeriod);
  const auto seeded = AnswerOf(rig);
  ASSERT_TRUE(seeded.has_value());
  ASSERT_TRUE(seeded->has_value()) << "the retry was refused";
  EXPECT_EQ(rig.state(), LioState::kInitializing);
  EXPECT_EQ(rig.epoch(), 2U);
  const auto restarts = rig.Events("LioRestart");
  ASSERT_EQ(restarts.size(), 1U);
  EXPECT_LT(Value(restarts[0], "seed_age_s"), 0.3);
  ASSERT_TRUE(rig.Drain());
  EXPECT_EQ(rig.runner().frontend().stats().results_discarded, discarded + 1)
      << "the scan of the refused epoch is discarded";

  rig.ImuUntil(rig.now() + kImuPeriod);
  ASSERT_EQ(rig.outputs().back().states.size(), 1U);
  EXPECT_EQ(rig.outputs().back().states[0].epoch, 2U);
  EXPECT_EQ(rig.outputs().back().states[0].sample.reset_counter, 1U);

  rig.SkipScansTo(rig.now());  // the LiDAR resumes now
  const auto [recovered, unused] = Bounded(rig, [&] {
    rig.RunUntilTracking();
    rig.Run(2);
    return true;
  });
  ASSERT_TRUE(recovered);
  EXPECT_EQ(rig.state(), LioState::kTracking) << "the confirm scans bring the new epoch to TRACKING";
  EXPECT_TRUE(rig.all_finite());

  // Never a rewind: the state outputs' times increase across the refusal and the retry.
  std::optional<time::SensorTime> last;
  for (const StepOutputs& o : rig.outputs()) {
    for (const StateOutput& s : o.states) {
      if (last) {
        EXPECT_GT(s.sample.t, *last);
      }
      last = s.sample.t;
    }
  }
}

// --- D41, D47, AR4: LIO restarts itself ------------------------------------------------------------------------

// Nobody calls anything but push_imu / push_scan / poll_results (there is no restart API): after a LiDAR gap LIO is
// LOST, and once the scans have come back non-empty and without a gap for restart_dwell_s (2 s) it restarts itself,
// seeded with its predictor's newest pose, so the reset is continuous (D47: the seed only picks the gauge).
TEST(LioRunner, RestartsItselfAfterTheDwellWithoutAnyCall) {
  Rig rig;
  rig.RunUntilTracking();
  rig.Run(10, std::nullopt);  // LiDAR gap -> LOST
  ASSERT_EQ(rig.state(), LioState::kLost);
  const time::SensorTime first = rig.scan_end() + kScanPeriod;  // the first scan after the gap
  for (int i = 0; i < 30 && rig.epoch() == 1U; ++i) {
    rig.Run(1);
    if (rig.epoch() == 1U && rig.state() == LioState::kLost) {
      EXPECT_TRUE(rig.Events("LioRestart").empty()) << "a restart inside the dwell";
    }
  }
  ASSERT_EQ(rig.epoch(), 2U);
  EXPECT_EQ(rig.state(), LioState::kInitializing);

  const std::vector<std::string> transitions = rig.Transitions();
  ASSERT_GE(transitions.size(), 5U);
  EXPECT_EQ(transitions[transitions.size() - 2], "LOST>RESTARTING:STABLE_DATA_DWELL");
  EXPECT_EQ(transitions.back(), "RESTARTING>INITIALIZING:RESTART_SEEDED");
  const auto changes = rig.Events("StateTransition");
  const time::SensorTime restart_t = first + time::seconds(2);
  EXPECT_DOUBLE_EQ(Value(changes[changes.size() - 2], "sensor_time_s"), static_cast<double>(restart_t.ns) / 1e9)
      << "RESTARTING at the IMU tick restart_dwell after the first scan";

  // The seed is the predictor's output at the restart tick, at base_link; the reset moves nothing.
  const auto restarts = rig.Events("LioRestart");
  ASSERT_EQ(restarts.size(), 1U);
  EXPECT_EQ(restarts[0].reason, "STABLE_DATA_DWELL");
  EXPECT_EQ(Value(restarts[0], "old_epoch"), 1.0);
  EXPECT_EQ(Value(restarts[0], "new_epoch"), 2.0);
  const std::optional<StateOutput> at_restart = rig.StateAt(restart_t);
  ASSERT_TRUE(at_restart.has_value());
  const SeedPose expected = BasePose(*at_restart);
  EXPECT_NEAR(Value(restarts[0], "seed_x_m"), expected.p_world_m.x(), 1e-9);
  EXPECT_NEAR(Value(restarts[0], "seed_y_m"), expected.p_world_m.y(), 1e-9);
  EXPECT_NEAR(Value(restarts[0], "seed_z_m"), expected.p_world_m.z(), 1e-9);
  EXPECT_NEAR(Value(restarts[0], "seed_yaw_rad"), Yaw(expected.q_world_base), 1e-9);
  for (const std::string_view key : {"delta_x_m", "delta_y_m", "delta_z_m", "delta_yaw_rad"}) {
    EXPECT_NEAR(Value(restarts[0], key), 0.0, 1e-6) << key;
  }
  EXPECT_TRUE(rig.Events("LioRestartRejected").empty());

  rig.RunUntilTracking();
  EXPECT_EQ(rig.epoch(), 2U);
  std::optional<OdometryOutput> odometry;
  for (const StepOutputs& o : rig.outputs()) {
    if (o.odometry) odometry = o.odometry;
  }
  ASSERT_TRUE(odometry.has_value());
  EXPECT_EQ(odometry->epoch, 2U);
  EXPECT_EQ(odometry->reset_counter, 1U);
  EXPECT_TRUE(rig.all_finite());
}

// D33/O16 with D41: a late answer is refused and the frontend issues the next command by itself, at the next IMU
// sample, seeded with the predictor's NEWEST pose and velocity (here the IMU accelerated the output about 0.08 m away
// from the refused command's pose meanwhile).
TEST(LioRunner, RefusedLateAnswerIsReissuedWithTheNewestPose) {
  Rig rig;
  const time::SensorTime cmd_t = ToRestartingHeld(rig);
  const auto push = [&rig](time::SensorTime t) {  // 1 m/s^2 along x
    rig.ImuUntilWith(ImuInput{t, Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, scene::kGravity)});
  };
  const auto [ready, ok] = Bounded(rig, [&] {
    while (rig.now() < cmd_t + time::milliseconds(400)) push(rig.now() + kImuPeriod);
    return Eventually([&] { return rig.clock().gate.blocked() == 1; }, &rig.aborted());
  });
  ASSERT_TRUE(ready && ok);
  rig.clock().gate.open();
  const auto stale = AnswerOf(rig);
  ASSERT_TRUE(stale.has_value());
  ASSERT_FALSE(stale->has_value());
  EXPECT_EQ(stale->error(), EstimatorReason::kStaleAnswer);
  EXPECT_EQ(rig.state(), LioState::kRestarting);

  const time::SensorTime retry_t = rig.now() + kImuPeriod;
  push(retry_t);
  const auto seeded = AnswerOf(rig);
  ASSERT_TRUE(seeded.has_value());
  ASSERT_TRUE(seeded->has_value()) << "the re-issued restart was refused";
  EXPECT_EQ(rig.state(), LioState::kInitializing);
  EXPECT_EQ(rig.epoch(), 2U);

  const std::optional<StateOutput> at_cmd = rig.StateAt(cmd_t);
  const std::optional<StateOutput> at_retry = rig.StateAt(retry_t);
  ASSERT_TRUE(at_cmd.has_value());
  ASSERT_TRUE(at_retry.has_value());
  const SeedPose newest = BasePose(*at_retry);
  EXPECT_GT((newest.p_world_m - BasePose(*at_cmd).p_world_m).norm(), 0.05) << "the output moved while held";
  const auto restarts = rig.Events("LioRestart");
  ASSERT_EQ(restarts.size(), 1U);
  EXPECT_NEAR(Value(restarts[0], "seed_x_m"), newest.p_world_m.x(), 1e-9);
  EXPECT_NEAR(Value(restarts[0], "seed_y_m"), newest.p_world_m.y(), 1e-9);
  EXPECT_NEAR(Value(restarts[0], "seed_z_m"), newest.p_world_m.z(), 1e-9);
  EXPECT_NEAR(Value(restarts[0], "seed_yaw_rad"), Yaw(newest.q_world_base), 1e-9);
  EXPECT_LT(Value(restarts[0], "seed_age_s"), 0.3);
  // Position and velocity are carried (the seed is the predictor's own state): the reset moves nothing.
  EXPECT_LT((*seeded)->position_m.norm(), 1e-6);
  EXPECT_LT((*seeded)->velocity_mps.norm(), 1e-6);
  EXPECT_GT(at_retry->sample.v_world_mps.x(), 0.35);
  const auto rejected = rig.Events("LioRestartRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), static_cast<double>(cmd_t.ns) / 1e9);
}

// Review Focus 2, the edge of geometry: scans that arrive and are not empty but never confirm (floor only). LIO
// alternates LOST (wait restart_dwell) -> RESTARTING -> INITIALIZING (no confirmation within restart_dwell) -> LOST,
// so two attempts are at least two dwells apart; there is no attempt counter and no give-up. While the scans are
// empty there is no attempt.
TEST(LioRunner, EdgeOfGeometryAttemptsAreAtLeastTwoDwellsApart) {
  Rig rig;
  rig.RunUntilTracking();
  rig.Run(150, ScanKind::kPlane);  // 15 s of floor-only scans
  const double plane_end_s = static_cast<double>(rig.scan_end().ns) / 1e9;
  rig.Run(60, ScanKind::kEmpty);  // 6 s of empty scans
  ASSERT_TRUE(rig.Drain());

  std::vector<double> attempts;  // sensor time of every LOST -> RESTARTING
  std::size_t reinit = 0;        // INITIALIZING -> LOST (REINIT_NOT_CONFIRMED)
  std::size_t tracking = 0;
  for (const auto& e : rig.Events("StateTransition")) {
    if (e.state_after == "RESTARTING") {
      EXPECT_EQ(e.reason, "STABLE_DATA_DWELL");
      attempts.push_back(Value(e, "sensor_time_s"));
    }
    if (e.state_before == "INITIALIZING" && e.state_after == "LOST") {
      EXPECT_EQ(e.reason, "REINIT_NOT_CONFIRMED");
      ++reinit;
    }
    if (e.state_after == "TRACKING") ++tracking;
  }
  EXPECT_EQ(tracking, 1U) << "only the initial confirmation (room scans); floor-only scans never confirm";
  RecordProperty("attempts", std::to_string(attempts.size()));
  ASSERT_GE(attempts.size(), 2U) << "LIO gave up";
  EXPECT_GE(reinit, 2U);
  for (std::size_t i = 1; i < attempts.size(); ++i) {
    EXPECT_GE(attempts[i] - attempts[i - 1], 2 * 2.0) << i;
  }
  // None while the scans are empty: every attempt precedes the first empty scan.
  for (const double a : attempts) EXPECT_LE(a, plane_end_s + 0.1) << a;
  EXPECT_NE(rig.state(), LioState::kTracking);
  EXPECT_EQ(rig.Events("LioRestart").size(), attempts.size()) << "every attempt is answered and applied";
  EXPECT_EQ(rig.epoch(), 1U + attempts.size());
}

// Review note (3): a throwing clock never kills the backend worker (AGENTS.md §2.6): event, last good snapshot.
TEST(LioRunner, ThrowingClockIsAnEventNotADeadWorker) {
  Rig rig;
  rig.clock().throws_left.store(1001);
  const auto [ready, ok] = Bounded(rig, [&] {
    // 1100 requests in batches of 100, each drained, so no IMU copy is dropped: the first 1001 clock calls throw.
    for (int batch = 1; batch <= 11; ++batch) {
      rig.ImuUntil(kT0 + time::Duration{(batch * 100 - 1) * kImuPeriod.ns});
      const bool drained = Eventually(
          [&] {
            const auto q = rig.runner().request_queue_stats();
            return q.popped == q.pushed;
          },
          &rig.aborted());
      if (!drained) return false;
    }
    return rig.runner().request_queue_stats().popped == 1100U;
  });
  ASSERT_TRUE(ready);
  ASSERT_TRUE(ok) << "the backend kept processing requests";
  EXPECT_EQ(rig.clock().throws_left.load(), 0);
  rig.SkipScansTo(rig.now());

  const auto [tracking, unused] = Bounded(rig, [&] {
    rig.RunUntilTracking();
    return true;
  });
  ASSERT_TRUE(tracking);
  EXPECT_EQ(rig.state(), LioState::kTracking);

  ASSERT_EQ(rig.Events("LioClockFailed").size(), 2U) << "rate-limited: the 1st and the kEventSummaryEvery-th";
  rig.runner().stop();  // run() returned: the final count is reported once more
  const auto failed = rig.Events("LioClockFailed");
  ASSERT_EQ(failed.size(), 3U);
  EXPECT_EQ(failed[0].reason, "CLOCK_FAILED");
  EXPECT_EQ(Value(failed[0], "count"), 1.0);
  EXPECT_EQ(Value(failed[1], "count"), static_cast<double>(lio::limits::kEventSummaryEvery));
  EXPECT_EQ(Value(failed[2], "count"), 1001.0) << "the summary at the end of run()";
}
