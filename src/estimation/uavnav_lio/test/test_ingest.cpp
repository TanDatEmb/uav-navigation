// LioIngest: the ROS-free ingest layer of the lio node over LioRunner (SYSTEM_DESIGN §3.3, §3.5, D31, D33/O20, O21).
// These tests run the real LioRunner (its backend jthread); `make sanitize` runs them under ThreadSanitizer.
//
// Conventions:
// - Synthetic data only (room_scene.hpp). ROS time equals sensor time (SITL, D29) unless a test sets it apart.
// - No wall-clock bound below 1 s. Every LioIngest call that must never block runs under Bounded() (std::async +
//   wait_for(10 s)). LioIngest has nothing a test can unblock, so a call that does not return in 10 s fails the
//   test and aborts the binary instead of hanging the suite. Polling loops (Eventually) give up after 10 s.
// - A phase on a std::async thread hands the ingest back through future::get (happens-before), so the calls of
//   one test stay serialised, except in ConcurrentCallsAreRefusedNotRaced, which overlaps them on purpose.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "room_scene.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio_node/ingest.hpp"
#include "uavnav/lio_node/limits.hpp"
#include "uavnav/lio_node/node_config.hpp"

using namespace uavnav;
using namespace uavnav::lio_node;
namespace node_limits = uavnav::lio_node::limits;

namespace {

constexpr auto kGuard = std::chrono::seconds(10);
constexpr time::SensorTime kT0{1'000'000'000};  // sensor time must be > 0

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

double Value(const events::EventRecord& r, std::string_view key) {
  for (std::uint8_t i = 0; i < r.value_count; ++i) {
    if (r.values[i].key == key) return r.values[i].value;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

lio::LioConfig SimLioConfig() {
  const auto values = config::load_params_file(UAVNAV_LIO_DEFAULT_YAML, lio::kLioSpecs);
  EXPECT_TRUE(values.has_value());
  const auto cfg = lio::load_lio_config(*values);
  EXPECT_TRUE(cfg.has_value());
  return *cfg;
}

LioNodeConfig SimNodeConfig() {
  const auto values = config::load_params_file(UAVNAV_LIO_NODE_YAML, kLioNodeSpecs);
  EXPECT_TRUE(values.has_value());
  const auto cfg = load_lio_node_config(*values);
  EXPECT_TRUE(cfg.has_value());
  return *cfg;
}

time::TimeSnapshot Snap(time::RosTime ros) { return {time::SteadyTime{ros.ns}, ros}; }
/// ROS now equal to the sensor stamp (D29).
time::TimeSnapshot At(time::SensorTime t) { return Snap(time::RosTime{t.ns}); }
time::RosTime Ros(time::SensorTime t) { return time::RosTime{t.ns}; }

/// Polls `pred` until it holds; false after kGuard (never hangs).
template <class Pred>
bool Eventually(Pred pred) {
  const auto deadline = std::chrono::steady_clock::now() + kGuard;
  while (!pred()) {
    if (std::chrono::steady_clock::now() > deadline) return pred();
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  return true;
}

/// Runs `fn` on a std::async thread and requires it to return within kGuard (see the conventions above).
template <class F>
auto Bounded(F&& fn) -> decltype(fn()) {
  auto future = std::async(std::launch::async, std::forward<F>(fn));
  if (future.wait_for(kGuard) != std::future_status::ready) {
    ADD_FAILURE() << "a LioIngest call did not return within 10 s: aborting instead of hanging";
    std::abort();
  }
  return future.get();
}

// ---------------------------------------------------------------------------------------------------

class Rig {
 public:
  explicit Rig(const lio::LioConfig& cfg = SimLioConfig(), const LioNodeConfig& node = SimNodeConfig())
      : store_(std::make_shared<Store>()) {
    auto recorder = events::EventRecorder::create(std::make_unique<FakeSink>(store_));
    EXPECT_TRUE(recorder.has_value());
    recorder_ = std::move(*recorder);
    auto made = LioIngest::create(cfg, node, *recorder_, ros_);
    EXPECT_TRUE(made.has_value()) << (made ? "" : made.error().detail);
    if (made) ingest_ = std::move(*made);
  }
  ~Rig() { ingest_.reset(); }  // before the recorder and the shared ROS time it uses
  Rig(const Rig&) = delete;
  Rig& operator=(const Rig&) = delete;

  LioIngest& ingest() { return *ingest_; }
  void Destroy() { ingest_.reset(); }
  lio::FrontendStats Frontend() const { return ingest_->runner().frontend().stats(); }
  std::uint64_t ResultsPushed() const { return ingest_->runner().result_queue_stats().pushed; }

  std::vector<events::EventRecord> Events(std::string_view name) {
    recorder_->flush();
    const std::scoped_lock lock(store_->mutex);
    std::vector<events::EventRecord> out;
    for (const auto& r : store_->records) {
      if (r.event == name) out.push_back(r);
    }
    return out;
  }

  /// Stationary IMU samples k in [from, to) at ROS = sensor time; every call must be accepted.
  void Imu(std::int64_t from, std::int64_t to, std::vector<LioStateOut>* states = nullptr) {
    for (std::int64_t k = from; k < to; ++k) {
      const time::SensorTime t = room::imu_time(kT0, k);
      auto r = ingest_->on_imu(room::imu(t), At(t));
      ASSERT_TRUE(r.has_value()) << "IMU " << k << ": " << to_string(r.error());
      if (states) states->insert(states->end(), r->states.begin(), r->states.end());
    }
  }

  /// Feeds the stationary stream from k until the first state output (the backend's kImuInitialized has been
  /// applied: the predictor is aligned). Returns the next k, or -1 on timeout.
  std::int64_t FeedUntilAligned(std::int64_t k) {
    for (; k < 4000; ++k) {
      const time::SensorTime t = room::imu_time(kT0, k);
      auto r = ingest_->on_imu(room::imu(t), At(t));
      if (!r) return -1;
      if (!r->states.empty()) return k + 1;
      // Past the initialiser's sample count: let the backend answer before the next sample applies it.
      if (k >= 200 && !Eventually([&] { return ResultsPushed() >= 1; })) return -1;
    }
    return -1;
  }

 private:
  std::shared_ptr<Store> store_;
  std::unique_ptr<events::EventRecorder> recorder_;
  SharedRosTime ros_;
  std::unique_ptr<LioIngest> ingest_;
};

lio::StateOutput MakeState(time::SensorTime t, std::uint32_t reset_counter, const Eigen::Quaterniond& q,
                           const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
  lio::StateOutput s;
  s.sample = lio::OutputSample{t, q, v, p, reset_counter};
  s.epoch = reset_counter + 1;
  s.state = lio::LioState::kTracking;
  return s;
}

Eigen::Isometry3d Pose(const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() = p;
  return t;
}

}  // namespace

// ---------------------------------------------------------------------------------------------------

TEST(StateDecimator, OnePerTenMillisecondsOfSensorTime) {
  StateDecimator d(node_limits::kStatePeriod);
  // 200 Hz with a deterministic +-1 ms jitter for 10 s: one due sample per 10 ms cell.
  int due = 0;
  std::optional<std::int64_t> prev_cell;
  time::SensorTime t{};
  for (int k = 0; k < 2000; ++k) {
    const std::int64_t jitter_ns = ((k * 7919) % 3 - 1) * 1'000'000;  // -1, 0 or +1 ms
    t = time::SensorTime{kT0.ns + k * room::kImuPeriod.ns + jitter_ns};
    const std::int64_t cell = t.ns / node_limits::kStatePeriod.ns;
    const bool expected = !prev_cell || cell > *prev_cell;
    ASSERT_EQ(d.due(t), expected) << "sample " << k;
    if (expected) ++due;
    prev_cell = cell;
  }
  EXPECT_NEAR(due, 1000, 1);

  // A 0.37 s jump keeps the phase: the cells are absolute (floor(t / period)), never relative to the last sample.
  const time::SensorTime after{t.ns + 370'000'000};
  for (int k = 0; k < 200; ++k) {
    const time::SensorTime s{after.ns + k * room::kImuPeriod.ns + 3'000'000};  // 3 ms into a cell, then 8 ms
    const std::int64_t cell = s.ns / node_limits::kStatePeriod.ns;
    ASSERT_EQ(d.due(s), cell > *prev_cell) << "after the jump, sample " << k;
    prev_cell = std::max(*prev_cell, cell);
  }

  // Non-monotonic time: a sample in the current or an earlier cell is never due, and changes nothing.
  StateDecimator m(node_limits::kStatePeriod);
  EXPECT_TRUE(m.due(time::SensorTime{1'005'000'000}));
  EXPECT_FALSE(m.due(time::SensorTime{995'000'000}));    // earlier cell
  EXPECT_FALSE(m.due(time::SensorTime{1'000'000'000}));  // same cell, earlier stamp
  EXPECT_TRUE(m.due(time::SensorTime{1'010'000'000}));   // the next cell
}

TEST(LioIngest, ScanWaitsForImuCoverageThenIsPushed) {
  Rig rig;
  rig.Imu(0, 11);
  const time::SensorTime newest = room::imu_time(kT0, 10);
  const time::SensorTime scan_end = newest + time::milliseconds(5);
  const Eigen::Isometry3d imu_T_lidar = lio_node::imu_T_lidar(SimLioConfig());

  auto held = Bounded([&] { return rig.ingest().on_scan(room::scan(scan_end, imu_T_lidar), At(newest)); });
  ASSERT_TRUE(held.has_value()) << "an uncovered scan waits, it is not refused: " << to_string(held.error());
  EXPECT_EQ(rig.Frontend().scans_admitted, 0U) << "the core never sees a scan before its IMU";
  EXPECT_EQ(rig.ingest().stats().scans_waited, 1U);

  auto covered = Bounded([&] { return rig.ingest().on_imu(room::imu(scan_end), At(scan_end)); });
  ASSERT_TRUE(covered.has_value());
  EXPECT_EQ(rig.Frontend().scans_admitted, 1U) << "the covering IMU sample pushes the waiting scan";
  EXPECT_EQ(rig.ingest().stats().scans_dropped_timeout, 0U);
  EXPECT_EQ(rig.Events("LioScanDropped").size(), 0U);
  EXPECT_EQ(rig.Events("LioInputRejected").size(), 0U) << "no SCAN_AHEAD_OF_IMU refusal by the core";
}

// Review Focus 1.
TEST(LioIngest, ScanWaitingForImuIsDroppedAfterTheBoundedWait) {
  Rig rig;
  rig.Imu(0, 11);
  const time::SensorTime t = room::imu_time(kT0, 10);  // the IMU stops here
  const time::RosTime r = Ros(t);                       // the scan arrives at ROS r
  const time::SensorTime scan_end = t + time::milliseconds(2);
  const Eigen::Isometry3d imu_T_lidar = lio_node::imu_T_lidar(SimLioConfig());

  ASSERT_TRUE(Bounded([&] { return rig.ingest().on_scan(room::scan(scan_end, imu_T_lidar), Snap(r)); }));
  // r + 99 ms: an IMU sample that does not cover the scan; it keeps waiting.
  ASSERT_TRUE(Bounded([&] {
    return rig.ingest().on_imu(room::imu(t + time::milliseconds(1)), Snap(r + time::milliseconds(99)));
  }));
  EXPECT_EQ(rig.ingest().stats().scans_dropped_timeout, 0U);
  // r + 100 ms exactly: still waiting (expiry is strict, limits.hpp).
  ASSERT_TRUE(Bounded([&] {
    return rig.ingest().on_imu(room::imu(t + time::Duration{1'500'000}), Snap(r + time::milliseconds(100)));
  }));
  EXPECT_EQ(rig.ingest().stats().scans_dropped_timeout, 0U);
  EXPECT_EQ(rig.Events("LioScanDropped").size(), 0U);

  // r + 101 ms: this sample WOULD cover the scan, but the expiry is checked first: dropped, never pushed.
  const auto late = Bounded([&] {
    return rig.ingest().on_imu(room::imu(t + time::milliseconds(5)), Snap(r + time::milliseconds(101)));
  });
  ASSERT_TRUE(late.has_value()) << "the IMU sample itself is accepted";
  EXPECT_EQ(rig.ingest().stats().scans_dropped_timeout, 1U);
  EXPECT_EQ(rig.Frontend().scans_admitted, 0U) << "the expired scan never reached the core";
  const auto dropped = rig.Events("LioScanDropped");
  ASSERT_EQ(dropped.size(), 1U);
  EXPECT_EQ(dropped[0].reason, "IMU_COVERAGE_TIMEOUT");
  EXPECT_EQ(dropped[0].component, events::Component::kLio);
  EXPECT_DOUBLE_EQ(Value(dropped[0], "scan_end_s"), time::to_seconds(scan_end));
  EXPECT_NEAR(Value(dropped[0], "waited_s"), 0.101, 1e-9);

  // Nothing is waiting any more: a later covering sample pushes nothing.
  ASSERT_TRUE(Bounded([&] {
    return rig.ingest().on_imu(room::imu(t + time::milliseconds(10)), Snap(r + time::milliseconds(105)));
  }));
  EXPECT_EQ(rig.Frontend().scans_admitted, 0U);
}

TEST(LioIngest, NewerScanSupersedesTheWaitingOne) {
  Rig rig;
  rig.Imu(0, 11);
  const time::SensorTime t = room::imu_time(kT0, 10);
  const Eigen::Isometry3d imu_T_lidar = lio_node::imu_T_lidar(SimLioConfig());
  const time::SensorTime first_end = t + time::milliseconds(3);
  const time::SensorTime second_end = t + time::milliseconds(4);

  ASSERT_TRUE(Bounded([&] { return rig.ingest().on_scan(room::scan(first_end, imu_T_lidar), At(t)); }));
  ASSERT_TRUE(Bounded([&] {
    return rig.ingest().on_scan(room::scan(second_end, imu_T_lidar), At(t + time::milliseconds(1)));
  }));
  EXPECT_EQ(rig.ingest().stats().scans_superseded, 1U);
  EXPECT_EQ(rig.ingest().stats().scans_waited, 2U);
  const auto dropped = rig.Events("LioScanDropped");
  ASSERT_EQ(dropped.size(), 1U);
  EXPECT_EQ(dropped[0].reason, "SUPERSEDED");
  EXPECT_DOUBLE_EQ(Value(dropped[0], "scan_end_s"), time::to_seconds(first_end));
  EXPECT_NEAR(Value(dropped[0], "waited_s"), 0.001, 1e-9);
  EXPECT_EQ(rig.Frontend().scans_admitted, 0U);

  const time::SensorTime cover = t + time::milliseconds(5);
  ASSERT_TRUE(Bounded([&] { return rig.ingest().on_imu(room::imu(cover), At(cover)); }));
  EXPECT_EQ(rig.Frontend().scans_admitted, 1U) << "only the newer scan is pushed";
  // The backend answers the one admitted scan (before IMU initialisation: not processed) with its end time.
  ASSERT_TRUE(Eventually([&] { return !rig.Events("LioScan").empty(); }));
  const auto scans = rig.Events("LioScan");
  ASSERT_EQ(scans.size(), 1U);
  EXPECT_DOUBLE_EQ(Value(scans[0], "sensor_time_s"), time::to_seconds(second_end));
}

TEST(LioIngest, FutureStampIsRefused) {
  Rig rig;
  rig.Imu(0, 11);
  const time::SensorTime t = room::imu_time(kT0, 10);
  const time::TimeSnapshot now = At(t);
  const std::uint64_t accepted = rig.Frontend().imu_accepted;
  const Eigen::Isometry3d imu_T_lidar = lio_node::imu_T_lidar(SimLioConfig());

  const time::SensorTime ahead = t + time::milliseconds(600);
  const auto imu = Bounded([&] { return rig.ingest().on_imu(room::imu(ahead), now); });
  ASSERT_FALSE(imu.has_value());
  EXPECT_EQ(imu.error(), IngestReason::kStampInFuture);
  EXPECT_EQ(rig.Frontend().imu_accepted, accepted) << "nothing pushed";

  const auto scan = Bounded([&] { return rig.ingest().on_scan(room::scan(ahead, imu_T_lidar), now); });
  ASSERT_FALSE(scan.has_value());
  EXPECT_EQ(scan.error(), IngestReason::kStampInFuture);
  EXPECT_EQ(rig.ingest().stats().scans_waited, 0U) << "a refused scan does not wait";
  EXPECT_EQ(rig.Frontend().scans_admitted, 0U);

  const auto rejected = rig.Events("LioInputRejected");
  ASSERT_EQ(rejected.size(), 1U) << "rate-limited: the 1st refusal is reported, the 2nd is counted";
  EXPECT_EQ(rejected[0].reason, "STAMP_IN_FUTURE");
  EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), time::to_seconds(ahead));
  EXPECT_DOUBLE_EQ(Value(rejected[0], "count"), 1.0);

  // The bound itself (now + 0.5 s) is not in the future.
  const time::SensorTime edge = t + node_limits::kMaxStampAhead;
  const auto at_edge = Bounded([&] { return rig.ingest().on_imu(room::imu(edge), now); });
  ASSERT_TRUE(at_edge.has_value());
  EXPECT_EQ(rig.Frontend().imu_accepted, accepted + 1);
}

// Review Focus 4.
TEST(LioIngest, ConcurrentCallsAreRefusedNotRaced) {
  Rig rig;
  struct Counts {
    std::uint64_t ok{0}, reentered{0}, runner_rejected{0}, other{0};
  };
  constexpr int kCalls = 20'000;
  std::atomic<int> ready{0};
  const auto worker = [&](std::int64_t parity) {
    Counts c;
    ready.fetch_add(1);
    while (ready.load() < 2) std::this_thread::yield();  // start together
    for (std::int64_t i = 0; i < kCalls; ++i) {
      const time::SensorTime t = room::imu_time(kT0, 2 * i + parity);  // interleaved stamps
      const auto r = rig.ingest().on_imu(room::imu(t), At(t));
      if (r) {
        ++c.ok;
      } else if (r.error() == IngestReason::kReentered) {
        ++c.reentered;
      } else if (r.error() == IngestReason::kRunnerRejected) {
        ++c.runner_rejected;  // out of order: the other thread pushed a later stamp first
      } else {
        ++c.other;
      }
    }
    return c;
  };
  auto a = std::async(std::launch::async, worker, 0);
  auto b = std::async(std::launch::async, worker, 1);
  ASSERT_EQ(a.wait_for(kGuard), std::future_status::ready);
  ASSERT_EQ(b.wait_for(kGuard), std::future_status::ready);
  const Counts ca = a.get();
  const Counts cb = b.get();

  RecordProperty("ok", std::to_string(ca.ok + cb.ok));
  RecordProperty("reentered", std::to_string(ca.reentered + cb.reentered));
  RecordProperty("runner_rejected", std::to_string(ca.runner_rejected + cb.runner_rejected));
  EXPECT_EQ(ca.other + cb.other, 0U) << "every result is Ok, kReentered or kRunnerRejected";
  EXPECT_EQ(ca.ok + ca.reentered + ca.runner_rejected, static_cast<std::uint64_t>(kCalls));
  EXPECT_EQ(cb.ok + cb.reentered + cb.runner_rejected, static_cast<std::uint64_t>(kCalls));
  EXPECT_EQ(ca.ok + cb.ok, rig.Frontend().imu_accepted) << "every Ok is one sample the core accepted";
  EXPECT_EQ(ca.reentered + cb.reentered, rig.ingest().stats().reentered);
  const auto reentered = rig.Events("LioIngestReentered");
  if (ca.reentered + cb.reentered > 0) {
    ASSERT_FALSE(reentered.empty()) << "the first overlap is reported";
    EXPECT_EQ(reentered[0].reason, "REENTERED");
    EXPECT_LE(reentered.size(), 1 + (ca.reentered + cb.reentered) / lio::limits::kEventSummaryEvery);
  }
}

TEST(LioIngest, StateIsBaseLinkPoseAndBodyVelocity) {
  // IMU 0.25 m forward of base_link, yawed 0.3 rad against it.
  Eigen::Isometry3d base_T_imu = Eigen::Isometry3d::Identity();
  base_T_imu.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  base_T_imu.translation() = Eigen::Vector3d(0.25, 0.0, 0.0);
  const Eigen::Vector3d gyro(0.0, 0.0, 1.0);  // yaw rate 1 rad/s
  // Lever arm r: from the IMU origin to the base origin, in base_link. A body turning at w about the (fixed) IMU
  // moves its base origin at w x r: here (0, 0, 1) x (-0.25, 0, 0) = (0, -0.25, 0).
  const Eigen::Vector3d r = -base_T_imu.translation();
  const Eigen::Vector3d w_base = base_T_imu.linear() * gyro;
  const Eigen::Vector3d w_cross_r = w_base.cross(r);
  ASSERT_TRUE(w_cross_r.isApprox(Eigen::Vector3d(0.0, -0.25, 0.0), 1e-12));

  // (1) The mapping against an independent calculation: pose by composing isometries, velocity by a central
  // finite difference of the base origin's world position while the IMU moves at v and turns at gyro.
  {
    OutputCollector collector(base_T_imu, node_limits::kStatePeriod);
    const Eigen::Quaterniond q_world_imu(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
                                         Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
    const Eigen::Vector3d p_world_imu(1.0, 2.0, 3.0);
    for (const Eigen::Vector3d& v_world_imu : {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.4, -1.2, 0.3)}) {
      const LioStateOut out =
          collector.map(MakeState(kT0, 0, q_world_imu, p_world_imu, v_world_imu), gyro);
      const Eigen::Isometry3d world_T_base = Pose(q_world_imu, p_world_imu) * base_T_imu.inverse();
      EXPECT_LT((out.p_world_base - world_T_base.translation()).norm(), 1e-12);
      EXPECT_TRUE(out.q_world_base.toRotationMatrix().isApprox(world_T_base.linear(), 1e-12));
      EXPECT_LT((out.w_base_rad_s - w_base).norm(), 1e-12);

      const double h = 1e-6;
      const auto base_at = [&](double dt) -> Eigen::Vector3d {  // a value: translation() of a temporary dangles
        const Eigen::Quaterniond q = q_world_imu * Eigen::Quaterniond(Eigen::AngleAxisd(dt, gyro.normalized()));
        return (Pose(q, p_world_imu + v_world_imu * dt) * base_T_imu.inverse()).translation();
      };
      const Eigen::Vector3d v_world_base = (base_at(h) - base_at(-h)) / (2.0 * h);
      const Eigen::Vector3d v_base_expected = world_T_base.linear().transpose() * v_world_base;
      EXPECT_LT((out.v_base_mps - v_base_expected).norm(), 1e-6) << out.v_base_mps.transpose() << " vs "
                                                                  << v_base_expected.transpose();
      if (v_world_imu.isZero()) {
        EXPECT_LT((out.v_base_mps - w_cross_r).norm(), 1e-6) << "IMU still, body turning: v_base == w x r";
      }
    }
  }

  // (2) End to end through LioIngest and the real LioRunner: a still IMU yawing at 1 rad/s.
  const lio::LioConfig cfg = SimLioConfig();
  LioNodeConfig node;
  node.base_T_lidar = base_T_imu * lio_node::imu_T_lidar(cfg);
  ASSERT_TRUE(compose_base_T_imu(node, cfg).isApprox(base_T_imu, 1e-12));
  Rig rig(cfg, node);
  std::int64_t k = rig.FeedUntilAligned(0);
  ASSERT_GT(k, 0) << "the predictor never aligned";
  int checked = 0;
  for (int i = 0; i < 40; ++i, ++k) {
    const time::SensorTime t = room::imu_time(kT0, k);
    const lio::ImuInput turning{t, gyro, Eigen::Vector3d(0.0, 0.0, room::kGravity)};
    const auto out = Bounded([&] { return rig.ingest().on_imu(turning, At(t)); });
    ASSERT_TRUE(out.has_value());
    for (const LioStateOut& s : out->states) {
      EXPECT_EQ(s.t, t);
      EXPECT_LT((s.w_base_rad_s - w_base).norm(), 1e-12) << "the raw gyro rotated into base_link";
      EXPECT_LT((s.v_base_mps - w_cross_r).norm(), 1e-6) << s.v_base_mps.transpose();
      ++checked;
    }
  }
  EXPECT_EQ(checked, 20) << "40 samples at 200 Hz = 20 states at 100 Hz";
}

TEST(LioIngest, ResetDeltaIsLatchedOnLaterStates) {
  // The ingest's own output path (OutputCollector), fed the way LioRunner answers a restart: StepOutputs::restart
  // set, and the states of that call and later ones in the new epoch (reset_counter + 1). The restart caller
  // itself is S3/S4 (D33/O25), so LioIngest has no restart entry yet.
  OutputCollector collector(Eigen::Isometry3d::Identity(), node_limits::kStatePeriod);
  const Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  const auto state = [&](int ms, std::uint32_t rc) {
    return MakeState(kT0 + time::milliseconds(ms), rc, q, zero, zero);
  };
  const auto zero_delta = [](const lio::ResetDelta& d) {
    return d.position_m.isZero() && d.velocity_mps.isZero() && d.yaw_rad == 0.0;
  };
  const lio::ResetDelta first{Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.1, 0.0, -0.2), 0.5};
  const lio::ResetDelta second{Eigen::Vector3d(-4.0, 0.0, 0.0), Eigen::Vector3d::Zero(), -1.0};

  IngestOutputs out;
  lio::StepOutputs before;
  before.states = {state(10, 0)};
  collector.add(std::move(before), zero, out);
  ASSERT_EQ(out.states.size(), 1U);
  EXPECT_TRUE(zero_delta(out.states[0].last_reset)) << "zero before the first reset";
  EXPECT_EQ(out.states[0].reset_counter, 0U);

  lio::StepOutputs answer;
  answer.restart = first;
  answer.states = {state(20, 1)};
  collector.add(std::move(answer), zero, out);
  lio::StepOutputs later;
  later.states = {state(30, 1), state(40, 1)};
  collector.add(std::move(later), zero, out);
  lio::StepOutputs refused;  // a refused answer (stale, D33/O16) changes nothing
  refused.restart = std::unexpected(lio::EstimatorReason::kStaleAnswer);
  refused.states = {state(50, 1)};
  collector.add(std::move(refused), zero, out);
  ASSERT_EQ(out.states.size(), 5U);
  for (std::size_t i = 1; i < 5; ++i) {
    EXPECT_EQ(out.states[i].reset_counter, 1U);
    EXPECT_TRUE(out.states[i].last_reset.position_m.isApprox(first.position_m)) << i;
    EXPECT_TRUE(out.states[i].last_reset.velocity_mps.isApprox(first.velocity_mps)) << i;
    EXPECT_EQ(out.states[i].last_reset.yaw_rad, first.yaw_rad) << i;
  }

  lio::StepOutputs again;
  again.restart = second;
  again.states = {state(60, 2)};
  collector.add(std::move(again), zero, out);
  ASSERT_EQ(out.states.size(), 6U);
  EXPECT_EQ(out.states[5].reset_counter, 2U);
  EXPECT_TRUE(out.states[5].last_reset.position_m.isApprox(second.position_m));
  EXPECT_EQ(out.states[5].last_reset.yaw_rad, second.yaw_rad);
}

TEST(OutputCollector, OdometryAndHealthLastWinsAndOverwritesAreCounted) {
  OutputCollector collector(Eigen::Isometry3d::Identity(), node_limits::kStatePeriod);
  IngestOutputs out;
  for (std::uint32_t i = 0; i < 3; ++i) {
    lio::StepOutputs step;
    const Eigen::Vector3d z = Eigen::Vector3d::Zero();
    step.odometry = lio::OdometryOutput{kT0, i, 0, z, Eigen::Quaterniond::Identity(), z,
                                        Eigen::Matrix<double, 6, 6>::Zero(), Eigen::Matrix3d::Zero(), 0};
    step.health = lio::HealthOutput{kT0, lio::LioState::kTracking, 10 + i, lio::LioReason::kNone, 0.0, 0.0, 0.0, z};
    collector.add(std::move(step), Eigen::Vector3d::Zero(), out);
  }
  ASSERT_TRUE(out.odometry.has_value());
  EXPECT_EQ(out.odometry->epoch, 2U);
  ASSERT_TRUE(out.health.has_value());
  EXPECT_EQ(out.health->epoch, 12U);
  EXPECT_EQ(collector.odometry_overwritten(), 2U);
}

TEST(LioIngest, ReachesTrackingOnASyntheticRoom) {
  Rig rig;
  const Eigen::Isometry3d imu_T_lidar = lio_node::imu_T_lidar(SimLioConfig());
  std::int64_t k = rig.FeedUntilAligned(0);
  ASSERT_GT(k, 0) << "the predictor never aligned";

  std::vector<LioStateOut> states;
  int odometry = 0;
  bool tracking = false;
  int periods_after_tracking = 0;
  const auto keep = [&](const IngestOutputs& o) {
    states.insert(states.end(), o.states.begin(), o.states.end());
    if (o.odometry) ++odometry;
    if (o.health && o.health->state == lio::LioState::kTracking) tracking = true;
  };
  for (int period = 0; period < 60 && periods_after_tracking < 10; ++period) {
    for (int i = 0; i < 20; ++i, ++k) {  // one scan period of IMU
      const time::SensorTime t = room::imu_time(kT0, k);
      const auto r = Bounded([&] { return rig.ingest().on_imu(room::imu(t), At(t)); });
      ASSERT_TRUE(r.has_value()) << to_string(r.error());
      keep(*r);
    }
    // Paced: the scan ends at the newest IMU sample (covered: pushed at once); wait for the backend's answer,
    // which the next IMU call applies.
    const time::SensorTime end = room::imu_time(kT0, k - 1);
    const std::uint64_t answered = rig.ResultsPushed();
    const auto s = Bounded([&] { return rig.ingest().on_scan(room::scan(end, imu_T_lidar), At(end)); });
    ASSERT_TRUE(s.has_value()) << to_string(s.error());
    keep(*s);
    ASSERT_TRUE(Eventually([&] { return rig.ResultsPushed() > answered; })) << "no backend answer within 10 s";
    if (tracking) ++periods_after_tracking;
  }
  EXPECT_TRUE(tracking) << "TRACKING never reached";
  EXPECT_GE(odometry, 1);
  EXPECT_EQ(rig.ingest().stats().scans_waited, 0U);
  ASSERT_GE(states.size(), 2U);
  // 100 Hz of sensor time: one state per 10 ms over the whole span, +-1 %.
  const double span_cells = time::to_seconds(states.back().t - states.front().t) / 0.01 + 1.0;
  EXPECT_NEAR(static_cast<double>(states.size()), span_cells, 0.01 * span_cells);
  for (std::size_t i = 1; i < states.size(); ++i) {
    ASSERT_GT(states[i].t.ns / node_limits::kStatePeriod.ns, states[i - 1].t.ns / node_limits::kStatePeriod.ns);
  }
  for (const LioStateOut& s : states) {
    ASSERT_TRUE(s.p_world_base.allFinite() && s.v_base_mps.allFinite() && s.q_world_base.coeffs().allFinite());
    EXPECT_EQ(s.reset_counter, 0U);
    EXPECT_TRUE(s.last_reset.position_m.isZero());
  }
}

TEST(LioIngest, DestroyWithQueuedWorkReturns) {
  const Eigen::Isometry3d imu_T_lidar = lio_node::imu_T_lidar(SimLioConfig());
  {
    Rig rig;
    // 300 IMU samples and 2 scans as fast as possible, not paced: the backend still has work queued.
    rig.Imu(0, 151);
    const time::SensorTime first = room::imu_time(kT0, 150);
    ASSERT_TRUE(rig.ingest().on_scan(room::scan(first, imu_T_lidar), At(first)).has_value());
    rig.Imu(151, 300);
    const time::SensorTime second = room::imu_time(kT0, 299);
    ASSERT_TRUE(rig.ingest().on_scan(room::scan(second, imu_T_lidar), At(second)).has_value());

    Bounded([&] {
      rig.Destroy();
      return 0;
    });
    const auto summary = rig.Events("IngestSummary");
    ASSERT_EQ(summary.size(), 1U) << "the destructor stops the runner and reports once";
    EXPECT_EQ(summary[0].reason, "ACCEPTED");
    EXPECT_DOUBLE_EQ(Value(summary[0], "scans_waited"), 0.0);
    EXPECT_DOUBLE_EQ(Value(summary[0], "reentered"), 0.0);
    EXPECT_DOUBLE_EQ(Value(summary[0], "scans_admitted"), 2.0);
    EXPECT_GE(Value(summary[0], "request_pushed"), 300.0);
    EXPECT_TRUE(std::isfinite(Value(summary[0], "request_popped")));
    EXPECT_TRUE(std::isfinite(Value(summary[0], "result_pushed")));
    EXPECT_TRUE(std::isfinite(Value(summary[0], "scans_refused_busy")));
  }
  {
    // stop() is idempotent and the destructor after it reports nothing more.
    Rig rig;
    rig.Imu(0, 10);
    Bounded([&] {
      rig.ingest().stop();
      rig.ingest().stop();
      return 0;
    });
    // After stop() the ingest keeps working on the frontend alone (LioRunner).
    rig.Imu(10, 20);
    Bounded([&] {
      rig.Destroy();
      return 0;
    });
    EXPECT_EQ(rig.Events("IngestSummary").size(), 1U);
  }
}
