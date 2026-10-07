#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <sstream>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "synthetic_scene.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/lio/backend.hpp"
#include "uavnav/lio/channels.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/estimator.hpp"  // transitional: OdometryMatchesTheS1aFacade (deleted in Task 6)
#include "uavnav/lio/types.hpp"

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

constexpr time::Duration kImuPeriod = time::milliseconds(5);     // 200 Hz
constexpr time::Duration kScanPeriod = time::milliseconds(100);  // 10 Hz
constexpr time::SensorTime kT0{1'000'000'000};                   // sensor time must be > 0
constexpr std::size_t kImuInitSamples = 200;                     // imu_init_min_samples of config/lio/sim.yaml

time::TimeSnapshot Snap(time::SensorTime t) {
  return {time::SteadyTime{3 * t.ns}, time::RosTime{t.ns + 7'000'000'000}};
}

enum class ScanKind { kRoom, kPlane, kEmpty, kHalfRoom };

ScanInput MakeScan(ScanKind kind, time::SensorTime start, time::SensorTime end) {
  const scene::Pose pose{};  // stationary
  switch (kind) {
    case ScanKind::kRoom:
      return scene::make_scan(pose, start, end);
    case ScanKind::kPlane:
      return scene::make_plane_only_scan(pose, start, end);
    case ScanKind::kEmpty:
      return ScanInput{start, end, {}, false};
    case ScanKind::kHalfRoom: {  // every second ray of the room scan: hits fewer map voxels
      ScanInput full = scene::make_scan(pose, start, end);
      ScanInput half{start, end, {}, false};
      for (std::size_t i = 0; i < full.points.size(); i += 2) half.points.push_back(full.points[i]);
      return half;
    }
  }
  return {};
}

std::size_t Count(const std::vector<BackendResult>& results, BackendResultKind kind) {
  std::size_t n = 0;
  for (const auto& r : results) n += r.kind == kind ? 1U : 0U;
  return n;
}

// Drives one LioBackend through its channels, synchronously: every request is pushed, then step() is pumped
// until the request queue is empty, draining the result queue after each step.
class BackendHarness {
 public:
  BackendHarness() : store_(std::make_shared<Store>()), channels_(std::make_unique<LioChannels>()) {
    auto recorder = events::EventRecorder::create(std::make_unique<FakeSink>(store_));
    EXPECT_TRUE(recorder.has_value());
    recorder_ = std::move(*recorder);
    auto backend =
        LioBackend::create(DefaultConfig(), scene::base_T_imu(), scene::imu_T_lidar(), *channels_, *recorder_);
    EXPECT_TRUE(backend.has_value()) << (backend ? "" : backend.error().detail);
    backend_ = std::move(*backend);
  }

  LioBackend& backend() { return *backend_; }
  LioChannels& channels() { return *channels_; }
  time::SensorTime newest_imu() const { return next_imu_ - kImuPeriod; }
  time::SensorTime scan_end() const { return scan_end_; }
  std::uint32_t epoch() const { return epoch_; }

  /// Pushes one request and processes it; returns the results it produced.
  std::vector<BackendResult> Push(BackendRequest&& request, time::SensorTime t) {
    EXPECT_TRUE(channels_->requests.try_push(std::move(request)).has_value());
    std::vector<BackendResult> out;
    while (backend_->step(Snap(t))) {
      while (auto r = channels_->results.try_pop()) out.push_back(std::move(*r));
    }
    for (const auto& r : out) results_.push_back(r);
    return out;
  }

  /// IMU samples up to and including `end`.
  void ImuUntil(time::SensorTime end) {
    while (next_imu_ <= end) {
      const ImuInput imu = scene::make_imu({}, next_imu_);
      imu_stamps_.push_back(imu.t);
      Push(imu, imu.t);
      next_imu_ = next_imu_ + kImuPeriod;
    }
  }

  /// One scan period: IMU up to the scan end, then one scan; returns that scan's result.
  BackendResult Scan(ScanKind kind, MapPolicy policy = MapPolicy::kInsert) {
    const time::SensorTime end = scan_end_ + kScanPeriod;
    ImuUntil(end);
    return ScanAt(kind, scan_end_, end, policy, epoch_);
  }

  BackendResult ScanAt(ScanKind kind, time::SensorTime start, time::SensorTime end, MapPolicy policy,
                       std::uint32_t epoch) {
    const std::uint64_t seq = ++seq_;
    const auto out = Push(ScanJob{seq, epoch, policy, MakeScan(kind, start, end)}, end);
    scan_end_ = end;
    EXPECT_EQ(out.size(), 1U);
    if (out.empty()) return BackendResult{};
    EXPECT_EQ(out.front().seq, seq);
    return out.front();
  }

  /// IMU initialisation (the 200th sample, kT0 + 995 ms); the next scan starts at the initialising sample.
  void InitialiseImu() {
    ImuUntil(kT0 + time::Duration{static_cast<std::int64_t>(kImuInitSamples - 1) * kImuPeriod.ns});
    ASSERT_EQ(Count(results_, BackendResultKind::kImuInitialized), 1U);
    scan_end_ = newest_imu();
  }

  /// IMU initialisation and the map bootstrap scan: the next room scan is corrected.
  void InitialiseAndBootstrap() {
    InitialiseImu();
    const BackendResult r = Scan(ScanKind::kRoom);
    ASSERT_EQ(r.kind, BackendResultKind::kMapReady);
  }

  BackendResult Restart(std::uint32_t new_epoch, const SeedPose& seed, const Eigen::Quaterniond& q_old,
                        const Eigen::Vector3d& v_old) {
    const time::SensorTime t = newest_imu();
    const auto out = Push(RestartCommand{new_epoch, t, seed, q_old, v_old}, t);
    EXPECT_EQ(out.size(), 1U);
    if (out.empty()) return BackendResult{};
    if (out.front().kind == BackendResultKind::kRestartSeeded) epoch_ = new_epoch;
    return out.front();
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

  const std::vector<BackendResult>& results() const { return results_; }
  const std::vector<time::SensorTime>& imu_stamps() const { return imu_stamps_; }

 private:
  // Declaration order = destruction in reverse: the backend goes first, then the channels and the recorder
  // it refers to.
  std::shared_ptr<Store> store_;
  std::unique_ptr<events::EventRecorder> recorder_;
  std::unique_ptr<LioChannels> channels_;  // heap: the rings hold their slots inline (channels.hpp)
  std::unique_ptr<LioBackend> backend_;
  time::SensorTime next_imu_{kT0};
  time::SensorTime scan_end_{kT0};
  std::uint64_t seq_{0};
  std::uint32_t epoch_{1};
  std::vector<BackendResult> results_;
  std::vector<time::SensorTime> imu_stamps_;
};

std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  EXPECT_TRUE(in.good()) << path;
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

template <class T>
concept HasStateMember = requires(const T& r) { r.state; };
template <class T>
concept HasLifecycleStateMember = requires(const T& r) { r.lifecycle_state; };

}  // namespace

// ---------------------------------------------------------------------------------------------------

TEST(LioBackend, ImuCopiesInitialiseAndReportOnce) {
  BackendHarness h;
  h.ImuUntil(kT0 + time::Duration{399 * kImuPeriod.ns});  // 400 stationary samples at 200 Hz
  ASSERT_EQ(h.imu_stamps().size(), 400U);
  ASSERT_EQ(Count(h.results(), BackendResultKind::kImuInitialized), 1U);
  ASSERT_EQ(h.results().size(), 1U);
  const BackendResult& r = h.results().front();
  EXPECT_EQ(r.epoch, 1U);
  EXPECT_EQ(r.seq, 0U);
  EXPECT_GE(r.t, h.imu_stamps()[kImuInitSamples - 1]);  // at or after the 200th sample
  EXPECT_NE(std::find(h.imu_stamps().begin(), h.imu_stamps().end(), r.t), h.imu_stamps().end());
  ASSERT_TRUE(r.snapshot.has_value());
  EXPECT_EQ(r.snapshot->t, r.t);
  EXPECT_NEAR(r.snapshot->gravity_world.norm(), 9.81, 0.05);
  EXPECT_FALSE(r.report.has_value());
  EXPECT_FALSE(r.odometry.has_value());
  EXPECT_TRUE(std::isfinite(r.position_sigma_m));
  EXPECT_EQ(h.Events("ImuInitialized").size(), 1U);
  EXPECT_EQ(h.backend().stats().imu_samples, 400U);
}

TEST(LioBackend, FirstScanOfAnEpochIsMapReady) {
  BackendHarness h;
  for (int i = 0; i < 9; ++i) {  // the IMU initialises at kT0 + 995 ms: scans ending before are not processed
    EXPECT_EQ(h.Scan(ScanKind::kRoom).kind, BackendResultKind::kScanNotProcessed) << i;
  }
  EXPECT_EQ(h.backend().stats().map_points, 0U);
  const BackendResult r = h.Scan(ScanKind::kRoom);  // ends at kT0 + 1 s, after the ESKF time
  EXPECT_EQ(r.kind, BackendResultKind::kMapReady);
  EXPECT_EQ(r.seq, 10U);
  EXPECT_EQ(r.epoch, 1U);
  EXPECT_EQ(r.t, h.scan_end());
  EXPECT_FALSE(r.odometry.has_value());
  EXPECT_GT(h.backend().stats().map_points, 0U);
  EXPECT_EQ(h.backend().stats().scans, 10U);
  EXPECT_EQ(h.Events("LioScan").back().reason, "MAP_BOOTSTRAP");
}

TEST(LioBackend, StaticRoomScansAreGoodWithOdometry) {
  BackendHarness h;
  h.InitialiseAndBootstrap();
  for (int i = 0; i < 5; ++i) {
    const BackendResult r = h.Scan(ScanKind::kRoom);
    ASSERT_EQ(r.kind, BackendResultKind::kScanGood) << i;
    ASSERT_TRUE(r.report.has_value());
    EXPECT_FALSE(r.report->degenerate);
    EXPECT_GT(r.report->quality, 50U);
    ASSERT_TRUE(r.snapshot.has_value());
    EXPECT_EQ(r.snapshot->t, h.scan_end());
    EXPECT_EQ(r.t, h.scan_end());
    ASSERT_TRUE(r.odometry.has_value());
    const BaseOdometry& o = *r.odometry;
    EXPECT_TRUE(o.p_world_m.allFinite() && o.q_world_base.coeffs().allFinite() && o.v_world_mps.allFinite());
    EXPECT_TRUE(o.pose_cov.allFinite() && o.vel_cov.allFinite());
    EXPECT_GT(o.pose_cov.diagonal().minCoeff(), 0.0);
    // World = the IMU frame at initialisation (level IMU, yaw 0): base_link sits at -t(base_T_imu).
    EXPECT_LT((o.p_world_m + scene::base_T_imu().translation()).norm(), 0.05);
    EXPECT_TRUE(std::isfinite(r.position_sigma_m));
  }
  const auto scans = h.Events("LioScan");
  ASSERT_FALSE(scans.empty());
  EXPECT_EQ(scans.back().reason, "SCAN_GOOD");
  EXPECT_TRUE(h.Events("OdometryInvalid").empty());
  EXPECT_TRUE(h.Events("EskfRebased").empty());
}

TEST(LioBackend, PlaneOnlyScanIsDegenerateWithoutOdometry) {
  BackendHarness h;
  h.InitialiseAndBootstrap();
  (void)h.Scan(ScanKind::kRoom);
  BackendResult r;
  EXPECT_NO_THROW(r = h.Scan(ScanKind::kPlane));
  EXPECT_EQ(r.kind, BackendResultKind::kScanDegenerate);
  ASSERT_TRUE(r.report.has_value());
  EXPECT_TRUE(r.report->degenerate);
  EXPECT_LT(r.report->quality, 50U);
  EXPECT_FALSE(r.odometry.has_value());
  EXPECT_EQ(h.Events("LioScan").back().reason, "SCAN_DEGENERATE");
}

TEST(LioBackend, EmptyScanIsScanEmpty) {
  BackendHarness h;
  h.InitialiseAndBootstrap();
  const BackendResult r = h.Scan(ScanKind::kEmpty);
  EXPECT_EQ(r.kind, BackendResultKind::kScanEmpty);
  EXPECT_FALSE(r.report.has_value());
  EXPECT_FALSE(r.odometry.has_value());
  EXPECT_EQ(h.Events("LioScan").back().reason, "SCAN_EMPTY");
}

TEST(LioBackend, ScanBeforeImuInitIsNotProcessed) {
  BackendHarness h;
  const BackendResult r = h.Scan(ScanKind::kRoom);
  EXPECT_EQ(r.kind, BackendResultKind::kScanNotProcessed);
  EXPECT_EQ(r.epoch, 1U);
  EXPECT_EQ(r.t, h.scan_end());
  EXPECT_FALSE(r.snapshot.has_value());
  EXPECT_FALSE(r.odometry.has_value());
  const auto scans = h.Events("LioScan");
  ASSERT_EQ(scans.size(), 1U);
  EXPECT_EQ(scans[0].reason, "BEFORE_IMU_INIT");
  EXPECT_EQ(h.backend().stats().map_points, 0U);
}

TEST(LioBackend, FrozenMapPolicyLeavesTheMapUnchanged) {
  BackendHarness h;
  // Bootstrap from every second ray, so the full room scans below reach map voxels the bootstrap did not.
  h.InitialiseImu();
  ASSERT_EQ(h.Scan(ScanKind::kHalfRoom).kind, BackendResultKind::kMapReady);
  const std::size_t bootstrap = h.backend().stats().map_points;
  ASSERT_GT(bootstrap, 0U);

  BackendResult r = h.Scan(ScanKind::kRoom, MapPolicy::kFrozen);
  ASSERT_EQ(r.kind, BackendResultKind::kScanGood);
  EXPECT_TRUE(r.odometry.has_value());
  EXPECT_EQ(h.backend().stats().map_points, bootstrap);

  r = h.Scan(ScanKind::kRoom, MapPolicy::kInsert);
  ASSERT_EQ(r.kind, BackendResultKind::kScanGood);
  EXPECT_GT(h.backend().stats().map_points, bootstrap);
}

TEST(LioBackend, RestartClearsMapAndSeedsAtCommandTime) {
  BackendHarness h;
  h.InitialiseAndBootstrap();
  const BackendResult good = h.Scan(ScanKind::kRoom);
  ASSERT_EQ(good.kind, BackendResultKind::kScanGood);
  ASSERT_GT(h.backend().stats().map_points, 0U);

  const SeedPose seed{Eigen::Vector3d(1.0, 2.0, 0.5),
                      Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()))};
  const time::SensorTime t = h.newest_imu();
  const BackendResult r = h.Restart(2, seed, good.snapshot->q_world_imu, good.snapshot->v_world_mps);
  ASSERT_EQ(r.kind, BackendResultKind::kRestartSeeded);
  EXPECT_EQ(r.epoch, 2U);
  EXPECT_EQ(r.seq, 0U);
  EXPECT_EQ(r.t, t);
  ASSERT_TRUE(r.snapshot.has_value());
  EXPECT_EQ(r.snapshot->t, t);
  const Eigen::Vector3d expected_imu = seed.p_world_m + seed.q_world_base * scene::base_T_imu().translation();
  EXPECT_LT((r.snapshot->p_world_m - expected_imu).norm(), 1e-9);
  EXPECT_LT(r.snapshot->q_world_imu.angularDistance(seed.q_world_base), 1e-9);  // base_T_imu has no rotation
  EXPECT_LT(r.snapshot->v_world_mps.norm(), 0.01);
  EXPECT_TRUE(std::isfinite(r.position_sigma_m));
  EXPECT_EQ(h.backend().stats().map_points, 0U);
  EXPECT_EQ(h.backend().stats().restarts, 1U);

  const BackendResult next = h.Scan(ScanKind::kRoom);
  EXPECT_EQ(next.kind, BackendResultKind::kMapReady);
  EXPECT_EQ(next.epoch, 2U);
  EXPECT_GT(h.backend().stats().map_points, 0U);
  const auto scans = h.Events("LioScan");
  EXPECT_EQ(scans.back().identity.lio_epoch, 2U);
}

TEST(LioBackend, RestartBeforeImuInitIsRejected) {
  BackendHarness h;
  h.ImuUntil(kT0 + time::milliseconds(100));  // not initialised yet
  const BackendResult r = h.Restart(2, SeedPose{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
                                    Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
  EXPECT_EQ(r.kind, BackendResultKind::kRestartRejected);
  EXPECT_EQ(r.epoch, 2U);  // the command's epoch, so the frontend matches it
  EXPECT_FALSE(r.snapshot.has_value());
  EXPECT_EQ(h.backend().stats().restarts, 0U);
  // Own epoch unchanged: a scan of epoch 1 is still the backend's epoch (not STALE_EPOCH).
  const BackendResult s = h.Scan(ScanKind::kRoom);
  EXPECT_EQ(s.kind, BackendResultKind::kScanNotProcessed);
  EXPECT_EQ(h.Events("LioScan").back().reason, "BEFORE_IMU_INIT");
}

TEST(LioBackend, ScansOfAnOlderEpochAreNotProcessed) {
  BackendHarness h;
  h.InitialiseAndBootstrap();
  const BackendResult good = h.Scan(ScanKind::kRoom);
  ASSERT_EQ(good.kind, BackendResultKind::kScanGood);
  ASSERT_EQ(h.Restart(2, SeedPose{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()},
                      good.snapshot->q_world_imu, good.snapshot->v_world_mps)
                .kind,
            BackendResultKind::kRestartSeeded);
  const time::SensorTime end = h.scan_end() + kScanPeriod;
  h.ImuUntil(end);
  const BackendResult r = h.ScanAt(ScanKind::kRoom, h.scan_end(), end, MapPolicy::kInsert, 1);
  EXPECT_EQ(r.kind, BackendResultKind::kScanNotProcessed);
  EXPECT_EQ(r.epoch, 1U);  // the ScanJob's epoch
  EXPECT_FALSE(r.snapshot.has_value());
  EXPECT_EQ(h.backend().stats().map_points, 0U);
  EXPECT_EQ(h.Events("LioScan").back().reason, "STALE_EPOCH");
}

// Transitional (deleted in Task 6 with LioEstimator): the extracted backend computes the same base_link
// odometry as the S1a facade on the same inputs.
TEST(LioBackend, OdometryMatchesTheS1aFacade) {
  auto store = std::make_shared<Store>();
  auto recorder = events::EventRecorder::create(std::make_unique<FakeSink>(store));
  ASSERT_TRUE(recorder.has_value());
  auto facade = LioEstimator::create(DefaultConfig(), scene::base_T_imu(), scene::imu_T_lidar(), **recorder);
  ASSERT_TRUE(facade.has_value());
  BackendHarness h;

  time::SensorTime next_imu = kT0;
  time::SensorTime scan_end = kT0;
  std::uint64_t seq = 0;
  int compared = 0;
  const auto scans = {std::pair{30, ScanKind::kRoom}, std::pair{2, ScanKind::kPlane}, std::pair{6, ScanKind::kRoom}};
  for (const auto& [n, kind] : scans) {
    for (int i = 0; i < n; ++i) {
      const time::SensorTime end = scan_end + kScanPeriod;
      for (; next_imu <= end; next_imu = next_imu + kImuPeriod) {
        const ImuInput imu = scene::make_imu({}, next_imu);
        ASSERT_TRUE((*facade)->push_imu(imu, Snap(imu.t)).has_value());
        (void)h.Push(imu, imu.t);
      }
      const auto f = (*facade)->push_scan(MakeScan(kind, scan_end, end), Snap(end));
      ASSERT_TRUE(f.has_value());
      const auto b = h.Push(ScanJob{++seq, 1, MapPolicy::kInsert, MakeScan(kind, scan_end, end)}, end);
      ASSERT_EQ(b.size(), 1U);
      scan_end = end;
      if (!f->odometry) continue;
      ASSERT_TRUE(b.front().odometry.has_value()) << seq;
      const BaseOdometry& o = *b.front().odometry;
      EXPECT_LT((o.p_world_m - f->odometry->p_world_m).norm(), 1e-12) << seq;
      EXPECT_LT(o.q_world_base.angularDistance(f->odometry->q_world_base), 1e-12) << seq;
      EXPECT_LT((o.pose_cov - f->odometry->pose_cov).cwiseAbs().maxCoeff(), 1e-12) << seq;
      ++compared;
    }
  }
  EXPECT_GE(compared, 15);
}

TEST(LioBackend, RunReturnsPromptlyOnStop) {
  BackendHarness h;
  std::atomic<int> clock_reads{0};
  const std::function<time::TimeSnapshot()> clock = [&clock_reads] {
    clock_reads.fetch_add(1);
    return Snap(kT0);
  };
  std::jthread thread([&](std::stop_token stop) { h.backend().run(stop, clock); });
  for (int i = 0; i < 50; ++i) {
    ASSERT_TRUE(h.channels().requests.try_push(scene::make_imu({}, kT0 + time::Duration{i * kImuPeriod.ns})).has_value());
  }
  auto stopped = std::async(std::launch::async, [&thread] {
    thread.request_stop();
    thread.join();
  });
  ASSERT_EQ(stopped.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  stopped.get();
  // run() read the clock once per processed request, never more than were queued.
  EXPECT_LE(clock_reads.load(), 50);
  EXPECT_EQ(static_cast<std::uint64_t>(clock_reads.load()), h.backend().stats().imu_samples);
}

// Shutdown rule (§3.5): stop returns after the request in progress and leaves the rest queued. The clock
// callable blocks inside the first step until the test has requested the stop.
TEST(LioBackend, RunReturnsAfterTheRequestInProgressOnStop) {
  BackendHarness h;
  for (int i = 0; i < 50; ++i) {
    ASSERT_TRUE(h.channels().requests.try_push(scene::make_imu({}, kT0 + time::Duration{i * kImuPeriod.ns})).has_value());
  }
  std::atomic<bool> entered{false};
  std::atomic<bool> release{false};
  const std::function<time::TimeSnapshot()> clock = [&] {
    entered.store(true);
    entered.notify_all();
    release.wait(false);
    return Snap(kT0);
  };
  struct Release {  // never leave the backend thread blocked in the clock, even when an assertion fails
    std::atomic<bool>& flag;
    ~Release() {
      flag.store(true);
      flag.notify_all();
    }
  };
  std::jthread thread([&](std::stop_token stop) { h.backend().run(stop, clock); });
  const Release guard{release};
  auto in_step = std::async(std::launch::async, [&entered] { entered.wait(false); });
  ASSERT_EQ(in_step.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  thread.request_stop();
  release.store(true);
  release.notify_all();
  auto stopped = std::async(std::launch::async, [&thread] { thread.join(); });
  ASSERT_EQ(stopped.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  EXPECT_EQ(h.backend().stats().imu_samples, 1U);
  EXPECT_EQ(h.channels().requests.stats().popped, 1U);
  EXPECT_EQ(h.channels().requests.stats().pushed, 50U);
}

TEST(LioBackend, RunReturnsWhenStoppedWhileIdle) {
  BackendHarness h;
  const std::function<time::TimeSnapshot()> clock = [] { return Snap(kT0); };
  std::jthread thread([&](std::stop_token stop) { h.backend().run(stop, clock); });
  auto stopped = std::async(std::launch::async, [&thread] {
    thread.request_stop();
    thread.join();
  });
  ASSERT_EQ(stopped.wait_for(std::chrono::seconds(10)), std::future_status::ready);
  EXPECT_EQ(h.backend().stats().imu_samples, 0U);
}

// Review Focus 3: the backend cannot name the lifecycle or the frontend's output types.
TEST(LioSplit, BackendSourcesNeverReferenceTheLifecycle) {
  const std::string dir = UAVNAV_LIO_CORE_SOURCE_DIR;
  for (const std::string file : {"/include/uavnav/lio/backend.hpp", "/src/backend.cpp", "/src/backend_math.hpp",
                                 "/src/backend_math.cpp", "/include/uavnav/lio/types.hpp",
                                 "/include/uavnav/lio/channels.hpp"}) {
    const std::string text = ReadFile(dir + file);
    ASSERT_FALSE(text.empty()) << file;
    for (const std::string_view token : {"LioLifecycle", "lifecycle.hpp", "LioState", "OutputPredictor",
                                         "output_predictor.hpp", "outputs.hpp"}) {
      EXPECT_EQ(text.find(token), std::string::npos) << file << " names " << token;
    }
  }
}

// Review Focus 3: a result carries no lifecycle state.
TEST(LioSplit, ResultsCarryNoLifecycleState) {
  static_assert(!HasStateMember<BackendResult>);
  static_assert(!HasLifecycleStateMember<BackendResult>);
  static_assert(std::is_nothrow_move_constructible_v<BackendRequest>);
  static_assert(std::is_nothrow_move_constructible_v<BackendResult>);
  SUCCEED();
}

TEST(LioBackend, ResultKindsHaveNames) {
  EXPECT_EQ(to_string(BackendResultKind::kScanGood), "SCAN_GOOD");
  EXPECT_EQ(to_string(BackendResultKind::kScanNotProcessed), "SCAN_NOT_PROCESSED");
  EXPECT_EQ(to_string(BackendResultKind::kRestartRejected), "RESTART_REJECTED");
  EXPECT_EQ(to_string(EstimatorReason::kBackendBusy), "BACKEND_BUSY");
  EXPECT_EQ(to_string(EstimatorEventReason::kStaleEpoch), "STALE_EPOCH");
  EXPECT_EQ(to_string(EstimatorEventReason::kResultQueueFull), "RESULT_QUEUE_FULL");
}
