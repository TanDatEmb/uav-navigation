#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "synthetic_scene.hpp"
#include "uavnav/core/config.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/lio/config.hpp"
#include "uavnav/lio/estimator.hpp"
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

constexpr time::Duration kImuPeriod = time::milliseconds(5);     // 200 Hz
constexpr time::Duration kScanPeriod = time::milliseconds(100);  // 10 Hz
constexpr time::SensorTime kT0{1'000'000'000};                   // sensor time must be > 0

// Beta degeneracy thresholds (config/lio/sim.yaml).
constexpr double kTranslationMinInfo = 1.1e5;
constexpr double kRotationMinInfo = 2.8e6;

constexpr time::Duration Times(time::Duration d, std::int64_t n) { return time::Duration{d.ns * n}; }

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
  if (out.odometry) {
    const OdometryOutput& o = *out.odometry;
    if (!o.p_world_m.allFinite() || !o.q_world_base.coeffs().allFinite() || !o.v_world_mps.allFinite() ||
        !o.pose_cov.allFinite() || !o.vel_cov.allFinite()) {
      return false;
    }
  }
  if (out.health) {
    const HealthOutput& h = *out.health;
    if (!std::isfinite(h.translation_min_eigenvalue) || !std::isfinite(h.rotation_min_eigenvalue) ||
        !std::isfinite(h.correction_age_s) || !h.output_tracking_error.allFinite()) {
      return false;
    }
  }
  return true;
}

enum class ScanKind { kRoom, kPlane, kCorridor, kEmpty };

// Drives one estimator with the synthetic scene. Every output is checked for NaN, and every call that
// changes the state is counted (at most one transition per call).
class Harness {
 public:
  explicit Harness(const LioConfig& cfg = DefaultConfig(), time::Duration imu_period = kImuPeriod)
      : store_(std::make_shared<Store>()), imu_period_(imu_period) {
    auto recorder = events::EventRecorder::create(std::make_unique<FakeSink>(store_));
    EXPECT_TRUE(recorder.has_value());
    recorder_ = std::move(*recorder);
    auto est = LioEstimator::create(cfg, scene::base_T_imu(), scene::imu_T_lidar(), *recorder_);
    EXPECT_TRUE(est.has_value()) << (est ? "" : est.error().detail);
    est_ = std::move(*est);
    traj_.t0 = kT0;
  }

  LioEstimator& est() { return *est_; }
  time::SensorTime now() const { return next_imu_ - imu_period_; }

  /// Event clocks deliberately on other time bases than sensor time (x3, +7 s), so a decision that used the
  /// snapshot instead of sensor time would show in the pinned-timing tests.
  static time::TimeSnapshot Snap(time::SensorTime t) {
    return {time::SteadyTime{3 * t.ns}, time::RosTime{t.ns + 7'000'000'000}};
  }

  Result<StepOutputs, EstimatorReason> Imu(const ImuInput& imu) {
    const LioState before = est_->state();
    auto r = est_->push_imu(imu, Snap(imu.t));
    Record(before, r);
    return r;
  }

  Result<StepOutputs, EstimatorReason> Scan(ScanInput&& scan) {
    const LioState before = est_->state();
    const time::SensorTime t = scan.end;
    auto r = est_->push_scan(std::move(scan), Snap(t));
    Record(before, r);
    return r;
  }

  Result<ResetDelta, EstimatorReason> Restart(const SeedPose& seed) {
    const LioState before = est_->state();
    auto r = est_->restart(seed, Snap(now()));
    if (est_->state() != before) ++state_changes_;
    return r;
  }

  /// IMU samples up to and including `end`.
  void ImuUntil(time::SensorTime end) {
    while (next_imu_ <= end) {
      const auto r = Imu(scene::make_imu(traj_, next_imu_));
      EXPECT_TRUE(r.has_value());
      next_imu_ = next_imu_ + imu_period_;
    }
  }

  ScanInput MakeScan(ScanKind kind, time::SensorTime start, time::SensorTime end) const {
    const scene::Pose pose = traj_.at(end);
    switch (kind) {
      case ScanKind::kRoom:
        return scene::make_scan(pose, start, end);
      case ScanKind::kPlane:
        return scene::make_plane_only_scan(pose, start, end);
      case ScanKind::kCorridor:
        return scene::make_corridor_scan(pose, start, end);
      case ScanKind::kEmpty:
        return ScanInput{start, end, {}, false};
    }
    return {};
  }

  /// `n` scan periods: IMU up to each scan end, then the scan (when `kind` is set).
  void Run(int n, std::optional<ScanKind> kind = ScanKind::kRoom) {
    for (int i = 0; i < n; ++i) {
      const time::SensorTime end = scan_end_ + kScanPeriod;
      ImuUntil(end + imu_lead_);
      if (kind) {
        const auto r = Scan(MakeScan(*kind, scan_end_, end));
        EXPECT_TRUE(r.has_value());
      }
      scan_end_ = end;
    }
  }

  /// A scan whose end is after the newest IMU sample (S1b precondition broken).
  Result<StepOutputs, EstimatorReason> ScanAheadOfImu() {
    const time::SensorTime end = scan_end_ + kScanPeriod;
    auto r = Scan(MakeScan(ScanKind::kRoom, scan_end_, end));
    scan_end_ = end;
    return r;
  }

  /// IMU pushed this far past each scan end before the scan (the IMU stream leads the LiDAR).
  void set_imu_lead(time::Duration lead) { imu_lead_ = lead; }

  void RunUntilTracking(int max_scans = 40) {
    for (int i = 0; i < max_scans && est_->state() != LioState::kTracking; ++i) Run(1);
    ASSERT_EQ(est_->state(), LioState::kTracking);
  }

  std::vector<events::EventRecord> Events() {
    recorder_->flush();
    const std::scoped_lock lock(store_->mutex);
    return store_->records;
  }

  std::vector<events::EventRecord> Events(std::string_view name) {
    std::vector<events::EventRecord> out;
    for (const auto& r : Events()) {
      if (r.event == name) out.push_back(r);
    }
    return out;
  }

  const std::vector<StepOutputs>& outputs() const { return outputs_; }
  int state_changes() const { return state_changes_; }
  bool all_finite() const { return all_finite_; }

 private:
  void Record(LioState before, const Result<StepOutputs, EstimatorReason>& r) {
    if (est_->state() != before) ++state_changes_;
    if (!r) return;
    if (!Finite(*r)) all_finite_ = false;
    EXPECT_TRUE(Finite(*r)) << "non-finite output";
    outputs_.push_back(*r);
  }

  std::shared_ptr<Store> store_;
  std::unique_ptr<events::EventRecorder> recorder_;
  std::unique_ptr<LioEstimator> est_;
  scene::Trajectory traj_;
  time::Duration imu_period_;
  time::SensorTime next_imu_{kT0};
  time::SensorTime scan_end_{kT0};
  time::Duration imu_lead_{};
  std::vector<StepOutputs> outputs_;
  int state_changes_{0};
  bool all_finite_{true};
};

std::vector<LioReason> HealthReasons(const std::vector<StepOutputs>& outputs, std::size_t from = 0) {
  std::vector<LioReason> reasons;
  for (std::size_t i = from; i < outputs.size(); ++i) {
    if (outputs[i].health) reasons.push_back(outputs[i].health->reason);
  }
  return reasons;
}

bool Contains(const std::vector<LioReason>& v, LioReason r) { return std::find(v.begin(), v.end(), r) != v.end(); }

}  // namespace

// ---------------------------------------------------------------------------------------------------

TEST(LioEstimator, ReachesTrackingInStaticRoom) {
  Harness h;
  h.Run(20);  // 2 s of 200 Hz IMU and 10 Hz scans
  EXPECT_EQ(h.est().state(), LioState::kTracking);
  EXPECT_EQ(h.est().epoch(), 1U);

  std::optional<OdometryOutput> last;
  std::size_t states = 0;
  for (const StepOutputs& o : h.outputs()) {
    if (o.odometry) last = o.odometry;
    states += o.states.size();
  }
  ASSERT_TRUE(last.has_value());
  EXPECT_GT(last->quality, 50U);
  EXPECT_EQ(last->epoch, 1U);
  EXPECT_EQ(last->reset_counter, 0U);
  EXPECT_EQ(last->t, kT0 + Times(kScanPeriod, 20));  // the scan END time
  // World = the IMU frame at initialisation (level IMU, yaw 0), so base_link sits at -t(base_T_imu).
  const Eigen::Vector3d expected_base = -scene::base_T_imu().translation();
  EXPECT_LT((last->p_world_m - expected_base).norm(), 0.05) << last->p_world_m.transpose();
  EXPECT_LT(last->q_world_base.angularDistance(Eigen::Quaterniond::Identity()), 0.01);
  EXPECT_LT(last->v_world_mps.norm(), 0.05);
  EXPECT_GT(last->pose_cov.diagonal().minCoeff(), 0.0);
  // One state output per IMU sample after the initialiser's 200 samples (aligned on the 200th).
  EXPECT_GT(states, 150U);
  EXPECT_TRUE(h.all_finite());

  // Every scan logged its eigenvalues (§3.2); room scans are good and the blocks are not swapped:
  // rotation information carries the squared lever arm (meters^2), so it is the larger block here.
  const auto scans = h.Events("LioScan");
  ASSERT_FALSE(scans.empty());
  const auto& good = scans.back();
  EXPECT_EQ(good.reason, "SCAN_GOOD");
  EXPECT_GE(Value(good, "translation_min_eigenvalue"), kTranslationMinInfo);
  EXPECT_GE(Value(good, "rotation_min_eigenvalue"), kRotationMinInfo);
  EXPECT_GT(Value(good, "rotation_min_eigenvalue"), Value(good, "translation_min_eigenvalue"));
  EXPECT_GT(Value(good, "quality"), 50.0);
  RecordProperty("room_translation_min_eigenvalue", std::to_string(Value(good, "translation_min_eigenvalue")));
  RecordProperty("room_rotation_min_eigenvalue", std::to_string(Value(good, "rotation_min_eigenvalue")));

  // Nominal run: no predictor rejection, no conversion failure, no IMU gap, no failed prediction.
  EXPECT_TRUE(h.Events("PredictorCorrectionRejected").empty());
  EXPECT_TRUE(h.Events("OdometryInvalid").empty());
  EXPECT_TRUE(h.Events("ImuGap").empty());
  for (const auto& r : scans) EXPECT_NE(r.reason, "PREDICTION_FAILED");
  // The IMU-rate output tracks the corrected state: the IMU stays at the world origin.
  std::optional<StateOutput> newest_state;
  for (const StepOutputs& o : h.outputs()) {
    if (!o.states.empty()) newest_state = o.states.back();
  }
  ASSERT_TRUE(newest_state.has_value());
  const StateOutput& newest = *newest_state;
  EXPECT_LT(newest.sample.p_world_m.norm(), 0.05) << newest.sample.p_world_m.transpose();
  EXPECT_EQ(newest.state, LioState::kTracking);
  EXPECT_EQ(newest.epoch, 1U);
}

TEST(LioEstimator, ImuOnlyGapReachesLost) {
  Harness h;
  h.RunUntilTracking();
  const std::size_t from = h.outputs().size();
  // F22, pinned in sensor time: the last scan ended at `last` (the newest IMU sample too). The gap rule is
  // strict (> 0.25 s, > 0.5 s) and evaluated on each 5 ms IMU tick.
  const time::SensorTime last = h.now();
  const auto at = [&](std::int64_t ms) { return last + time::milliseconds(ms); };
  h.ImuUntil(at(250));
  EXPECT_EQ(h.est().state(), LioState::kTracking) << "a gap of exactly 0.25 s is not > 0.25 s";
  h.ImuUntil(at(255));
  EXPECT_EQ(h.est().state(), LioState::kDegraded);
  ASSERT_TRUE(h.outputs().back().health.has_value());
  EXPECT_EQ(h.outputs().back().health->t, at(255));
  EXPECT_EQ(h.outputs().back().health->reason, LioReason::kLidarGapDegraded);
  h.ImuUntil(at(500));
  EXPECT_EQ(h.est().state(), LioState::kDegraded) << "a gap of exactly 0.5 s is not > 0.5 s";
  h.ImuUntil(at(505));
  EXPECT_EQ(h.est().state(), LioState::kLost);
  ASSERT_TRUE(h.outputs().back().health.has_value());
  EXPECT_EQ(h.outputs().back().health->t, at(505));
  EXPECT_EQ(h.outputs().back().health->reason, LioReason::kLidarGapLost);
  const auto transitions = h.Events("StateTransition");
  ASSERT_GE(transitions.size(), 2U);
  EXPECT_EQ(Value(transitions[transitions.size() - 2], "sensor_time_s"), static_cast<double>(at(255).ns) / 1e9);
  EXPECT_EQ(Value(transitions.back(), "sensor_time_s"), static_cast<double>(at(505).ns) / 1e9);
  h.ImuUntil(at(1000));  // 1 s of IMU, no scans
  EXPECT_EQ(h.est().state(), LioState::kLost);

  const auto reasons = HealthReasons(h.outputs(), from);
  const auto degraded = std::find(reasons.begin(), reasons.end(), LioReason::kLidarGapDegraded);
  const auto lost = std::find(reasons.begin(), reasons.end(), LioReason::kLidarGapLost);
  ASSERT_NE(degraded, reasons.end());
  ASSERT_NE(lost, reasons.end());
  EXPECT_LT(degraded, lost);
  for (std::size_t i = from; i < h.outputs().size(); ++i) EXPECT_FALSE(h.outputs()[i].odometry.has_value());
  // The predictor keeps publishing IMU-rate states while the LiDAR is gone, tagged with the state
  // (the last call of an IMU-only run is an IMU sample).
  ASSERT_EQ(h.outputs().back().states.size(), 1U);
  EXPECT_EQ(h.outputs().back().states.back().state, LioState::kLost);
  // correction_age_s grows with IMU time.
  for (std::size_t i = h.outputs().size(); i-- > from;) {
    if (h.outputs()[i].health) {
      EXPECT_GT(h.outputs()[i].health->correction_age_s, 0.9);
      break;
    }
  }
}

TEST(LioEstimator, PlaneOnlyScansDegradeWithoutThrowing) {
  Harness h;
  h.RunUntilTracking();
  const std::size_t from = h.outputs().size();
  EXPECT_NO_THROW(h.Run(2, ScanKind::kPlane));
  // The degenerate window: the lifecycle still counts 2 degenerate scans in TRACKING, yet no odometry.
  EXPECT_EQ(h.est().state(), LioState::kTracking);
  for (std::size_t i = from; i < h.outputs().size(); ++i) EXPECT_FALSE(h.outputs()[i].odometry.has_value()) << i;
  EXPECT_NO_THROW(h.Run(3, ScanKind::kPlane));
  EXPECT_EQ(h.est().state(), LioState::kDegraded);
  EXPECT_TRUE(Contains(HealthReasons(h.outputs(), from), LioReason::kDegenerateScans));
  for (std::size_t i = from; i < h.outputs().size(); ++i) EXPECT_FALSE(h.outputs()[i].odometry.has_value()) << i;
  const auto scans = h.Events("LioScan");
  ASSERT_GE(scans.size(), 5U);
  for (std::size_t i = scans.size() - 5; i < scans.size(); ++i) {
    EXPECT_EQ(scans[i].reason, "SCAN_DEGENERATE");
    EXPECT_LT(Value(scans[i], "translation_min_eigenvalue"), 0.05 * kTranslationMinInfo);
    EXPECT_LT(Value(scans[i], "quality"), 50.0);
  }
}

// Ruling 6: the information matrix reaches evaluate_degeneracy with the right blocks. A floor-only scan
// never confirms TRACKING (translation block rank-deficient); a corridor (floor, ceiling, x walls) is
// translation-degenerate along y but rotation well constrained, which a swapped block would invert.
TEST(LioEstimator, PlaneOnlyScansNeverReachTrackingAndBlocksAreNotSwapped) {
  Harness h;
  h.Run(11);  // IMU init (1 s) and the map bootstrap scan, all faces
  h.Run(20, ScanKind::kPlane);
  EXPECT_EQ(h.est().state(), LioState::kInitializing);
  for (const StepOutputs& o : h.outputs()) EXPECT_FALSE(o.odometry.has_value());

  auto scans = h.Events("LioScan");
  ASSERT_GE(scans.size(), 20U);
  const auto& plane = scans.back();
  EXPECT_EQ(plane.reason, "SCAN_DEGENERATE");
  // Floor: only z translation is observed. The smallest translation eigenvalue is not exactly zero (map
  // neighbourhoods at the floor/wall edge still pass the plane fit with a slightly tilted normal) but it is
  // below 5 % of the threshold, i.e. rank-deficient for the check.
  EXPECT_LT(Value(plane, "translation_min_eigenvalue"), 0.05 * kTranslationMinInfo);
  RecordProperty("plane_translation_min_eigenvalue", std::to_string(Value(plane, "translation_min_eigenvalue")));
  RecordProperty("plane_rotation_min_eigenvalue", std::to_string(Value(plane, "rotation_min_eigenvalue")));

  h.Run(1, ScanKind::kCorridor);
  scans = h.Events("LioScan");
  const auto& corridor = scans.back();
  EXPECT_EQ(corridor.reason, "SCAN_DEGENERATE");
  EXPECT_LT(Value(corridor, "translation_min_eigenvalue"), 0.05 * kTranslationMinInfo);  // y unobserved
  EXPECT_GE(Value(corridor, "rotation_min_eigenvalue"), kRotationMinInfo);               // every rotation axis observed
  RecordProperty("corridor_translation_min_eigenvalue", std::to_string(Value(corridor, "translation_min_eigenvalue")));
  RecordProperty("corridor_rotation_min_eigenvalue", std::to_string(Value(corridor, "rotation_min_eigenvalue")));
  EXPECT_EQ(h.est().state(), LioState::kInitializing);

  // A full room scan afterwards is non-degenerate with quality > 50.
  h.Run(1, ScanKind::kRoom);
  scans = h.Events("LioScan");
  EXPECT_EQ(scans.back().reason, "SCAN_GOOD");
  EXPECT_GT(Value(scans.back(), "quality"), 50.0);
}

TEST(LioEstimator, EmptyScanIsAnEventNotAnException) {
  Harness h;
  h.RunUntilTracking();
  const std::size_t from = h.outputs().size();
  for (int i = 0; i < 3; ++i) EXPECT_NO_THROW(h.Run(1, ScanKind::kEmpty));
  EXPECT_EQ(h.est().state(), LioState::kDegraded);
  EXPECT_TRUE(Contains(HealthReasons(h.outputs(), from), LioReason::kDegenerateScans));
  const auto scans = h.Events("LioScan");
  ASSERT_GE(scans.size(), 3U);
  for (std::size_t i = scans.size() - 3; i < scans.size(); ++i) EXPECT_EQ(scans[i].reason, "SCAN_EMPTY");
  // Health shows no stale eigenvalues of the last good scan, and empty scans do not refresh the correction age.
  h.Run(1, std::nullopt);
  std::optional<HealthOutput> health;
  for (std::size_t i = from; i < h.outputs().size(); ++i) {
    if (h.outputs()[i].health) health = h.outputs()[i].health;
  }
  ASSERT_TRUE(health.has_value());
  EXPECT_EQ(health->translation_min_eigenvalue, 0.0);
  EXPECT_EQ(health->rotation_min_eigenvalue, 0.0);
  EXPECT_GE(health->correction_age_s, 0.35);
  EXPECT_TRUE(h.Events("OdometryInvalid").empty());
}

// The S1b precondition (IMU reached scan.end) broken: the scan is never trusted, the ESKF is rebased with an
// inflated covariance, and operation continues once the IMU leads again.
TEST(LioEstimator, ScanAheadOfImuIsDegenerateAndRebased) {
  Harness h;
  h.RunUntilTracking();
  const std::size_t from = h.outputs().size();
  const auto r = h.ScanAheadOfImu();
  ASSERT_TRUE(r.has_value());
  EXPECT_FALSE(r->odometry.has_value());
  const auto scans = h.Events("LioScan");
  EXPECT_EQ(scans.back().reason, "SCAN_AHEAD_OF_IMU");
  const auto rebased = h.Events("EskfRebased");
  ASSERT_EQ(rebased.size(), 1U);
  EXPECT_DOUBLE_EQ(Value(rebased[0], "skipped_s"), 0.1);
  EXPECT_GT(Value(rebased[0], "position_sigma_m"), 0.0);
  EXPECT_LT(Value(rebased[0], "position_sigma_m"), 0.5);  // one rebase does not lose the state
  EXPECT_EQ(h.est().state(), LioState::kTracking);        // one degenerate scan of three
  h.Run(3);
  EXPECT_EQ(h.est().state(), LioState::kTracking);
  bool odometry = false;
  for (std::size_t i = from; i < h.outputs().size(); ++i) odometry = odometry || h.outputs()[i].odometry.has_value();
  EXPECT_TRUE(odometry);
}

// D28 (the IMU leads each scan by 120 ms, so a periodic health at scan end + 100 ms precedes the scan-path\n//
// transition): health stamps follow the newest IMU time, so they never decrease, even when the IMU leads the LiDAR and
// a scan-path transition reports health for an older scan end.
TEST(LioEstimator, HealthStampsNeverDecrease) {
  Harness h;
  h.set_imu_lead(time::milliseconds(120));  // past the next 100 ms health grid point
  h.RunUntilTracking();                     // scan-path transition INITIALIZING -> TRACKING
  h.Run(3, ScanKind::kEmpty);               // scan-path transition -> DEGRADED
  h.Run(6);                                 // scan-path transition -> TRACKING
  h.Run(10, std::nullopt);                  // IMU-path transitions -> DEGRADED -> LOST
  ASSERT_EQ(h.est().state(), LioState::kLost);
  std::optional<time::SensorTime> previous;
  int count = 0;
  for (const StepOutputs& o : h.outputs()) {
    if (!o.health) continue;
    ++count;
    if (previous) {
      EXPECT_GE(o.health->t, *previous) << count;
    }
    previous = o.health->t;
  }
  EXPECT_GT(count, 20);
}

TEST(LioEstimator, RejectsOutOfOrderInput) {
  Harness h;
  const time::SensorTime t1{kT0.ns + 1'000'000'000};
  ASSERT_TRUE(h.Imu(scene::make_imu({}, t1)).has_value());
  const auto r = h.Imu(scene::make_imu({}, time::SensorTime{t1.ns - 100'000'000}));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), EstimatorReason::kOutOfOrder);
  // The regression changed nothing: the next sample after t1 is accepted, an equal stamp too (duplicate).
  EXPECT_TRUE(h.Imu(scene::make_imu({}, time::SensorTime{t1.ns + 5'000'000})).has_value());
  EXPECT_TRUE(h.Imu(scene::make_imu({}, time::SensorTime{t1.ns + 5'000'000})).has_value());
  EXPECT_EQ(h.Events("ImuDuplicate").size(), 1U);
  EXPECT_EQ(h.Events("LioInputRejected").size(), 1U);
}

// Per-sample events are rate-limited: 1st, 1000th, 2000th occurrence per reason and epoch.
TEST(LioEstimator, PerSampleEventsAreRateLimited) {
  Harness h;
  const time::SensorTime t1{kT0.ns + 1'000'000'000};
  ASSERT_TRUE(h.Imu(scene::make_imu({}, t1)).has_value());
  for (int i = 0; i < 2500; ++i) ASSERT_TRUE(h.Imu(scene::make_imu({}, t1)).has_value());
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(h.Imu(scene::make_imu({}, kT0)).has_value());
  const auto duplicates = h.Events("ImuDuplicate");
  ASSERT_EQ(duplicates.size(), 3U);
  EXPECT_EQ(Value(duplicates[0], "count"), 1.0);
  EXPECT_EQ(Value(duplicates[1], "count"), 1000.0);
  EXPECT_EQ(Value(duplicates[2], "count"), 2000.0);
  const auto rejected = h.Events("LioInputRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "OUT_OF_ORDER");
}

TEST(LioEstimator, RejectsOutOfOrderScan) {
  Harness h;
  h.RunUntilTracking();
  const time::SensorTime end = h.now();
  const auto stale = h.Scan(h.MakeScan(ScanKind::kRoom, end - Times(kScanPeriod, 2), end - kScanPeriod));
  ASSERT_FALSE(stale.has_value());
  EXPECT_EQ(stale.error(), EstimatorReason::kOutOfOrder);
  const auto reversed = h.Scan(h.MakeScan(ScanKind::kRoom, end + kScanPeriod, end));
  ASSERT_FALSE(reversed.has_value());
  EXPECT_EQ(reversed.error(), EstimatorReason::kOutOfOrder);
  EXPECT_EQ(h.est().state(), LioState::kTracking);
  h.Run(3);
  EXPECT_EQ(h.est().state(), LioState::kTracking);
}

TEST(LioEstimator, NotFiniteInputsAreRejectedAndChangeNothing) {
  Harness h;
  h.RunUntilTracking();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ImuInput bad = scene::make_imu({}, h.now() + kImuPeriod);
  bad.gyro_rad_s.x() = nan;
  auto r = h.Imu(bad);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), EstimatorReason::kNotFinite);
  bad = scene::make_imu({}, h.now() + kImuPeriod);
  bad.accel_mps2.z() = std::numeric_limits<double>::infinity();
  EXPECT_EQ(h.Imu(bad).error(), EstimatorReason::kNotFinite);

  ScanInput scan = h.MakeScan(ScanKind::kRoom, h.now(), h.now() + kScanPeriod);
  scan.points[17].position_lidar_m.y() = std::numeric_limits<float>::quiet_NaN();
  const auto s = h.Scan(std::move(scan));
  ASSERT_FALSE(s.has_value());
  EXPECT_EQ(s.error(), EstimatorReason::kNotFinite);
  EXPECT_EQ(h.est().state(), LioState::kTracking);
  h.Run(3);  // the NaN stamps were never stored: normal operation continues
  EXPECT_EQ(h.est().state(), LioState::kTracking);
}

TEST(LioEstimator, TooManyPointsIsRejected) {
  Harness h;
  h.RunUntilTracking();
  ScanInput scan{h.now(), h.now() + kScanPeriod, {}, false};
  scan.points.resize(lio::limits::kMaxScanPoints + 1);
  for (auto& p : scan.points) p.position_lidar_m = Eigen::Vector3f(5.0F, 0.0F, 0.0F);
  const auto r = h.Scan(std::move(scan));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), EstimatorReason::kTooManyPoints);
  EXPECT_EQ(h.est().state(), LioState::kTracking);
}

TEST(LioEstimator, RestartSeedsNewEpoch) {
  Harness h;
  h.RunUntilTracking();
  h.Run(10, std::nullopt);  // LiDAR gap -> LOST
  ASSERT_EQ(h.est().state(), LioState::kLost);
  h.Run(1);  // geometry returned
  ASSERT_EQ(h.est().state(), LioState::kRestarting);

  // One more IMU sample, so the newest predictor output is known exactly (the RESTARTING scan's correction
  // shifted the buffer after the previous output was returned).
  h.ImuUntil(h.now() + kImuPeriod);
  // Three duplicate IMU samples: one ImuDuplicate event now, a summary (count 3) at the end of the epoch.
  for (int i = 0; i < 3; ++i) ASSERT_TRUE(h.Imu(scene::make_imu({}, h.now())).has_value());
  EXPECT_EQ(h.Events("ImuDuplicate").size(), 1U);

  // The predictor's newest output before the restart (the old epoch's IMU state at the newest IMU time).
  std::optional<StateOutput> old_state;
  for (const StepOutputs& o : h.outputs()) {
    if (!o.states.empty()) old_state = o.states.back();
  }
  ASSERT_TRUE(old_state.has_value());
  ASSERT_EQ(old_state->sample.t, h.now());

  const double yaw = 0.3;
  const SeedPose seed{Eigen::Vector3d(1.0, 2.0, 0.5),
                      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))};
  const auto delta = h.Restart(seed);
  ASSERT_TRUE(delta.has_value());
  // ResetDelta = new - old output at the newest IMU time: the seeded IMU pose minus the old one.
  const Eigen::Vector3d new_imu_p = seed.p_world_m + seed.q_world_base * scene::base_T_imu().translation();
  EXPECT_LT((delta->position_m - (new_imu_p - old_state->sample.p_world_m)).norm(), 1e-9);
  // The old velocity (near zero) is carried into the seed's world: v_new = R(seed) R(old)^-1 v_old.
  const Eigen::Vector3d v_old = old_state->sample.v_world_mps;
  const Eigen::Vector3d v_new = (seed.q_world_base * old_state->sample.q_world_imu.conjugate()) * v_old;
  EXPECT_LT((delta->velocity_mps - (v_new - v_old)).norm(), 1e-9);
  EXPECT_LT(delta->velocity_mps.norm(), 0.01);
  const Eigen::Matrix3d r_old = old_state->sample.q_world_imu.toRotationMatrix();
  EXPECT_NEAR(delta->yaw_rad, yaw - std::atan2(r_old(1, 0), r_old(0, 0)), 1e-9);
  const auto duplicates = h.Events("ImuDuplicate");
  ASSERT_EQ(duplicates.size(), 2U);
  EXPECT_EQ(Value(duplicates.back(), "count"), 3.0);
  EXPECT_EQ(duplicates.back().identity.lio_epoch, 1U);
  EXPECT_EQ(h.est().epoch(), 2U);
  EXPECT_EQ(h.est().state(), LioState::kInitializing);

  const auto restarts = h.Events("LioRestart");
  ASSERT_EQ(restarts.size(), 1U);
  EXPECT_EQ(Value(restarts[0], "old_epoch"), 1.0);
  EXPECT_EQ(Value(restarts[0], "new_epoch"), 2.0);
  EXPECT_DOUBLE_EQ(Value(restarts[0], "seed_x_m"), 1.0);
  EXPECT_DOUBLE_EQ(Value(restarts[0], "seed_y_m"), 2.0);
  EXPECT_DOUBLE_EQ(Value(restarts[0], "seed_z_m"), 0.5);
  EXPECT_NEAR(Value(restarts[0], "seed_yaw_rad"), yaw, 1e-12);
  EXPECT_EQ(restarts[0].identity.lio_epoch, 2U);

  const std::size_t from = h.outputs().size();
  h.RunUntilTracking();
  // reset_counter + 1 on the first state output of the new epoch, and on the odometry.
  bool saw_state = false;
  std::optional<OdometryOutput> odom;
  for (std::size_t i = from; i < h.outputs().size(); ++i) {
    for (const StateOutput& s : h.outputs()[i].states) {
      EXPECT_EQ(s.sample.reset_counter, 1U);
      EXPECT_EQ(s.epoch, 2U);
      saw_state = true;
    }
    if (h.outputs()[i].odometry) odom = h.outputs()[i].odometry;
  }
  EXPECT_TRUE(saw_state);
  h.Run(2);
  for (std::size_t i = from; i < h.outputs().size(); ++i) {
    if (h.outputs()[i].odometry) odom = h.outputs()[i].odometry;
  }
  ASSERT_TRUE(odom.has_value());
  EXPECT_EQ(odom->epoch, 2U);
  EXPECT_EQ(odom->reset_counter, 1U);
  // The new epoch's world is the seed's: a stationary vehicle stays at the seed pose.
  EXPECT_LT((odom->p_world_m - seed.p_world_m).norm(), 0.05) << odom->p_world_m.transpose();
  EXPECT_LT(odom->q_world_base.angularDistance(seed.q_world_base), 0.01);
  EXPECT_TRUE(h.all_finite());
}

TEST(LioEstimator, RestartRejectedOutsideRestarting) {
  Harness h;
  const SeedPose seed{Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  auto r = h.Restart(seed);  // INITIALIZING, not even IMU-initialised
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), EstimatorReason::kWrongState);
  h.RunUntilTracking();
  r = h.Restart(seed);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), EstimatorReason::kWrongState);
  EXPECT_EQ(h.est().state(), LioState::kTracking);
  EXPECT_EQ(h.est().epoch(), 1U);
  EXPECT_TRUE(h.Events("LioRestart").empty());
}

TEST(LioEstimator, OneStateTransitionEventPerTransition) {
  Harness h;
  h.RunUntilTracking();        // INITIALIZING -> TRACKING
  h.Run(3, ScanKind::kEmpty);  // -> DEGRADED
  h.Run(10, std::nullopt);     // -> LOST
  h.Run(1);                    // -> RESTARTING
  ASSERT_TRUE(h.Restart({Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()}).has_value());  // -> INIT
  h.RunUntilTracking();                                                                           // -> TRACKING
  const auto transitions = h.Events("StateTransition");
  EXPECT_EQ(static_cast<int>(transitions.size()), h.state_changes());
  EXPECT_EQ(h.state_changes(), 6);
  const std::vector<std::pair<std::string_view, std::string_view>> expected{
      {"INITIALIZING", "TRACKING"}, {"TRACKING", "DEGRADED"},       {"DEGRADED", "LOST"},
      {"LOST", "RESTARTING"},       {"RESTARTING", "INITIALIZING"}, {"INITIALIZING", "TRACKING"}};
  ASSERT_EQ(transitions.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(transitions[i].state_before, expected[i].first) << i;
    EXPECT_EQ(transitions[i].state_after, expected[i].second) << i;
    EXPECT_FALSE(transitions[i].reason.empty());
  }
}

TEST(LioEstimator, ImuRateTooHighIsReportedOncePerEpoch) {
  Harness fast(DefaultConfig(), time::nanoseconds(2'500'000));  // 400 Hz: 64 x 2.5 ms < 300 ms
  fast.Run(20);
  const auto events = fast.Events("ImuRateTooHigh");
  ASSERT_EQ(events.size(), 1U);
  EXPECT_NEAR(Value(events[0], "imu_period_s"), 0.0025, 1e-4);

  Harness nominal;  // 200 Hz: 64 x 5 ms >= 300 ms
  nominal.Run(20);
  EXPECT_TRUE(nominal.Events("ImuRateTooHigh").empty());
}

TEST(LioEstimator, IdenticalRunsProduceIdenticalOutputs) {
  const auto scenario = [](Harness& h) {
    h.RunUntilTracking();
    h.Run(2, ScanKind::kPlane);
    h.Run(4);
    h.Run(10, std::nullopt);
    h.Run(1);
  };
  Harness a;
  Harness b;
  scenario(a);
  scenario(b);
  ASSERT_EQ(a.outputs().size(), b.outputs().size());
  for (std::size_t i = 0; i < a.outputs().size(); ++i) {
    const StepOutputs& x = a.outputs()[i];
    const StepOutputs& y = b.outputs()[i];
    ASSERT_EQ(x.states.size(), y.states.size()) << i;
    for (std::size_t k = 0; k < x.states.size(); ++k) {
      EXPECT_EQ(x.states[k].sample.p_world_m, y.states[k].sample.p_world_m) << i;
      EXPECT_EQ(x.states[k].sample.v_world_mps, y.states[k].sample.v_world_mps) << i;
      EXPECT_EQ(x.states[k].sample.q_world_imu.coeffs(), y.states[k].sample.q_world_imu.coeffs()) << i;
      EXPECT_EQ(x.states[k].state, y.states[k].state) << i;
    }
    ASSERT_EQ(x.odometry.has_value(), y.odometry.has_value()) << i;
    if (x.odometry) {
      EXPECT_EQ(x.odometry->p_world_m, y.odometry->p_world_m) << i;
      EXPECT_EQ(x.odometry->pose_cov, y.odometry->pose_cov) << i;
      EXPECT_EQ(x.odometry->quality, y.odometry->quality) << i;
    }
    ASSERT_EQ(x.health.has_value(), y.health.has_value()) << i;
    if (x.health) {
      EXPECT_EQ(x.health->reason, y.health->reason) << i;
      EXPECT_EQ(x.health->translation_min_eigenvalue, y.health->translation_min_eigenvalue) << i;
    }
  }
  EXPECT_EQ(a.est().state(), b.est().state());
}

TEST(LioEstimator, CreateRejectsAnExtrinsicThatDisagreesWithTheConfig) {
  auto store = std::make_shared<Store>();
  auto recorder = events::EventRecorder::create(std::make_unique<FakeSink>(store));
  ASSERT_TRUE(recorder.has_value());
  Eigen::Isometry3d imu_T_lidar = scene::imu_T_lidar();
  imu_T_lidar.translation().x() += 0.01;
  const auto r = LioEstimator::create(DefaultConfig(), scene::base_T_imu(), imu_T_lidar, **recorder);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().key, "extrinsic_imu_lidar_x_m");

  Eigen::Isometry3d bad_base = scene::base_T_imu();
  bad_base.translation().z() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(LioEstimator::create(DefaultConfig(), bad_base, scene::imu_T_lidar(), **recorder).has_value());
}
