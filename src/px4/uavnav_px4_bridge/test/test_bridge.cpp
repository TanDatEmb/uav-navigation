// Px4Bridge: the ROS-free decision layer of the px4_bridge node (SYSTEM_DESIGN §4.1, D11, D21, D28, F13, F14,
// D33/O21, O22).
//
// Conventions: synthetic data only. ROS time equals sensor time (SITL, D29). PX4 local position streams at 50 Hz
// on a 20 ms grid, LIO odometry at 10 Hz stamped 3 ms after the 100 ms grid and delivered 50 ms after its stamp
// (after the PX4 samples around it), ticks every 100 ms. The LIO moves with constant velocity (linear PX4
// interpolation is then exact) and PX4 reports the same motion through the truth T.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <random>
#include <set>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uavnav/core/event_recorder.hpp"
#include "uavnav/px4bridge/bridge.hpp"
#include "uavnav/px4bridge/frames.hpp"
#include "uavnav/px4bridge/limits.hpp"

using namespace uavnav;
using namespace uavnav::px4bridge;

namespace {

constexpr double kPi = std::numbers::pi;

double Wrap(double a) { return std::remainder(a, 2.0 * kPi); }

AlignmentConfig BetaConfig() {
  return AlignmentConfig{time::seconds(2), 20U, 0.5, 0.0873, time::seconds(1), time::seconds(10)};
}

time::SensorTime Ms(std::int64_t ms) { return time::SensorTime{ms * 1'000'000}; }
time::TimeSnapshot Snap(time::SensorTime t) { return {time::SteadyTime{t.ns}, time::RosTime{t.ns}}; }

// ---------------------------------------------------------------------------------------------------------------
// Event capture: the recorder writes on its own thread; Events() flushes and then reads the store.

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

std::unique_ptr<events::EventRecorder> MakeRecorder(const std::shared_ptr<Store>& store) {
  return std::move(events::EventRecorder::create(std::make_unique<FakeSink>(store)).value());
}

struct Rig {
  explicit Rig(const AlignmentConfig& cfg = BetaConfig())
      : store(std::make_shared<Store>()), recorder(MakeRecorder(store)), bridge(cfg, *recorder) {}

  std::vector<events::EventRecord> Events(std::string_view name) {
    recorder->flush();
    const std::scoped_lock lock(store->mutex);
    std::vector<events::EventRecord> out;
    for (const events::EventRecord& r : store->records) {
      if (r.event == name) out.push_back(r);
    }
    return out;
  }

  std::shared_ptr<Store> store;
  std::unique_ptr<events::EventRecorder> recorder;
  Px4Bridge bridge;
};

// ---------------------------------------------------------------------------------------------------------------
// Synthetic world.

Eigen::Vector2d Rot(double yaw, const Eigen::Vector2d& v) {
  return {std::cos(yaw) * v.x() - std::sin(yaw) * v.y(), std::sin(yaw) * v.x() + std::cos(yaw) * v.y()};
}

// T(p) = (Rz(yaw_T) p_xy + t_xy, p_z + t_z): where T puts the lio_odom FRD point p in PX4 local NED.
Eigen::Vector3d Apply(const Pose4& T, const Eigen::Vector3d& p) {
  const Eigen::Vector2d xy = Rot(T.yaw_rad, p.head<2>()) + Eigen::Vector2d{T.x_m, T.y_m};
  return {xy.x(), xy.y(), p.z() + T.z_m};
}

// The PX4 frame change G of a reset (alignment.hpp, F13) applied to T: G o T, rotation dh about `pivot`, then d.
Pose4 FrameChange(const Pose4& T, const Eigen::Vector3d& pivot, const Eigen::Vector3d& d, double dh) {
  const Eigen::Vector2d t_xy = Rot(dh, Eigen::Vector2d{T.x_m, T.y_m} - pivot.head<2>()) + pivot.head<2>() + d.head<2>();
  return Pose4{t_xy.x(), t_xy.y(), T.z_m + d.z(), Wrap(T.yaw_rad + dh)};
}

void ExpectPoseNear(const Pose4& a, const Pose4& b, double tol) {
  EXPECT_NEAR(a.x_m, b.x_m, tol);
  EXPECT_NEAR(a.y_m, b.y_m, tol);
  EXPECT_NEAR(a.z_m, b.z_m, tol);
  EXPECT_NEAR(Wrap(a.yaw_rad - b.yaw_rad), 0.0, tol);
}

struct World {
  Pose4 truth{3.0, -2.0, 0.5, 0.6};     // lio_odom FRD -> PX4 local NED
  Eigen::Vector3d p0_flu{1.0, -2.0, 3.0};  // LIO position at t = 0, FLU
  Eigen::Vector3d v_flu{0.5, 0.2, 0.0};    // LIO velocity, FLU
  double yaw_flu{0.4};                     // LIO heading about FLU z (FRD heading is -yaw_flu)
  std::uint32_t epoch{1};
  // What every PX4 sample carries (the last reset).
  std::uint8_t xy_counter{0}, z_counter{0}, heading_counter{0};
  Eigen::Vector2d delta_xy{Eigen::Vector2d::Zero()};
  double delta_z{0.0}, delta_heading{0.0};

  Eigen::Vector3d PFlu(time::SensorTime t) const { return p0_flu + v_flu * time::to_seconds(t); }
  Eigen::Vector3d PFrd(time::SensorTime t) const { return flu_to_frd(PFlu(t)); }
  Eigen::Vector3d PNed(time::SensorTime t) const { return Apply(truth, PFrd(t)); }

  LioOdometryIn Odom(time::SensorTime t) const {
    LioOdometryIn o;
    o.t = t;
    o.epoch = epoch;
    o.p_flu_m = PFlu(t);
    o.q_flu = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_flu, Eigen::Vector3d::UnitZ()));
    o.v_flu_mps = v_flu;
    o.pose_cov = 1e-4 * Eigen::Matrix<double, 6, 6>::Identity();
    o.vel_cov = 1e-4 * Eigen::Matrix3d::Identity();
    o.quality = 80;
    return o;
  }

  Px4LocalPositionIn Px4(time::SensorTime t) const {
    Px4PoseSample s;
    s.t = t;
    s.p_ned_m = PNed(t);
    s.yaw_rad = Wrap(truth.yaw_rad - yaw_flu);
    s.xy_reset_counter = xy_counter;
    s.z_reset_counter = z_counter;
    s.heading_reset_counter = heading_counter;
    s.delta_xy_m = delta_xy;
    s.delta_z_m = delta_z;
    s.delta_heading_rad = delta_heading;
    return Px4LocalPositionIn{s, true, true};
  }

  LioHealthIn Health(time::SensorTime t, LioHealthCode code) const { return LioHealthIn{t, code, epoch}; }
};

enum class Kind { kPx4, kOdometry, kTick };

struct Step {
  Kind kind;
  time::SensorTime t;
  BridgeOutputs out;
};

/// Streams the world from from_ms (inclusive) to to_ms (exclusive), in time order (see the file comment).
std::vector<Step> Stream(Rig& rig, const World& w, std::int64_t from_ms, std::int64_t to_ms, bool odometry = true) {
  std::vector<Step> steps;
  for (std::int64_t m = from_ms; m < to_ms; ++m) {
    const time::SensorTime now = Ms(m);
    if (m % 100 == 0) steps.push_back({Kind::kTick, now, rig.bridge.on_tick(now, Snap(now))});
    if (m % 20 == 0) steps.push_back({Kind::kPx4, now, rig.bridge.on_px4_local_position(w.Px4(now), Snap(now))});
    if (odometry && m % 100 == 53) {
      const time::SensorTime stamp = Ms(m - 50);
      steps.push_back({Kind::kOdometry, stamp, rig.bridge.on_lio_odometry(w.Odom(stamp), Snap(now))});
    }
  }
  return steps;
}

/// The newest alignment output of `steps` from an odometry call.
AlignmentOutput LastOdometryAlignment(const std::vector<Step>& steps) {
  for (auto it = steps.rbegin(); it != steps.rend(); ++it) {
    if (it->kind == Kind::kOdometry && it->out.alignment) return *it->out.alignment;
  }
  ADD_FAILURE() << "no odometry alignment output";
  return AlignmentOutput{};
}

/// Runs the world 1.0 s .. 3.6 s: 26 scans, VALID after the 20th.
std::vector<Step> RunToValid(Rig& rig, const World& w) {
  auto steps = Stream(rig, w, 1000, 3600);
  EXPECT_EQ(rig.bridge.alignment_state(), AlignmentState::kValid);
  // VALID exactly at the 20th consistent pair (alignment_consistent_pairs).
  int pairs = 0;
  for (const Step& s : steps) {
    if (s.kind != Kind::kOdometry) continue;
    ++pairs;
    if (s.out.alignment->state == AlignmentState::kValid) {
      EXPECT_EQ(pairs, 20);
      EXPECT_EQ(s.out.alignment->reason, AlignmentReason::kPairsConsistent);
      break;
    }
  }
  return steps;
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------

TEST(Px4Bridge, EvOnlyFromTrackingOdometryWithItsOwnEpoch) {
  Rig rig;
  World w;
  w.epoch = 3;
  // No tick yet: nothing is refused as ahead, even at 1000 s.
  const time::SensorTime t = Ms(1'000'000);
  // A LOST health of ANOTHER epoch, newer than the odometry, does not gate it (F14: only the sample's own epoch).
  ASSERT_TRUE(rig.bridge.on_lio_health(LioHealthIn{Ms(1'000'200), LioHealthCode::kLost, 2}, Snap(t)).alignment);

  LioOdometryIn odom = w.Odom(t);
  odom.quality = 200;
  const BridgeOutputs out = rig.bridge.on_lio_odometry(odom, Snap(t));
  ASSERT_TRUE(out.ev);
  EXPECT_EQ(out.ev->reset_counter, 3U);  // the sample's own epoch (F14)
  EXPECT_EQ(out.ev->quality, 0);         // above 100: out of contract -> 0 (D28)
  EXPECT_EQ(out.ev->pose_frame, kPoseFrameFrd);
  EXPECT_EQ(out.ev->velocity_frame, kVelocityFrameFrd);
  EXPECT_EQ(out.ev->timestamp_sample_us, static_cast<std::uint64_t>(t.ns / 1000));
  const Eigen::Vector3d p = w.PFlu(t);
  EXPECT_FLOAT_EQ(out.ev->position[0], static_cast<float>(p.x()));
  EXPECT_FLOAT_EQ(out.ev->position[1], static_cast<float>(-p.y()));
  EXPECT_FLOAT_EQ(out.ev->position[2], static_cast<float>(-p.z()));
  EXPECT_TRUE(out.alignment);  // fed to the alignment (refused there: older than the health it already saw)
  EXPECT_TRUE(rig.Events("EvGated").empty());

  // The epoch wraps into the uint8 counter: 300 -> 44.
  LioOdometryIn later = w.Odom(Ms(1'000'300));
  later.epoch = 300;
  const BridgeOutputs out2 = rig.bridge.on_lio_odometry(later, Snap(later.t));
  ASSERT_TRUE(out2.ev);
  EXPECT_EQ(out2.ev->reset_counter, 44U);
}

TEST(Px4Bridge, NewerNonTrackingHealthBlocksEv) {
  Rig rig;
  World w;
  // A non-TRACKING health is fed to the alignment (an output); a TRACKING one is not.
  EXPECT_TRUE(rig.bridge.on_lio_health(w.Health(Ms(9'000), LioHealthCode::kTracking), Snap(Ms(9'000))).alignment ==
              std::nullopt);
  EXPECT_TRUE(rig.bridge.on_lio_health(w.Health(Ms(10'000), LioHealthCode::kDegraded), Snap(Ms(10'000))).alignment);

  // Same epoch, health stamp == odometry stamp: gated, not fed to the alignment.
  BridgeOutputs out = rig.bridge.on_lio_odometry(w.Odom(Ms(10'000)), Snap(Ms(10'010)));
  EXPECT_FALSE(out.ev);
  EXPECT_FALSE(out.alignment);
  // Health newer than the odometry: gated.
  out = rig.bridge.on_lio_odometry(w.Odom(Ms(9'950)), Snap(Ms(10'020)));
  EXPECT_FALSE(out.ev);
  const auto gated = rig.Events("EvGated");
  ASSERT_EQ(gated.size(), 1U);  // rate-limited: the 2nd is counted, not reported
  EXPECT_EQ(gated[0].component, events::Component::kPx4Bridge);
  EXPECT_EQ(gated[0].reason, "HEALTH_NOT_TRACKING");
  EXPECT_EQ(gated[0].identity.lio_epoch, 1U);
  EXPECT_DOUBLE_EQ(Value(gated[0], "odometry_time_s"), 10.0);
  EXPECT_DOUBLE_EQ(Value(gated[0], "health_time_s"), 10.0);
  EXPECT_DOUBLE_EQ(Value(gated[0], "health_state"), 2.0);
  EXPECT_DOUBLE_EQ(Value(gated[0], "count"), 1.0);

  // An OLDER DEGRADED health does not block.
  ASSERT_TRUE(rig.bridge.on_lio_health(w.Health(Ms(10'050), LioHealthCode::kDegraded), Snap(Ms(10'050))).alignment);
  out = rig.bridge.on_lio_odometry(w.Odom(Ms(10'100)), Snap(Ms(10'150)));
  EXPECT_TRUE(out.ev);
  EXPECT_TRUE(out.alignment);

  // The newest health is TRACKING: an odometry older than it is not blocked.
  EXPECT_FALSE(rig.bridge.on_lio_health(w.Health(Ms(10'300), LioHealthCode::kTracking), Snap(Ms(10'300))).alignment);
  EXPECT_TRUE(rig.bridge.on_lio_odometry(w.Odom(Ms(10'200)), Snap(Ms(10'310))).ev);

  // An unknown state code counts as not TRACKING (fail closed).
  ASSERT_TRUE(rig.bridge.on_lio_health(LioHealthIn{Ms(10'500), static_cast<LioHealthCode>(9), 1}, Snap(Ms(10'500)))
                  .alignment);
  EXPECT_FALSE(rig.bridge.on_lio_odometry(w.Odom(Ms(10'400)), Snap(Ms(10'510))).ev);
}

TEST(Px4Bridge, PerInputRejectionEventsAreRateLimited) {
  Rig rig;
  World w;
  ASSERT_TRUE(rig.bridge.on_lio_health(w.Health(Ms(100'000), LioHealthCode::kLost), Snap(Ms(100'000))).alignment);
  for (std::int64_t i = 0; i < 1000; ++i) {
    EXPECT_FALSE(rig.bridge.on_lio_odometry(w.Odom(Ms(1'000 + i)), Snap(Ms(100'000))).ev);
  }
  auto gated = rig.Events("EvGated");
  ASSERT_EQ(gated.size(), 2U);  // the 1st and the kEventSummaryEvery-th (safety net inside one episode)
  EXPECT_DOUBLE_EQ(Value(gated[0], "count"), 1.0);
  EXPECT_DOUBLE_EQ(Value(gated[1], "count"), static_cast<double>(px4bridge::limits::kEventSummaryEvery));

  // Two more, then flush() (the node's shutdown): one summary with the unreported ones, nothing lost.
  rig.bridge.on_lio_odometry(w.Odom(Ms(2'000)), Snap(Ms(100'000)));
  rig.bridge.on_lio_odometry(w.Odom(Ms(2'001)), Snap(Ms(100'000)));
  rig.bridge.flush(Snap(Ms(100'000)));
  gated = rig.Events("EvGated");
  ASSERT_EQ(gated.size(), 3U);
  EXPECT_DOUBLE_EQ(Value(gated[2], "count"), 1002.0);
  EXPECT_DOUBLE_EQ(Value(gated[2], "suppressed"), 2.0);
  EXPECT_DOUBLE_EQ(Value(gated[2], "summary"), 1.0);
  EXPECT_EQ(gated[2].reason, "HEALTH_NOT_TRACKING");
  rig.bridge.flush(Snap(Ms(100'000)));  // nothing open: no record
  EXPECT_EQ(rig.Events("EvGated").size(), 3U);
}

// Two separate fault episodes on one stream are both reported: an accepted sample ends the first (with a summary
// of what it suppressed) and re-arms the slot.
TEST(Px4Bridge, EachFaultEpisodeIsReported) {
  Rig rig;
  World w;
  Px4LocalPositionIn bad = w.Px4(Ms(1'000));
  bad.xy_valid = false;
  rig.bridge.on_px4_local_position(bad, Snap(Ms(1'000)));
  bad = w.Px4(Ms(1'020));
  bad.xy_valid = false;
  rig.bridge.on_px4_local_position(bad, Snap(Ms(1'020)));
  rig.bridge.on_px4_local_position(w.Px4(Ms(1'040)), Snap(Ms(1'040)));  // accepted: the episode ends
  rig.bridge.on_px4_local_position(w.Px4(Ms(1'060)), Snap(Ms(1'060)));
  bad = w.Px4(Ms(1'080));
  bad.xy_valid = false;
  rig.bridge.on_px4_local_position(bad, Snap(Ms(1'080)));  // a new episode: reported at once

  const auto rejected = rig.Events("Px4SampleRejected");
  ASSERT_EQ(rejected.size(), 3U);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "count"), 1.0);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), 1.0);
  EXPECT_DOUBLE_EQ(Value(rejected[1], "count"), 2.0);  // the summary of the first episode
  EXPECT_DOUBLE_EQ(Value(rejected[1], "suppressed"), 1.0);
  EXPECT_DOUBLE_EQ(Value(rejected[1], "summary"), 1.0);
  EXPECT_EQ(rejected[1].reason, "PX4_POSITION_INVALID");
  EXPECT_DOUBLE_EQ(Value(rejected[2], "count"), 1.0);
  EXPECT_DOUBLE_EQ(Value(rejected[2], "sensor_time_s"), 1.08);
  EXPECT_TRUE(std::isnan(Value(rejected[2], "summary")));
}

// The routine LIO kInputRejected of a DEGRADED -> TRACKING edge (odometry stamped before the DEGRADED health fed to
// the alignment) is its own slot: it does not silence a later PX4 stamp regression.
TEST(Px4Bridge, LioRejectionsDoNotSilencePx4Rejections) {
  Rig rig;
  World w;
  RunToValid(rig, w);
  ASSERT_EQ(rig.bridge.on_lio_health(w.Health(Ms(3'560), LioHealthCode::kDegraded), Snap(Ms(3'560)))
                .alignment->state,
            AlignmentState::kFrozen);
  EXPECT_FALSE(rig.bridge.on_lio_health(w.Health(Ms(3'570), LioHealthCode::kTracking), Snap(Ms(3'570))).alignment);
  for (const std::int64_t t : {3'553, 3'555}) {
    const BridgeOutputs out = rig.bridge.on_lio_odometry(w.Odom(Ms(t)), Snap(Ms(3'575)));
    EXPECT_TRUE(out.ev);  // not gated: the newest health is TRACKING
    ASSERT_TRUE(out.alignment);
    EXPECT_EQ(out.alignment->reason, AlignmentReason::kInputRejected);  // before the DEGRADED health's stamp
  }
  const BridgeOutputs px4 = rig.bridge.on_px4_local_position(w.Px4(Ms(3'400)), Snap(Ms(3'580)));
  ASSERT_TRUE(px4.alignment);
  EXPECT_EQ(px4.alignment->reason, AlignmentReason::kInputRejected);

  const auto rejected = rig.Events("AlignmentPairRejected");
  ASSERT_EQ(rejected.size(), 2U);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "input"), 0.0);
  EXPECT_DOUBLE_EQ(Value(rejected[1], "input"), 2.0);
  EXPECT_EQ(rejected[1].reason, "INPUT_REJECTED");
  EXPECT_TRUE(std::isnan(Value(rejected[1], "residual_position_m")));  // a PX4 sample forms no pair
}

// A health stamped before the newest health of the same epoch is refused and changes neither the gate nor T.
TEST(Px4Bridge, OutOfOrderHealthIsRefused) {
  Rig rig;
  World w;
  EXPECT_FALSE(rig.bridge.on_lio_health(w.Health(Ms(10'000), LioHealthCode::kTracking), Snap(Ms(10'000))).alignment);
  const BridgeOutputs out = rig.bridge.on_lio_health(w.Health(Ms(9'900), LioHealthCode::kLost), Snap(Ms(10'010)));
  EXPECT_FALSE(out.alignment);  // not fed to the alignment
  EXPECT_TRUE(rig.bridge.on_lio_odometry(w.Odom(Ms(9'800)), Snap(Ms(10'020))).ev);  // the gate still says TRACKING
  const auto rejected = rig.Events("LioInputRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "OUT_OF_ORDER");
  EXPECT_DOUBLE_EQ(Value(rejected[0], "stream"), 1.0);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "newest_health_s"), 10.0);
  // Another epoch is not compared.
  EXPECT_TRUE(rig.bridge.on_lio_health(LioHealthIn{Ms(9'950), LioHealthCode::kLost, 2}, Snap(Ms(10'030))).alignment);
}

// A non-TRACKING health in INIT throws the accumulation away: one event per episode, re-armed by an accepted pair.
TEST(Px4Bridge, InitAccumulationRestartIsReported) {
  Rig rig;
  World w;
  Stream(rig, w, 1'000, 1'600);  // 6 pairs accumulated, still INIT
  ASSERT_EQ(rig.bridge.alignment_state(), AlignmentState::kInit);
  for (const std::int64_t t : {1'600, 1'610, 1'620}) {
    const BridgeOutputs out = rig.bridge.on_lio_health(w.Health(Ms(t), LioHealthCode::kDegraded), Snap(Ms(t)));
    ASSERT_TRUE(out.alignment);
    EXPECT_EQ(out.alignment->reason, AlignmentReason::kLioLost);
  }
  auto restarts = rig.Events("AlignmentAccumulationRestarted");
  ASSERT_EQ(restarts.size(), 1U);
  EXPECT_EQ(restarts[0].reason, "LIO_LOST");
  EXPECT_DOUBLE_EQ(Value(restarts[0], "input"), 1.0);
  EXPECT_FALSE(rig.bridge.on_lio_health(w.Health(Ms(1'630), LioHealthCode::kTracking), Snap(Ms(1'630))).alignment);
  Stream(rig, w, 1'640, 1'760);  // the 1.603 s scan is older than the DEGRADED health; 1.703 s is accepted
  restarts = rig.Events("AlignmentAccumulationRestarted");
  ASSERT_EQ(restarts.size(), 2U);
  EXPECT_DOUBLE_EQ(Value(restarts[1], "count"), 3.0);
  EXPECT_DOUBLE_EQ(Value(restarts[1], "suppressed"), 2.0);
}

TEST(Px4Bridge, LioLostStopsEvAndFreezesAlignment) {  // Review Focus 3
  Rig rig;
  World w;
  RunToValid(rig, w);

  // LOST, stamped at the newest IMU sample (after the last scan): FROZEN in that call.
  const time::SensorTime lost_at = Ms(3'610);
  const BridgeOutputs lost = rig.bridge.on_lio_health(w.Health(lost_at, LioHealthCode::kLost), Snap(lost_at));
  ASSERT_TRUE(lost.alignment);
  EXPECT_EQ(lost.alignment->state, AlignmentState::kFrozen);
  EXPECT_EQ(lost.alignment->reason, AlignmentReason::kLioLost);
  auto transitions = rig.Events("AlignmentTransition");
  ASSERT_EQ(transitions.size(), 2U);  // INIT -> VALID, VALID -> FROZEN
  EXPECT_EQ(transitions[1].state_before, "VALID");
  EXPECT_EQ(transitions[1].state_after, "FROZEN");
  EXPECT_EQ(transitions[1].reason, "LIO_LOST");
  EXPECT_DOUBLE_EQ(Value(transitions[1], "input"), 1.0);

  // PX4 keeps flying at 50 Hz; LIO repeats LOST every 100 ms; odometry of that epoch still in flight (stamped
  // before the newest LOST health) arrives after it. No EV, ever; FROZEN, then INVALID by kFrozenTooLong.
  std::size_t ev_count = 0;
  std::size_t gated_odometry = 0;
  for (std::int64_t m = 3'620; m < 14'200; ++m) {
    const time::SensorTime now = Ms(m);
    std::vector<AlignmentOutput> outs;
    auto take = [&](const BridgeOutputs& o) {
      if (o.ev) ++ev_count;
      if (o.alignment) outs.push_back(*o.alignment);
    };
    if (m % 100 == 0) take(rig.bridge.on_tick(now, Snap(now)));
    if (m % 20 == 0) take(rig.bridge.on_px4_local_position(w.Px4(now), Snap(now)));
    if (m % 100 == 10) take(rig.bridge.on_lio_health(w.Health(now, LioHealthCode::kLost), Snap(now)));
    if (m % 100 == 11) {
      const BridgeOutputs o = rig.bridge.on_lio_odometry(w.Odom(Ms(m - 60)), Snap(now));
      EXPECT_FALSE(o.alignment);  // gated: not fed
      ++gated_odometry;
      take(o);
    }
    for (const AlignmentOutput& o : outs) {
      EXPECT_NE(o.reason, AlignmentReason::kStale);
      EXPECT_NE(o.state, AlignmentState::kValid);
      if (m <= lost_at.ns / 1'000'000 + 10'000) {
        EXPECT_EQ(o.state, AlignmentState::kFrozen) << m;
      }
    }
  }
  EXPECT_EQ(ev_count, 0U);
  EXPECT_GT(gated_odometry, 100U);
  EXPECT_EQ(rig.bridge.alignment_state(), AlignmentState::kInvalid);
  transitions = rig.Events("AlignmentTransition");
  ASSERT_EQ(transitions.size(), 3U);
  EXPECT_EQ(transitions[2].state_before, "FROZEN");
  EXPECT_EQ(transitions[2].state_after, "INVALID");
  EXPECT_EQ(transitions[2].reason, "FROZEN_TOO_LONG");
  for (const auto& r : transitions) EXPECT_NE(r.reason, "STALE");
}

TEST(Px4Bridge, DoubleResetDuringHoverKeepsTheVehiclePointContinuous) {  // Review Focus 2
  Rig rig;
  World w;
  w.v_flu.setZero();
  w.p0_flu = Eigen::Vector3d{2.0, -1.0, 3.0};  // p_lio FRD = (2, 1, -3)
  const auto steps = RunToValid(rig, w);
  const Pose4 before = LastOdometryAlignment(steps).filtered;
  ExpectPoseNear(before, w.truth, 1e-9);

  // One sample: xy counter +2 (delta (1, 0)), heading counter +1 (dh 0.1). The vehicle position just before
  // the reset is the pivot; EKF2 keeps it continuous across the heading reset and adds the xy delta.
  const Eigen::Vector3d p_lio = w.PFrd(Ms(3'600));
  const Eigen::Vector3d pivot = w.PNed(Ms(3'600));
  const Eigen::Vector3d d{1.0, 0.0, 0.0};
  constexpr double kDh = 0.1;
  w.truth = FrameChange(w.truth, pivot, d, kDh);
  w.xy_counter = 2;
  w.heading_counter = 1;
  w.delta_xy = d.head<2>();
  w.delta_heading = kDh;
  const Px4LocalPositionIn reset = w.Px4(Ms(3'600));
  ASSERT_LT((reset.sample.p_ned_m - (pivot + d)).norm(), 1e-12);

  const BridgeOutputs out = rig.bridge.on_px4_local_position(reset, Snap(Ms(3'600)));
  ASSERT_TRUE(out.alignment);
  EXPECT_EQ(out.alignment->state, AlignmentState::kValid);
  EXPECT_EQ(out.alignment->reason, AlignmentReason::kPx4ResetApplied);
  ExpectPoseNear(out.alignment->filtered, FrameChange(before, pivot, d, kDh), 1e-9);
  // The vehicle point moves with the frame: T'(p_lio) is the new PX4 position.
  EXPECT_LT((Apply(out.alignment->filtered, p_lio) - reset.sample.p_ned_m).norm(), 1e-9);

  const auto resets = rig.Events("Px4ResetApplied");
  ASSERT_EQ(resets.size(), 1U);
  EXPECT_EQ(resets[0].reason, "PX4_RESET_APPLIED");
  EXPECT_DOUBLE_EQ(Value(resets[0], "xy_reset_counter"), 2.0);
  EXPECT_DOUBLE_EQ(Value(resets[0], "heading_reset_counter"), 1.0);
  EXPECT_DOUBLE_EQ(Value(resets[0], "xy_reset_step"), 2.0);
  EXPECT_DOUBLE_EQ(Value(resets[0], "heading_reset_step"), 1.0);
  EXPECT_DOUBLE_EQ(Value(resets[0], "z_reset_step"), 0.0);
  EXPECT_DOUBLE_EQ(Value(resets[0], "delta_x_m"), 1.0);
  EXPECT_DOUBLE_EQ(Value(resets[0], "delta_heading_rad"), kDh);

  // The next 50 pairs, in the new frame, are all accepted.
  const auto after = Stream(rig, w, 3'601, 8'600);
  int pairs = 0;
  for (const Step& s : after) {
    if (s.kind != Kind::kOdometry) continue;
    ASSERT_TRUE(s.out.alignment);
    EXPECT_EQ(s.out.alignment->reason, AlignmentReason::kPairAccepted) << s.t.ns;
    ++pairs;
  }
  EXPECT_EQ(pairs, 50);
  EXPECT_EQ(rig.bridge.alignment_state(), AlignmentState::kValid);
}

TEST(Px4Bridge, InvalidPx4PositionIsSkippedWithAnEvent) {
  Rig rig;
  World w;
  RunToValid(rig, w);

  // xy_valid false and a garbage position: not buffered. The scan stamped exactly at it pairs with the valid
  // neighbours (buffered, an exact stamp match would use the garbage sample and jump-reject the pair).
  Px4LocalPositionIn bad = w.Px4(Ms(3'600));
  bad.sample.p_ned_m.x() += 100.0;
  bad.xy_valid = false;
  BridgeOutputs out = rig.bridge.on_px4_local_position(bad, Snap(Ms(3'600)));
  EXPECT_FALSE(out.alignment);
  EXPECT_FALSE(out.ev);
  auto rejected = rig.Events("Px4SampleRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "PX4_POSITION_INVALID");
  EXPECT_DOUBLE_EQ(Value(rejected[0], "xy_valid"), 0.0);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "z_valid"), 1.0);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), 3.6);
  ASSERT_TRUE(rig.bridge.on_px4_local_position(w.Px4(Ms(3'620)), Snap(Ms(3'620))).alignment == std::nullopt);
  out = rig.bridge.on_lio_odometry(w.Odom(Ms(3'600)), Snap(Ms(3'630)));
  ASSERT_TRUE(out.alignment);
  EXPECT_EQ(out.alignment->reason, AlignmentReason::kPairAccepted);
  const Pose4 T = out.alignment->filtered;

  // An invalid sample carries an xy reset (counter +1, delta (0.5, 0)): skipped, its counter not consumed; the
  // next valid sample (same counter and delta, PX4 keeps them) applies it.
  const Eigen::Vector3d d{0.5, 0.0, 0.0};
  w.truth = FrameChange(w.truth, w.PNed(Ms(3'640)), d, 0.0);
  w.xy_counter = 1;
  w.delta_xy = d.head<2>();
  bad = w.Px4(Ms(3'640));
  bad.z_valid = false;
  EXPECT_FALSE(rig.bridge.on_px4_local_position(bad, Snap(Ms(3'640))).alignment);
  EXPECT_TRUE(rig.Events("Px4ResetApplied").empty());
  out = rig.bridge.on_px4_local_position(w.Px4(Ms(3'660)), Snap(Ms(3'660)));
  ASSERT_TRUE(out.alignment);
  EXPECT_EQ(out.alignment->reason, AlignmentReason::kPx4ResetApplied);
  EXPECT_NEAR(out.alignment->filtered.x_m, T.x_m + 0.5, 1e-12);
  EXPECT_NEAR(out.alignment->filtered.y_m, T.y_m, 1e-12);
  EXPECT_NEAR(out.alignment->filtered.yaw_rad, T.yaw_rad, 1e-12);
  const auto resets = rig.Events("Px4ResetApplied");
  ASSERT_EQ(resets.size(), 1U);
  EXPECT_DOUBLE_EQ(Value(resets[0], "xy_reset_step"), 1.0);
  EXPECT_DOUBLE_EQ(Value(resets[0], "delta_x_m"), 0.5);
}

TEST(Px4Bridge, GarbageInputsAreRejectedWithEvents) {  // Review Focus 5
  Rig rig;
  World w;
  RunToValid(rig, w);  // the newest tick is at 3.5 s
  const auto valid_state = [&] { EXPECT_EQ(rig.bridge.alignment_state(), AlignmentState::kValid); };

  // NaN odometry: EvRejected kNonFiniteState; the alignment refuses it (no pair).
  LioOdometryIn nan_odom = w.Odom(Ms(3'603));
  nan_odom.p_flu_m.x() = std::numeric_limits<double>::quiet_NaN();
  BridgeOutputs out = rig.bridge.on_lio_odometry(nan_odom, Snap(Ms(3'653)));
  EXPECT_FALSE(out.ev);
  ASSERT_TRUE(out.alignment);
  EXPECT_EQ(out.alignment->reason, AlignmentReason::kInputRejected);
  EXPECT_FALSE(out.alignment->raw);
  valid_state();
  const auto ev_rejected = rig.Events("EvRejected");
  ASSERT_EQ(ev_rejected.size(), 1U);
  EXPECT_EQ(ev_rejected[0].reason, "NON_FINITE_STATE");
  auto pair_rejected = rig.Events("AlignmentPairRejected");
  ASSERT_EQ(pair_rejected.size(), 1U);
  EXPECT_EQ(pair_rejected[0].reason, "INPUT_REJECTED");
  EXPECT_DOUBLE_EQ(Value(pair_rejected[0], "input"), 0.0);

  // Stamps 0.6 s after the newest tick (3.5 s): refused on each input before anything else, nothing changes.
  const time::SensorTime future = Ms(4'100);
  const time::TimeSnapshot now = Snap(Ms(3'560));
  out = rig.bridge.on_lio_odometry(w.Odom(future), now);
  EXPECT_FALSE(out.ev || out.alignment);
  out = rig.bridge.on_lio_health(w.Health(future, LioHealthCode::kLost), now);  // would freeze T if accepted
  EXPECT_FALSE(out.ev || out.alignment);
  out = rig.bridge.on_px4_local_position(w.Px4(future), now);
  EXPECT_FALSE(out.ev || out.alignment);
  valid_state();
  const auto lio_rejected = rig.Events("LioInputRejected");
  ASSERT_EQ(lio_rejected.size(), 2U);  // one slot per stream
  for (std::size_t k = 0; k < 2; ++k) {
    EXPECT_EQ(lio_rejected[k].reason, "STAMP_IN_FUTURE");
    EXPECT_DOUBLE_EQ(Value(lio_rejected[k], "stream"), static_cast<double>(k));  // 0 odometry, 1 health
    EXPECT_DOUBLE_EQ(Value(lio_rejected[k], "newest_tick_s"), 3.5);
  }
  const auto px4_rejected = rig.Events("Px4SampleRejected");
  ASSERT_EQ(px4_rejected.size(), 1U);
  EXPECT_EQ(px4_rejected[0].reason, "STAMP_IN_FUTURE");
  // Exactly at the bound is not ahead.
  EXPECT_FALSE(rig.bridge.on_lio_health(w.Health(Ms(4'000), LioHealthCode::kTracking), now).alignment);
  EXPECT_EQ(rig.Events("LioInputRejected").size(), 2U);

  // A regressing PX4 stamp: the alignment refuses it (kInputRejected), with an event.
  out = rig.bridge.on_px4_local_position(w.Px4(Ms(3'400)), now);
  ASSERT_TRUE(out.alignment);
  EXPECT_EQ(out.alignment->reason, AlignmentReason::kInputRejected);
  pair_rejected = rig.Events("AlignmentPairRejected");
  ASSERT_EQ(pair_rejected.size(), 2U);  // its own slot (PX4 stream), not silenced by the odometry one
  EXPECT_EQ(pair_rejected[1].reason, "INPUT_REJECTED");
  EXPECT_DOUBLE_EQ(Value(pair_rejected[1], "input"), 2.0);
  // A non-finite PX4 heading: refused the same way (same episode: counted).
  Px4LocalPositionIn nan_px4 = w.Px4(Ms(3'580));
  nan_px4.sample.yaw_rad = std::numeric_limits<double>::infinity();
  out = rig.bridge.on_px4_local_position(nan_px4, now);
  ASSERT_TRUE(out.alignment);
  EXPECT_EQ(out.alignment->reason, AlignmentReason::kInputRejected);
  EXPECT_EQ(rig.Events("AlignmentPairRejected").size(), 2U);
  // The next accepted PX4 sample ends the episode: one summary, the suppressed refusal is not lost.
  ASSERT_TRUE(rig.bridge.on_px4_local_position(w.Px4(Ms(3'600)), now).alignment == std::nullopt);
  pair_rejected = rig.Events("AlignmentPairRejected");
  ASSERT_EQ(pair_rejected.size(), 3U);
  EXPECT_DOUBLE_EQ(Value(pair_rejected[2], "input"), 2.0);
  EXPECT_DOUBLE_EQ(Value(pair_rejected[2], "count"), 2.0);
  EXPECT_DOUBLE_EQ(Value(pair_rejected[2], "suppressed"), 1.0);

  valid_state();
  EXPECT_EQ(rig.Events("AlignmentTransition").size(), 1U);  // INIT -> VALID only
}

TEST(Px4Bridge, TickDrivesStaleTimeout) {
  Rig rig;
  World w;
  RunToValid(rig, w);  // last accepted pair at 3.503 s
  const auto steps = Stream(rig, w, 3'600, 4'700, /*odometry=*/false);
  std::optional<Step> invalid;
  for (const Step& s : steps) {
    if (s.kind == Kind::kPx4) continue;
    ASSERT_TRUE(s.out.alignment);  // every tick output is published (O21)
    if (s.out.alignment->state == AlignmentState::kInvalid && !invalid) invalid = s;
    if (!invalid) {
      EXPECT_EQ(s.out.alignment->state, AlignmentState::kValid);
    }
  }
  ASSERT_TRUE(invalid);
  EXPECT_EQ(invalid->kind, Kind::kTick);
  EXPECT_EQ(invalid->t, Ms(4'600));  // the first tick more than 1 s after 3.503 s
  EXPECT_EQ(invalid->out.alignment->reason, AlignmentReason::kStale);
  const auto transitions = rig.Events("AlignmentTransition");
  ASSERT_EQ(transitions.size(), 2U);
  EXPECT_EQ(transitions[1].state_before, "VALID");
  EXPECT_EQ(transitions[1].state_after, "INVALID");
  EXPECT_EQ(transitions[1].reason, "STALE");
  EXPECT_DOUBLE_EQ(Value(transitions[1], "input"), 3.0);
}

// The FROZEN -> VALID edge whose returning pair is jump-rejected reports kPairRejectedJump, not kLioTracking
// (alignment.hpp on_lio step 5): the transition event is keyed on the state change, so it is still emitted.
TEST(Px4Bridge, FrozenToValidWithAJumpRejectedPairIsOneTransition) {
  Rig rig;
  World w;
  RunToValid(rig, w);
  ASSERT_EQ(rig.bridge.on_lio_health(w.Health(Ms(3'560), LioHealthCode::kLost), Snap(Ms(3'560))).alignment->state,
            AlignmentState::kFrozen);
  rig.bridge.on_px4_local_position(w.Px4(Ms(3'600)), Snap(Ms(3'600)));
  rig.bridge.on_px4_local_position(w.Px4(Ms(3'620)), Snap(Ms(3'620)));
  LioOdometryIn jumped = w.Odom(Ms(3'603));
  jumped.p_flu_m.x() += 2.0;
  const BridgeOutputs out = rig.bridge.on_lio_odometry(jumped, Snap(Ms(3'653)));
  ASSERT_TRUE(out.alignment);
  EXPECT_EQ(out.alignment->state, AlignmentState::kValid);
  EXPECT_EQ(out.alignment->reason, AlignmentReason::kPairRejectedJump);
  const auto transitions = rig.Events("AlignmentTransition");
  ASSERT_EQ(transitions.size(), 3U);
  EXPECT_EQ(transitions[2].state_before, "FROZEN");
  EXPECT_EQ(transitions[2].state_after, "VALID");
  EXPECT_EQ(transitions[2].reason, "PAIR_REJECTED_JUMP");
  const auto rejected = rig.Events("AlignmentPairRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "PAIR_REJECTED_JUMP");
  EXPECT_GT(Value(rejected[0], "residual_position_m"), 1.9);
}

// DV2: an odometry delivered before the PX4 sample after its stamp (LIO latency 20 ms < PX4 interval + lag in SITL)
// waits; the next PX4 sample pairs it. No rejection event, the pair is published from the PX4 call.
TEST(Px4Bridge, OdometryAheadOfPx4IsPairedByTheNextPx4Sample) {
  Rig rig;
  World w;
  RunToValid(rig, w);  // PX4 up to 3.580 s
  rig.bridge.on_tick(Ms(3'600), Snap(Ms(3'600)));
  rig.bridge.on_px4_local_position(w.Px4(Ms(3'600)), Snap(Ms(3'600)));
  const BridgeOutputs wait = rig.bridge.on_lio_odometry(w.Odom(Ms(3'603)), Snap(Ms(3'610)));
  EXPECT_TRUE(wait.ev);  // the EV never waits (D42)
  ASSERT_TRUE(wait.alignment);
  EXPECT_EQ(wait.alignment->reason, AlignmentReason::kNone);
  EXPECT_FALSE(wait.alignment->raw.has_value());
  const BridgeOutputs paired = rig.bridge.on_px4_local_position(w.Px4(Ms(3'620)), Snap(Ms(3'620)));
  ASSERT_TRUE(paired.alignment);  // published: on_px4 output with a reason (O21)
  EXPECT_EQ(paired.alignment->reason, AlignmentReason::kPairAccepted);
  EXPECT_EQ(paired.alignment->stamp, Ms(3'603));
  ASSERT_TRUE(paired.alignment->raw.has_value());
  ExpectPoseNear(*paired.alignment->raw, w.truth, 1e-9);
  EXPECT_TRUE(rig.Events("AlignmentPairRejected").empty());
}

// A waiting odometry that the next PX4 sample cannot pair (beyond kPairingWindow) is reported as the odometry's
// NO_PX4_SAMPLE, with the scan's stamp.
TEST(Px4Bridge, WaitingOdometryWithoutAPx4SampleInTheWindowIsReported) {
  Rig rig;
  World w;
  RunToValid(rig, w);
  rig.bridge.on_tick(Ms(3'600), Snap(Ms(3'600)));
  rig.bridge.on_px4_local_position(w.Px4(Ms(3'600)), Snap(Ms(3'600)));
  rig.bridge.on_lio_odometry(w.Odom(Ms(3'603)), Snap(Ms(3'610)));
  const BridgeOutputs late = rig.bridge.on_px4_local_position(w.Px4(Ms(3'660)), Snap(Ms(3'660)));  // 57 ms after
  ASSERT_TRUE(late.alignment);
  EXPECT_EQ(late.alignment->reason, AlignmentReason::kNoPx4Sample);
  const auto rejected = rig.Events("AlignmentPairRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "NO_PX4_SAMPLE");
  EXPECT_DOUBLE_EQ(Value(rejected[0], "input"), 0.0);  // the odometry's pair, decided at a PX4 sample
  EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), 3.603);
}

// A waiting odometry dropped by the next one (no PX4 sample in between) is reported with its own stamp.
TEST(Px4Bridge, WaitingOdometryDroppedByANewerOneIsReportedWithItsStamp) {
  Rig rig;
  World w;
  RunToValid(rig, w);
  rig.bridge.on_tick(Ms(3'600), Snap(Ms(3'600)));
  rig.bridge.on_px4_local_position(w.Px4(Ms(3'600)), Snap(Ms(3'600)));
  rig.bridge.on_lio_odometry(w.Odom(Ms(3'603)), Snap(Ms(3'610)));
  // A newer scan, also within kPairingWindow of the newest PX4 sample: it waits, the older one is dropped.
  const BridgeOutputs next = rig.bridge.on_lio_odometry(w.Odom(Ms(3'610)), Snap(Ms(3'612)));
  ASSERT_TRUE(next.alignment);
  EXPECT_EQ(next.alignment->reason, AlignmentReason::kNoPx4Sample);
  const auto rejected = rig.Events("AlignmentPairRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected[0].reason, "NO_PX4_SAMPLE");
  EXPECT_DOUBLE_EQ(Value(rejected[0], "input"), 0.0);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), 3.603);
  // The newer one is paired by the next PX4 sample after it; the accepted pair ends the episode (no suppressed
  // occurrence, so no summary record).
  const BridgeOutputs paired = rig.bridge.on_px4_local_position(w.Px4(Ms(3'620)), Snap(Ms(3'624)));
  ASSERT_TRUE(paired.alignment);
  EXPECT_EQ(paired.alignment->reason, AlignmentReason::kPairAccepted);
  EXPECT_EQ(paired.alignment->stamp, Ms(3'610));
  EXPECT_EQ(rig.Events("AlignmentPairRejected").size(), 1U);
}

// A persistent EV fault (bad covariance, finite pose) while scans wait: the pair of a waiting scan accepted at a PX4
// sample ends the odometry episodes only if that scan's own EV was encoded, so the EvRejected episode stays one
// record (no summary and re-report at count 1 on every waiting scan).
TEST(Px4Bridge, PairDecidedAtAPx4SampleDoesNotEndTheEvRejectedEpisodeOfItsScan) {
  Rig rig;
  World w;
  RunToValid(rig, w);
  auto bad = [&](std::int64_t stamp_ms) {
    LioOdometryIn o = w.Odom(Ms(stamp_ms));
    o.pose_cov = -1e-4 * Eigen::Matrix<double, 6, 6>::Identity();
    return o;
  };
  rig.bridge.on_tick(Ms(3'600), Snap(Ms(3'600)));
  for (std::int64_t m = 3'600; m <= 3'620; m += 20) rig.bridge.on_px4_local_position(w.Px4(Ms(m)), Snap(Ms(m)));
  // 1st: bracketed at once (paired), EV rejected: the episode opens.
  BridgeOutputs out = rig.bridge.on_lio_odometry(bad(3'603), Snap(Ms(3'625)));
  ASSERT_FALSE(out.ev);
  ASSERT_EQ(out.alignment->reason, AlignmentReason::kPairAccepted);
  // 2nd: waits; its pair is accepted at the next PX4 sample.
  for (std::int64_t m = 3'640; m <= 3'700; m += 20) rig.bridge.on_px4_local_position(w.Px4(Ms(m)), Snap(Ms(m)));
  out = rig.bridge.on_lio_odometry(bad(3'703), Snap(Ms(3'705)));
  ASSERT_FALSE(out.ev);
  ASSERT_EQ(out.alignment->reason, AlignmentReason::kNone);
  const BridgeOutputs decided = rig.bridge.on_px4_local_position(w.Px4(Ms(3'720)), Snap(Ms(3'720)));
  ASSERT_EQ(decided.alignment->reason, AlignmentReason::kPairAccepted);
  ASSERT_EQ(decided.alignment->stamp, Ms(3'703));
  // 3rd: the fault goes on.
  for (std::int64_t m = 3'740; m <= 3'820; m += 20) rig.bridge.on_px4_local_position(w.Px4(Ms(m)), Snap(Ms(m)));
  ASSERT_FALSE(rig.bridge.on_lio_odometry(bad(3'803), Snap(Ms(3'825))).ev);
  const auto rejected = rig.Events("EvRejected");
  ASSERT_EQ(rejected.size(), 1U);  // one episode, still open
  EXPECT_DOUBLE_EQ(Value(rejected[0], "count"), 1.0);
  EXPECT_EQ(rejected[0].reason, "BAD_COVARIANCE");
}

// A waiting scan is not an accepted odometry: an open odometry pair-rejection episode stays open until the scan's
// pair is decided, so a NO_PX4_SAMPLE decided at a PX4 sample is counted in it (one record, not two).
TEST(Px4Bridge, WaitingOdometryDoesNotEndTheOdometryEpisodeAtOnce) {
  Rig rig;
  World w;
  RunToValid(rig, w);
  rig.bridge.on_tick(Ms(3'600), Snap(Ms(3'600)));
  rig.bridge.on_px4_local_position(w.Px4(Ms(3'600)), Snap(Ms(3'600)));
  // 100 ms ahead of the newest PX4 sample: beyond kPairingWindow, refused at once. The episode opens.
  ASSERT_EQ(rig.bridge.on_lio_odometry(w.Odom(Ms(3'700)), Snap(Ms(3'610))).alignment->reason,
            AlignmentReason::kNoPx4Sample);
  for (std::int64_t m = 3'620; m <= 3'700; m += 20) rig.bridge.on_px4_local_position(w.Px4(Ms(m)), Snap(Ms(m)));
  ASSERT_EQ(rig.bridge.on_lio_odometry(w.Odom(Ms(3'703)), Snap(Ms(3'705))).alignment->reason, AlignmentReason::kNone);
  ASSERT_EQ(rig.bridge.on_px4_local_position(w.Px4(Ms(3'760)), Snap(Ms(3'760))).alignment->reason,
            AlignmentReason::kNoPx4Sample);  // 57 ms after the scan
  const auto rejected = rig.Events("AlignmentPairRejected");
  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), 3.7);
  // flush() closes the episode: both occurrences are in it.
  rig.bridge.flush(Snap(Ms(3'800)));
  const auto all = rig.Events("AlignmentPairRejected");
  ASSERT_EQ(all.size(), 2U);
  EXPECT_DOUBLE_EQ(Value(all[1], "count"), 2.0);
  EXPECT_DOUBLE_EQ(Value(all[1], "summary"), 1.0);
}

// A waiting scan that leaves without a decision (a newer scan paired at once, a state change) is reported as its
// odometry's NO_PX4_SAMPLE with its own stamp: never a silent loss.
TEST(Px4Bridge, WaitingOdometryDroppedWithoutADecisionIsReported) {
  {
    // Dropped by a newer scan that pairs at once: a reset sample after both brackets the newer one.
    Rig rig;
    World w;
    RunToValid(rig, w);
    rig.bridge.on_tick(Ms(3'600), Snap(Ms(3'600)));
    rig.bridge.on_px4_local_position(w.Px4(Ms(3'600)), Snap(Ms(3'600)));
    ASSERT_EQ(rig.bridge.on_lio_odometry(w.Odom(Ms(3'603)), Snap(Ms(3'605))).alignment->reason,
              AlignmentReason::kNone);
    const Eigen::Vector3d d{0.2, 0.0, 0.0};
    w.truth = FrameChange(w.truth, w.PNed(Ms(3'620)), d, 0.0);
    w.xy_counter = 1;
    w.delta_xy = d.head<2>();
    ASSERT_EQ(rig.bridge.on_px4_local_position(w.Px4(Ms(3'620)), Snap(Ms(3'620))).alignment->reason,
              AlignmentReason::kPx4ResetApplied);
    const BridgeOutputs paired = rig.bridge.on_lio_odometry(w.Odom(Ms(3'610)), Snap(Ms(3'622)));
    ASSERT_EQ(paired.alignment->reason, AlignmentReason::kPairAccepted);
    const auto rejected = rig.Events("AlignmentPairRejected");
    ASSERT_EQ(rejected.size(), 1U);
    EXPECT_EQ(rejected[0].reason, "NO_PX4_SAMPLE");
    EXPECT_DOUBLE_EQ(Value(rejected[0], "input"), 0.0);
    EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), 3.603);
  }
  {
    // Dropped by a state change: a DEGRADED health freezes T while the scan waits.
    Rig rig;
    World w;
    RunToValid(rig, w);
    rig.bridge.on_tick(Ms(3'600), Snap(Ms(3'600)));
    rig.bridge.on_px4_local_position(w.Px4(Ms(3'600)), Snap(Ms(3'600)));
    ASSERT_EQ(rig.bridge.on_lio_odometry(w.Odom(Ms(3'603)), Snap(Ms(3'605))).alignment->reason,
              AlignmentReason::kNone);
    ASSERT_EQ(rig.bridge.on_lio_health(w.Health(Ms(3'608), LioHealthCode::kDegraded), Snap(Ms(3'608)))
                  .alignment->state,
              AlignmentState::kFrozen);
    const auto rejected = rig.Events("AlignmentPairRejected");
    ASSERT_EQ(rejected.size(), 1U);
    EXPECT_EQ(rejected[0].reason, "NO_PX4_SAMPLE");
    EXPECT_DOUBLE_EQ(Value(rejected[0], "input"), 0.0);
    EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), 3.603);
    EXPECT_EQ(rejected[0].state_after, "FROZEN");
  }
  {
    // Dropped by a state change at a tick: T goes STALE while the scan waits (no PX4 sample after it arrives).
    Rig rig;
    World w;
    RunToValid(rig, w);  // last accepted pair at 3.503 s
    rig.bridge.on_tick(Ms(3'600), Snap(Ms(3'600)));
    rig.bridge.on_px4_local_position(w.Px4(Ms(3'600)), Snap(Ms(3'600)));
    ASSERT_EQ(rig.bridge.on_lio_odometry(w.Odom(Ms(3'603)), Snap(Ms(3'605))).alignment->reason,
              AlignmentReason::kNone);
    ASSERT_EQ(rig.bridge.on_tick(Ms(4'600), Snap(Ms(4'600))).alignment->reason, AlignmentReason::kStale);
    const auto rejected = rig.Events("AlignmentPairRejected");
    ASSERT_EQ(rejected.size(), 1U);
    EXPECT_DOUBLE_EQ(Value(rejected[0], "sensor_time_s"), 3.603);
    EXPECT_EQ(rejected[0].state_after, "INVALID");
  }
}

TEST(Px4Bridge, EveryAlignmentStateChangeIsOneEvent) {
  // Short timeouts and few pairs so that 20000 inputs visit every state many times (5000 reached the rare FROZEN
  // edges only by luck of the seed once one odometry in three waits, DV2).
  const AlignmentConfig cfg{time::milliseconds(500), 3U, 0.5, 0.0873, time::milliseconds(300), time::seconds(1)};
  Rig rig(cfg);
  World w;
  std::mt19937 rng(20261013);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  std::uniform_int_distribution<int> step_ms(1, 25);

  struct Change {
    AlignmentState before, after;
  };
  std::vector<Change> changes;
  std::int64_t m = 1'000;
  std::int64_t last_px4 = 0;
  std::int64_t last_odom = 0;
  std::optional<LioHealthIn> latest_health;  // the newest health the bridge accepted (the gate's reference)
  std::optional<time::SensorTime> newest_tick;
  int px4_decided_pairs = 0;
  int px4_init_to_valid = 0;
  std::int64_t outage_until = 0;  // a LIO outage: no odometry, LOST health (so FROZEN can time out or see a reset)
  for (int i = 0; i < 20000; ++i) {
    m += step_ms(rng);
    const time::SensorTime now = Ms(m);
    if (u(rng) < 0.003) outage_until = m + 1'500;
    const bool outage = m < outage_until;
    const AlignmentState before = rig.bridge.alignment_state();
    const double r = u(rng);
    if (r < 0.35) {  // PX4 sample: resets, invalid flags, regressions, NaN
      std::int64_t t = m;
      if (u(rng) < 0.05) {
        const Eigen::Vector3d d{u(rng) - 0.5, u(rng) - 0.5, 0.0};
        const double dh = 0.2 * (u(rng) - 0.5);
        w.truth = FrameChange(w.truth, w.PNed(now), d, dh);
        w.xy_counter = static_cast<std::uint8_t>(w.xy_counter + 1 + (u(rng) < 0.5 ? 1 : 0));
        w.heading_counter = static_cast<std::uint8_t>(w.heading_counter + 1);
        w.delta_xy = d.head<2>();
        w.delta_heading = dh;
      }
      if (u(rng) < 0.03) t = last_px4 - 30;
      Px4LocalPositionIn s = w.Px4(Ms(t));
      if (u(rng) < 0.05) s.xy_valid = false;
      if (u(rng) < 0.02) s.sample.p_ned_m.z() = std::numeric_limits<double>::quiet_NaN();
      const BridgeOutputs po = rig.bridge.on_px4_local_position(s, Snap(now));
      if (po.alignment && po.alignment->raw) ++px4_decided_pairs;  // a waiting odometry decided here (DV2)
      if (before == AlignmentState::kInit && rig.bridge.alignment_state() == AlignmentState::kValid) {
        ++px4_init_to_valid;
      }
      last_px4 = std::max(last_px4, t);
    } else if (r < 0.55) {  // odometry: sometimes jumped, NaN, regressing, new epoch or from the future
      if (outage) continue;
      // Stamped just before the newest PX4 sample (so it can pair), or in one case of three just after it (so it
      // waits for the next one, DV2); increasing, sometimes regressing.
      std::int64_t t = std::min(m, std::max(last_odom + 1, last_px4 - static_cast<std::int64_t>(10 * u(rng))));
      if (u(rng) < 0.33) t = std::min(m, std::max(last_odom + 1, last_px4 + 1 + static_cast<std::int64_t>(10 * u(rng))));
      if (u(rng) < 0.03) t = last_odom - 20;
      last_odom = std::max(last_odom, t);
      LioOdometryIn o = w.Odom(Ms(t));
      if (u(rng) < 0.05) o.p_flu_m.y() += 3.0;
      if (u(rng) < 0.02) o.q_flu.coeffs().setZero();
      if (u(rng) < 0.01) ++w.epoch;
      if (u(rng) < 0.01) o.t = Ms(m + 2'000);
      o.epoch = w.epoch;
      const BridgeOutputs out = rig.bridge.on_lio_odometry(o, Snap(now));
      // Gate invariant (O22): no EV while the newest same-epoch health is not TRACKING and not older.
      if (out.ev && latest_health) {
        EXPECT_FALSE(latest_health->epoch == o.epoch && latest_health->state != LioHealthCode::kTracking &&
                     latest_health->t >= o.t)
            << i;
      }
    } else if (r < 0.75) {  // health: mostly TRACKING
      auto code = static_cast<LioHealthCode>(u(rng) < 0.75 ? 1 : static_cast<int>(5 * u(rng)));
      if (outage) code = LioHealthCode::kLost;
      // Stamps increase (never out of order); a health more than kMaxStampAhead after the newest tick is refused.
      if (!newest_tick || now <= *newest_tick + px4bridge::limits::kMaxStampAhead) latest_health = w.Health(now, code);
      rig.bridge.on_lio_health(w.Health(now, code), Snap(now));
    } else if (r < 0.9) {
      rig.bridge.on_tick(now, Snap(now));
      newest_tick = now;
    } else {
      m += 200;  // a gap
    }
    const AlignmentState after = rig.bridge.alignment_state();
    if (after != before) changes.push_back({before, after});
    if (i % 200 == 0) rig.recorder->flush();
  }

  const auto transitions = rig.Events("AlignmentTransition");
  ASSERT_EQ(transitions.size(), changes.size());
  for (std::size_t k = 0; k < changes.size(); ++k) {
    EXPECT_EQ(transitions[k].state_before, to_string(changes[k].before)) << k;
    EXPECT_EQ(transitions[k].state_after, to_string(changes[k].after)) << k;
  }
  // The fuzz really exercises the state machine: every listed edge, by its reason (FROZEN -> VALID also counts
  // with kPairRejectedJump, alignment.hpp on_lio step 5).
  EXPECT_GT(changes.size(), 100U);
  for (const AlignmentEdge& e : kAlignmentTransitions) {
    const bool seen = std::any_of(transitions.begin(), transitions.end(), [&](const events::EventRecord& r) {
      return r.state_before == to_string(e.from) && r.state_after == to_string(e.to) && r.reason == to_string(e.reason);
    });
    EXPECT_TRUE(seen) << to_string(e.from) << " -> " << to_string(e.to) << " " << to_string(e.reason);
  }
  EXPECT_EQ(rig.recorder->stats().dropped, 0U);
  // The waiting path is really fuzzed: pairs decided by a PX4 sample, and INIT -> VALID at a PX4 sample.
  EXPECT_GT(px4_decided_pairs, 0);
  EXPECT_GT(px4_init_to_valid, 0);
}
