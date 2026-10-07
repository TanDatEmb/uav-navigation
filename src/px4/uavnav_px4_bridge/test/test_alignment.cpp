#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <set>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

#include "uavnav/px4bridge/alignment.hpp"
#include "uavnav/px4bridge/limits.hpp"

using namespace uavnav;
using namespace uavnav::px4bridge;

namespace {

constexpr double kPi = std::numbers::pi;

double Wrap(double a) { return std::remainder(a, 2.0 * kPi); }

AlignmentConfig BetaConfig() {
  return AlignmentConfig{time::seconds(2), 20U, 0.5, 0.0873, time::seconds(1), time::seconds(10)};
}

time::SensorTime At(double s) { return time::SensorTime{std::llround(s * 1e9)}; }

void ExpectFinite(const AlignmentOutput& o) {
  EXPECT_TRUE(std::isfinite(o.filtered.x_m) && std::isfinite(o.filtered.y_m) && std::isfinite(o.filtered.z_m) &&
              std::isfinite(o.filtered.yaw_rad));
  EXPECT_TRUE(std::isfinite(o.age_s));
  EXPECT_TRUE(std::isfinite(o.residual_position_m));
  EXPECT_TRUE(std::isfinite(o.residual_yaw_rad));
  if (o.raw) {
    EXPECT_TRUE(std::isfinite(o.raw->x_m) && std::isfinite(o.raw->y_m) && std::isfinite(o.raw->z_m) &&
                std::isfinite(o.raw->yaw_rad));
  }
}

void ExpectPoseNear(const Pose4& a, const Pose4& b, double tol) {
  EXPECT_NEAR(a.x_m, b.x_m, tol);
  EXPECT_NEAR(a.y_m, b.y_m, tol);
  EXPECT_NEAR(a.z_m, b.z_m, tol);
  EXPECT_NEAR(Wrap(a.yaw_rad - b.yaw_rad), 0.0, tol);
}

void ExpectPoseEq(const Pose4& a, const Pose4& b) {
  EXPECT_EQ(a.x_m, b.x_m);
  EXPECT_EQ(a.y_m, b.y_m);
  EXPECT_EQ(a.z_m, b.z_m);
  EXPECT_EQ(a.yaw_rad, b.yaw_rad);
}

Eigen::Vector2d Rot(double yaw, const Eigen::Vector2d& v) {
  return {std::cos(yaw) * v.x() - std::sin(yaw) * v.y(), std::sin(yaw) * v.x() + std::cos(yaw) * v.y()};
}

// T(p) = (Rz(yaw_T) p_xy + t_xy, p_z + t_z): where T puts the lio_odom point p in PX4 local NED.
Eigen::Vector3d Apply(const Pose4& T, const Eigen::Vector3d& p) {
  const Eigen::Vector2d xy = Rot(T.yaw_rad, p.head<2>()) + Eigen::Vector2d{T.x_m, T.y_m};
  return {xy.x(), xy.y(), p.z() + T.z_m};
}

Eigen::Vector3d Translation(const Pose4& T) { return {T.x_m, T.y_m, T.z_m}; }

// Synthetic world. LIO moves with constant velocity in FRD lio_odom (so linear PX4 interpolation is exact),
// PX4 reports the same motion through the truth T (p_px4 = Rz(yaw_T) p_lio + t). PX4 runs at 100 Hz on a
// 10 ms grid, LIO at 10 Hz offset 3 ms from that grid so every pair is interpolated.
struct Sim {
  explicit Sim(AlignmentConfig cfg = BetaConfig()) : est(cfg) {}

  AlignmentEstimator est;
  Pose4 truth{3.0, -2.0, 0.5, 0.6};
  Eigen::Vector3d p0{1.0, 2.0, -3.0};
  Eigen::Vector3d v{0.8, -0.3, 0.05};
  double yaw0{0.2};
  double yaw_rate{0.05};
  double t_px4_next{100.0};  // next PX4 sample time [s]
  double t_lio_next{100.103};
  std::uint8_t xy_counter{7}, z_counter{3}, heading_counter{250};
  Eigen::Vector2d delta_xy{0.0, 0.0};
  double delta_z{0.0}, delta_heading{0.0};
  double px4_yaw_noise_amp{0.0};
  double px4_pos_noise_amp{0.0};
  std::vector<AlignmentOutput> px4_outputs;

  Eigen::Vector3d p_lio(double t) const { return p0 + v * (t - 100.0); }
  double yaw_lio(double t) const { return Wrap(yaw0 + yaw_rate * (t - 100.0)); }

  Px4PoseSample px4_at(double t) const {
    const Eigen::Vector3d pl = p_lio(t);
    const Eigen::Vector2d xy = Rot(truth.yaw_rad, pl.head<2>()) + Eigen::Vector2d{truth.x_m, truth.y_m};
    Px4PoseSample s{};
    s.t = At(t);
    // Smooth deterministic noise (1 Hz): small between PX4 samples, varied between scans.
    const double w = 2.0 * kPi * (t - 100.0);
    s.p_ned_m = {xy.x() + px4_pos_noise_amp * std::sin(w), xy.y() + px4_pos_noise_amp * std::cos(1.3 * w),
                 pl.z() + truth.z_m};
    s.yaw_rad = Wrap(truth.yaw_rad + yaw_lio(t) + px4_yaw_noise_amp * std::sin(w));
    s.xy_reset_counter = xy_counter;
    s.z_reset_counter = z_counter;
    s.heading_reset_counter = heading_counter;
    s.delta_xy_m = delta_xy;
    s.delta_z_m = delta_z;
    s.delta_heading_rad = delta_heading;
    return s;
  }

  LioPoseSample lio_at(double t, bool tracking = true) const {
    return LioPoseSample{At(t), p_lio(t), yaw_lio(t), tracking};
  }

  // PX4 samples up to and including t_end.
  void feed_px4_until(double t_end) {
    while (t_px4_next <= t_end + 1e-9) {
      px4_outputs.push_back(est.on_px4(px4_at(t_px4_next)));
      t_px4_next += 0.01;
    }
  }

  // One LIO step: PX4 up to 20 ms past the scan, then the scan.
  AlignmentOutput step(bool tracking = true) {
    const double t = t_lio_next;
    t_lio_next += 0.1;
    feed_px4_until(t + 0.02);
    const AlignmentOutput o = est.on_lio(lio_at(t, tracking));
    ExpectFinite(o);
    return o;
  }

  // Runs pairs until VALID (fails the test after `max_steps`).
  AlignmentOutput run_until_valid(int max_steps = 200) {
    for (int i = 0; i < max_steps; ++i) {
      const AlignmentOutput o = step();
      if (o.state == AlignmentState::kValid) return o;
    }
    ADD_FAILURE() << "never reached VALID";
    return AlignmentOutput{};
  }

  // PX4 reset at the next PX4 sample: the PX4 local frame changes by G (rotate by dh about the vehicle's
  // position, then translate by (dxy, dz)); the truth T follows (T' = G o T). Counter steps may be any value.
  void schedule_reset(std::uint8_t xy_step, std::uint8_t z_step, std::uint8_t heading_step, Eigen::Vector2d dxy,
                      double dz, double dh) {
    const double t = t_px4_next;
    const Eigen::Vector2d pivot = Rot(truth.yaw_rad, p_lio(t).head<2>()) + Eigen::Vector2d{truth.x_m, truth.y_m};
    const Eigen::Vector2d txy{truth.x_m, truth.y_m};
    const Eigen::Vector2d new_t = Rot(dh, txy - pivot) + pivot + dxy;
    truth = Pose4{new_t.x(), new_t.y(), truth.z_m + dz, Wrap(truth.yaw_rad + dh)};
    xy_counter = static_cast<std::uint8_t>(xy_counter + xy_step);
    z_counter = static_cast<std::uint8_t>(z_counter + z_step);
    heading_counter = static_cast<std::uint8_t>(heading_counter + heading_step);
    if (xy_step != 0) delta_xy = dxy;
    if (z_step != 0) delta_z = dz;
    if (heading_step != 0) delta_heading = dh;
  }

  // Feeds exactly one PX4 sample and returns its output.
  AlignmentOutput one_px4() {
    const AlignmentOutput o = est.on_px4(px4_at(t_px4_next));
    ExpectFinite(o);
    t_px4_next += 0.01;
    return o;
  }
};

}  // namespace

// ---------------------------------------------------------------------------------------------------------
// The brief's eight.

TEST(Alignment, ConvergesToTrueOffset) {
  Sim sim;
  AlignmentOutput o{};
  for (int i = 0; i < 30; ++i) o = sim.step();
  EXPECT_EQ(o.state, AlignmentState::kValid);
  for (int i = 0; i < 170; ++i) o = sim.step();  // 20 s of pairs in total
  ASSERT_EQ(o.state, AlignmentState::kValid);
  ExpectPoseNear(o.filtered, sim.truth, 1e-3);
  EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
}

TEST(Alignment, RejectsGnssGlitchJump) {
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  ASSERT_EQ(before.state, AlignmentState::kValid);
  // The next scan is at t_lio_next; both PX4 samples bracketing it carry a +2 m glitch.
  const double t = sim.t_lio_next;
  sim.feed_px4_until(t - 0.01);
  for (int i = 0; i < 2; ++i) {
    Px4PoseSample g = sim.px4_at(sim.t_px4_next);
    g.p_ned_m.x() += 2.0;
    sim.est.on_px4(g);
    sim.t_px4_next += 0.01;
  }
  const AlignmentOutput o = sim.est.on_lio(sim.lio_at(t));
  sim.t_lio_next += 0.1;
  EXPECT_EQ(o.reason, AlignmentReason::kPairRejectedJump);
  EXPECT_EQ(o.state, AlignmentState::kValid);
  ExpectPoseEq(o.filtered, before.filtered);
  ASSERT_TRUE(o.raw.has_value());
  EXPECT_NEAR(o.raw->x_m - sim.truth.x_m, 2.0, 1e-9);
  EXPECT_NEAR(o.residual_position_m, 2.0, 1e-6);
  // Clean data afterwards is accepted again.
  EXPECT_EQ(sim.step().reason, AlignmentReason::kPairAccepted);
}

TEST(Alignment, AppliesDoubleResetDeltas) {  // F13
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  ASSERT_EQ(before.state, AlignmentState::kValid);
  sim.feed_px4_until(sim.t_lio_next - 0.005);  // the reset sample is the first one after the next scan
  sim.schedule_reset(2, 0, 0, {1.0, 0.0}, 0.0, 0.0);
  const AlignmentOutput r = sim.one_px4();
  EXPECT_EQ(r.reason, AlignmentReason::kPx4ResetApplied);
  EXPECT_EQ(r.state, AlignmentState::kValid);
  EXPECT_EQ(r.filtered.x_m, before.filtered.x_m + 1.0);
  EXPECT_EQ(r.filtered.y_m, before.filtered.y_m);
  EXPECT_EQ(r.filtered.z_m, before.filtered.z_m);
  EXPECT_EQ(r.filtered.yaw_rad, before.filtered.yaw_rad);
  // The next pair brackets a pre-reset (shifted) and a post-reset sample: still consistent.
  for (int i = 0; i < 5; ++i) {
    const AlignmentOutput o = sim.step();
    EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
    EXPECT_LT(o.residual_position_m, 1e-9);
  }
}

TEST(Alignment, FreezesOnLioLostAndInvalidatesOnResetWhileFrozen) {
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput lost = sim.step(false);
  EXPECT_EQ(lost.state, AlignmentState::kFrozen);
  EXPECT_EQ(lost.reason, AlignmentReason::kLioLost);
  sim.schedule_reset(1, 0, 0, {0.3, 0.0}, 0.0, 0.0);
  const AlignmentOutput r = sim.one_px4();
  EXPECT_EQ(r.state, AlignmentState::kInvalid);
  EXPECT_EQ(r.reason, AlignmentReason::kPx4ResetWhileFrozen);
  EXPECT_EQ(sim.est.state(), AlignmentState::kInvalid);
}

TEST(Alignment, FrozenTimesOut) {
  Sim sim;
  sim.run_until_valid();
  const double t_lost = sim.t_lio_next;
  ASSERT_EQ(sim.step(false).state, AlignmentState::kFrozen);
  EXPECT_EQ(sim.est.on_tick(At(t_lost + 9.9)).state, AlignmentState::kFrozen);
  const AlignmentOutput o = sim.est.on_tick(At(t_lost + 10.1));
  EXPECT_EQ(o.state, AlignmentState::kInvalid);
  EXPECT_EQ(o.reason, AlignmentReason::kFrozenTooLong);
  ExpectFinite(o);
}

TEST(Alignment, ValidGoesStale) {
  Sim sim;
  sim.run_until_valid();
  const double t_last = sim.t_lio_next;
  ASSERT_EQ(sim.step().reason, AlignmentReason::kPairAccepted);
  const AlignmentOutput fresh = sim.est.on_tick(At(t_last + 0.9));
  EXPECT_EQ(fresh.state, AlignmentState::kValid);
  EXPECT_NEAR(fresh.age_s, 0.9, 1e-9);
  const AlignmentOutput o = sim.est.on_tick(At(t_last + 1.1));
  EXPECT_EQ(o.state, AlignmentState::kInvalid);
  EXPECT_EQ(o.reason, AlignmentReason::kStale);
  EXPECT_NEAR(o.age_s, 1.1, 1e-9);
}

TEST(Alignment, NoPx4SampleNearPair) {
  AlignmentEstimator est(BetaConfig());
  const AlignmentOutput o = est.on_lio(LioPoseSample{At(100.0), {1.0, 2.0, 3.0}, 0.1, true});
  EXPECT_EQ(o.reason, AlignmentReason::kNoPx4Sample);
  EXPECT_EQ(o.state, AlignmentState::kInit);
  EXPECT_FALSE(o.raw.has_value());
  ExpectFinite(o);
}

// ---------------------------------------------------------------------------------------------------------
// Pairing and interpolation.

TEST(Alignment, NoPx4SampleInValidLeavesEverythingUnchanged) {
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  // Only PX4 samples before the scan (none after it): one side missing.
  const double t = sim.t_lio_next;
  sim.feed_px4_until(t - 0.005);
  const AlignmentOutput o = sim.est.on_lio(sim.lio_at(t));
  EXPECT_EQ(o.reason, AlignmentReason::kNoPx4Sample);
  EXPECT_EQ(o.state, AlignmentState::kValid);
  ExpectPoseEq(o.filtered, before.filtered);
  EXPECT_FALSE(o.raw.has_value());
}

TEST(Alignment, PairingWindowIsFiftyMillisecondsOnEachSide) {
  static_assert(limits::kPairingWindow == time::milliseconds(50));
  auto make = [](double t) {
    Px4PoseSample s{};
    s.t = At(t);
    s.p_ned_m = {t, 0.0, 0.0};
    s.yaw_rad = 0.0;
    return s;
  };
  {
    AlignmentEstimator est(BetaConfig());
    est.on_px4(make(100.00));
    est.on_px4(make(100.10));  // 60 ms after the scan: too far
    EXPECT_EQ(est.on_lio(LioPoseSample{At(100.04), {0, 0, 0}, 0.0, true}).reason, AlignmentReason::kNoPx4Sample);
  }
  {
    AlignmentEstimator est(BetaConfig());
    est.on_px4(make(100.00));
    est.on_px4(make(100.09));  // 50 ms on both sides: accepted
    const AlignmentOutput o = est.on_lio(LioPoseSample{At(100.04), {0, 0, 0}, 0.0, true});
    EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
    ASSERT_TRUE(o.raw.has_value());
    EXPECT_NEAR(o.raw->x_m, 100.04, 1e-9);
  }
  {
    // An exact stamp match needs no later sample.
    AlignmentEstimator est(BetaConfig());
    est.on_px4(make(100.00));
    const AlignmentOutput o = est.on_lio(LioPoseSample{At(100.00), {0, 0, 0}, 0.0, true});
    EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
  }
  {
    // A scan newer than every PX4 sample.
    AlignmentEstimator est(BetaConfig());
    est.on_px4(make(100.00));
    EXPECT_EQ(est.on_lio(LioPoseSample{At(100.01), {0, 0, 0}, 0.0, true}).reason, AlignmentReason::kNoPx4Sample);
  }
}

TEST(Alignment, InterpolationIsExactForConstantVelocity) {
  Sim sim;
  sim.v = {2.0, -1.5, 0.3};
  sim.yaw_rate = 0.4;
  AlignmentOutput o = sim.run_until_valid();
  for (int i = 0; i < 50; ++i) {
    o = sim.step();
    ASSERT_EQ(o.reason, AlignmentReason::kPairAccepted);
    ASSERT_TRUE(o.raw.has_value());
    ExpectPoseNear(*o.raw, sim.truth, 1e-9);
    EXPECT_LT(o.residual_position_m, 1e-9);
    EXPECT_LT(std::abs(o.residual_yaw_rad), 1e-9);
  }
}

TEST(Alignment, InterpolatesYawAcrossTheBranchCut) {
  auto make = [](double t, double yaw) {
    Px4PoseSample s{};
    s.t = At(t);
    s.p_ned_m = {0.0, 0.0, 0.0};
    s.yaw_rad = yaw;
    return s;
  };
  AlignmentEstimator est(BetaConfig());
  est.on_px4(make(100.00, kPi - 0.01));
  est.on_px4(make(100.02, -kPi + 0.01));  // +0.02 rad through +-pi
  const AlignmentOutput o = est.on_lio(LioPoseSample{At(100.01), {0, 0, 0}, 0.0, true});
  ASSERT_TRUE(o.raw.has_value());
  EXPECT_NEAR(std::abs(o.raw->yaw_rad), kPi, 1e-9);  // midpoint is pi, not 0
}

TEST(Alignment, RingWrapKeepsOnlyTheNewestSpan) {
  static_assert(limits::kPx4BufferSpan == time::seconds(1));
  Sim sim;
  sim.feed_px4_until(105.0);  // 5 s at 100 Hz: the ring wraps
  // Within the span: interpolated exactly.
  AlignmentOutput o = sim.est.on_lio(sim.lio_at(104.503));
  EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
  ASSERT_TRUE(o.raw.has_value());
  ExpectPoseNear(*o.raw, sim.truth, 1e-9);
  // Overwritten by the ring (older than 2.55 s at 100 Hz), and older than the span (newest - 1 s) though
  // still stored. Queried oldest first: LIO stamps must increase.
  Sim sim2;
  sim2.feed_px4_until(105.0);
  EXPECT_EQ(sim2.est.on_lio(sim2.lio_at(101.003)).reason, AlignmentReason::kNoPx4Sample);
  EXPECT_EQ(sim2.est.on_lio(sim2.lio_at(103.503)).reason, AlignmentReason::kNoPx4Sample);
  EXPECT_EQ(sim2.est.on_lio(sim2.lio_at(104.003)).reason, AlignmentReason::kPairAccepted);
}

TEST(Alignment, RingCapacityBoundsHistoryAtHighRate) {
  // 400 Hz for 2 s: more samples than the ring holds. The newest ones are still usable.
  AlignmentEstimator est(BetaConfig());
  for (int k = 0; k <= 800; ++k) {
    Px4PoseSample s{};
    s.t = At(100.0 + 0.0025 * k);
    s.p_ned_m = {0.0025 * k, 0.0, 0.0};
    s.yaw_rad = 0.0;
    est.on_px4(s);
  }
  const AlignmentOutput o = est.on_lio(LioPoseSample{At(101.90125), {0, 0, 0}, 0.0, true});
  EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
  ASSERT_TRUE(o.raw.has_value());
  EXPECT_NEAR(o.raw->x_m, 1.90125, 1e-9);
  static_assert(limits::kPx4BufferCapacity < 800);
}

TEST(Alignment, NonMonotonicPx4TimeIsRejected) {
  auto make = [](double t, double x) {
    Px4PoseSample s{};
    s.t = At(t);
    s.p_ned_m = {x, 0.0, 0.0};
    s.yaw_rad = 0.0;
    return s;
  };
  AlignmentEstimator est(BetaConfig());
  EXPECT_EQ(est.on_px4(make(100.00, 0.0)).reason, AlignmentReason::kNone);
  EXPECT_EQ(est.on_px4(make(100.02, 2.0)).reason, AlignmentReason::kNone);
  EXPECT_EQ(est.on_px4(make(100.01, 50.0)).reason, AlignmentReason::kInputRejected);  // older
  EXPECT_EQ(est.on_px4(make(100.02, 70.0)).reason, AlignmentReason::kInputRejected);  // same stamp
  const AlignmentOutput o = est.on_lio(LioPoseSample{At(100.01), {0, 0, 0}, 0.0, true});
  ASSERT_TRUE(o.raw.has_value());
  EXPECT_NEAR(o.raw->x_m, 1.0, 1e-12);
}

TEST(Alignment, RejectedPx4SampleDoesNotConsumeAResetCounter) {
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  sim.feed_px4_until(sim.t_lio_next - 0.05);
  sim.schedule_reset(1, 0, 0, {0.7, 0.0}, 0.0, 0.0);
  // A time-regressed copy carrying the new counter is ignored...
  Px4PoseSample stale = sim.px4_at(sim.t_px4_next);
  stale.t = At(sim.t_px4_next - 0.02);
  const AlignmentOutput rej = sim.est.on_px4(stale);
  EXPECT_EQ(rej.reason, AlignmentReason::kInputRejected);
  ExpectPoseEq(rej.filtered, before.filtered);
  // ...and the next accepted sample applies the delta exactly once.
  const AlignmentOutput r = sim.one_px4();
  EXPECT_EQ(r.reason, AlignmentReason::kPx4ResetApplied);
  EXPECT_EQ(r.filtered.x_m, before.filtered.x_m + 0.7);
  EXPECT_EQ(sim.one_px4().reason, AlignmentReason::kNone);  // same counters again: nothing
}

// ---------------------------------------------------------------------------------------------------------
// PX4 resets (F13).

TEST(Alignment, ResetCounterWrapIsAReset) {
  Sim sim;
  sim.xy_counter = 255;
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  sim.feed_px4_until(sim.t_lio_next - 0.05);
  sim.schedule_reset(1, 0, 0, {-0.4, 0.25}, 0.0, 0.0);
  ASSERT_EQ(sim.xy_counter, 0);
  const AlignmentOutput r = sim.one_px4();
  EXPECT_EQ(r.reason, AlignmentReason::kPx4ResetApplied);
  EXPECT_EQ(r.filtered.x_m, before.filtered.x_m - 0.4);
  EXPECT_EQ(r.filtered.y_m, before.filtered.y_m + 0.25);
}

TEST(Alignment, ZResetStepOfThreeAppliesDeltaZ) {
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  sim.feed_px4_until(sim.t_lio_next - 0.05);
  sim.schedule_reset(0, 3, 0, {0.0, 0.0}, -1.25, 0.0);
  const AlignmentOutput r = sim.one_px4();
  EXPECT_EQ(r.reason, AlignmentReason::kPx4ResetApplied);
  EXPECT_EQ(r.filtered.z_m, before.filtered.z_m - 1.25);
  EXPECT_EQ(r.filtered.x_m, before.filtered.x_m);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(sim.step().reason, AlignmentReason::kPairAccepted);
}

TEST(Alignment, HeadingResetRotatesTAboutTheVehicleAndShiftsBufferedYaw) {
  Sim sim;
  sim.p0 = {40.0, 30.0, -5.0};  // far from the lio_odom origin: a pure yaw add would be off by metres
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  sim.feed_px4_until(sim.t_lio_next - 0.005);  // the next scan brackets one pre-reset sample
  const double dh = 0.3;
  sim.schedule_reset(0, 0, 1, {0.0, 0.0}, 0.0, dh);
  const AlignmentOutput r = sim.one_px4();
  EXPECT_EQ(r.reason, AlignmentReason::kPx4ResetApplied);
  EXPECT_EQ(r.state, AlignmentState::kValid);
  EXPECT_NEAR(Wrap(r.filtered.yaw_rad - (before.filtered.yaw_rad + dh)), 0.0, 1e-12);
  ExpectPoseNear(r.filtered, sim.truth, 1e-6);
  // The pair right after the reset interpolates between a shifted pre-reset sample and a post-reset one.
  const AlignmentOutput o = sim.step();
  EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
  EXPECT_LT(o.residual_position_m, 1e-6);
  EXPECT_LT(std::abs(o.residual_yaw_rad), 1e-9);
}

TEST(Alignment, SimultaneousResetsOfAllQuantities) {
  Sim sim;
  sim.p0 = {-12.0, 25.0, -4.0};
  sim.heading_counter = 255;
  sim.run_until_valid();
  sim.step();
  sim.feed_px4_until(sim.t_lio_next - 0.005);
  sim.schedule_reset(2, 1, 1, {0.6, -0.9}, 0.4, -0.25);  // xy +2, z +1, heading 255 -> 0
  const AlignmentOutput r = sim.one_px4();
  EXPECT_EQ(r.reason, AlignmentReason::kPx4ResetApplied);
  EXPECT_EQ(r.state, AlignmentState::kValid);
  ExpectPoseNear(r.filtered, sim.truth, 1e-6);
  for (int i = 0; i < 5; ++i) {
    const AlignmentOutput o = sim.step();
    EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
    EXPECT_LT(o.residual_position_m, 1e-6);
  }
}

TEST(Alignment, FirstPx4SampleOnlyInitialisesCounters) {
  Sim sim;
  sim.xy_counter = 200;
  sim.delta_xy = {5.0, 5.0};  // a reset that happened before we started listening
  const AlignmentOutput o = sim.one_px4();
  EXPECT_EQ(o.reason, AlignmentReason::kNone);
  EXPECT_EQ(sim.run_until_valid().state, AlignmentState::kValid);
  ExpectPoseNear(sim.step().filtered, sim.truth, 1e-9);
}

TEST(Alignment, ResetInInitRestartsAccumulation) {
  Sim sim;
  for (int i = 0; i < 10; ++i) ASSERT_EQ(sim.step().state, AlignmentState::kInit);
  sim.schedule_reset(1, 0, 0, {1.0, 0.0}, 0.0, 0.0);
  const AlignmentOutput r = sim.one_px4();
  EXPECT_EQ(r.state, AlignmentState::kInit);
  EXPECT_EQ(r.reason, AlignmentReason::kPx4ResetApplied);
  for (int i = 0; i < 19; ++i) ASSERT_EQ(sim.step().state, AlignmentState::kInit) << i;
  const AlignmentOutput v = sim.step();
  EXPECT_EQ(v.state, AlignmentState::kValid);
  EXPECT_EQ(v.reason, AlignmentReason::kPairsConsistent);
  ExpectPoseNear(v.filtered, sim.truth, 1e-9);
}

// ---------------------------------------------------------------------------------------------------------
// INIT, yaw handling, transitions.

TEST(Alignment, InitNeedsConsecutiveConsistentPairs) {
  Sim sim;
  for (int i = 0; i < 19; ++i) ASSERT_EQ(sim.step().state, AlignmentState::kInit);
  sim.truth.x_m += 1.0;  // inconsistent pair: the accumulation restarts from it
  const AlignmentOutput bad = sim.step();
  EXPECT_EQ(bad.state, AlignmentState::kInit);
  EXPECT_EQ(bad.reason, AlignmentReason::kPairRejectedJump);
  for (int i = 0; i < 18; ++i) ASSERT_EQ(sim.step().state, AlignmentState::kInit) << i;
  const AlignmentOutput v = sim.step();
  EXPECT_EQ(v.state, AlignmentState::kValid);
  EXPECT_EQ(v.reason, AlignmentReason::kPairsConsistent);
  ExpectPoseNear(v.filtered, sim.truth, 1e-9);
}

TEST(Alignment, YawNearPiStaysContinuousAndConverges) {
  Sim sim;
  sim.truth.yaw_rad = 3.1;
  sim.yaw_rate = 0.0;
  sim.yaw0 = 0.0;
  // Hover near the lio_odom origin: heading noise e times |p_lio| becomes translation noise of the instant T.
  sim.p0 = {0.2, -0.1, -1.0};
  sim.v = {0.0, 0.0, 0.0};
  sim.px4_yaw_noise_amp = 0.06;  // instant yaw in [3.04, 3.16]: crosses +-pi every cycle
  const AlignmentOutput v = sim.run_until_valid(60);
  ASSERT_EQ(v.state, AlignmentState::kValid);
  EXPECT_NEAR(Wrap(v.filtered.yaw_rad - 3.1), 0.0, 0.02);
  AlignmentOutput prev = v;
  bool crossed = false;
  for (int i = 0; i < 150; ++i) {
    const AlignmentOutput o = sim.step();
    ASSERT_EQ(o.state, AlignmentState::kValid);
    ASSERT_TRUE(o.raw.has_value());
    if (o.raw->yaw_rad < 0.0) crossed = true;
    // No separate rate limiter (D30/O12): the gate and tau bound a yaw step to jump_yaw x dt / tau.
    EXPECT_LE(std::abs(Wrap(o.filtered.yaw_rad - prev.filtered.yaw_rad)), 0.0873 * 0.1 / 2.0 + 1e-9);
    EXPECT_LT(std::abs(o.residual_yaw_rad), 0.0873);
    prev = o;
  }
  EXPECT_TRUE(crossed) << "noise never crossed the branch cut";
  EXPECT_NEAR(Wrap(prev.filtered.yaw_rad - 3.1), 0.0, 0.01);
}

TEST(Alignment, FrozenReturnsToValidWhenTrackingReturns) {
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  for (int i = 0; i < 5; ++i) {
    const AlignmentOutput o = sim.step(false);
    EXPECT_EQ(o.state, AlignmentState::kFrozen);
    ExpectPoseEq(o.filtered, before.filtered);
  }
  const AlignmentOutput back = sim.step(true);
  EXPECT_EQ(back.state, AlignmentState::kValid);
  EXPECT_EQ(back.reason, AlignmentReason::kLioTracking);
  ExpectPoseNear(back.filtered, sim.truth, 1e-9);
  EXPECT_EQ(sim.step().reason, AlignmentReason::kPairAccepted);
}

TEST(Alignment, TrackingFalseNeverYieldsValid) {
  Sim sim;
  sim.run_until_valid();
  for (int i = 0; i < 30; ++i) EXPECT_NE(sim.step(false).state, AlignmentState::kValid);
  Sim init;
  for (int i = 0; i < 40; ++i) {
    const AlignmentOutput o = init.step(false);
    EXPECT_EQ(o.state, AlignmentState::kInit);
  }
}

TEST(Alignment, LioLostInInitRestartsAccumulation) {
  Sim sim;
  for (int i = 0; i < 15; ++i) sim.step();
  EXPECT_EQ(sim.step(false).reason, AlignmentReason::kLioLost);
  for (int i = 0; i < 19; ++i) ASSERT_EQ(sim.step().state, AlignmentState::kInit);
  EXPECT_EQ(sim.step().state, AlignmentState::kValid);
}

TEST(Alignment, InvalidRecoversThroughInitWithNewAccumulation) {
  Sim sim;
  sim.run_until_valid();
  const double t_last = sim.t_lio_next;
  sim.step();
  ASSERT_EQ(sim.est.on_tick(At(t_last + 1.2)).state, AlignmentState::kInvalid);
  // While INVALID the truth moved far (e.g. PX4 re-initialised); the new accumulation must find it.
  sim.truth = Pose4{-7.0, 4.0, 1.5, -2.0};
  sim.t_lio_next = t_last + 1.3;
  sim.t_px4_next = t_last + 1.2;
  const AlignmentOutput init = sim.step();
  EXPECT_EQ(init.state, AlignmentState::kInit);
  EXPECT_EQ(init.reason, AlignmentReason::kLioTracking);
  for (int i = 0; i < 19; ++i) ASSERT_EQ(sim.step().state, AlignmentState::kInit) << i;
  const AlignmentOutput v = sim.step();
  EXPECT_EQ(v.state, AlignmentState::kValid);
  ExpectPoseNear(v.filtered, sim.truth, 1e-9);
}

TEST(Alignment, StaleIsAlsoDetectedLazilyInOnLio) {
  Sim sim;
  sim.run_until_valid();
  sim.step();
  sim.t_lio_next += 1.5;  // a gap with no ticks
  sim.t_px4_next = sim.t_lio_next - 0.05;
  const AlignmentOutput o = sim.step();
  EXPECT_EQ(o.state, AlignmentState::kInvalid);
  EXPECT_EQ(o.reason, AlignmentReason::kStale);
}

TEST(Alignment, FrozenTimeoutIsAlsoDetectedLazilyInOnLio) {
  Sim sim;
  sim.run_until_valid();
  sim.step(false);
  sim.t_lio_next += 11.0;
  sim.t_px4_next = sim.t_lio_next - 0.05;
  const AlignmentOutput o = sim.step(true);
  EXPECT_EQ(o.state, AlignmentState::kInvalid);
  EXPECT_EQ(o.reason, AlignmentReason::kFrozenTooLong);
}

TEST(Alignment, ResumedValidHasAFreshStaleWindow) {
  Sim sim;
  sim.run_until_valid();
  for (int i = 0; i < 20; ++i) sim.step(false);  // 2 s FROZEN
  // Tracking returns but no PX4 sample brackets the scan yet: VALID resumes without an accepted pair.
  const double t_back = sim.t_lio_next;
  const AlignmentOutput back = sim.est.on_lio(sim.lio_at(t_back));
  EXPECT_EQ(back.state, AlignmentState::kValid);
  EXPECT_EQ(back.reason, AlignmentReason::kLioTracking);
  EXPECT_FALSE(back.raw.has_value());
  // The stale window runs from the return to VALID, not from the last pair before FROZEN (2.1 s ago).
  EXPECT_EQ(sim.est.on_tick(At(t_back + 0.5)).state, AlignmentState::kValid);
  const AlignmentOutput stale = sim.est.on_tick(At(t_back + 1.05));
  EXPECT_EQ(stale.state, AlignmentState::kInvalid);
  EXPECT_EQ(stale.reason, AlignmentReason::kStale);
}

TEST(Alignment, OnTickEarlierThanLastSeenDoesNothing) {
  Sim sim;
  sim.run_until_valid();
  const double t_last = sim.t_lio_next;
  sim.step();
  EXPECT_EQ(sim.est.on_tick(At(t_last + 0.5)).state, AlignmentState::kValid);
  const AlignmentOutput o = sim.est.on_tick(At(t_last + 0.2));
  EXPECT_EQ(o.state, AlignmentState::kValid);
  EXPECT_EQ(o.reason, AlignmentReason::kNone);
  EXPECT_NEAR(o.age_s, 0.5, 1e-9);  // unchanged: the tick was ignored
}

// ---------------------------------------------------------------------------------------------------------
// Fail closed.

TEST(Alignment, NonFiniteInputsAreRejected) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  const double t = sim.t_lio_next;
  sim.feed_px4_until(t - 0.03);
  for (int field = 0; field < 6; ++field) {
    Px4PoseSample s = sim.px4_at(sim.t_px4_next);
    switch (field) {
      case 0: s.p_ned_m.x() = kNan; break;
      case 1: s.p_ned_m.z() = kInf; break;
      case 2: s.yaw_rad = kNan; break;
      case 3: s.delta_xy_m.y() = kNan; break;
      case 4: s.delta_z_m = -kInf; break;
      default: s.delta_heading_rad = kNan; break;
    }
    const AlignmentOutput o = sim.est.on_px4(s);
    EXPECT_EQ(o.reason, AlignmentReason::kInputRejected) << field;
    ExpectFinite(o);
  }
  Px4PoseSample neg = sim.px4_at(sim.t_px4_next);
  neg.t = time::SensorTime{-1};
  EXPECT_EQ(sim.est.on_px4(neg).reason, AlignmentReason::kInputRejected);
  sim.feed_px4_until(t + 0.02);
  for (int field = 0; field < 3; ++field) {
    LioPoseSample l = sim.lio_at(t);
    if (field == 0) l.p_frd_m.y() = kNan;
    if (field == 1) l.p_frd_m.z() = kInf;
    if (field == 2) l.yaw_frd_rad = kNan;
    const AlignmentOutput o = sim.est.on_lio(l);
    EXPECT_EQ(o.reason, AlignmentReason::kInputRejected) << field;
    EXPECT_EQ(o.state, AlignmentState::kValid);
    ExpectPoseEq(o.filtered, before.filtered);
    ExpectFinite(o);
  }
  // A lost LIO sample with a garbage pose still freezes (the pose of a lost sample is not used).
  LioPoseSample lost = sim.lio_at(t);
  lost.p_frd_m.x() = kNan;
  lost.tracking = false;
  const AlignmentOutput f = sim.est.on_lio(lost);
  EXPECT_EQ(f.state, AlignmentState::kFrozen);
  ExpectFinite(f);
}

TEST(Alignment, LioTimeRegressionIsRejected) {
  Sim sim;
  sim.run_until_valid();
  const double t_prev = sim.t_lio_next;
  const AlignmentOutput before = sim.step();
  const AlignmentOutput dup = sim.est.on_lio(sim.lio_at(t_prev));
  EXPECT_EQ(dup.reason, AlignmentReason::kInputRejected);
  const AlignmentOutput old = sim.est.on_lio(sim.lio_at(t_prev - 0.1, false));
  EXPECT_EQ(old.reason, AlignmentReason::kInputRejected);
  EXPECT_EQ(old.state, AlignmentState::kValid);
  ExpectPoseEq(old.filtered, before.filtered);
}

TEST(Alignment, InvalidConfigNeverReachesValid) {
  AlignmentConfig bad = BetaConfig();
  bad.consistent_pairs = 0;
  Sim a(bad);
  for (int i = 0; i < 50; ++i) EXPECT_EQ(a.step().state, AlignmentState::kInit);
  bad = BetaConfig();
  bad.consistent_pairs = limits::kMaxConsistentPairs + 1;
  Sim b(bad);
  for (int i = 0; i < 300; ++i) EXPECT_EQ(b.step().state, AlignmentState::kInit);
  bad = BetaConfig();
  bad.jump_position_m = std::numeric_limits<double>::quiet_NaN();
  Sim c(bad);
  for (int i = 0; i < 50; ++i) EXPECT_EQ(c.step().state, AlignmentState::kInit);
  bad = BetaConfig();
  bad.tau = time::milliseconds(100);  // below kFilterDtMax: alpha could exceed 1
  Sim d(bad);
  for (int i = 0; i < 50; ++i) EXPECT_EQ(d.step().state, AlignmentState::kInit);
}

// ---------------------------------------------------------------------------------------------------------
// Determinism, enums, transition table.

TEST(Alignment, IsDeterministic) {
  auto run = [] {
    Sim sim;
    sim.px4_yaw_noise_amp = 0.02;
    sim.px4_pos_noise_amp = 0.05;
    std::vector<AlignmentOutput> out;
    for (int i = 0; i < 60; ++i) out.push_back(sim.step());
    sim.schedule_reset(2, 1, 1, {0.3, 0.1}, -0.2, 0.05);
    for (int i = 0; i < 20; ++i) out.push_back(sim.step());
    for (int i = 0; i < 5; ++i) out.push_back(sim.step(false));
    for (int i = 0; i < 20; ++i) out.push_back(sim.step());
    out.push_back(sim.est.on_tick(At(sim.t_lio_next + 2.0)));
    for (const AlignmentOutput& o : sim.px4_outputs) out.push_back(o);
    return out;
  };
  const auto a = run();
  const auto b = run();
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].state, b[i].state);
    EXPECT_EQ(a[i].reason, b[i].reason);
    ExpectPoseEq(a[i].filtered, b[i].filtered);
    EXPECT_EQ(a[i].raw.has_value(), b[i].raw.has_value());
    if (a[i].raw && b[i].raw) ExpectPoseEq(*a[i].raw, *b[i].raw);
    EXPECT_EQ(a[i].stamp, b[i].stamp);
    EXPECT_EQ(a[i].age_s, b[i].age_s);
    EXPECT_EQ(a[i].residual_position_m, b[i].residual_position_m);
    EXPECT_EQ(a[i].residual_yaw_rad, b[i].residual_yaw_rad);
  }
}

TEST(Alignment, StampIsTheLioTimeOfTheNewestPair) {
  Sim sim;
  sim.run_until_valid();
  const double t = sim.t_lio_next;
  const AlignmentOutput o = sim.step();
  EXPECT_EQ(o.stamp, At(t));
  EXPECT_EQ(o.age_s, 0.0);
  const AlignmentOutput tick = sim.est.on_tick(At(t + 0.25));
  EXPECT_EQ(tick.stamp, At(t));
  EXPECT_NEAR(tick.age_s, 0.25, 1e-12);
}

TEST(Alignment, StateValuesMatchAlignmentMsg) {
  static_assert(static_cast<std::uint8_t>(AlignmentState::kInit) == 0);
  static_assert(static_cast<std::uint8_t>(AlignmentState::kValid) == 1);
  static_assert(static_cast<std::uint8_t>(AlignmentState::kFrozen) == 2);
  static_assert(static_cast<std::uint8_t>(AlignmentState::kInvalid) == 3);
  static_assert(ReasonEnum<AlignmentState>);
  static_assert(ReasonEnum<AlignmentReason>);
  EXPECT_EQ(to_string(AlignmentState::kInit), "INIT");
  EXPECT_EQ(to_string(AlignmentState::kValid), "VALID");
  EXPECT_EQ(to_string(AlignmentState::kFrozen), "FROZEN");
  EXPECT_EQ(to_string(AlignmentState::kInvalid), "INVALID");
  EXPECT_EQ(to_string(AlignmentReason::kNone), "NONE");
  EXPECT_EQ(to_string(AlignmentReason::kPairsConsistent), "PAIRS_CONSISTENT");
  EXPECT_EQ(to_string(AlignmentReason::kPairAccepted), "PAIR_ACCEPTED");
  EXPECT_EQ(to_string(AlignmentReason::kPairRejectedJump), "PAIR_REJECTED_JUMP");
  EXPECT_EQ(to_string(AlignmentReason::kLioLost), "LIO_LOST");
  EXPECT_EQ(to_string(AlignmentReason::kLioTracking), "LIO_TRACKING");
  EXPECT_EQ(to_string(AlignmentReason::kPx4ResetApplied), "PX4_RESET_APPLIED");
  EXPECT_EQ(to_string(AlignmentReason::kPx4ResetWhileFrozen), "PX4_RESET_WHILE_FROZEN");
  EXPECT_EQ(to_string(AlignmentReason::kStale), "STALE");
  EXPECT_EQ(to_string(AlignmentReason::kFrozenTooLong), "FROZEN_TOO_LONG");
  EXPECT_EQ(to_string(AlignmentReason::kNoPx4Sample), "NO_PX4_SAMPLE");
  EXPECT_EQ(to_string(AlignmentReason::kInputRejected), "INPUT_REJECTED");
}

TEST(Alignment, TransitionTableIsTheSpecDiagram) {
  // SYSTEM_DESIGN §4.1: INIT -> VALID <-> FROZEN -> INVALID, VALID -> INVALID (stale), INVALID -> INIT.
  ASSERT_EQ(kAlignmentTransitions.size(), 7U);
  auto listed = [](AlignmentState f, AlignmentState t, AlignmentReason r) {
    for (const auto& e : kAlignmentTransitions) {
      if (e.from == f && e.to == t && e.reason == r) return true;
    }
    return false;
  };
  using S = AlignmentState;
  using R = AlignmentReason;
  EXPECT_TRUE(listed(S::kInit, S::kValid, R::kPairsConsistent));
  EXPECT_TRUE(listed(S::kValid, S::kFrozen, R::kLioLost));
  EXPECT_TRUE(listed(S::kFrozen, S::kValid, R::kLioTracking));
  EXPECT_TRUE(listed(S::kValid, S::kInvalid, R::kStale));
  EXPECT_TRUE(listed(S::kFrozen, S::kInvalid, R::kFrozenTooLong));
  EXPECT_TRUE(listed(S::kFrozen, S::kInvalid, R::kPx4ResetWhileFrozen));
  EXPECT_TRUE(listed(S::kInvalid, S::kInit, R::kLioTracking));
}

// ---------------------------------------------------------------------------------------------------------
// Boundary behaviour (review fix round 1).

namespace {

Px4PoseSample Px4At(time::SensorTime t, Eigen::Vector3d p, double yaw) {
  Px4PoseSample s{};
  s.t = t;
  s.p_ned_m = p;
  s.yaw_rad = yaw;
  return s;
}

// LIO at the origin with yaw 0, so the instant T is exactly the PX4 pose (no rotation, no rounding).
LioPoseSample LioOrigin(time::SensorTime t, bool tracking = true) { return LioPoseSample{t, {0, 0, 0}, 0.0, tracking}; }

}  // namespace

TEST(AlignmentBoundary, PairingWindowBeforeTheScanIsInclusive) {
  constexpr time::SensorTime kScan{100'000'000'000};
  for (const std::int64_t extra_ns : {0LL, 1LL}) {
    AlignmentEstimator est(BetaConfig());
    est.on_px4(Px4At(kScan - limits::kPairingWindow - time::nanoseconds(extra_ns), {0, 0, 0}, 0.0));
    est.on_px4(Px4At(kScan + time::milliseconds(10), {0, 0, 0}, 0.0));
    EXPECT_EQ(est.on_lio(LioOrigin(kScan)).reason,
              extra_ns == 0 ? AlignmentReason::kPairAccepted : AlignmentReason::kNoPx4Sample)
        << "a-side 50 ms + " << extra_ns << " ns";
  }
}

TEST(AlignmentBoundary, PairingWindowAfterTheScanIsInclusive) {
  constexpr time::SensorTime kScan{100'000'000'000};
  for (const std::int64_t extra_ns : {0LL, 1LL}) {
    AlignmentEstimator est(BetaConfig());
    est.on_px4(Px4At(kScan - time::milliseconds(10), {0, 0, 0}, 0.0));
    est.on_px4(Px4At(kScan + limits::kPairingWindow + time::nanoseconds(extra_ns), {0, 0, 0}, 0.0));
    EXPECT_EQ(est.on_lio(LioOrigin(kScan)).reason,
              extra_ns == 0 ? AlignmentReason::kPairAccepted : AlignmentReason::kNoPx4Sample)
        << "b-side 50 ms + " << extra_ns << " ns";
  }
}

TEST(AlignmentBoundary, ValidStaleFiresOnlyAfterTheLimit) {
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput last = sim.step();
  ASSERT_EQ(last.reason, AlignmentReason::kPairAccepted);
  const time::SensorTime limit = last.stamp + time::seconds(1);
  EXPECT_EQ(sim.est.on_tick(limit).state, AlignmentState::kValid);  // exactly valid_stale: not stale
  const AlignmentOutput o = sim.est.on_tick(limit + time::nanoseconds(1));
  EXPECT_EQ(o.state, AlignmentState::kInvalid);
  EXPECT_EQ(o.reason, AlignmentReason::kStale);
}

TEST(AlignmentBoundary, FrozenTimeoutFiresOnlyAfterTheLimit) {
  Sim sim;
  sim.run_until_valid();
  const time::SensorTime lost_at = At(sim.t_lio_next);
  ASSERT_EQ(sim.step(false).state, AlignmentState::kFrozen);
  const time::SensorTime limit = lost_at + time::seconds(10);
  EXPECT_EQ(sim.est.on_tick(limit).state, AlignmentState::kFrozen);  // exactly frozen_max: still FROZEN
  const AlignmentOutput o = sim.est.on_tick(limit + time::nanoseconds(1));
  EXPECT_EQ(o.state, AlignmentState::kInvalid);
  EXPECT_EQ(o.reason, AlignmentReason::kFrozenTooLong);
}

TEST(AlignmentBoundary, JumpThresholdIsInclusive) {
  // consistent_pairs = 1: the first exact pair (T = 0) makes VALID with filtered T exactly 0.
  AlignmentConfig cfg = BetaConfig();
  cfg.consistent_pairs = 1;
  constexpr time::SensorTime kT0{100'000'000'000};
  constexpr time::SensorTime kT1{100'100'000'000};
  struct Case {
    Eigen::Vector3d p;
    double yaw;
    AlignmentReason expected;
  };
  const double jp = cfg.jump_position_m;
  const double jy = cfg.jump_yaw_rad;
  const Case cases[] = {
      {{jp, 0, 0}, 0.0, AlignmentReason::kPairAccepted},
      {{std::nextafter(jp, 1.0), 0, 0}, 0.0, AlignmentReason::kPairRejectedJump},
      {{0, 0, jp}, 0.0, AlignmentReason::kPairAccepted},
      {{0, 0, std::nextafter(jp, 1.0)}, 0.0, AlignmentReason::kPairRejectedJump},
      {{0, 0, 0}, jy, AlignmentReason::kPairAccepted},
      {{0, 0, 0}, -jy, AlignmentReason::kPairAccepted},
      {{0, 0, 0}, std::nextafter(jy, 1.0), AlignmentReason::kPairRejectedJump},
      {{0, 0, 0}, -std::nextafter(jy, 1.0), AlignmentReason::kPairRejectedJump},
  };
  for (const Case& c : cases) {
    AlignmentEstimator est(cfg);
    est.on_px4(Px4At(kT0, {0, 0, 0}, 0.0));
    const AlignmentOutput v = est.on_lio(LioOrigin(kT0));
    ASSERT_EQ(v.state, AlignmentState::kValid);
    ASSERT_EQ(v.filtered.x_m, 0.0);
    ASSERT_EQ(v.filtered.yaw_rad, 0.0);
    est.on_px4(Px4At(kT1, c.p, c.yaw));
    const AlignmentOutput o = est.on_lio(LioOrigin(kT1));
    EXPECT_EQ(o.reason, c.expected) << c.p.transpose() << " yaw " << c.yaw;
    EXPECT_EQ(o.state, AlignmentState::kValid);
  }
}

TEST(AlignmentBoundary, InitFinalMeanRecheckRestartsTheAccumulation) {
  // Each pair is within 0.5 m of the running mean at its arrival (0 | 0.5 vs 0 | 0.75 vs 0.25 |
  // 0.91 vs 0.4167), but the final mean 0.54 is 0.54 m from the first member: the re-check fails and
  // the accumulation restarts from the newest pair.
  AlignmentConfig cfg = BetaConfig();
  cfg.consistent_pairs = 4;
  AlignmentEstimator est(cfg);
  std::int64_t t_ns = 100'000'000'000;
  auto pair = [&](double x) {
    t_ns += 100'000'000;
    est.on_px4(Px4At(time::SensorTime{t_ns}, {x, 0, 0}, 0.0));
    return est.on_lio(LioOrigin(time::SensorTime{t_ns}));
  };
  EXPECT_EQ(pair(0.0).reason, AlignmentReason::kPairAccepted);
  EXPECT_EQ(pair(0.5).reason, AlignmentReason::kPairAccepted);
  EXPECT_EQ(pair(0.75).reason, AlignmentReason::kPairAccepted);
  const AlignmentOutput restarted = pair(0.91);
  EXPECT_EQ(restarted.state, AlignmentState::kInit);
  EXPECT_EQ(restarted.reason, AlignmentReason::kPairRejectedJump);
  EXPECT_NEAR(restarted.filtered.x_m, 0.91, 1e-12);  // the accumulation holds only the newest pair
  EXPECT_EQ(pair(0.91).state, AlignmentState::kInit);
  EXPECT_EQ(pair(0.91).state, AlignmentState::kInit);
  const AlignmentOutput v = pair(0.91);
  EXPECT_EQ(v.state, AlignmentState::kValid);
  EXPECT_EQ(v.reason, AlignmentReason::kPairsConsistent);
  EXPECT_NEAR(v.filtered.x_m, 0.91, 1e-12);
}

TEST(AlignmentBoundary, Px4ResetWhileInvalidAppliesGAndStaysInvalid) {
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput last = sim.step();
  ASSERT_EQ(sim.est.on_tick(last.stamp + time::milliseconds(1100)).state, AlignmentState::kInvalid);
  sim.t_px4_next = time::to_seconds(time::Duration{last.stamp.ns}) + 1.2;
  sim.schedule_reset(1, 0, 0, {0.8, -0.2}, 0.0, 0.0);
  const AlignmentOutput r = sim.one_px4();
  EXPECT_EQ(r.state, AlignmentState::kInvalid);
  EXPECT_EQ(r.reason, AlignmentReason::kPx4ResetApplied);
  EXPECT_EQ(r.filtered.x_m, last.filtered.x_m + 0.8);  // G applied to the kept (untrusted) T
  EXPECT_EQ(r.filtered.y_m, last.filtered.y_m - 0.2);
  // No VALID output follows directly: the next tracking scan only re-enters INIT.
  sim.t_lio_next = time::to_seconds(time::Duration{last.stamp.ns}) + 1.203;
  const AlignmentOutput n = sim.step();
  EXPECT_EQ(n.state, AlignmentState::kInit);
  EXPECT_EQ(n.reason, AlignmentReason::kLioTracking);
}

// ---------------------------------------------------------------------------------------------------------
// Every measurement of T is taken at the vehicle position, not at the lio_odom origin (D30/O12, §4.1).

namespace {

constexpr time::SensorTime kFarT0{100'000'000'000};
constexpr time::SensorTime kFarT1{100'100'000'000};  // dt = 0.1 s, alpha = 0.05 with tau = 2 s
const Eigen::Vector3d kFar{300.0, 0.0, 0.0};

// consistent_pairs = 1 and one exact pair at p_lio = p_px4 = (300, 0, 0), yaw 0: VALID with T exactly 0.
AlignmentOutput ValidAtFar(AlignmentEstimator& est) {
  est.on_px4(Px4At(kFarT0, kFar, 0.0));
  return est.on_lio(LioPoseSample{kFarT0, kFar, 0.0, true});
}

AlignmentConfig OnePairConfig() {
  AlignmentConfig cfg = BetaConfig();
  cfg.consistent_pairs = 1;
  return cfg;
}

void ExpectZero(const Pose4& T) {
  EXPECT_EQ(T.x_m, 0.0);
  EXPECT_EQ(T.y_m, 0.0);
  EXPECT_EQ(T.z_m, 0.0);
  EXPECT_EQ(T.yaw_rad, 0.0);
}

}  // namespace

TEST(Alignment, FarFromOriginYawNoiseStaysValid) {  // Review Focus 1, O12
  Sim sim;
  sim.p0 = {300.0, 0.0, -20.0};
  sim.v = {0.8, -0.3, 0.05};
  sim.px4_yaw_noise_amp = 0.005;  // 5 mrad: x 300 m = 1.5 m at the origin, nothing at the vehicle
  AlignmentOutput o{};
  for (int i = 0; i < 19; ++i) {
    o = sim.step();
    ASSERT_EQ(o.state, AlignmentState::kInit) << i;
    ASSERT_EQ(o.reason, AlignmentReason::kPairAccepted) << i;
  }
  o = sim.step();
  ASSERT_EQ(o.state, AlignmentState::kValid);
  EXPECT_EQ(o.reason, AlignmentReason::kPairsConsistent);
  const double jump = BetaConfig().jump_position_m;
  int origin_gate_would_reject = 0;
  AlignmentOutput prev = o;
  for (int i = 0; i < 600; ++i) {
    const Eigen::Vector3d p_lio = sim.p_lio(sim.t_lio_next);
    o = sim.step();
    ASSERT_EQ(o.state, AlignmentState::kValid) << i;
    ASSERT_EQ(o.reason, AlignmentReason::kPairAccepted) << i;
    EXPECT_LT(o.residual_position_m, 0.05) << i;
    // The PX4 position is noise-free: Apply(truth, p_lio) is where the vehicle really is in PX4 NED.
    EXPECT_LT((Apply(o.filtered, p_lio) - Apply(sim.truth, p_lio)).norm(), 0.05) << i;
    EXPECT_LT(std::abs(Wrap(o.filtered.yaw_rad - sim.truth.yaw_rad)), 0.01) << i;
    ASSERT_TRUE(o.raw.has_value());
    // The S1a origin measure |t_inst - t_filtered| of the same pair: beyond the gate for most pairs.
    if ((Translation(*o.raw) - Translation(prev.filtered)).norm() > jump) ++origin_gate_would_reject;
    prev = o;
  }
  RecordProperty("origin_gate_would_reject", origin_gate_would_reject);
  EXPECT_GT(origin_gate_would_reject, 100) << "the test no longer distinguishes vehicle from origin gating";
}

TEST(Alignment, YawStepRotatesAboutTheVehicle) {
  AlignmentEstimator est(OnePairConfig());
  const AlignmentOutput v = ValidAtFar(est);
  ASSERT_EQ(v.state, AlignmentState::kValid);
  ExpectZero(v.filtered);
  est.on_px4(Px4At(kFarT1, kFar, 0.05));
  const AlignmentOutput o = est.on_lio(LioPoseSample{kFarT1, kFar, 0.0, true});
  EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
  EXPECT_NEAR(o.residual_position_m, 0.0, 1e-12);
  EXPECT_NEAR(o.residual_yaw_rad, 0.05, 1e-12);
  EXPECT_NEAR(o.filtered.yaw_rad, 0.05 * 0.05, 1e-12);
  const Eigen::Vector3d at_vehicle = Apply(o.filtered, kFar);
  EXPECT_NEAR(at_vehicle.x(), 300.0, 1e-9);
  EXPECT_NEAR(at_vehicle.y(), 0.0, 1e-9);
  EXPECT_NEAR(at_vehicle.z(), 0.0, 1e-9);
  // The translation moved (rotation about the vehicle, not about the origin): 300 m x 0.0025 rad.
  EXPECT_GT(Translation(o.filtered).norm(), 0.7);
}

TEST(Alignment, TranslationStepIsAlphaOfTheVehicleResidual) {
  AlignmentEstimator est(OnePairConfig());
  const AlignmentOutput v = ValidAtFar(est);
  ASSERT_EQ(v.state, AlignmentState::kValid);
  ExpectZero(v.filtered);
  est.on_px4(Px4At(kFarT1, {300.2, 0.0, 0.1}, 0.0));
  const AlignmentOutput o = est.on_lio(LioPoseSample{kFarT1, kFar, 0.0, true});
  EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted);
  EXPECT_NEAR(o.residual_position_m, std::hypot(0.2, 0.1), 1e-12);
  const Eigen::Vector3d moved = Apply(o.filtered, kFar) - kFar;
  EXPECT_NEAR(moved.x(), 0.05 * 0.2, 1e-12);
  EXPECT_NEAR(moved.y(), 0.0, 1e-12);
  EXPECT_NEAR(moved.z(), 0.05 * 0.1, 1e-12);
  EXPECT_EQ(o.filtered.yaw_rad, 0.0);
}

TEST(Alignment, TrackingRateIsBoundedByGateOverTau) {  // replaces RateLimitsSlowDrift (D30)
  // jump_position_m x dt / tau = 0.5 x 0.1 / 2 = 0.025 m per 0.1 s scan (0.25 m/s) at the vehicle point.
  const double max_step_m = 0.5 * 0.1 / 2.0 + 1e-9;
  {
    Sim sim;
    AlignmentOutput prev = sim.run_until_valid();
    double largest = 0.0;
    for (int i = 0; i < 300; ++i) {  // 0.24 m/s for 30 s
      sim.truth.x_m += 0.24 * 0.1;
      const Eigen::Vector3d p_lio = sim.p_lio(sim.t_lio_next);
      const AlignmentOutput o = sim.step();
      ASSERT_EQ(o.state, AlignmentState::kValid) << i;
      ASSERT_EQ(o.reason, AlignmentReason::kPairAccepted) << i;
      const double moved = (Apply(o.filtered, p_lio) - Apply(prev.filtered, p_lio)).norm();
      EXPECT_LE(moved, max_step_m) << i;
      largest = std::max(largest, moved);
      prev = o;
    }
    EXPECT_GE(largest, 0.02) << "the drift never approached the gate/tau bound";
  }
  {
    Sim sim;
    sim.v = {0.0, 0.0, 0.0};  // |p_lio| ~ 3.7 m: the vehicle point moves ~0.15 m/s with the yaw drift
    AlignmentOutput prev = sim.run_until_valid();
    for (int i = 0; i < 300; ++i) {  // 0.04 rad/s for 30 s
      sim.truth.yaw_rad = Wrap(sim.truth.yaw_rad + 0.04 * 0.1);
      const Eigen::Vector3d p_lio = sim.p_lio(sim.t_lio_next);
      const AlignmentOutput o = sim.step();
      ASSERT_EQ(o.state, AlignmentState::kValid) << i;
      ASSERT_EQ(o.reason, AlignmentReason::kPairAccepted) << i;
      EXPECT_LE(std::abs(Wrap(o.filtered.yaw_rad - prev.filtered.yaw_rad)), 0.0873 * 0.1 / 2.0 + 1e-12) << i;
      EXPECT_LE((Apply(o.filtered, p_lio) - Apply(prev.filtered, p_lio)).norm(), max_step_m) << i;
      prev = o;
    }
  }
}

TEST(Alignment, DriftFasterThanGateOverTauInvalidates) {
  Sim sim;
  AlignmentOutput prev = sim.run_until_valid();
  std::optional<int> left_valid;
  for (int i = 0; i < 20; ++i) {  // 2 m/s for at most 2 s
    sim.truth.x_m += 2.0 * 0.1;
    const Eigen::Vector3d p_lio = sim.p_lio(sim.t_lio_next);
    const AlignmentOutput o = sim.step();
    if (o.state == AlignmentState::kValid) {
      EXPECT_LE((Apply(o.filtered, p_lio) - Apply(prev.filtered, p_lio)).norm(), 0.025 + 1e-9) << i;
      prev = o;
      continue;
    }
    EXPECT_EQ(o.state, AlignmentState::kInvalid) << i;
    EXPECT_EQ(o.reason, AlignmentReason::kStale) << i;
    left_valid = i;
    break;
  }
  EXPECT_TRUE(left_valid.has_value()) << "still VALID after 2 s of a 2 m/s drift";
}

TEST(AlignmentBoundary, JumpGateIsMeasuredAtTheVehicle) {
  struct Case {
    Eigen::Vector3d p_px4;
    double yaw_px4;
    AlignmentReason expected;
  };
  const Case cases[] = {
      // yaw +0.08 at the vehicle: residual 0 m at the vehicle (|t_inst - t_filtered| would be ~24 m).
      {kFar, 0.08, AlignmentReason::kPairAccepted},
      {{300.5, 0.0, 0.0}, 0.0, AlignmentReason::kPairAccepted},  // exactly 0.5 m at the vehicle: inclusive
      {{std::nextafter(300.5, 400.0), 0.0, 0.0}, 0.0, AlignmentReason::kPairRejectedJump},
  };
  for (const Case& c : cases) {
    AlignmentEstimator est(OnePairConfig());
    const AlignmentOutput v = ValidAtFar(est);
    ASSERT_EQ(v.state, AlignmentState::kValid);
    ExpectZero(v.filtered);
    est.on_px4(Px4At(kFarT1, c.p_px4, c.yaw_px4));
    const AlignmentOutput o = est.on_lio(LioPoseSample{kFarT1, kFar, 0.0, true});
    EXPECT_EQ(o.reason, c.expected) << c.p_px4.transpose() << " yaw " << c.yaw_px4;
    EXPECT_EQ(o.state, AlignmentState::kValid);
    if (c.expected == AlignmentReason::kPairRejectedJump) ExpectZero(o.filtered);  // T unchanged
  }
}

TEST(AlignmentBoundary, InitFinalRecheckIsAtEachMembersOwnVehiclePoint) {
  // Four exact pairs far from the origin, p_lio = p_px4 = (300 + 10 k, 0, 0), yaw_lio = 0, PX4 yaw alternating
  // +-0.04. Arrival residuals 0.400 / 0 / 0.267 m and 0.08 / 0.04 / 0.053 rad pass the gate; the final mean is
  // yaw 0, t = 0, and every member is exact at its OWN vehicle point. Measured at another point (the newest
  // pair's: 1.2 m, the centroid: 0.6 m, the origin: 12 m) the +-0.04 rad members would fail the re-check.
  AlignmentConfig cfg = OnePairConfig();
  cfg.consistent_pairs = 4;
  AlignmentEstimator est(cfg);
  const double yaw_px4[] = {0.04, -0.04, 0.04, -0.04};
  const double arrival_m[] = {0.0, 20.0 * std::sin(0.02), 0.0,
                              40.0 * std::sin(0.5 * std::atan(std::tan(0.04) / 3.0))};
  AlignmentOutput o{};
  for (int k = 0; k < 4; ++k) {
    const time::SensorTime t = kFarT0 + time::milliseconds(100 * k);
    const Eigen::Vector3d p{300.0 + 10.0 * k, 0.0, 0.0};
    est.on_px4(Px4At(t, p, yaw_px4[k]));
    o = est.on_lio(LioPoseSample{t, p, 0.0, true});
    EXPECT_NEAR(o.residual_position_m, arrival_m[k], 1e-9) << k;
    if (k < 3) {
      EXPECT_EQ(o.state, AlignmentState::kInit) << k;
      EXPECT_EQ(o.reason, AlignmentReason::kPairAccepted) << k;
    }
  }
  EXPECT_NEAR(arrival_m[1], 0.400, 1e-3);
  EXPECT_NEAR(arrival_m[3], 0.267, 1e-3);
  ASSERT_EQ(o.state, AlignmentState::kValid);
  EXPECT_EQ(o.reason, AlignmentReason::kPairsConsistent);
  ExpectPoseNear(o.filtered, Pose4{}, 1e-12);
}

TEST(Alignment, HeadingDriftAboutTheVehicleIsTrackedFarFromOrigin) {
  // The PX4 frame turns 1 mrad/s about the VEHICLE (the frame change G of schedule_reset, without a reported
  // reset): the vehicle point does not move, so every pair is accepted and the yaw lags by rate x tau = 2 mrad.
  // At 300 m that lag is 0.6 m of lever arm: the S1a origin measure would have rejected the pairs.
  Sim sim;
  sim.p0 = {300.0, 0.0, -20.0};
  AlignmentOutput prev = sim.run_until_valid();
  constexpr double kDh = 1e-4;  // per 0.1 s scan
  int origin_over_gate = 0;
  for (int i = 0; i < 300; ++i) {
    const double t = sim.t_lio_next;
    const Eigen::Vector3d p_lio = sim.p_lio(t);
    const Eigen::Vector2d pivot = Apply(sim.truth, p_lio).head<2>();
    const Eigen::Vector2d txy = Rot(kDh, Eigen::Vector2d{sim.truth.x_m, sim.truth.y_m} - pivot) + pivot;
    sim.truth = Pose4{txy.x(), txy.y(), sim.truth.z_m, Wrap(sim.truth.yaw_rad + kDh)};
    // The S1a origin residual of this pair, from the inputs: instant T of (PX4 at t, LIO at t) vs filtered T.
    const Px4PoseSample px4 = sim.px4_at(t);
    const double yaw_inst = Wrap(px4.yaw_rad - sim.yaw_lio(t));
    const Eigen::Vector2d t_inst_xy = px4.p_ned_m.head<2>() - Rot(yaw_inst, p_lio.head<2>());
    const Eigen::Vector3d t_inst{t_inst_xy.x(), t_inst_xy.y(), px4.p_ned_m.z() - p_lio.z()};
    if ((t_inst - Translation(prev.filtered)).norm() > 0.5) ++origin_over_gate;
    const AlignmentOutput o = sim.step();
    ASSERT_EQ(o.state, AlignmentState::kValid) << i;
    ASSERT_EQ(o.reason, AlignmentReason::kPairAccepted) << i;
    EXPECT_LT(o.residual_position_m, 0.01) << i;
    prev = o;
  }
  EXPECT_NEAR(Wrap(sim.truth.yaw_rad - prev.filtered.yaw_rad), 1e-3 * 2.0, 0.2 * 2e-3);
  RecordProperty("origin_over_gate", origin_over_gate);
  EXPECT_GT(origin_over_gate, 100);
}

TEST(Alignment, PositionsBeyondTheBoundAreRejected) {
  // |component| > kMaxPositionAbsM (reset deltas: 2 x) is refused like a non-finite value, so no sum in the
  // estimator can overflow to inf - inf = NaN.
  constexpr double kHuge = 1e300;
  const double just_over = std::nextafter(limits::kMaxPositionAbsM, kHuge);
  Sim sim;
  sim.run_until_valid();
  const AlignmentOutput before = sim.step();
  const double t = sim.t_lio_next;
  sim.feed_px4_until(t - 0.03);
  for (int field = 0; field < 6; ++field) {
    Px4PoseSample s = sim.px4_at(sim.t_px4_next);
    switch (field) {
      case 0: s.p_ned_m.x() = kHuge; break;
      case 1: s.p_ned_m.y() = -just_over; break;
      case 2: s.p_ned_m.z() = kHuge; break;
      case 3: s.delta_xy_m.x() = -kHuge; break;
      case 4: s.delta_xy_m.y() = std::nextafter(2.0 * limits::kMaxPositionAbsM, kHuge); break;
      default: s.delta_z_m = kHuge; break;
    }
    const AlignmentOutput o = sim.est.on_px4(s);
    EXPECT_EQ(o.reason, AlignmentReason::kInputRejected) << field;
    EXPECT_EQ(o.state, AlignmentState::kValid) << field;
    ExpectPoseEq(o.filtered, before.filtered);
  }
  sim.feed_px4_until(t + 0.02);
  for (int field = 0; field < 3; ++field) {
    LioPoseSample l = sim.lio_at(t);
    l.p_frd_m(field) = field == 1 ? just_over : (field == 0 ? kHuge : -kHuge);
    const AlignmentOutput o = sim.est.on_lio(l);
    EXPECT_EQ(o.reason, AlignmentReason::kInputRejected) << field;
    EXPECT_EQ(o.state, AlignmentState::kValid) << field;
    ExpectPoseEq(o.filtered, before.filtered);
    ExpectFinite(o);
  }
  // The same scan with its real pose is still accepted (nothing was consumed).
  sim.t_lio_next = t;
  EXPECT_EQ(sim.step().reason, AlignmentReason::kPairAccepted);
}

TEST(Alignment, NanGateThresholdFailsClosed) {
  // A NaN in a gate comparison must reject, never accept: the gates are written !(value <= threshold).
  // A NaN threshold is the only way to reach that comparison with valid inputs (the config is then invalid,
  // so VALID is blocked anyway); no pair after the first may be reported accepted.
  for (int which = 0; which < 2; ++which) {
    AlignmentConfig bad = BetaConfig();
    if (which == 0) bad.jump_position_m = std::numeric_limits<double>::quiet_NaN();
    if (which == 1) bad.jump_yaw_rad = std::numeric_limits<double>::quiet_NaN();
    Sim sim(bad);
    EXPECT_EQ(sim.step().reason, AlignmentReason::kPairAccepted) << which;  // empty accumulation: no comparison
    for (int i = 0; i < 50; ++i) {
      const AlignmentOutput o = sim.step();
      EXPECT_EQ(o.state, AlignmentState::kInit) << which << " " << i;
      EXPECT_EQ(o.reason, AlignmentReason::kPairRejectedJump) << which << " " << i;
    }
  }
}

// ---------------------------------------------------------------------------------------------------------
// Allow-list fuzz: random but VALID input sequences (monotonic stamps, finite values). Every state change
// observed in an output must be a listed edge carrying the output's reason, an input that is valid is never
// reported kInputRejected (which would reveal a refused, unlisted edge), and every listed edge is reached.

TEST(AlignmentFuzz, EveryStateChangeIsAListedEdge) {
  using Edge = std::tuple<AlignmentState, AlignmentState, AlignmentReason>;
  auto listed = [](const Edge& e) {
    for (const auto& l : kAlignmentTransitions) {
      if (l.from == std::get<0>(e) && l.to == std::get<1>(e) && l.reason == std::get<2>(e)) return true;
    }
    return false;
  };
  AlignmentConfig cfg = BetaConfig();
  cfg.consistent_pairs = 5;
  cfg.valid_stale = time::milliseconds(500);
  cfg.frozen_max = time::seconds(2);
  std::set<Edge> seen;
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    Sim sim(cfg);
    sim.v = {1.0, 0.5, 0.0};
    bool lost = false;
    double t = 100.0;
    auto check = [&](AlignmentState before, const AlignmentOutput& o, bool lio_lost_sample) {
      ExpectFinite(o);
      ASSERT_NE(o.reason, AlignmentReason::kInputRejected) << "seed " << seed << " t " << t;
      if (lio_lost_sample) {
        ASSERT_NE(o.state, AlignmentState::kValid);
      }
      if (o.state != before) {
        const Edge e{before, o.state, o.reason};
        ASSERT_TRUE(listed(e)) << "seed " << seed << " " << to_string(before) << " -> " << to_string(o.state)
                               << " (" << to_string(o.reason) << ")";
        seen.insert(e);
      }
    };
    for (int step = 0; step < 6000; ++step) {
      t += 0.01;
      const double g = u(rng);
      if (g < 0.002) t += 1.5;          // a gap longer than valid_stale
      else if (g < 0.0025) t += 3.0;    // a gap longer than frozen_max
      if (u(rng) < 0.002) sim.truth.x_m += 2.0;  // a jump of the truth (PX4 re-initialised)
      if (u(rng) < 0.95) {
        sim.t_px4_next = t;
        if (u(rng) < 0.004) {
          const auto step_of = [&] { return static_cast<std::uint8_t>(u(rng) < 0.5 ? 0 : 1 + static_cast<int>(u(rng) * 3)); };
          sim.schedule_reset(step_of(), step_of(), step_of(), {u(rng) - 0.5, u(rng) - 0.5}, u(rng) - 0.5,
                             0.2 * (u(rng) - 0.5));
        }
        const AlignmentState before = sim.est.state();
        check(before, sim.est.on_px4(sim.px4_at(t)), false);
      }
      if (step % 10 == 3) {
        if (u(rng) < (lost ? 0.08 : 0.03)) lost = !lost;
        const AlignmentState before = sim.est.state();
        // The scan is stamped in the past (LIO latency), so PX4 samples exist on both sides of it.
        check(before, sim.est.on_lio(sim.lio_at(t - 0.0495, !lost)), lost);
      }
      if (step % 5 == 0) {
        const AlignmentState before = sim.est.state();
        check(before, sim.est.on_tick(At(t + 0.001)), false);
      }
      if (::testing::Test::HasFatalFailure()) return;
    }
  }
  for (const auto& l : kAlignmentTransitions) {
    EXPECT_TRUE(seen.contains(Edge{l.from, l.to, l.reason}))
        << "fuzz never reached " << to_string(l.from) << " -> " << to_string(l.to) << " (" << to_string(l.reason)
        << ")";
  }
}
