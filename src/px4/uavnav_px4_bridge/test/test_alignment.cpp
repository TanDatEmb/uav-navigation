#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
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
  return AlignmentConfig{time::seconds(2), 20U, 0.5, 0.0873, 0.5, 0.0873, time::seconds(1), time::seconds(10)};
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

// Per-step bound on the filtered T: |translation step| <= max_rate * dt, |yaw step| <= max_yaw_rate * dt.
// Returns whether a step reached the bound (so the limiter was actually exercised).
struct DriftResult {
  bool translation_limited{false};
  bool yaw_limited{false};
  double moved_m{0.0};
};

DriftResult DriftAndCheck(AlignmentConfig cfg, double drift_mps, double drift_rad_s) {
  Sim sim(cfg);
  AlignmentOutput prev = sim.run_until_valid();
  const AlignmentOutput start = prev;
  DriftResult r;
  for (int i = 0; i < 30; ++i) {
    sim.truth.x_m += drift_mps * 0.1;
    sim.truth.yaw_rad = Wrap(sim.truth.yaw_rad + drift_rad_s * 0.1);
    const AlignmentOutput o = sim.step();
    if (o.state != AlignmentState::kValid) break;
    const double dx = o.filtered.x_m - prev.filtered.x_m;
    const double dy = o.filtered.y_m - prev.filtered.y_m;
    const double dz = o.filtered.z_m - prev.filtered.z_m;
    const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double yaw_step = std::abs(Wrap(o.filtered.yaw_rad - prev.filtered.yaw_rad));
    // dt between scans is 0.1 s up to the nanosecond rounding of the synthetic stamps.
    EXPECT_LE(step, cfg.max_rate_mps * 0.1 + 1e-8) << "step " << i;
    EXPECT_LE(yaw_step, cfg.max_yaw_rate_rad_s * 0.1 + 1e-9) << "step " << i;
    if (step > cfg.max_rate_mps * 0.1 - 1e-9) r.translation_limited = true;
    if (yaw_step > cfg.max_yaw_rate_rad_s * 0.1 - 1e-9) r.yaw_limited = true;
    prev = o;
  }
  r.moved_m = std::abs(prev.filtered.x_m - start.filtered.x_m);
  return r;
}

TEST(Alignment, RateLimitsSlowDrift) {
  // The brief's case with the beta config: the truth drifts 2 m/s, the filtered T moves <= 0.5 m/s.
  // (With the beta values the bound also follows from jump/tau = 0.25 m/s: see the report.)
  const DriftResult beta = DriftAndCheck(BetaConfig(), 2.0, 0.0);
  EXPECT_GT(beta.moved_m, 0.0);
  // tau = 0.2 s (alpha = 0.5 per scan): the limiter itself is what bounds the step.
  AlignmentConfig fast = BetaConfig();
  fast.tau = time::milliseconds(200);
  const DriftResult t = DriftAndCheck(fast, 2.0, 0.0);
  EXPECT_TRUE(t.translation_limited) << "the translation limiter never engaged";
  const DriftResult y = DriftAndCheck(fast, 0.0, 0.5);
  EXPECT_TRUE(y.yaw_limited) << "the yaw limiter never engaged";
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
    EXPECT_LE(std::abs(Wrap(o.filtered.yaw_rad - prev.filtered.yaw_rad)), 0.0873 * 0.1 + 1e-9);
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
