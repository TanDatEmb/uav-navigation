#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "uavnav/core/result.hpp"
#include "uavnav/lio/limits.hpp"
#include "uavnav/lio/output_predictor.hpp"

using namespace uavnav;
using namespace uavnav::lio;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// Sign convention used by every test (see output_predictor.hpp): gravity_world is the world gravity
// vector (0, 0, -9.81) in a z-up world, delta_velocity_mps is the accelerometer's specific force
// integrated over the step in the IMU (body) frame, so world acceleration = R * f + g_world. A body at
// rest therefore reads f = (0, 0, +9.81): the "gravity-compensating dv" below.
namespace {

constexpr double kG = 9.81;
constexpr std::int64_t kImuNs = 5'000'000;  // 200 Hz
constexpr double kImuS = 0.005;
const Vector3d kGravity(0.0, 0.0, -kG);

time::SensorTime At(std::int64_t ns) { return time::SensorTime{ns}; }

PredictorConfig Beta() { return PredictorConfig{time::milliseconds(250), time::milliseconds(250)}; }

EstimatorSnapshot Snap(time::SensorTime t, const Vector3d& p = Vector3d::Zero(), const Vector3d& v = Vector3d::Zero(),
                       const Quaterniond& q = Quaterniond::Identity()) {
  return EstimatorSnapshot{t, q, v, p, Vector3d::Zero(), Vector3d::Zero(), kGravity};
}

ImuDelta Stationary(time::SensorTime t, double dt = kImuS) {
  return ImuDelta{t, Vector3d::Zero(), Vector3d(0.0, 0.0, kG * dt), dt};
}

Quaterniond Yaw(double rad) { return Quaterniond(Eigen::AngleAxisd(rad, Vector3d::UnitZ())); }

double YawOf(const Quaterniond& q) {
  const Eigen::Matrix3d r = q.toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}

bool Finite(const OutputSample& s) {
  return s.q_world_imu.coeffs().allFinite() && s.v_world_mps.allFinite() && s.p_world_m.allFinite();
}

void ExpectSame(const OutputSample& a, const OutputSample& b) {
  EXPECT_EQ(a.t, b.t);
  EXPECT_EQ(a.q_world_imu.coeffs(), b.q_world_imu.coeffs());
  EXPECT_EQ(a.v_world_mps, b.v_world_mps);
  EXPECT_EQ(a.p_world_m, b.p_world_m);
  EXPECT_EQ(a.reset_counter, b.reset_counter);
}

// Truth: constant yaw rate about world z and constant world acceleration, starting at t0. The IMU
// deltas are generated so the predictor's integrator (attitude first, then dv rotated by the attitude at
// the END of the step, trapezoidal position) reproduces the truth exactly at every sample.
struct Motion {
  std::int64_t t0_ns{1'000'000'000};
  double yaw_rate{0.5};
  Vector3d v0{1.0, 0.0, 0.0};
  Vector3d a{0.5, -0.2, 0.1};

  double Tau(std::int64_t ns) const { return static_cast<double>(ns - t0_ns) / 1e9; }
  Quaterniond Q(std::int64_t ns) const { return Yaw(yaw_rate * Tau(ns)); }
  Vector3d V(std::int64_t ns) const { return v0 + a * Tau(ns); }
  Vector3d P(std::int64_t ns) const { return v0 * Tau(ns) + 0.5 * a * Tau(ns) * Tau(ns); }
  EstimatorSnapshot Snapshot(std::int64_t ns) const { return Snap(At(ns), P(ns), V(ns), Q(ns)); }
  ImuDelta Delta(std::int64_t end_ns, double dt = kImuS) const {
    const Vector3d f_body = Q(end_ns).conjugate() * (a - kGravity);
    return ImuDelta{At(end_ns), Vector3d(0.0, 0.0, yaw_rate * dt), f_body * dt, dt};
  }
};

}  // namespace

// --- the brief's six ------------------------------------------------------------------------------

TEST(OutputPredictor, IntegratesConstantVelocityWithoutCorrections) {
  OutputPredictor op(Beta());
  op.align(Snap(At(0), Vector3d::Zero(), Vector3d(1.0, 0.0, 0.0)));
  std::optional<OutputSample> out;
  for (std::int64_t k = 1; k <= 200; ++k) out = op.on_imu(Stationary(At(k * kImuNs)));
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->t, At(200 * kImuNs));
  EXPECT_NEAR(out->p_world_m.x(), 1.0, 1e-3);
  EXPECT_NEAR(out->p_world_m.y(), 0.0, 1e-9);
  EXPECT_NEAR(out->p_world_m.z(), 0.0, 1e-9);
  EXPECT_NEAR((out->v_world_mps - Vector3d(1.0, 0.0, 0.0)).norm(), 0.0, 1e-9);
  EXPECT_EQ(out->reset_counter, 0U);
}

TEST(OutputPredictor, CorrectionConvergesWithoutJump) {
  // Aligned at the origin, at rest; the estimator says the body is at (0.5, 0, 0). A correction stamped
  // 50 ms (10 samples) in the past arrives every 100 ms. The whole-buffer PI correction must move the
  // output gradually: no step between consecutive outputs above 0.05 m, and within 0.05 m after 2 s.
  const Vector3d truth(0.5, 0.0, 0.0);
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  Vector3d last = Vector3d::Zero();
  double max_step = 0.0;
  double max_vel_step = 0.0;
  Vector3d last_v = Vector3d::Zero();
  std::optional<OutputSample> out;
  for (std::int64_t k = 1; k <= 400; ++k) {
    out = op.on_imu(Stationary(At(k * kImuNs)));
    ASSERT_TRUE(out.has_value());
    max_step = std::max(max_step, (out->p_world_m - last).norm());
    max_vel_step = std::max(max_vel_step, (out->v_world_mps - last_v).norm());
    last = out->p_world_m;
    last_v = out->v_world_mps;
    if (k % 20 == 0) {
      const auto r = op.on_correction(Snap(At((k - 10) * kImuNs), truth));
      ASSERT_TRUE(r.has_value()) << to_string(r.error());
    }
  }
  EXPECT_LT(max_step, 0.05);
  EXPECT_LT(max_vel_step, 0.05);
  EXPECT_LT((out->p_world_m - truth).norm(), 0.05) << out->p_world_m.transpose();
}

TEST(OutputPredictor, CorrectionMatchedByTimestampNotOldest) {
  // 250 ms of truth-consistent IMU after the align: 51 buffered samples (< capacity), so the oldest is
  // the align sample at t0. A correction carrying the exact truth at newest - 150 ms must see zero error
  // there; a predictor comparing against the oldest sample would see ~0.1 m, ~0.05 rad and 0.05 m/s.
  const Motion m;
  OutputPredictor op(Beta());
  op.align(m.Snapshot(m.t0_ns));
  std::int64_t now = m.t0_ns;
  for (int k = 1; k <= 50; ++k) {
    now += kImuNs;
    ASSERT_TRUE(op.on_imu(m.Delta(now)).has_value());
  }
  const std::int64_t delayed = now - 150'000'000;
  const auto r = op.on_correction(m.Snapshot(delayed));
  ASSERT_TRUE(r.has_value()) << to_string(r.error());
  const Vector3d e = op.tracking_error();
  EXPECT_LT(e(0), 1e-9);
  EXPECT_LT(e(1), 1e-9);
  EXPECT_LT(e(2), 1e-9);
}

TEST(OutputPredictor, RejectsCorrectionOlderThanBuffer) {
  OutputPredictor op(Beta());
  OutputPredictor twin(Beta());
  op.align(Snap(At(0)));
  twin.align(Snap(At(0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 200; ++k) {
    now += kImuNs;
    op.on_imu(Stationary(At(now)));
    twin.on_imu(Stationary(At(now)));
  }
  const auto r = op.on_correction(Snap(At(now - 500'000'000), Vector3d(3.0, 2.0, 1.0)));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), PredictorReason::kOlderThanBuffer);
  EXPECT_EQ(op.tracking_error(), twin.tracking_error());
  for (int k = 1; k <= 40; ++k) {
    now += kImuNs;
    const auto a = op.on_imu(Stationary(At(now)));
    const auto b = twin.on_imu(Stationary(At(now)));
    ASSERT_TRUE(a.has_value() && b.has_value());
    ExpectSame(*a, *b);
  }
}

TEST(OutputPredictor, ResetReportsDeltaAndCounter) {
  const Vector3d p0(1.0, 2.0, 3.0);
  const Vector3d shift(4.0, -5.0, 6.0);
  OutputPredictor op(Beta());
  op.align(Snap(At(0), p0));
  std::int64_t now = 0;
  for (int k = 1; k <= 20; ++k) {
    now += kImuNs;
    op.on_imu(Stationary(At(now)));
  }
  const ResetDelta d = op.reset_to(Snap(At(now - 50'000'000), p0 + shift));
  EXPECT_NEAR((d.position_m - shift).norm(), 0.0, 1e-9);
  EXPECT_NEAR(d.velocity_mps.norm(), 0.0, 1e-9);
  EXPECT_NEAR(d.yaw_rad, 0.0, 1e-12);
  const auto out = op.on_imu(Stationary(At(now + kImuNs)));
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->reset_counter, 1U);
  EXPECT_NEAR((out->p_world_m - (p0 + shift)).norm(), 0.0, 1e-9);
}

TEST(OutputPredictor, JitteryDtIsClamped) {
  // (a) dt 0.0 and 0.1 interleaved on a regular stamp grid: never NaN, also through a correction with
  // zero delay (where an unclamped dt average of 0 would make the attitude gain 0/0).
  {
    OutputPredictor op(Beta());
    op.align(Snap(At(0)));
    std::int64_t now = 0;
    for (int k = 1; k <= 100; ++k) {
      now += kImuNs;
      const auto out = op.on_imu(Stationary(At(now), (k % 2 == 0) ? 0.1 : 0.0));
      ASSERT_TRUE(out.has_value());
      ASSERT_TRUE(Finite(*out)) << k;
    }
    const auto r = op.on_correction(Snap(At(now), Vector3d(0.1, 0.0, 0.0), Vector3d::Zero(), Yaw(0.01)));
    ASSERT_TRUE(r.has_value()) << to_string(r.error());
    EXPECT_TRUE(op.tracking_error().allFinite());
    for (int k = 1; k <= 20; ++k) {
      now += kImuNs;
      const auto out = op.on_imu(Stationary(At(now), 0.0));
      ASSERT_TRUE(out.has_value());
      ASSERT_TRUE(Finite(*out)) << k;
    }
  }
  // (b) dt 0.1 is clamped to 0.03 in the averaged IMU dt that sets the attitude gain 0.5 * dt / delay:
  // after a 0.01 rad yaw error observed 200 ms back, one zero-rate IMU step turns the output by
  // 0.5 * 0.03 / 0.2 * 0.01 = 7.5e-4 rad (unclamped it would be 2.5e-3 rad).
  {
    OutputPredictor op(Beta());
    op.align(Snap(At(0)));
    std::int64_t now = 0;
    for (int k = 1; k <= 60; ++k) {
      now += 100'000'000;
      ASSERT_TRUE(op.on_imu(Stationary(At(now), 0.1)).has_value());
    }
    const auto r = op.on_correction(Snap(At(now - 200'000'000), Vector3d::Zero(), Vector3d::Zero(), Yaw(0.01)));
    ASSERT_TRUE(r.has_value()) << to_string(r.error());
    const auto out = op.on_imu(Stationary(At(now + 100'000'000), 0.1));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(YawOf(out->q_world_imu), 0.5 * 0.03 / 0.2 * 0.01, 1e-7);
  }
  // (c) dt 0.0 is clamped to 1e-4: with the error 20 ms back the gain is 0.5 * 1e-4 / 0.02, so one
  // zero-length step still turns the output by 2.5e-5 rad (unclamped: 0).
  {
    OutputPredictor op(Beta());
    op.align(Snap(At(0)));
    std::int64_t now = 0;
    for (int k = 1; k <= 60; ++k) {
      now += 1'000'000;
      ASSERT_TRUE(op.on_imu(Stationary(At(now), 0.0)).has_value());
    }
    const auto r = op.on_correction(Snap(At(now - 20'000'000), Vector3d::Zero(), Vector3d::Zero(), Yaw(0.01)));
    ASSERT_TRUE(r.has_value()) << to_string(r.error());
    const auto out = op.on_imu(Stationary(At(now + 1'000'000), 0.0));
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(YawOf(out->q_world_imu), 0.5 * 1e-4 / 0.02 * 0.01, 1e-8);
  }
}

// --- attitude correction window (third difference from PX4) ---------------------------------------

TEST(OutputPredictor, HeldAttitudeCorrectionStopsAfterTheDelayWindow) {
  // One applied 0.01 rad yaw correction (delay 50 ms), then 2 s of IMU with NO further corrections. The
  // held delta-angle correction must act for one delay's worth of IMU steps only (removing ~0.5 of the
  // error, PX4's per-delay share) and then stop: no false angular rate once corrections stop.
  const double err = 0.01;
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 20; ++k) op.on_imu(Stationary(At(now += kImuNs)));
  ASSERT_TRUE(op.on_correction(Snap(At(now - 50'000'000), Vector3d::Zero(), Vector3d::Zero(), Yaw(err))).has_value());
  std::optional<OutputSample> out;
  double yaw_at_1_5_s = 0.0;
  for (int k = 1; k <= 400; ++k) {
    out = op.on_imu(Stationary(At(now += kImuNs)));
    ASSERT_TRUE(out.has_value());
    if (k == 300) yaw_at_1_5_s = YawOf(out->q_world_imu);
  }
  const double yaw = YawOf(out->q_world_imu);
  EXPECT_LE(yaw, 0.6 * err);
  EXPECT_GE(yaw, 0.4 * err);
  EXPECT_NEAR(yaw - yaw_at_1_5_s, 0.0, 1e-12) << "output still rotating after the correction window";
}

TEST(OutputPredictor, AttitudeTrackingDoesNotOvershootAtScanRate) {
  // 200 Hz IMU, 10 Hz corrections of a constant 0.01 rad yaw error, delay 20 / 50 / 100 ms: the attitude
  // error never exceeds the initial error by more than 20 % and is below 10 % of it within 2 s.
  const double err = 0.01;
  for (const std::int64_t delay_ns : {20'000'000LL, 50'000'000LL, 100'000'000LL}) {
    OutputPredictor op(Beta());
    op.align(Snap(At(0)));
    double peak = 0.0;
    double last = err;
    for (std::int64_t k = 1; k <= 400; ++k) {
      const auto out = op.on_imu(Stationary(At(k * kImuNs)));
      ASSERT_TRUE(out.has_value());
      last = out->q_world_imu.angularDistance(Yaw(err));
      peak = std::max(peak, last);
      if (k % 20 == 0) {
        const auto r = op.on_correction(Snap(At(k * kImuNs - delay_ns), Vector3d::Zero(), Vector3d::Zero(), Yaw(err)));
        ASSERT_TRUE(r.has_value()) << to_string(r.error());
      }
    }
    EXPECT_LE(peak, 1.2 * err) << "delay " << delay_ns;
    EXPECT_LT(last, 0.1 * err) << "delay " << delay_ns;
  }
}

// --- lookup and boundaries ------------------------------------------------------------------------

TEST(OutputPredictor, LooksUpTheClosestSampleAtOrBeforeTheStamp) {
  // Stamp 4 ms after a sample (1 ms before the next one). The match is the sample AT OR BEFORE the stamp,
  // so a snapshot equal to the truth at that earlier sample gives zero error; matching the nearer later
  // sample would not.
  const Motion m;
  OutputPredictor op(Beta());
  op.align(m.Snapshot(m.t0_ns));
  std::int64_t now = m.t0_ns;
  for (int k = 1; k <= 50; ++k) {
    now += kImuNs;
    op.on_imu(m.Delta(now));
  }
  const std::int64_t sample = m.t0_ns + 20 * kImuNs;
  EstimatorSnapshot s = m.Snapshot(sample);
  s.t = At(sample + 4'000'000);
  const auto r = op.on_correction(s);
  ASSERT_TRUE(r.has_value()) << to_string(r.error());
  EXPECT_LT(op.tracking_error().maxCoeff(), 1e-9);
}

TEST(OutputPredictor, RejectsWhenNoSampleWithinOneImuPeriod) {
  // An IMU gap of 100 ms inside the buffer: a stamp in the gap has no sample within one IMU period
  // (the last ImuDelta dt, 5 ms) at or before it, so it is rejected and nothing changes. Exactly one
  // period after the last sample before the gap is still accepted.
  const Motion m;
  OutputPredictor op(Beta());
  OutputPredictor twin(Beta());
  op.align(m.Snapshot(m.t0_ns));
  twin.align(m.Snapshot(m.t0_ns));
  std::int64_t now = m.t0_ns;
  auto feed = [&](std::int64_t t) {
    op.on_imu(m.Delta(t));
    twin.on_imu(m.Delta(t));
  };
  for (int k = 1; k <= 20; ++k) feed(now += kImuNs);  // up to t0 + 100 ms
  const std::int64_t before_gap = now;
  now += 100'000'000;
  for (int k = 1; k <= 10; ++k) feed(now += kImuNs);
  const auto in_gap = op.on_correction(m.Snapshot(before_gap + 50'000'000));
  ASSERT_FALSE(in_gap.has_value());
  EXPECT_EQ(in_gap.error(), PredictorReason::kOlderThanBuffer);
  const auto a = op.on_imu(m.Delta(now + kImuNs));
  const auto b = twin.on_imu(m.Delta(now + kImuNs));
  ASSERT_TRUE(a.has_value() && b.has_value());
  ExpectSame(*a, *b);
  now += kImuNs;

  EstimatorSnapshot s = m.Snapshot(before_gap);
  s.t = At(before_gap + kImuNs);
  EXPECT_TRUE(op.on_correction(s).has_value());
  s.t = At(before_gap + kImuNs + 1);
  EXPECT_EQ(op.on_correction(s).error(), PredictorReason::kOlderThanBuffer);
}

TEST(OutputPredictor, CorrectionAtOldestSampleIsAcceptedOneNanosecondOlderIsNot) {
  OutputPredictor op(Beta());
  op.align(Snap(At(1'000)));
  for (int k = 1; k <= 10; ++k) op.on_imu(Stationary(At(1'000 + k * kImuNs)));
  const auto older = op.on_correction(Snap(At(999)));
  ASSERT_FALSE(older.has_value());
  EXPECT_EQ(older.error(), PredictorReason::kOlderThanBuffer);
  EXPECT_TRUE(op.on_correction(Snap(At(1'000))).has_value());
}

TEST(OutputPredictor, CorrectionAtSpanLimitIsAcceptedOneNanosecondOlderIsNot) {
  // Buffer full at 200 Hz (64 samples = 315 ms), so only the 300 ms span rule rejects here.
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 100; ++k) op.on_imu(Stationary(At(now += kImuNs)));
  const std::int64_t limit = now - limits::kPredictorBufferSpan.ns;
  const auto older = op.on_correction(Snap(At(limit - 1)));
  ASSERT_FALSE(older.has_value());
  EXPECT_EQ(older.error(), PredictorReason::kOlderThanBuffer);
  EXPECT_TRUE(op.on_correction(Snap(At(limit))).has_value());
}

TEST(OutputPredictor, CorrectionNewerThanNewestOutputIsRejected) {
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 10; ++k) op.on_imu(Stationary(At(now += kImuNs)));
  const auto r = op.on_correction(Snap(At(now + 1)));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), PredictorReason::kNewerThanOutput);
  EXPECT_TRUE(op.on_correction(Snap(At(now))).has_value());
}

TEST(OutputPredictor, CorrectionBeforeAlignIsNotInitialized) {
  OutputPredictor op(Beta());
  EXPECT_FALSE(op.on_imu(Stationary(At(kImuNs))).has_value());
  const auto r = op.on_correction(Snap(At(0)));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), PredictorReason::kNotInitialized);
  EXPECT_EQ(op.tracking_error(), Vector3d::Zero());
}

// --- non-finite input -----------------------------------------------------------------------------

TEST(OutputPredictor, NonFiniteOrUnusableImuDeltaIsIgnored) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  OutputPredictor op(Beta());
  OutputPredictor twin(Beta());
  op.align(Snap(At(0), Vector3d::Zero(), Vector3d(1.0, 0.0, 0.0)));
  twin.align(Snap(At(0), Vector3d::Zero(), Vector3d(1.0, 0.0, 0.0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 10; ++k) {
    op.on_imu(Stationary(At(now += kImuNs)));
    twin.on_imu(Stationary(At(now)));
  }
  std::vector<ImuDelta> bad;
  for (int i = 0; i < 3; ++i) {
    ImuDelta d = Stationary(At(now + kImuNs));
    d.delta_angle_rad(i) = nan;
    bad.push_back(d);
    d = Stationary(At(now + kImuNs));
    d.delta_velocity_mps(i) = inf;
    bad.push_back(d);
  }
  ImuDelta d = Stationary(At(now + kImuNs));
  d.dt_s = nan;
  bad.push_back(d);
  d.dt_s = inf;
  bad.push_back(d);
  d.dt_s = -0.005;
  bad.push_back(d);
  // Longer than the whole buffer span: not a delta between consecutive IMU samples.
  bad.push_back(Stationary(At(now + 600'000'000), 0.6));
  // Stamp not after the newest output (duplicate / out of order).
  bad.push_back(Stationary(At(now)));
  bad.push_back(Stationary(At(now - kImuNs)));
  for (const ImuDelta& b : bad) EXPECT_FALSE(op.on_imu(b).has_value());

  const auto a = op.on_imu(Stationary(At(now + kImuNs)));
  const auto c = twin.on_imu(Stationary(At(now + kImuNs)));
  ASSERT_TRUE(a.has_value() && c.has_value());
  ExpectSame(*a, *c);
  EXPECT_TRUE(Finite(*a));
}

TEST(OutputPredictor, NonFiniteSnapshotIsRejectedAndChangesNothing) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  OutputPredictor op(Beta());
  OutputPredictor twin(Beta());
  op.align(Snap(At(0)));
  twin.align(Snap(At(0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 20; ++k) {
    op.on_imu(Stationary(At(now += kImuNs)));
    twin.on_imu(Stationary(At(now)));
  }
  const EstimatorSnapshot good = Snap(At(now - 50'000'000), Vector3d(0.3, 0.0, 0.0));
  std::vector<EstimatorSnapshot> bad;
  for (int i = 0; i < 3; ++i) {
    EstimatorSnapshot s = good;
    s.p_world_m(i) = nan;
    bad.push_back(s);
    s = good;
    s.v_world_mps(i) = nan;
    bad.push_back(s);
    s = good;
    s.gyro_bias(i) = nan;
    bad.push_back(s);
    s = good;
    s.accel_bias(i) = nan;
    bad.push_back(s);
    s = good;
    s.gravity_world(i) = std::numeric_limits<double>::infinity();
    bad.push_back(s);
  }
  EstimatorSnapshot s = good;
  s.q_world_imu.w() = nan;
  bad.push_back(s);
  s = good;
  s.q_world_imu = Quaterniond(0.0, 0.0, 0.0, 0.0);  // no rotation can be recovered from it
  bad.push_back(s);
  for (const EstimatorSnapshot& b : bad) {
    const auto r = op.on_correction(b);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), PredictorReason::kNonFinite);
  }
  EXPECT_EQ(op.tracking_error(), twin.tracking_error());
  for (int k = 1; k <= 20; ++k) {
    const auto a = op.on_imu(Stationary(At(now += kImuNs)));
    const auto b = twin.on_imu(Stationary(At(now)));
    ASSERT_TRUE(a.has_value() && b.has_value());
    ExpectSame(*a, *b);
  }

  // A non-finite snapshot is ignored by align and reset_to too.
  OutputPredictor fresh(Beta());
  fresh.align(bad.front());
  EXPECT_FALSE(fresh.on_imu(Stationary(At(kImuNs))).has_value());
  const ResetDelta d = op.reset_to(bad.front());
  EXPECT_EQ(d.position_m, Vector3d::Zero());
  EXPECT_EQ(d.velocity_mps, Vector3d::Zero());
  EXPECT_EQ(d.yaw_rad, 0.0);
  const auto out = op.on_imu(Stationary(At(now += kImuNs)));
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->reset_counter, 0U);
}

// --- sequences ------------------------------------------------------------------------------------

TEST(OutputPredictor, TwoConsecutiveCorrectionsAreBothApplied) {
  const Vector3d truth(0.5, 0.0, 0.0);
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 20; ++k) op.on_imu(Stationary(At(now += kImuNs)));
  ASSERT_TRUE(op.on_correction(Snap(At(now - 50'000'000), truth)).has_value());
  const double first = op.tracking_error()(2);
  EXPECT_NEAR(first, 0.5, 1e-9);
  ASSERT_TRUE(op.on_correction(Snap(At(now - 45'000'000), truth)).has_value());
  const double second = op.tracking_error()(2);
  EXPECT_LT(second, first);
  EXPECT_GT(second, 0.0);
  const auto out = op.on_imu(Stationary(At(now + kImuNs)));
  ASSERT_TRUE(out.has_value());
  EXPECT_GT(out->p_world_m.x(), 0.0);
  EXPECT_LT(out->p_world_m.x(), 0.5);
}

TEST(OutputPredictor, StallThenOldCorrectionIsRejectedAndOutputStaysContinuous) {
  // Review Focus 2. Moving at 1 m/s. (a) Processing stall: IMU keeps flowing for 600 ms while the
  // estimator is stuck; its correction then arrives with the pre-stall scan stamp. (b) IMU stall: no
  // IMU at all for 600 ms, then IMU resumes and the old correction arrives. Both: kOlderThanBuffer, and
  // the next output continues from the last one by exactly one integration step (no jump, no rewind).
  const Vector3d v(1.0, 0.0, 0.0);
  for (const bool imu_flows : {true, false}) {
    OutputPredictor op(Beta());
    op.align(Snap(At(0), Vector3d::Zero(), v));
    std::int64_t now = 0;
    for (int k = 1; k <= 200; ++k) op.on_imu(Stationary(At(now += kImuNs)));
    const std::int64_t scan_stamp = now - 50'000'000;
    std::optional<OutputSample> last;
    if (imu_flows) {
      for (int k = 1; k <= 120; ++k) last = op.on_imu(Stationary(At(now += kImuNs)));
    } else {
      now += 600'000'000;
      last = op.on_imu(Stationary(At(now)));
    }
    ASSERT_TRUE(last.has_value());
    const auto r = op.on_correction(Snap(At(scan_stamp), Vector3d(10.0, 10.0, 10.0), Vector3d::Zero(), Yaw(1.0)));
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), PredictorReason::kOlderThanBuffer);
    const auto next = op.on_imu(Stationary(At(now + kImuNs)));
    ASSERT_TRUE(next.has_value());
    EXPECT_GT(next->t, last->t);
    EXPECT_NEAR((next->p_world_m - last->p_world_m - v * kImuS).norm(), 0.0, 1e-9) << imu_flows;
    EXPECT_NEAR((next->v_world_mps - last->v_world_mps).norm(), 0.0, 1e-9) << imu_flows;
    EXPECT_NEAR(next->q_world_imu.angularDistance(last->q_world_imu), 0.0, 1e-12) << imu_flows;
  }
}

TEST(OutputPredictor, ResetTwiceCountsTwo) {
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 10; ++k) op.on_imu(Stationary(At(now += kImuNs)));
  op.reset_to(Snap(At(now), Vector3d(1.0, 0.0, 0.0)));
  const ResetDelta d = op.reset_to(Snap(At(now), Vector3d(1.5, 0.0, 0.0)));
  EXPECT_NEAR((d.position_m - Vector3d(0.5, 0.0, 0.0)).norm(), 0.0, 1e-9);
  const auto out = op.on_imu(Stationary(At(now + kImuNs)));
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->reset_counter, 2U);
}

TEST(OutputPredictor, ResetYawDeltaIsTheShortestArc) {
  OutputPredictor op(Beta());
  op.align(Snap(At(0), Vector3d::Zero(), Vector3d::Zero(), Yaw(3.0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 10; ++k) op.on_imu(Stationary(At(now += kImuNs)));
  const ResetDelta d = op.reset_to(Snap(At(now), Vector3d::Zero(), Vector3d::Zero(), Yaw(-3.0)));
  EXPECT_NEAR(d.yaw_rad, 2.0 * std::numbers::pi - 6.0, 1e-9);
  const ResetDelta back = op.reset_to(Snap(At(now), Vector3d::Zero(), Vector3d::Zero(), Yaw(-2.7)));
  EXPECT_NEAR(back.yaw_rad, 0.3, 1e-9);
}

TEST(OutputPredictor, ResetNewerThanOutputRestartsAtTheSnapshot) {
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 10; ++k) op.on_imu(Stationary(At(now += kImuNs)));
  const ResetDelta d = op.reset_to(Snap(At(now + 100'000'000), Vector3d(2.0, 0.0, 0.0), Vector3d(0.5, 0.0, 0.0)));
  EXPECT_NEAR((d.position_m - Vector3d(2.0, 0.0, 0.0)).norm(), 0.0, 1e-9);
  EXPECT_NEAR((d.velocity_mps - Vector3d(0.5, 0.0, 0.0)).norm(), 0.0, 1e-9);
  EXPECT_FALSE(op.on_imu(Stationary(At(now + kImuNs))).has_value());  // before the new output time
  const auto out = op.on_imu(Stationary(At(now + 100'000'000 + kImuNs)));
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->reset_counter, 1U);
  EXPECT_NEAR(out->p_world_m.x(), 2.0 + 0.5 * kImuS, 1e-9);
  // The pre-reset history is gone.
  EXPECT_EQ(op.on_correction(Snap(At(now))).error(), PredictorReason::kOlderThanBuffer);
}

TEST(OutputPredictor, ResetBeforeAlignAligns) {
  OutputPredictor op(Beta());
  const ResetDelta d = op.reset_to(Snap(At(0), Vector3d(1.0, 1.0, 1.0)));
  EXPECT_EQ(d.position_m, Vector3d::Zero());
  const auto out = op.on_imu(Stationary(At(kImuNs)));
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->reset_counter, 1U);
  EXPECT_NEAR((out->p_world_m - Vector3d(1.0, 1.0, 1.0)).norm(), 0.0, 1e-9);
}

TEST(OutputPredictor, QuaternionStaysNormalizedOverLongRuns) {
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  std::optional<OutputSample> out;
  for (std::int64_t k = 1; k <= 100'000; ++k) {
    ImuDelta d = Stationary(At(k * kImuNs));
    d.delta_angle_rad = Vector3d(0.01, -0.02, 0.03);
    out = op.on_imu(d);
  }
  ASSERT_TRUE(out.has_value());
  EXPECT_NEAR(out->q_world_imu.norm(), 1.0, 1e-12);
  EXPECT_TRUE(Finite(*out));
}

TEST(OutputPredictor, TrackingErrorIsZeroAfterAlign) {
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  op.on_imu(Stationary(At(kImuNs)));
  ASSERT_TRUE(op.on_correction(Snap(At(0), Vector3d(1.0, 0.0, 0.0))).has_value());
  EXPECT_GT(op.tracking_error()(2), 0.0);
  op.align(Snap(At(2 * kImuNs)));
  EXPECT_EQ(op.tracking_error(), Vector3d::Zero());
}

TEST(OutputPredictor, VerticalChannelCorrectsPositionThroughVelocity) {
  // PX4's separate vertical channel: a vertical position error is removed by a velocity correction
  // whose integral is the vertical position, so the channel's vertical velocity becomes non-zero while
  // the main channel's velocity is untouched by a pure position error.
  OutputPredictor op(Beta());
  op.align(Snap(At(0)));
  std::int64_t now = 0;
  for (int k = 1; k <= 20; ++k) op.on_imu(Stationary(At(now += kImuNs)));
  EXPECT_EQ(op.vertical_velocity_mps(), 0.0);
  ASSERT_TRUE(op.on_correction(Snap(At(now - 50'000'000), Vector3d(0.0, 0.0, 0.5))).has_value());
  const auto out = op.on_imu(Stationary(At(now + kImuNs)));
  ASSERT_TRUE(out.has_value());
  EXPECT_EQ(out->v_world_mps.z(), 0.0);
  // PX4 vertical PD law: correction = vert_pos_err * pos_gain + vert_vel_err * vel_gain * 1.1, with
  // vert_pos_err = 0.5 m, vert_vel_err = 0 and pos_gain = dt_corr_avg / tau_pos = 0.010 / 0.25 (first
  // correction: the average still holds PX4's initial 0.010 s). Stationary IMU adds nothing after it.
  EXPECT_NEAR(op.vertical_velocity_mps(), 0.5 * (0.010 / 0.25), 1e-12);
}

TEST(OutputPredictor, ReasonNames) {
  static_assert(ReasonEnum<PredictorReason>);
  EXPECT_EQ(to_string(PredictorReason::kApplied), "APPLIED");
  EXPECT_EQ(to_string(PredictorReason::kNotInitialized), "NOT_INITIALIZED");
  EXPECT_EQ(to_string(PredictorReason::kOlderThanBuffer), "OLDER_THAN_BUFFER");
  EXPECT_EQ(to_string(PredictorReason::kNewerThanOutput), "NEWER_THAN_OUTPUT");
  EXPECT_EQ(to_string(PredictorReason::kNonFinite), "NON_FINITE");
}
