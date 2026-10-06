#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

#include "uavnav/px4bridge/ev_encoder.hpp"

using namespace uavnav;
using namespace uavnav::px4bridge;

namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

EvInput Good() {
  EvInput in{};
  in.t = time::SensorTime{5'123'456'789};
  in.epoch = 3;
  in.tracking = true;
  in.p_m = {1.0, 2.0, 3.0};
  in.q = Eigen::Quaterniond::Identity();
  in.v_mps = {0.5, -0.25, 0.125};
  in.pose_cov = Eigen::Matrix<double, 6, 6>::Zero();
  in.pose_cov.diagonal() << 0.01, 0.02, 0.03, 0.001, 0.002, 0.003;
  in.vel_cov = Eigen::Matrix3d::Zero();
  in.vel_cov.diagonal() << 0.04, 0.05, 0.06;
  in.quality = 73;
  return in;
}

EvSample MustEncode(const EvInput& in) {
  const auto r = encode_ev(in, ClockMode::kSimulationIdentity);
  EXPECT_TRUE(r.has_value()) << (r ? "" : std::string(to_string(r.error())));
  return r.value_or(EvSample{});
}

void ExpectReason(const EvInput& in, EvReason reason) {
  const auto r = encode_ev(in, ClockMode::kSimulationIdentity);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error(), reason) << to_string(r.error());
}

}  // namespace

TEST(EvEncoder, UsesSampleEpochAsResetCounter) {
  EvInput in = Good();
  in.epoch = 257;
  EXPECT_EQ(MustEncode(in).reset_counter, 1);
  in.epoch = 255;
  EXPECT_EQ(MustEncode(in).reset_counter, 255);
  in.epoch = 256;
  EXPECT_EQ(MustEncode(in).reset_counter, 0);
  in.epoch = 0;
  EXPECT_EQ(MustEncode(in).reset_counter, 0);
  in.epoch = std::numeric_limits<std::uint32_t>::max();
  EXPECT_EQ(MustEncode(in).reset_counter, 255);
}

TEST(EvEncoder, LabelsFrdNotNed) {
  const EvSample s = MustEncode(Good());
  EXPECT_EQ(s.pose_frame, 2);
  EXPECT_EQ(s.velocity_frame, 2);
  EXPECT_EQ(kPoseFrameFrd, 2);
  EXPECT_EQ(kVelocityFrameFrd, 2);
}

TEST(EvEncoder, RejectsWhenNotTracking) {
  EvInput in = Good();
  in.tracking = false;
  ExpectReason(in, EvReason::kNotTracking);
}

TEST(EvEncoder, NotTrackingIsCheckedFirst) {
  EvInput in = Good();
  in.tracking = false;
  in.p_m.x() = kNan;
  in.pose_cov(0, 0) = -1.0;
  in.t = time::SensorTime{-1};
  in.q = Eigen::Quaterniond{0, 0, 0, 0};
  EXPECT_EQ(encode_ev(in, ClockMode::kSimulationIdentity).error(), EvReason::kNotTracking);
  EXPECT_EQ(encode_ev(in, ClockMode::kRealtime).error(), EvReason::kNotTracking);
}

TEST(EvEncoder, RejectsNonPositiveVariance) {
  EvInput in = Good();
  in.pose_cov(0, 0) = 0.0;
  ExpectReason(in, EvReason::kBadCovariance);
}

TEST(EvEncoder, EveryDiagonalEntryIsChecked) {
  // 6 pose diagonal entries (position 0..2, orientation 3..5) and 3 velocity entries.
  for (double bad : {0.0, -1e-3, kNan, kInf, -kInf}) {
    for (int i = 0; i < 6; ++i) {
      EvInput in = Good();
      in.pose_cov(i, i) = bad;
      ExpectReason(in, EvReason::kBadCovariance);
    }
    for (int i = 0; i < 3; ++i) {
      EvInput in = Good();
      in.vel_cov(i, i) = bad;
      ExpectReason(in, EvReason::kBadCovariance);
    }
  }
}

TEST(EvEncoder, VarianceThatUnderflowsOrOverflowsFloatIsRejected) {
  EvInput in = Good();
  in.pose_cov(1, 1) = 1e-60;  // finite and > 0 as double, 0 as float
  ExpectReason(in, EvReason::kBadCovariance);
  in = Good();
  in.vel_cov(2, 2) = 1e60;  // inf as float
  ExpectReason(in, EvReason::kBadCovariance);
}

TEST(EvEncoder, CopiesQuality) {
  EvInput in = Good();
  in.quality = 73;
  EXPECT_EQ(MustEncode(in).quality, 73);
  in.quality = 0;
  EXPECT_EQ(MustEncode(in).quality, 0);
  in.quality = 100;
  EXPECT_EQ(MustEncode(in).quality, 100);
}

TEST(EvEncoder, QualityAbove100MapsToInvalidZero) {
  EvInput in = Good();
  in.quality = 100;
  EXPECT_EQ(MustEncode(in).quality, 100);
  in.quality = 101;
  EXPECT_EQ(MustEncode(in).quality, 0);
  in.quality = 127;
  EXPECT_EQ(MustEncode(in).quality, 0);
  in.quality = 128;
  EXPECT_EQ(MustEncode(in).quality, 0);
  in.quality = 255;
  EXPECT_EQ(MustEncode(in).quality, 0);
  in.quality = 0;
  EXPECT_EQ(MustEncode(in).quality, 0);
}

TEST(EvEncoder, PositionAndVelocityAreFlippedIntoFrd) {
  const EvSample s = MustEncode(Good());
  EXPECT_FLOAT_EQ(s.position[0], 1.0F);
  EXPECT_FLOAT_EQ(s.position[1], -2.0F);
  EXPECT_FLOAT_EQ(s.position[2], -3.0F);
  EXPECT_FLOAT_EQ(s.velocity[0], 0.5F);
  EXPECT_FLOAT_EQ(s.velocity[1], 0.25F);
  EXPECT_FLOAT_EQ(s.velocity[2], -0.125F);
}

TEST(EvEncoder, YawOfPlus30DegBecomesMinus30InFrd) {
  EvInput in = Good();
  const double yaw = 30.0 * std::numbers::pi / 180.0;
  in.q = Eigen::Quaterniond{Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}};
  const EvSample s = MustEncode(in);
  EXPECT_NEAR(s.q_wxyz[0], std::cos(yaw / 2), 1e-6);
  EXPECT_NEAR(s.q_wxyz[1], 0.0, 1e-6);
  EXPECT_NEAR(s.q_wxyz[2], 0.0, 1e-6);
  EXPECT_NEAR(s.q_wxyz[3], -std::sin(yaw / 2), 1e-6);
}

TEST(EvEncoder, QuaternionIsUnitAndCanonicalHemisphere) {
  EvInput in = Good();
  in.q = Eigen::Quaterniond{-0.5, 0.5, 0.5, -0.5};  // w < 0: same rotation as (0.5,-0.5,-0.5,0.5)
  const EvSample s = MustEncode(in);
  EXPECT_GE(s.q_wxyz[0], 0.0F);
  const double n = std::sqrt(double{s.q_wxyz[0]} * s.q_wxyz[0] + double{s.q_wxyz[1]} * s.q_wxyz[1] +
                             double{s.q_wxyz[2]} * s.q_wxyz[2] + double{s.q_wxyz[3]} * s.q_wxyz[3]);
  EXPECT_NEAR(n, 1.0, 1e-6);
  // Rotation preserved: FRD of (0.5,-0.5,-0.5,0.5) is (0.5,-0.5,0.5,-0.5).
  EXPECT_FLOAT_EQ(s.q_wxyz[0], 0.5F);
  EXPECT_FLOAT_EQ(s.q_wxyz[1], -0.5F);
  EXPECT_FLOAT_EQ(s.q_wxyz[2], 0.5F);
  EXPECT_FLOAT_EQ(s.q_wxyz[3], -0.5F);
}

TEST(EvEncoder, SlightlyDriftedQuaternionIsNormalised) {
  EvInput in = Good();
  in.q = Eigen::Quaterniond{1.0005, 0, 0, 0};
  const EvSample s = MustEncode(in);
  EXPECT_FLOAT_EQ(s.q_wxyz[0], 1.0F);
}

TEST(EvEncoder, RotationPropertyHoldsThroughTheEncoder) {
  // Body vector rotated by q then flipped == flipped body vector rotated by the encoded q_frd.
  EvInput in = Good();
  in.q = Eigen::Quaterniond{Eigen::AngleAxisd{0.9, Eigen::Vector3d{1, 2, -1}.normalized()}};
  const EvSample s = MustEncode(in);
  const Eigen::Quaterniond qf{s.q_wxyz[0], s.q_wxyz[1], s.q_wxyz[2], s.q_wxyz[3]};
  const Eigen::Vector3d v{0.3, -1.0, 2.0};
  const Eigen::Vector3d lhs{(in.q * v).x(), -(in.q * v).y(), -(in.q * v).z()};
  const Eigen::Vector3d rhs = qf * Eigen::Vector3d{v.x(), -v.y(), -v.z()};
  EXPECT_NEAR((lhs - rhs).norm(), 0.0, 1e-6);
}

TEST(EvEncoder, VariancesAreDiagonalsOfBlocks) {
  const EvSample s = MustEncode(Good());
  EXPECT_FLOAT_EQ(s.position_variance[0], 0.01F);
  EXPECT_FLOAT_EQ(s.position_variance[1], 0.02F);
  EXPECT_FLOAT_EQ(s.position_variance[2], 0.03F);
  EXPECT_FLOAT_EQ(s.orientation_variance[0], 0.001F);
  EXPECT_FLOAT_EQ(s.orientation_variance[1], 0.002F);
  EXPECT_FLOAT_EQ(s.orientation_variance[2], 0.003F);
  EXPECT_FLOAT_EQ(s.velocity_variance[0], 0.04F);
  EXPECT_FLOAT_EQ(s.velocity_variance[1], 0.05F);
  EXPECT_FLOAT_EQ(s.velocity_variance[2], 0.06F);
}

TEST(EvEncoder, PositionAndVelocityUseOnlyDiagonals) {
  // The position and velocity variances are the diagonals; off-diagonals (asymmetric, NaN, inf)
  // and the position/rotation cross blocks are ignored. (The orientation block is different: its
  // off-diagonals are needed for the world->body rotation, see the tests below.)
  EvInput in = Good();
  in.pose_cov(0, 1) = 5.0;   // asymmetric: (1,0) stays 0
  in.pose_cov(1, 2) = kNan;
  in.pose_cov(4, 2) = kNan;  // cross block
  in.pose_cov(0, 5) = kInf;
  in.vel_cov(0, 2) = kInf;
  const EvSample s = MustEncode(in);
  EXPECT_FLOAT_EQ(s.position_variance[0], 0.01F);
  EXPECT_FLOAT_EQ(s.position_variance[1], 0.02F);
  EXPECT_FLOAT_EQ(s.orientation_variance[2], 0.003F);
  EXPECT_FLOAT_EQ(s.velocity_variance[0], 0.04F);
}

namespace {

// Explicit 3x3 arithmetic, independent of the encoder: R = Rz(yaw) Ry(pitch) Rx(roll), body->world.
Eigen::Matrix3d ExplicitRzyx(double yaw, double pitch, double roll) {
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cr = std::cos(roll), sr = std::sin(roll);
  Eigen::Matrix3d r;
  r << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
       sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
       -sp,     cp * sr,                cp * cr;
  return r;
}

Eigen::Quaterniond QuatZyx(double yaw, double pitch, double roll) {
  return Eigen::Quaterniond{Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}} *
         Eigen::Quaterniond{Eigen::AngleAxisd{pitch, Eigen::Vector3d::UnitY()}} *
         Eigen::Quaterniond{Eigen::AngleAxisd{roll, Eigen::Vector3d::UnitX()}};
}

// out = R^T S R with plain loops.
Eigen::Matrix3d ExplicitRtSR(const Eigen::Matrix3d& r, const Eigen::Matrix3d& s) {
  Eigen::Matrix3d out = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
        for (int l = 0; l < 3; ++l) out(i, j) += r(k, i) * s(k, l) * r(l, j);
  return out;
}

}  // namespace

TEST(EvEncoder, OrientationVarianceIsRotatedFromWorldIntoBodyAxes) {
  const double d2r = std::numbers::pi / 180.0;
  const double yaw = 50 * d2r, pitch = 20 * d2r, roll = 30 * d2r;
  Eigen::Matrix3d world;  // SPD, non-isotropic, nonzero off-diagonals
  world << 0.040, 0.010, -0.008,
           0.010, 0.010, 0.004,
           -0.008, 0.004, 0.002;
  EvInput in = Good();
  in.q = QuatZyx(yaw, pitch, roll);
  in.pose_cov.block<3, 3>(3, 3) = world;
  const Eigen::Matrix3d body = ExplicitRtSR(ExplicitRzyx(yaw, pitch, roll), world);
  const EvSample s = MustEncode(in);
  // The diagonal of the body block differs from the world diagonal: the old diagonal copy fails here.
  EXPECT_GT(std::fabs(body(0, 0) - world(0, 0)), 1e-3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(s.orientation_variance[static_cast<std::size_t>(i)], body(i, i), 1e-7) << i;
  }
  // The FLU->FRD flip D S D^T leaves a diagonal unchanged, so the same numbers are FRD body variances.
  // The trace is invariant under rotation.
  EXPECT_NEAR(s.orientation_variance[0] + s.orientation_variance[1] + s.orientation_variance[2],
              world.trace(), 1e-7);
}

TEST(EvEncoder, OrientationBlockIsSymmetrisedBeforeRotation) {
  const double d2r = std::numbers::pi / 180.0;
  Eigen::Matrix3d sym;
  sym << 0.040, 0.010, -0.008,
         0.010, 0.010, 0.004,
         -0.008, 0.004, 0.002;
  Eigen::Matrix3d skew = sym;
  skew(0, 1) += 0.004;  // asymmetric by +-0.004 around the symmetric value
  skew(1, 0) -= 0.004;
  EvInput a = Good(), b = Good();
  a.q = b.q = QuatZyx(50 * d2r, 20 * d2r, 30 * d2r);
  a.pose_cov.block<3, 3>(3, 3) = sym;
  b.pose_cov.block<3, 3>(3, 3) = skew;
  const EvSample sa = MustEncode(a), sb = MustEncode(b);
  EXPECT_EQ(sa.orientation_variance, sb.orientation_variance);
}

TEST(EvEncoder, RotatedNonPositiveBodyVarianceIsRejected) {
  // Non-PSD world block with an all-positive diagonal: with yaw 45 deg the body y axis is
  // (-1,1,0)/sqrt2 in world, and (1 + 1 - 2*3)/2 = -2 < 0.
  Eigen::Matrix3d world = Eigen::Matrix3d::Identity();
  world(0, 1) = world(1, 0) = 3.0;
  EvInput in = Good();
  in.q = QuatZyx(std::numbers::pi / 4, 0, 0);
  in.pose_cov.block<3, 3>(3, 3) = world;
  ExpectReason(in, EvReason::kBadCovariance);
  // The same block with identity attitude is accepted (diagonal all positive): the rotation matters.
  in.q = Eigen::Quaterniond::Identity();
  EXPECT_TRUE(encode_ev(in, ClockMode::kSimulationIdentity).has_value());
}

TEST(EvEncoder, NonFiniteOrientationOffDiagonalIsRejected) {
  for (double bad : {kNan, kInf, -kInf}) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        if (i == j) continue;
        EvInput in = Good();
        in.pose_cov(3 + i, 3 + j) = bad;
        ExpectReason(in, EvReason::kBadCovariance);
      }
    }
  }
}

TEST(EvEncoder, PositionBlockUsesUpperLeftAndOrientationBlockLowerRight) {
  EvInput in = Good();
  in.pose_cov.diagonal() << 1, 2, 3, 4, 5, 6;
  const EvSample s = MustEncode(in);
  EXPECT_FLOAT_EQ(s.position_variance[2], 3.0F);
  EXPECT_FLOAT_EQ(s.orientation_variance[0], 4.0F);
}

TEST(EvEncoder, TimestampIsMicrosecondsOfSensorTime) {
  const EvSample s = MustEncode(Good());  // 5'123'456'789 ns
  EXPECT_EQ(s.timestamp_sample_us, 5'123'456U);
}

TEST(EvEncoder, TimeErrorsMapToKTime) {
  EvInput in = Good();
  in.t = time::SensorTime{-1};
  ExpectReason(in, EvReason::kTime);
  in = Good();
  const auto rt = encode_ev(in, ClockMode::kRealtime);
  ASSERT_FALSE(rt.has_value());
  EXPECT_EQ(rt.error(), EvReason::kTime);
}

TEST(EvEncoder, RejectsNonFiniteStateInEachField) {
  for (double bad : {kNan, kInf, -kInf}) {
    for (int i = 0; i < 3; ++i) {
      EvInput in = Good();
      in.p_m[i] = bad;
      ExpectReason(in, EvReason::kNonFiniteState);
      in = Good();
      in.v_mps[i] = bad;
      ExpectReason(in, EvReason::kNonFiniteState);
    }
    EvInput q = Good();
    q.q = Eigen::Quaterniond{bad, 0, 0, 0};
    ExpectReason(q, EvReason::kNonFiniteState);
    q = Good();
    q.q = Eigen::Quaterniond{1, bad, 0, 0};
    ExpectReason(q, EvReason::kNonFiniteState);
    q = Good();
    q.q = Eigen::Quaterniond{1, 0, bad, 0};
    ExpectReason(q, EvReason::kNonFiniteState);
    q = Good();
    q.q = Eigen::Quaterniond{1, 0, 0, bad};
    ExpectReason(q, EvReason::kNonFiniteState);
  }
}

TEST(EvEncoder, RejectsValueThatOverflowsFloat) {
  EvInput in = Good();
  in.p_m.x() = 1e300;
  ExpectReason(in, EvReason::kNonFiniteState);
  in = Good();
  in.v_mps.z() = -1e300;
  ExpectReason(in, EvReason::kNonFiniteState);
}

TEST(EvEncoder, RejectsZeroAndNonUnitQuaternion) {
  EvInput in = Good();
  in.q = Eigen::Quaterniond{0, 0, 0, 0};
  ExpectReason(in, EvReason::kNonFiniteState);
  in.q = Eigen::Quaterniond{2, 0, 0, 0};
  ExpectReason(in, EvReason::kNonFiniteState);
  in.q = Eigen::Quaterniond{0.5, 0, 0, 0};
  ExpectReason(in, EvReason::kNonFiniteState);
  in.q = Eigen::Quaterniond{1.01, 0, 0, 0};
  ExpectReason(in, EvReason::kNonFiniteState);
}

TEST(EvEncoder, StateIsCheckedBeforeCovariance) {
  EvInput in = Good();
  in.p_m.y() = kNan;
  in.pose_cov(0, 0) = -1.0;
  ExpectReason(in, EvReason::kNonFiniteState);
}

TEST(EvEncoder, TimeIsCheckedBeforeStateAndCovariance) {
  EvInput in = Good();
  in.t = time::SensorTime{-1};
  in.p_m.y() = kNan;
  in.pose_cov(0, 0) = -1.0;
  ExpectReason(in, EvReason::kTime);
}

TEST(EvEncoder, IsDeterministicAndDoesNotMutateInput) {
  const EvInput in = Good();
  const EvSample a = MustEncode(in);
  const EvSample b = MustEncode(in);
  EXPECT_EQ(a.timestamp_sample_us, b.timestamp_sample_us);
  EXPECT_EQ(a.position, b.position);
  EXPECT_EQ(a.q_wxyz, b.q_wxyz);
  EXPECT_EQ(a.velocity, b.velocity);
  EXPECT_EQ(a.position_variance, b.position_variance);
  EXPECT_EQ(a.reset_counter, b.reset_counter);
  EXPECT_EQ(in.p_m, Good().p_m);
}

TEST(EvEncoder, ReasonNamesAreUpperSnake) {
  EXPECT_EQ(to_string(EvReason::kEncoded), "ENCODED");
  EXPECT_EQ(to_string(EvReason::kNotTracking), "NOT_TRACKING");
  EXPECT_EQ(to_string(EvReason::kNonFiniteState), "NON_FINITE_STATE");
  EXPECT_EQ(to_string(EvReason::kBadCovariance), "BAD_COVARIANCE");
  EXPECT_EQ(to_string(EvReason::kTime), "TIME");
  static_assert(ReasonEnum<EvReason>);
}
