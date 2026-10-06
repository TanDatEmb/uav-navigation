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
  in.quality = 127;
  EXPECT_EQ(MustEncode(in).quality, 127);
}

TEST(EvEncoder, QualityAbove127ClampsToInt8Max) {
  EvInput in = Good();
  in.quality = 128;
  EXPECT_EQ(MustEncode(in).quality, 127);
  in.quality = 255;
  EXPECT_EQ(MustEncode(in).quality, 127);
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

TEST(EvEncoder, OnlyDiagonalsAreUsedAsymmetricAndNonFiniteOffDiagonalIgnored) {
  EvInput in = Good();
  in.pose_cov(0, 1) = 5.0;   // asymmetric: (1,0) stays 0
  in.pose_cov(4, 2) = kNan;  // off-diagonal garbage, including the cross block
  in.pose_cov(3, 5) = -9.0;
  in.vel_cov(0, 2) = kInf;
  const EvSample s = MustEncode(in);
  EXPECT_FLOAT_EQ(s.position_variance[0], 0.01F);
  EXPECT_FLOAT_EQ(s.orientation_variance[2], 0.003F);
  EXPECT_FLOAT_EQ(s.velocity_variance[0], 0.04F);
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
