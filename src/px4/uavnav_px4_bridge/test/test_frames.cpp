#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "uavnav/px4bridge/frames.hpp"

using namespace uavnav::px4bridge;

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kEps = 1e-12;

Eigen::Quaterniond Axis(const Eigen::Vector3d& axis, double rad) {
  return Eigen::Quaterniond{Eigen::AngleAxisd{rad, axis.normalized()}};
}

// Yaw/pitch/roll about z, y, x (intrinsic ZYX), body->world.
Eigen::Quaterniond Ypr(double yaw, double pitch, double roll) {
  return Axis(Eigen::Vector3d::UnitZ(), yaw) * Axis(Eigen::Vector3d::UnitY(), pitch) *
         Axis(Eigen::Vector3d::UnitX(), roll);
}

double YawOf(const Eigen::Quaterniond& q) {
  const Eigen::Vector3d x = q * Eigen::Vector3d::UnitX();
  return std::atan2(x.y(), x.x());
}

}  // namespace

TEST(Frames, FluToFrdFlipsYZ) {
  const Eigen::Vector3d v = flu_to_frd(Eigen::Vector3d{1.0, 2.0, 3.0});
  EXPECT_EQ(v, (Eigen::Vector3d{1.0, -2.0, -3.0}));
  const Eigen::Quaterniond q = flu_to_frd(Eigen::Quaterniond::Identity());
  EXPECT_DOUBLE_EQ(q.w(), 1.0);
  EXPECT_DOUBLE_EQ(q.x(), 0.0);
  EXPECT_DOUBLE_EQ(q.y(), 0.0);
  EXPECT_DOUBLE_EQ(q.z(), 0.0);
}

TEST(Frames, VectorConversionIsSelfInverse) {
  const Eigen::Vector3d v{0.3, -4.5, 7.25};
  EXPECT_EQ(flu_to_frd(flu_to_frd(v)), v);
}

TEST(Frames, YawSignFlips) {
  const double yaw = 30.0 * kPi / 180.0;
  const Eigen::Quaterniond frd = flu_to_frd(Axis(Eigen::Vector3d::UnitZ(), yaw));
  EXPECT_NEAR(YawOf(frd), -yaw, kEps);
  // Same thing as the quaternion components: rotation about FRD z of -30 deg.
  EXPECT_NEAR(frd.w(), std::cos(yaw / 2), kEps);
  EXPECT_NEAR(frd.z(), -std::sin(yaw / 2), kEps);
  EXPECT_NEAR(frd.x(), 0.0, kEps);
  EXPECT_NEAR(frd.y(), 0.0, kEps);
}

TEST(Frames, RollKeepsSignPitchFlips) {
  const double a = 0.4;
  // Roll about forward x: the axis is unchanged by F, so the quaternion keeps its sign.
  const Eigen::Quaterniond roll = flu_to_frd(Axis(Eigen::Vector3d::UnitX(), a));
  EXPECT_NEAR(roll.x(), std::sin(a / 2), kEps);
  EXPECT_NEAR(roll.y(), 0.0, kEps);
  EXPECT_NEAR(roll.z(), 0.0, kEps);
  // Pitch about y: y axis -> -y axis, so the y component flips sign (nose-up FLU = nose-up FRD
  // pitch of the opposite sign about the right-pointing axis).
  const Eigen::Quaterniond pitch = flu_to_frd(Axis(Eigen::Vector3d::UnitY(), a));
  EXPECT_NEAR(pitch.y(), -std::sin(a / 2), kEps);
  EXPECT_NEAR(pitch.w(), std::cos(a / 2), kEps);
}

TEST(Frames, QuaternionStaysUnitAndSelfInverse) {
  const Eigen::Quaterniond q = Ypr(0.7, -0.3, 1.1);
  const Eigen::Quaterniond f = flu_to_frd(q);
  EXPECT_NEAR(f.norm(), 1.0, kEps);
  const Eigen::Quaterniond back = flu_to_frd(f);
  EXPECT_NEAR(back.w(), q.w(), kEps);
  EXPECT_NEAR(back.x(), q.x(), kEps);
  EXPECT_NEAR(back.y(), q.y(), kEps);
  EXPECT_NEAR(back.z(), q.z(), kEps);
}

TEST(Frames, HemisphereIsPreserved) {
  // Conversion never flips the overall sign: w keeps its sign (canonicalisation is the encoder's job).
  const Eigen::Quaterniond neg{-0.5, 0.5, 0.5, -0.5};
  const Eigen::Quaterniond f = flu_to_frd(neg);
  EXPECT_DOUBLE_EQ(f.w(), -0.5);
  EXPECT_DOUBLE_EQ(f.x(), 0.5);
  EXPECT_DOUBLE_EQ(f.y(), -0.5);
  EXPECT_DOUBLE_EQ(f.z(), 0.5);
}

// Rotating a body vector by q then converting equals converting then rotating by q_frd.
TEST(Frames, RotateThenConvertEqualsConvertThenRotate) {
  const double yaws[] = {-2.9, -1.0, 0.0, 0.5, 2.2};
  const double pitches[] = {-1.2, -0.3, 0.0, 0.8};
  const double rolls[] = {-3.0, -0.6, 0.0, 1.4, 3.0};
  const Eigen::Vector3d vs[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0.3, -2.0, 5.5}, {-7.1, 0.4, -0.9}};
  int checked = 0;
  for (double y : yaws) {
    for (double p : pitches) {
      for (double r : rolls) {
        const Eigen::Quaterniond q = Ypr(y, p, r);
        const Eigen::Quaterniond qf = flu_to_frd(q);
        for (const auto& v : vs) {
          const Eigen::Vector3d lhs = flu_to_frd(q * v);        // body FLU -> world FLU -> world FRD
          const Eigen::Vector3d rhs = qf * flu_to_frd(v);       // body FLU -> body FRD -> world FRD
          EXPECT_NEAR((lhs - rhs).norm(), 0.0, 1e-12);
          ++checked;
        }
      }
    }
  }
  EXPECT_EQ(checked, 5 * 4 * 5 * 5);
}

TEST(Frames, CovarianceIsConjugatedByDiagonalFlip) {
  Eigen::Matrix3d s;
  s << 1.0, 0.2, 0.3,
       0.2, 2.0, 0.4,
       0.3, 0.4, 3.0;
  const Eigen::Matrix3d f = flu_to_frd_cov(s);
  Eigen::Matrix3d expected;
  expected << 1.0, -0.2, -0.3,
              -0.2, 2.0, 0.4,
              -0.3, 0.4, 3.0;
  EXPECT_LT((f - expected).cwiseAbs().maxCoeff(), kEps);
  EXPECT_EQ(f.diagonal(), s.diagonal());
  EXPECT_LT((flu_to_frd_cov(f) - s).cwiseAbs().maxCoeff(), kEps);
}

TEST(Frames, CovarianceMatchesMatrixFormAndStaysPositive) {
  Eigen::Matrix3d a;
  a << 0.9, 0.1, -0.2, 0.3, 1.2, 0.5, -0.4, 0.7, 1.5;
  const Eigen::Matrix3d s = a * a.transpose();  // SPD
  const Eigen::Matrix3d d = Eigen::Vector3d{1, -1, -1}.asDiagonal();
  EXPECT_LT((flu_to_frd_cov(s) - d * s * d.transpose()).cwiseAbs().maxCoeff(), kEps);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es{flu_to_frd_cov(s)};
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}
