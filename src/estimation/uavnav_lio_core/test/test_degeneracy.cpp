#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <Eigen/Core>

#include "uavnav/lio/degeneracy.hpp"

using namespace uavnav::lio;

namespace {

using Matrix6 = Eigen::Matrix<double, 6, 6>;

// Beta thresholds (see config.hpp for the derivation).
constexpr double kT = 1.1e5;
constexpr double kR = 2.8e6;
const DegeneracyConfig kConfig{kT, kR};

// Block-diagonal information: translation block diag(t0,t1,t2), rotation block diag(r0,r1,r2).
Matrix6 Diag(double t0, double t1, double t2, double r0, double r1, double r2) {
  Matrix6 m = Matrix6::Zero();
  m.diagonal() << t0, t1, t2, r0, r1, r2;
  return m;
}

Matrix6 AtThreshold() { return Diag(kT, kT, kT, kR, kR, kR); }

DegeneracyReport Eval(const Matrix6& m) { return evaluate_degeneracy(std::optional<Matrix6>(m), kConfig); }

}  // namespace

TEST(Degeneracy, NoInformationIsDegenerateWithZeroQuality) {
  const DegeneracyReport r = evaluate_degeneracy(std::nullopt, kConfig);
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
  EXPECT_EQ(r.translation_min_eigenvalue, 0.0);
  EXPECT_EQ(r.rotation_min_eigenvalue, 0.0);
}

TEST(Degeneracy, QualityIsFiftyAtThreshold) {
  const DegeneracyReport r = Eval(AtThreshold());
  EXPECT_FALSE(r.degenerate);
  EXPECT_EQ(r.quality, 50U);
  EXPECT_DOUBLE_EQ(r.translation_min_eigenvalue, kT);
  EXPECT_DOUBLE_EQ(r.rotation_min_eigenvalue, kR);
}

TEST(Degeneracy, QualityCapsAtHundred) {
  const DegeneracyReport r = Eval(Diag(10 * kT, 10 * kT, 10 * kT, 10 * kR, 10 * kR, 10 * kR));
  EXPECT_FALSE(r.degenerate);
  EXPECT_EQ(r.quality, 100U);
}

TEST(Degeneracy, DoubleThresholdIsHundred) {
  const DegeneracyReport r = Eval(Diag(2 * kT, 2 * kT, 2 * kT, 2 * kR, 2 * kR, 2 * kR));
  EXPECT_FALSE(r.degenerate);
  EXPECT_EQ(r.quality, 100U);
}

TEST(Degeneracy, WeakTranslationAxisIsDegenerate) {
  const DegeneracyReport r = Eval(Diag(kT, kT, 10.0, kR, kR, kR));
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
  EXPECT_DOUBLE_EQ(r.translation_min_eigenvalue, 10.0);
}

TEST(Degeneracy, RotationBlockUsedIndependently) {
  // Translation block is very strong; only the rotation block is below its threshold.
  const DegeneracyReport r = Eval(Diag(10 * kT, 10 * kT, 10 * kT, kR, kR, kR / 2));
  EXPECT_TRUE(r.degenerate);
  EXPECT_DOUBLE_EQ(r.rotation_min_eigenvalue, kR / 2);
  EXPECT_DOUBLE_EQ(r.translation_min_eigenvalue, 10 * kT);
  EXPECT_EQ(r.quality, 25U);  // the rotation ratio (0.5) is the minimum
}

TEST(Degeneracy, JustBelowThresholdIsDegenerate) {
  EXPECT_TRUE(Eval(Diag(kT * 0.999, kT, kT, kR, kR, kR)).degenerate);
  EXPECT_TRUE(Eval(Diag(kT, kT, kT, kR, kR, kR * 0.999)).degenerate);
  EXPECT_LE(Eval(Diag(kT * 0.999, kT, kT, kR, kR, kR)).quality, 50U);
}

TEST(Degeneracy, JustAboveThresholdIsNotDegenerate) {
  const DegeneracyReport r = Eval(Diag(kT * 1.001, kT * 1.001, kT * 1.001, kR * 1.001, kR * 1.001, kR * 1.001));
  EXPECT_FALSE(r.degenerate);
  EXPECT_EQ(r.quality, 50U);
}

TEST(Degeneracy, RotationOnlyDeficiencyIsDegenerate) {
  const DegeneracyReport r = Eval(Diag(kT, kT, kT, kR, kR, 100.0));
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
  EXPECT_DOUBLE_EQ(r.translation_min_eigenvalue, kT);
  EXPECT_DOUBLE_EQ(r.rotation_min_eigenvalue, 100.0);
}

TEST(Degeneracy, QualityTracksTheWeakerBlock) {
  // Translation at 1.5x (ratio 1.5), rotation at 0.8x (ratio 0.8) -> 50 * 0.8 = 40.
  const DegeneracyReport r = Eval(Diag(1.5 * kT, 1.5 * kT, 1.5 * kT, 0.8 * kR, 0.8 * kR, 0.8 * kR));
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 40U);
}

TEST(Degeneracy, NanEntryIsDegenerateWithZeroQuality) {
  Matrix6 m = AtThreshold();
  m(1, 1) = std::numeric_limits<double>::quiet_NaN();
  const DegeneracyReport r = Eval(m);
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
}

TEST(Degeneracy, NanInRotationBlockIsDegenerate) {
  Matrix6 m = AtThreshold();
  m(4, 5) = std::numeric_limits<double>::quiet_NaN();
  const DegeneracyReport r = Eval(m);
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
}

TEST(Degeneracy, InfEntryIsDegenerateWithZeroQuality) {
  Matrix6 m = AtThreshold();
  m(0, 0) = std::numeric_limits<double>::infinity();
  const DegeneracyReport r = Eval(m);
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
}

TEST(Degeneracy, NegativeInfEntryIsDegenerateWithZeroQuality) {
  Matrix6 m = AtThreshold();
  m(5, 5) = -std::numeric_limits<double>::infinity();
  const DegeneracyReport r = Eval(m);
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
}

TEST(Degeneracy, NonFiniteEntryOutsideTheDiagonalBlocksIsStillDegenerate) {
  // The cross-coupling blocks are ignored by the eigen analysis but a non-finite value there
  // still means the matrix cannot be trusted.
  Matrix6 m = AtThreshold();
  m(0, 4) = std::numeric_limits<double>::quiet_NaN();
  const DegeneracyReport r = Eval(m);
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
}

TEST(Degeneracy, SlightlyAsymmetricMatrixIsHandled) {
  Matrix6 m = Diag(1.5 * kT, 1.5 * kT, 1.5 * kT, 1.5 * kR, 1.5 * kR, 1.5 * kR);
  m(0, 1) = 1.0;
  m(1, 0) = 1.0 + 1e-9;  // asymmetric by rounding-noise amounts
  m(3, 4) = 2.0;
  m(4, 3) = 2.0 - 1e-9;
  const DegeneracyReport r = Eval(m);
  EXPECT_FALSE(r.degenerate);
  EXPECT_EQ(r.quality, 75U);
  EXPECT_NEAR(r.translation_min_eigenvalue, 1.5 * kT - 1.0, 1e-3);
  EXPECT_NEAR(r.rotation_min_eigenvalue, 1.5 * kR - 2.0, 1e-3);
}

TEST(Degeneracy, AsymmetryIsSymmetrisedNotReadFromOneTriangle) {
  // Upper triangle says the coupling is 0, lower says 2e5: the symmetrised coupling is 1e5, so the
  // translation block {{kT, 1e5},{1e5, kT}} has min eigenvalue kT - 1e5 = 1e4 -> degenerate.
  Matrix6 m = AtThreshold();
  m(0, 1) = 0.0;
  m(1, 0) = 2e5;
  const DegeneracyReport r = Eval(m);
  EXPECT_TRUE(r.degenerate);
  EXPECT_NEAR(r.translation_min_eigenvalue, kT - 1e5, 1e-3);
}

TEST(Degeneracy, NegativeEigenvalueBlockHasZeroQualityAndRawEigenvalue) {
  Matrix6 m = AtThreshold();
  m(2, 2) = -5.0;
  const DegeneracyReport r = Eval(m);
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
  EXPECT_DOUBLE_EQ(r.translation_min_eigenvalue, -5.0);  // raw value is reported
}

TEST(Degeneracy, ZeroMatrixIsDegenerateWithZeroQuality) {
  const DegeneracyReport r = Eval(Matrix6::Zero());
  EXPECT_TRUE(r.degenerate);
  EXPECT_EQ(r.quality, 0U);
}

TEST(Degeneracy, HugeValuesDoNotOverflowTheQualityCast) {
  const DegeneracyReport r = Eval(Diag(1e300, 1e300, 1e300, 1e300, 1e300, 1e300));
  EXPECT_FALSE(r.degenerate);
  EXPECT_EQ(r.quality, 100U);
}

TEST(Degeneracy, ExtremeFiniteValuesNeverThrowOrReturnOutOfRangeQuality) {
  const double big = std::numeric_limits<double>::max();
  const DegeneracyReport r = Eval(Diag(big, big, big, big, big, big));
  EXPECT_LE(r.quality, 100U);
  if (r.degenerate) {
    EXPECT_EQ(r.quality, 0U);  // either fully trusted or reported as unusable
  }
}

TEST(Degeneracy, OffDiagonalCouplingReducesTheMinimumEigenvalue) {
  Matrix6 m = Diag(2 * kT, 2 * kT, 2 * kT, 2 * kR, 2 * kR, 2 * kR);
  m(0, 1) = m(1, 0) = 2 * kT - 1000.0;  // eigenvalues of {{a, a-1000},{a-1000, a}}: 1000 and 2a-1000
  const DegeneracyReport r = Eval(m);
  EXPECT_TRUE(r.degenerate);
  EXPECT_NEAR(r.translation_min_eigenvalue, 1000.0, 1e-3);
}

TEST(Degeneracy, InvalidThresholdsAreDegenerateRatherThanDividingByZero) {
  const Matrix6 m = AtThreshold();
  for (const DegeneracyConfig bad : {DegeneracyConfig{0.0, kR}, DegeneracyConfig{kT, -1.0},
                                     DegeneracyConfig{std::numeric_limits<double>::quiet_NaN(), kR},
                                     DegeneracyConfig{kT, std::numeric_limits<double>::infinity()}}) {
    const DegeneracyReport r = evaluate_degeneracy(std::optional<Matrix6>(m), bad);
    EXPECT_TRUE(r.degenerate);
    EXPECT_EQ(r.quality, 0U);
  }
}
