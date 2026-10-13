#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

#include "uavnav/core/angles.hpp"

using namespace uavnav::angles;

namespace {
constexpr double kPi = std::numbers::pi;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

// The boundary behaviour is that of std::remainder(a, 2*pi), which both old copies (the LIO output
// predictor's wrap_pi and the PX4 bridge's wrap) used: ties go to the even multiple.
TEST(Angles, WrapPiKeepsRangeAndNan) {
  EXPECT_DOUBLE_EQ(wrap_pi(3.5), 3.5 - 2.0 * kPi);
  EXPECT_DOUBLE_EQ(wrap_pi(-3.5), -3.5 + 2.0 * kPi);
  EXPECT_EQ(wrap_pi(0.0), 0.0);
  EXPECT_EQ(wrap_pi(1.0), 1.0);
  EXPECT_EQ(wrap_pi(-1.0), -1.0);
  EXPECT_EQ(wrap_pi(-kPi), -kPi);  // -pi stays
  EXPECT_EQ(wrap_pi(kPi), kPi);    // +pi stays: the tie 0.5 * (2 pi) goes to the even multiple 0
  EXPECT_TRUE(std::isnan(wrap_pi(kNaN)));
}

TEST(Angles, WrapPiOfOddMultiplesOfPiLandOnTheOppositeEdge) {
  // 3 pi is the tie 1.5 * (2 pi): the even multiple 2 is taken, so the result is -pi (and +pi for -3 pi).
  EXPECT_NEAR(wrap_pi(3.0 * kPi), -kPi, 1e-12);
  EXPECT_NEAR(wrap_pi(-3.0 * kPi), kPi, 1e-12);
  EXPECT_DOUBLE_EQ(wrap_pi(3.0 * kPi), std::remainder(3.0 * kPi, 2.0 * kPi));
  EXPECT_DOUBLE_EQ(wrap_pi(-3.0 * kPi), std::remainder(-3.0 * kPi, 2.0 * kPi));
}

TEST(Angles, WrapPiResultStaysInClosedRange) {
  for (double a = -50.0; a <= 50.0; a += 0.37) {
    const double w = wrap_pi(a);
    EXPECT_GE(w, -kPi) << a;
    EXPECT_LE(w, kPi) << a;
    EXPECT_NEAR(std::cos(w), std::cos(a), 1e-12) << a;
    EXPECT_NEAR(std::sin(w), std::sin(a), 1e-12) << a;
  }
}

TEST(Angles, WrapPiOfInfinityIsNaN) {
  EXPECT_TRUE(std::isnan(wrap_pi(kInf)));
  EXPECT_TRUE(std::isnan(wrap_pi(-kInf)));
}

TEST(Angles, YawFromRotationIsAtan2OfTheFirstColumn) {
  EXPECT_DOUBLE_EQ(yaw_from_rotation(0.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(yaw_from_rotation(1.0, 0.0), kPi / 2.0);
  EXPECT_DOUBLE_EQ(yaw_from_rotation(0.0, -1.0), kPi);
  EXPECT_DOUBLE_EQ(yaw_from_rotation(-1.0, 0.0), -kPi / 2.0);
  EXPECT_DOUBLE_EQ(yaw_from_rotation(std::sin(0.7), std::cos(0.7)), 0.7);
  EXPECT_TRUE(std::isnan(yaw_from_rotation(kNaN, 1.0)));
}
