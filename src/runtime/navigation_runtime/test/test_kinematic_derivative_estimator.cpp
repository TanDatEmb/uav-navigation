#include <gtest/gtest.h>

#include <Eigen/Core>

#include "navigation_runtime/kinematic_derivative_estimator.hpp"

namespace {

using navigation_runtime::KinematicDerivativeEstimator;

TEST(KinematicDerivativeEstimator, RequiresTwoSamplesForAccelerationAndThreeForJerk) {
  KinematicDerivativeEstimator estimator;

  const auto first = estimator.update(1'000'000'000, 7U, Eigen::Vector3d::Zero());
  EXPECT_FALSE(first.acceleration_estimated);
  EXPECT_FALSE(first.jerk_estimated);

  const auto second = estimator.update(1'100'000'000, 7U, Eigen::Vector3d{0.1, 0.0, 0.0});
  EXPECT_TRUE(second.acceleration_estimated);
  EXPECT_FALSE(second.jerk_estimated);
  EXPECT_TRUE(second.acceleration_world.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));

  const auto third = estimator.update(1'200'000'000, 7U, Eigen::Vector3d{0.3, 0.0, 0.0});
  EXPECT_TRUE(third.acceleration_estimated);
  EXPECT_TRUE(third.jerk_estimated);
  EXPECT_TRUE(third.acceleration_world.isApprox(Eigen::Vector3d(2.0, 0.0, 0.0)));
  EXPECT_TRUE(third.jerk_world.isApprox(Eigen::Vector3d(10.0, 0.0, 0.0)));
}

TEST(KinematicDerivativeEstimator, EpochChangeDoesNotBridgeDerivativeHistory) {
  KinematicDerivativeEstimator estimator;
  (void)estimator.update(1'000'000'000, 7U, Eigen::Vector3d{4.0, 0.0, 0.0});

  const auto after_reset =
      estimator.update(1'100'000'000, 8U, Eigen::Vector3d{20.0, 0.0, 0.0});
  EXPECT_FALSE(after_reset.acceleration_estimated);
  EXPECT_FALSE(after_reset.jerk_estimated);

  const auto next =
      estimator.update(1'200'000'000, 8U, Eigen::Vector3d{20.1, 0.0, 0.0});
  EXPECT_TRUE(next.acceleration_estimated);
  EXPECT_FALSE(next.jerk_estimated);
  EXPECT_TRUE(next.acceleration_world.isApprox(Eigen::Vector3d(1.0, 0.0, 0.0)));
}

TEST(KinematicDerivativeEstimator, LongOrRegressingStampStartsNewHistory) {
  KinematicDerivativeEstimator estimator;
  (void)estimator.update(1'000'000'000, 7U, Eigen::Vector3d::Zero());
  const auto long_gap =
      estimator.update(1'600'000'000, 7U, Eigen::Vector3d{10.0, 0.0, 0.0});
  EXPECT_FALSE(long_gap.acceleration_estimated);

  const auto regressing =
      estimator.update(1'500'000'000, 7U, Eigen::Vector3d{11.0, 0.0, 0.0});
  EXPECT_FALSE(regressing.acceleration_estimated);

  const auto recovered =
      estimator.update(1'600'000'000, 7U, Eigen::Vector3d{12.0, 0.0, 0.0});
  EXPECT_TRUE(recovered.acceleration_estimated);
  EXPECT_TRUE(recovered.acceleration_world.isApprox(Eigen::Vector3d(10.0, 0.0, 0.0)));
}

}  // namespace
