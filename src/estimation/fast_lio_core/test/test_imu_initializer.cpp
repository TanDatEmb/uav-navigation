#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "fast_lio_core/initialization/imu_initializer.hpp"

namespace uav::nav::lio {
namespace {

ImuSample initializerSample(std::int64_t time_ns, const Eigen::Vector3d& gyro,
                            const Eigen::Vector3d& accel) {
  ImuSample sample;
  sample.time = Timestamp(time_ns, ClockDomain::kSensorTime);
  sample.angular_velocity_imu_rad_s = gyro;
  sample.linear_acceleration_imu_m_s2 = accel;
  return sample;
}

TEST(ImuInitializerTest, EstimatesStationaryGravityAndGyroBias) {
  ImuInitializerConfig config;
  config.minimum_imu_samples = 200;
  config.maximum_imu_samples = 200;
  ImuInitializer initializer(config);
  const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.005);
  for (std::int64_t index = 0; index < 200; ++index) {
    ASSERT_TRUE(initializer
                    .addSample(initializerSample(1 + index * 5'000'000, gyro_bias,
                                                 Eigen::Vector3d(0.0, 0.0, kStandardGravityMps2)))
                    .ok());
  }
  const auto result = initializer.tryInitialize();
  ASSERT_TRUE(result.ok()) << result.status().message();
  EXPECT_TRUE(result.value().gyro_bias_rad_s.isApprox(gyro_bias, 1e-12));
  EXPECT_TRUE(result.value().gravity_odom_m_s2.isApprox(
      Eigen::Vector3d(0.0, 0.0, -kStandardGravityMps2), 1e-12));
  EXPECT_TRUE(result.value().orientation_odom_imu.isApprox(Eigen::Quaterniond::Identity(), 1e-12));
  EXPECT_TRUE(result.value().accel_bias_m_s2.isZero(1e-12));
  EXPECT_TRUE(result.value().quality.stationary);
}

TEST(ImuInitializerTest, RejectsInvalidConfiguration) {
  ImuInitializerConfig config;
  config.minimum_imu_samples = 11;
  config.maximum_imu_samples = 10;
  EXPECT_THROW({ ImuInitializer initializer(config); }, std::invalid_argument);

  config = {};
  config.maximum_gyro_variance_rad2_s2 =
      std::numeric_limits<double>::infinity();
  EXPECT_THROW({ ImuInitializer initializer(config); }, std::invalid_argument);
}

TEST(ImuInitializerTest, AlignsTiltedMeasuredGravityToOdomUp) {
  ImuInitializerConfig config;
  config.minimum_imu_samples = 10;
  config.maximum_imu_samples = 10;
  ImuInitializer initializer(config);
  for (std::int64_t index = 0; index < 10; ++index) {
    ASSERT_TRUE(initializer
                    .addSample(initializerSample(index + 1, Eigen::Vector3d::Zero(),
                                                 Eigen::Vector3d(kStandardGravityMps2, 0.0, 0.0)))
                    .ok());
  }
  const auto result = initializer.tryInitialize();
  ASSERT_TRUE(result.ok());
  const Eigen::Vector3d aligned = result.value().orientation_odom_imu * Eigen::Vector3d::UnitX();
  EXPECT_TRUE(aligned.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
}

TEST(ImuInitializerTest, RejectsNonStationaryWindow) {
  ImuInitializerConfig config;
  config.minimum_imu_samples = 10;
  config.maximum_imu_samples = 10;
  ImuInitializer initializer(config);
  for (std::int64_t index = 0; index < 10; ++index) {
    const double sign = index % 2 == 0 ? 1.0 : -1.0;
    ASSERT_TRUE(initializer
                    .addSample(initializerSample(index + 1, Eigen::Vector3d(sign, 0.0, 0.0),
                                                 Eigen::Vector3d(0.0, 0.0, kStandardGravityMps2)))
                    .ok());
  }
  const auto result = initializer.tryInitialize();
  ASSERT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), StatusCode::kInitializationRejected);
}

}  // namespace
}  // namespace uav::nav::lio
