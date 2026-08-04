#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

#include "px4_odometry_bridge/external_odometry_conversion.hpp"

namespace px4_odometry_bridge {
namespace {

nav_msgs::msg::Odometry valid_message() {
  nav_msgs::msg::Odometry message;
  message.header.stamp.sec = 1;
  message.header.stamp.nanosec = 2;
  message.header.frame_id = "lio_odom";
  message.child_frame_id = "base_link";
  message.pose.pose.orientation.w = 1.0;
  for (int index = 0; index < 3; ++index) {
    message.pose.covariance[static_cast<std::size_t>(index * 6 + index)] = 1.0;
    message.pose.covariance[static_cast<std::size_t>((index + 3) * 6 + index + 3)] = 1.0;
    message.twist.covariance[static_cast<std::size_t>(index * 6 + index)] = 1.0;
  }
  return message;
}

TEST(ExternalOdometryConversionTest, ConvertsRosEnuFluToPx4NedFrd) {
  auto message = valid_message();
  message.pose.pose.position.x = 1.0;
  message.pose.pose.position.y = 2.0;
  message.pose.pose.position.z = 3.0;
  message.twist.twist.linear.y = 4.0;
  message.twist.twist.angular.z = 5.0;
  const auto converted = convert_ros_lio_odometry(message);
  ASSERT_TRUE(converted.has_value());
  EXPECT_EQ(converted->timestamp_ns, 1'000'000'002LL);
  EXPECT_DOUBLE_EQ(converted->position_ned.x(), 2.0);
  EXPECT_DOUBLE_EQ(converted->position_ned.y(), 1.0);
  EXPECT_DOUBLE_EQ(converted->position_ned.z(), -3.0);
  // Identity attitude: body-FLU velocity [0,4,0] is ENU [0,4,0], then NED
  // [4,0,0].  It is not a body-FRD vector in the new /fmu/in contract.
  EXPECT_DOUBLE_EQ(converted->velocity_ned.x(), 4.0);
  EXPECT_DOUBLE_EQ(converted->velocity_ned.y(), 0.0);
  EXPECT_DOUBLE_EQ(converted->velocity_ned.z(), 0.0);
  EXPECT_DOUBLE_EQ(converted->angular_velocity_body_frd.z(), -5.0);
  EXPECT_TRUE(converted->frame_valid);
  EXPECT_TRUE(converted->covariance_valid);
}

TEST(ExternalOdometryConversionTest, UsesSeparateWorldAndBodyFrameMatrices) {
  auto message = valid_message();
  const Eigen::Quaterniond ros_orientation(
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  message.pose.pose.orientation.w = ros_orientation.w();
  message.pose.pose.orientation.x = ros_orientation.x();
  message.pose.pose.orientation.y = ros_orientation.y();
  message.pose.pose.orientation.z = ros_orientation.z();
  const auto converted = convert_ros_lio_odometry(message);
  ASSERT_TRUE(converted.has_value());
  const Eigen::Matrix3d expected =
      C_ned_from_lio_enu() * ros_orientation.toRotationMatrix() *
      C_body_frd_from_body_flu().inverse();
  EXPECT_TRUE(converted->orientation_ned.toRotationMatrix().isApprox(expected));
  EXPECT_NEAR(converted->orientation_ned.norm(), 1.0, 1e-12);

  auto negated = message;
  negated.pose.pose.orientation.w = -negated.pose.pose.orientation.w;
  negated.pose.pose.orientation.x = -negated.pose.pose.orientation.x;
  negated.pose.pose.orientation.y = -negated.pose.pose.orientation.y;
  negated.pose.pose.orientation.z = -negated.pose.pose.orientation.z;
  const auto converted_negated = convert_ros_lio_odometry(negated);
  ASSERT_TRUE(converted_negated.has_value());
  EXPECT_TRUE(converted->orientation_ned.toRotationMatrix().isApprox(
      converted_negated->orientation_ned.toRotationMatrix()));
}

TEST(ExternalOdometryConversionTest, RotatesBodyVelocityIntoNedWorldFrame) {
  auto message = valid_message();
  const Eigen::Quaterniond yaw_quarter_turn(
      Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()));
  message.pose.pose.orientation.w = yaw_quarter_turn.w();
  message.pose.pose.orientation.x = yaw_quarter_turn.x();
  message.pose.pose.orientation.y = yaw_quarter_turn.y();
  message.pose.pose.orientation.z = yaw_quarter_turn.z();
  // Body-FLU +X becomes world-ENU +Y after the yaw, then NED [1, 0, 0].
  message.twist.twist.linear.x = 1.0;
  const auto converted = convert_ros_lio_odometry(message);
  ASSERT_TRUE(converted.has_value());
  EXPECT_NEAR(converted->velocity_ned.x(), 1.0, 1e-12);
  EXPECT_NEAR(converted->velocity_ned.y(), 0.0, 1e-12);
  EXPECT_NEAR(converted->velocity_ned.z(), 0.0, 1e-12);
}

TEST(ExternalOdometryConversionTest, TransformsFullNonDiagonalCovarianceBeforePublishingDiagonal) {
  std::array<double, 36> covariance{};
  covariance[0] = 2.0;
  covariance[7] = 3.0;
  covariance[14] = 4.0;
  covariance[1] = covariance[6] = 0.25;
  covariance[2] = covariance[12] = 0.5;
  covariance[8] = covariance[13] = 0.75;
  const auto diagonal = transformed_covariance_diagonal(
      covariance, 0, C_ned_from_lio_enu());
  ASSERT_TRUE(diagonal.has_value());
  EXPECT_DOUBLE_EQ((*diagonal)[0], 3.0);
  EXPECT_DOUBLE_EQ((*diagonal)[1], 2.0);
  EXPECT_DOUBLE_EQ((*diagonal)[2], 4.0);

  const Eigen::Matrix3d quarter_turn =
      Eigen::AngleAxisd(std::numbers::pi / 2.0,
                        Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto rotated_diagonal =
      transformed_covariance_diagonal(covariance, 0, quarter_turn);
  ASSERT_TRUE(rotated_diagonal.has_value());
  EXPECT_NEAR((*rotated_diagonal)[0], 3.0, 1e-12);
  EXPECT_NEAR((*rotated_diagonal)[1], 2.0, 1e-12);
  EXPECT_NEAR((*rotated_diagonal)[2], 4.0, 1e-12);
}

TEST(ExternalOdometryConversionTest, PreservesPositiveVarianceBelowExampleFloor) {
  auto message = valid_message();
  constexpr double small_variance = 1e-12;
  message.pose.covariance[0] = small_variance;
  const auto converted = convert_ros_lio_odometry(message);
  ASSERT_TRUE(converted.has_value());
  EXPECT_DOUBLE_EQ(converted->position_variance.y(), small_variance);
}

TEST(ExternalOdometryConversionTest, RequiresPositiveFiniteFloatRepresentableVariance) {
  EXPECT_TRUE(positive_variance_to_px4_float(1e-12).has_value());
  EXPECT_TRUE(positive_variance_to_px4_float(
                  static_cast<double>(std::numeric_limits<float>::denorm_min()))
                  .has_value());
  EXPECT_FALSE(positive_variance_to_px4_float(
                   static_cast<double>(std::numeric_limits<float>::denorm_min()) * 0.5)
                   .has_value());
  EXPECT_FALSE(positive_variance_to_px4_float(
                   static_cast<double>(std::numeric_limits<float>::max()) * 2.0)
                   .has_value());
  EXPECT_FALSE(positive_variance_to_px4_float(0.0).has_value());
  EXPECT_FALSE(positive_variance_to_px4_float(-1.0).has_value());
  EXPECT_FALSE(positive_variance_to_px4_float(
                   std::numeric_limits<double>::quiet_NaN())
                   .has_value());
  EXPECT_FALSE(positive_variance_to_px4_float(
                   std::numeric_limits<double>::infinity())
                   .has_value());
}

TEST(ExternalOdometryConversionTest, RejectsZeroTimestampQuaternionFrameAndUnavailableCovariance) {
  auto message = valid_message();
  message.header.stamp.sec = 0;
  message.header.stamp.nanosec = 0;
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
  message = valid_message();
  message.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
  message = valid_message();
  message.header.frame_id = "wrong";
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
  message = valid_message();
  message.pose.covariance[0] = 0.0;
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
  message = valid_message();
  message.pose.covariance[1] = 1.0;
  message.pose.covariance[6] = 0.0;
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
  message = valid_message();
  message.pose.covariance[0] = -1.0;
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
  message = valid_message();
  message.pose.covariance[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(convert_ros_lio_odometry(message).has_value());
}

}  // namespace
}  // namespace px4_odometry_bridge
