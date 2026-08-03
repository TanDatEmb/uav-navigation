#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "fast_lio_core/navigation/base_link_state_converter.hpp"
#include "fast_lio_ros/ros_propagated_odometry_publisher.hpp"

namespace uav::nav::lio {

class RosPropagatedOdometryPublisherTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

KinematicStateEstimate estimateAt(std::int64_t time_ns) {
  KinematicStateEstimate estimate;
  estimate.estimate.time = Timestamp(time_ns);
  estimate.angular_velocity_imu_rad_s = Eigen::Vector3d(0.1, 0.2, 0.3);
  return estimate;
}

TEST_F(RosPropagatedOdometryPublisherTest, ConvertsAndPublishesOwnedEstimate) {
  rclcpp::Node node("propagated_publisher_test");
  RosParameters parameters;
  parameters.odom_frame = "lio_odom";
  parameters.base_frame = "base_link";
  parameters.imu_frame = "livox_imu_frame";
  auto covariance_runtime = std::make_shared<CovarianceProjectionRuntime>();
  RosPropagatedOdometryPublisher publisher(node, parameters, covariance_runtime);
  publisher.setBaseLinkConverter(std::make_shared<const BaseLinkStateConverter>(
      RigidTransform(baseFrame(), imuFrame(), Eigen::Quaterniond::Identity(),
                     Eigen::Vector3d::Zero())));
  EXPECT_NO_THROW(publisher.publish(estimateAt(20'000'000)));
}

}  // namespace uav::nav::lio
