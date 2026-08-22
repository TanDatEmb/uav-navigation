#include "px4_navigation_external_mode/super_polynomial.hpp"

#include <gtest/gtest.h>

namespace {

mars_quadrotor_msgs::msg::PolynomialTrajectory makeTrajectory() {
  mars_quadrotor_msgs::msg::PolynomialTrajectory message;
  message.trajectory_id = 1U;
  message.type = mars_quadrotor_msgs::msg::PolynomialTrajectory::POSITION_TRAJ;
  message.piece_num_pos = 1U;
  message.order_pos = 3U;
  message.start_wt_pos.sec = 10;
  message.time_pos = {2.0};
  // x(t) = 1 + 2t + 3t^2 + 4t^3; y(t)=2; z(t)=3.
  message.coef_pos_x = {1.0, 2.0, 3.0, 4.0};
  message.coef_pos_y = {2.0, 0.0, 0.0, 0.0};
  message.coef_pos_z = {3.0, 0.0, 0.0, 0.0};
  return message;
}

}  // namespace

TEST(SuperPolynomialTrajectoryTest, ValidatesAndEvaluatesPositionDerivatives) {
  px4_navigation_external_mode::SuperPolynomialTrajectory trajectory;
  std::string error;
  ASSERT_TRUE(trajectory.assign(makeTrajectory(), &error)) << error;
  const auto state = trajectory.evaluate(11.0);
  EXPECT_NEAR(state.position.x(), 10.0, 1e-9);
  EXPECT_NEAR(state.velocity.x(), 20.0, 1e-9);
  EXPECT_NEAR(state.acceleration.x(), 30.0, 1e-9);
  EXPECT_NEAR(state.position.y(), 2.0, 1e-9);
  EXPECT_FALSE(state.finished);
}

TEST(SuperPolynomialTrajectoryTest, RejectsDimensionMismatch) {
  auto message = makeTrajectory();
  message.coef_pos_z.pop_back();
  px4_navigation_external_mode::SuperPolynomialTrajectory trajectory;
  EXPECT_FALSE(trajectory.assign(message));
  EXPECT_FALSE(trajectory.valid());
}

TEST(SuperPolynomialTrajectoryTest, RejectsEmergencyStop) {
  auto message = makeTrajectory();
  message.type |= mars_quadrotor_msgs::msg::PolynomialTrajectory::EMER_STOP;
  px4_navigation_external_mode::SuperPolynomialTrajectory trajectory;
  EXPECT_FALSE(trajectory.assign(message));
}

TEST(SuperPolynomialTrajectoryTest, PreservesPieceBoundaryContinuity) {
  auto message = makeTrajectory();
  message.order_pos = 1U;
  message.piece_num_pos = 2U;
  message.start_wt_pos.sec = 0;
  message.time_pos = {1.0, 1.0};
  // x=t for the first piece and x=1+t for the second piece. Both pieces
  // meet at x=1 with equal velocity, so the concatenated trajectory is C1.
  message.coef_pos_x = {0.0, 1.0, 1.0, 1.0};
  message.coef_pos_y = {2.0, 0.0, 2.0, 0.0};
  message.coef_pos_z = {3.0, 0.0, 3.0, 0.0};
  px4_navigation_external_mode::SuperPolynomialTrajectory trajectory;
  ASSERT_TRUE(trajectory.assign(message));
  const auto boundary = trajectory.evaluate(1.0);
  const auto continuation = trajectory.evaluate(1.25);
  EXPECT_NEAR(boundary.position.x(), 1.0, 1e-9);
  EXPECT_NEAR(continuation.position.x(), 1.25, 1e-9);
  EXPECT_NEAR(boundary.velocity.x(), continuation.velocity.x(), 1e-9);
}
