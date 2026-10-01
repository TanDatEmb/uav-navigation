#include <gtest/gtest.h>

#include <Eigen/Core>

#include "fast_lio_core/estimation/ikfom_state.hpp"

namespace uav::nav::lio {
namespace {

TEST(IkfomJacobianTest, ProcessStateJacobianMatchesRightPerturbationFiniteDifference) {
  IkfomState state;
  state.pos = Eigen::Vector3d(1.0, -2.0, 0.5);
  state.rot = IkfomSo3{Eigen::Quaterniond(
      Eigen::AngleAxisd(0.3, Eigen::Vector3d(0.2, -0.7, 0.4).normalized()))};
  state.offset_R_L_I = IkfomSo3{};
  state.offset_T_L_I = Eigen::Vector3d(0.1, -0.02, 0.08);
  state.vel = Eigen::Vector3d(0.4, -0.3, 0.2);
  state.bg = Eigen::Vector3d(0.01, -0.02, 0.005);
  state.ba = Eigen::Vector3d(-0.1, 0.05, 0.02);
  state.grav = IkfomGravity{IkfomVector3{Eigen::Vector3d(0.0, 0.0, -9.809)}};
  IkfomInput input;
  input.gyro = Eigen::Vector3d(0.2, -0.1, 0.05);
  input.acc = Eigen::Vector3d(0.5, -0.4, 9.6);

  const auto analytical = ikfomProcessJacobianState(state, input);
  const auto nominal = ikfomProcessModel(state, input);
  constexpr double kStep = 1e-7;
  Eigen::Matrix<double, 24, 23> numerical;
  for (int column = 0; column < IkfomState::DOF; ++column) {
    IkfomState perturbed = state;
    Eigen::Matrix<double, IkfomState::DOF, 1> delta =
        Eigen::Matrix<double, IkfomState::DOF, 1>::Zero();
    delta[column] = kStep;
    perturbed.boxplus(delta);
    numerical.col(column) =
        (ikfomProcessModel(perturbed, input) - nominal) / kStep;
  }

  // Euclidean velocity/bias blocks and the SO(3)/S2 tangent blocks all share
  // the upstream 23-DoF ordering. Forward-difference truncation dominates.
  EXPECT_LT((analytical - numerical).cwiseAbs().maxCoeff(), 2e-5);
  EXPECT_TRUE((analytical.block<24, 3>(0, 6).isZero(1e-14)));
  EXPECT_TRUE((analytical.block<24, 3>(0, 9).isZero(1e-14)));
}

TEST(IkfomJacobianTest, ProcessNoiseJacobianUsesUpstreamNoiseOrdering) {
  IkfomState state;
  state.rot = IkfomSo3{Eigen::Quaterniond(
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()))};
  IkfomInput input;
  const auto jacobian = ikfomProcessJacobianNoise(state, input);
  EXPECT_TRUE((jacobian.block<3, 3>(3, 0).isApprox(
      -Eigen::Matrix3d::Identity())));
  EXPECT_TRUE((jacobian.block<3, 3>(12, 3).isApprox(
      -state.rot.toRotationMatrix())));
  EXPECT_TRUE((jacobian.block<3, 3>(15, 6).isApprox(
      Eigen::Matrix3d::Identity())));
  EXPECT_TRUE((jacobian.block<3, 3>(18, 9).isApprox(
      Eigen::Matrix3d::Identity())));
}

}  // namespace
}  // namespace uav::nav::lio
