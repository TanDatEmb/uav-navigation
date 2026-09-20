#include "fast_lio_core/estimation/ikfom_state.hpp"

namespace uav::nav::lio {

Eigen::Matrix<double, 24, 1> ikfomProcessModel(
    IkfomState& state, const IkfomInput& input) {
  Eigen::Matrix<double, 24, 1> derivative =
      Eigen::Matrix<double, 24, 1>::Zero();
  IkfomVector3 omega;
  input.gyro.boxminus(omega, state.bg);
  const IkfomVector3 acceleration_odom =
      state.rot * (input.acc - state.ba);
  for (int index = 0; index < 3; ++index) {
    derivative(index) = state.vel[index];
    derivative(index + 3) = omega[index];
    derivative(index + 12) =
        acceleration_odom[index] + state.grav[index];
  }
  return derivative;
}

Eigen::Matrix<double, 24, 23> ikfomProcessJacobianState(
    IkfomState& state, const IkfomInput& input) {
  Eigen::Matrix<double, 24, 23> jacobian =
      Eigen::Matrix<double, 24, 23>::Zero();
  jacobian.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
  IkfomVector3 acceleration;
  input.acc.boxminus(acceleration, state.ba);
  jacobian.block<3, 3>(12, 3) =
      -state.rot.toRotationMatrix() * MTK::hat(acceleration);
  jacobian.block<3, 3>(12, 18) = -state.rot.toRotationMatrix();
  const Eigen::Matrix<double, 2, 1> zero =
      Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, 3, 2> gravity_jacobian;
  state.S2_Mx(gravity_jacobian, zero, 21);
  jacobian.block<3, 2>(12, 21) = gravity_jacobian;
  jacobian.block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
  return jacobian;
}

Eigen::Matrix<double, 24, 12> ikfomProcessJacobianNoise(
    IkfomState& state, const IkfomInput&) {
  Eigen::Matrix<double, 24, 12> jacobian =
      Eigen::Matrix<double, 24, 12>::Zero();
  jacobian.block<3, 3>(12, 3) = -state.rot.toRotationMatrix();
  jacobian.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
  return jacobian;
}

}  // namespace uav::nav::lio
