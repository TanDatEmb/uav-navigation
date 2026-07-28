#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>

#include "reference/compact_ikfom_measurement_update.hpp"
#include "reference/dense_ikfom_measurement_update.hpp"

namespace uav::nav::lio {
namespace {

enum class Conditioning {
  kWellConditioned,
  kMultipleNoiseScales,
  kNearDegenerate,
  kWideCovarianceSpectrum,
};

struct CaseInput {
  Eigen::Matrix<double, 23, 23> covariance;
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd variance;
  Eigen::VectorXd innovation;
  Eigen::Matrix<double, 23, 1> dx_new;
  double condition_number{};
};

struct ErrorEnvelope {
  double gain_action{};
  double gain_times_jacobian{};
  double increment{};
  double manifold_state{};
  double covariance{};
  double increment_norm{};
};

ErrorEnvelope maximum_errors;

CaseInput makeCase(std::size_t measurement_dimension, std::uint32_t seed,
                   Conditioning conditioning) {
  std::mt19937 generator(seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  const auto random_matrix = [&](Eigen::Index rows, Eigen::Index cols) {
    Eigen::MatrixXd value(rows, cols);
    for (Eigen::Index row = 0; row < rows; ++row) {
      for (Eigen::Index column = 0; column < cols; ++column) {
        value(row, column) = normal(generator);
      }
    }
    return value;
  };

  CaseInput input;
  Eigen::Matrix<double, 23, 23> basis = random_matrix(23, 23);
  input.covariance =
      basis * basis.transpose() +
      Eigen::Matrix<double, 23, 23>::Identity() * 2.0;
  input.jacobian = random_matrix(measurement_dimension, 23);
  input.variance = Eigen::VectorXd::Constant(measurement_dimension, 0.05);
  input.innovation = random_matrix(measurement_dimension, 1);
  input.dx_new = random_matrix(23, 1);
  input.dx_new *= 1e-3;

  if (conditioning == Conditioning::kMultipleNoiseScales) {
    for (Eigen::Index index = 0; index < input.variance.size(); ++index) {
      const double exponent =
          -4.0 + 8.0 * static_cast<double>(index) /
                     std::max<Eigen::Index>(input.variance.size() - 1, 1);
      input.variance[index] = std::pow(10.0, exponent);
    }
  } else if (conditioning == Conditioning::kNearDegenerate) {
    for (Eigen::Index row = 1; row < input.jacobian.rows(); row += 3) {
      input.jacobian.row(row) =
          input.jacobian.row(row - 1) +
          1e-8 * random_matrix(1, 23);
    }
  } else if (conditioning == Conditioning::kWideCovarianceSpectrum) {
    Eigen::HouseholderQR<Eigen::Matrix<double, 23, 23>> qr(basis);
    const Eigen::Matrix<double, 23, 23> orthogonal =
        qr.householderQ() * Eigen::Matrix<double, 23, 23>::Identity();
    Eigen::Matrix<double, 23, 1> eigenvalues;
    for (Eigen::Index index = 0; index < eigenvalues.size(); ++index) {
      eigenvalues[index] = std::pow(
          10.0, -3.0 + 6.0 * static_cast<double>(index) / 22.0);
    }
    input.covariance =
        orthogonal * eigenvalues.asDiagonal() * orthogonal.transpose();
  }

  const Eigen::Matrix<double, 23, 23> information =
      input.jacobian.transpose() *
          input.variance.cwiseInverse().asDiagonal() * input.jacobian +
      input.covariance.inverse();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 23, 23>> solver(
      0.5 * (information + information.transpose()));
  input.condition_number =
      solver.eigenvalues().maxCoeff() / solver.eigenvalues().minCoeff();
  return input;
}

void expectEquivalent(const CaseInput& input, double absolute_tolerance,
                      double relative_tolerance) {
  IkfomState state;
  state.pos = IkfomVector3{Eigen::Vector3d(1.0, -2.0, 0.5)};
  state.rot =
      IkfomSo3{Eigen::Quaterniond(Eigen::AngleAxisd(
          0.2, Eigen::Vector3d(0.3, -0.4, 0.5).normalized()))};
  constexpr double kConvergenceLimit = 0.2;
  const auto dense = test_reference::denseMeasurementUpdate(
      input.covariance, input.jacobian, input.variance, input.innovation,
      input.dx_new, state, kConvergenceLimit);
  test_reference::MeasurementUpdate compact;
  ASSERT_TRUE(test_reference::compactMeasurementUpdate(
      input.covariance, input.jacobian, input.variance, input.innovation,
      input.dx_new, state, kConvergenceLimit, compact))
      << "condition number: " << input.condition_number;

  const auto error_limit = [&](double reference_norm) {
    return absolute_tolerance +
           relative_tolerance * std::max(reference_norm, 1.0);
  };
  const double gain_action_error =
      (compact.gain * input.innovation - dense.gain * input.innovation)
          .cwiseAbs()
          .maxCoeff();
  const double kh_error =
      (compact.gain_times_jacobian - dense.gain_times_jacobian)
          .cwiseAbs()
          .maxCoeff();
  const double increment_error =
      (compact.increment - dense.increment).cwiseAbs().maxCoeff();
  maximum_errors.gain_action =
      std::max(maximum_errors.gain_action, gain_action_error);
  maximum_errors.gain_times_jacobian =
      std::max(maximum_errors.gain_times_jacobian, kh_error);
  maximum_errors.increment =
      std::max(maximum_errors.increment, increment_error);
  EXPECT_LE(gain_action_error,
            error_limit((dense.gain * input.innovation).norm()));
  EXPECT_LE(kh_error,
            error_limit(dense.gain_times_jacobian.norm()));
  EXPECT_LE(increment_error,
            error_limit(dense.increment.norm()));
  Eigen::Matrix<double, 23, 1> state_difference;
  compact.corrected_state.boxminus(state_difference, dense.corrected_state);
  const double state_error = state_difference.cwiseAbs().maxCoeff();
  const double covariance_error =
      (compact.corrected_covariance - dense.corrected_covariance)
          .cwiseAbs()
          .maxCoeff();
  const double norm_error =
      std::abs(compact.final_increment_norm - dense.final_increment_norm);
  maximum_errors.manifold_state =
      std::max(maximum_errors.manifold_state, state_error);
  maximum_errors.covariance =
      std::max(maximum_errors.covariance, covariance_error);
  maximum_errors.increment_norm =
      std::max(maximum_errors.increment_norm, norm_error);
  EXPECT_LE(state_error,
            error_limit(dense.increment.norm()));
  EXPECT_LE(covariance_error,
            error_limit(dense.corrected_covariance.norm()));
  EXPECT_EQ(compact.converged, dense.converged);
  EXPECT_LE(norm_error,
            error_limit(dense.final_increment_norm));
}

void runMatrix(Conditioning conditioning, double absolute_tolerance,
               double relative_tolerance) {
  constexpr std::array<std::size_t, 6> kMeasurementDimensions{
      5, 22, 23, 24, 50, 200};
  for (const auto dimension : kMeasurementDimensions) {
    for (std::uint32_t seed = 0; seed < 20; ++seed) {
      SCOPED_TRACE("M=" + std::to_string(dimension) +
                   " seed=" + std::to_string(seed));
      expectEquivalent(makeCase(dimension, 0x5A17U + seed, conditioning),
                       absolute_tolerance, relative_tolerance);
    }
  }
}

TEST(IkfomCompactEquivalence, DenseVsCompactWellConditioned) {
  runMatrix(Conditioning::kWellConditioned, 1e-9, 1e-9);
}

TEST(IkfomCompactEquivalence, DenseVsCompactLargeMeasurement) {
  for (std::uint32_t seed = 0; seed < 20; ++seed) {
    expectEquivalent(
        makeCase(200, 0xB00U + seed, Conditioning::kWellConditioned),
        1e-9, 1e-9);
  }
}

TEST(IkfomCompactEquivalence, DenseVsCompactNearDegenerate) {
  runMatrix(Conditioning::kNearDegenerate, 1e-8, 1e-7);
  runMatrix(Conditioning::kWideCovarianceSpectrum, 1e-8, 1e-7);
}

TEST(IkfomCompactEquivalence, DenseVsCompactMultipleNoiseScales) {
  runMatrix(Conditioning::kMultipleNoiseScales, 1e-8, 1e-7);
}

TEST(IkfomCompactEquivalence, CompactSolverFailureIsRejected) {
  Eigen::Matrix<double, 23, 23> covariance =
      Eigen::Matrix<double, 23, 23>::Zero();
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Random(50, 23);
  Eigen::VectorXd variance = Eigen::VectorXd::Ones(50);
  Eigen::Matrix<double, 23, Eigen::Dynamic> gain;
  EXPECT_FALSE((esekfom::detail::solve_compact_normal_equations<double, 23>(
      covariance, jacobian, variance, gain)));
  covariance.setIdentity();
  variance[3] = 0.0;
  EXPECT_FALSE((esekfom::detail::solve_compact_normal_equations<double, 23>(
      covariance, jacobian, variance, gain)));
}

TEST(IkfomCompactEquivalence, ReportsMaximumDifferentialErrors) {
  std::cout << "maximum_gain_action_error="
            << maximum_errors.gain_action
            << " maximum_KH_error="
            << maximum_errors.gain_times_jacobian
            << " maximum_increment_error=" << maximum_errors.increment
            << " maximum_manifold_state_error="
            << maximum_errors.manifold_state
            << " maximum_covariance_error=" << maximum_errors.covariance
            << " maximum_increment_norm_error="
            << maximum_errors.increment_norm << '\n';
}

}  // namespace
}  // namespace uav::nav::lio
