#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <mars_quadrotor_msgs/msg/polynomial_trajectory.hpp>

namespace px4_navigation_external_mode {

struct SuperPolynomialState {
  Eigen::Vector3d position{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d velocity{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  bool finished{false};
};

class SuperPolynomialTrajectory {
 public:
  static constexpr std::uint32_t kPositionTrajectory = 2U;
  static constexpr std::uint32_t kEmergencyStop = 16U;

  bool assign(const mars_quadrotor_msgs::msg::PolynomialTrajectory& message,
              std::string* error = nullptr);

  [[nodiscard]] bool valid() const noexcept { return valid_; }
  [[nodiscard]] std::uint32_t trajectoryId() const noexcept { return trajectory_id_; }
  [[nodiscard]] double startTimeSeconds() const noexcept { return start_time_s_; }
  [[nodiscard]] double totalDurationSeconds() const noexcept { return total_duration_s_; }
  [[nodiscard]] SuperPolynomialState evaluate(double now_seconds) const;

 private:
  static bool finiteVector(const std::vector<double>& values);
  static double evaluate(const std::vector<double>& coefficients,
                         std::size_t piece_count, std::size_t order,
                         const std::vector<double>& durations, double t,
                         int derivative);
  static bool fail(std::string* error, const char* message);

  bool valid_{false};
  bool emergency_stop_{false};
  std::uint32_t trajectory_id_{0U};
  double start_time_s_{0.0};
  double total_duration_s_{0.0};
  std::size_t piece_count_{0U};
  std::size_t order_{0U};
  std::vector<double> durations_;
  std::vector<double> coef_x_;
  std::vector<double> coef_y_;
  std::vector<double> coef_z_;
};

}  // namespace px4_navigation_external_mode
