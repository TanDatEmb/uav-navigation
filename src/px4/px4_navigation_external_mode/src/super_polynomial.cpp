#include "px4_navigation_external_mode/super_polynomial.hpp"

#include <algorithm>
#include <numeric>

namespace px4_navigation_external_mode {
namespace {

double timeSeconds(const builtin_interfaces::msg::Time& time) {
  return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
}

}  // namespace

bool SuperPolynomialTrajectory::fail(std::string* error, const char* message) {
  if (error != nullptr) *error = message;
  return false;
}

bool SuperPolynomialTrajectory::finiteVector(const std::vector<double>& values) {
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

bool SuperPolynomialTrajectory::assign(
    const mars_quadrotor_msgs::msg::PolynomialTrajectory& message,
    std::string* error) {
  valid_ = false;
  emergency_stop_ = (message.type & kEmergencyStop) != 0U;
  trajectory_id_ = message.trajectory_id;
  durations_.clear();
  coef_x_.clear();
  coef_y_.clear();
  coef_z_.clear();
  total_duration_s_ = 0.0;

  if (emergency_stop_) return fail(error, "SUPER trajectory contains EMER_STOP");
  if ((message.type & kPositionTrajectory) == 0U) {
    return fail(error, "SUPER trajectory does not contain POSITION_TRAJ");
  }
  if (message.trajectory_id == 0U || message.piece_num_pos == 0U ||
      message.order_pos == 0U || message.order_pos > 15U) {
    return fail(error, "SUPER trajectory has invalid id, piece count, or order");
  }
  const auto piece_count = static_cast<std::size_t>(message.piece_num_pos);
  const auto degree = static_cast<std::size_t>(message.order_pos);
  const auto coefficient_count = degree + 1U;
  if (message.time_pos.size() != piece_count ||
      message.coef_pos_x.size() != piece_count * coefficient_count ||
      message.coef_pos_y.size() != piece_count * coefficient_count ||
      message.coef_pos_z.size() != piece_count * coefficient_count) {
    return fail(error, "SUPER trajectory coefficient dimensions do not match metadata");
  }
  if (!finiteVector(message.time_pos) || !finiteVector(message.coef_pos_x) ||
      !finiteVector(message.coef_pos_y) || !finiteVector(message.coef_pos_z)) {
    return fail(error, "SUPER trajectory contains a non-finite value");
  }
  for (double duration : message.time_pos) {
    if (!(duration > 0.0)) return fail(error, "SUPER trajectory has non-positive duration");
    total_duration_s_ += duration;
  }
  const double start_time = timeSeconds(message.start_wt_pos);
  if (!std::isfinite(start_time) || start_time < 0.0) {
    return fail(error, "SUPER trajectory has an invalid start wall time");
  }

  piece_count_ = piece_count;
  order_ = coefficient_count;
  start_time_s_ = start_time;
  durations_ = message.time_pos;
  coef_x_ = message.coef_pos_x;
  coef_y_ = message.coef_pos_y;
  coef_z_ = message.coef_pos_z;

  valid_ = true;
  return true;
}

double SuperPolynomialTrajectory::evaluate(const std::vector<double>& coefficients,
                                           std::size_t piece_count,
                                           std::size_t order,
                                           const std::vector<double>& durations,
                                           double t, int derivative) {
  if (piece_count == 0U || order == 0U || derivative < 0 ||
      derivative >= static_cast<int>(order)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double local_t = std::clamp(t, 0.0, std::accumulate(durations.begin(), durations.end(), 0.0));
  std::size_t piece = 0U;
  while (piece + 1U < piece_count && local_t > durations[piece]) {
    local_t -= durations[piece++];
  }
  double result = 0.0;
  for (std::size_t i = order; i-- > static_cast<std::size_t>(derivative);) {
    const auto exponent = static_cast<int>(i);
    double multiplier = 1.0;
    for (int d = 0; d < derivative; ++d) multiplier *= static_cast<double>(exponent - d);
    result = result * local_t + coefficients[piece * order + i] * multiplier;
  }
  return result;
}

SuperPolynomialState SuperPolynomialTrajectory::evaluate(double now_seconds) const {
  SuperPolynomialState state;
  if (!valid_ || !std::isfinite(now_seconds)) return state;
  const double t = now_seconds - start_time_s_;
  state.finished = t >= total_duration_s_;
  const double local_t = std::max(0.0, t);
  state.position.x() = evaluate(coef_x_, piece_count_, order_, durations_, local_t, 0);
  state.position.y() = evaluate(coef_y_, piece_count_, order_, durations_, local_t, 0);
  state.position.z() = evaluate(coef_z_, piece_count_, order_, durations_, local_t, 0);
  state.velocity.x() = evaluate(coef_x_, piece_count_, order_, durations_, local_t, 1);
  state.velocity.y() = evaluate(coef_y_, piece_count_, order_, durations_, local_t, 1);
  state.velocity.z() = evaluate(coef_z_, piece_count_, order_, durations_, local_t, 1);
  state.acceleration.x() = evaluate(coef_x_, piece_count_, order_, durations_, local_t, 2);
  state.acceleration.y() = evaluate(coef_y_, piece_count_, order_, durations_, local_t, 2);
  state.acceleration.z() = evaluate(coef_z_, piece_count_, order_, durations_, local_t, 2);
  return state;
}

}  // namespace px4_navigation_external_mode
