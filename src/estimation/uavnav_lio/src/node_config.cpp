#include "uavnav/lio_node/node_config.hpp"

#include <cmath>
#include <string>
#include <string_view>
#include <utility>

namespace uavnav::lio_node {

namespace {

using config::ConfigError;
using Kind = ConfigError::Kind;

std::unexpected<ConfigError> fail(Kind kind, std::string_view key, std::string detail) {
  return std::unexpected(ConfigError{kind, std::string(key), std::move(detail)});
}

// The value of `key`, re-checked against its kLioNodeSpecs entry (hand-built values never went through load_params).
Result<double, ConfigError> checked(const config::ParamValues& values, const config::ParamSpec& spec) {
  const auto v = config::value(values, spec.key);
  if (!v) return std::unexpected(v.error());
  if (!std::isfinite(*v)) return fail(Kind::kNotFinite, spec.key, std::string(spec.key) + " must be finite");
  if (*v < spec.min || *v > spec.max) {
    return fail(Kind::kOutOfRange, spec.key,
                std::string(spec.key) + " = " + std::to_string(*v) + " is outside [" + std::to_string(spec.min) +
                    ", " + std::to_string(spec.max) + "]");
  }
  return *v;
}

}  // namespace

Result<LioNodeConfig, config::ConfigError> load_lio_node_config(const config::ParamValues& values) {
  double v[kLioNodeSpecs.size()];
  for (std::size_t i = 0; i < kLioNodeSpecs.size(); ++i) {
    const auto value = checked(values, kLioNodeSpecs[i]);
    if (!value) return std::unexpected(value.error());
    v[i] = *value;
  }
  // kLioNodeSpecs order: x, y, z, roll, pitch, yaw. Fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
  LioNodeConfig out{Eigen::Isometry3d::Identity()};
  out.base_T_lidar.linear() = (Eigen::AngleAxisd(v[5], Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(v[4], Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(v[3], Eigen::Vector3d::UnitX()))
                                  .toRotationMatrix();
  out.base_T_lidar.translation() = Eigen::Vector3d(v[0], v[1], v[2]);
  return out;
}

Eigen::Isometry3d imu_T_lidar(const lio::LioConfig& lio) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = lio.math.t_imu_lidar_m;
  return t;
}

Eigen::Isometry3d compose_base_T_imu(const LioNodeConfig& node, const lio::LioConfig& lio) {
  return node.base_T_lidar * imu_T_lidar(lio).inverse();
}

}  // namespace uavnav::lio_node
