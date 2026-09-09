#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "data_structure/base/trajectory.h"
#include <navigation_planning/planner_diagnostics.hpp>
#include <navigation_mapping/mapping_world_snapshot.hpp>
#include <utils/geometry/geometry_utils.h>
#include <data_structure/base/polytope.h>
#include <planner_core/config.hpp>
#include <planner_core/corridor_generator.h>
#include <planner_core/guide_vertical_envelope.hpp>
#include <planner_core/corridor_bezier_seed.hpp>
#include <planner_core/corridor_plane_validation.hpp>
#include <planner_core/deterministic_nominal_seed.hpp>
#include <planner_core/trajectory_world_validator.hpp>
#include <traj_opt/config.hpp>
#include <traj_opt/minco.h>
#include <traj_opt/nominal_trajectory_optimizer.hpp>
#include <traj_opt/trajectory_dynamics.hpp>

namespace {

using navigation_math::Mat3Df;
using navigation_math::MatD4f;
using navigation_math::PolyhedraH;
using navigation_math::PolyhedronV;
using navigation_math::Vec3f;
using navigation_math::VecDf;
using navigation_math::VecDi;
using navigation_math::vec_Vec3f;

constexpr double kDiagnosticTolerance = 1.0e-8;

double scalarOrNaN(const YAML::Node& node) {
  if (!node || node.IsNull()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return node.as<double>();
}

navigation_world_model::Point3 readWorldPoint(const YAML::Node& node) {
  if (!node || !node.IsSequence() || node.size() != 3U) {
    return navigation_world_model::Point3::Constant(
        std::numeric_limits<double>::quiet_NaN());
  }
  navigation_world_model::Point3 result;
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    const auto value = node[axis].IsSequence() ? node[axis][0] : node[axis];
    result(static_cast<Eigen::Index>(axis)) = scalarOrNaN(value);
  }
  return result;
}

navigation_world_model::GridIndex3 readWorldIndex(const YAML::Node& node) {
  if (!node || !node.IsSequence() || node.size() != 3U) {
    return navigation_world_model::GridIndex3::Zero();
  }
  navigation_world_model::GridIndex3 result;
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    result(static_cast<Eigen::Index>(axis)) = node[axis].as<int>();
  }
  return result;
}

std::vector<std::uint8_t> readByteVector(const YAML::Node& node) {
  std::vector<std::uint8_t> result;
  if (!node || !node.IsSequence()) return result;
  result.reserve(node.size());
  for (std::size_t index = 0; index < node.size(); ++index) {
    const int value = node[index].as<int>();
    if (value < 0 || value > 255) {
      throw std::runtime_error("world snapshot byte is out of range");
    }
    result.push_back(static_cast<std::uint8_t>(value));
  }
  return result;
}

std::shared_ptr<const navigation_world_model::WorldModelView>
readWorldSnapshot(const YAML::Node& node) {
  if (!node || !node.IsMap() || !node["complete"].as<bool>()) return {};
  const auto geometry = node["geometry"];
  if (!geometry || !geometry.IsMap()) return {};

  navigation_mapping::PlanningGrid grid;
  grid.base_layout = navigation_mapping::PlanningGridLayout{
      geometry["evidence_resolution_m"].as<double>(),
      readWorldIndex(geometry["evidence_bounds"]["global_min_index"]),
      readWorldIndex(geometry["evidence_bounds"]["dimensions"]),
      readWorldPoint(geometry["local_center_m"]),
      readWorldPoint(geometry["local_size_m"]),
  };
  grid.inflated.layout = navigation_mapping::PlanningGridLayout{
      geometry["inflated_resolution_m"].as<double>(),
      readWorldIndex(geometry["inflated_bounds"]["global_min_index"]),
      readWorldIndex(geometry["inflated_bounds"]["dimensions"]),
      readWorldPoint(geometry["local_center_m"]),
      readWorldPoint(geometry["local_size_m"]),
  };
  grid.base_state = readByteVector(node["evidence_states"]);
  const auto inflated_states = readByteVector(node["inflated_states"]);
  grid.unknown_inflation_enabled =
      node["unknown_inflation_enabled"].as<bool>();
  grid.inflated.occupied.assign(inflated_states.size(), 0U);
  if (grid.unknown_inflation_enabled) {
    grid.inflated.unknown.assign(inflated_states.size(), 0U);
  }
  for (std::size_t index = 0; index < inflated_states.size(); ++index) {
    const auto state = static_cast<navigation_world_model::CellState>(
        inflated_states[index]);
    if (state == navigation_world_model::CellState::kOccupied) {
      grid.inflated.occupied[index] = 1U;
    }
    if (grid.unknown_inflation_enabled &&
        state == navigation_world_model::CellState::kUnknown) {
      grid.inflated.unknown[index] = 1U;
    }
  }
  auto nearest_offsets = std::make_shared<
      std::vector<navigation_world_model::GridIndex3>>();
  const auto offsets = node["nearest_offsets"];
  if (!offsets || !offsets.IsSequence()) return {};
  nearest_offsets->reserve(offsets.size());
  for (const auto& offset : offsets) {
    nearest_offsets->push_back(readWorldIndex(offset));
  }
  grid.nearest_offsets = std::move(nearest_offsets);
  grid.virtual_ground_ceiling_enabled =
      node["virtual_ground_ceiling_enabled"].as<bool>();
  grid.virtual_ground_m = node["virtual_ground_m"].as<double>();
  grid.virtual_ceiling_m = node["virtual_ceiling_m"].as<double>();
  grid.inflated_virtual_ground_m =
      node["inflated_virtual_ground_m"].as<double>();
  grid.inflated_virtual_ceiling_m =
      node["inflated_virtual_ceiling_m"].as<double>();
  grid.occupied_inflation_radius_m =
      geometry["occupied_inflation_radius_m"].as<double>();

  const auto identity_node = node["identity"];
  if (!identity_node || !identity_node.IsMap()) return {};
  const navigation_world_model::WorldSnapshotIdentity identity{
      identity_node["localization_epoch"].as<std::uint64_t>(),
      identity_node["generation"].as<std::uint64_t>(),
      identity_node["revision"].as<std::uint64_t>(),
      identity_node["observation_stamp_ns"].as<std::int64_t>(),
  };
  return std::make_shared<navigation_mapping::MappingWorldSnapshot>(
      std::move(grid), identity);
}

Eigen::MatrixXd readDynamicMatrix(const YAML::Node& node) {
  if (!node || !node.IsSequence()) return {};
  if (node.size() == 0U) return {};
  const std::size_t rows = node.size();
  if (!node[0].IsSequence()) return {};
  const std::size_t cols = node[0].size();
  Eigen::MatrixXd result(static_cast<Eigen::Index>(rows),
                         static_cast<Eigen::Index>(cols));
  for (std::size_t row = 0; row < rows; ++row) {
    if (!node[row].IsSequence() || node[row].size() != cols) return {};
    for (std::size_t col = 0; col < cols; ++col) {
      result(static_cast<Eigen::Index>(row),
             static_cast<Eigen::Index>(col)) =
          scalarOrNaN(node[row][col]);
    }
  }
  return result;
}

Mat3Df readMatrix(const YAML::Node& node) {
  const Eigen::MatrixXd generic = readDynamicMatrix(node);
  if (generic.rows() != 3) return {};
  return generic;
}

MatD4f readPlaneMatrix(const YAML::Node& node) {
  const Eigen::MatrixXd generic = readDynamicMatrix(node);
  if (generic.rows() == 0 || generic.cols() != 4) return {};
  return generic;
}

navigation_math::StatePVAJ readState(const YAML::Node& node) {
  const Mat3Df state = readMatrix(node);
  if (state.rows() != 3 || state.cols() != 4) {
    return navigation_math::StatePVAJ::Constant(
        std::numeric_limits<double>::quiet_NaN());
  }
  return state;
}

void printStateColumns(const char* label,
                       const navigation_math::StatePVAJ& state) {
  std::cout << label << "_matrix_rows=" << state.rows()
            << " cols=" << state.cols() << '\n';
  for (int derivative = 0; derivative < 4; ++derivative) {
    const char* name = derivative == 0 ? "P" :
                       derivative == 1 ? "V" :
                       derivative == 2 ? "A" : "J";
    std::cout << label << '_' << name << '='
              << state.col(derivative).transpose() << '\n';
  }
}

bool matrixNodeMatches(const YAML::Node& node,
                       const Eigen::MatrixXd& matrix) {
  if (!node || !node.IsSequence() ||
      static_cast<Eigen::Index>(node.size()) != matrix.rows() ||
      matrix.cols() == 0) {
    return false;
  }
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    if (!node[static_cast<std::size_t>(row)].IsSequence() ||
        static_cast<Eigen::Index>(
            node[static_cast<std::size_t>(row)].size()) != matrix.cols()) {
      return false;
    }
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      const double serialized = scalarOrNaN(
          node[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)]);
      if (serialized != matrix(row, column)) {
        return false;
      }
    }
  }
  return true;
}

VecDf readDoubleVector(const YAML::Node& node) {
  if (!node || !node.IsSequence()) return {};
  VecDf result(static_cast<Eigen::Index>(node.size()));
  for (std::size_t index = 0; index < node.size(); ++index) {
    result(static_cast<Eigen::Index>(index)) = scalarOrNaN(node[index]);
  }
  return result;
}

VecDi readIntVector(const YAML::Node& node) {
  if (!node || !node.IsSequence()) return {};
  VecDi result(static_cast<Eigen::Index>(node.size()));
  for (std::size_t index = 0; index < node.size(); ++index) {
    result(static_cast<Eigen::Index>(index)) =
        node[index].IsNull() ? -1 : node[index].as<int>();
  }
  return result;
}

vec_Vec3f readPointSequence(const YAML::Node& node) {
  vec_Vec3f result;
  if (!node || !node.IsSequence()) return result;
  result.reserve(node.size());
  for (std::size_t index = 0; index < node.size(); ++index) {
    const auto& point = node[index];
    if (!point.IsSequence() || point.size() != 3U) {
      result.clear();
      return result;
    }
    const auto coordinate = [&point](const std::size_t axis) {
      return point[axis].IsSequence()
          ? scalarOrNaN(point[axis][0])
          : scalarOrNaN(point[axis]);
    };
    result.emplace_back(
        coordinate(0),
        coordinate(1),
        coordinate(2));
  }
  return result;
}

PolyhedraH readPolyhedra(const YAML::Node& node) {
  PolyhedraH result;
  if (!node || !node.IsSequence()) return result;
  result.reserve(node.size());
  for (std::size_t index = 0; index < node.size(); ++index) {
    result.emplace_back(readPlaneMatrix(node[index]));
  }
  return result;
}

geometry_utils::PolytopeVec makePolytopes(
    const PolyhedraH& planes,
    const YAML::Node& gates,
    const YAML::Node& points,
    const YAML::Node& radii) {
  geometry_utils::PolytopeVec result;
  result.reserve(planes.size());
  for (std::size_t index = 0; index < planes.size(); ++index) {
    MatD4f normalized = planes[index];
    if (!navigation_planning_backend::normalizeCorridorPlanes(normalized)) {
      throw std::runtime_error("snapshot contains invalid corridor planes");
    }
    geometry_utils::Polytope polytope(std::move(normalized));
    const bool gated = gates && index < gates.size() &&
                       gates[index].as<int>() != 0;
    if (gated && points && radii && index < points.size() &&
        index < radii.size()) {
      const auto point_sequence = points[index];
      if (point_sequence.IsSequence() && point_sequence.size() == 3U) {
        const auto coordinate = [&point_sequence](const std::size_t axis) {
          return point_sequence[axis].IsSequence()
              ? scalarOrNaN(point_sequence[axis][0])
              : scalarOrNaN(point_sequence[axis]);
        };
        const Vec3f point(
            coordinate(0),
            coordinate(1),
            coordinate(2));
        const double radius = scalarOrNaN(radii[index]);
        polytope.SetRouteBoundaryContract(point, radius);
      }
    }
    result.emplace_back(std::move(polytope));
  }
  return result;
}

struct CandidateReport {
  bool constructed{false};
  double head_residual{std::numeric_limits<double>::infinity()};
  double tail_residual{std::numeric_limits<double>::infinity()};
  double corridor_violation{std::numeric_limits<double>::infinity()};
  bool route_boundary{false};
  std::string route_boundary_verdict{"NOT_REACHED"};
  int production_certificate_stage{-1};
  bool production_certificate_valid{false};
  double maximum_velocity{std::numeric_limits<double>::infinity()};
  double maximum_acceleration{std::numeric_limits<double>::infinity()};
  double maximum_jerk{std::numeric_limits<double>::infinity()};
  bool va_j{false};
  bool flatness{false};
  bool world{false};
  bool complete_executable_bundle{false};
  std::string world_failure{"not_replayed"};
  double duration{std::numeric_limits<double>::quiet_NaN()};
};

double boundaryResidual(const navigation_math::StatePVAJ& actual,
                        const navigation_math::StatePVAJ& expected) {
  if (!actual.allFinite() || !expected.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  return (actual - expected).cwiseAbs().maxCoeff();
}

bool worldTrajectoryPass(
        const geometry_utils::Trajectory& trajectory,
        const navigation_world_model::WorldModelView& world,
        const navigation_world_model::UnknownPolicy policy,
        std::string& failure) {
  const auto geometry = world.geometry();
  const double resolution = geometry.inflated_resolution_m;
  const double spatial_step = 0.5 * resolution;
  const double curve_deviation_tolerance =
      0.25 * resolution;
  const double duration = trajectory.getTotalDuration();
  if (trajectory.empty() || !std::isfinite(duration) || duration <= 0.0 ||
      !std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(spatial_step) || spatial_step <= 0.0) {
    failure = "invalid_world_or_trajectory";
    return false;
  }
  double time = 0.0;
  auto previous = trajectory.getPos(0.0);
  if (!previous.allFinite()) {
    failure = "nonfinite_initial_position";
    return false;
  }
  if (!navigation_world_model::isCellTraversable(
          world.classify(previous, navigation_world_model::GridLayer::kInflated),
          policy)) {
    failure = "initial_point_blocked";
    return false;
  }
  while (time < duration) {
    const auto location = navigation_planning_backend::locatePieceForSweep(
        trajectory, time);
    if (!location) {
      failure = "piece_lookup_failed";
      return false;
    }
    const auto& piece = trajectory[location->index];
    const double speed = std::max(0.1, trajectory.getVel(time).norm());
    double dt = std::clamp(spatial_step / speed, 0.002, 0.05);
    auto next_time = std::min({duration, time + dt, location->end_time});
    if (!(next_time > time)) {
      failure = "nonadvancing_world_sweep";
      return false;
    }
    auto next = trajectory.getPos(next_time);
    while (next.allFinite() && (next - previous).norm() > spatial_step &&
           dt > 0.002 + 1.0e-12) {
      dt = std::max(0.002, 0.5 * dt);
      next_time = std::min({duration, time + dt, location->end_time});
      next = trajectory.getPos(next_time);
    }
    const double segment_dt = next_time - time;
    const double local_end = std::clamp(
        location->local_time + segment_dt, 0.0, piece.getDuration());
    const double acceleration_bound =
        navigation_planning_backend::polynomialAccelerationBoundOverInterval(
            piece, location->local_time, local_end);
    const double curve_deviation =
        acceleration_bound * segment_dt * segment_dt / 8.0;
    if (!std::isfinite(curve_deviation) ||
        curve_deviation > curve_deviation_tolerance) {
      failure = "curve_deviation_bound_exceeded";
      return false;
    }
    navigation_world_model::CellState blocked_state =
        navigation_world_model::CellState::kUndefined;
    navigation_world_model::Point3 blocked_position =
        navigation_world_model::Point3::Constant(
            std::numeric_limits<double>::quiet_NaN());
    if (!navigation_planning_backend::certificateTubeIsSafe(
            world, previous, next, curve_deviation, policy, resolution,
            &blocked_state, &blocked_position)) {
      failure = "tube_blocked_state=" +
          std::to_string(static_cast<int>(blocked_state));
      if (blocked_position.allFinite()) {
        failure += " position=" + std::to_string(blocked_position.x()) +
            "," + std::to_string(blocked_position.y()) +
            "," + std::to_string(blocked_position.z());
      }
      return false;
    }
    if (!world.isSegmentTraversable(
            previous, next, navigation_world_model::GridLayer::kInflated,
            policy) ||
        !navigation_world_model::isCellTraversable(
            world.classify(next, navigation_world_model::GridLayer::kInflated),
            policy)) {
      failure = "endpoint_or_segment_blocked";
      return false;
    }
    previous = next;
    time = next_time;
  }
  return true;
}

CandidateReport certify(const geometry_utils::Trajectory& trajectory,
                        const navigation_math::StatePVAJ& head,
                        const navigation_math::StatePVAJ& tail,
                        const PolyhedraH& corridors,
                        const VecDi& piece_to_corridor,
                        const geometry_utils::PolytopeVec& polytope_vec,
                        const traj_opt::Config& config,
                        const navigation_world_model::WorldModelView* world) {
  CandidateReport report;
  report.constructed = !trajectory.empty();
  report.world = false;
  report.world_failure = world == nullptr ? "not_replayed" : "failed";
  if (!report.constructed || piece_to_corridor.size() != trajectory.getPieceNum()) {
    return report;
  }
  report.duration = trajectory.getTotalDuration();
  navigation_math::StatePVAJ actual_head;
  navigation_math::StatePVAJ actual_tail;
  report.head_residual = trajectory.getState(0.0, actual_head)
      ? boundaryResidual(actual_head, head)
      : std::numeric_limits<double>::infinity();
  report.tail_residual = trajectory.getState(report.duration, actual_tail)
      ? boundaryResidual(actual_tail, tail)
      : std::numeric_limits<double>::infinity();

  report.corridor_violation = -std::numeric_limits<double>::infinity();
  for (int piece = 0; piece < trajectory.getPieceNum(); ++piece) {
    const int corridor = piece_to_corridor(piece);
    if (corridor < 0 || corridor >= static_cast<int>(corridors.size())) {
      report.corridor_violation = std::numeric_limits<double>::infinity();
      break;
    }
    auto planes = corridors[static_cast<std::size_t>(corridor)];
    if (!navigation_planning_backend::normalizeCorridorPlanes(planes)) {
      report.corridor_violation = std::numeric_limits<double>::infinity();
      break;
    }
    report.corridor_violation = std::max(
        report.corridor_violation,
        navigation_planning_backend::maximumContinuousCorridorPlaneViolation(
            trajectory[piece], planes));
  }
  std::vector<unsigned char> route_boundary_gates(polytope_vec.size(), 0U);
  std::vector<Vec3f> route_boundary_points(
      polytope_vec.size(), Vec3f::Constant(
          std::numeric_limits<float>::quiet_NaN()));
  std::vector<double> route_boundary_radii(
      polytope_vec.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < polytope_vec.size(); ++index) {
    if (!polytope_vec[index].IsRouteBoundaryGate()) continue;
    route_boundary_gates[index] = 1U;
    route_boundary_points[index] = polytope_vec[index].GetRouteBoundaryPoint();
    route_boundary_radii[index] = polytope_vec[index].GetRouteBoundaryRadius();
  }
  const auto production_certificate =
      navigation_planning_backend::certifyDeterministicNominalSeed(
          trajectory, corridors, piece_to_corridor, route_boundary_gates,
          route_boundary_points, route_boundary_radii, head, tail, config);
  report.production_certificate_stage =
      static_cast<int>(production_certificate.failure_stage);
  report.production_certificate_valid = production_certificate.valid;
  const auto certificate_stage = production_certificate.failure_stage;
  report.route_boundary =
      certificate_stage == navigation_planning_backend::
          DeterministicNominalSeedFailureStage::kDynamics ||
      certificate_stage == navigation_planning_backend::
          DeterministicNominalSeedFailureStage::kFlatness ||
      certificate_stage == navigation_planning_backend::
          DeterministicNominalSeedFailureStage::kNone;
  report.route_boundary_verdict =
      certificate_stage == navigation_planning_backend::
          DeterministicNominalSeedFailureStage::kRouteBoundary
          ? "FAIL"
          : (report.route_boundary ? "PASS" : "NOT_REACHED");
  report.maximum_velocity = trajectory.getMaxVelRate();
  report.maximum_acceleration = trajectory.getMaxAccRate();
  report.maximum_jerk = trajectory.getMaxJerRate();
  report.va_j = std::isfinite(report.maximum_velocity) &&
               std::isfinite(report.maximum_acceleration) &&
               std::isfinite(report.maximum_jerk) &&
               report.maximum_velocity <= config.max_vel + kDiagnosticTolerance &&
               report.maximum_acceleration <= config.max_acc + kDiagnosticTolerance &&
               report.maximum_jerk <= config.max_jerk + kDiagnosticTolerance;
  report.flatness = traj_opt::trajectorySatisfiesFlatnessEnvelope(
      trajectory, config, nullptr);
  if (world != nullptr) {
    report.world = worldTrajectoryPass(
        trajectory, *world,
        navigation_world_model::UnknownPolicy::kAllowUnknown,
        report.world_failure);
  }
  // A world-only PASS is deliberately not an executable-bundle verdict. The
  // replay has no planner-owned MAIN/BACKUP role schedule or commit lease.
  report.complete_executable_bundle = false;
  return report;
}

void printReport(const std::string& label, const CandidateReport& report,
                const bool config_exact) {
  std::cout << label
            << " constructed=" << report.constructed
            << " head_residual=" << report.head_residual
            << " tail_residual=" << report.tail_residual
            << " corridor_violation=" << report.corridor_violation
            << " route_boundary=" << report.route_boundary
            << " route_boundary_verdict=" << report.route_boundary_verdict
            << " production_certificate_stage="
            << report.production_certificate_stage
            << " production_certificate=" << report.production_certificate_valid
            << " V=" << report.maximum_velocity
            << " A=" << report.maximum_acceleration
            << " J=" << report.maximum_jerk
            << " V/A/J=" << report.va_j
            << " flatness=" << report.flatness
            << " world=" << (report.world ? "PASS" : report.world_failure)
            << " complete_executable_bundle="
            << report.complete_executable_bundle
            << " exact_verdict="
            << (config_exact ? "ELIGIBLE" : "INCONCLUSIVE_LEGACY_CONFIG")
            << " duration=" << report.duration << '\n';
}

struct ConfigReplay {
  traj_opt::Config config;
  bool exact{true};
  std::vector<std::string> missing;
};

ConfigReplay makeConfig(const YAML::Node& snapshot_config) {
  traj_opt::Config config(PLANNER_EXP_CONFIG_PATH, "exp_traj");
  ConfigReplay result{config};
  if (!snapshot_config || !snapshot_config.IsMap()) {
    result.exact = false;
    result.missing.emplace_back("config");
    return result;
  }
  const auto require = [&result, &snapshot_config](const char* name) {
    if (!snapshot_config[name] || snapshot_config[name].IsNull()) {
      result.exact = false;
      result.missing.emplace_back(name);
    }
  };
  const auto setBool = [&snapshot_config, &require](const char* name, bool& value) {
    require(name);
    if (snapshot_config[name] && !snapshot_config[name].IsNull()) {
      value = snapshot_config[name].as<bool>();
    }
  };
  const auto setDouble = [&snapshot_config](const char* name, double& value) {
    if (snapshot_config[name] && !snapshot_config[name].IsNull()) {
      value = snapshot_config[name].as<double>();
    }
  };
  const auto setInt = [&snapshot_config](const char* name, int& value) {
    if (snapshot_config[name] && !snapshot_config[name].IsNull()) {
      value = snapshot_config[name].as<int>();
    }
  };
  setBool("uniform_time_en", config.uniform_time_en);
  setBool("print_optimizer_log", config.print_optimizer_log);
  setBool("save_log_en", config.save_log_en);
  setBool("block_energy_cost", config.block_energy_cost);
  const auto setRequiredInt = [&require, &setInt](const char* name, int& value) {
    require(name);
    setInt(name, value);
  };
  const auto setRequiredDouble = [&require, &setDouble](const char* name, double& value) {
    require(name);
    setDouble(name, value);
  };
  setRequiredInt("pos_constraint_type", config.pos_constraint_type);
  setRequiredInt("piece_num", config.piece_num);
  setRequiredInt("integral_reso", config.integral_reso);
  setRequiredInt("feasibility_retry_max_iterations",
         config.feasibility_retry_max_iterations);
  setRequiredInt("lbfgs_memory_size", config.lbfgs_memory_size);
  setRequiredDouble("mass", config.mass); setRequiredDouble("dh", config.dh);
  setRequiredDouble("dv", config.dv); setRequiredDouble("grav", config.grav);
  setRequiredDouble("cp", config.cp); setRequiredDouble("v_eps", config.v_eps);
  setRequiredDouble("max_vel", config.max_vel); setRequiredDouble("max_acc", config.max_acc);
  setRequiredDouble("max_jerk", config.max_jerk); setRequiredDouble("max_omg", config.max_omg);
  setRequiredDouble("max_acc_thr", config.max_acc_thr);
  setRequiredDouble("min_acc_thr", config.min_acc_thr);
  setRequiredDouble("velocity_penalty_weight", config.velocity_penalty_weight);
  setRequiredDouble("acceleration_penalty_weight", config.acceleration_penalty_weight);
  setRequiredDouble("jerk_penalty_weight", config.jerk_penalty_weight);
  setRequiredDouble("angular_rate_penalty_weight", config.angular_rate_penalty_weight);
  setRequiredDouble("thrust_penalty_weight", config.thrust_penalty_weight);
  setRequiredDouble("time_weight", config.time_weight);
  setRequiredDouble("position_penalty_weight", config.position_penalty_weight);
  setRequiredDouble("waypoint_attraction_weight", config.waypoint_attraction_weight);
  setRequiredDouble("terminal_time_weight", config.terminal_time_weight);
  setRequiredDouble("smooth_eps", config.smooth_eps);
  setRequiredDouble("corridor_plane_tolerance_m", config.corridor_plane_tolerance_m);
  setRequiredDouble("route_reference_lateral_weight",
            config.route_reference_lateral_weight);
  setRequiredDouble("route_reference_vertical_weight",
            config.route_reference_vertical_weight);
  setRequiredDouble("route_reference_lateral_deadband_m",
            config.route_reference_lateral_deadband_m);
  setRequiredDouble("route_reference_vertical_deadband_m",
            config.route_reference_vertical_deadband_m);
  setRequiredDouble("optimization_dynamic_reserve_ratio",
            config.optimization_dynamic_reserve_ratio);
  setRequiredDouble("opt_accuracy", config.opt_accuracy);
  config.quadrotor_flatness.reset(
      config.mass, config.grav, config.dh, config.dv, config.cp, config.v_eps);
  config.validate();
  result.config = config;
  return result;
}

std::shared_ptr<navigation_planner_context::PlannerRuntimeContext>
makeContext();

geometry_utils::PolytopeVec reconstructPreSimplifyCorridor(
    const vec_Vec3f& guide_path,
    const std::shared_ptr<const navigation_world_model::WorldModelView>& world,
    bool& search_ok,
    bool& vertical_envelope_ok) {
  search_ok = false;
  vertical_envelope_ok = false;
  if (!world || guide_path.size() < 2U) return {};

  // This is an offline provenance aid for legacy snapshots which predate the
  // pre-SimplifySFC field.  It invokes the same corridor producer with the
  // snapshot's immutable world and guide; it never feeds the regenerated
  // chain into the optimizer or changes production behavior.
  navigation_planning_backend::Config planner_config(
      PLANNER_EXP_CONFIG_PATH);
  planner_config.bindWorldGeometry(world->geometry());
  auto context = makeContext();
  navigation_planning_backend::CorridorGenerator generator(
      context, world, planner_config.corridor_bound_distance_m,
      planner_config.corridor_segment_max_length_m, planner_config.resolution,
      planner_config.robot_r, planner_config.iris_iter_num,
      navigation_world_model::UnknownPolicy::kAllowUnknown);
  generator.SetLineNeighborList(planner_config.seed_line_neighbour);

  geometry_utils::PolytopeVec corridor;
  Vec3f shifted_start = Vec3f::Constant(9999.0F);
  search_ok = generator.SearchPolytopeOnPath(
      guide_path, corridor, shifted_start, planner_config.use_fov_cut);
  if (!search_ok || corridor.empty()) return corridor;

  const auto envelope = navigation_planning_backend::deriveGuideVerticalEnvelope(
      guide_path, world->geometry().inflated_resolution_m);
  vertical_envelope_ok =
      navigation_planning_backend::applyGuideVerticalEnvelope(corridor, envelope);
  return corridor;
}

std::shared_ptr<navigation_planner_context::PlannerRuntimeContext>
makeContext() {
  return std::make_shared<navigation_planner_context::PlannerRuntimeContext>(
      [] { return 0.0; });
}

bool productionSetupPass(
    const traj_opt::Config& config,
    const navigation_planner_context::PlannerRuntimeContext::Ptr& context,
    const navigation_math::StatePVAJ& head,
    const navigation_math::StatePVAJ& tail,
    const vec_Vec3f& guide_path,
    const std::vector<double>& guide_times,
    const geometry_utils::PolytopeVec& input) {
  traj_opt::ExpTrajOpt optimizer(config, context);
  return optimizer.diagnosticSetupFromFrozenCorridor(
      head, tail, guide_path, guide_times, input);
}

int run(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node problem = root["problem"];
  if (!problem || !root["config"]) {
    throw std::runtime_error("snapshot is missing problem/config");
  }
  const auto head = readState(problem["head_pvaj"]);
  const auto tail = readState(problem["tail_pvaj"]);
  const auto guide_path = readPointSequence(problem["guide_path"]);
  const VecDf guide_stamp = readDoubleVector(problem["guide_stamp"]);
  const std::vector<double> guide_times(
      guide_stamp.data(), guide_stamp.data() + guide_stamp.size());
  const Mat3Df initial_points_matrix =
      readMatrix(problem["initial_spatial_variables"]);
  const VecDf serialized_initial_times =
      readDoubleVector(problem["initial_times"]);
  const VecDf serialized_initial_durations =
      readDoubleVector(problem["initial_durations_s"]);
  const bool initial_times_from_duration_fallback =
      serialized_initial_times.size() == 0 &&
      serialized_initial_durations.size() > 0;
  const VecDf initial_times = initial_times_from_duration_fallback
      ? serialized_initial_durations
      : serialized_initial_times;
  auto initial_points = readPointSequence(problem["initial_points"]);
  if (initial_points.empty() && initial_points_matrix.rows() == 3) {
    initial_points.reserve(
        static_cast<std::size_t>(initial_points_matrix.cols()));
    for (Eigen::Index column = 0;
         column < initial_points_matrix.cols(); ++column) {
      initial_points.emplace_back(initial_points_matrix.col(column));
    }
  }
  const VecDi h_poly_idx = readIntVector(problem["h_poly_idx"]);
  const PolyhedraH pre_simplify_h_polytopes =
      problem["pre_simplify_h_polytopes"]
          ? readPolyhedra(problem["pre_simplify_h_polytopes"])
          : PolyhedraH{};
  const auto pre_route_gates = problem["pre_simplify_route_boundary_gates"]
      ? problem["pre_simplify_route_boundary_gates"] : YAML::Node{};
  const auto pre_route_points = problem["pre_simplify_route_boundary_points"]
      ? problem["pre_simplify_route_boundary_points"] : YAML::Node{};
  const auto pre_route_radii = problem["pre_simplify_route_boundary_radii"]
      ? problem["pre_simplify_route_boundary_radii"] : YAML::Node{};
  const PolyhedraH h_polytopes = readPolyhedra(problem["h_polytopes"]);
  const auto route_gates = problem["route_boundary_gates"];
  const auto route_points = problem["route_boundary_points"];
  const auto route_radii = problem["route_boundary_radii"];
  const auto config_replay = makeConfig(root["config"]);
  auto config = config_replay.config;
  const auto context = makeContext();
  const auto polytope_vec = makePolytopes(
      h_polytopes, route_gates, route_points, route_radii);
  const auto world_snapshot = readWorldSnapshot(
      problem["diagnostic_world_snapshot"]);

  std::cout << std::setprecision(17)
            << "snapshot_kind=" << root["snapshot_kind"].as<std::string>()
            << " target_failure_signature="
            << root["target_failure_signature"].as<bool>()
            << " source_revision="
            << root["provenance"]["source_revision"].as<std::string>()
            << " solve_generation="
            << (root["provenance"]["solve_generation"]
                    ? root["provenance"]["solve_generation"].as<std::uint64_t>()
                    : 0U)
            << " planner_cycle="
            << (root["provenance"]["planner_cycle"]
                    ? root["provenance"]["planner_cycle"].as<std::uint64_t>()
                    : 0U)
            << " world_available=" << (world_snapshot != nullptr) << '\n'
            << "config_exact=" << config_replay.exact
            << " config_missing=";
  for (std::size_t index = 0; index < config_replay.missing.size(); ++index) {
    if (index != 0U) std::cout << ',';
    std::cout << config_replay.missing[index];
  }
  std::cout << '\n'
            << "head=\n" << head << "tail=\n" << tail
            << "initial_times=" << initial_times.transpose()
            << " initial_times_from_duration_fallback="
            << initial_times_from_duration_fallback << '\n'
            << "h_poly_idx=" << h_poly_idx.transpose() << '\n'
            << "pre_simplify_h_count=" << pre_simplify_h_polytopes.size()
            << " post_simplify_h_count=" << h_polytopes.size() << '\n';
  printStateColumns("head", head);
  printStateColumns("tail", tail);
  std::cout << "head_serialized_vs_replay="
            << matrixNodeMatches(problem["head_pvaj"], head) << '\n'
            << "tail_serialized_vs_replay="
            << matrixNodeMatches(problem["tail_pvaj"], tail) << '\n';

  // Current snapshots retain the exact PRE chain even when setup succeeds.
  // Replay both views through the production setup seam so a successful
  // request can also prove that the stored pre/post pair is coherent.  If
  // route metadata was overwritten by the legacy post-only fields, do not
  // invent it for PRE; report that limitation explicitly in the output.
  if (!pre_simplify_h_polytopes.empty()) {
    const bool pre_route_metadata_aligned =
        pre_route_gates && pre_route_points && pre_route_radii &&
        pre_route_gates.size() == pre_simplify_h_polytopes.size() &&
        pre_route_points.size() == pre_simplify_h_polytopes.size() &&
        pre_route_radii.size() == pre_simplify_h_polytopes.size();
    const YAML::Node empty_metadata;
    const auto pre_polytope_vec = makePolytopes(
        pre_simplify_h_polytopes,
        pre_route_metadata_aligned ? pre_route_gates : empty_metadata,
        pre_route_metadata_aligned ? pre_route_points : empty_metadata,
        pre_route_metadata_aligned ? pre_route_radii : empty_metadata);
    std::cout << "direct_pre_route_metadata_aligned="
              << pre_route_metadata_aligned << '\n'
              << "direct_pre_production_setup="
              << productionSetupPass(
                     config, context, head, tail, guide_path, guide_times,
                     pre_polytope_vec) << '\n'
              << "direct_post_production_setup="
              << productionSetupPass(
                     config, context, head, tail, guide_path, guide_times,
                     polytope_vec) << '\n';
  }

  // A setup-rejected snapshot has no initialized MINCO time/spatial state.
  // Keep replay total and analyze the exact production setup seam instead of
  // passing empty vectors into MINCO.  This is diagnostic-only: it does not
  // repair or reinterpret a degenerate corridor.
  const bool setup_completed = root["timing"]["setup_completed"].as<bool>();
  if (!setup_completed) {
    std::cout << "setup_rejected failure_stage="
              << root["timing"]["setup_failure_stage"].as<int>()
              << " failure_index="
              << root["timing"]["setup_failure_index"].as<int>()
              << " failure_metric="
              << scalarOrNaN(root["timing"]["setup_failure_metric"]) << '\n';
    const auto printOverlaps = [](const char* label,
                                  const PolyhedraH& polytopes) {
      std::cout << label << "_overlaps count="
                << (polytopes.size() > 0U ? polytopes.size() - 1U : 0U)
                << '\n';
      for (std::size_t index = 0; index + 1U < polytopes.size(); ++index) {
        MatD4f overlap(
            polytopes[index].rows() + polytopes[index + 1U].rows(), 4);
        overlap.topRows(polytopes[index].rows()) = polytopes[index];
        overlap.bottomRows(polytopes[index + 1U].rows()) =
            polytopes[index + 1U];
        Vec3f interior;
        const double interior_radius_m =
            geometry_utils::findInteriorDist(overlap, interior);
        Vec3f strict_interior;
        const bool strict_interior_found =
            geometry_utils::findInterior(overlap, strict_interior);
        PolyhedronV vertices;
        geometry_utils::enumerateVs(overlap, interior, vertices);
        Eigen::FullPivLU<Eigen::MatrixXd> rank_check(
            overlap.leftCols(3).cast<double>());
        std::cout << label << " overlap index=" << index
                  << " interior_radius_m=" << interior_radius_m
                  << " interior=" << interior.transpose()
                  << " find_interior=" << strict_interior_found
                  << " vertex_enumeration_ok=" << (vertices.cols() > 0)
                  << " vertex_count=" << vertices.cols()
                  << " plane_rows=" << overlap.rows()
                  << " normal_rank=" << rank_check.rank() << '\n';
        for (Eigen::Index row = 0; row < overlap.rows(); ++row) {
          std::cout << "  " << label << " plane[" << row << "]="
                    << overlap.row(row) << '\n';
        }
      }
    };
    const auto printSeamConditioning = [](const PolyhedraH& polytopes) {
      if (polytopes.size() < 2U) return;
      MatD4f seam(
          polytopes[0].rows() + polytopes[1].rows(), 4);
      seam.topRows(polytopes[0].rows()) = polytopes[0];
      seam.bottomRows(polytopes[1].rows()) = polytopes[1];
      const auto report = [](const char* label, const MatD4f& planes) {
        Vec3f interior;
        const double radius = geometry_utils::findInteriorDist(planes, interior);
        Vec3f strict_interior;
        const bool strict = geometry_utils::findInterior(planes, strict_interior);
        PolyhedronV vertices;
        geometry_utils::enumerateVs(planes, interior, vertices);
        std::cout << "seam_conditioning " << label
                  << " radius_m=" << radius
                  << " find_interior=" << strict
                  << " vertex_count=" << vertices.cols()
                  << " interior=" << interior.transpose() << '\n';
      };
      report("original", seam);

      Vec3f seam_interior;
      (void)geometry_utils::findInteriorDist(seam, seam_interior);
      const Vec3f origin = seam_interior;
      MatD4f translated = seam;
      translated.col(3) += translated.leftCols(3) * origin;
      report("translated_origin_lp_interior", translated);

      int first_opposing_plane = -1;
      int second_opposing_plane = -1;
      double best_active_slack = std::numeric_limits<double>::infinity();
      for (Eigen::Index first = 0; first < polytopes[0].rows(); ++first) {
        const Eigen::Vector3d first_normal =
            polytopes[0].row(first).head<3>().cast<double>();
        const double first_norm = first_normal.norm();
        if (!std::isfinite(first_norm) || first_norm <= 0.0) continue;
        for (Eigen::Index second = 0; second < polytopes[1].rows(); ++second) {
          const Eigen::Vector3d second_normal =
              polytopes[1].row(second).head<3>().cast<double>();
          const double second_norm = second_normal.norm();
          if (!std::isfinite(second_norm) || second_norm <= 0.0) continue;
          const double opposing_error =
              (first_normal / first_norm + second_normal / second_norm).norm();
          if (opposing_error > 1.0e-12) continue;
          const double first_slack =
              std::abs(first_normal.dot(seam_interior.cast<double>()) +
                       polytopes[0](first, 3));
          const double second_slack =
              std::abs(second_normal.dot(seam_interior.cast<double>()) +
                       polytopes[1](second, 3));
          const double active_slack = first_slack + second_slack;
          if (active_slack < best_active_slack) {
            best_active_slack = active_slack;
            first_opposing_plane = static_cast<int>(first);
            second_opposing_plane = static_cast<int>(
                polytopes[0].rows() + second);
          }
        }
      }
      std::cout << "seam_conditioning opposing_planes="
                << first_opposing_plane << "," << second_opposing_plane
                << " active_slack=" << best_active_slack << '\n';
      if (first_opposing_plane < 0 || second_opposing_plane < 0) return;

      const double first_offset = seam(first_opposing_plane, 3);
      const double second_offset = seam(second_opposing_plane, 3);
      const double first_ulp = std::nextafter(
          first_offset, std::numeric_limits<double>::infinity()) - first_offset;
      const double second_ulp = std::nextafter(
          second_offset, std::numeric_limits<double>::infinity()) - second_offset;
      for (const int upper_sign : {-1, 0, 1}) {
        for (const int lower_sign : {-1, 0, 1}) {
          MatD4f perturbed = seam;
          // Move only the closest opposing plane pair by one representable
          // offset step. This is diagnostic conditioning analysis; it never
          // changes production corridor data.
          perturbed(first_opposing_plane, 3) =
              first_offset + static_cast<double>(upper_sign) * first_ulp;
          perturbed(second_opposing_plane, 3) =
              second_offset + static_cast<double>(lower_sign) * second_ulp;
          const std::string label =
              "ulp_upper_" + std::to_string(upper_sign) +
              "_lower_" + std::to_string(lower_sign);
          report(label.c_str(), perturbed);
        }
      }
    };
    const auto printChain = [](const char* label,
                               const PolyhedraH& polytopes) {
      std::cout << label << "_chain count=" << polytopes.size() << '\n';
      for (std::size_t index = 0; index < polytopes.size(); ++index) {
        std::cout << label << " corridor index=" << index
                  << " plane_rows=" << polytopes[index].rows() << '\n';
        for (Eigen::Index row = 0; row < polytopes[index].rows(); ++row) {
          std::cout << "  " << label << "[" << index << "][" << row
                    << "]=" << polytopes[index].row(row) << '\n';
        }
      }
    };
    printOverlaps("pre_simplify", pre_simplify_h_polytopes.empty()
                                      ? h_polytopes
                                      : pre_simplify_h_polytopes);
    printOverlaps("post_simplify", h_polytopes);
    printSeamConditioning(h_polytopes);

    if (pre_simplify_h_polytopes.empty() && world_snapshot != nullptr) {
      bool search_ok = false;
      bool vertical_envelope_ok = false;
      const auto regenerated = reconstructPreSimplifyCorridor(
          guide_path, world_snapshot, search_ok, vertical_envelope_ok);
      PolyhedraH regenerated_h;
      regenerated_h.reserve(regenerated.size());
      for (const auto& polytope : regenerated) {
        regenerated_h.push_back(polytope.GetPlanes());
      }
      std::cout << "reconstructed_pre search_ok=" << search_ok
                << " vertical_envelope_ok=" << vertical_envelope_ok << '\n';
      printChain("reconstructed_pre", regenerated_h);

      auto regenerated_post = regenerated;
      const bool simplify_ok = !regenerated_post.empty() &&
          geometry_utils::SimplifySFC(
              head.col(0), tail.col(0), regenerated_post);
      PolyhedraH regenerated_post_h;
      regenerated_post_h.reserve(regenerated_post.size());
      for (const auto& polytope : regenerated_post) {
        regenerated_post_h.push_back(polytope.GetPlanes());
      }
      std::cout << "reconstructed_post simplify_ok=" << simplify_ok << '\n';
      printChain("reconstructed_post", regenerated_post_h);
      printOverlaps("reconstructed_pre", regenerated_h);
      printOverlaps("reconstructed_post", regenerated_post_h);
      std::cout << "reconstructed_pre_production_setup="
                << productionSetupPass(
                    config, context, head, tail, guide_path, guide_times,
                    regenerated) << '\n'
                << "reconstructed_post_production_setup="
                << productionSetupPass(
                    config, context, head, tail, guide_path, guide_times,
                    regenerated_post) << '\n';
      printSeamConditioning(regenerated_post_h);
    }
    return 0;
  }

  // A: current corridor-contained deterministic Bezier seed.
  const auto initial_bezier =
      navigation_planning_backend::buildCorridorContainedBezierSeed(
          head, tail, initial_points_matrix, initial_times, h_polytopes,
          h_poly_idx, config.max_vel * config.optimization_dynamic_reserve_ratio,
          config.corridor_plane_tolerance_m);
  printReport("A_bezier", certify(
      initial_bezier.trajectory, head, tail, h_polytopes, h_poly_idx,
      polytope_vec, config, world_snapshot.get()), config_replay.exact);
  std::cout << "A_seed_valid=" << initial_bezier.valid
            << " failure_stage=" << static_cast<int>(initial_bezier.failure_stage)
            << " failure_piece=" << initial_bezier.failing_piece_index
            << " failure_control=" << initial_bezier.failing_control_index
            << " failure_plane=" << initial_bezier.failing_plane_index
            << " violation=" << initial_bezier.maximum_plane_violation_m << '\n';

  // B: exactly the duration retry family captured by the production solve.
  const YAML::Node retries = root["duration_retries"];
  if (retries && retries.IsSequence()) {
    for (std::size_t index = 0; index < retries.size(); ++index) {
      const VecDf times = readDoubleVector(retries[index]["duration_s"]);
      const auto seed =
          navigation_planning_backend::buildCorridorContainedBezierSeed(
              head, tail, initial_points_matrix, times, h_polytopes,
              h_poly_idx, config.max_vel * config.optimization_dynamic_reserve_ratio,
              config.corridor_plane_tolerance_m);
      printReport("B_bezier_retry_" + std::to_string(index), certify(
          seed.trajectory, head, tail, h_polytopes, h_poly_idx, polytope_vec,
          config, world_snapshot.get()), config_replay.exact);
      std::cout << "B_retry_" << index << "_captured_build_valid="
                << retries[index]["build_valid"].as<bool>()
                << " replay_build_valid=" << seed.valid
                << " failure_piece=" << seed.failing_piece_index
                << " failure_control=" << seed.failing_control_index
                << " failure_plane=" << seed.failing_plane_index
                << " violation=" << seed.maximum_plane_violation_m << '\n';
    }
  }

  // C: immutable MINCO interpolation from the exact captured initial state.
  geometry_utils::Trajectory immutable_minco;
  traj_opt::MINCO_S4NU minco;
  minco.setConditions(head, tail, static_cast<int>(initial_times.size()));
  minco.setParameters(initial_points_matrix, initial_times);
  minco.getTrajectory(immutable_minco);
  printReport("C_immutable_minco", certify(
      immutable_minco, head, tail, h_polytopes, h_poly_idx, polytope_vec,
      config, world_snapshot.get()), config_replay.exact);

  // D: production MINCO/L-BFGS with the deadline disabled. This is a
  // diagnostic replay only; all physical certificates remain unchanged.
  {
    geometry_utils::Trajectory trajectory;
    auto sfcs = polytope_vec;
    traj_opt::ExpTrajOpt optimizer(config, context);
    optimizer.setSolveBudget(nullptr, 0, 0);
    const bool success = optimizer.optimize(
        head, tail, guide_path, guide_times,
        sfcs, trajectory, false, false);
    std::cout << "D_production_lbfgs success=" << success
              << " lbfgs_attempts=" << optimizer.diagnostics().lbfgs_attempt_count
              << " evaluations=" << optimizer.diagnostics().lbfgs_evaluation_count
              << " first_return=" << optimizer.diagnostics().first_lbfgs_return_code
              << " last_return=" << optimizer.diagnostics().last_lbfgs_return_code
              << " retry_count=" << optimizer.diagnostics().retry_count << '\n';
    printReport("D_production_lbfgs_candidate", certify(
        trajectory, head, tail, h_polytopes, h_poly_idx, polytope_vec,
        config, world_snapshot.get()), config_replay.exact);
  }

  // D-budget: replay the same production optimizer under the development
  // 10 Hz timing contract.  This is diagnostic only; the deadline is not
  // relaxed and no result is used as qualification evidence.
  for (const auto [refinement_ms, hard_ms] :
       std::array<std::pair<std::int64_t, std::int64_t>, 2>{
           std::pair<std::int64_t, std::int64_t>{40, 80},
           std::pair<std::int64_t, std::int64_t>{0, 80}}) {
    geometry_utils::Trajectory trajectory;
    auto sfcs = polytope_vec;
    traj_opt::ExpTrajOpt optimizer(config, context);
    const auto start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    optimizer.setSolveBudget(
        nullptr, start_ns + refinement_ms * 1000000,
        start_ns + hard_ms * 1000000);
    const bool success = optimizer.optimize(
        head, tail, guide_path, guide_times,
        sfcs, trajectory, false, false);
    std::cout << "D_budget_refinement_ms=" << refinement_ms
              << " hard_ms=" << hard_ms
              << " success=" << success
              << " hard_deadline_observed="
              << optimizer.diagnostics().hard_deadline_observed
              << " refinement_budget_entry_us="
              << optimizer.diagnostics().refinement_budget_at_entry_us
              << " lbfgs_attempts="
              << optimizer.diagnostics().lbfgs_attempt_count
              << " evaluations="
              << optimizer.diagnostics().lbfgs_evaluation_count
              << " first_return="
              << optimizer.diagnostics().first_lbfgs_return_code
              << " last_return="
              << optimizer.diagnostics().last_lbfgs_return_code
              << " retry_count=" << optimizer.diagnostics().retry_count
              << '\n';
    printReport("D_budget_candidate", certify(
        trajectory, head, tail, h_polytopes, h_poly_idx, polytope_vec,
        config, world_snapshot.get()), config_replay.exact);
  }

  // Recovery-mode replay: PlanFromRest calls solve(..., baseline_only=true,
  // suppress_optional_refinement=false).  Keep this separate from the
  // generic D probe above so the offline result matches that production
  // entrypoint rather than silently testing a different mode.
  for (const auto [label, refinement_ms, hard_ms] :
       std::array<std::tuple<const char*, std::int64_t, std::int64_t>, 3>{
           std::tuple<const char*, std::int64_t, std::int64_t>{
               "D_recovery_no_deadline", 0, 0},
           std::tuple<const char*, std::int64_t, std::int64_t>{
               "D_recovery_budget_40_80", 40, 80},
           std::tuple<const char*, std::int64_t, std::int64_t>{
               "D_recovery_budget_0_80", 0, 80}}) {
    geometry_utils::Trajectory trajectory;
    auto sfcs = polytope_vec;
    traj_opt::ExpTrajOpt optimizer(config, context);
    if (hard_ms > 0) {
      const auto start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      optimizer.setSolveBudget(
          nullptr, start_ns + refinement_ms * 1000000,
          start_ns + hard_ms * 1000000);
    } else {
      optimizer.setSolveBudget(nullptr, 0, 0);
    }
    const auto nominal_result = optimizer.solve(
        head, tail, guide_path, guide_times,
        sfcs, trajectory, false, true, false);
    std::cout << label
              << " status=" << static_cast<int>(nominal_result.status)
              << " deadline_observed=" << nominal_result.deadline_observed
              << " candidate_available=" << nominal_result.candidateAvailable()
              << " hard_deadline_observed="
              << optimizer.diagnostics().hard_deadline_observed
              << " refinement_budget_entry_us="
              << optimizer.diagnostics().refinement_budget_at_entry_us
              << " baseline_fallback="
              << optimizer.diagnostics().baseline_fallback_to_optimizer
              << " lbfgs_attempts="
              << optimizer.diagnostics().lbfgs_attempt_count
              << " evaluations="
              << optimizer.diagnostics().lbfgs_evaluation_count
              << " retry_count=" << optimizer.diagnostics().retry_count
              << '\n';
    printReport(label, certify(
        trajectory, head, tail, h_polytopes, h_poly_idx, polytope_vec,
        config, world_snapshot.get()), config_replay.exact);
  }

  // E: high-effort generic MINCO/L-BFGS. Multiple deterministic time starts
  // are allowed here only as a feasibility probe. No physical limit changes.
  config.feasibility_retry_max_iterations =
      traj_opt::Config::kMaximumFeasibilityRetryIterations;
  const std::array<double, 4> effort_scales{0.5, 1.0, 2.0, 4.0};
  for (const double scale : effort_scales) {
    geometry_utils::Trajectory trajectory;
    auto sfcs = polytope_vec;
    const VecDf times = initial_times * scale;
    traj_opt::ExpTrajOpt optimizer(config, context);
    optimizer.setSolveBudget(nullptr, 0, 0);
    const bool success = optimizer.optimize(
        head, tail, sfcs, initial_points, times, trajectory);
    std::cout << "E_high_effort_scale=" << scale
              << " success=" << success
              << " lbfgs_attempts=" << optimizer.diagnostics().lbfgs_attempt_count
              << " evaluations=" << optimizer.diagnostics().lbfgs_evaluation_count
              << " first_return=" << optimizer.diagnostics().first_lbfgs_return_code
              << " last_return=" << optimizer.diagnostics().last_lbfgs_return_code
              << " retry_count=" << optimizer.diagnostics().retry_count << '\n';
    printReport("E_high_effort_candidate", certify(
        trajectory, head, tail, h_polytopes, h_poly_idx, polytope_vec,
        config, world_snapshot.get()), config_replay.exact);
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: replay_nominal_problem_snapshot SNAPSHOT.json\n";
    return 2;
  }
  try {
    return run(argv[1]);
  } catch (const std::exception& exception) {
    std::cerr << "replay error: " << exception.what() << '\n';
    return 1;
  }
}
