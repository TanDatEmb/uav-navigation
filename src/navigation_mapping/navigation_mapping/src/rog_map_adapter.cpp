#include "navigation_mapping/rog_map_adapter.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace navigation_mapping {
namespace {

std::string yamlBool(bool value) { return value ? "true" : "false"; }

// Translates the coherent product configuration surface (section 20) into
// the vendored ROG-Map's own upstream YAML schema (section 20: "use actual
// upstream ROG parameter names internally"). Kept as a free function instead
// of exposing every upstream parameter as a product parameter.
std::string toUpstreamYaml(const RogMapProductConfig& config) {
  const int effective_inflation_step =
      config.occupied_inflation_enabled ? std::max(1, config.inflation_step) : 0;
  std::ostringstream out;
  out << "rog_map:\n"
      << "  resolution: " << config.resolution_m << "\n"
      << "  inflation_resolution: " << config.resolution_m << "\n"
      << "  inflation_step: " << effective_inflation_step << "\n"
      << "  unk_inflation_en: " << yamlBool(config.unknown_inflation_enabled) << "\n"
      << "  unk_inflation_step: " << std::max(1, config.inflation_step) << "\n"
      << "  intensity_thresh: -1\n"
      << "  point_filt_num: " << config.point_filt_num << "\n"
      << "  map_size: [" << config.local_map_size_m[0] << ", " << config.local_map_size_m[1]
      << ", " << config.local_map_size_m[2] << "]\n"
      << "  fix_map_origin: [0.0, 0.0, 0.0]\n"
      << "  virtual_ground_height: " << config.virtual_ground_height_m << "\n"
      << "  virtual_ceil_height: " << config.virtual_ceil_height_m << "\n"
      << "  map_sliding:\n"
      << "    enable: " << yamlBool(config.sliding_enabled) << "\n"
      << "    threshold: -1.0\n"
      << "  load_pcd_en: false\n"
      << "  frontier_extraction_en: " << yamlBool(config.frontier_enabled) << "\n"
      << "  ros_callback:\n"
      << "    enable: false\n"
      << "    cloud_topic: \"/unused\"\n"
      << "    odom_topic: \"/unused\"\n"
      << "    odom_timeout: 0.05\n"
      << "  visualization:\n"
      << "    enable: false\n"
      << "    use_dynamic_reconfigure: false\n"
      << "    pub_unknown_map_en: false\n"
      << "    frame_id: \"lio_odom\"\n"
      << "    time_rate: 0.0\n"
      << "    frame_rate: 0\n"
      << "    range: [0.0, 0.0, 0.0]\n"
      << "  esdf:\n"
      << "    enable: " << yamlBool(config.esdf_enabled) << "\n"
      << "    resolution: " << config.resolution_m << "\n"
      << "    local_update_box: [" << config.local_map_size_m[0] << ", "
      << config.local_map_size_m[1] << ", " << config.local_map_size_m[2] << "]\n"
      << "  raycasting:\n"
      << "    enable: " << yamlBool(config.raycasting_enabled) << "\n"
      << "    batch_update_size: 1\n"
      << "    unk_thresh: 0.70\n"
      << "    p_hit: 0.70\n"
      << "    p_miss: 0.35\n"
      << "    p_min: 0.12\n"
      << "    p_max: 0.97\n"
      << "    p_occ: 0.80\n"
      << "    p_free: 0.30\n"
      << "    ray_range: [" << config.ray_range_min_m << ", " << config.ray_range_max_m << "]\n"
      << "    local_update_box: [" << config.local_map_size_m[0] << ", "
      << config.local_map_size_m[1] << ", " << config.local_map_size_m[2] << "]\n";
  return out.str();
}

}  // namespace

ConcreteRogMap::ConcreteRogMap(std::function<double()> wall_clock_seconds)
    : wall_clock_seconds_(std::move(wall_clock_seconds)) {}

const double ConcreteRogMap::getSystemWalltimeNow() { return wall_clock_seconds_(); }

RogMapAdapter::RogMapAdapter(std::function<double()> wall_clock_seconds,
                            std::string generated_config_directory)
    : wall_clock_seconds_(std::move(wall_clock_seconds)),
      generated_config_directory_(std::move(generated_config_directory)) {}

void RogMapAdapter::reset(const RogMapProductConfig& config) {
  // Destroying the previous instance (if any) before constructing a new one
  // exercises the P1 lifecycle patch: rog_map_vendor's per-instance init
  // guard (see rog_map_vendor/UPSTREAM.md) is required for this to succeed
  // more than once in the same process.
  map_.reset();

  const std::string config_path = generated_config_directory_ + "/rog_map_generated.yaml";
  {
    std::ofstream file(config_path, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
      throw std::runtime_error("RogMapAdapter: failed to write generated config to " +
                               config_path);
    }
    file << toUpstreamYaml(config);
  }

  auto new_map = std::make_unique<ConcreteRogMap>(wall_clock_seconds_);
  new_map->loadConfigAndInit(config_path);
  map_ = std::move(new_map);
  ++reset_count_;
}

void RogMapAdapter::updateMap(const rog_map::PointCloud& cloud_odom_m,
                              const T_odom_lidar& sensor_pose) {
  if (!map_) {
    throw std::runtime_error("RogMapAdapter::updateMap called before reset()");
  }
  const rog_map::Pose pose(sensor_pose.translation_odom_m, sensor_pose.rotation_odom_lidar);
  map_->updateMap(cloud_odom_m, pose);
}

rog_map::ROGMap& RogMapAdapter::map() {
  if (!map_) {
    throw std::runtime_error("RogMapAdapter::map() called before reset()");
  }
  return *map_;
}

const rog_map::ROGMap& RogMapAdapter::map() const {
  if (!map_) {
    throw std::runtime_error("RogMapAdapter::map() called before reset()");
  }
  return *map_;
}

const rog_map::ProbMap::RaycastDiagnostics& RogMapAdapter::lastDiagnostics() const {
  if (!map_) {
    throw std::runtime_error("RogMapAdapter::lastDiagnostics() called before reset()");
  }
  return map_->lastDiagnostics();
}

std::uint64_t RogMapAdapter::deterministicDigest() const {
  if (!map_) {
    throw std::runtime_error("RogMapAdapter::deterministicDigest() called before reset()");
  }
  return map_->deterministicDigest();
}

}  // namespace navigation_mapping
