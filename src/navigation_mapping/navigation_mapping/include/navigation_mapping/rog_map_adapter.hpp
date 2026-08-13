#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rog_map/rog_map.h>

#include "navigation_mapping/rigid_pose.hpp"

namespace navigation_mapping {

// Product-level ROG configuration. Vendor policy is deliberately not exposed
// as a second public configuration surface; the adapter owns that policy.
struct RogMapProductConfig {
  double resolution_m{0.20};
  double inflation_resolution_m{0.20};
  std::array<double, 3> local_map_size_m{30.0, 30.0, 12.0};
  double ray_range_min_m{0.3};
  double ray_range_max_m{15.0};
  double virtual_ground_height_m{-1000.0};
  double virtual_ceil_height_m{1000.0};
  bool frontier_debug{false};
};

// Concrete instantiation of the abstract vendored ROGMap: only supplies the
// wall-clock hook (pure virtual upstream). Kept intentionally tiny; all
// product logic lives in RogMapAdapter, not here.
class ConcreteRogMap : public rog_map::ROGMap {
 public:
  explicit ConcreteRogMap(std::function<double()> wall_clock_seconds);
  const double getSystemWalltimeNow() override;

  // cfg_ is a protected ProbMap member (see rog_map_vendor); only a derived
  // class can set it directly. This mirrors upstream's own ROS2 wrapper
  // pattern (rog_map_ros2.hpp).
  void loadConfigAndInit(const std::string& yaml_path) {
    cfg_ = rog_map::Config(yaml_path);
    init();
  }

 private:
  std::function<double()> wall_clock_seconds_;
};

// Owns the vendored ROGMap instance's lifecycle and is the single
// product-owned entry point for map mutation. All
// reset/update/query calls must be made from a single serialized caller
// (e.g. one ROS callback group); this class performs no internal locking.
class RogMapAdapter {
 public:
  explicit RogMapAdapter(std::function<double()> wall_clock_seconds,
                         std::string generated_config_directory);

  static void validateProductConfig(const RogMapProductConfig& config);

  // Destroys any existing map instance and constructs+initializes a fresh
  // one. Exercises the P1 lifecycle patch in rog_map_vendor (see
  // rog_map_vendor/UPSTREAM.md); safe to call repeatedly in the same process.
  void reset(const RogMapProductConfig& config, std::uint64_t generation);

  [[nodiscard]] bool isInitialized() const noexcept { return static_cast<bool>(map_); }

  // cloud_odom_m must already be expressed in lio_odom, at the same epoch as
  // sensor_origin_odom.
  void updateMap(const rog_map::PointCloud& cloud_odom_m, const T_odom_lidar& sensor_pose);

  [[nodiscard]] rog_map::ROGMap& map();
  [[nodiscard]] const rog_map::ROGMap& map() const;
  [[nodiscard]] const rog_map::ProbMap::RaycastDiagnostics& lastDiagnostics() const;
  [[nodiscard]] std::uint64_t deterministicDigest() const;
  [[nodiscard]] std::size_t resetCount() const noexcept { return reset_count_; }
  [[nodiscard]] std::uint64_t generation() const noexcept { return generation_; }

 private:
  std::function<double()> wall_clock_seconds_;
  std::string generated_config_directory_;
  std::unique_ptr<ConcreteRogMap> map_;
  std::size_t reset_count_{0};
  std::uint64_t generation_{0};
};

}  // namespace navigation_mapping
