// Minimal concrete ROGMap for tests: only supplies the pure-virtual system
// walltime hook the ROS2 wrapper would normally provide via rclcpp::Clock.
#pragma once

#include <chrono>

#include <rog_map/rog_map_core/config.hpp>
#include <rog_map/rog_map.h>

namespace navigation_mapping::test {

class TestRogMap : public rog_map::ROGMap {
 public:
  const double getSystemWalltimeNow() override {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  // cfg_ is a protected ProbMap member; upstream's own ROS2 wrapper sets it
  // the same way (see rog_map_ros2.hpp) before calling init().
  void loadConfigAndInit(const std::string& yaml_path) {
    cfg_ = rog_map::Config(yaml_path);
    init();
  }
};

}  // namespace navigation_mapping::test
