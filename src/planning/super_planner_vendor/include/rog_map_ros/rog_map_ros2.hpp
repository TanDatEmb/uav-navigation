#pragma once

#include <functional>
#include <memory>
#include <string>

#include <rog_map/rog_map.h>

namespace rog_map {

// Compatibility shell for the upstream SUPER core. ROS subscription and
// conversion work is owned by navigation_runtime; this class only supplies
// the wall-clock hook and preserves the upstream pointer type.
class ROGMapROS : public ROGMap {
 public:
  using Ptr = std::shared_ptr<ROGMapROS>;

  explicit ROGMapROS(std::function<double()> wall_clock_seconds = {})
      : wall_clock_seconds_(std::move(wall_clock_seconds)) {}

  const double getSystemWalltimeNow() override {
    return wall_clock_seconds_ ? wall_clock_seconds_() : 0.0;
  }

  void loadConfigAndInit(const std::string& yaml_path) {
    cfg_ = rog_map::Config(yaml_path);
    init();
  }

 private:
  std::function<double()> wall_clock_seconds_;
};

}  // namespace rog_map
