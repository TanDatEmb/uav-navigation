#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <fmt/format.h>

#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>
#include <utils/header/color_msg_utils.hpp>
#include <utils/header/eigen_alias.hpp>
#include <utils/header/fmt_eigen.hpp>

namespace ros_interface {

class RosInterface {
 public:
  using Ptr = std::shared_ptr<RosInterface>;
  explicit RosInterface(std::function<double()> clock_seconds = {})
      : clock_seconds_(std::move(clock_seconds)) {}
  virtual ~RosInterface() = default;

  template <typename... Args>
  void debug(const char* format, Args&&... args) { log("DEBUG", format, std::forward<Args>(args)...); }
  template <typename... Args>
  void info(const char* format, Args&&... args) { log("INFO", format, std::forward<Args>(args)...); }
  template <typename... Args>
  void warn(const char* format, Args&&... args) { log("WARN", format, std::forward<Args>(args)...); }
  template <typename... Args>
  void error(const char* format, Args&&... args) { log("ERROR", format, std::forward<Args>(args)...); }
  template <typename... Args>
  void fatal(const char* format, Args&&... args) { log("FATAL", format, std::forward<Args>(args)...); }

  void debug(const std::string& message) const { logText("DEBUG", message); }
  void info(const std::string& message) const { logText("INFO", message); }
  void warn(const std::string& message) const { logText("WARN", message); }
  void error(const std::string& message) const { logText("ERROR", message); }
  void fatal(const std::string& message) const { logText("FATAL", message); }

  [[nodiscard]] virtual double getSimTime() const {
    return clock_seconds_
               ? clock_seconds_()
               : std::chrono::duration<double>(
                     std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void getSimTime(std::int32_t& seconds, std::uint32_t& nanoseconds) const {
    const double now = getSimTime();
    seconds = static_cast<std::int32_t>(now);
    nanoseconds = static_cast<std::uint32_t>((now - seconds) * 1e9);
  }

  void setResolution(double resolution) { resolution_ = resolution; }
  void setVisualizationEn(bool enabled) { visualization_en_ = enabled; }

  void vizExpTraj(const geometry_utils::Trajectory&, const std::string& = "exp_traj") {}
  void vizBackupTraj(const geometry_utils::Trajectory&) {}
  void vizFrontendPath(const navigation_math::vec_Vec3f&) {}
  void vizExpSfc(const geometry_utils::PolytopeVec&) {}
  void vizBackupSfc(const geometry_utils::Polytope&) {}
  void vizGoalPath(const navigation_math::vec_Vec3f&) {}
  void vizCommittedTraj(const geometry_utils::Trajectory&, double) {}
  void vizYawTraj(const geometry_utils::Trajectory&, const geometry_utils::Trajectory&) {}
  void vizAstarBoundingBox(const navigation_math::Vec3f&, const navigation_math::Vec3f&) {}
  void vizAstarPoints(const navigation_math::Vec3f&, const Color&, const std::string&, double = 0.1, int = 0) {}
  void vizReplanLog(const geometry_utils::Trajectory&, const geometry_utils::Trajectory&,
                    const geometry_utils::Trajectory&, const geometry_utils::Trajectory&,
                    const geometry_utils::PolytopeVec&, const geometry_utils::Polytope&,
                    const navigation_math::vec_Vec3f&, int) {}
  void vizCiriSeedLine(const navigation_math::Vec3f&, const navigation_math::Vec3f&, double) {}
  void vizCiriEllipsoid(const geometry_utils::Ellipsoid&) {}
  void vizCiriInfeasiblePoint(const navigation_math::Vec3f&) {}
  void vizCiriPolytope(const geometry_utils::Polytope&, const std::string&) {}
  void vizCiriPointCloud(const navigation_math::vec_Vec3f&) {}

 private:
  template <typename... Args>
  static void log(const char* level, const char* format, Args&&... args) {
    logText(level, fmt::format(fmt::runtime(format), std::forward<Args>(args)...));
  }
  static void logText(const char* level, const std::string& message) {
    fmt::print("[planner {}] {}\n", level, message);
  }

  std::function<double()> clock_seconds_;
  double resolution_{0.1};
  bool visualization_en_{false};
};

}  // namespace ros_interface
