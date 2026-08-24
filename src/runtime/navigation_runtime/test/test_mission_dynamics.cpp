#include "navigation_runtime/mission_dynamics.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

namespace {

class TemporaryMission {
 public:
  explicit TemporaryMission(const std::string& contents) {
    path_ = std::filesystem::temp_directory_path() /
            ("super_mission_" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
             ".yaml");
    std::ofstream stream(path_);
    stream << contents;
  }
  ~TemporaryMission() { std::error_code ignored; std::filesystem::remove(path_, ignored); }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

}  // namespace

TEST(MissionDynamics, LoadsPlannerLimitsBeforeSuperConstruction) {
  const TemporaryMission mission(R"(
mission:
  planning:
    max_velocity_mps: 7.0
    max_acceleration_mps2: 5.0
    max_jerk_mps3: 12.0
)");
  const auto limits = navigation_runtime::loadMissionDynamicLimits(mission.path());
  EXPECT_DOUBLE_EQ(limits.max_velocity_mps, 7.0);
  EXPECT_DOUBLE_EQ(limits.max_acceleration_mps2, 5.0);
  EXPECT_DOUBLE_EQ(limits.max_jerk_mps3, 12.0);
}

TEST(MissionDynamics, RejectsNonPositiveLimits) {
  const TemporaryMission mission(R"(
mission:
  planning:
    max_velocity_mps: 0.0
)");
  EXPECT_THROW(navigation_runtime::loadMissionDynamicLimits(mission.path()),
               std::invalid_argument);
}
