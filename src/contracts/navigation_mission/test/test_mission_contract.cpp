#include <navigation_mission/mission.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

namespace {

class TemporaryMission {
 public:
  explicit TemporaryMission(const char* contents) {
    path_ = std::filesystem::temp_directory_path() /
            ("navigation_mission_" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
             ".yaml");
    std::ofstream stream(path_);
    stream << contents;
  }
  ~TemporaryMission() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }
  const std::string string() const { return path_.string(); }

 private:
  std::filesystem::path path_;
};

}  // namespace

TEST(MissionContract, LoadsOneValidatedProductSchema) {
  const TemporaryMission mission(R"(
mission:
  version: 1
  id: smoke
  frame: lio_odom
  waypoints:
    - id: finish
      position: [1.0, 2.0, 3.0]
      acceptance_radius_m: 0.5
      behavior: stop
  planning:
    max_velocity_mps: 7.0
    max_acceleration_mps2: 5.0
    max_jerk_mps3: 12.0
    unknown_policy: allow_unknown
)");
  const auto loaded = navigation_mission::loadMission(mission.string(), "lio_odom");
  EXPECT_EQ(loaded.id, "smoke");
  EXPECT_DOUBLE_EQ(loaded.planning.max_velocity_mps, 7.0);
  EXPECT_EQ(loaded.planning.unknown_policy,
            navigation_world_model::UnknownPolicy::kAllowUnknown);
}

TEST(MissionContract, RejectsFrameMismatchAndUnknownKeys) {
  const TemporaryMission mission(R"(
mission:
  version: 1
  id: smoke
  frame: map
  waypoints:
    - id: finish
      position: [1.0, 2.0, 3.0]
      acceptance_radius_m: 0.5
      unexpected: true
)");
  EXPECT_THROW(navigation_mission::loadMission(mission.string(), "lio_odom"),
               std::invalid_argument);
}

TEST(MissionContract, RejectsAValidMissionWithTheWrongPlanningFrame) {
  const TemporaryMission mission(R"(
mission:
  version: 1
  id: smoke
  frame: map
  waypoints:
    - id: finish
      position: [1.0, 2.0, 3.0]
      acceptance_radius_m: 0.5
      behavior: stop
)");
  EXPECT_THROW(navigation_mission::loadMission(mission.string(), "lio_odom"),
               std::invalid_argument);
}
