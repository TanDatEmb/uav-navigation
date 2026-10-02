#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "nav_safety_profile/generated/profile.hpp"
#include "nav_safety_profile/witness.hpp"

namespace {
std::filesystem::path writeTemp(const YAML::Node& document, const char* name) {
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream stream(path);
  stream << document;
  return path;
}

TEST(SafetyProfile, LoadsTypedFieldsAndStableHash) {
  const auto first = nav_safety_profile::load(SAFETY_PROFILE_SOURCE);
  const auto second = nav_safety_profile::load(SAFETY_PROFILE_SOURCE);
  ASSERT_TRUE(first) << first.error;
  ASSERT_TRUE(second) << second.error;
  EXPECT_EQ(first.value->sha256, second.value->sha256);
  EXPECT_EQ(first.value->hash64,
            std::stoull(first.value->sha256.substr(0, 16), nullptr, 16));
  EXPECT_EQ(first.value->typed.timing_planner_period_ns, 100000000);
  EXPECT_EQ(first.value->typed.geometry_planning_radius_sum_m, 0.8);
}

TEST(SafetyProfile, MissingRequiredKeyFails) {
  auto document = YAML::LoadFile(SAFETY_PROFILE_SOURCE);
  document["timing"].remove("planner_period_s");
  const auto path = writeTemp(document, "nav_safety_profile_missing.yaml");
  const auto result = nav_safety_profile::load(path.string());
  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("planner_period_s"), std::string::npos);
  std::filesystem::remove(path);
}

TEST(SafetyProfile, InvalidDomainFails) {
  auto document = YAML::LoadFile(SAFETY_PROFILE_SOURCE);
  document["geometry"]["vehicle_radius_m"]["value"] = -0.1;
  const auto path = writeTemp(document, "nav_safety_profile_invalid.yaml");
  const auto result = nav_safety_profile::load(path.string());
  EXPECT_FALSE(result);
  EXPECT_NE(result.error.find("vehicle_radius_m"), std::string::npos);
  std::filesystem::remove(path);
}

TEST(SafetyProfile, WitnessComparesOnlyOwnedEffectiveValues) {
  nav_safety_profile::Profile profile;
  profile.hash64 = 0x1234;
  profile.entries.emplace("timing.solve_deadline_s",
      nav_safety_profile::Entry{0.08, "s", "PROVISIONAL", {"runtime"}});
  profile.entries.emplace("geometry.vehicle_radius_m",
      nav_safety_profile::Entry{0.25, "m", "PROVISIONAL", {"judge"}});
  const auto line = nav_safety_profile::witness_line(
      profile, "runtime", {{"timing.solve_deadline_s", 0.1}});
  EXPECT_EQ(line,
      "SAFETY_PROFILE_WITNESS hash=0000000000001234 keys=1 "
      "mismatches=[timing.solve_deadline_s=0.10000000000000001/0.080000000000000002]");
}

TEST(SafetyProfile, MissingOwnedEffectiveValueIsRecorded) {
  nav_safety_profile::Profile profile;
  profile.entries.emplace("timing.solve_deadline_s",
      nav_safety_profile::Entry{0.08, "s", "PROVISIONAL", {"runtime"}});
  const auto line = nav_safety_profile::witness_line(profile, "runtime", {});
  EXPECT_EQ(line,
      "SAFETY_PROFILE_WITNESS hash=0000000000000000 keys=1 "
      "mismatches=[timing.solve_deadline_s=<unavailable>/0.080000000000000002]");
}
}  // namespace
