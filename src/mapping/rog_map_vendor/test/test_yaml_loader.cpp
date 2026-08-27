#include <navigation_math/yaml_loader.hpp>
#include <rog_map/rog_map_core/config.hpp>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

namespace {

std::string configVariant(const std::string& replacement,
                         const std::string& name) {
  const std::string source_path =
      std::string(ROG_MAP_VENDOR_TEST_FIXTURE_DIR) + "/rog_map_test.yaml";
  std::ifstream input(source_path);
  std::string contents((std::istreambuf_iterator<char>(input)),
                       std::istreambuf_iterator<char>());
  const auto marker = replacement.substr(0, replacement.find(':') + 1);
  const auto position = contents.find(marker);
  if (position == std::string::npos) return {};
  const auto end = contents.find('\n', position);
  contents.replace(position, end == std::string::npos ? std::string::npos : end - position,
                   replacement);

  const std::string path = "/tmp/" + name;
  std::ofstream output(path);
  if (!output.good()) return {};
  output << contents;
  return path;
}

TEST(YamlLoader, ParsesDocumentOnceAndReadsNestedValues) {
  const std::string path = "/tmp/uav_navigation_yaml_loader_test.yaml";
  {
    std::ofstream config(path);
    ASSERT_TRUE(config.good());
    config << "planner:\n  limit: 0.25\n  enabled: true\n";
  }

  yaml_loader::YamlLoader loader(path);
  double limit = 0.0;
  bool enabled = false;
  EXPECT_TRUE(loader.LoadParam("planner/limit", limit, 0.0, true));
  EXPECT_TRUE(loader.LoadParam("planner/enabled", enabled, false, true));
  EXPECT_DOUBLE_EQ(limit, 0.25);
  EXPECT_TRUE(enabled);

  std::remove(path.c_str());
}

TEST(YamlLoader, RequiredMissingValueFailsClosed) {
  const std::string path = "/tmp/uav_navigation_yaml_loader_missing_test.yaml";
  {
    std::ofstream config(path);
    ASSERT_TRUE(config.good());
    config << "planner: {}\n";
  }

  yaml_loader::YamlLoader loader(path);
  int value = 3;
  EXPECT_THROW(loader.LoadParam("planner/missing", value, 7, true),
               std::invalid_argument);

  std::remove(path.c_str());
}

TEST(YamlLoader, RequiredTypeMismatchFailsClosed) {
  const std::string path = "/tmp/uav_navigation_yaml_loader_type_test.yaml";
  {
    std::ofstream config(path);
    ASSERT_TRUE(config.good());
    config << "planner:\n  limit: not-a-number\n";
  }

  yaml_loader::YamlLoader loader(path);
  double value = 3.0;
  EXPECT_THROW(loader.LoadParam("planner/limit", value, 7.0, true),
               std::invalid_argument);

  std::remove(path.c_str());
}

TEST(YamlLoader, OptionalTypeMismatchFailsClosed) {
  const std::string path = "/tmp/uav_navigation_yaml_loader_optional_type_test.yaml";
  {
    std::ofstream config(path);
    ASSERT_TRUE(config.good());
    config << "planner:\n  limit: not-a-number\n";
  }

  yaml_loader::YamlLoader loader(path);
  double value = 3.0;
  EXPECT_THROW(loader.LoadParam("planner/limit", value, 7.0, false),
               std::invalid_argument);

  std::remove(path.c_str());
}

TEST(YamlLoader, PresentNullFailsClosed) {
  const std::string path = "/tmp/uav_navigation_yaml_loader_null_test.yaml";
  {
    std::ofstream config(path);
    ASSERT_TRUE(config.good());
    config << "planner:\n  limit: null\n";
  }

  yaml_loader::YamlLoader loader(path);
  double value = 3.0;
  EXPECT_THROW(loader.LoadParam("planner/limit", value, 7.0, false),
               std::invalid_argument);

  std::remove(path.c_str());
}

TEST(YamlLoader, ScalarIntermediatePathFailsClosed) {
  const std::string path = "/tmp/uav_navigation_yaml_loader_scalar_test.yaml";
  {
    std::ofstream config(path);
    ASSERT_TRUE(config.good());
    config << "planner: scalar\n";
  }

  yaml_loader::YamlLoader loader(path);
  int value = 3;
  EXPECT_THROW(loader.LoadParam("planner/limit", value, 7, true),
               std::invalid_argument);

  std::remove(path.c_str());
}

TEST(YamlLoader, MappingSafetyConfigurationRejectsInvalidValues) {
  const auto negative_threshold = configVariant(
      "    threshold: -1.0", "uav_navigation_invalid_map_threshold.yaml");
  ASSERT_FALSE(negative_threshold.empty());
  EXPECT_THROW({ rog_map::Config config(negative_threshold); }, std::invalid_argument);
  std::remove(negative_threshold.c_str());

  const auto uninformative_miss_probability = configVariant(
      "    p_miss: 0.5", "uav_navigation_invalid_map_probability.yaml");
  ASSERT_FALSE(uninformative_miss_probability.empty());
  EXPECT_THROW({ rog_map::Config config(uninformative_miss_probability); },
               std::invalid_argument);
  std::remove(uninformative_miss_probability.c_str());

  const auto invalid_point_filter = configVariant(
      "  point_filt_num: 0", "uav_navigation_invalid_point_filter.yaml");
  ASSERT_FALSE(invalid_point_filter.empty());
  EXPECT_THROW({ rog_map::Config config(invalid_point_filter); }, std::invalid_argument);
  std::remove(invalid_point_filter.c_str());

  const auto invalid_batch_size = configVariant(
      "    batch_update_size: 0", "uav_navigation_invalid_batch_size.yaml");
  ASSERT_FALSE(invalid_batch_size.empty());
  EXPECT_THROW({ rog_map::Config config(invalid_batch_size); }, std::invalid_argument);
  std::remove(invalid_batch_size.c_str());
}

}  // namespace
