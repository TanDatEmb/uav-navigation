// The core target of uavnav_px4_bridge (uavnav_px4_bridge_core) stays ROS-free (SYSTEM_DESIGN §4.1, D33/O19; the
// arch check enforces the same by #include, this test also catches a use without an include and a linked library).
// Every header and source of the package except the ROS files named below is scanned, comments stripped.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace fs = std::filesystem;

// The files where ROS is allowed (tools/uavnav/arch_rules.yaml ros_allowed of this package): the conversion layer,
// the node (include/uavnav/px4bridge_node/, src/px4_bridge_node.cpp) and the executable's main. The main names no
// ROS type itself but belongs to the node target.
constexpr std::array<std::string_view, 4> kRosFiles = {"ros_conversion.hpp", "ros_conversion.cpp",
                                                       "px4_bridge_node.cpp", "px4_bridge_main.cpp"};
constexpr std::string_view kRosDirectory = "px4bridge_node";

// What the core must not name: the ROS client library, the PX4 messages, the generated v2 messages and the ROS
// helpers of uavnav_ros ("uavnav/ros/").
constexpr std::array<std::string_view, 5> kForbidden = {"rclcpp", "px4_msgs", "uavnav_interfaces/msg", "uavnav/ros/",
                                                        "uavnav_ros"};

std::string ReadAll(const fs::path& file) {
  std::ifstream in(file);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

/// The text without // and /* */ comments (a comment may say what the type maps onto without depending on it).
std::string StripComments(const std::string& text) {
  std::string out;
  out.reserve(text.size());
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (text.compare(i, 2, "//") == 0) {
      while (i < text.size() && text[i] != '\n') ++i;
      out += '\n';
    } else if (text.compare(i, 2, "/*") == 0) {
      const std::size_t end = text.find("*/", i + 2);
      i = end == std::string::npos ? text.size() : end + 1;
    } else {
      out += text[i];
    }
  }
  return out;
}

bool IsRosFile(const fs::path& file) {
  if (file.parent_path().filename() == kRosDirectory) return true;
  return std::any_of(kRosFiles.begin(), kRosFiles.end(), [&](std::string_view n) { return file.filename() == n; });
}

std::vector<fs::path> CoreFiles(const fs::path& package) {
  std::vector<fs::path> files;
  for (const char* dir : {"include", "src"}) {
    for (const auto& entry : fs::recursive_directory_iterator(package / dir)) {
      const fs::path& p = entry.path();
      if (!entry.is_regular_file() || IsRosFile(p)) continue;
      if (p.extension() == ".hpp" || p.extension() == ".cpp" || p.extension() == ".h") files.push_back(p);
    }
  }
  return files;
}

}  // namespace

TEST(Px4BridgeLibrary, CoreTargetHasNoRosDependency) {
  const fs::path package = UAVNAV_PX4_BRIDGE_SOURCE_DIR;
  const std::vector<fs::path> files = CoreFiles(package);
  // Not vacuous: the core has these (and the scan has not silently found nothing).
  ASSERT_GE(files.size(), 12U) << "core headers and sources found under " << package;
  const auto has = [&](const char* name) {
    return std::any_of(files.begin(), files.end(), [&](const fs::path& p) { return p.filename() == name; });
  };
  EXPECT_TRUE(has("bridge.hpp") && has("bridge.cpp") && has("alignment.cpp") && has("ev_encoder.hpp"));
  EXPECT_FALSE(has("ros_conversion.hpp")) << "the conversion layer is not a core file";

  for (const fs::path& file : files) {
    const std::string code = StripComments(ReadAll(file));
    for (const std::string_view word : kForbidden) {
      EXPECT_EQ(code.find(word), std::string::npos) << file << " names \"" << word << "\"";
    }
  }

  // The libraries the core links (comma separated, read from the target at configure time).
  const std::string libs = UAVNAV_PX4_BRIDGE_CORE_LIBS;
  EXPECT_NE(libs.find("uavnav_core"), std::string::npos) << "the link list was read: " << libs;
  for (const std::string_view word : kForbidden) {
    EXPECT_EQ(libs.find(word), std::string::npos) << "uavnav_px4_bridge_core links \"" << word << "\": " << libs;
  }
}
