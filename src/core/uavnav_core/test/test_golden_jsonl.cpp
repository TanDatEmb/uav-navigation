#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <string>

#include "uavnav/core/event.hpp"
#include "uavnav/core/jsonl_sink.hpp"

// event_v1.jsonl is written once, by hand, and pins the wire format of schema v1
// (key order, integer and double forms, non-finite value as null). The Python reader
// test loads the same file. Changing the format means a new golden file, not an edit.
using namespace uavnav::events;

TEST(GoldenJsonl, CppLineMatchesGoldenFile) {
  std::ifstream in(UAVNAV_CORE_GOLDEN_DIR "/event_v1.jsonl");
  ASSERT_TRUE(in.is_open()) << "cannot open " UAVNAV_CORE_GOLDEN_DIR "/event_v1.jsonl";
  std::string golden;
  ASSERT_TRUE(static_cast<bool>(std::getline(in, golden)));

  EventRecord r;
  r.t_steady_ns = 2'000'000'000;
  r.t_ros_ns = 1'500'000'000;
  r.component = Component::kLio;
  r.event = "LioStateChanged";
  r.state_before = "RUNNING";
  r.state_after = "STALE";
  r.reason = "STALE_ODOMETRY";
  r.identity = EventIdentity{7, 3, 42, 5, 9};
  ASSERT_TRUE(r.add_value("prefix_s", 1.5));
  ASSERT_TRUE(r.add_value("nan_value", std::numeric_limits<double>::quiet_NaN()));

  EXPECT_EQ(to_json_line(r), golden);
}
