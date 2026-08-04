#include <gtest/gtest.h>

#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "odometry_supervisor/diagnostic_adapter.hpp"

TEST(OdometryDiagnosticAdapter, SelectsNamedSchemaAndTypedValues) {
  diagnostic_msgs::msg::DiagnosticArray array;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "px4_odometry_bridge";
  for (const auto& [key, value] : {std::pair<std::string, std::string>{"diagnostic_schema_version", "1"},
                                   {"continuity_valid", "true"},
                                   {"reset_generation", "7"},
                                   {"px4_age_ns", "2.5"}}) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  }
  array.status.push_back(status);
  const auto snapshot = odometry_supervisor::selectDiagnostic(array, "px4_odometry_bridge");
  EXPECT_TRUE(snapshot.found);
  EXPECT_TRUE(snapshot.hasSchemaV1());
  EXPECT_TRUE(snapshot.boolean("continuity_valid"));
  EXPECT_EQ(snapshot.unsignedInteger("reset_generation"), 7U);
  EXPECT_DOUBLE_EQ(snapshot.number("px4_age_ns"), 2.5);
}

TEST(OdometryDiagnosticAdapter, MissingSchemaFailsClosed) {
  diagnostic_msgs::msg::DiagnosticArray array;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fast_lio/estimator";
  array.status.push_back(status);
  const auto snapshot = odometry_supervisor::selectDiagnostic(array, "fast_lio/estimator");
  EXPECT_TRUE(snapshot.found);
  EXPECT_FALSE(snapshot.hasSchemaV1());
}

TEST(OdometryDiagnosticAdapter, ExternalSchemaV2RequiresPublisherReadinessAndFreshStamp) {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp.sec = 10;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "px4_external_odometry_bridge";
  for (const auto& [key, value] : {std::pair<std::string, std::string>{"diagnostic_schema_version", "2"},
                                   {"publisher_ready", "true"}}) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  }
  array.status.push_back(status);
  const auto snapshot = odometry_supervisor::selectDiagnostic(
      array, "px4_external_odometry_bridge");
  EXPECT_TRUE(odometry_supervisor::externalPublisherReady(snapshot, 10'100'000'000LL,
                                                           200'000'000LL));
  EXPECT_FALSE(odometry_supervisor::externalPublisherReady(snapshot, 10'300'000'001LL,
                                                            200'000'000LL));

  auto not_ready = snapshot;
  not_ready.values["publisher_ready"] = "false";
  EXPECT_FALSE(odometry_supervisor::externalPublisherReady(not_ready, 10'100'000'000LL,
                                                            200'000'000LL));
  auto schema_one = snapshot;
  schema_one.values["diagnostic_schema_version"] = "1";
  EXPECT_FALSE(odometry_supervisor::externalPublisherReady(schema_one, 10'100'000'000LL,
                                                            200'000'000LL));
}
