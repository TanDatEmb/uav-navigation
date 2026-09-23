#include <navigation_contracts/audit_event_sink.hpp>

#include <gtest/gtest.h>

TEST(AuditSink, BoundedQueueDropsWithoutBlockingProducer) {
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("audit_sink_test");
    navigation_contracts::audit::Sink sink(*node, 9U);
    navigation_contracts::audit::Event event{};
    event.event_type = navigation_contracts::audit::Event::MISSION_GATE;
    for (std::size_t i = 0; i < sink.kCapacity + 1; ++i) sink.emit(event);
    const auto stats = sink.stats();
    EXPECT_EQ(stats.enqueued, sink.kCapacity);
    EXPECT_EQ(stats.dropped, 1U);
    EXPECT_EQ(stats.max_occupancy, sink.kCapacity);
  }
  rclcpp::shutdown();
}

TEST(AuditSink, MissionHashIsStableAndSensitive) {
  EXPECT_EQ(navigation_contracts::audit::missionHash("mission"),
            navigation_contracts::audit::missionHash("mission"));
  EXPECT_NE(navigation_contracts::audit::missionHash("mission"),
            navigation_contracts::audit::missionHash("mission2"));
}
