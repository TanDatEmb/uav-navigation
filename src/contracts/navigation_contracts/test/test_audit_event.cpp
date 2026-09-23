#include <navigation_contracts/msg/audit_event.hpp>

#include <gtest/gtest.h>

TEST(AuditEvent, FixedSchemaKeepsIndependentClockAndIdentityFields) {
  navigation_contracts::msg::AuditEvent event{};
  event.event_type = navigation_contracts::msg::AuditEvent::MISSION_GATE;
  event.producer_id = 2;
  event.process_incarnation = 17;
  event.diagnostic_sequence = 18;
  event.steady_ns = 19;
  event.ros_now_ns = 20;
  event.source_stamp_ns = 21;
  event.previous_sample_stamp_ns = 22;
  event.current_sample_stamp_ns = 23;
  event.request_id = 24;
  event.sample_id = 25;
  EXPECT_EQ(event.process_incarnation, 17U);
  EXPECT_NE(event.ros_now_ns, event.source_stamp_ns);
  EXPECT_NE(event.previous_sample_stamp_ns, event.current_sample_stamp_ns);
  EXPECT_NE(event.request_id, event.sample_id);
}
