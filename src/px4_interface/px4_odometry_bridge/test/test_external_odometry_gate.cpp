#include <gtest/gtest.h>

#include "px4_odometry_bridge/external_odometry_gate.hpp"

namespace px4_odometry_bridge {
namespace {

ExternalOdometryGateInput ready_input() {
  ExternalOdometryGateInput input;
  input.node_ready = true;
  input.transport_ready = true;
  input.timestamp_ready = true;
  input.covariance_ready = true;
  input.supervisor_authorized = true;
  input.public_frame_generation_valid = true;
  input.corrected_propagated_fresh = true;
  input.supervisor_fresh = true;
  input.frame_valid = true;
  return input;
}

TEST(ExternalOdometryGateTest, RequiresEveryIndependentGate) {
  const auto ready = evaluate_external_odometry_gate(ready_input());
  EXPECT_TRUE(ready.publication_ready);
  EXPECT_EQ(ready.reason, "READY");
  for (auto gate : {&ExternalOdometryGateInput::node_ready,
                    &ExternalOdometryGateInput::transport_ready,
                    &ExternalOdometryGateInput::timestamp_ready,
                    &ExternalOdometryGateInput::covariance_ready,
                    &ExternalOdometryGateInput::supervisor_authorized,
                    &ExternalOdometryGateInput::public_frame_generation_valid,
                    &ExternalOdometryGateInput::corrected_propagated_fresh,
                    &ExternalOdometryGateInput::supervisor_fresh,
                    &ExternalOdometryGateInput::frame_valid}) {
    auto input = ready_input();
    input.*gate = false;
    EXPECT_FALSE(evaluate_external_odometry_gate(input).publication_ready);
  }
}

TEST(ExternalOdometryGateTest, JumpLatchClosesGateWithoutChangingGeneration) {
  auto input = ready_input();
  input.geometric_jump_latched = true;
  const auto result = evaluate_external_odometry_gate(input);
  EXPECT_FALSE(result.publication_ready);
  EXPECT_EQ(result.reason, "GEOMETRIC_JUMP_LATCHED");
}

}  // namespace
}  // namespace px4_odometry_bridge
