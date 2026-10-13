#pragma once

#include "uavnav/lio_node/ingest.hpp"
#include "uavnav/lio_node/ros_conversion.hpp"

// Internals of LioNode's publishing, header-only so a test can drive it with publish functions that throw,
// without any test hook in the node (AGENTS §2.8; the pattern of uavnav_ros detail::publish_batch).
namespace uavnav::lio_node::detail {

/// "topic" value of LioPublishFailed.
inline constexpr int kTopicState = 0;
inline constexpr int kTopicHealth = 1;
inline constexpr int kTopicOdometry = 2;

/// Converts and publishes one call's outputs, each message in its own failure scope: the health first (a
/// LOST/DEGRADED transition is what a consumer must not miss), then every state, then the odometry. An exception
/// from one conversion or publish calls failed(topic) and never skips another message. Never throws as long as
/// failed() does not.
template <class State, class Health, class Odometry, class Failed>
void publish_outputs(const IngestOutputs& out, State&& state, Health&& health, Odometry&& odometry,
                     Failed&& failed) noexcept {
  if (out.health) {
    try {
      health(to_msg(*out.health));
    } catch (...) {
      failed(kTopicHealth);
    }
  }
  for (const LioStateOut& s : out.states) {
    try {
      state(to_msg(s));
    } catch (...) {
      failed(kTopicState);
    }
  }
  if (out.odometry) {
    try {
      odometry(to_msg(*out.odometry));
    } catch (...) {
      failed(kTopicOdometry);
    }
  }
}

}  // namespace uavnav::lio_node::detail
