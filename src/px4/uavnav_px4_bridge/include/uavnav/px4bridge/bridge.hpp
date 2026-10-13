#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uavnav/core/event_builder.hpp"
#include "uavnav/core/event_recorder.hpp"
#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/px4bridge/alignment.hpp"
#include "uavnav/px4bridge/ev_encoder.hpp"

// Px4Bridge: the ROS-free decision layer of the px4_bridge node (SYSTEM_DESIGN §4.1, D11, D21, D28, F13, F14;
// D33/O21, O22). The node (Task 10) converts messages and publishes; every decision is here. Single-threaded: the
// node's one executor thread makes every call (S1b runtime design); nothing here is thread-safe or blocks.
//
// Inputs (O22).
//   /lio/odometry. LIO publishes odometry only while TRACKING (lio frontend), so an odometry sample is a tracking
//     pose unless the bridge knows better: the EV is GATED (no EV, no alignment call, "EvGated") when the newest
//     /lio/health of the SAME epoch is not TRACKING and its stamp is >= the odometry stamp. An older non-TRACKING
//     health, or one of another epoch, does not gate. Otherwise the sample is encoded (encode_ev with its OWN epoch,
//     F14, and limits::kClockMode; a refusal is "EvRejected" with the EvReason) and, independently, fed to the
//     alignment as on_lio(tracking = true) with the FRD pose (frames.hpp; a quaternion that is not a rotation gives
//     a NaN yaw, which the alignment refuses). A gated sample is not fed: its non-TRACKING health already was.
//   /lio/health. Every non-TRACKING health (any epoch; an unknown state code counts as not TRACKING) is fed as
//     on_lio(tracking = false), so a LIO loss takes T VALID -> FROZEN (D11's BACKUP path), never VALID -> INVALID
//     by kStale. A TRACKING health only updates the gate. The newest ARRIVED health is the gate's reference.
//   vehicle_local_position. A sample with xy_valid or z_valid false is skipped ("Px4SampleRejected",
//     kPx4PositionInvalid): not buffered, its reset counters not consumed, so a reset it carries is applied by the
//     next valid sample (PX4 keeps the counter and delta until the next reset). A valid one goes to on_px4.
//   on_tick, every limits::kTickPeriod of ROS time (sensor time in SITL, D29): the alignment's timeouts.
// Future stamps: any input stamp later than the newest tick time + limits::kMaxStampAhead is refused
// (kStampInFuture; "LioInputRejected" for odometry and health, "Px4SampleRejected" for PX4) before any other
// check, and changes nothing. Before the first tick nothing is refused as ahead. A tick earlier than the newest
// one keeps the newest as the reference.
// A tracking odometry stamped at or before a non-TRACKING health already fed is refused by the alignment
// (kInputRejected, its stamps are strictly increasing); this happens at a DEGRADED -> TRACKING edge until the
// scan stamps pass the last DEGRADED health, and is reported like any pair rejection.
//
// Outputs (O21): `alignment` is set for every on_lio and on_tick output and for an on_px4 output whose reason is
// not kNone; `ev` only for an encoded odometry. A refused input returns no output.
//
// Events (component px4_bridge; `now` stamps them, identity.lio_epoch = the input's epoch, or the newest LIO
// input's epoch for PX4 samples and ticks). Rate-limited ones are emitted on the 1st and every
// limits::kEventSummaryEvery-th occurrence per event and reason, with the running "count".
//   "AlignmentTransition"   one per alignment STATE CHANGE (before != after), keyed on the change and not on the
//                           reason: FROZEN -> VALID reports kLioTracking, or kPairRejectedJump when the returning
//                           pair was jump-rejected (alignment.hpp on_lio step 5). States, AlignmentReason; values
//                           input, sensor_time_s, the filtered T (x_m, y_m, z_m, yaw_rad), age_s, residuals.
//   "Px4ResetApplied"       one per applied PX4 reset (reason kPx4ResetApplied, or kPx4ResetWhileFrozen together
//                           with its FROZEN -> INVALID transition): states; the three counters, their steps since
//                           the previous accepted sample (uint8, modulo 256) and the sample's deltas.
//   "AlignmentPairRejected" rate-limited: an alignment output with kPairRejectedJump, kNoPx4Sample or
//                           kInputRejected (also a refused PX4 sample: non-finite, beyond limits, stamp not newer);
//                           states; values input, sensor_time_s, residual_position_m, residual_yaw_rad, count.
//   "EvGated"               rate-limited, BridgeReason::kHealthNotTracking: values odometry_time_s, health_time_s,
//                           health_state (LioHealthCode value), count.
//   "EvRejected"            rate-limited, EvReason of encode_ev: values sensor_time_s, count.
//   "Px4SampleRejected"     rate-limited, BridgeReason kPx4PositionInvalid (values xy_valid, z_valid) or
//                           kStampInFuture (values newest_tick_s); both with sensor_time_s, count.
//   "LioInputRejected"      rate-limited, BridgeReason::kStampInFuture for odometry (stream 0) and health (stream 1):
//                           values stream, sensor_time_s, newest_tick_s, count.
// "input" values: 0 odometry, 1 health, 2 PX4, 3 tick.
namespace uavnav::px4bridge {

/// LioHealth.msg state values.
enum class LioHealthCode : std::uint8_t { kInitializing = 0, kTracking, kDegraded, kLost, kRestarting };

static_assert(static_cast<std::uint8_t>(LioHealthCode::kTracking) == 1);
static_assert(static_cast<std::uint8_t>(LioHealthCode::kRestarting) == 4);

constexpr std::string_view to_string(LioHealthCode c) {
  switch (c) {
    case LioHealthCode::kInitializing: return "INITIALIZING";
    case LioHealthCode::kTracking: return "TRACKING";
    case LioHealthCode::kDegraded: return "DEGRADED";
    case LioHealthCode::kLost: return "LOST";
    case LioHealthCode::kRestarting: return "RESTARTING";
  }
  return "";
}
static_assert(ReasonEnum<LioHealthCode>);

/// One /lio/odometry sample (LioOdometry.msg): base_link in lio_odom, FLU, at the scan time.
struct LioOdometryIn {
  time::SensorTime t;
  std::uint32_t epoch;                    ///< LIO reset epoch of THIS sample (F14)
  Eigen::Vector3d p_flu_m;
  Eigen::Quaterniond q_flu;               ///< q_world_body, lio_odom <- base_link
  Eigen::Vector3d v_flu_mps;              ///< lio_odom axes
  Eigen::Matrix<double, 6, 6> pose_cov;   ///< (position, rotation), lio_odom (world) axes
  Eigen::Matrix3d vel_cov;                ///< lio_odom axes
  std::uint8_t quality;                   ///< 0..100 (D28)
};

/// One /lio/health sample: t is the SensorTime of the newest IMU sample evaluated (D28).
struct LioHealthIn {
  time::SensorTime t;
  LioHealthCode state;
  std::uint32_t epoch;
};

/// One vehicle_local_position sample, stamped in SensorTime (from_px4_us), with PX4's validity flags.
struct Px4LocalPositionIn {
  Px4PoseSample sample;
  bool xy_valid;
  bool z_valid;
};

/// kNotFinite is for the node's message conversions (Task 10); the core reports a non-finite PX4 sample through
/// the alignment (kInputRejected) and a non-finite odometry through the encoder (kNonFiniteState).
enum class BridgeReason : std::uint8_t {
  kAccepted,
  kHealthNotTracking,
  kPx4PositionInvalid,
  kStampInFuture,
  kNotFinite
};

constexpr std::string_view to_string(BridgeReason r) {
  switch (r) {
    case BridgeReason::kAccepted: return "ACCEPTED";
    case BridgeReason::kHealthNotTracking: return "HEALTH_NOT_TRACKING";
    case BridgeReason::kPx4PositionInvalid: return "PX4_POSITION_INVALID";
    case BridgeReason::kStampInFuture: return "STAMP_IN_FUTURE";
    case BridgeReason::kNotFinite: return "NOT_FINITE";
  }
  return "";
}
static_assert(ReasonEnum<BridgeReason>);

/// What the node publishes for one call (header comment "Outputs").
struct BridgeOutputs {
  std::optional<EvSample> ev;                ///< -> /fmu/in/vehicle_visual_odometry
  std::optional<AlignmentOutput> alignment;  ///< -> /alignment
};

class Px4Bridge {
 public:
  /// The alignment is built from `cfg` (load_alignment_config; an invalid one never leaves INIT, alignment.hpp).
  /// The EV clock mode is limits::kClockMode. `events` must outlive the bridge.
  Px4Bridge(const AlignmentConfig& cfg, events::EventRecorder& events) noexcept;

  BridgeOutputs on_lio_odometry(const LioOdometryIn& in, const time::TimeSnapshot& now);
  BridgeOutputs on_lio_health(const LioHealthIn& in, const time::TimeSnapshot& now);
  BridgeOutputs on_px4_local_position(const Px4LocalPositionIn& in, const time::TimeSnapshot& now);
  /// now_sensor: the tick's ROS time as SensorTime (D29).
  BridgeOutputs on_tick(time::SensorTime now_sensor, const time::TimeSnapshot& now);

  AlignmentState alignment_state() const noexcept { return alignment_.state(); }

 private:
  enum class Input : std::uint8_t { kOdometry = 0, kHealth = 1, kPx4 = 2, kTick = 3 };
  struct Counters {
    std::uint8_t xy, z, heading;
  };
  /// Running counts of the rate-limited events, one slot per reason value.
  template <std::size_t N>
  using Counts = std::array<std::uint64_t, N>;

  bool in_future(time::SensorTime t) const noexcept;
  /// Emits the events of one alignment output (transition, pair rejection) and returns it.
  AlignmentOutput report(AlignmentState before, const AlignmentOutput& out, Input input, time::SensorTime t,
                         std::uint32_t epoch, const time::TimeSnapshot& now) noexcept;
  void report_reset(AlignmentState before, const AlignmentOutput& out, const Px4PoseSample& s,
                    const time::TimeSnapshot& now) noexcept;
  void refuse_lio_future(Input input, time::SensorTime t, std::uint32_t epoch, const time::TimeSnapshot& now) noexcept;
  void refuse_px4(BridgeReason why, const Px4LocalPositionIn& in, const time::TimeSnapshot& now) noexcept;
  events::EventBuilder event(std::string_view name, std::uint32_t epoch, const time::TimeSnapshot& now) const noexcept;
  void emit(const events::EventBuilder& b) noexcept;
  /// Increments `count` and says whether this occurrence is reported (the 1st and every kEventSummaryEvery-th).
  static bool due(std::uint64_t& count) noexcept;

  events::EventRecorder& events_;
  AlignmentEstimator alignment_;  // the one state machine (AlignmentState)

  // Data about recent inputs, not state.
  std::optional<LioHealthIn> latest_health_;     // newest arrived health (the EV gate's reference)
  std::optional<time::SensorTime> newest_tick_;  // reference of the future-stamp check
  std::uint32_t lio_epoch_{0};                   // epoch of the newest LIO input (event identity)
  std::optional<Counters> px4_counters_;         // of the newest sample on_px4 accepted (reset steps in events)

  Counts<1> ev_gated_{};
  Counts<6> ev_rejected_{};      // by EvReason
  Counts<12> pair_rejected_{};   // by AlignmentReason
  Counts<5> px4_rejected_{};     // by BridgeReason
  Counts<1> lio_future_{};
};

}  // namespace uavnav::px4bridge
