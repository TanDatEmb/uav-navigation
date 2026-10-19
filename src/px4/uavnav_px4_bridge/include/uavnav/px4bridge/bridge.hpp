#pragma once

#include <algorithm>
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
//     A scan stamped after the newest PX4 sample waits in the alignment (alignment.hpp on_lio step 5, DV2): the
//     EV is still sent at once (D42), and the PX4 sample that decides the scan reports its pair (below).
//     Limitation (deferred to O25, S3/S4): an odometry of an OLDER epoch than the newest health is not gated.
//   /lio/health. Every non-TRACKING health (any epoch; the core treats every code other than kTracking as not
//     TRACKING) is fed as on_lio(tracking = false), so a LIO loss takes T VALID -> FROZEN (D11's BACKUP path),
//     never VALID -> INVALID by kStale. A state code outside the known range never reaches the core: the node
//     refuses it in the conversion (ros_conversion.cpp to_lio_health_in; rate-limited "LioInputRejected" with
//     reason NOT_FINITE, counted in rejected_health), so such a health neither gates the EV nor freezes T and T
//     stays as it was. A LIO state code added later (S4) must be added to LioHealthCode and the conversion range,
//     or its healths are ignored. A TRACKING health only updates the gate. The newest ARRIVED health is the gate's reference; a
//     health stamped before the stored one of the same epoch is refused (kOutOfOrder, "LioInputRejected") and
//     changes neither the gate nor the alignment (fail closed against an out-of-order delivery).
//   vehicle_local_position. A sample with xy_valid or z_valid false is skipped ("Px4SampleRejected",
//     kPx4PositionInvalid): not buffered, its reset counters not consumed, so a reset it carries is applied by the
//     next valid sample (PX4 keeps the counter and delta until the next reset). A valid one goes to on_px4.
//     When it decides the waiting scan (its output is a pair outcome: kPairAccepted, kPairsConsistent,
//     kPairRejectedJump or kNoPx4Sample), the outcome is the odometry's: its events are reported on the odometry
//     stream (input 0, sensor_time_s = the scan's stamp), and an accepted pair ends the odometry episodes if the
//     scan's own EV was encoded.
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
// input's epoch for PX4 samples and ticks).
// Rate limit of the per-input events (fault EPISODES, never a silent loss). A slot is (event, input stream, reason).
// The first occurrence of an episode is emitted at once ("count" 1); later ones are counted, with one emission on
// every limits::kEventSummaryEvery-th as a safety net. The episode ends at the next ACCEPTED input of the slot's
// stream: if occurrences went unreported, one SUMMARY record is emitted (same event and reason; values input or stream,
// count = the episode total, suppressed = the unreported ones, summary 1), and the slot re-arms, so the next fault
// episode is reported at once. Accepted: odometry = not refused, not gated, EV encoded and its pair not rejected
// (a waiting scan is accepted when the PX4 sample that decides it accepts its pair AND its own EV was encoded at its
// call, which the bridge keeps as the waiting scan's stamp);
// health = not refused and not rejected by the alignment; PX4 = not refused and accepted by the alignment; tick =
// not rejected. flush() ends every open episode (the node calls it at shutdown).
//   "AlignmentTransition"   one per alignment STATE CHANGE (before != after), keyed on the change and not on the
//                           reason: FROZEN -> VALID reports kLioTracking, or kPairRejectedJump when the returning
//                           pair was jump-rejected (alignment.hpp on_lio step 5). States, AlignmentReason; values
//                           input, sensor_time_s, the filtered T (x_m, y_m, z_m, yaw_rad), age_s, residuals.
//   "Px4ResetApplied"       one per applied PX4 reset (reason kPx4ResetApplied, or kPx4ResetWhileFrozen together
//                           with its FROZEN -> INVALID transition): states; the three counters, their steps since
//                           the previous accepted sample (uint8, modulo 256), the sample's deltas and the new T.
//   "AlignmentPairRejected" rate-limited per input: an alignment output with kPairRejectedJump, kNoPx4Sample or
//                           kInputRejected (also a refused PX4 sample: non-finite, beyond limits, stamp not newer);
//                           states; values input, sensor_time_s, count, and for a LIO input (which formed or tried
//                           a pair) residual_position_m, residual_yaw_rad (a PX4 sample forms no pair: none). A
//                           waiting scan decided by a PX4 sample is reported as input 0 with the scan's stamp; one
//                           dropped without a decision (a newer LIO input, a state change) as NO_PX4_SAMPLE, input 0,
//                           sensor_time_s = its stamp, without residuals, by the call that dropped it.
//   "AlignmentAccumulationRestarted" rate-limited (reason kLioLost, input 1): a non-TRACKING health in INIT threw
//                           away the INIT accumulation (no state change). Its episode ends at the next accepted
//                           odometry, i.e. when the accumulation grows again. Values sensor_time_s, count.
//   "EvGated"               rate-limited, BridgeReason::kHealthNotTracking: values odometry_time_s, health_time_s,
//                           health_state (LioHealthCode value), count.
//   "EvRejected"            rate-limited, EvReason of encode_ev: values sensor_time_s, count.
//   "Px4SampleRejected"     rate-limited, BridgeReason kPx4PositionInvalid (values xy_valid, z_valid) or
//                           kStampInFuture (values newest_tick_s); both with sensor_time_s, count.
//   "LioInputRejected"      rate-limited per stream, BridgeReason kStampInFuture for odometry (stream 0) and health
//                           (stream 1), kOutOfOrder for health: values stream, sensor_time_s, newest_tick_s (future)
//                           or newest_health_s (out of order), count.
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
/// kOutOfOrder: a health stamped before the newest health of the same epoch.
enum class BridgeReason : std::uint8_t {
  kAccepted,
  kHealthNotTracking,
  kPx4PositionInvalid,
  kStampInFuture,
  kNotFinite,
  kOutOfOrder
};

constexpr std::string_view to_string(BridgeReason r) {
  switch (r) {
    case BridgeReason::kAccepted: return "ACCEPTED";
    case BridgeReason::kHealthNotTracking: return "HEALTH_NOT_TRACKING";
    case BridgeReason::kPx4PositionInvalid: return "PX4_POSITION_INVALID";
    case BridgeReason::kStampInFuture: return "STAMP_IN_FUTURE";
    case BridgeReason::kNotFinite: return "NOT_FINITE";
    case BridgeReason::kOutOfOrder: return "OUT_OF_ORDER";
  }
  return "";
}
static_assert(ReasonEnum<BridgeReason>);

namespace detail {
/// Number of enumerators of a reason enum numbered 0..N-1: to_string is empty past the last one. Used to size the
/// rate-limit slots, so an appended enumerator can never index past them.
template <ReasonEnum E>
constexpr std::size_t enumerator_count() noexcept {
  std::size_t n = 0;
  while (n < 256 && !to_string(static_cast<E>(n)).empty()) ++n;
  return n;
}
}  // namespace detail

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

  /// Ends every open rate-limit episode: one summary per slot with unreported occurrences, then re-arms them all.
  /// The node calls it once at shutdown, before the recorder is destroyed, so no count is lost.
  void flush(const time::TimeSnapshot& now) noexcept;

 private:
  /// Input stream of a call; also the "input"/"stream" value of the events.
  enum class Input : std::uint8_t { kOdometry = 0, kHealth = 1, kPx4 = 2, kTick = 3 };
  static constexpr std::size_t kInputs = 4;
  /// The rate-limited events.
  enum class Spam : std::uint8_t {
    kEvGated,
    kEvRejected,
    kPairRejected,
    kAccumulationRestarted,
    kPx4SampleRejected,
    kLioInputRejected,
    kCount  ///< not an event: the number of rate-limited events
  };
  static constexpr std::size_t kSpamEvents = static_cast<std::size_t>(Spam::kCount);
  /// Reason values per slot: the largest enumerator count of the reason enums the slots hold.
  static constexpr std::size_t kReasonSlots =
      std::max({detail::enumerator_count<EvReason>(), detail::enumerator_count<AlignmentReason>(),
                detail::enumerator_count<BridgeReason>()});
  struct SpamCount {
    std::uint64_t count{0};     ///< occurrences in the current episode
    std::uint64_t reported{0};  ///< of which the last emitted record covered this many
  };
  struct Counters {
    std::uint8_t xy, z, heading;
  };

  bool in_future(time::SensorTime t) const noexcept;
  /// Emits the events of one alignment output (transition, pair rejection, accumulation restart) and returns it.
  AlignmentOutput report(AlignmentState before, const AlignmentOutput& out, Input input, time::SensorTime t,
                         std::uint32_t epoch, const time::TimeSnapshot& now) noexcept;
  /// A scan that waited before the call (`waiting`) and left without a decision (a newer LIO input, a state change;
  /// alignment.hpp on_lio step 5) is reported as its odometry's kNoPx4Sample: rate-limited AlignmentPairRejected,
  /// input 0, sensor_time_s = its stamp.
  void report_dropped(std::optional<time::SensorTime> waiting, AlignmentState before, const AlignmentOutput& out,
                      std::uint32_t epoch, const time::TimeSnapshot& now) noexcept;
  void report_reset(AlignmentState before, const AlignmentOutput& out, const Px4PoseSample& s,
                    const time::TimeSnapshot& now) noexcept;
  void refuse_lio(BridgeReason why, Input input, time::SensorTime t, std::uint32_t epoch,
                  const time::TimeSnapshot& now) noexcept;
  void refuse_px4(BridgeReason why, const Px4LocalPositionIn& in, const time::TimeSnapshot& now) noexcept;
  events::EventBuilder event(std::string_view name, std::uint32_t epoch, const time::TimeSnapshot& now) const noexcept;
  void emit(const events::EventBuilder& b) noexcept;

  static std::size_t slot(Spam e, Input stream, std::size_t reason) noexcept;
  /// Counts one occurrence of a slot; true when it is to be emitted (the 1st of the episode, every
  /// kEventSummaryEvery-th), which marks it reported.
  bool due(std::size_t slot) noexcept;
  /// End of the episodes of `stream` (an accepted input): summaries of unreported occurrences, then re-arm.
  void accepted(Input stream, const time::TimeSnapshot& now) noexcept;
  void summarise(std::size_t slot, const time::TimeSnapshot& now) noexcept;

  events::EventRecorder& events_;
  AlignmentEstimator alignment_;  // the one state machine (AlignmentState)

  // Data about recent inputs, not state.
  std::optional<LioHealthIn> latest_health_;     // newest arrived health (the EV gate's reference)
  std::optional<time::SensorTime> newest_tick_;  // reference of the future-stamp check
  std::uint32_t lio_epoch_{0};                   // epoch of the newest LIO input (event identity)
  std::optional<Counters> px4_counters_;         // of the newest sample on_px4 accepted (reset steps in events)
  std::optional<time::SensorTime> waiting_ev_encoded_;  // stamp of the waiting scan if its EV was encoded

  std::array<SpamCount, kSpamEvents * kInputs * kReasonSlots> spam_{};
};

}  // namespace uavnav::px4bridge
