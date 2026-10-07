#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include <Eigen/Dense>

#include "uavnav/core/result.hpp"
#include "uavnav/core/time.hpp"
#include "uavnav/px4bridge/limits.hpp"

// Alignment T_px4<-lio (SYSTEM_DESIGN §4.1, D7, D21, F13, D12 rule 1, D28).
//
// Pure state machine: no ROS, no clock reads, no threads, no allocation after construction. One state
// enum (AlignmentState), one transition table (kAlignmentTransitions), one writer (the three on_* calls,
// each applying at most one state change through the table).
//
// ---- Frames and the meaning of T -------------------------------------------------------------------------
// LIO poses enter already converted to FRD (frames.hpp): lio_odom FRD is gravity-aligned with z down, like
// PX4's local NED frame; the two differ by a translation and a rotation about the common down axis. T maps
// lio_odom FRD coordinates to PX4 local NED:
//     p_px4 = Rz(yaw_T) * p_lio_frd + (x_T, y_T, z_T)        yaw_px4 = yaw_T + yaw_lio_frd
// with Rz the rotation about z (down): [x', y'] = [c x - s y, s x + c y]. z is a pure translation.
// Instant T of one pair: yaw = wrap(yaw_px4 - yaw_lio), (x, y) = p_px4_xy - Rz(yaw) p_lio_xy,
// z = z_px4 - z_lio. Every angle that leaves this module is wrapped to [-pi, pi]; every yaw difference,
// mean and filter error is a shortest-arc (wrapped) difference, never a raw difference across +-pi.
//
// ---- Every measurement of T at the vehicle (D30/O12) -------------------------------------------------------
// Far from the lio_odom origin a small yaw error times the lever arm |p_lio| is a large translation error of
// T (at 300 m, 1.7 mrad is 0.5 m), but what must stay right is where T puts the VEHICLE (the setpoint). So
// residual, jump gate, filter step and INIT consistency are all measured at the vehicle point of the pair.
// For a pair (p_lio, yaw_lio | p_px4, yaw_px4 interpolated at the scan time), its instant T (yaw_inst, t_inst)
// and a reference T (the filtered T in VALID, the accumulation mean in INIT):
//     q     = T(p_lio) = (Rz(yaw_T) p_lio_xy + t_xy, z_lio + t_z)    where T puts the vehicle
//     e     = p_px4 - q                                            position error at the vehicle (3-D)
//     e_yaw = wrap(yaw_inst - yaw_T)
// The S1a origin measure t_inst - t_T = e - (Rz(yaw_inst) - Rz(yaw_T)) p_lio_xy adds a lever-arm term that
// grows with |p_lio| and says nothing about the vehicle; it is not used.
//   Jump gate: |e| > jump_position_m or |e_yaw| > jump_yaw_rad (strict) rejects the pair, T unchanged. Every
//   gate is written !(within), so a NaN residual or threshold rejects (fail closed).
//   Filter step (alpha = dt / tau, dt clamped): the frame change G below with pivot q, shift alpha e and
//   rotation dh = alpha e_yaw:  yaw_T += dh,  t <- Rz(dh)(t - q) + q + alpha e.
//   Since Rz(yaw_T) p_lio_xy = q_xy - t_xy,
//       T'(p_lio)_xy = Rz(dh)(q_xy - t_xy) + Rz(dh)(t_xy - q_xy) + q_xy + alpha e_xy = q_xy + alpha e_xy,
//   and z: t_z += alpha e_z. So the vehicle point moves by exactly alpha e and the yaw part is a rotation about
//   the vehicle: a pure yaw step does not move the vehicle point.
//   Implied rate bound (there is no separate rate limiter, D30): an accepted pair has |e| <= jump_position_m
//   and |e_yaw| <= jump_yaw_rad, so one step moves T at the vehicle by at most jump_position_m dt / tau and
//   its yaw by at most jump_yaw_rad dt / tau: 0.25 m/s and 0.0436 rad/s (2.5 deg/s) with the beta values.
//   A change of T that moves the vehicle point faster than that is not followed: its pairs end in
//   kPairRejectedJump, then kStale -> INVALID (spec intent; e.g. a frame rotation about the LIO origin of
//   1 mrad/s at 300 m moves the vehicle point 0.3 m/s). A rotation about the vehicle is tracked with a yaw
//   lag of rate x tau.
//   INIT: each member keeps (instant T, p_lio, p_px4). Mean yaw psi = circular mean of the instant yaws; the
//   mean translation is the least-squares t given psi (argmin_t sum |p_px4_i - Rz(psi) p_lio_i - t|^2):
//       t_xy = mean(p_px4_xy) - Rz(psi) mean(p_lio_xy)        t_z = mean(z_px4 - z_lio)
//   A new pair is checked at its vehicle point against the running mean; the final re-check measures every
//   member at its OWN vehicle point against the final mean.
// With p_lio = 0 every rule reduces to the origin rules (q = t_T, e = t_inst - t_T, mean t = mean t_inst).
//
// ---- PX4 reset deltas (F13) --------------------------------------------------------------------------------
// vehicle_local_position carries one uint8 counter per quantity (xy, z, heading) and the delta of the
// latest reset: delta = estimate after the reset - estimate before it (PX4 EKF2: posNE_change = new - old,
// quat_change = q_new * q_old^-1), i.e. "delta is ADDED to the PX4 position/heading estimate at the reset".
// When several resets of one quantity happen inside one EKF2 update, PX4 accumulates them into the one
// delta and the counter steps by 2 or more; so ANY counter difference (any step, including the uint8 wrap
// 255 -> 0) means "the PX4 local frame changed by this sample's delta" and is applied exactly once.
// The frame change G (old PX4 frame -> new) is a heading rotation about the vehicle followed by the
// position shift (EKF2 keeps the position continuous across a heading reset):
//     G(p)   = Rz(dh) (p - pivot) + pivot + (dxy, dz)        G(yaw) = yaw + dh
//     pivot  = sample position - (dxy, dz) = the vehicle position just before the reset (old frame)
// where dxy / dz / dh are the sample's deltas for the quantities whose counter changed (0 for the others).
// G is applied to the filtered T (T' = G o T: yaw_T += dh exactly, t' = t + (Rz(dh) - I)(t - pivot) + d,
// so a translation-only reset adds d exactly) and to every buffered PX4 sample, so an interpolation across
// the reset stays continuous. Limitation: a lost vehicle_local_position message that held an earlier reset
// is not recoverable from the counters (only the newest delta is carried); its jump is then caught by the
// jump rejection and, if persistent, by kStale.
//
// ---- Time -------------------------------------------------------------------------------------------------
// All stamps are SensorTime from the range-checked converters (time::from_stamp<SensorTag>), so they are
// >= 0 and differences cannot overflow; a negative stamp is rejected (kInputRejected). The SITL PX4 clock is
// the sensor clock (px4_time.hpp), so PX4 samples are stamped in SensorTime by the node.
namespace uavnav::px4bridge {

/// Numeric values equal the Alignment.msg constants (INIT=0, VALID=1, FROZEN=2, INVALID=3).
enum class AlignmentState : std::uint8_t { kInit = 0, kValid = 1, kFrozen = 2, kInvalid = 3 };

static_assert(static_cast<std::uint8_t>(AlignmentState::kInit) == 0);
static_assert(static_cast<std::uint8_t>(AlignmentState::kValid) == 1);
static_assert(static_cast<std::uint8_t>(AlignmentState::kFrozen) == 2);
static_assert(static_cast<std::uint8_t>(AlignmentState::kInvalid) == 3);

/// Why the output looks as it does. kInputRejected is an addition to the brief's list (required to fail
/// closed on a non-finite value, a negative stamp or a time regression without misreporting another reason).
enum class AlignmentReason : std::uint8_t {
  kNone,               ///< nothing happened (an ordinary PX4 sample, an on_tick without timeout, ...)
  kPairsConsistent,    ///< INIT -> VALID
  kPairAccepted,       ///< the pair entered the INIT accumulation or the VALID filter
  kPairRejectedJump,   ///< VALID: the pair beyond a jump threshold of the filtered T, measured at the vehicle
                       ///< (T unchanged); INIT: the pair inconsistent with the running mean at the vehicle, or
                       ///< a member with the final mean (accumulation restarted from the pair)
  kLioLost,            ///< VALID -> FROZEN; in INIT the accumulation restarts
  kLioTracking,        ///< FROZEN -> VALID, INVALID -> INIT
  kPx4ResetApplied,    ///< a PX4 reset was applied to T and the buffer (VALID/INIT/INVALID)
  kPx4ResetWhileFrozen,  ///< FROZEN -> INVALID
  kStale,              ///< VALID -> INVALID: no accepted pair for valid_stale
  kFrozenTooLong,      ///< FROZEN -> INVALID: FROZEN for longer than frozen_max
  kNoPx4Sample,        ///< no PX4 sample within kPairingWindow on both sides of the scan; nothing changed
  kInputRejected       ///< non-finite value, position beyond limits::kMaxPositionAbsM, negative stamp, or
                       ///< stamp not newer than the previous one;
                       ///< also a refused unlisted state change (a programming error, state unchanged)
};

constexpr std::string_view to_string(AlignmentState s) {
  switch (s) {
    case AlignmentState::kInit: return "INIT";
    case AlignmentState::kValid: return "VALID";
    case AlignmentState::kFrozen: return "FROZEN";
    case AlignmentState::kInvalid: return "INVALID";
  }
  return "";
}

constexpr std::string_view to_string(AlignmentReason r) {
  switch (r) {
    case AlignmentReason::kNone: return "NONE";
    case AlignmentReason::kPairsConsistent: return "PAIRS_CONSISTENT";
    case AlignmentReason::kPairAccepted: return "PAIR_ACCEPTED";
    case AlignmentReason::kPairRejectedJump: return "PAIR_REJECTED_JUMP";
    case AlignmentReason::kLioLost: return "LIO_LOST";
    case AlignmentReason::kLioTracking: return "LIO_TRACKING";
    case AlignmentReason::kPx4ResetApplied: return "PX4_RESET_APPLIED";
    case AlignmentReason::kPx4ResetWhileFrozen: return "PX4_RESET_WHILE_FROZEN";
    case AlignmentReason::kStale: return "STALE";
    case AlignmentReason::kFrozenTooLong: return "FROZEN_TOO_LONG";
    case AlignmentReason::kNoPx4Sample: return "NO_PX4_SAMPLE";
    case AlignmentReason::kInputRejected: return "INPUT_REJECTED";
  }
  return "";
}

static_assert(ReasonEnum<AlignmentState>);
static_assert(ReasonEnum<AlignmentReason>);

/// T maps FRD-converted lio_odom -> PX4 local NED (see the header comment). yaw_rad in [-pi, pi].
struct Pose4 {
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double yaw_rad{0.0};
};

/// From px4_msgs vehicle_local_position (x, y, z, heading, *_reset_counter, delta_*), stamped in SensorTime.
struct Px4PoseSample {
  time::SensorTime t;
  Eigen::Vector3d p_ned_m{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
  std::uint8_t xy_reset_counter{0};
  std::uint8_t z_reset_counter{0};
  std::uint8_t heading_reset_counter{0};
  Eigen::Vector2d delta_xy_m{Eigen::Vector2d::Zero()};
  double delta_z_m{0.0};
  double delta_heading_rad{0.0};
};

/// LIO pose at the scan time, in lio_odom FRD (converted with flu_to_frd). yaw_frd_rad is the heading about
/// the FRD down axis.
struct LioPoseSample {
  time::SensorTime t;
  Eigen::Vector3d p_frd_m{Eigen::Vector3d::Zero()};
  double yaw_frd_rad{0.0};
  bool tracking{false};  ///< LIO state is TRACKING for this sample
};

/// Tier (b), loaded from kAlignmentSpecs (config.hpp). Beta values in parentheses.
struct AlignmentConfig {
  time::Duration tau;                ///< first-order filter time constant (2 s)
  std::uint32_t consistent_pairs;    ///< consecutive consistent pairs INIT -> VALID (20)
  double jump_position_m;            ///< |p_px4 - T(p_lio)| (3-D, at the vehicle) rejection threshold (0.5 m)
  double jump_yaw_rad;               ///< |wrap(yaw_inst - yaw_T)| rejection threshold (0.0873 rad)
  time::Duration valid_stale;        ///< VALID without an accepted pair for longer -> INVALID (1 s)
  time::Duration frozen_max;         ///< FROZEN for longer -> INVALID (10 s = lio_recovery_timeout_s, D18)
};

/// One output per call. Every double is finite in every output (no NaN, no inf). The S1b node publishes it as
/// Alignment.msg: x_m..yaw_rad <- filtered, raw_* <- raw, residual_* <- residual_*, age_s, stamp.
struct AlignmentOutput {
  AlignmentState state{AlignmentState::kInit};
  /// The lio_odom FRD -> PX4 local NED transform (header comment), after this call's step.
  /// VALID/FROZEN: the filtered T. INIT: the running (least-squares) mean of the accumulation (zero when
  /// empty); not usable. INVALID: the last T, kept for logging only.
  Pose4 filtered{};
  /// Instant T of the pair formed in THIS call (also when rejected); nullopt when no pair was formed. Same
  /// transform as `filtered`. raw - filtered is NOT a residual: for this pair's p_lio its translation part is
  ///     (p_px4 - filtered(p_lio)) - (Rz(yaw_raw) - Rz(yaw_filtered)) p_lio_xy
  /// i.e. it includes the lever arm (Rz(yaw_raw) - Rz(yaw_filtered)) p_lio_xy, which grows with |p_lio| (1.5 m
  /// for 5 mrad at 300 m). Use residual_* for the error at the vehicle.
  std::optional<Pose4> raw;
  /// SensorTime of the LIO sample of the newest formed pair (D28); 0 before the first pair.
  time::SensorTime stamp{};
  /// Seconds from the last accepted pair to the newest time seen (LIO stamp or tick); 0 before the first
  /// accepted pair (the state then says INIT, so the value is never used to trust T).
  double age_s{0.0};
  /// Residual of the newest formed pair, measured at its vehicle point against the reference T BEFORE this
  /// pair's step (VALID: the filtered T; INIT: the running mean before the pair was added; header comment):
  /// residual_position_m = |p_px4 - T(p_lio)| (3-D), residual_yaw_rad = wrap(yaw_inst - yaw_T). This is what
  /// the jump gate compares. 0 before the first pair and for a pair that enters an empty accumulation.
  /// When the INIT final re-check rejects (kPairRejectedJump with the accumulation restarted), these still
  /// hold the NEWEST pair's arrival residual, which passed the gate; the member that failed the final mean
  /// is not reported here.
  double residual_position_m{0.0};
  double residual_yaw_rad{0.0};
  AlignmentReason reason{AlignmentReason::kNone};
};

/// A legal state change. Anything not listed cannot happen: AlignmentEstimator refuses an unlisted edge,
/// keeps its state and reports kInputRejected (fail closed, never a silent transition reason).
struct AlignmentEdge {
  AlignmentState from;
  AlignmentState to;
  AlignmentReason reason;
};

/// SYSTEM_DESIGN §4.1 diagram. INIT and INVALID have no lost/stale edges: they hold no trusted T.
inline constexpr auto kAlignmentTransitions = std::to_array<AlignmentEdge>({
    {AlignmentState::kInit, AlignmentState::kValid, AlignmentReason::kPairsConsistent},
    {AlignmentState::kValid, AlignmentState::kFrozen, AlignmentReason::kLioLost},
    {AlignmentState::kFrozen, AlignmentState::kValid, AlignmentReason::kLioTracking},
    {AlignmentState::kValid, AlignmentState::kInvalid, AlignmentReason::kStale},
    {AlignmentState::kFrozen, AlignmentState::kInvalid, AlignmentReason::kFrozenTooLong},
    {AlignmentState::kFrozen, AlignmentState::kInvalid, AlignmentReason::kPx4ResetWhileFrozen},
    {AlignmentState::kInvalid, AlignmentState::kInit, AlignmentReason::kLioTracking},
});

class AlignmentEstimator {
 public:
  /// A config outside the kAlignmentSpecs bounds (or with tau < kFilterDtMax) cannot be reported from a
  /// constructor; such an estimator fails closed by never leaving INIT. Use load_alignment_config.
  explicit AlignmentEstimator(const AlignmentConfig& cfg) noexcept;

  /// Buffers one PX4 sample (fixed ring, kPx4BufferSpan) and applies its reset deltas (F13).
  /// Rejected (kInputRejected, nothing changes, counters not consumed): a non-finite field, a position
  /// component beyond limits::kMaxPositionAbsM or a delta_xy/delta_z component beyond twice that, t < 0, or
  /// t <= the newest buffered stamp. A rejected sample's reset is therefore applied by the next accepted
  /// sample, which carries the same counter and delta (PX4 keeps both until the next reset).
  /// The first accepted sample only initialises the last-seen counters (kNone).
  /// A counter change: G is applied to T and to the buffer, then
  ///   VALID   -> stays VALID, kPx4ResetApplied
  ///   INIT    -> the accumulation restarts, kPx4ResetApplied
  ///   FROZEN  -> INVALID, kPx4ResetWhileFrozen (T can no longer be trusted)
  ///   INVALID -> stays INVALID, kPx4ResetApplied
  /// Returns an output (the brief's signature returned void; the reset reason must reach the caller's
  /// event at the decision point, AGENTS.md §2.3). Ignoring it is allowed.
  AlignmentOutput on_px4(const Px4PoseSample& s) noexcept;

  /// Pairs the scan with PX4 interpolated at s.t (linear position, shortest-arc yaw, the two buffered
  /// samples around s.t, each within kPairingWindow; an exact stamp match uses that sample alone).
  /// Order of checks, first match wins:
  ///  1. s.t < 0 or s.t <= previous accepted LIO stamp          -> kInputRejected, nothing changes
  ///  2. tracking and a non-finite pose or a position component beyond limits::kMaxPositionAbsM
  ///                                                             -> kInputRejected, nothing changes
  ///  3. VALID and s.t - stale_since > valid_stale               -> INVALID, kStale
  ///     FROZEN and s.t - frozen_since > frozen_max              -> INVALID, kFrozenTooLong
  ///  4. !tracking: VALID -> FROZEN (kLioLost); INIT restarts the accumulation (kLioLost); FROZEN and
  ///     INVALID unchanged (kNone). The pose of a lost sample is never used.
  ///  5. tracking:
  ///     INVALID -> INIT (kLioTracking); the new accumulation starts with the NEXT pair.
  ///     FROZEN  -> VALID (kLioTracking); the pair, if any, is then processed as in VALID, but the
  ///                reason stays kLioTracking (the jump rejection protects T).
  ///     no PX4 pair                                             -> kNoPx4Sample, nothing changes
  ///     INIT: consistent with the running mean at the vehicle -> added (kPairAccepted), and at
  ///           consistent_pairs and every member within the thresholds of the final mean at its own vehicle
  ///           point -> VALID with T = mean (kPairsConsistent); inconsistent -> the accumulation restarts
  ///           from this pair (kPairRejectedJump).
  ///     VALID: beyond a jump threshold of the filtered T at the vehicle (strictly greater) ->
  ///            kPairRejectedJump, T unchanged; else one filter step (kPairAccepted).
  /// Filter step (header comment): dt = clamp(s.t - previous accepted pair, kFilterDtMin, kFilterDtMax),
  /// alpha = dt / tau; yaw rotated by alpha e_yaw about the vehicle point q, vehicle point moved by alpha e.
  /// No separate rate limiter (D30). The INIT -> VALID pair sets T = mean and takes no step.
  /// An output with tracking=false is never VALID.
  AlignmentOutput on_lio(const LioPoseSample& s) noexcept;

  /// Staleness and frozen timeout at `now` (same rules as on_lio step 3). now earlier than the newest time
  /// seen (LIO stamp or tick) does nothing (kNone).
  AlignmentOutput on_tick(time::SensorTime now) noexcept;

  AlignmentState state() const noexcept { return state_; }

 private:
  struct Interpolated {
    Eigen::Vector3d p;
    double yaw;
  };
  /// One formed pair: its instant T and the two vehicle positions it was made from.
  struct Pair {
    Pose4 inst;
    Eigen::Vector3d p_lio;  // lio_odom FRD
    Eigen::Vector3d p_px4;  // PX4 local NED, interpolated at the scan time
  };

  /// The one writer of state_. Returns false, leaving the state unchanged, for an edge not in
  /// kAlignmentTransitions (a programming error; asserted in debug builds, reported as kInputRejected).
  bool go(AlignmentState to, AlignmentReason reason, time::SensorTime t) noexcept;
  AlignmentOutput output(AlignmentReason reason, const std::optional<Pose4>& raw = std::nullopt) const noexcept;
  std::optional<Interpolated> px4_at(time::SensorTime t) const noexcept;
  void restart_accumulation() noexcept;
  void add_to_accumulation(const Pair& pair) noexcept;
  void push(const Px4PoseSample& s) noexcept;
  const Px4PoseSample& sample(std::size_t i) const noexcept;  // i = 0 is the oldest
  Pose4 accumulation_mean() const noexcept;
  AlignmentReason accumulate(const Pair& pair, time::SensorTime t) noexcept;
  AlignmentReason filter(const Pair& pair, time::SensorTime t) noexcept;
  void accept(time::SensorTime t) noexcept;
  std::optional<AlignmentReason> timeouts(time::SensorTime now) noexcept;

  AlignmentConfig cfg_;
  bool cfg_valid_;  // constant after construction; not a state

  AlignmentState state_{AlignmentState::kInit};  // the one and only state

  // Data about recent inputs, not state.
  Pose4 filtered_{};
  std::array<Px4PoseSample, limits::kPx4BufferCapacity> ring_{};
  std::size_t ring_head_{0};  // index of the next write
  std::size_t ring_size_{0};
  struct Counters {
    std::uint8_t xy, z, heading;
  };
  std::optional<Counters> last_counters_;  // empty until the first accepted PX4 sample

  std::array<Pair, limits::kMaxConsistentPairs> acc_{};  // INIT accumulation
  std::uint32_t acc_n_{0};
  Eigen::Vector2d acc_sum_px4_xy_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d acc_sum_lio_xy_{Eigen::Vector2d::Zero()};
  double acc_sum_z_{0.0};  // sum of z_px4 - z_lio
  double acc_sum_sin_{0.0};
  double acc_sum_cos_{0.0};

  std::optional<time::SensorTime> last_lio_;       // newest accepted LIO stamp (any tracking value)
  std::optional<time::SensorTime> last_accepted_;  // LIO stamp of the newest accepted pair
  std::optional<time::SensorTime> newest_pair_;    // LIO stamp of the newest formed pair (D28 stamp)
  time::SensorTime last_seen_{};                   // newest LIO stamp or tick
  time::SensorTime stale_since_{};                 // VALID: max(last accepted pair, entry into VALID)
  time::SensorTime frozen_since_{};                // FROZEN: stamp of the lost sample that froze T
  double residual_position_m_{0.0};
  double residual_yaw_rad_{0.0};
};

}  // namespace uavnav::px4bridge
