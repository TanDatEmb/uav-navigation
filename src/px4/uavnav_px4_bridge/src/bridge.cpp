#include "uavnav/px4bridge/bridge.hpp"

#include <cmath>
#include <limits>

#include "uavnav/core/angles.hpp"
#include "uavnav/px4bridge/config.hpp"
#include "uavnav/px4bridge/frames.hpp"
#include "uavnav/px4bridge/limits.hpp"

namespace uavnav::px4bridge {

namespace {

using events::Component;
using events::EventBuilder;

// The tick resolves every timeout at most one period late (limits.hpp kTickPeriod).
static_assert(find_alignment_spec("alignment_valid_stale_s")->min * 1e9 >= 2.0 * limits::kTickPeriod.ns);
static_assert(find_alignment_spec("alignment_frozen_max_s")->min * 1e9 >= 2.0 * limits::kTickPeriod.ns);

template <class E>
constexpr std::size_t slot(E e) noexcept {
  return static_cast<std::size_t>(e);
}

/// Yaw about the FRD down axis of a FLU q_world_body, or NaN when q is not a rotation (non-finite, or a norm
/// beyond the encoder's kQuaternionNormTolerance): the alignment then refuses the sample (kInputRejected)
/// instead of reading a zero quaternion as yaw 0.
double frd_yaw(const Eigen::Quaterniond& q_flu) noexcept {
  const double n = q_flu.coeffs().norm();
  if (!(std::abs(n - 1.0) <= kQuaternionNormTolerance)) return std::numeric_limits<double>::quiet_NaN();
  const Eigen::Matrix3d r = flu_to_frd(Eigen::Quaterniond(q_flu.coeffs() / n)).toRotationMatrix();
  return angles::yaw_from_rotation(r(1, 0), r(0, 0));
}

bool is_pair_rejection(AlignmentReason r) noexcept {
  return r == AlignmentReason::kPairRejectedJump || r == AlignmentReason::kNoPx4Sample ||
         r == AlignmentReason::kInputRejected;
}

bool is_reset(AlignmentReason r) noexcept {
  return r == AlignmentReason::kPx4ResetApplied || r == AlignmentReason::kPx4ResetWhileFrozen;
}

double step(std::uint8_t now, std::uint8_t before) noexcept {
  return static_cast<double>(static_cast<std::uint8_t>(now - before));  // modulo 256 (F13: any step, any wrap)
}

}  // namespace

Px4Bridge::Px4Bridge(const AlignmentConfig& cfg, events::EventRecorder& events) noexcept
    : events_(events), alignment_(cfg) {
  static_assert(slot(EvReason::kBadQuaternion) < std::tuple_size_v<decltype(ev_rejected_)>);
  static_assert(slot(AlignmentReason::kInputRejected) < std::tuple_size_v<decltype(pair_rejected_)>);
  static_assert(slot(BridgeReason::kNotFinite) < std::tuple_size_v<decltype(px4_rejected_)>);
}

// ---- inputs ------------------------------------------------------------------------------------------------

BridgeOutputs Px4Bridge::on_lio_odometry(const LioOdometryIn& in, const time::TimeSnapshot& now) {
  if (in_future(in.t)) {
    refuse_lio_future(Input::kOdometry, in.t, in.epoch, now);
    return {};
  }
  lio_epoch_ = in.epoch;

  // EV gate (O22): the newest health of the same epoch is not TRACKING and not older than this sample.
  if (latest_health_ && latest_health_->epoch == in.epoch && latest_health_->state != LioHealthCode::kTracking &&
      latest_health_->t >= in.t) {
    if (due(ev_gated_[0])) {
      emit(event("EvGated", in.epoch, now)
               .reason(BridgeReason::kHealthNotTracking)
               .value("odometry_time_s", time::to_seconds(in.t))
               .value("health_time_s", time::to_seconds(latest_health_->t))
               .value("health_state", static_cast<double>(static_cast<std::uint8_t>(latest_health_->state)))
               .value("count", static_cast<double>(ev_gated_[0])));
    }
    return {};
  }

  BridgeOutputs out;
  // /lio/odometry is published only in TRACKING, so the sample is a tracking one (F14: its own epoch).
  const EvInput ev_in{in.t, in.epoch, true, in.p_flu_m, in.q_flu, in.v_flu_mps, in.pose_cov, in.vel_cov, in.quality};
  if (auto ev = encode_ev(ev_in, limits::kClockMode)) {
    out.ev = *ev;
  } else if (due(ev_rejected_[slot(ev.error())])) {
    emit(event("EvRejected", in.epoch, now)
             .reason(ev.error())
             .value("sensor_time_s", time::to_seconds(in.t))
             .value("count", static_cast<double>(ev_rejected_[slot(ev.error())])));
  }

  const AlignmentState before = alignment_.state();
  const LioPoseSample pose{in.t, flu_to_frd(in.p_flu_m), frd_yaw(in.q_flu), true};
  out.alignment = report(before, alignment_.on_lio(pose), Input::kOdometry, in.t, in.epoch, now);
  return out;
}

BridgeOutputs Px4Bridge::on_lio_health(const LioHealthIn& in, const time::TimeSnapshot& now) {
  if (in_future(in.t)) {
    refuse_lio_future(Input::kHealth, in.t, in.epoch, now);
    return {};
  }
  lio_epoch_ = in.epoch;
  latest_health_ = in;
  if (in.state == LioHealthCode::kTracking) return {};

  // Every non-TRACKING health (an unknown code too) freezes T (O22); the pose of a lost sample is never used.
  const AlignmentState before = alignment_.state();
  const LioPoseSample lost{in.t, Eigen::Vector3d::Zero(), 0.0, false};
  return BridgeOutputs{std::nullopt,
                       report(before, alignment_.on_lio(lost), Input::kHealth, in.t, in.epoch, now)};
}

BridgeOutputs Px4Bridge::on_px4_local_position(const Px4LocalPositionIn& in, const time::TimeSnapshot& now) {
  if (in_future(in.sample.t)) {
    refuse_px4(BridgeReason::kStampInFuture, in, now);
    return {};
  }
  if (!in.xy_valid || !in.z_valid) {
    refuse_px4(BridgeReason::kPx4PositionInvalid, in, now);
    return {};
  }
  const AlignmentState before = alignment_.state();
  const AlignmentOutput out = alignment_.on_px4(in.sample);
  if (is_reset(out.reason)) report_reset(before, out, in.sample, now);
  if (out.reason != AlignmentReason::kInputRejected) {
    px4_counters_ = Counters{in.sample.xy_reset_counter, in.sample.z_reset_counter, in.sample.heading_reset_counter};
  }
  report(before, out, Input::kPx4, in.sample.t, lio_epoch_, now);
  if (out.reason == AlignmentReason::kNone) return {};  // O21: on_px4 is published only with a reason
  return BridgeOutputs{std::nullopt, out};
}

BridgeOutputs Px4Bridge::on_tick(time::SensorTime now_sensor, const time::TimeSnapshot& now) {
  if (!newest_tick_ || now_sensor > *newest_tick_) newest_tick_ = now_sensor;
  const AlignmentState before = alignment_.state();
  return BridgeOutputs{std::nullopt,
                       report(before, alignment_.on_tick(now_sensor), Input::kTick, now_sensor, lio_epoch_, now)};
}

// ---- private -----------------------------------------------------------------------------------------------

bool Px4Bridge::in_future(time::SensorTime t) const noexcept {
  // Both stamps come from the range-checked converters (<= kMaxStampSeconds), so adding 0.5 s cannot overflow.
  return newest_tick_ && t > *newest_tick_ + limits::kMaxStampAhead;
}

AlignmentOutput Px4Bridge::report(AlignmentState before, const AlignmentOutput& out, Input input,
                                  time::SensorTime t, std::uint32_t epoch, const time::TimeSnapshot& now) noexcept {
  // Keyed on the state change, not on the reason: FROZEN -> VALID may report kPairRejectedJump (alignment.hpp).
  if (out.state != before) {
    emit(event("AlignmentTransition", epoch, now)
             .states(before, out.state)
             .reason(out.reason)
             .value("input", static_cast<double>(input))
             .value("sensor_time_s", time::to_seconds(t))
             .value("x_m", out.filtered.x_m)
             .value("y_m", out.filtered.y_m)
             .value("z_m", out.filtered.z_m)
             .value("yaw_rad", out.filtered.yaw_rad)
             .value("age_s", out.age_s)
             .value("residual_position_m", out.residual_position_m)
             .value("residual_yaw_rad", out.residual_yaw_rad));
  }
  if (is_pair_rejection(out.reason) && due(pair_rejected_[slot(out.reason)])) {
    emit(event("AlignmentPairRejected", epoch, now)
             .states(before, out.state)
             .reason(out.reason)
             .value("input", static_cast<double>(input))
             .value("sensor_time_s", time::to_seconds(t))
             .value("residual_position_m", out.residual_position_m)
             .value("residual_yaw_rad", out.residual_yaw_rad)
             .value("count", static_cast<double>(pair_rejected_[slot(out.reason)])));
  }
  return out;
}

void Px4Bridge::report_reset(AlignmentState before, const AlignmentOutput& out, const Px4PoseSample& s,
                             const time::TimeSnapshot& now) noexcept {
  // px4_counters_ is set: a reset needs a previous accepted sample (the first one only initialises).
  const Counters prev = px4_counters_.value_or(Counters{s.xy_reset_counter, s.z_reset_counter,
                                                        s.heading_reset_counter});
  emit(event("Px4ResetApplied", lio_epoch_, now)
           .states(before, out.state)
           .reason(out.reason)
           .value("sensor_time_s", time::to_seconds(s.t))
           .value("xy_reset_counter", s.xy_reset_counter)
           .value("z_reset_counter", s.z_reset_counter)
           .value("heading_reset_counter", s.heading_reset_counter)
           .value("xy_reset_step", step(s.xy_reset_counter, prev.xy))
           .value("z_reset_step", step(s.z_reset_counter, prev.z))
           .value("heading_reset_step", step(s.heading_reset_counter, prev.heading))
           .value("delta_x_m", s.delta_xy_m.x())
           .value("delta_y_m", s.delta_xy_m.y())
           .value("delta_z_m", s.delta_z_m)
           .value("delta_heading_rad", s.delta_heading_rad)
           .value("x_m", out.filtered.x_m)
           .value("y_m", out.filtered.y_m)
           .value("yaw_rad", out.filtered.yaw_rad));
}

void Px4Bridge::refuse_lio_future(Input input, time::SensorTime t, std::uint32_t epoch,
                                  const time::TimeSnapshot& now) noexcept {
  if (!due(lio_future_[0])) return;
  emit(event("LioInputRejected", epoch, now)
           .reason(BridgeReason::kStampInFuture)
           .value("stream", static_cast<double>(input))
           .value("sensor_time_s", time::to_seconds(t))
           .value("newest_tick_s", time::to_seconds(*newest_tick_))
           .value("count", static_cast<double>(lio_future_[0])));
}

void Px4Bridge::refuse_px4(BridgeReason why, const Px4LocalPositionIn& in, const time::TimeSnapshot& now) noexcept {
  std::uint64_t& count = px4_rejected_[slot(why)];
  if (!due(count)) return;
  EventBuilder b = event("Px4SampleRejected", lio_epoch_, now);
  b.reason(why).value("sensor_time_s", time::to_seconds(in.sample.t));
  if (why == BridgeReason::kStampInFuture) {
    b.value("newest_tick_s", time::to_seconds(*newest_tick_));
  } else {
    b.value("xy_valid", in.xy_valid ? 1.0 : 0.0).value("z_valid", in.z_valid ? 1.0 : 0.0);
  }
  emit(b.value("count", static_cast<double>(count)));
}

events::EventBuilder Px4Bridge::event(std::string_view name, std::uint32_t epoch,
                                      const time::TimeSnapshot& now) const noexcept {
  events::EventIdentity id;
  id.lio_epoch = epoch;
  return EventBuilder(Component::kPx4Bridge, name, now).identity(id);
}

void Px4Bridge::emit(const events::EventBuilder& b) noexcept { (void)events_.emit(b.build()); }

bool Px4Bridge::due(std::uint64_t& count) noexcept {
  ++count;
  return count == 1 || count % limits::kEventSummaryEvery == 0;
}

}  // namespace uavnav::px4bridge
