#include "uavnav/px4bridge/alignment.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <numbers>

#include "uavnav/px4bridge/config.hpp"

namespace uavnav::px4bridge {

namespace {

/// Shortest-arc representative in [-pi, pi]. std::remainder is exact for every finite input.
double wrap(double a) { return std::remainder(a, 2.0 * std::numbers::pi); }

/// Rz(yaw) * v about the down axis: [c x - s y, s x + c y].
Eigen::Vector2d rz(double yaw, const Eigen::Vector2d& v) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v.x() - s * v.y(), s * v.x() + c * v.y()};
}

Eigen::Vector3d translation(const Pose4& p) { return {p.x_m, p.y_m, p.z_m}; }

bool finite(const Px4PoseSample& s) {
  return s.p_ned_m.allFinite() && std::isfinite(s.yaw_rad) && s.delta_xy_m.allFinite() &&
         std::isfinite(s.delta_z_m) && std::isfinite(s.delta_heading_rad);
}

bool within_spec(std::string_view key, double v) {
  const config::ParamSpec* spec = find_alignment_spec(key);
  return spec != nullptr && std::isfinite(v) && v >= spec->min && v <= spec->max;
}

/// The same bounds load_alignment_config enforces (single source: kAlignmentSpecs, looked up by name), so a
/// hand-built config is held to them too. The cross-field rule is not needed for the estimator's own safety.
bool valid_config(const AlignmentConfig& c) {
  return within_spec("alignment_tau_s", time::to_seconds(c.tau)) &&
         within_spec("alignment_consistent_pairs", static_cast<double>(c.consistent_pairs)) &&
         within_spec("alignment_jump_position_m", c.jump_position_m) &&
         within_spec("alignment_jump_yaw_rad", c.jump_yaw_rad) &&
         within_spec("alignment_valid_stale_s", time::to_seconds(c.valid_stale)) &&
         within_spec("alignment_frozen_max_s", time::to_seconds(c.frozen_max)) && c.tau >= limits::kFilterDtMax;
}

/// Instant T of one pair (header comment): yaw = wrap(yaw_px4 - yaw_lio), t_xy = p_px4_xy - Rz(yaw) p_lio_xy,
/// z = z_px4 - z_lio.
Pose4 instant(const Eigen::Vector3d& p_px4, double yaw_px4, const Eigen::Vector3d& p_lio, double yaw_lio) {
  const double yaw = wrap(yaw_px4 - yaw_lio);
  const Eigen::Vector2d t_xy = p_px4.head<2>() - rz(yaw, p_lio.head<2>());
  return Pose4{t_xy.x(), t_xy.y(), p_px4.z() - p_lio.z(), yaw};
}

/// G(p) = p + (Rz(dh) - I)(p - pivot)_xy + d. With dh == 0, Rz(0) - I is exactly zero, so a translation-only
/// reset adds d exactly.
Eigen::Vector3d frame_change(const Eigen::Vector3d& p, const Eigen::Vector3d& pivot, const Eigen::Vector3d& d,
                             double dh) {
  const Eigen::Vector2d r = p.head<2>() - pivot.head<2>();
  const Eigen::Vector2d rotated_minus_r = rz(dh, r) - r;
  return {p.x() + rotated_minus_r.x() + d.x(), p.y() + rotated_minus_r.y() + d.y(), p.z() + d.z()};
}

/// A pair measured against a reference T at the pair's vehicle point (header comment, D30/O12).
struct AtVehicle {
  Eigen::Vector3d q;  // T(p_lio): where the reference T puts the vehicle
  Eigen::Vector3d e;  // p_px4 - q
  double e_yaw;       // wrap(yaw_inst - yaw_T)
};

AtVehicle at_vehicle(const Pose4& T, const Pose4& inst, const Eigen::Vector3d& p_lio,
                     const Eigen::Vector3d& p_px4) {
  const Eigen::Vector2d q_xy = rz(T.yaw_rad, p_lio.head<2>()) + Eigen::Vector2d{T.x_m, T.y_m};
  const Eigen::Vector3d q{q_xy.x(), q_xy.y(), p_lio.z() + T.z_m};
  return AtVehicle{q, p_px4 - q, wrap(inst.yaw_rad - T.yaw_rad)};
}

}  // namespace

AlignmentEstimator::AlignmentEstimator(const AlignmentConfig& cfg) noexcept
    : cfg_(cfg), cfg_valid_(valid_config(cfg)) {}

// ---- state machine -------------------------------------------------------------------------------------------

bool AlignmentEstimator::go(AlignmentState to, AlignmentReason reason, time::SensorTime t) noexcept {
  bool listed = false;
  for (const AlignmentEdge& e : kAlignmentTransitions) {
    if (e.from == state_ && e.to == to && e.reason == reason) listed = true;
  }
  // Every state change must be a listed edge. An unlisted one is a programming error (code and table
  // drifted apart): it is refused, the state stays as it is, and the caller reports kInputRejected instead
  // of the transition's reason, so the drift is visible in the output and the event log. The assert
  // stops a debug build at the drift; the allow-list fuzz test pins it as unreachable.
  assert(listed && "alignment transition not in kAlignmentTransitions");
  if (!listed) return false;
  state_ = to;
  // Entry actions.
  switch (to) {
    case AlignmentState::kValid: stale_since_ = t; break;
    case AlignmentState::kFrozen: frozen_since_ = t; break;
    case AlignmentState::kInit: restart_accumulation(); break;
    case AlignmentState::kInvalid: break;
  }
  return true;
}

std::optional<AlignmentReason> AlignmentEstimator::timeouts(time::SensorTime now) noexcept {
  // Both stamps are >= 0 (rejected otherwise), so the differences cannot overflow.
  if (state_ == AlignmentState::kValid && now - stale_since_ > cfg_.valid_stale) {
    if (!go(AlignmentState::kInvalid, AlignmentReason::kStale, now)) return AlignmentReason::kInputRejected;
    return AlignmentReason::kStale;
  }
  if (state_ == AlignmentState::kFrozen && now - frozen_since_ > cfg_.frozen_max) {
    if (!go(AlignmentState::kInvalid, AlignmentReason::kFrozenTooLong, now)) return AlignmentReason::kInputRejected;
    return AlignmentReason::kFrozenTooLong;
  }
  return std::nullopt;
}

AlignmentOutput AlignmentEstimator::output(AlignmentReason reason, const std::optional<Pose4>& raw) const noexcept {
  AlignmentOutput o{};
  o.state = state_;
  o.filtered = state_ == AlignmentState::kInit ? accumulation_mean() : filtered_;
  o.raw = raw;
  o.stamp = newest_pair_.value_or(time::SensorTime{});
  o.age_s = last_accepted_ ? std::max(0.0, time::to_seconds(last_seen_ - *last_accepted_)) : 0.0;
  o.residual_position_m = residual_position_m_;
  o.residual_yaw_rad = residual_yaw_rad_;
  o.reason = reason;
  return o;
}

// ---- PX4 buffer --------------------------------------------------------------------------------------------

void AlignmentEstimator::push(const Px4PoseSample& s) noexcept {
  ring_[ring_head_] = s;
  ring_[ring_head_].yaw_rad = wrap(s.yaw_rad);
  ring_head_ = (ring_head_ + 1) % ring_.size();
  ring_size_ = std::min(ring_size_ + 1, ring_.size());
}

const Px4PoseSample& AlignmentEstimator::sample(std::size_t i) const noexcept {
  return ring_[(ring_head_ + ring_.size() - ring_size_ + i) % ring_.size()];
}

std::optional<AlignmentEstimator::Interpolated> AlignmentEstimator::px4_at(time::SensorTime t) const noexcept {
  if (ring_size_ == 0) return std::nullopt;
  const time::SensorTime oldest_usable = sample(ring_size_ - 1).t - limits::kPx4BufferSpan;
  // Newest first: scans are recent, so the bracket is usually found within a few samples.
  for (std::size_t k = ring_size_; k-- > 0;) {
    const Px4PoseSample& a = sample(k);
    if (a.t < oldest_usable) return std::nullopt;
    if (a.t == t) return Interpolated{a.p_ned_m, a.yaw_rad};
    if (a.t > t) continue;
    // a.t < t: a is the sample just before the scan; b (if any) the one just after it.
    if (k + 1 >= ring_size_) return std::nullopt;
    const Px4PoseSample& b = sample(k + 1);
    if (t - a.t > limits::kPairingWindow || b.t - t > limits::kPairingWindow) return std::nullopt;
    const double w = static_cast<double>((t - a.t).ns) / static_cast<double>((b.t - a.t).ns);
    return Interpolated{a.p_ned_m + w * (b.p_ned_m - a.p_ned_m), wrap(a.yaw_rad + w * wrap(b.yaw_rad - a.yaw_rad))};
  }
  return std::nullopt;
}

AlignmentOutput AlignmentEstimator::on_px4(const Px4PoseSample& s) noexcept {
  if (!finite(s) || s.t.ns < 0) return output(AlignmentReason::kInputRejected);
  if (ring_size_ > 0 && s.t <= sample(ring_size_ - 1).t) return output(AlignmentReason::kInputRejected);

  const Counters now{s.xy_reset_counter, s.z_reset_counter, s.heading_reset_counter};
  if (!last_counters_) {
    last_counters_ = now;  // the first sample only initialises the counters
    push(s);
    return output(AlignmentReason::kNone);
  }
  // F13: any difference, by any step (uint8, so 255 -> 0 is a difference too), is a reset of that quantity.
  const bool xy = now.xy != last_counters_->xy;
  const bool z = now.z != last_counters_->z;
  const bool heading = now.heading != last_counters_->heading;
  last_counters_ = now;
  if (!xy && !z && !heading) {
    push(s);
    return output(AlignmentReason::kNone);
  }

  const Eigen::Vector3d d{xy ? s.delta_xy_m.x() : 0.0, xy ? s.delta_xy_m.y() : 0.0, z ? s.delta_z_m : 0.0};
  const double dh = heading ? wrap(s.delta_heading_rad) : 0.0;
  const Eigen::Vector3d pivot = s.p_ned_m - d;  // vehicle position just before the reset, old frame

  for (std::size_t k = 0; k < ring_size_; ++k) {
    Px4PoseSample& b = ring_[(ring_head_ + ring_.size() - ring_size_ + k) % ring_.size()];
    b.p_ned_m = frame_change(b.p_ned_m, pivot, d, dh);
    b.yaw_rad = wrap(b.yaw_rad + dh);
  }
  const Eigen::Vector3d t = frame_change(translation(filtered_), pivot, d, dh);
  filtered_ = Pose4{t.x(), t.y(), t.z(), wrap(filtered_.yaw_rad + dh)};
  push(s);

  switch (state_) {
    case AlignmentState::kFrozen:
      if (!go(AlignmentState::kInvalid, AlignmentReason::kPx4ResetWhileFrozen, s.t)) {
        return output(AlignmentReason::kInputRejected);
      }
      return output(AlignmentReason::kPx4ResetWhileFrozen);
    case AlignmentState::kInit:
      restart_accumulation();
      return output(AlignmentReason::kPx4ResetApplied);
    case AlignmentState::kValid:
    case AlignmentState::kInvalid:
      break;
  }
  return output(AlignmentReason::kPx4ResetApplied);
}

// ---- INIT accumulation -----------------------------------------------------------------------------------------

void AlignmentEstimator::restart_accumulation() noexcept {
  acc_n_ = 0;
  acc_sum_px4_xy_.setZero();
  acc_sum_lio_xy_.setZero();
  acc_sum_z_ = 0.0;
  acc_sum_sin_ = 0.0;
  acc_sum_cos_ = 0.0;
}

void AlignmentEstimator::add_to_accumulation(const Pair& pair) noexcept {
  if (acc_n_ >= acc_.size()) restart_accumulation();  // only reachable with an invalid config
  acc_[acc_n_++] = pair;
  acc_sum_px4_xy_ += pair.p_px4.head<2>();
  acc_sum_lio_xy_ += pair.p_lio.head<2>();
  acc_sum_z_ += pair.inst.z_m;  // z_px4 - z_lio
  acc_sum_sin_ += std::sin(pair.inst.yaw_rad);
  acc_sum_cos_ += std::cos(pair.inst.yaw_rad);
}

Pose4 AlignmentEstimator::accumulation_mean() const noexcept {
  if (acc_n_ == 0) return Pose4{};
  const double n = static_cast<double>(acc_n_);
  // Circular mean: the direction of the summed unit vectors. Members are within jump_yaw_rad (<= 0.5 rad)
  // of each other, so the sum is never near zero.
  const double yaw = std::atan2(acc_sum_sin_, acc_sum_cos_);
  // Least-squares translation given that yaw (header comment): the mean PX4 vehicle position minus the
  // rotated mean LIO vehicle position. With every p_lio = 0 this is the mean of the instant translations.
  const Eigen::Vector2d t_xy = acc_sum_px4_xy_ / n - rz(yaw, acc_sum_lio_xy_ / n);
  return Pose4{t_xy.x(), t_xy.y(), acc_sum_z_ / n, yaw};
}

AlignmentReason AlignmentEstimator::accumulate(const Pair& pair, time::SensorTime t) noexcept {
  // Each pair is measured at its OWN vehicle point (D30/O12, D31).
  auto outside = [this](const Pair& p, const Pose4& mean) {
    const AtVehicle m = at_vehicle(mean, p.inst, p.p_lio, p.p_px4);
    return m.e.norm() > cfg_.jump_position_m || std::abs(m.e_yaw) > cfg_.jump_yaw_rad;
  };
  if (acc_n_ > 0) {
    const Pose4 mean = accumulation_mean();
    const AtVehicle m = at_vehicle(mean, pair.inst, pair.p_lio, pair.p_px4);
    residual_position_m_ = m.e.norm();
    residual_yaw_rad_ = m.e_yaw;
    // A NaN threshold (invalid config) makes every comparison false; cfg_valid_ still blocks VALID below.
    if (outside(pair, mean)) {
      restart_accumulation();
      add_to_accumulation(pair);
      return AlignmentReason::kPairRejectedJump;
    }
  } else {
    residual_position_m_ = 0.0;
    residual_yaw_rad_ = 0.0;
  }
  add_to_accumulation(pair);
  last_accepted_ = t;
  if (!cfg_valid_ || acc_n_ < cfg_.consistent_pairs) return AlignmentReason::kPairAccepted;

  // Every member must be within the thresholds of the FINAL mean, not only of the mean it was added to.
  const Pose4 mean = accumulation_mean();
  for (std::uint32_t i = 0; i < acc_n_; ++i) {
    if (outside(acc_[i], mean)) {
      restart_accumulation();
      add_to_accumulation(pair);
      return AlignmentReason::kPairRejectedJump;
    }
  }
  if (!go(AlignmentState::kValid, AlignmentReason::kPairsConsistent, t)) return AlignmentReason::kInputRejected;
  filtered_ = mean;
  return AlignmentReason::kPairsConsistent;
}

// ---- VALID filter ------------------------------------------------------------------------------------------

AlignmentReason AlignmentEstimator::filter(const Pair& pair, time::SensorTime t) noexcept {
  // Residual and gate at the vehicle point q = T(p_lio) (D30/O12), against the filtered T before the step.
  const AtVehicle m = at_vehicle(filtered_, pair.inst, pair.p_lio, pair.p_px4);
  residual_position_m_ = m.e.norm();
  residual_yaw_rad_ = m.e_yaw;
  if (residual_position_m_ > cfg_.jump_position_m || std::abs(m.e_yaw) > cfg_.jump_yaw_rad) {
    return AlignmentReason::kPairRejectedJump;  // T unchanged
  }
  // dt in LIO sensor time since the previous accepted pair (always set in VALID), clamped (limits.hpp).
  const time::Duration raw_dt = last_accepted_ ? t - *last_accepted_ : limits::kFilterDtMax;
  const double dt = time::to_seconds(std::clamp(raw_dt, limits::kFilterDtMin, limits::kFilterDtMax));
  const double alpha = dt / time::to_seconds(cfg_.tau);  // <= 1 (tau >= kFilterDtMax)

  // One step = the frame change G with pivot q, rotation dh and shift alpha e: yaw_T += dh,
  // t <- Rz(dh)(t - q) + q + alpha e, so T'(p_lio) = q + alpha e (the yaw part rotates about the vehicle).
  // No separate rate limiter (D30): |e| <= jump_position_m and |e_yaw| <= jump_yaw_rad already bound a step.
  const double dh = alpha * m.e_yaw;
  const Eigen::Vector3d new_t = frame_change(translation(filtered_), m.q, alpha * m.e, dh);
  filtered_ = Pose4{new_t.x(), new_t.y(), new_t.z(), wrap(filtered_.yaw_rad + dh)};
  last_accepted_ = t;
  stale_since_ = t;
  return AlignmentReason::kPairAccepted;
}

// ---- LIO and tick ------------------------------------------------------------------------------------------

AlignmentOutput AlignmentEstimator::on_lio(const LioPoseSample& s) noexcept {
  if (s.t.ns < 0 || (last_lio_ && s.t <= *last_lio_)) return output(AlignmentReason::kInputRejected);
  if (s.tracking && !(s.p_frd_m.allFinite() && std::isfinite(s.yaw_frd_rad))) {
    return output(AlignmentReason::kInputRejected);
  }
  last_lio_ = s.t;
  last_seen_ = std::max(last_seen_, s.t);

  if (const auto r = timeouts(s.t)) return output(*r);

  if (!s.tracking) {
    switch (state_) {
      case AlignmentState::kValid:
        if (!go(AlignmentState::kFrozen, AlignmentReason::kLioLost, s.t)) {
          return output(AlignmentReason::kInputRejected);
        }
        return output(AlignmentReason::kLioLost);
      case AlignmentState::kInit:
        restart_accumulation();
        return output(AlignmentReason::kLioLost);
      case AlignmentState::kFrozen:
      case AlignmentState::kInvalid:
        break;
    }
    return output(AlignmentReason::kNone);
  }

  if (state_ == AlignmentState::kInvalid) {
    // The new accumulation starts with the next pair.
    if (!go(AlignmentState::kInit, AlignmentReason::kLioTracking, s.t)) return output(AlignmentReason::kInputRejected);
    return output(AlignmentReason::kLioTracking);
  }
  std::optional<AlignmentReason> transition;
  if (state_ == AlignmentState::kFrozen) {
    if (!go(AlignmentState::kValid, AlignmentReason::kLioTracking, s.t)) return output(AlignmentReason::kInputRejected);
    transition = AlignmentReason::kLioTracking;
  }

  const auto px4 = px4_at(s.t);
  if (!px4) return output(transition.value_or(AlignmentReason::kNoPx4Sample));
  const Pair pair{instant(px4->p, px4->yaw, s.p_frd_m, s.yaw_frd_rad), s.p_frd_m, px4->p};
  newest_pair_ = s.t;
  const AlignmentReason r = state_ == AlignmentState::kInit ? accumulate(pair, s.t) : filter(pair, s.t);
  return output(transition.value_or(r), pair.inst);
}

AlignmentOutput AlignmentEstimator::on_tick(time::SensorTime now) noexcept {
  if (now < last_seen_) return output(AlignmentReason::kNone);
  last_seen_ = now;
  if (const auto r = timeouts(now)) return output(*r);
  return output(AlignmentReason::kNone);
}

}  // namespace uavnav::px4bridge
