#include "odometry_supervisor/alignment_lifecycle_manager.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Cholesky>

namespace odometry_supervisor {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

bool finiteCovariance(const Eigen::Matrix4d& covariance) {
  return covariance.allFinite();
}
}  // namespace

AlignmentLifecycleManager::AlignmentLifecycleManager(AlignmentLifecycleConfig config)
    : config_(std::move(config)) {
  if (config_.stable_candidate_estimates == 0) config_.stable_candidate_estimates = 1;
  if (config_.minimum_novel_pairs == 0) config_.minimum_novel_pairs = 1;
  if (config_.candidate_history_capacity < config_.stable_candidate_estimates) {
    config_.candidate_history_capacity = config_.stable_candidate_estimates;
  }
  if (config_.revalidation_samples == 0) config_.revalidation_samples = 1;
  if (config_.revalidation_failure_limit == 0) config_.revalidation_failure_limit = 1;
}

void AlignmentLifecycleManager::reset() {
  state_ = AlignmentLifecycleState::kUnaligned;
  candidate_alignment_.reset();
  locked_alignment_.reset();
  clearCandidateProof();
  revalidation_sample_count_ = 0;
  revalidation_success_count_ = 0;
  revalidation_failure_count_ = 0;
  revalidation_start_count_ = 0;
  lock_count_ = 0;
  revalidation_start_epoch_ns_ = 0;
  lio_generation_ = 0;
  frame_generation_ = 0;
  time_generation_ = 0;
  reset_event_generation_ = 0;
  rejection_reason_.clear();
}

void AlignmentLifecycleManager::invalidate(const std::string& reason) {
  locked_alignment_.reset();
  clearCandidateProof();
  state_ = AlignmentLifecycleState::kInvalid;
  rejection_reason_ = reason;
}

void AlignmentLifecycleManager::rejectCandidate(const std::string& reason) {
  if (!locked_alignment_) {
    clearCandidateProof();
    state_ = AlignmentLifecycleState::kCollecting;
  }
  rejection_reason_ = reason;
}

void AlignmentLifecycleManager::clearCandidateProof() {
  candidate_alignment_.reset();
  candidate_history_.clear();
  stable_candidate_count_ = 0;
  candidate_estimate_count_ = 0;
  candidate_transition_count_ = 0;
  accumulated_novel_pair_count_ = 0;
  last_evidence_id_ = 0;
}

void AlignmentLifecycleManager::observeBindingGeneration(std::uint64_t lio_generation,
                                                         std::uint64_t frame_generation,
                                                         std::uint64_t time_generation) {
  const bool initialized = frame_generation_ != 0 || lio_generation_ != 0 || time_generation_ != 0;
  const bool frame_changed = initialized && frame_generation != frame_generation_;
  const bool binding_changed = initialized &&
                                (lio_generation != lio_generation_ ||
                                 time_generation != time_generation_);
  lio_generation_ = lio_generation;
  frame_generation_ = frame_generation;
  time_generation_ = time_generation;
  if (frame_changed) {
    candidate_alignment_.reset();
    locked_alignment_.reset();
    clearCandidateProof();
    state_ = AlignmentLifecycleState::kInvalid;
    rejection_reason_ = "public frame generation changed";
    return;
  }
  if (binding_changed) {
    clearCandidateProof();
    if (locked_alignment_) {
      beginRevalidation("alignment binding generation changed", 0);
    } else {
      state_ = AlignmentLifecycleState::kCollecting;
      rejection_reason_ = "alignment binding generation changed";
    }
  }
}

void AlignmentLifecycleManager::observeResetEvent(std::uint64_t reset_event_generation,
                                                  std::int64_t epoch_ns,
                                                  bool compensated) {
  if (reset_event_generation == reset_event_generation_) return;
  reset_event_generation_ = reset_event_generation;
  if (compensated && locked_alignment_) {
    beginRevalidation("compensated PX4 reset", epoch_ns);
  } else if (!locked_alignment_) {
    clearCandidateProof();
    state_ = AlignmentLifecycleState::kCollecting;
    rejection_reason_ = compensated ? "compensated reset before alignment lock"
                                    : "uncompensated reset requires new frame";
  }
}

void AlignmentLifecycleManager::beginRevalidation(const std::string& reason,
                                                   std::int64_t epoch_ns) {
  if (!locked_alignment_) {
    state_ = AlignmentLifecycleState::kCollecting;
    rejection_reason_ = reason;
    return;
  }
  state_ = AlignmentLifecycleState::kRevalidating;
  clearCandidateProof();
  revalidation_sample_count_ = 0;
  revalidation_success_count_ = 0;
  revalidation_failure_count_ = 0;
  ++revalidation_start_count_;
  revalidation_start_epoch_ns_ = epoch_ns;
  rejection_reason_ = reason;
}

double AlignmentLifecycleManager::wrappedYawDelta(double lhs, double rhs) noexcept {
  double delta = lhs - rhs;
  while (delta > kPi) delta -= 2.0 * kPi;
  while (delta < -kPi) delta += 2.0 * kPi;
  return delta;
}

bool AlignmentLifecycleManager::covarianceNisAcceptable(const WorldAlignment& a,
                                                        const WorldAlignment& b) const {
  if (!finiteCovariance(a.covariance) || !finiteCovariance(b.covariance)) return false;
  Eigen::Vector4d delta;
  delta.head<3>() = a.target_from_source_translation - b.target_from_source_translation;
  delta[3] = wrappedYawDelta(a.yaw_rad, b.yaw_rad);
  Eigen::Matrix4d covariance = a.covariance + b.covariance;
  covariance = (covariance + covariance.transpose()) * 0.5;
  Eigen::LDLT<Eigen::Matrix4d> factor(covariance);
  if (factor.info() != Eigen::Success) return false;
  const Eigen::Vector4d solved = factor.solve(delta);
  if (factor.info() != Eigen::Success || !solved.allFinite()) return false;
  const double nis = delta.dot(solved);
  return std::isfinite(nis) && nis >= 0.0 && nis <= config_.covariance_nis_chi_square;
}

bool AlignmentLifecycleManager::passesProof(const WorldAlignment& candidate) const {
  if (candidate_history_.empty()) return true;
  const auto& previous = candidate_history_.back().alignment;
  if ((candidate.target_from_source_translation - previous.target_from_source_translation).norm() >
      config_.max_translation_step_m) return false;
  if (std::abs(wrappedYawDelta(candidate.yaw_rad, previous.yaw_rad)) >
      config_.max_yaw_step_rad) return false;
  for (const auto& record : candidate_history_) {
    const auto& prior = record.alignment;
    if ((candidate.target_from_source_translation - prior.target_from_source_translation).norm() >
            config_.max_cluster_translation_m ||
        std::abs(wrappedYawDelta(candidate.yaw_rad, prior.yaw_rad)) >
            config_.max_cluster_yaw_rad ||
        !covarianceNisAcceptable(candidate, prior)) {
      return false;
    }
  }
  return true;
}

bool AlignmentLifecycleManager::observeCandidate(
    const AlignmentCandidateObservation& observation) {
  observeBindingGeneration(observation.lio_generation, observation.frame_generation,
                           observation.time_generation);
  if (!observation.alignment.valid || observation.evidence_id == 0 ||
      observation.novel_pair_count == 0) {
    rejection_reason_ = "candidate lacks valid novel exact-time evidence";
    if (!locked_alignment_) state_ = AlignmentLifecycleState::kCollecting;
    return false;
  }
  if (state_ == AlignmentLifecycleState::kLocked ||
      state_ == AlignmentLifecycleState::kRevalidating) {
    rejection_reason_ = state_ == AlignmentLifecycleState::kLocked
                            ? "locked transform is frozen"
                            : "candidate adaptation is disabled during revalidation";
    return false;
  }
  if (observation.evidence_id <= last_evidence_id_) {
    rejection_reason_ = "candidate evidence overlaps without a new pair";
    return false;
  }
  last_evidence_id_ = observation.evidence_id;
  accumulated_novel_pair_count_ += observation.novel_pair_count;
  if (accumulated_novel_pair_count_ < config_.minimum_novel_pairs) {
    rejection_reason_ = "waiting for novel exact-time pairs";
    return false;
  }
  accumulated_novel_pair_count_ = 0;
  ++candidate_estimate_count_;
  if (!passesProof(observation.alignment)) {
    candidate_history_.clear();
    stable_candidate_count_ = 0;
    ++candidate_transition_count_;
    candidate_alignment_ = observation.alignment;
    candidate_history_.push_back(
        {observation.alignment, observation.evidence_id, observation.novel_pair_count});
    stable_candidate_count_ = 1;
    state_ = AlignmentLifecycleState::kProvisional;
    rejection_reason_ = "candidate stability proof reset";
    return false;
  }
  candidate_alignment_ = observation.alignment;
  candidate_history_.push_back(
      {observation.alignment, observation.evidence_id, observation.novel_pair_count});
  while (candidate_history_.size() > config_.candidate_history_capacity) {
    candidate_history_.pop_front();
  }
  stable_candidate_count_ = candidate_history_.size();
  state_ = AlignmentLifecycleState::kProvisional;
  rejection_reason_.clear();
  if (stable_candidate_count_ >= config_.stable_candidate_estimates) {
    locked_alignment_ = observation.alignment;
    state_ = AlignmentLifecycleState::kLocked;
    ++lock_count_;
    rejection_reason_.clear();
  }
  return true;
}

bool AlignmentLifecycleManager::observeRevalidation(
    const AlignmentRevalidationObservation& observation) {
  if (state_ != AlignmentLifecycleState::kRevalidating || !locked_alignment_) return false;
  ++revalidation_sample_count_;
  const bool generation_matches = observation.lio_generation == lio_generation_ &&
                                  observation.frame_generation == frame_generation_ &&
                                  observation.time_generation == time_generation_;
  if (observation.exact_time_pair_valid && observation.residual_valid && generation_matches) {
    ++revalidation_success_count_;
    rejection_reason_.clear();
    if (revalidation_success_count_ >= config_.revalidation_samples) {
      state_ = AlignmentLifecycleState::kLocked;
      revalidation_failure_count_ = 0;
      rejection_reason_.clear();
      return true;
    }
    return false;
  }
  ++revalidation_failure_count_;
  rejection_reason_ = "frozen alignment revalidation failed";
  if (revalidation_failure_count_ >= config_.revalidation_failure_limit) {
    invalidateLocked(rejection_reason_);
  }
  return false;
}

void AlignmentLifecycleManager::invalidateLocked(const std::string& reason) {
  locked_alignment_.reset();
  clearCandidateProof();
  state_ = AlignmentLifecycleState::kInvalid;
  rejection_reason_ = reason;
}

bool AlignmentLifecycleManager::candidateValid() const noexcept {
  return candidate_alignment_.has_value() && candidate_alignment_->valid;
}

bool AlignmentLifecycleManager::locked() const noexcept {
  return state_ == AlignmentLifecycleState::kLocked && locked_alignment_.has_value();
}

bool AlignmentLifecycleManager::revalidating() const noexcept {
  return state_ == AlignmentLifecycleState::kRevalidating && locked_alignment_.has_value();
}

AlignmentLifecycleSnapshot AlignmentLifecycleManager::snapshot(std::int64_t now_epoch_ns) const {
  AlignmentLifecycleSnapshot result;
  result.state = state_;
  result.candidate_alignment = candidate_alignment_;
  result.locked_alignment = locked_alignment_;
  result.stable_candidate_count = stable_candidate_count_;
  result.candidate_estimate_count = candidate_estimate_count_;
  result.candidate_transition_count = candidate_transition_count_;
  result.accumulated_novel_pair_count = accumulated_novel_pair_count_;
  result.revalidation_sample_count = revalidation_sample_count_;
  result.revalidation_success_count = revalidation_success_count_;
  result.revalidation_failure_count = revalidation_failure_count_;
  result.revalidation_start_count = revalidation_start_count_;
  result.lock_count = lock_count_;
  result.revalidation_start_epoch_ns = revalidation_start_epoch_ns_;
  result.lio_generation = lio_generation_;
  result.frame_generation = frame_generation_;
  result.time_generation = time_generation_;
  result.reset_event_generation = reset_event_generation_;
  if (locked_alignment_ && now_epoch_ns >= locked_alignment_->epoch_ns) {
    result.locked_transform_age_ns = now_epoch_ns - locked_alignment_->epoch_ns;
  }
  result.rejection_reason = rejection_reason_;
  return result;
}

}  // namespace odometry_supervisor
