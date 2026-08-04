#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include "odometry_supervisor/supervisor_types.hpp"

namespace odometry_supervisor {

// This component deliberately has no ROS dependency.  A candidate is the
// result of one bounded exact-time estimator window; the manager decides
// whether enough independent evidence exists to make it public.
struct AlignmentCandidateObservation {
  WorldAlignment alignment;
  std::uint64_t lio_generation{0};
  std::uint64_t frame_generation{0};
  std::uint64_t time_generation{0};
  std::uint64_t evidence_id{0};
  std::size_t novel_pair_count{0};
};

struct AlignmentRevalidationObservation {
  bool exact_time_pair_valid{false};
  Residual residual;
  bool covariance_available{false};
  double nis{0.0};
  std::int64_t epoch_ns{0};
  std::uint64_t lio_generation{0};
  std::uint64_t frame_generation{0};
  std::uint64_t time_generation{0};
  std::uint64_t evidence_id{0};
};

struct AlignmentLifecycleConfig {
  std::size_t stable_candidate_estimates{3};
  std::size_t minimum_novel_pairs{4};
  std::size_t candidate_history_capacity{8};
  double max_translation_step_m{0.10};
  double max_yaw_step_rad{0.05};
  double max_cluster_translation_m{0.15};
  double max_cluster_yaw_rad{0.10};
  // 95% chi-square quantile for four coupled translation/yaw deltas.
  // The absolute translation/yaw caps above remain active even when the
  // covariance is large.
  double covariance_nis_chi_square{9.487729};
  std::size_t revalidation_samples{3};
  std::size_t revalidation_failure_limit{3};
  ResidualThresholds revalidation_residual{0.75, 0.75, 0.5235987756, 0.3490658504};
  double revalidation_covariance_nis_chi_square{9.487729};
};

struct AlignmentLifecycleSnapshot {
  AlignmentLifecycleState state{AlignmentLifecycleState::kUnaligned};
  std::optional<WorldAlignment> candidate_alignment;
  std::optional<WorldAlignment> locked_alignment;
  std::size_t stable_candidate_count{0};
  std::size_t candidate_estimate_count{0};
  std::size_t candidate_transition_count{0};
  std::size_t accumulated_novel_pair_count{0};
  std::size_t revalidation_sample_count{0};
  std::size_t revalidation_success_count{0};
  std::size_t revalidation_failure_count{0};
  std::size_t revalidation_start_count{0};
  std::size_t lock_count{0};
  std::int64_t revalidation_start_epoch_ns{0};
  std::int64_t locked_transform_age_ns{-1};
  std::uint64_t lio_generation{0};
  std::uint64_t frame_generation{0};
  std::uint64_t time_generation{0};
  std::uint64_t reset_event_generation{0};
  std::string rejection_reason;
};

class AlignmentLifecycleManager {
 public:
  explicit AlignmentLifecycleManager(AlignmentLifecycleConfig config = {});

  void reset();
  void invalidate(const std::string& reason);
  void rejectCandidate(const std::string& reason);
  // A transport/service failure does not constitute geometric evidence.  It
  // records the reason while preserving a locked transform and revalidation
  // counters.
  void observeTransportFailure(const std::string& reason);

  // A public frame change means the old transform no longer has a defined
  // geometric meaning.  Time/LIO changes clear the candidate proof; the
  // frozen transform is retained for explicit revalidation until a public
  // frame change or persistent validation failure.
  void observeBindingGeneration(std::uint64_t lio_generation,
                                std::uint64_t frame_generation,
                                std::uint64_t time_generation);

  // Reset events are intentionally separate from frame generation.  A
  // compensated reset enters revalidation without changing the transform.
  void observeResetEvent(std::uint64_t reset_event_generation,
                         std::int64_t epoch_ns,
                         bool compensated);

  void beginRevalidation(const std::string& reason, std::int64_t epoch_ns);

  // Returns true when the candidate is accepted into the bounded proof
  // history, and false when it was rejected.  A rejected candidate never
  // mutates locked_alignment.
  bool observeCandidate(const AlignmentCandidateObservation& observation);

  // Revalidation uses the frozen locked transform and never adapts it.
  bool observeRevalidation(const AlignmentRevalidationObservation& observation);

  [[nodiscard]] AlignmentLifecycleSnapshot snapshot(std::int64_t now_epoch_ns = 0) const;
  [[nodiscard]] bool candidateValid() const noexcept;
  [[nodiscard]] bool locked() const noexcept;
  [[nodiscard]] bool revalidating() const noexcept;
  [[nodiscard]] const std::optional<WorldAlignment>& candidateAlignment() const noexcept {
    return candidate_alignment_;
  }
  [[nodiscard]] const std::optional<WorldAlignment>& lockedAlignment() const noexcept {
    return locked_alignment_;
  }
  [[nodiscard]] AlignmentLifecycleState state() const noexcept { return state_; }

 private:
  struct CandidateRecord {
    WorldAlignment alignment;
    std::uint64_t evidence_id{0};
    std::size_t novel_pair_count{0};
  };

  void clearCandidateProof();
  void invalidateLocked(const std::string& reason);
  bool passesProof(const WorldAlignment& candidate) const;
  static bool covarianceValid(const Eigen::Matrix4d& covariance);
  bool covarianceNisAcceptable(const WorldAlignment& a,
                               const WorldAlignment& b) const;
  bool residualPasses(const AlignmentRevalidationObservation& observation) const;
  static double wrappedYawDelta(double lhs, double rhs) noexcept;

  AlignmentLifecycleConfig config_;
  AlignmentLifecycleState state_{AlignmentLifecycleState::kUnaligned};
  std::optional<WorldAlignment> candidate_alignment_;
  std::optional<WorldAlignment> locked_alignment_;
  std::deque<CandidateRecord> candidate_history_;
  std::size_t stable_candidate_count_{0};
  std::size_t candidate_estimate_count_{0};
  std::size_t candidate_transition_count_{0};
  std::size_t accumulated_novel_pair_count_{0};
  std::uint64_t last_evidence_id_{0};
  std::size_t revalidation_sample_count_{0};
  std::size_t revalidation_success_count_{0};
  std::size_t revalidation_failure_count_{0};
  std::size_t revalidation_start_count_{0};
  std::size_t lock_count_{0};
  std::int64_t revalidation_start_epoch_ns_{0};
  std::uint64_t lio_generation_{0};
  std::uint64_t frame_generation_{0};
  std::uint64_t time_generation_{0};
  std::uint64_t reset_event_generation_{0};
  std::int64_t last_revalidation_epoch_ns_{0};
  std::uint64_t last_revalidation_evidence_id_{0};
  std::string rejection_reason_;
};

}  // namespace odometry_supervisor
