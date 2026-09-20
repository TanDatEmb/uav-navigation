#pragma once

#include <functional>
#include <memory>

#include <navigation_world_model/world_model_view.hpp>

namespace navigation_world_model {

struct WorldValidationLease {
  WorldModelViewPtr view;
  WorldSnapshotIdentity identity;

  [[nodiscard]] explicit operator bool() const noexcept {
    return static_cast<bool>(view);
  }
};

enum class WorldCommitDecision {
  kNotAttempted,
  kCommitted,
  kNoPublishedWorld,
  kWorldAdvanced,
  // A dependent transaction was prepared from an older execution timeline.
  // The immutable world must not be published by that attempt, and callers
  // must preserve the newer execution state instead of invalidating it.
  kSuperseded,
  kCancelled,
  kCandidateRejected,
};

// Product boundary used to linearize an already-built, already-validated
// command with immutable WorldModel publication. Implementations must not hold
// their publication gate while the candidate is built or swept.
class WorldCommitAuthorizer {
 public:
  virtual ~WorldCommitAuthorizer() = default;

  [[nodiscard]] virtual WorldValidationLease latest() const noexcept = 0;
  virtual WorldCommitDecision commitIfCurrent(
      const WorldSnapshotIdentity& validated_identity,
      const std::function<bool()>& final_commit) = 0;

  // Commit against a validated world unless the immutable world can prove
  // that subsequent changes intersect the candidate's protected region. The
  // callback receives the identity that is actually being committed. The
  // default preserves compatibility for authorizers without change history
  // and therefore remains fail-closed on a stale identity.
  virtual WorldCommitDecision commitIfCurrentOrUnaffected(
      const WorldSnapshotIdentity& validated_identity,
      const AxisAlignedBox& protected_region,
      const std::function<bool(const WorldValidationLease&)>& final_commit) {
    return commitIfCurrent(validated_identity, [&] {
      const auto lease = latest();
      return final_commit(lease);
    });
  }
};

using WorldCommitAuthorizerPtr = std::shared_ptr<WorldCommitAuthorizer>;

}  // namespace navigation_world_model
