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
};

using WorldCommitAuthorizerPtr = std::shared_ptr<WorldCommitAuthorizer>;

}  // namespace navigation_world_model
