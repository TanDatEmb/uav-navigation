#pragma once

#include <cstdint>
#include <optional>

#include <navigation_execution/committed_bundle_store.hpp>

namespace navigation_execution {

struct SampleResult {
  std::shared_ptr<const navigation_planning::CandidateBundle> bundle;
  std::optional<navigation_planning::TrajectoryPoint> point;
  bool awaiting_activation{false};

  [[nodiscard]] explicit operator bool() const noexcept {
    return static_cast<bool>(bundle) && point.has_value();
  }
};

// The sampler only loads an immutable bundle and evaluates it. It does not
// acquire planner/world locks, call ROS, or perform trajectory validation.
class CommandSampler final {
 public:
  explicit CommandSampler(const CommittedBundleStore& store) : store_(store) {}

  [[nodiscard]] SampleResult sample(
      std::int64_t stamp_ns,
      std::uint64_t expected_goal_epoch = 0U) const noexcept {
    auto bundle = store_.load();
    if (!bundle) return {};
    // A retained bundle may outlive a hot-retarget transition in the store.
    // Never let the runtime relabel that old trajectory as the new goal.
    if (expected_goal_epoch != 0U && bundle->goal_epoch != expected_goal_epoch) {
      return {};
    }
    if (stamp_ns < bundle->valid_from_ns) {
      return {std::move(bundle), std::nullopt, true};
    }
    try {
      auto point = bundle->sample(stamp_ns);
      return {std::move(bundle), std::move(point), false};
    } catch (...) {
      // Preserve the bundle identity so the runtime can distinguish a
      // malformed/expired sample from a world-recertification gap.
      return {std::move(bundle), std::nullopt, false};
    }
  }

 private:
  const CommittedBundleStore& store_;
};

}  // namespace navigation_execution
