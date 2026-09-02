#pragma once

#include <cstdint>
#include <optional>
#include <utility>

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

// The sampler evaluates an immutable bundle. It does not acquire world locks,
// call ROS, or perform trajectory validation; the runtime may supply a small
// activation finalizer for the planner transaction at the swap boundary.
class CommandSampler final {
 public:
  explicit CommandSampler(const CommittedBundleStore& store) : store_(store) {}

  [[nodiscard]] SampleResult sample(
      std::int64_t stamp_ns,
      std::uint64_t expected_goal_epoch = 0U) const noexcept {
    // Activation is the only active-pointer swap. Keep it on the sampler's
    // bounded 50 Hz path so a planning completion can never interrupt the
    // command currently being published.
    (void)store_.activatePendingIfDue(stamp_ns);
    return sampleActive(stamp_ns, expected_goal_epoch);
  }

  // Runtime uses this overload to finalize planner ownership exactly when the
  // execution timeline swaps the successor. Generic consumers retain the
  // no-callback overload above, which activates a fully store-owned bundle.
  template <typename FinalizeFn>
  [[nodiscard]] SampleResult sample(
      std::int64_t stamp_ns, std::uint64_t expected_goal_epoch,
      FinalizeFn&& finalize) const noexcept {
    (void)store_.activatePendingIfDueAndFinalize(
        stamp_ns, std::forward<FinalizeFn>(finalize));
    return sampleActive(stamp_ns, expected_goal_epoch);
  }

 private:
  [[nodiscard]] SampleResult sampleActive(
      std::int64_t stamp_ns,
      std::uint64_t expected_goal_epoch) const noexcept {
    auto bundle = store_.load();
    if (!bundle) {
      auto pending = store_.loadPending();
      if (!pending || (expected_goal_epoch != 0U &&
                       pending->goal_epoch != expected_goal_epoch)) return {};
      return {std::move(pending), std::nullopt, true};
    }
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

  const CommittedBundleStore& store_;
};

}  // namespace navigation_execution
