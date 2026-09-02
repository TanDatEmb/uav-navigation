#pragma once

#include <cstdint>
#include <optional>

#include <navigation_execution/committed_bundle_store.hpp>

namespace navigation_execution {

enum class SampleStatus : std::uint8_t {
  kNoActiveBundle,
  kAwaitingActivation,
  kActiveSample,
  kStoppedHold,
  kExpiredLease,
  kGoalMismatch,
  kEvaluatorFailure,
};

struct SampleResult {
  std::shared_ptr<const navigation_planning::CandidateBundle> bundle;
  std::optional<navigation_planning::TrajectoryPoint> point;
  bool awaiting_activation{false};
  // The analytic lease ended. The point is a bounded STOPPED_HOLD fallback,
  // never an extension of the trajectory evaluator.
  bool planned_stop_hold{false};
  SampleStatus status{SampleStatus::kNoActiveBundle};
  std::uint64_t timeline_version{0};

  [[nodiscard]] explicit operator bool() const noexcept {
    return static_cast<bool>(bundle) && point.has_value();
  }
};

// The sampler evaluates an immutable bundle. It does not acquire world locks,
// call ROS, perform trajectory validation, or finalize planner state. The
// execution timeline is the sole activation owner.
class CommandSampler final {
 public:
  explicit CommandSampler(const CommittedBundleStore& store) : store_(store) {}

  [[nodiscard]] SampleResult sample(
      std::int64_t stamp_ns,
      std::uint64_t expected_goal_epoch = 0U) const noexcept {
    // Activation is owned by the runtime execution transaction. Sampling is
    // read-only and cannot independently swap the active pointer or finalize
    // planner history.
    return sampleActive(stamp_ns, expected_goal_epoch);
  }

 private:
  [[nodiscard]] SampleResult sampleActive(
      std::int64_t stamp_ns,
      std::uint64_t expected_goal_epoch) const noexcept {
    const auto timeline = store_.snapshot();
    auto bundle = timeline.active;
    if (!bundle) {
      auto pending = timeline.pending;
      if (!pending) return {nullptr, std::nullopt, false, false,
                            SampleStatus::kNoActiveBundle, timeline.version};
      if (expected_goal_epoch != 0U &&
          pending->goal_epoch != expected_goal_epoch) {
        return {std::move(pending), std::nullopt, false, false,
                SampleStatus::kGoalMismatch, timeline.version};
      }
      return {std::move(pending), std::nullopt, true, false,
              SampleStatus::kAwaitingActivation, timeline.version};
    }
    // A retained bundle may outlive a hot-retarget transition in the store.
    // Never let the runtime relabel that old trajectory as the new goal.
    if (expected_goal_epoch != 0U && bundle->goal_epoch != expected_goal_epoch) {
      return {std::move(bundle), std::nullopt, false, false,
              SampleStatus::kGoalMismatch, timeline.version};
    }
    if (stamp_ns < bundle->valid_from_ns) {
      return {std::move(bundle), std::nullopt, true, false,
              SampleStatus::kAwaitingActivation, timeline.version};
    }
    try {
      auto point = bundle->sample(stamp_ns);
      const auto declared_end_ns = bundle->declared_end_ns > 0
          ? std::optional<std::int64_t>{bundle->declared_end_ns}
          : navigation_common::secondsSumToNanoseconds(
              bundle->start_wall_time_s, bundle->duration_s);
      if (!point && stamp_ns > bundle->valid_until_ns && declared_end_ns &&
          stamp_ns >= *declared_end_ns) {
        // The lease is intentionally not extended. A terminal sample is
        // converted into an explicit hold only after the runtime performs its
        // known-free and measured-proximity checks.
        auto endpoint = bundle->sampleAtDeclaredEnd();
        if (endpoint) {
          return {std::move(bundle), std::move(endpoint), false, true,
                  SampleStatus::kStoppedHold, timeline.version};
        }
      }
      const auto status = point
          ? SampleStatus::kActiveSample
          : (stamp_ns > bundle->valid_until_ns
                 ? SampleStatus::kExpiredLease
                 : SampleStatus::kEvaluatorFailure);
      return {std::move(bundle), std::move(point), false, false, status,
              timeline.version};
    } catch (...) {
      // Preserve the bundle identity so the runtime can distinguish a
      // malformed/expired sample from a world-recertification gap.
      return {std::move(bundle), std::nullopt, false, false,
              SampleStatus::kEvaluatorFailure, timeline.version};
    }
  }

  const CommittedBundleStore& store_;
};

}  // namespace navigation_execution
