#pragma once

#include <cstdint>
#include <optional>

#include <navigation_execution/committed_bundle_store.hpp>

namespace navigation_execution {

struct SampleResult {
  std::shared_ptr<const navigation_planning::CandidateBundle> bundle;
  std::optional<navigation_planning::TrajectoryPoint> point;

  [[nodiscard]] explicit operator bool() const noexcept {
    return static_cast<bool>(bundle) && point.has_value();
  }
};

// The sampler only loads an immutable bundle and evaluates it. It does not
// acquire planner/world locks, call ROS, or perform trajectory validation.
class CommandSampler final {
 public:
  explicit CommandSampler(const CommittedBundleStore& store) : store_(store) {}

  [[nodiscard]] SampleResult sample(std::int64_t stamp_ns) const noexcept {
    auto bundle = store_.load();
    if (!bundle) return {};
    try {
      return {bundle, bundle->sample(stamp_ns)};
    } catch (...) {
      return {};
    }
  }

 private:
  const CommittedBundleStore& store_;
};

}  // namespace navigation_execution
