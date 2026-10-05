#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <variant>

#include <navigation_contracts/tracking_experiment.hpp>

namespace {

struct TestNode {
  using Value = std::variant<bool, double, std::string>;
  struct Parameter {
    bool value;
    bool as_bool() const { return value; }
  };
  std::unordered_map<std::string, Value> values;

  template<class T>
  T declare_parameter(const std::string& name, T fallback) {
    const auto it = values.find(name);
    return it == values.end() ? fallback : std::get<T>(it->second);
  }
  Parameter get_parameter(const std::string& name) const {
    return {std::get<bool>(values.at(name))};
  }
};

TEST(TrackingExperimentPolicy, SimulatedTimeDoesNotEnableOffMode) {
  TestNode node{{{"use_sim_time", true}, {"tracking_experiment.mode", std::string("off")}}};
  const auto policy = navigation_contracts::loadTrackingExperimentPolicy(node);
  EXPECT_FALSE(policy.enabled);
  EXPECT_FALSE(policy.suppress_braking);
  EXPECT_FALSE(policy.suppress_estimator_health_response);
  EXPECT_FALSE(policy.velocity_only_enabled);
}

TEST(TrackingExperimentPolicy, RelaxedModeRequiresExplicitSelection) {
  TestNode node{{{"use_sim_time", true},
      {"tracking_experiment.mode", std::string("relaxed")},
      {"tracking_experiment.base_m", 0.2}}};
  const auto policy = navigation_contracts::loadTrackingExperimentPolicy(node);
  EXPECT_TRUE(policy.enabled);
  EXPECT_TRUE(policy.suppress_braking);
  EXPECT_TRUE(policy.suppress_estimator_health_response);
}

TEST(TrackingExperimentPolicy, ExplicitAdaptiveRelaxationIsDiagnostic) {
  TestNode node{{{"use_sim_time", true},
      {"tracking_experiment.mode", std::string("adaptive")},
      {"tracking_experiment.tracking_gate_relaxed", true},
      {"tracking_experiment.base_m", 0.2}}};
  const auto policy = navigation_contracts::loadTrackingExperimentPolicy(node);
  EXPECT_TRUE(policy.enabled);
  EXPECT_TRUE(policy.suppress_braking);
  EXPECT_TRUE(policy.suppress_estimator_health_response);
}

TEST(TrackingExperimentPolicy, NonSimulatedExperimentRejected) {
  TestNode node{{{"use_sim_time", false}, {"tracking_experiment.mode", std::string("relaxed")}}};
  EXPECT_THROW(navigation_contracts::loadTrackingExperimentPolicy(node), std::invalid_argument);
}

TEST(TrackingExperimentPolicy, AdaptiveGateDoesNotSuppressWithPositiveEnvelope) {
  TestNode node{{{"use_sim_time", true},
      {"tracking_experiment.mode", std::string("adaptive")},
      {"tracking_experiment.base_m", 0.2}}};
  const auto policy = navigation_contracts::loadTrackingExperimentPolicy(node);
  EXPECT_TRUE(policy.enabled);
  EXPECT_FALSE(policy.suppress_braking);
}

TEST(TrackingExperimentPolicy, VelocityOnlyFlagMustMatchMode) {
  TestNode node{{{"use_sim_time", true},
      {"tracking_experiment.mode", std::string("off")},
      {"tracking_experiment.velocity_only_enabled", true}}};
  EXPECT_THROW(navigation_contracts::loadTrackingExperimentPolicy(node), std::invalid_argument);
}

}  // namespace
