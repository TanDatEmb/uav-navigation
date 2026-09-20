#include <cassert>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>

#include "navigation_contracts/tracking_experiment.hpp"

struct FakeParam {
  bool value{};
  bool as_bool() const { return value; }
};
struct FakeNode {
  bool use_sim_time{false};
  std::map<std::string, std::variant<bool, double>> params;
  template<class T> T declare_parameter(const std::string& name, T fallback) {
    auto it = params.find(name);
    if (it == params.end()) return fallback;
    return std::get<T>(it->second);
  }
  FakeParam get_parameter(const std::string& name) {
    assert(name == "use_sim_time");
    return FakeParam{use_sim_time};
  }
};
int main() {
  using navigation_contracts::loadTrackingExperimentPolicy;
  FakeNode sim_off;
  auto off = loadTrackingExperimentPolicy(sim_off);
  assert(!off.enabled && !off.suppress_braking &&
         !off.suppress_estimator_health_response && !off.trackingGateEnabled());
  std::cout << "sim=false, coefficients=zero -> enabled=false, suppression=false\n";
  FakeNode sim_on_zero;
  sim_on_zero.use_sim_time = true;
  auto zero = loadTrackingExperimentPolicy(sim_on_zero);
  assert(zero.enabled && zero.suppress_braking &&
         zero.suppress_estimator_health_response && !zero.trackingGateEnabled());
  std::cout << "sim=true, coefficients=zero -> enabled=true, both suppressions=true\n";
  FakeNode sim_on_gate;
  sim_on_gate.use_sim_time = true;
  sim_on_gate.params["tracking_experiment.base_m"] = 0.2;
  auto gate = loadTrackingExperimentPolicy(sim_on_gate);
  assert(gate.enabled && gate.trackingGateEnabled() &&
         !gate.suppress_braking && !gate.suppress_estimator_health_response);
  std::cout << "sim=true, base_m=0.2 -> adaptive gate=true, suppression=false\n";
  FakeNode velocity_off;
  velocity_off.params["tracking_experiment.velocity_only_enabled"] = true;
  bool rejected = false;
  try { (void)loadTrackingExperimentPolicy(velocity_off); }
  catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
  std::cout << "sim=false, velocity_only=true -> loader rejects\n";
  FakeNode velocity_on;
  velocity_on.use_sim_time = true;
  velocity_on.params["tracking_experiment.velocity_only_enabled"] = true;
  velocity_on.params["tracking_experiment.velocity_only_gain_s_inv"] = 1.0;
  velocity_on.params["tracking_experiment.velocity_only_cap_mps"] = 3.0;
  velocity_on.params["tracking_experiment.velocity_only_max_acceleration_mps2"] = 2.0;
  velocity_on.params["tracking_experiment.velocity_only_max_jerk_mps3"] = 5.0;
  velocity_on.params["tracking_experiment.velocity_only_max_timing_bound_s"] = 0.1;
  velocity_on.params["tracking_experiment.velocity_only_max_reference_age_s"] = 0.2;
  auto vo = loadTrackingExperimentPolicy(velocity_on);
  assert(vo.enabled && vo.velocity_only_enabled && vo.valid());
  assert(vo.suppress_braking && vo.suppress_estimator_health_response);
  std::cout << "sim=true, valid velocity_only=true, zero tracking coefficients -> valid; suppressions remain true\n";
}
