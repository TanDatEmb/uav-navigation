#include <cassert>
#include <iostream>
#include <optional>

#include "px4_navigation_external_mode/mission_controller.hpp"

using namespace px4_navigation_external_mode;

Mission makeMission() {
  Mission m;
  m.id = "h1-local";
  m.frame = "map";
  m.control.acceptance_speed_mps = 0.15;
  m.control.acceptance_confirmation_s = 0.5;
  for (const auto [id, x, behavior] : {
           std::tuple<const char*, double, MissionWaypoint::Behavior>{"start", 0.0, MissionWaypoint::Behavior::PassThrough},
           {"cross", 1.0, MissionWaypoint::Behavior::PassThrough},
           {"end", 3.0, MissionWaypoint::Behavior::Stop}}) {
    MissionWaypoint w;
    w.id = id;
    w.position_enu = Eigen::Vector3d{x, 0.0, 0.0};
    w.acceptance_radius_m = 0.1;
    w.behavior = behavior;
    m.waypoints.push_back(w);
  }
  return m;
}

MissionControllerEvent startAtCrossing(MissionController& c) {
  c.activate(1.0);
  const auto first_goal = c.update(
      1.0, Eigen::Vector3d{0.0, 0.0, 0.0}, true,
      Eigen::Vector3d{0.0, 0.0, 0.0});
  assert(first_goal.type == MissionControllerEvent::Type::PublishGoal);
  assert(!first_goal.waypoint_accepted && c.activeRequestId() == 1U);
  c.onNativeTrajectoryReady();
  const auto accepted_start = c.update(
      1.005, Eigen::Vector3d{0.0, 0.0, 0.0}, true,
      Eigen::Vector3d{0.0, 0.0, 0.0});
  assert(accepted_start.type == MissionControllerEvent::Type::PublishGoal);
  assert(accepted_start.waypoint_accepted && accepted_start.accepted_waypoint_index == 0U);
  assert(c.activeWaypointIndex() == 1U);
  c.onNativeTrajectoryReady();
  const auto approach = c.update(
      1.01, Eigen::Vector3d{0.5, 0.0, 0.0}, true,
      Eigen::Vector3d{2.0, 0.0, 0.0});
  assert(approach.type == MissionControllerEvent::Type::None);
  return accepted_start;
}

CertifiedContinuation witness(const MissionController& c, std::uint64_t request_delta=0) {
  return CertifiedContinuation{"h1-local", 1U,
      c.activeRequestId() + request_delta, 10'620'000'000ULL};
}

void positiveControl() {
  MissionController c(makeMission());
  startAtCrossing(c);
  const auto accepted = c.update(
      1.06, Eigen::Vector3d{1.5, 0.0, 0.0}, true,
      Eigen::Vector3d{2.0, 0.0, 0.0}, witness(c));
  assert(accepted.type == MissionControllerEvent::Type::PublishGoal);
  assert(accepted.waypoint_accepted && accepted.accepted_waypoint_index == 1U);
  assert(c.activeWaypointIndex() == 2U);
}

void noWitnessThenLateWitness() {
  MissionController c(makeMission());
  startAtCrossing(c);
  const auto missed = c.update(
      1.06, Eigen::Vector3d{1.5, 0.0, 0.0}, true,
      Eigen::Vector3d{2.0, 0.0, 0.0});
  assert(missed.type == MissionControllerEvent::Type::None);
  const auto late = c.update(
      1.07, Eigen::Vector3d{1.5, 0.0, 0.0}, true,
      Eigen::Vector3d{2.0, 0.0, 0.0}, witness(c));
  assert(late.type == MissionControllerEvent::Type::None);
  assert(c.activeWaypointIndex() == 1U);
}

void wrongIdentityNegativeControl() {
  MissionController c(makeMission());
  startAtCrossing(c);
  const auto rejected = c.update(
      1.06, Eigen::Vector3d{1.5, 0.0, 0.0}, true,
      Eigen::Vector3d{2.0, 0.0, 0.0}, witness(c, 1U));
  assert(rejected.type == MissionControllerEvent::Type::None);
  assert(c.activeWaypointIndex() == 1U);
}

int main() {
  positiveControl();
  noWitnessThenLateWitness();
  wrongIdentityNegativeControl();
  std::cout << "H1 native MissionController harness PASS\n"
            << "positive: current measured crossing + matching continuation advances once\n"
            << "counterexample: crossing with absent witness then same-position late witness does not recover\n"
            << "negative control: wrong request identity does not advance\n";
}

