// Audit-only replay of the pinned TARGET RouteProgress geometry.
// It evaluates every recorded propagated-odometry sample, not the unrecorded
// MissionController update callback schedule. Thus it yields crossing
// opportunities, not proof of a product acceptance decision.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <navigation_mission/mission.hpp>
#include <navigation_mission/route_progress.hpp>

struct Goal { std::int64_t bag_ns{}; std::size_t waypoint{}; std::uint64_t request{}; };

std::vector<std::string> cells(const std::string& line) {
  std::vector<std::string> result;
  std::stringstream stream(line);
  std::string cell;
  while (std::getline(stream, cell, ',')) result.push_back(cell);
  return result;
}

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: route_crossing_replay MISSION.yaml odom.csv goal.csv\n";
    return 2;
  }
  const auto mission = navigation_mission::loadMission(argv[1]);
  navigation_mission::RouteProgress route(mission);
  std::vector<Goal> goals;
  std::ifstream gf(argv[3]);
  std::string line;
  std::getline(gf, line);
  while (std::getline(gf, line)) {
    const auto row = cells(line);
    if (row.size() < 5) continue;
    goals.push_back({std::stoll(row[0]), static_cast<std::size_t>(std::stoul(row[3])), std::stoull(row[4])});
  }
  std::sort(goals.begin(), goals.end(), [](const Goal& a, const Goal& b) { return a.bag_ns < b.bag_ns; });
  std::ifstream of(argv[2]);
  std::getline(of, line);
  std::size_t goal_cursor = 0;
  std::optional<Eigen::Vector3d> previous;
  std::int64_t previous_source_ns = 0;
  std::uint64_t previous_epoch = 0;
  std::size_t checked = 0;
  std::cout << "bag_ns,source_ns,epoch,sequence,waypoint,request,previous_source_ns,gap_s,error_m,inside_ball,speed_mps,segment_index,progress_arc_m\n";
  while (std::getline(of, line)) {
    const auto r = cells(line);
    if (r.size() != 10) continue;
    const auto bag_ns = std::stoll(r[0]);
    while (goal_cursor + 1 < goals.size() && goals[goal_cursor + 1].bag_ns <= bag_ns) ++goal_cursor;
    if (goals.empty() || goals[goal_cursor].bag_ns > bag_ns) continue;
    const auto wp = goals[goal_cursor].waypoint;
    if (wp >= mission.waypoints.size() ||
        mission.waypoints[wp].behavior != navigation_mission::MissionWaypoint::Behavior::PassThrough) continue;
    const auto source_ns = std::stoll(r[1]);
    const auto epoch = std::stoull(r[2]);
    const auto sequence = std::stoull(r[3]);
    const Eigen::Vector3d position{std::stod(r[4]), std::stod(r[5]), std::stod(r[6])};
    const Eigen::Vector3d velocity{std::stod(r[7]), std::stod(r[8]), std::stod(r[9])};
    if (epoch != previous_epoch) {
      route.reset();
      previous.reset();
      previous_source_ns = 0;
      previous_epoch = epoch;
    }
    const double gap_s = previous && source_ns >= previous_source_ns
        ? static_cast<double>(source_ns - previous_source_ns) * 1e-9 : NAN;
    const auto state = route.update(position);
    const auto error = route.measuredWaypointCrossingError(wp, position, previous, gap_s, 0.25);
    const bool inside = route.insideAcceptance(wp, position);
    if (error) {
      std::cout << bag_ns << ',' << source_ns << ',' << epoch << ',' << sequence << ','
                << wp << ',' << goals[goal_cursor].request << ',' << previous_source_ns << ','
                << gap_s << ',' << *error << ',' << inside << ',' << velocity.norm() << ','
                << state.projection.segment_index << ',' << state.progress_arc_m << '\n';
    }
    previous = position;
    previous_source_ns = source_ns;
    ++checked;
  }
  std::cerr << "evaluated_samples=" << checked << '\n';
}
