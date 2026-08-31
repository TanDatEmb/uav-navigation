#include <navigation_mission/mission.hpp>
#include <navigation_mission/route_progress.hpp>

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace {

class TemporaryMission {
 public:
  explicit TemporaryMission(const char* contents)
      : path_(std::filesystem::temp_directory_path() /
              ("navigation_mission_" +
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
               ".yaml")) {
    std::ofstream stream(path_);
    stream << contents;
  }
  ~TemporaryMission() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }
  const std::string string() const { return path_.string(); }

 private:
  std::filesystem::path path_;
};

}  // namespace

TEST(MissionContract, LoadsOneValidatedProductSchema) {
  const TemporaryMission mission(R"(
mission:
  version: 1
  id: smoke
  frame: lio_odom
  waypoints:
    - id: finish
      position: [1.0, 2.0, 3.0]
      acceptance_radius_m: 0.5
      behavior: stop
  planning:
    max_velocity_mps: 7.0
    max_acceleration_mps2: 5.0
    max_jerk_mps3: 12.0
    unknown_policy: allow_unknown
)");
  const auto loaded = navigation_mission::loadMission(mission.string(), "lio_odom");
  EXPECT_EQ(loaded.id, "smoke");
  EXPECT_DOUBLE_EQ(loaded.planning.max_velocity_mps, 7.0);
  EXPECT_EQ(loaded.planning.unknown_policy,
            navigation_world_model::UnknownPolicy::kAllowUnknown);
}

TEST(MissionContract, RejectsFrameMismatchAndUnknownKeys) {
  const TemporaryMission mission(R"(
mission:
  version: 1
  id: smoke
  frame: map
  waypoints:
    - id: finish
      position: [1.0, 2.0, 3.0]
      acceptance_radius_m: 0.5
      unexpected: true
)");
  EXPECT_THROW(navigation_mission::loadMission(mission.string(), "lio_odom"),
               std::invalid_argument);
}

TEST(MissionContract, LoaderRejectsOversizedWaypointSequenceBeforeConstruction) {
  std::ostringstream yaml;
  yaml << "mission:\n"
       << "  version: 1\n"
       << "  id: bounded\n"
       << "  frame: lio_odom\n"
       << "  waypoints:\n";
  for (std::size_t index = 0U;
       index < navigation_mission::kMaximumMissionWaypoints + 1U; ++index) {
    yaml << "    - id: waypoint_" << index << "\n"
         << "      position: [0.0, 0.0, 1.0]\n"
         << "      acceptance_radius_m: 0.5\n";
  }
  const TemporaryMission mission(yaml.str().c_str());
  EXPECT_THROW(navigation_mission::loadMission(mission.string(), "lio_odom"),
               std::invalid_argument);
}

TEST(MissionContract, RejectsAValidMissionWithTheWrongPlanningFrame) {
  const TemporaryMission mission(R"(
mission:
  version: 1
  id: smoke
  frame: map
  waypoints:
    - id: finish
      position: [1.0, 2.0, 3.0]
      acceptance_radius_m: 0.5
      behavior: stop
)");
  EXPECT_THROW(navigation_mission::loadMission(mission.string(), "lio_odom"),
               std::invalid_argument);
}

namespace {

navigation_mission::Mission makeRouteMission() {
  navigation_mission::Mission mission;
  mission.id = "route";
  mission.frame = "lio_odom";
  mission.waypoints = {
      {"start", Eigen::Vector3d{0.0, 0.0, 2.0}, 0.5, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"duplicate", Eigen::Vector3d{0.0, 0.0, 2.0}, 0.5, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"corner", Eigen::Vector3d{10.0, 0.0, 3.0}, 0.7, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"finish", Eigen::Vector3d{10.0, 10.0, 5.0}, 0.8, 0.2,
       navigation_mission::MissionWaypoint::Behavior::Stop},
  };
  return mission;
}

}  // namespace

TEST(MissionContract, RejectsOversizedMissionPayloadsAtContractBoundary) {
  auto mission = makeRouteMission();
  mission.id.assign(navigation_mission::kMaximumMissionIdLength + 1U, 'm');
  EXPECT_FALSE(mission.valid());

  mission = makeRouteMission();
  mission.waypoints.front().id.assign(
      navigation_mission::kMaximumWaypointIdLength + 1U, 'w');
  EXPECT_FALSE(mission.valid());

  mission = makeRouteMission();
  mission.waypoints.resize(navigation_mission::kMaximumMissionWaypoints + 1U);
  EXPECT_FALSE(mission.valid());
}

TEST(RouteProgress, SkipsDuplicateWaypointsAndInterpolatesAltitude) {
  const navigation_mission::RouteProgress route(makeRouteMission());

  ASSERT_EQ(route.segments().size(), 2U);
  EXPECT_DOUBLE_EQ(route.totalLengthM(), std::sqrt(101.0) + std::sqrt(104.0));
  EXPECT_DOUBLE_EQ(route.waypointArcLengthM(0U), 0.0);
  EXPECT_DOUBLE_EQ(route.waypointArcLengthM(1U), 0.0);
  EXPECT_DOUBLE_EQ(route.waypointArcLengthM(2U), std::sqrt(101.0));
  EXPECT_TRUE(route.insideAcceptance(2U, Eigen::Vector3d{10.4, 0.0, 3.0}));
  EXPECT_FALSE(route.insideAcceptance(2U, Eigen::Vector3d{10.8, 0.0, 3.0}));

  const auto point = route.pointAtArc(std::sqrt(101.0) / 2.0);
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->z(), 2.5, 1.0e-12);
  EXPECT_NEAR(route.altitudeAtArc(std::sqrt(101.0) / 2.0), 2.5, 1.0e-12);
}

TEST(RouteProgress, TreatsCoincidentPassThroughThenStopAsTerminalBoundary) {
  auto mission = makeRouteMission();
  mission.waypoints = {
      {"pass", Eigen::Vector3d{7.0, 0.0, 3.0}, 0.5, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"stop", Eigen::Vector3d{7.0, 0.0, 3.0}, 0.5, 0.0,
       navigation_mission::MissionWaypoint::Behavior::Stop},
  };
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(Eigen::Vector3d{6.5, 0.0, 3.0}).valid);
  const auto route = progress.snapshot(mission.id, mission.frame, 1U, 1U, 0U);

  EXPECT_TRUE(navigation_mission::passThroughNextWaypointIsCoincidentStop(route));

  mission.waypoints[1].position_enu.x() +=
      navigation_world_model::kGoalConnectionToleranceM + 0.01;
  navigation_mission::RouteProgress separated_progress(mission);
  ASSERT_TRUE(separated_progress.update(Eigen::Vector3d{6.5, 0.0, 3.0}).valid);
  const auto separated = separated_progress.snapshot(
      mission.id, mission.frame, 1U, 1U, 0U);
  EXPECT_FALSE(
      navigation_mission::passThroughNextWaypointIsCoincidentStop(separated));
}

TEST(RouteProgress, ProgressDoesNotRegressWithinOrBeyondNoiseTolerance) {
  navigation_mission::RouteProgress route(
      makeRouteMission(), navigation_mission::RouteProgressConfig{0.5});

  const auto first = route.update(Eigen::Vector3d{6.0, 0.0, 2.6});
  ASSERT_TRUE(first.valid);
  const auto small_backtrack = route.update(Eigen::Vector3d{5.6, 0.0, 2.56});
  EXPECT_FALSE(small_backtrack.backtracking_exceeded);
  EXPECT_GE(small_backtrack.progress_arc_m, first.progress_arc_m);
  const auto large_backtrack = route.update(Eigen::Vector3d{3.0, 0.0, 2.3});
  EXPECT_TRUE(large_backtrack.backtracking_exceeded);
  EXPECT_GE(large_backtrack.progress_arc_m, small_backtrack.progress_arc_m);
}

TEST(RouteProgress, ExposesCornerTangentsAndFiniteProjection) {
  const navigation_mission::RouteProgress route(makeRouteMission());
  const auto incoming = route.incomingTangent(2U);
  const auto outgoing = route.outgoingTangent(2U);
  ASSERT_TRUE(incoming.has_value());
  ASSERT_TRUE(outgoing.has_value());
  EXPECT_NEAR(incoming->x(), 10.0 / std::sqrt(101.0), 1.0e-12);
  EXPECT_NEAR(incoming->z(), 1.0 / std::sqrt(101.0), 1.0e-12);
  EXPECT_DOUBLE_EQ(outgoing->x(), 0.0);
  EXPECT_NEAR(outgoing->y(), 10.0 / std::sqrt(104.0), 1.0e-12);
  EXPECT_NEAR(outgoing->z(), 2.0 / std::sqrt(104.0), 1.0e-12);

  const auto projection = route.project(Eigen::Vector3d{4.0, 1.0, 2.4});
  ASSERT_TRUE(projection.valid);
  EXPECT_TRUE(projection.point.allFinite());
  EXPECT_TRUE(projection.tangent.allFinite());
  EXPECT_GT(projection.lateral_error_m, 0.0);
}

TEST(RouteProgress, MeasuredBoundaryAcceptsForwardJumpButRejectsLateralMiss) {
  const navigation_mission::RouteProgress route(makeRouteMission());
  const auto crossed = route.measuredWaypointCrossingError(
      2U, Eigen::Vector3d{10.8, 0.0, 3.08},
      Eigen::Vector3d{9.2, 0.0, 2.92}, 0.05, 0.25);
  ASSERT_TRUE(crossed.has_value());
  EXPECT_LT(*crossed, 0.7);

  EXPECT_FALSE(route.measuredWaypointCrossingError(
      2U, Eigen::Vector3d{10.8, 1.0, 3.08},
      Eigen::Vector3d{9.2, 1.0, 2.92}, 0.05, 0.25).has_value());
  EXPECT_FALSE(route.measuredWaypointCrossingError(
      2U, Eigen::Vector3d{9.2, 0.0, 2.92},
      Eigen::Vector3d{10.8, 0.0, 3.08}, 0.05, 0.25).has_value());
}

TEST(RouteProgress, MonotonicProjectionSelectsReturnBranchAtReversal) {
  navigation_mission::Mission mission;
  mission.id = "reverse";
  mission.frame = "lio_odom";
  mission.waypoints = {
      {"start", Eigen::Vector3d{0.0, 0.0, 3.0}, 0.5, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"reverse", Eigen::Vector3d{10.0, 0.0, 3.0}, 0.5, 0.0,
       navigation_mission::MissionWaypoint::Behavior::PassThrough},
      {"finish", Eigen::Vector3d{0.0, 0.0, 3.0}, 0.5, 0.0,
       navigation_mission::MissionWaypoint::Behavior::Stop},
  };
  navigation_mission::RouteProgress route(mission);
  EXPECT_NEAR(route.update(Eigen::Vector3d{8.0, 0.0, 3.0}).progress_arc_m, 8.0, 1.0e-12);
  EXPECT_NEAR(route.update(Eigen::Vector3d{10.0, 0.0, 3.0}).progress_arc_m, 10.0, 1.0e-12);
  const auto returning = route.update(Eigen::Vector3d{9.0, 0.0, 3.0});
  EXPECT_NEAR(returning.progress_arc_m, 11.0, 1.0e-12);
  EXPECT_EQ(returning.projection.segment_index, 1U);
  EXPECT_FALSE(returning.backtracking_exceeded);
}

TEST(RouteProgress, ImmutableSnapshotCarriesMeasuredRouteIdentityAndLookahead) {
  navigation_mission::RouteProgress route(makeRouteMission());
  const auto state = route.update(Eigen::Vector3d{4.0, 0.2, 1.4});
  ASSERT_TRUE(state.valid);
  const auto snapshot = route.snapshot("mission", "lio_odom", 1U, 7U, 1U);
  ASSERT_TRUE(snapshot.valid());
  EXPECT_EQ(snapshot.mission_id, "mission");
  EXPECT_EQ(snapshot.active_waypoint_index, 1U);
  EXPECT_DOUBLE_EQ(snapshot.measured_progress.progress_arc_m,
                   state.progress_arc_m);
  EXPECT_GT(snapshot.measured_progress.projection.segment_fraction, 0.0);
  EXPECT_LT(snapshot.measured_progress.projection.segment_fraction, 1.0);
  const auto lookahead = snapshot.routeLookaheadPoint(3.0);
  const auto expected = route.pointAtArc(state.progress_arc_m + 3.0);
  ASSERT_TRUE(lookahead.has_value());
  ASSERT_TRUE(expected.has_value());
  EXPECT_TRUE(lookahead->isApprox(*expected, 1.0e-12));

  auto malformed = snapshot;
  malformed.request_id = 0U;
  EXPECT_FALSE(malformed.valid());
}

TEST(RouteProgress, ImmutableSnapshotRepresentsSingleWaypointWithoutSegment) {
  navigation_mission::Mission mission;
  mission.id = "hold-only";
  mission.frame = "lio_odom";
  navigation_mission::MissionWaypoint waypoint;
  waypoint.id = "hold";
  waypoint.position_enu = Eigen::Vector3d{2.0, 3.0, 4.0};
  waypoint.acceptance_radius_m = 0.5;
  mission.waypoints = {waypoint};
  navigation_mission::RouteProgress progress(mission);
  ASSERT_TRUE(progress.update(Eigen::Vector3d{2.1, 3.0, 4.0}).valid);

  const auto snapshot = progress.snapshot(
      mission.id, mission.frame, 1U, 1U, 0U);
  ASSERT_TRUE(snapshot.valid());
  EXPECT_TRUE(snapshot.segments.empty());
  ASSERT_TRUE(snapshot.routeLookaheadPoint(10.0).has_value());
  EXPECT_TRUE(snapshot.routeLookaheadPoint(10.0)->isApprox(
      mission.waypoints.front().position_enu));
}

TEST(RouteProgress, RejectsNonFiniteArcQueriesInsteadOfClampingToOrigin) {
  const navigation_mission::RouteProgress route(makeRouteMission());
  EXPECT_FALSE(route.pointAtArc(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(std::isnan(route.altitudeAtArc(
      std::numeric_limits<double>::infinity())));
}

TEST(RouteProgress, ImmutableSnapshotRequiresCompleteOrderedGeometry) {
  navigation_mission::RouteProgress route(makeRouteMission());
  ASSERT_TRUE(route.update(Eigen::Vector3d{4.0, 0.0, 2.4}).valid);
  auto snapshot = route.snapshot("mission", "lio_odom", 1U, 1U, 1U);
  ASSERT_TRUE(snapshot.valid());

  snapshot.segments.pop_back();
  EXPECT_FALSE(snapshot.valid());

  snapshot = route.snapshot("mission", "lio_odom", 1U, 1U, 1U);
  snapshot.segments.front().tangent = Eigen::Vector3d::Zero();
  EXPECT_FALSE(snapshot.valid());

  snapshot = route.snapshot("mission", "lio_odom", 1U, 1U, 1U);
  snapshot.measured_progress.projection.segment_fraction = 1.5;
  EXPECT_FALSE(snapshot.valid());

  snapshot = route.snapshot("mission", "lio_odom", 1U, 1U, 1U);
  snapshot.measured_progress.projection.point += Eigen::Vector3d{0.1, 0.0, 0.0};
  EXPECT_FALSE(snapshot.valid());

  snapshot = route.snapshot("mission", "lio_odom", 1U, 1U, 1U);
  snapshot.segments.front().end_arc_m += 0.1;
  EXPECT_FALSE(snapshot.valid());

  snapshot = route.snapshot("mission", "lio_odom", 1U, 1U, 1U);
  snapshot.waypoint_arc_lengths_m[1] += 0.1;
  EXPECT_FALSE(snapshot.valid());
}

TEST(RouteProgress, RejectsInvalidDirectWaypointSemantics) {
  auto mission = makeRouteMission();
  mission.waypoints.front().hold_s = -1.0;
  EXPECT_THROW({ const navigation_mission::RouteProgress route(mission); },
               std::invalid_argument);

  mission = makeRouteMission();
  mission.waypoints.front().behavior =
      static_cast<navigation_mission::MissionWaypoint::Behavior>(255U);
  EXPECT_THROW({ const navigation_mission::RouteProgress route(mission); },
               std::invalid_argument);
}

TEST(RouteProgress, RejectsInvalidDirectMissionIdentityAndLimits) {
  auto mission = makeRouteMission();
  mission.id.clear();
  EXPECT_THROW({ const navigation_mission::RouteProgress route(mission); },
               std::invalid_argument);

  mission = makeRouteMission();
  mission.planning.unknown_policy =
      static_cast<navigation_world_model::UnknownPolicy>(255U);
  EXPECT_THROW({ const navigation_mission::RouteProgress route(mission); },
               std::invalid_argument);

  mission = makeRouteMission();
  mission.control.acceptance_speed_mps = -1.0;
  EXPECT_THROW({ const navigation_mission::RouteProgress route(mission); },
               std::invalid_argument);
}
