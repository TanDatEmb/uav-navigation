#pragma once

#include <navigation_mission/mission.hpp>

namespace px4_navigation_external_mode {

using MissionWaypoint = navigation_mission::MissionWaypoint;
using MissionPlanningConfig = navigation_mission::MissionPlanningConfig;
using MissionControlConfig = navigation_mission::MissionControlConfig;
using Mission = navigation_mission::Mission;
using navigation_mission::loadMission;

}  // namespace px4_navigation_external_mode
