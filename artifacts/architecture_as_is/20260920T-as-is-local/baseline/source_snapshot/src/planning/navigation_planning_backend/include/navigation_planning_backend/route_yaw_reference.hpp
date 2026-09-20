#pragma once

// Public runtime/planner integration surface. The implementation remains in
// planner_core, but the command loop must consume the same bounded heading
// primitive without depending on a private include path.
#include "planner_core/route_yaw_reference.hpp"
