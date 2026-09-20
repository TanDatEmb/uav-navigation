# navigation_mission

Product-owned mission schema and validated YAML loader shared by the runtime
planner boundary and PX4 External Mode. The package owns waypoint identity,
planning limits, UNKNOWN policy, control values, frame validation and the
mission schema version.

Python runner/report code may read the same YAML for orchestration and artifact
analysis, but it is not a flight or planner authority.
