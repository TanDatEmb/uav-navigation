# ADR-008: Registration map is not a world model

**Status:** accepted. `RegistrationMap` is an odom-framed nearest-neighbour and
plane-support structure for LIO. It has no occupancy, free-space, obstacle,
collision, planning, dynamic-object, or global-map semantics. Those concerns
remain outside M1.
