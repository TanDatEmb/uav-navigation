# EMERGENCY runtime contract audit

## Owner path

The planner backend's measured emergency-brake path creates and certifies the emergency bundle; Core reserves/commits it through `ExecutionAuthority`. The runtime emits that active record as a `NavigationCommand` carrying `ROLE_EMERGENCY`, `STATUS_BRAKING`, emergency authorization reason, commit result, and execution identity. The PX4 adapter accepts an emergency command only through its existing typed contract/identity/state checks. The setpoint path independently rejects an emergency role unless the command carries the current actual-anchor certificate reason and successful candidate commit receipt. No separate emergency authority owner is present.

## Evidence already present in source/tests

- `navigation_runtime/test/test_navigation_runtime_terminal_monitor.cpp` exercises measured emergency brake construction, commit evidence, and rejected/accepted commit dispositions.
- `navigation_planning_backend/test/test_planner_facade.cpp` tests measured-state emergency certification, UNKNOWN/blocked World policy, and request-owned emergency predecessor authorization.
- `navigation_mode_node.cpp::publishVelocityOnlySetpoint()` gates emergency use by role, braking status, actual-anchor certificate reason, and committed candidate result before invoking the existing tracking adapter.
- `NavigationCommand` boundary tests cover emergency role/schema acceptance and rejection cases.
- `NavigationMode` PX4 input trace is produced around `TrajectorySetpoint::update()` and carries the selected command identity/role when the normal command path reaches the native setpoint boundary.

## Evidence limitation

The current progression fixture does not construct a real certified emergency Core result, feed it through the ROS command subscription, and observe a PX4 setpoint update. Existing test coverage is split across Core/planner and adapter contract boundaries. This branch has not yet produced a focused end-to-end Emergency SITL run or measured PX4 input trace with an emergency identity. Therefore the complete positive chain and uncertifiable-emergency-to-Hold fallback remain unproven at runtime for this branch.

## No behavior shortcut

No emergency injection or command fabrication was added. Existing source checks and planner certificates were not bypassed. This audit does not claim firmware consumption from a ROS/PX4 setpoint trace.
