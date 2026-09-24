# Terminal recovery contract (source-backed)

`NavigationRuntimeNode::publishCommand` sets `STATUS_COMPLETED` when an immutable candidate sample is `finished` or is a validated `planned_stop_hold` (`src/runtime/navigation_runtime/src/navigation_runtime_node.cpp`, near `traj_finish` and status selection). This is **analytic execution completion**, not a measured mission endpoint witness. The command still carries an exact request/generation and endpoint PVAJ.

The PX4 adapter independently compares its current odometry position to the completed command position against `kCommandAnchorErrorLimitM=0.750` and, when outside, publishes the exact endpoint position hold for a bounded 5 s recovery window. Core also checks a STOPPED_HOLD against fresh execution state, known-free world and the same 0.750 m anchor bound before exposing it. An expired BACKUP/EMERGENCY endpoint may be retained as an endpoint witness; this does not extend the polynomial lease or imply mission acceptance. MissionProgress alone accepts the measured terminal waypoint.

Therefore a `STATUS_COMPLETED` sample may precede physical stopping. No analysis or change on this branch treats it as measured completion or relaxes either 0.750 m gate. The source points are `navigation_runtime_node.cpp` near lines 1045–1135, 8669–8712, 8822–8898 and 9106–9136, and `navigation_mode_node.cpp` near lines 1269–1287 and 2412–2508 at the pinned base.

