# Open questions for later design discussion

These are evidence/contract questions, not architecture proposals. Do not carry an unconfirmed defect to a redesign backlog.

1. Provide exact deployment parameter dump and profile: clock mode, world freshness, tracking/health suppression, frames, mission, dynamic/planner limits, command rates/lease, RMW/QoS, PX4 adapter configuration. Constructor-validated values must correspond to deployed binary.
2. Provide source-time trace joining runtime bundle/generation/world revision/localization epoch, adapter receive/header/valid-until, measured odometry/health, setpoint publication and VehicleStatus. Needed before naming a real bottleneck or stating PX4 accepted a command.
3. Decide whether a cross-layer relation is required between runtime stopped-recovery retry budget and adapter completed-endpoint recovery lease. Code uses separate steady and ROS clocks/start events/owners/consumers.
4. If PASS_THROUGH liveness is required, specify bound/order among measured crossing, command callback, continuation reserve, state source/receive freshness and mission timer. Source has a local overwrite counterexample and immediate admitted-continuation update; product reachability is not shown.
5. To close world suspension, exercise actual mapping callback and adapter receiver lease in one controlled fixture: fresh/stale, unchanged/changed geometry, MAIN/BACKUP/completed endpoint, and new command before/after terminal latch. Existing tests do not join those boundaries.
6. Record desired postcondition after stationary recovery timeout and Hold request failure; distinguish API callback, status confirmation and operator authority in target traces.
7. For H7, collect matched request/solve/candidate/admission/activation/publish/receive/mission outcomes and stage latency/rejection reasons. Current audit gives verification priority only, not frequency or CPU ranking.

Before implementation/design work, resolve 1–3 for the target deployment or bound discussion to source contracts only.
