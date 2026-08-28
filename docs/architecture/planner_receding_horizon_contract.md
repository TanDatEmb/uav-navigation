# planner backend receding-horizon contract

The current runtime uses planner backend's in-process committed trajectory state. The
ROS adapter must preserve that state rather than inventing a second bundle or
horizon transport.

At each planning timer tick:

1. A new waypoint or invalidated command starts `PlanFromRest`. A valid MAIN
   is re-certified on each published map snapshot but does not run the
   optimizer on every timer tick.
2. planner backend replans from a short future state (`replan_forward_dt_s: 0.2`) and
   retains a bounded prefix (`receding_distance_m: 3.0`) so position, velocity, and
   acceleration remain continuous. `ReplanOnce` is scheduled when the time to
   the certified BACKUP switch (or a main-only endpoint) is no greater than
   one scheduler period plus one solve deadline plus the replan-forward
   interval. Goal transitions, recovery, missing commands and malformed
   horizon metadata bypass this deferral gate.
3. ROG-Map and the planner validate the committed path against the current
   inflated map. A hot-replan failure may retain a valid backup suffix.
4. If the committed suffix cannot be anchored to fresh propagated odometry,
   a hot solve first drops its historical stitch and builds the replacement
   from fresh measured PVAJ in the same solve. The previous immutable command
   remains exposed until that replacement is atomically certified. If no
   replacement is available, the runtime may retain a certified suffix or
   commit an emergency brake using measured position and velocity; otherwise
   it removes the command and fails closed.
5. The active mission waypoint remains the mission-identity target. A
   `PASS_THROUGH` goal carries `next_target` as route metadata; when the
   certified map horizon permits, the planner extends the executable guide
   through a bounded prefix of the outgoing segment for straight, shallow, and
   genuine-corner legs. The corridor generator inserts a route-boundary
   junction contract at the active waypoint and preserves its marker through
   a hard acceptance-region cell. A successful hot retarget clears its
   one-shot transition flag, so later timer ticks retain that exact certified
   MAIN until normal horizon renewal is due instead of repeatedly moving the
   future junction.
   SFC simplification. The adjacent MINCO junction is held at the recorded
   waypoint while optimizing, then independently hard-checked against the
   waypoint acceptance ball; the surrounding collision-certified corridor
   remains wide enough for a smooth fillet. The MINCO hot seed assigns the
   overlap after that boundary to the first post-waypoint guide timestamp (or
   splits the remaining interval when the look-ahead endpoint is that only
   post-waypoint sample), so the hard boundary does not create a near-zero-
   duration turn piece. A corner uses an acceptance-room terminal speed cap;
   straight/shallow legs retain the outgoing tangent for continuity. The
   MissionController still accepts the current waypoint only from measured
   position inside its configured acceptance radius.

The relevant runtime bounds are:

| Contract | Current value |
|---|---:|
| Planning loop | 5 Hz |
| Command sampling | 50 Hz |
| Input pair maximum skew | 0.1 s |
| Input maximum age | 0.5 s |
| Planner solve timeout | 0.18 s |
| Replan-forward interval | 0.2 s |
| Receding distance | 3.0 m |
| Planning horizon | 45.0 m |

The current ROG-Map geometry is an axis-aligned ENU AABB. Its local window
translates with the map origin and has no yaw/orientation field. The planned
route-oriented window is therefore a planner query/selection frame; it must
project selected ENU points into the existing ROG evidence contract rather
than rotate voxel storage or snapshot indices. Replay may draw a rotated
window only from validated runtime frame metadata.

This contract is implemented by `navigation_runtime_node`,
`planner.yaml`, and the PX4 External Mode command boundary. It does not
define `TrajectoryBundle`, `WorldSnapshot`, or a separate controller node.
