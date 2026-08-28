# planner backend receding-horizon contract

The current runtime uses planner backend's in-process committed trajectory state. The
ROS adapter must preserve that state rather than inventing a second bundle or
horizon transport.

At each planning tick:

1. A new waypoint starts `PlanFromRest`; later ticks use `ReplanOnce`.
2. planner backend replans from a short future state (`replan_forward_dt_s: 0.2`) and
   retains a bounded prefix (`receding_distance_m: 3.0`) so position, velocity, and
   acceleration remain continuous.
3. ROG-Map and the planner validate the committed path against the current
   inflated map. A hot-replan failure may retain a valid backup suffix.
4. If the committed suffix cannot be anchored to fresh propagated odometry,
   the runtime may commit an emergency brake using measured position and
   velocity. Otherwise it removes the command and fails closed.
5. The active mission waypoint remains the mission-identity target. A
   `PASS_THROUGH` goal carries `next_target` as route metadata; when the
   certified map horizon permits, the planner extends the executable guide
   through a bounded prefix of the outgoing segment for straight, shallow, and
   genuine-corner legs. The corridor generator inserts a route-boundary
   junction contract at the active waypoint and preserves its marker through
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
| Planning loop | 10 Hz |
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
