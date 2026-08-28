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
   certified map horizon permits and the outgoing tangent is a genuine corner,
   the planner extends the executable guide through a bounded prefix of that
   outgoing segment. The corridor generator inserts a bounded route-boundary
   gate at the active waypoint and preserves it through SFC simplification, so
   a convex corridor cannot cut across the waypoint. The MINCO hot seed assigns
   the overlap after that gate to the first post-waypoint guide timestamp (or
   splits the remaining interval when the look-ahead endpoint is that only
   post-waypoint sample), so the hard boundary does not create a near-zero-
   duration turn piece.
   Shallow/straight legs
   terminate at the active waypoint so the nominal curve cannot trade away its
   measured acceptance boundary for a soft look-ahead endpoint. The
   MissionController still accepts the current waypoint only from measured
   position inside its configured acceptance radius.

The relevant runtime bounds are:

| Contract | Current value |
|---|---:|
| Planning loop | 10 Hz |
| Command sampling | 50 Hz |
| Input pair maximum skew | 0.1 s |
| Input maximum age | 0.5 s |
| Planner solve timeout | 1.0 s |
| Replan-forward interval | 0.2 s |
| Receding distance | 3.0 m |
| Planning horizon | 45.0 m |

This contract is implemented by `navigation_runtime_node`,
`planner.yaml`, and the PX4 External Mode command boundary. It does not
define `TrajectoryBundle`, `WorldSnapshot`, or a separate controller node.
