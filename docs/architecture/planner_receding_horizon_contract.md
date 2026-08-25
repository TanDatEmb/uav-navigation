# planner backend receding-horizon contract

The current runtime uses planner backend's in-process committed trajectory state. The
ROS adapter must preserve that state rather than inventing a second bundle or
horizon transport.

At each planning tick:

1. A new waypoint starts `PlanFromRest`; later ticks use `ReplanOnce`.
2. planner backend replans from a short future state (`replan_forward_dt: 0.2`) and
   retains a bounded prefix (`receding_dis: 3.0`) so position, velocity, and
   acceleration remain continuous.
3. ROG-Map and the planner validate the committed path against the current
   inflated map. A hot-replan failure may retain a valid backup suffix.
4. If the committed suffix cannot be anchored to fresh propagated odometry,
   the runtime may commit an emergency brake using measured position and
   velocity. Otherwise it removes the command and fails closed.
5. The active mission waypoint remains the terminal target. A
   `PASS_THROUGH` goal carries `next_target` as directional metadata; it does
   not skip the current waypoint.

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

This contract is implemented by `navigation_runtime_node`,
`planner.yaml`, and the PX4 External Mode command boundary. It does not
define `TrajectoryBundle`, `WorldSnapshot`, or a separate controller node.
