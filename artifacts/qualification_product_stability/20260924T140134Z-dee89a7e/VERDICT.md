# Product Stability Gate verdict

**`PARTIAL_PRODUCT_STABILITY` — not merge-ready as a frozen behavior baseline.**

## Track A — terminal recovery

The two predecessor nominal failures are exact measured-state gate events, 0.762/0.759 m versus 0.750 m, in an EMERGENCY-derived STOPPED_HOLD. `STATUS_COMPLETED` means analytic candidate completion, not measured mission acceptance. Command endpoint identity stayed fixed and the adapter continued endpoint position hold. LIO speed at rejection was 0.580/0.863 m/s; evaluation-only truth speed also rose to 0.600/0.879 m/s. LIO displacement differed from locally aligned truth displacement by 0.058/0.044 m, so the physical position threshold crossing is not independently proved. The immediate category is `ACTUAL_TRACKING_ERROR` in the authoritative LIO execution frame. Why the closed-loop vehicle/estimator diverges under this stop/hold, and whether the emergency envelope is achievable, remain unresolved. No threshold, controller, planner, terminal gate or PX4 behavior was changed without that evidence. The historical 8/10 COMPLETE cohort, despite eight consecutive completions, remains a reliability failure. There is no post-fix ten-run cohort because there is no justified fix yet.

## Track B — temporal attribution

The historical 481.677 ms accepted-state event had matching ROS clock/IMU slowdown but lacked native/host witnesses; exact first component remains **unresolved historically**. New optional diagnostics recorded one natural pilot with accepted-state gaps 784.659 and 269.229 ms. Native clock/stats source-progress deficits exceeded 150 ms while the observer loop remained scheduled, so both are classified `GAZEBO_SIMULATION_STALL` at the **first observed layer**. That label does not prove a Gazebo code defect or why its simulator stopped progressing. The pilot later had an emergency/hot-replan failure and Hold; no exact causal link from the earlier tail to that later outcome was established. One no-native control and two other native runs completed with maxima 26.813/27.969/29.733 ms; the comparison cannot establish or exclude observer perturbation.

A controlled 350.272 ms pause of the exact Gazebo server PID reproduced a 373.874 ms state gap, native progress deficit and `ODOMETRY_STALE` → PX4 Hold safety response. A separate 350.150 ms pause of the exact Core runtime PID kept odometry live (max 27.669 ms), but produced stale PVA → PX4 Hold. The runner's `FAILED_COMPONENT` and `PAUSED_SAFETY_STOP` labels are retained for these injected faults. The 100 ms command lease, 200 ms state boundary and 500 ms world freshness are unchanged.

## Gate decision

- Safety: no gate relaxed; real command or state starvation still transfers to Hold. Mission, desired, execution, world and PX4 authority owners unchanged.
- Reliability: historical 2/10 terminal failures still open; new natural diagnostic pilot also failed after a later emergency/hot-replan event.
- Causal attribution: newly reproduced temporal tails have an observed native simulation first layer; historical exact cause and Gazebo-internal mechanism remain open.
- Qualification: evaluator acceptance semantics untouched; all source reports' `NOT_EVALUABLE` statuses retained. These runs do not establish Q3, HITL or flight qualification.
- **`BEHAVIOR_BEARING_SHA`: not frozen.** Product C++/flight configuration bytes equal base `dee89a7e`; this branch adds only optional Python diagnostics/offline evidence. A stable product behavior SHA cannot be declared under the user's gate while terminal reliability remains uncorrected.

Next work is a bounded terminal stopping/estimator experiment with exact PX4 setpoint and measured/truth dynamics, followed by a product-owned fix only if causally justified and a fresh ≥10-run nominal cohort. The separate `qualification-evidence-contract-closure` branch should wait for a behavior freeze.
