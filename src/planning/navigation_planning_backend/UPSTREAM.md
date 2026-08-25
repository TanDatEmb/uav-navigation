# SUPER planner provenance

The planner core in this package is ported from `hku-mars/SUPER` at commit
`2ad3419c127a617c6d7df6925e81a14175a9c096`. ROS 1/ROS 2 visualization,
simulator, FSM and research logging wrappers are intentionally excluded from
the realtime target. The product adapter keeps the upstream planner data
structures and optimization path while using the workspace-owned ROG-Map
implementation and ROS 2 input/output boundaries.

The imported source retains its upstream copyright and license headers. Any
future modification must preserve the corresponding provenance and license
notices.

## Product integration deltas

The vendored planner deliberately differs from upstream in a bounded set of
core files. Those files are enumerated by `tools/check_planner_backend_parity.py`; a
change in any other core file fails the parity audit. The documented changes
fall into these categories:

- ROS 2 and workspace dependency portability (headers, logging, YAML loading,
  ROG-Map adapter types, and removal of ROS 1-only dependencies).
- Bounded A* cancellation and time slicing so both searches fit inside the
  realtime replan deadline. Point-to-point and escape searches prefer the
  start/goal altitude slab before an unrestricted 3-D retry. Under
  `UNKNOWN_AS_FREE`, an already non-occupied exact start returns `NO_NEED`
  instead of being quantized to a lower voxel centre on every replan.
- Collision-checked mission-altitude projection of A* lateral detours, with
  fallback to the original 3-D route whenever vertical avoidance is required.
- Atomic main-plus-backup publication, cancellation propagation, and explicit
  planner return codes required by the runtime safety FSM.
- SUPER-owned measured-state emergency braking when a previously committed
  suffix is no longer trackable. Position/velocity are refreshed from LIO,
  unmeasured acceleration/jerk retain the command boundary and are saturated
  to mission limits, and the new BACKUP-only bundle is committed atomically
  only after dense inflated-map and V/A/J/body-rate/thrust checks.
  A physically unavoidable measured overspeed is retained at the polynomial
  boundary and certified against its initial speed rather than being hidden by
  clamping the measured velocity to the mission limit.
- Backup visibility checks against the inflated occupied tube, while retaining
  upstream optimistic UNKNOWN-space semantics when raycasting is disabled.
- Configuration validation for inflation radius, braking visibility horizon,
  and planner deadline relationships.
- Optimizer finite-value, dynamic-limit, corridor, vertical-guide-envelope,
  and cancellation post-checks used to
  prevent an invalid trajectory from reaching the PX4 command boundary.
  Mission velocity, acceleration, and jerk are strict command limits;
  optimizer penalty margin cannot raise them.
- Analytic velocity, acceleration, and jerk extrema for polynomial pieces,
  bounded full MINCO feasibility re-optimization with an A*-seeded spatial
  trust cost, at most two point-and-time L-BFGS retries, and fail-closed hard
  gates after every candidate. Retry one preserves nominal dynamic penalties;
  retry two strengthens only configured penalties whose dimension still
  violates its hard bound. Segment duration floors are additive optimizer
  bounds rather than fixed-point scaling of a polynomial with non-zero PVAJ.
  The vertical guide envelope prevents a wide SFC from creating metre-scale
  uncommanded altitude extrema while retaining a small solver transition band.
  These changes add no mission YAML tuning parameter.

  The product-only
  `include/planner_core/backup_braking.hpp` helper. The backup seed uses the full
  PVAJ state at the latest visibility-derived switch whose complete braking
  hull fits the safety corridor, and the natural free-end minimum-snap stop
  endpoint. Its Bezier hull certifies containment in the convex backup SFC and
  initializes every MINCO piece. Duration and endpoint therefore move together
  instead of stretching time against a fixed point. The certified seed remains
  the fallback if optional L-BFGS refinement fails, and its duration is a hard
  lower bound enforced by the product delta in
  `include/traj_opt/backup_traj_optimizer_s4.h`. The product backup retains
  a positive jerk objective plus an analytic post-solve gate; the larger EXP
  optimizer uses the analytic gate without the destabilizing jerk objective.
  Refinement can only preserve or increase certified seed duration; failure
  returns the seed unchanged and is reported through success/fallback counters.

- The product removes the unused `ExpTraj::flag_whole_known_free_` storage and
  accessors. The flag had no live writer or reader in the product runtime; this
  is an API cleanup, not a change to trajectory feasibility or known-free
  policy.

- `GuideEndpoint` uses an inclusive connection boundary so the planner and
  runtime agree when the three-dimensional goal error is exactly the shared
  completion tolerance.

Run the audit against the pinned checkout from the repository root:

```bash
python3 tools/check_planner_backend_parity.py --upstream-dir /path/to/SUPER
```

The argument is the upstream repository root, not its `navigation_planning_backend`
subdirectory. A documented delta is reported but accepted; missing files,
newly changed files outside the allowlist, or a wrong upstream commit fail.
SUPER_PORT_SHA256.json pins the reviewed content of every differing or
product-only core file, so allowlist membership alone cannot accept later
unreviewed drift.
