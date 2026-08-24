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
core files. Those files are enumerated by `tools/check_super_parity.py`; a
change in any other core file fails the parity audit. The documented changes
fall into these categories:

- ROS 2 and workspace dependency portability (headers, logging, YAML loading,
  ROG-Map adapter types, and removal of ROS 1-only dependencies).
- Bounded A* cancellation and time slicing so both searches fit inside the
  realtime replan deadline.
- Atomic main-plus-backup publication, cancellation propagation, and explicit
  planner return codes required by the runtime safety FSM.
- SUPER-owned measured-state emergency braking when a previously committed
  suffix is no longer trackable. Position/velocity are refreshed from LIO,
  unmeasured acceleration/jerk retain the command boundary and are saturated
  to mission limits, and the new BACKUP-only bundle is committed atomically
  only after dense inflated-map and V/A/J/body-rate/thrust checks.
- Backup visibility checks against the inflated occupied tube, while retaining
  upstream optimistic UNKNOWN-space semantics when raycasting is disabled.
- Configuration validation for inflation radius, braking visibility horizon,
  and planner deadline relationships.
- Optimizer finite-value, dynamic-limit, and cancellation post-checks used to
  prevent an invalid trajectory from reaching the PX4 command boundary.
  Mission velocity, acceleration, and jerk are strict command limits;
  optimizer penalty margin cannot raise them.
- Analytic velocity, acceleration, and jerk extrema for polynomial pieces,
  main-trajectory time feasibility projection, and the product-only
  `include/super_core/backup_braking.hpp` helper. The backup seed uses the full
  PVAJ state at the latest visibility-derived switch whose complete braking
  hull fits the safety corridor, and the natural free-end minimum-snap stop
  endpoint. Its Bezier hull certifies containment in the convex backup SFC and
  initializes every MINCO piece. Duration and endpoint therefore move together
  instead of stretching time against a fixed point. The certified seed remains
  the fallback if optional L-BFGS refinement fails. The product backup retains
  a positive jerk objective plus an analytic post-solve gate; the larger EXP
  optimizer uses the analytic gate without the destabilizing jerk objective.

Run the audit against the pinned checkout from the repository root:

```bash
python3 tools/check_super_parity.py --upstream-dir /path/to/SUPER
```

The argument is the upstream repository root, not its `super_planner`
subdirectory. A documented delta is reported but accepted; missing files,
newly changed files outside the allowlist, or a wrong upstream commit fail.
SUPER_PORT_SHA256.json pins the reviewed content of every differing or
product-only core file, so allowlist membership alone cannot accept later
unreviewed drift.
