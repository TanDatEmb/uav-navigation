# Dual planning for unknown space

Status: nominal/safety candidate generation and dual verification are integrated
into `navigation_runtime`, but optimistic unknown-space execution is gated by
`navigation.planner.allow_nominal_unknown`. The real and default simulation
profiles keep that flag `false`.

## Decision

The planner must produce two different products when optimistic navigation is
eventually enabled:

```text
nominal candidate: may use bounded unknown space to make progress
safety candidate:  known-free only, dynamically stoppable, always executable
```

The nominal candidate is never accepted merely because A* found a geometric
route. It needs a bounded commitment horizon, a swept-volume collision check,
and a valid safety candidate. If any check fails, the runtime publishes the
safety candidate or fails closed.

The current implementation provides the safety side as a known-free route, with
a dynamically stoppable braking trajectory as the last fallback. It integrates
`TrajectoryVerifier` into the runtime before a trajectory is published. With the
explicit simulation-only gate enabled, the runtime generates both candidates
and calls `verifyDual()`; a nominal result is selected only when the safety
result and both swept-volume checks are valid. A safety route is a valid
waypoint trajectory; a braking stop is never treated as waypoint progress.
When the gate is disabled, the runtime remains on the known-free baseline.

The only simulation startup exception is `allow_unknown_start`: it may trust
the current vehicle footprint when the LiDAR mount leaves the body in a sensor
shadow. The simulation runner creates a non-persistent `KnownFree` overlay of
radius `0.37 m` (vehicle radius plus safety margin). It converts only `Unknown`;
`Occupied` evidence wins and every voxel outside that small footprint remains
`KnownFree`-only for safety route and braking-stop validation. The exception is
rejected when the current cell is occupied, the state is stale, or map
generation/revision is no longer current.

Run the experiment explicitly with:

```text
DUAL_PLANNING=1 make external-mode-check
```

This flag is not a flight permission. It is a simulation test profile used to
measure nominal selection, safety fallback, verifier failures, and map-revision
rejection. It must not be enabled for real flight until the revealed-obstacle
and stop-distance gates below are satisfied.

## Reference interpretation

- FASTER is the closest reference for the dual-output safety idea: the planner
  can optimize through unknown space while retaining a safe backup in
  free-known space. The backup is a runtime invariant, not a best-effort
  fallback ([paper](https://arxiv.org/abs/2001.04420)).
- FIRI is a convex-region/corridor construction method. Its value here is
  producing manageable convex free-space regions around a seed path; it does
  not classify unknown voxels as free and must not be applied to already
  inflated points without an explicit clearance contract
  ([paper](https://arxiv.org/abs/2403.02977)).
- Safe Local Exploration supports bounded intermediate goals and conservative
  replanning when a local planner is stuck ([paper](https://arxiv.org/abs/1710.00604)).
- No authoritative CIRI paper or implementation is currently identified in
  this repository or the pinned references. The name must not be treated as an
  algorithm specification until its exact paper/repository is supplied.

## Proposed runtime architecture

1. Keep the current `navigation_runtime` node and A* known-free planner.
2. Add a planner result with explicit `NOMINAL` and `SAFETY` roles, map
   generation/revision, request ID, and a commitment horizon.
3. Build the safety route first. It must be sampled against inflated
   `KnownFree` cells and respect velocity/acceleration limits. If no route is
   available, build a braking stop in the currently known-free swept volume;
   it must terminate at zero velocity and acceleration.
4. Build a nominal candidate only for a short horizon. Unknown cells may be
   considered by this candidate, but occupied and inflated occupied cells are
   always blocked.
5. Verify the nominal swept volume and its smoothed trajectory. A verifier
   failure, stale map revision, planner timeout, or missing safety candidate
   selects `SAFETY` and never publishes the nominal result.
6. Use FIRI-like convex regions only after raw obstacle geometry and the
   vehicle-clearance model are defined. Every generated corridor must retain
   the seed path and be rechecked against the current map revision.
7. External Mode continues to consume one correlated trajectory contract. It
   does not know how the planner was implemented and never chooses between
   nominal and safety based on a loose parameter.

## Required gates before implementation

- Unit tests for unknown/occupied/known-free classification and map revision
  invalidation.
- Regression tests where the current voxel is unknown but the next braking
  voxel is unknown; the stop must fail closed. The current voxel exception must
  not become a free-space radius.
- Deterministic tests where nominal planning succeeds but safety planning does
  not; result must be `FAILED`, not nominal-only flight.
- Deterministic tests where the nominal route enters unknown and the safety
  stop remains known-free; only the committed prefix may be published.
- Swept-volume tests through a narrow opening and around the pillar scenario.
- Runtime metrics for commitment length, nominal-to-safety switches, stop
  distance, verifier latency, stale-map rejects, and trajectory discontinuity.
- Simulation stages: open textured scene, single pillar, unknown wall revealed
  at the commitment boundary, then multi-obstacle route. The runner exposes
  these as `MAP_PROFILE=open|pillar|occlusion`; real profile remains
  `unknown_policy: blocked` until all gates pass.

The implementation deliberately stops at the smallest useful experiment: a
simulation-only nominal candidate producer, dual verifier, and runtime metrics.
It does not add a full FIRI/MPC port or a general planner plugin framework. The
data contract is in `navigation_planning/trajectory_verifier.hpp`; map revision
correlation and known-free runtime verification remain active in every profile.
Real/default flight profiles remain fail-closed until the nominal candidate and
safety route/stop fallback are exercised against revealed-obstacle scenarios.
