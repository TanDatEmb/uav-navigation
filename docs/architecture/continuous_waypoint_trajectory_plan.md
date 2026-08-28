# Continuous waypoint trajectory architecture plan

**Status:** implementation in staged checkpoints; route/progress, command
continuity, explicit visibility evidence, native 3D visibility production, and
bounded pass-through corner route-window endpoint are implemented. Full
multiwaypoint SITL and dataset evidence remain open.

This document is the implementation plan for the high-speed, multi-waypoint
navigation behavior. It is intentionally broader than a parameter-tuning
exercise: the current failure is a mismatch between route ownership,
time-horizon management, trajectory representation, and map evidence.

## 1. Problem statement and current evidence

The current runtime gives the planner one mission waypoint at a time. A
`PASS_THROUGH` goal supplies the next waypoint only as terminal-tangent
metadata; the active waypoint remains the trajectory endpoint. This preserves
waypoint acceptance, but creates the following failure at speed:

1. The vehicle approaches the active waypoint while the planner keeps a fixed
   short replan lead and a fixed receding prefix.
2. The remaining guide path shrinks with the distance to the endpoint.
3. The incoming velocity is still high, while one MINCO polynomial is asked to
   satisfy a short endpoint and a non-zero outgoing velocity.
4. The optimizer fails dynamic/corridor checks or produces repeated hot-replan
   churn. The committed command is eventually exhausted and the runtime
   correctly fails closed.

The latest structured three-column run is a concrete example: at roughly
3 m/s, `guide_path_length_m` decreased from about 4.9 m to 0.84 m while the
planning-state speed remained about 3.1 m/s. Removing the earlier endpoint
extension fixed waypoint capture, but exposed the missing multi-segment
handoff. The earlier extension experiment was also rejected because a single
high-order polynomial bowed around the active waypoint and missed its
acceptance ball.

There is a separate evidence problem. The strict backup contract requires
`KNOWN_FREE`, while the current `RegisteredScan` carries only returned XYZ
points. A return-only point cloud cannot represent open/no-return beams, so
the mapper cannot certify all open future space from that message alone. This
must be solved as an explicit sensing contract, not by treating `UNKNOWN` as
free or clearing a larger arbitrary neighborhood.

## 2. Safety invariants and non-goals

The following are invariant across every phase:

- `UNKNOWN` and `OUT_OF_MAP` remain non-traversable for the certified backup.
- A candidate is executable only after atomic world, goal, localization, and
  command-lease identity checks.
- Waypoint acceptance is based on measured vehicle state, never on a planned
  endpoint alone.
- A new route segment may be planned speculatively, but it may not become the
  active mission checkpoint until the current checkpoint is measured accepted.
- Every executable segment and backup suffix is independently swept-volume
  validated against the pinned world snapshot.
- PX4 receives position, velocity, and acceleration setpoints with continuous
  boundary state. Jerk remains an internal planning/diagnostic quantity unless
  a target PX4 contract explicitly supports it.
- No hard gate, unknown-space policy, or deadline is relaxed to increase map
  completion rate. Threshold changes require distributions from representative
  recorded data and repeated SITL.

The plan does not assume that SUPER, MINCO, ROG-Map, or PX4 must be replaced.
It first changes the ownership and contracts around them. A replacement
algorithm is considered only if the staged architecture cannot meet the
measured acceptance matrix.

## 3. Target architecture

### 3.1 Route owner and progress model

Introduce a route/mission planning view that owns the validated waypoint
polyline and exposes:

- active waypoint identity and acceptance policy;
- projection of measured position onto the current route segment;
- monotonic arc-length progress with bounded backtracking tolerance;
- outgoing segment and a bounded route look-ahead window;
- stop, pass-through, and corner-transition policies;
- an altitude profile, including recovery after an avoidance excursion.

The route owner is the only component allowed to advance a waypoint. Runtime,
planner, and PX4 adapters consume the same identity and do not infer progress
from a trajectory endpoint.

### 3.2 Adaptive temporal look-ahead

The planner should target a route window, not an isolated point. The window is
computed from measured speed, dynamic limits, braking distance, solve/replan
latency, map freshness, and the available certified free-space horizon:

```text
lookahead distance >= braking distance
                    + speed * (solve budget + transport margin)
                    + continuity margin
```

The result is clamped by the route and map window. When the route window cannot
be certified, the planner shortens the *new candidate* and retains only a
previously certified command or a certified emergency brake. It never crosses
the unknown-space boundary to manufacture look-ahead.

### 3.3 Piecewise trajectory at waypoint boundaries

The executable nominal trajectory is a sequence of certified pieces:

```text
measured PVAJ -> active waypoint -> route look-ahead -> terminal/renewal point
```

The active waypoint is an exact mission-identity boundary. For a genuine
pass-through corner, the planner may represent that boundary with a bounded
three-point fillet entirely inside the mission acceptance ball: incoming entry,
outgoing blend, and outgoing endpoint. The handoff contract requires
continuous P/V/A, bounded J, and an explicit corner velocity policy. Each
piece has its own corridor and world swept certificate; the complete candidate
also has one atomic validity interval and route identity.

This prevents a single high-order polynomial from bending around an accepted
waypoint. It also permits replanning the future piece without changing the
already accepted boundary piece, subject to whole-candidate recertification.

The existing `Trajectory`/MINCO implementation can be reused as the piece
solver. The current checkpoint introduces the first route-window boundary:
when a pass-through guide reaches a genuine corner, the terminal point may be
placed inside the active acceptance ball on the outgoing tangent. The measured
mission acceptance contract remains authoritative, and the route-window
segment is checked before the existing corridor/MINCO/backup certificates.
The required long-term change is still route-aware piece assembly, not an
immediate optimizer replacement.

### 3.4 Mapping evidence contract

Add an optional, timestamped and frame-validated free-space ray representation
to the registered observation contract. A ray must identify its origin,
endpoint/range, hit/no-return state, sensor epoch, and source sequence. The
mapper marks only the explicitly observed free portion of each ray; hit points
retain occupied-endpoint semantics.

Return-only `PointCloud2` remains valid input for mapping and performance
replay, but it does not gain no-return semantics by inference. If ray evidence
is absent, strict backup certification remains fail-closed. The simulator must
bridge the actual scan/ray source, and real Mid-360 ingestion must provide an
equivalent producer before this evidence can be used for certification.

For the Gazebo Mid-360, the producer is a native `gz.msgs.LaserScan` adapter.
The generic `ros_gz_bridge` LaserScan converter is not a valid 3D path: the
message carries 720 x 28 flattened rays while ROS `LaserScan` has no vertical
axis, and the installed converter crashes when given that shape. The adapter
therefore validates the complete 3D metadata/array, keeps only explicit
max-range or positive-infinity rays, emits a lidar-frame `PointCloud2`, and
uses deterministic bounded beam sampling (currently 4096 endpoints) to keep
mapping work bounded. Any malformed, incomplete, empty, or mismatched message
produces no visibility evidence.

### 3.5 PX4 handoff and altitude behavior

The PX4 adapter continues to publish position/velocity/acceleration in the
existing frame/time contract. At a piece boundary it checks that the next
sample is consistent with the measured state and that velocity does not jump.
The route altitude profile is planned as part of the same piecewise trajectory;
avoidance-induced altitude deviation is recovered with bounded vertical
acceleration/jerk rather than a separate z snap or an unvalidated shortcut.

## 4. Staged implementation and commit boundaries

Each phase ends with focused tests, a clean build/provenance check, and one
commit. If a phase fails its acceptance criteria, revert to the immediately
preceding commit and keep the failure artifact.

### Phase 0 — baseline and observability freeze

**Purpose:** establish a comparable baseline before behavior changes.

Record, for repeated 3-column missions and representative recorded data:

- waypoint acceptance/completion and fail-closed reason;
- guide length/duration, route progress, command generation and world revision;
- speed, acceleration, jerk, altitude error/recovery, cross-track error;
- mapping, snapshot, A*, corridor, nominal, backup and publish latency p50/p95/p99;
- known-free certificate failure stage and first blocked cell;
- PX4 mode/health/odometry freshness and command gaps.

**Commit:** `chore: freeze waypoint stability baseline and metrics`.

### Phase 1 — route/progress contract

Add a pure route-progress API and contract tests. It must cover projection,
monotonic progress, duplicate/degenerate waypoints, corner direction,
acceptance-ball policy, and altitude profile lookup. Integrate it into mission
identity handling without changing planner geometry yet.

**Commit:** `feat: add route progress and waypoint boundary contract`.

### Phase 2 — piecewise boundary representation

Add a planner-side segment/boundary representation and candidate metadata for
exact waypoint boundaries. Reuse current MINCO pieces, assemble only when P/V/A
continuity and per-piece certificates pass, and test a corner where a single
polynomial would bow outside the acceptance ball. The current staged
implementation adds a bounded acceptance route-window endpoint for genuine
pass-through corners; exact-goal behavior remains the fallback when the
endpoint or its predecessor segment is not certified. The full multi-piece
boundary representation is still required before release.

**Commit:** `feat: assemble certified piecewise trajectories at waypoints`.

### Phase 3 — adaptive look-ahead and hot-replan policy

Use route progress and measured PVA to select a temporal/spatial look-ahead.
Separate “refresh/recertify the current piece” from “advance the future piece”.
Do not repeatedly solve an endpoint whose remaining distance is shorter than
the continuity/braking requirement. A retained command is allowed only while
the existing world/anchor/lease certificate is still valid.

The current corner checkpoint uses the acceptance ball as a bounded route
window, rather than extending a trajectory beyond the active waypoint. It
must be replaced or subsumed by a measured, route-aware look-ahead policy that
handles arbitrary corner radius, obstacle detours, and multi-piece handoff.

Validate speed recovery, reduced hot-replan churn, continuous velocity, and no
regression in measured waypoint acceptance.

**Commit:** `feat: use adaptive route lookahead for pass-through planning`.

### Phase 4 — explicit free-space ray evidence

Extend contracts and adapters end-to-end: sensor producer, ROS message,
mapping observation, ROG update, immutable snapshot diagnostics, replay parser,
and simulator bridge. Add negative tests for missing, stale, mismatched-frame,
and non-monotonic ray evidence. Keep the strict backup policy unchanged.

**Commit:** `feat: carry explicit free-space rays for backup certification`.

### Phase 5 — altitude profile and recovery

Integrate vertical route references into the piecewise planner. Require
bounded altitude error and recovery after a horizontal avoidance detour, with
the same world/dynamic/certificate checks as horizontal motion. Do not add an
independent vertical gate without an owner and distribution evidence.

**Commit:** `feat: preserve and recover route altitude through avoidance`.

### Phase 6 — measured performance optimization

Profile by cloud density, occupied voxels, corridor planes, route-piece count,
and solve outcome. Optimize duplicate copies/queries, map snapshot export,
queue replacement, and bounded optimizer retries only after Phase 0 metrics are
available. Keep correctness/observability changes separate from behavior and
do not increase deadlines as a substitute for profiling.

**Commit:** `perf: reduce measured mapping and planning tail latency`.

### Phase 7 — acceptance and release evidence

Run unit/contract tests, sanitizer builds, recorded-data shadow planning, then
repeated SITL at a speed ladder (2/3/4/5 m/s or the declared product cap) on
the 3-column multi-waypoint map. Acceptance requires repeated complete runs,
not one pass:

- all waypoints accepted in order and mission complete;
- no collision and no unexplained safety stop;
- bounded speed/acceleration/jerk distributions and velocity continuity;
- altitude tracking and recovery distributions;
- clearances and strict backup-certificate evidence;
- fresh propagated odometry, PX4 state, and command lease throughout;
- p50/p95/p99 latency and scheduling-gap evidence within the declared budget;
- final commit/build manifest points to the exact tested HEAD with a clean tree.

**Commit:** `docs: record repeated waypoint stability acceptance evidence`.

## 5. Decision gates and rollback rules

Advance only when the phase-specific unit/contract tests and the relevant
runtime evidence pass. A failed optimizer or missing ray source is evidence to
shorten the candidate or fail closed, not permission to use `UNKNOWN` as free.

Rollback is by the last phase commit. Never mix a route behavior change with a
new hard-gate value, a temporary bypass, or a large observability refactor. Any
temporary experiment must be recorded in
`runtime_safety_decision_ledger.md` with owner, scope, impact, evidence,
removal condition, and verification command.

The project is not flight-ready merely because dataset shadow planning or one
SITL speed passes. The release claim is made only after the full Phase 7
evidence matrix is archived.
