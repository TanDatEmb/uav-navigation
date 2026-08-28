# Continuous waypoint trajectory architecture plan

**Status:** architecture reset after the 2026-08-28 multi-waypoint failure.
Several route/progress, command-continuity, visibility, and bounded-look-ahead
primitives are implemented, but they do not yet form the product behavior
defined below. The current MINCO MAIN/backup bundle, route-derived yaw,
direction-independent map support, and full multi-waypoint acceptance remain
open. No focused primitive or dataset result is equivalent to this contract.

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

The 2026-08-28 nine-waypoint run adds a stronger causal boundary. It accepted
only waypoints 0 and 1. Repeated MAIN solves then failed acceleration, jerk, or
corridor certificates; the old committed bundle continued until its BACKUP
interval, and the PX4 boundary stopped on command-anchor divergence. The same
run showed a committed route-altitude excursion above 4 m for a 3 m route and
repeated measured-state rebase attempts. Mapping and planner latency contribute
to command-horizon pressure, but lowering latency alone cannot make the current
trajectory ownership correct.

The final BACKUP command samples were locally smooth; the safety stop was not a
single-sample jitter event. The deeper defect is that the committed brake began
near 4.10 m/s and later reached about 4.73 m/s while repeated MAIN failures left
no replacement. In addition, candidate commit diagnostics recorded nominal
generation residuals as large as approximately 0.636 m position, 0.591 m/s
velocity, and 2.093 m/s^2 acceleration without making those residuals a hard
replacement decision. Absolute world/dynamic feasibility is therefore being
checked more strongly than relative executable continuity.

There are also two PX4-adapter correctness blockers independent of optimizer
quality. Every pass-through `PublishGoal` currently clears the accepted
navigation command immediately; the 50 Hz publication path then emits a
stationary setpoint until a command with the next identity arrives. This can
insert a zero-velocity pulse at each waypoint even though runtime retains a
certified hot-retarget command. Separately, the adapter intentionally accepts
an exact late terminal `COMPLETED` sample, but its logging path can still call
`activeWaypoint()` after the controller index has advanced to
`waypoints.size()`. Both must be fixed before planner behavior is evaluated.

### 1.1 Why current MAIN and BACKUP are unstable

| Current mechanism | Failure consequence |
| --- | --- |
| MAIN uses one optimized polynomial for retained command prefix, local A* guide, waypoint boundary, outgoing look-ahead, altitude, and terminal PVA. | A change in any local target or corridor reshapes the whole command and can make a previously feasible route unavailable. |
| MINCO is the only nominal trajectory made executable; feasibility is stronger than route-quality ownership. | Non-finite cost or a small V/A/J/corridor violation discards the route instead of falling back to a certified deterministic trajectory. |
| Hot replans feed the previous executable command back into the next guide and may rebase after tracking/yaw error. | Planner shape error and altitude drift become history; repeated rebases change boundary conditions and increase solve churn. |
| Candidate commit records generation residuals but does not require a connector or reject a discontinuous nominal switch. | Each trajectory can be absolutely feasible while the setpoint stream presented to PX4 is relatively discontinuous. |
| A new MAIN normally requires a new BACKUP in one atomic bundle. If either solve fails, the old bundle remains active. | Repeated MAIN/BACKUP failures consume the old NOMINAL horizon until runtime enters an old brake or loses its command anchor. |
| BACKUP switch state is selected on the planned EXP command and its optional refinement is primarily bounded by position trace/corridor/world checks. | A terminally stopped backup can still have an unnecessarily high intermediate speed, altered time law, or poor altitude/tracking behavior. |
| Historical BACKUP intervals can be inherited into a later EXP prefix. | Safety role and nominal continuity history become entangled; planner recovery is no longer a clean measured-state transaction. |

The fail-closed atomic bundle and independent world certificates are sound and
remain. The redesign changes route, transition, and braking ownership rather
than weakening those safety properties.

## 2. Safety invariants and non-goals

The following are invariant across every milestone:

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

### 2.1 PX4 command ownership

The product publishes position, velocity, acceleration, yaw, and yaw-rate
setpoints to PX4. PX4 position control tracks those setpoints, but an externally
published trajectory setpoint suppresses PX4's internal `GotoControl`
trajectory generator. Jerk in the PX4 trajectory message is diagnostic, not an
executed setpoint. Therefore:

- PX4 owns closed-loop position/velocity/attitude tracking and its physical
  velocity, tilt, thrust, and body-rate limits;
- navigation owns the kinematic consistency and continuity of the P/V/A/yaw
  stream, including bounded acceleration slew at every generation boundary;
- PX4 Mission behavior is a design reference, not a hidden smoothing layer for
  this External Mode;
- a non-finite or discontinuous external trajectory may not be justified by
  assuming that PX4 will retime or repair it.

The upstream PX4 Mission design supplies previous/current/next waypoint context
for fly-through motion, removes the next point when a multicopter must hold, and
computes an auto-mode speed profile from distance, waypoint velocity,
acceleration, and jerk. This product must explicitly provide the equivalent
route context and speed policy before publishing external PVA.

### 2.2 Required operational behavior

The route and measured state determine behavior; optimizer success does not.

| Operating case | Required behavior |
| --- | --- |
| Start or resume from rest | Hold measured yaw until a finite horizontal route direction exists; generate a certified acceleration ramp from measured PVA. |
| Long straight leg in any horizontal direction | Track a bounded route window, accelerate toward the allowed cruise speed, and renew the future horizon before the committed command reaches its brake interval. A remote waypoint need not be inside the local map. |
| Straight or shallow pass-through | Keep continuous P/V/A and advance exactly once from measured acceptance. Yaw and route look-ahead move to the outgoing segment without a generation jump. |
| Sharp pass-through corner | Decelerate before the acceptance region to a speed that can produce a certified fillet through that region, then recover speed on the outgoing leg. Do not request full cruise at the corner. |
| Reversal or approximately 180-degree turn | Treat as STOP-TURN-GO: brake, hold position, rotate by the shortest commanded direction, then accelerate on the outgoing leg. A reversal is not a fly-through corner. |
| Obstacle detour | Follow the certified local detour while retaining monotonic mission-route progress. Recover route altitude and speed after clearance; do not advance a waypoint from a planned endpoint. |
| Temporary planner failure | Continue the currently certified NOMINAL interval only while its world, lease, and tracking activation envelopes remain valid. Otherwise activate the certified brake or hand over to PX4 Hold. |
| BACKUP activation | Brake from the certified activation state to a finite known-free stop. BACKUP never continues the mission, advances a waypoint, or becomes inherited NOMINAL history. |
| Final STOP waypoint | Enter the acceptance volume, reduce measured speed below the stop threshold for the confirmation interval, retain the last valid yaw, and then report completion. |
| Missing/stale map, state, route, or certificate | Fail closed. A shorter horizon, slower command, or PX4 Hold is valid; treating unknown space as free is not. |

Pass-through uses a two-phase command handoff. Measured crossing latches a
pending route transition, but the old execution identity and command remain
valid until a bundle for the next request is committed and accepted at the PX4
boundary. Route/goal identity and command identity are swapped atomically. The
old trajectory is never relabeled as the new goal, and a stationary setpoint is
published only when the retained command lease/certificate is no longer valid.

### 2.3 Yaw behavior contract

`FaceRouteLookahead` is the default mission yaw mode. It intentionally does not
point at the raw active waypoint coordinate: immediately after crossing that
coordinate, the raw vector reverses and can demand an approximately 180-degree
turn. Instead:

```text
L_yaw = clamp(max(L_min, horizontal_speed * T_yaw), L_min, L_max)
yaw_target = bearing(measured_position,
                     route_point(monotonic_progress + L_yaw))
```

The route point may lie on the outgoing segment before the active waypoint is
accepted, so the target identity does not change at the acceptance event. The
target is unwrapped with shortest angular distance and passed through explicit
yaw-rate and yaw-acceleration limits. The initial operating limits must not
exceed the active PX4 autonomous yaw limits recorded by the runner; upstream
defaults of 60 deg/s and 20 deg/s^2 are starting references, not unverified
vehicle certification. The current planner-only 3 rad/s rate cap is not an
acceptable stability argument by itself.

At low horizontal speed, pure vertical motion, a STOP hold, or missing finite
route support, the system holds the last valid yaw with zero requested yaw-rate.
An obstacle detour does not change mission yaw ownership; if a future vehicle
requires body-forward sensing, that is a separate explicit
`FaceCertifiedPathLookahead` mode and must not be introduced as an implicit
fallback.

### 2.4 Waypoint acceptance and continuous velocity

For a multicopter pass-through waypoint, the acceptance radius is both a
mission event volume and part of the available turning geometry. Acceptance is
measured and monotonic: the current sample is inside the ball, or the two most
recent finite measured samples cross the ball in forward route order. The
segment-crossing rule prevents a 50 ms mission timer from skipping a small
ball; it does not allow an unlimited passed-waypoint plane. STOP waypoints also
require measured low speed and confirmation time.

Acceptance radius is not derived from LiDAR range. Sensor range determines how
far free space can be evidenced; acceptance radius determines route fidelity
and turn room. Increasing it may enable a larger-radius, smoother corner, but it
also permits larger waypoint miss distance and is therefore a mission-design
choice with explicit evidence.

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

Mapping publication uses the same certificate ownership boundary. A committed
trajectory stores its conservative swept protected region. When immutable
change history proves that all newer map changes are disjoint from that region,
the runtime refreshes the world identity without re-sweeping the complete
trajectory. If provenance is missing, intersects the region, crosses an epoch,
or the command is expired, the existing full validation/fail-closed path is
used.

The mutable map update cadence and immutable snapshot publication cadence are
separate. Every admitted map update remains serialized and contributes one
revision plus a bounded change record. The actor may coalesce only snapshot
export: it accumulates the changed-region union and publishes the newest
revision at the configured period (200 ms in the canonical runtime profile),
immediately forcing a full export for a new localization epoch or a whole-map
change. A patch may therefore span multiple revisions; its history must cover
the complete interval or publication fails closed. Runtime freshness is measured
from the last published snapshot, while diagnostics distinguish map updates
from deferred snapshot exports.

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
placed inside the active acceptance ball on the outgoing tangent. Straight or
shallow pass-through legs instead receive a bounded certified outgoing guide
prefix while retaining the active waypoint as an explicit route-boundary gate.
The measured mission acceptance contract remains authoritative, and the
route-window segment is checked before the existing corridor/MINCO/backup
certificates. The required long-term change is still route-aware piece
assembly, not an immediate optimizer replacement.

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

### 3.6 Route-oriented planning window without rotating evidence storage

The ROG-Map local grid and immutable `WorldSnapshot` remain fixed in ENU. A
planner window must not be implemented by changing voxel axes when UAV yaw
changes: that would require resampling occupancy, invalidating cell indices,
rewriting changed-region history, and re-proving every snapshot/certificate.

The planner instead owns a separate `PlanningWindowFrame` with:

- ENU center and timestamped identity;
- horizontal forward and lateral unit vectors, vertical axis, and yaw;
- half-length, half-width, and vertical support;
- source and age of the orientation (`route_tangent`, measured horizontal
  velocity, or yaw fallback);
- the route/mission identity and world snapshot identity used to derive it.

The orientation policy is route tangent first, measured horizontal velocity
second, and vehicle yaw only as an explicit fallback. Yaw is not equivalent to
flight direction during sideslip, braking, or camera-first turns. A zero or
stale vector is rejected rather than normalized or silently replaced.

The oriented frame is used to select route samples, corridor seeds, and
look-ahead endpoints. Every selected point is converted back to ENU before
ROG queries, collision checks, A*, immutable snapshot certification, or PX4
publication. `UNKNOWN` and `OUT_OF_MAP` remain non-traversable for the
certified backup. The current `110 x 15 x 6 m` baseline AABB is retained for
its measured latency/memory profile. It does not cover the visibility horizon
in every horizontal direction. Route-specific support is checked against the
measured start and requested route before search; unsupported Y/diagonal
progress fails closed rather than being hidden by the long X side. A wider or
sparse/chunked map is a later measured design decision, not an implicit
consequence of this contract.

The implementation order is:

1. add the immutable frame contract and finite/age/frame tests;
2. integrate it into route-window selection while preserving AABB map queries;
3. export frame source, yaw, support, clipping, and fallback reason in planner
   diagnostics and rotate the replay overlay from recorded metadata only;
4. measure map footprint, snapshot export, slide, A*, corridor, and optimizer
   tails before considering a wider or sparse/chunked evidence store.

No report overlay may imply that the backend grid rotated unless the runtime
has emitted and validated the frame metadata for that exact command/world
identity.

The yaw continuity contract is part of this same boundary. Free yaw follows a
finite horizontal trajectory tangent; pure vertical motion and a stationary
terminal segment retain the incoming heading. A hot replan compares measured
yaw with the committed yaw state using the shortest angular distance. When the
configured continuity envelope is exceeded, the planner rebases on the fresh
measured state and retains the existing certified connector rules. PX4
stationary, terminal-hold, and recovery setpoints carry the latest valid
odometry heading with zero yaw-rate, so a missing yaw field cannot delegate
heading selection to PX4. The yaw envelope is a recovery trigger, not a
waypoint acceptance or obstacle-clearance relaxation.

### 3.7 NOMINAL generation and MINCO ownership

The product needs a deterministic feasible baseline before an optimizer is
allowed to improve it. The baseline is assembled from route pieces: a
collision-checked geometric guide or corner fillet plus a bounded S-curve time
law that starts at measured/retained PVA and obeys the selected speed profile.
It is independently swept-volume, corridor, dynamic, flatness, and world
certified.

MINCO becomes an optional refinement inside the same route tube. A refined
candidate is accepted only when it preserves the exact boundary identities and
all independent certificates while improving declared route-quality metrics.
Optimizer failure, non-finite cost, or a worse route/altitude trace returns the
certified baseline; it does not turn a feasible route into a planner failure.
Hard physical limits remain authoritative. Jerk may remain absent from the
MINCO objective, but P/V/A continuity, acceleration slew, body-rate, thrust,
and a declared command-shape envelope cannot be removed merely because PX4
closes the control loop.

Every non-emergency bundle replacement is evaluated at the exact publication
switch time against the command currently exposed to PX4. Position, velocity,
acceleration, practical jerk, yaw, and yaw-rate residuals must either satisfy a
named continuity contract or be connected by a separately world-certified
piece. Recording a residual without rejecting or connecting it is not an
acceptance gate. Emergency/Hold handover is a distinct transition with its own
measured-state contract and diagnostics.

### 3.8 BRAKE ownership and activation

The existing atomic NOMINAL-plus-BACKUP safety idea is retained, but historical
roles are not. A candidate has explicit `CONNECTOR`, `NOMINAL`, and `BRAKE`
sections. `BRAKE` is the deterministic certified minimum-snap stop generated
from a declared switch state on the new NOMINAL command. The optional backup
optimizer is disabled from the product path until an A/B proves that it improves
stopping evidence without adding feasibility or latency failures.

`BRAKE` has a stopping envelope, not only a zero terminal state. It bounds stop
time, stop distance, peak speed, and altitude excursion from the switch PVA. If
the initial acceleration still points along velocity, a short speed increase
may be physically required before bounded jerk reverses that acceleration; the
peak must be analytically derived and no larger than the deterministic seed.
After that transient, speed/kinetic energy must decrease monotonically. Any
refinement must preserve the seed's P/V/A/J timing and stopping envelope, not
only remain within a sampled position tolerance.

Every brake certificate contains an activation tube around its switch PVA and
world identity. If command renewal fails, runtime may enter BRAKE only while the
measured state remains inside that tube. Outside it, the bundle is not a valid
brake from the current vehicle state and runtime requests PX4 Hold. A BRAKE
interval is never copied into a later NOMINAL prefix. Recovery after braking is
a new PlanFromRest transaction from measured state.

### 3.9 Speed governor and direction-independent support

Cruise speed is a cap, not a promise at every waypoint. Each route sample uses:

```text
v_allowed = min(v_mission,
                sqrt(a_lateral_max / max(curvature, epsilon)),
                v_stop(certified_free_distance, latency, brake_model),
                v_tracking,
                v_yaw)
```

For the current 2 m/s^2 lateral envelope, constant-radius demand is 4.5 m at
3 m/s, 8 m at 4 m/s, and 12.5 m at 5 m/s. The nine-waypoint mission contains
90-degree turns separated by 10 m and a 0.9 m acceptance radius; demanding
5 m/s through those corners is geometrically contradictory. The initial
functional envelope is 3 m/s cruise with automatic corner reduction; the
performance ladder is 2/3/4/5 m/s, where 5 m/s is expected only on sufficiently
long straight certified sections. Speeds above 5 m/s are not a current
multi-waypoint product objective, and speeds above 12 m/s are out of scope.

Tracking error does not authorize generic speed-dependent safety relaxation.
A bounded longitudinal lag model may account for measured command/transport
delay, but lateral and vertical envelopes remain limited by certified clearance
and route altitude. When uncertainty, curvature, or error grows, the governor
reduces speed instead of widening every gate.

The ROG voxel axes remain ENU. A route-oriented selector cannot create map
support that the backing AABB does not contain, so the current 110 m by 30 m
horizontal store is not direction invariant. Before rotating voxel storage,
benchmark an approximately square rolling AABB under the same voxel budget
(for example 60 m by 60 m) and an oriented planner subset. A remote target is
clipped to the certified local route horizon and the map slides with position.
Sparse far-field evidence is considered only if the square dense baseline
cannot provide the required braking/look-ahead distance within mapping and
snapshot p99 budgets.

## 4. Debt register before further behavior tuning

The following debts are stop-the-line items. Local objective or threshold
tuning must not resume ahead of them.

| Debt | Required closure |
| --- | --- |
| D0-1 Final behavior/state contract | This document's operating cases have one deterministic owner, transition, fallback, and expected output. |
| D0-2 External PVA/PX4 contract | Capture active PX4 velocity, acceleration, tilt, yaw-rate, and yaw-acceleration limits; prove that external setpoints bypass internal Goto smoothing; reject incompatible configuration. |
| D0-3 Single route-progress owner | Mission controller alone advances waypoint identity; planner/runtime consume immutable route progress and cannot infer acceptance from trajectory endpoints. |
| D0-4 Command-role separation | Replace historical MAIN/BACKUP inheritance with CONNECTOR/NOMINAL/BRAKE and a measured activation contract. |
| D0-5 Feasible baseline | Provide a deterministic certified nominal seed so MINCO is refinement rather than the sole mission-availability path. |
| D0-6 Quality versus safety metrics | Log route progress, route/corner identity, yaw source/target, active speed limiter, generation-boundary PVA, role, activation-tube margin, altitude recovery, and stage latency. |
| D0-7 Direction-independent map decision | Benchmark square dense support versus current anisotropic support on dataset and loaded SITL before selecting map geometry. |
| D0-8 Temporary bypass closure | Run the registered `iris_iter_num=1` versus `2` dense-map A/B for TB-003; keep it explicitly temporary until its removal condition passes. |
| D0-9 Executable replacement continuity | Turn exact-switch P/V/A/J/yaw residuals into a reject-or-certified-connector gate; keep emergency/Hold transitions separate. |
| D0-10 Braking invariant | Certify stop time/distance, derived peak speed, post-transient monotonic deceleration, altitude excursion, and seed-versus-refinement PVA/time deviation. |
| D0-11 Atomic pass-through handoff | Remove the command reset/stationary pulse; retain the previous certified execution identity until the next bundle is accepted without relabeling it. |
| D0-12 Terminal-safe command access | Late terminal `COMPLETED` handling must use the latched completed checkpoint and never dereference an out-of-range active waypoint. |

The current exact physical gates are not automatically debts. A repeated
2.018 m/s^2 result against a 2 m/s^2 mission limit is a real infeasible command,
not proof that the limit should be relaxed. Numerical tolerance may cover only
representation error; operating reserve and deterministic fallback must keep
normal candidates away from the boundary.

## 5. Functional milestones and commit boundaries

Each milestone has its own acceptance evidence and one clean commit. Full
three-column mission completion is required only at the integration milestone;
it must not be used to mark a partially fixed function complete.

### M0 — behavior, evidence, and ownership freeze

Archive the current failing artifact and a scenario matrix covering start,
straight 0/45/90-degree map directions, shallow/90/135/180-degree transitions,
STOP, obstacle detour, altitude recovery, planner failure, stale world, and
tracking lag. Every case names the expected owner and fallback.

**Acceptance:** documentation review plus source/evidence anchors; no flight PASS
claim and no behavior change.

**Commit:** `docs: freeze navigation behavior and debt contract`.

### M0.1 — terminal command safety

Make late `COMPLETED` validation and logging consume the latched terminal
waypoint/request record, including duplicate and delayed samples after mission
completion.

**Acceptance:** direct tests cover exact late completion, wrong terminal
identity, duplicate completion, empty/one-point mission, and terminal index at
`waypoints.size()`; no exception or stale waypoint dereference is possible.

**Commit:** `fix: make late terminal command handling index safe`.

### M0.2 — atomic pass-through command handoff

Introduce an explicit `HANDOFF_PENDING` execution state. Latch measured
waypoint acceptance, request the next route bundle, retain and revalidate the
old command under its original identity, then atomically switch goal and command
identity when the replacement is accepted.

**Acceptance:** a 50 Hz trace across straight and 90-degree pass-through contains
no stationary/zero-velocity sample caused by goal publication, no command
relabeling, no message-identity false accept, and a valid old command is retained
when the replacement is late. Lease/certificate expiry still requests BRAKE or
Hold.

**Commit:** `fix: preserve certified command through waypoint handoff`.

### M1 — route progress and waypoint transition owner

Make route progress the single immutable input used by mission, planner, yaw,
and diagnostics. Cover duplicate points, a sample that jumps through an
acceptance ball, overshoot outside its lateral gate, stop versus pass-through,
and 180-degree reversal.

**Acceptance:** route arc-length never regresses beyond declared numerical
tolerance; each waypoint advances exactly once; no planner endpoint can advance
mission state. Unit/contract tests are sufficient for this milestone.

**Commit:** `feat: add route progress and waypoint boundary contract`.

### M2 — stable route-lookahead yaw

Implement the yaw state machine independently of MINCO shape. Exercise all
horizontal bearings, standstill, vertical motion, pass-through, final stop,
replan generation changes, and reversal.

**Acceptance:** yaw target is continuous under shortest-angle unwrapping,
commanded yaw rate/acceleration remain inside captured PX4 limits, straight-leg
replans do not change the semantic target, and standstill does not command an
unowned rotation. Open-map component SITL is sufficient; mission completion is
not required.

**Commit:** `feat: add stable route-lookahead yaw control`.

### M3 — deterministic piecewise NOMINAL baseline

With MINCO disabled, generate and certify straight, fillet, STOP, and
measured-state connector pieces. Preserve P/V/A at every boundary and keep the
route altitude profile explicit.

**Acceptance:** frozen synthetic/open-map cases produce an executable NOMINAL
trajectory or a precise infeasibility reason; no BACKUP is entered without an
injected fault; generation splices are continuous; the corner piece intersects
the measured acceptance volume.

**Commit:** `feat: add certified deterministic nominal trajectory`.

### M4 — curvature, stopping, and horizon speed governor

Apply the speed policy to 0/45/90/135/180-degree transitions, short and long
legs, reduced known-free horizon, and delayed planning. A 180-degree case must
select STOP-TURN-GO.

**Acceptance:** selected speed is no higher than every active limiter; the
planned turn fits the route/acceptance geometry; unavailable horizon lowers
speed before command exhaustion; straight sections recover cruise without a
generation velocity drop.

**Commit:** `feat: govern route speed from curvature and certified horizon`.

### M5 — BRAKE role and fault-injection contract

Generate deterministic brakes at several speeds, accelerations, corners,
altitudes, and activation offsets. Inject nominal solver failure, world change,
and command-renewal loss before and after the activation tube.

**Acceptance:** valid activation produces a known-free finite stop; invalid
activation requests Hold; BRAKE never advances route progress; a later plan is
from measured state and contains no inherited BRAKE-as-NOMINAL interval. Stop
time/distance, analytically allowed peak speed, post-transient monotonic speed,
altitude excursion, and seed-versus-refinement PVA/time envelopes all pass.

**Commit:** `feat: isolate certified brake activation from nominal history`.

### M6 — optional MINCO refinement

Run deterministic frozen-input A/B with seed-only and seed-plus-MINCO for
straight, corner, detour, and hot-replan cases.

**Acceptance:** MINCO is selected only when every certificate passes and route,
altitude, and continuity quality improve. Failure or deadline returns the
certified seed. Solver availability is no longer mission availability.

**Commit:** `feat: make minco a certified nominal refinement`.

### M7 — direction-independent map support and runtime budget

Compare the current 110-by-30-m map, square equal-budget candidates, and the
oriented selector at route bearings 0/45/90 degrees on representative recorded
data and loaded SITL. Profile map update, snapshot export, A*, corridor,
NOMINAL, BRAKE, publication, scheduling gaps, memory, and dropped observations.

**Acceptance:** the chosen support covers the speed governor's required
look-ahead in all bearings and meets declared p50/p95/p99 plus memory budgets.
Dataset PASS is performance/mapping evidence only, not flight acceptance.

**Commit:** `perf: establish direction-independent planning support budget`.

### M8 — altitude recovery and subsystem integration

Integrate obstacle detour, vertical route recovery, yaw, speed governor,
NOMINAL renewal, and BRAKE fault injection first on two- and three-waypoint
missions. Repeat each focused scenario enough to estimate distributions.

**Acceptance:** each subsystem metric passes its own declared gate; a failure is
attributed to one owner and is not hidden by an aggregate mission verdict.

**Commit:** `feat: integrate stable route flight behavior`.

### M9 — full mission and release evidence

Run sanitizers, full recorded-data replay, and repeated three-column
multi-waypoint SITL at 2/3/4/5 m/s. Only this milestone requires all waypoints,
mission completion, collision/clearance, speed recovery, altitude recovery,
yaw stability, PX4/odometry health, command availability, strict brake evidence,
and p50/p95/p99 runtime budgets in the same representative runs.

**Commit:** `docs: record repeated waypoint stability acceptance evidence`.

## 6. Decision gates and rollback rules

Advance only when the milestone-specific unit/contract tests and the relevant
runtime evidence pass. A failed optimizer or missing ray source is evidence to
shorten the candidate or fail closed, not permission to use `UNKNOWN` as free.

Rollback is by the last milestone commit. Never mix a route behavior change with a
new hard-gate value, a temporary bypass, or a large observability refactor. Any
temporary experiment must be recorded in
`runtime_safety_decision_ledger.md` with owner, scope, impact, evidence,
removal condition, and verification command.

The project is not flight-ready merely because dataset shadow planning or one
SITL speed passes. The release claim is made only after the full M9 evidence
matrix is archived.

## 7. PX4 reference anchors

The external contract above is based on the following upstream primary
references and must be rechecked against the exact PX4 revision used by each
release artifact:

- [PX4 TrajectorySetpoint](https://github.com/PX4/PX4-Autopilot/blob/main/msg/TrajectorySetpoint.msg): external PVA must be kinematically consistent and feasible; jerk is logging-only.
- [PX4 MulticopterPositionControl](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/MulticopterPositionControl.cpp): a fresh external trajectory setpoint suppresses the internal Goto trajectory generator.
- [PX4 multicopter Mission mode](https://docs.px4.io/main/en/flight_modes_mc/mission): multicopter inter-waypoint motion uses previous/current/next context, a rounded turn, and acceptance-radius switching.
- [PX4 jerk-limited Auto trajectory](https://docs.px4.io/main/en/config_mc/mc_jerk_limited_type_trajectory): Auto speed is adjusted from waypoint distance and the acceleration/jerk envelope.
- [PX4 autonomous parameters](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/multicopter_autonomous_params.yaml): upstream yaw modes and autonomous yaw-rate/acceleration defaults used only as initial comparison values.
