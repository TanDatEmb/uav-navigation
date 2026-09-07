# Navigation qualification status — 2026-09-07

## Purpose and verdict

This document records the planning, runtime, SITL-diagnostic and offline
replay results accumulated through the current Q1 investigation. It separates
source/unit evidence from SITL evidence and does not promote a diagnostic run
to flight or hardware qualification.

Current verdict:

```text
Q1_10HZ_NOT_READY
ROOT CLASSIFICATION: UNRESOLVED
PRODUCTION PATCH FOR THE CURRENT main_minco SEAM: NONE
SITL qualification: NOT CLAIMED
Hardware qualification: NOT RUN
```

The current blocker is the first nominal `main_minco` failure on the complex
mission. Its deterministic corridor-contained Bezier seed is known to be
inadequate for one duration family, while the exact authoritative world
certificate cannot yet be replayed from the captured optimizer snapshot.

## 1. Provenance

The current branch and worktree were recorded as:

```text
repository: /home/letandat/Dev/uav-navigation
branch: codex/runtime-evidence-for-analysis
HEAD: 5b98873f23e56bc4cba62dbe045b7696e7f09ba3
HEAD subject: perf(planning): enable 3 m/s development stress profile
worktree: dirty
```

The dirty state contains pre-existing Q1/runtime WIP and diagnostic snapshot
instrumentation. It is not an authoritative clean qualification provenance.
The Release build manifest recorded:

```text
manifest source HEAD: 5b98873f23e56bc4cba62dbe045b7696e7f09ba3
git_dirty: true
source SHA-256: 6246330598b64eb9e9191e1cdc7dbe09b328e41e2a7bfe9b825b986f8956bb76
```

The project-customized PX4 checkout is intentionally dirty and is accepted by
the project provenance policy:

```text
PX4 path: /home/letandat/Dev/Autopilot
PX4 HEAD: deaff86ee335dd697677bcfc2415a23878e1b895
PX4 policy: project_customized
```

The old `fix(runtime): fail closed on dirty PX4 provenance` change was removed;
PX4 dirty status is now recorded as provenance rather than treated as a
product error.

## 2. Completed contracts and decisions

### Measured-start current-body exception

The measured stopped-start contract is closed and remains unchanged:

```text
UNKNOWN is allowed only in the initial contiguous physical B0 prefix.
KNOWN_FREE does not consume the exception.
Physical exit closes the exception permanently.
Re-entry does not reopen it.
OCCUPIED, OUT_OF_MAP and UNDEFINED always reject.
Committed-future replanning receives no new B0 exception.
```

The implementation uses the physical body support, not planner inflation
radius. Mapping/oracle, re-entry and final trajectory regressions passed. No
moving-support lifecycle was introduced.

### Runtime transaction and handoff

The following issues were fixed and covered by focused/static evidence:

- certified PASS_THROUGH successor handoff;
- same-identity renewal and fresh scheduler-due failure injection;
- request isolation and RAII cleanup;
- execution-owner versus desired-goal identity;
- predecessor anchor retention across successor planning;
- emergency candidates no longer advertising nominal route-boundary metadata;
- BACKUP ownership only after the sampled role reaches the BACKUP interval;
- pending successor staging and activation identity checks.

The committed-future transaction baseline on `509379b3` passed the T2-A
plan/export/stage/activate path. The supplied T1/R6 evidence also showed the
real handoff flow accepting waypoints `0..4`, reaching mission COMPLETE, with
`1746` PVA commands, zero command failures, zero collision and no safety stop.
This is diagnostic SITL evidence, not flight qualification.

### Pre-existing PlannerFacade fixture failure

The `0.35 -> 1.0` failure was reproduced on clean base and patched trees with
the same assertion and result. It was therefore not introduced by the
current-body repair. The fixture was not changed merely to make the suite
green; the later planning-backend baseline recorded `8/8`.

### PX4 and frame policy

The custom PX4 checkout is valid project provenance. No PX4 source, parameter,
ENU/NED conversion or command-lease change was made for the current Q1
investigation. Earlier traces did not prove a basic frame-serialization bug.

## 3. Development directions evaluated

### 10 Hz planner contract

The development timing contract is:

```text
planner period:        0.10 s
planner rate:          10 Hz
hard solve deadline:   0.08 s
command period:        0.02 s
command timeout:       0.10 s
replan forward:        0.40 s
stitch duration:       0.40 s
commit guard:          unchanged
```

The contract preserves one active solve and at most one latest pending
request. It does not permit overlapping solves or a longer lease. A* budgets
were derived for the shorter deadline; the hard deadline remains authoritative.

This is still development characterization. One earlier Q1 timing sample
measured total planning p50 `20.673 ms`, p95 `32.544 ms`, max `42.626 ms`.
The later E1 trace measured p50 `11.609 ms`, p95 `30.229 ms`, p99 `40.140 ms`,
max `45.966 ms`, with deadline flag `0`. Low latency from failed solves is not
treated as planning-throughput evidence.

### Faster yaw

The development target was changed from `1.0/0.3` to:

```text
yaw rate:          1.5 rad/s
yaw acceleration:  1.0 rad/s2
yaw tracking budget: unchanged at 0.35 rad
```

The later `5b98873f` development stress profile currently loads nominal yaw
limits `2.0/2.0`; this is a subsequent experimental configuration, not a
qualification result. The requested `1.5/1.0` target remains separately
covered by the focused certificate work.

No yaw budget or certificate was relaxed. PX4 autonomous yaw values were
captured as `MPC_YAWRAUTO_MAX=60 deg/s` and `MPC_YAWRAUTO_ACC=20 deg/s2`, but
the trace did not prove that either value clipped the active command path.
The correct current classification is therefore
`YAW_LIMIT_APPLICABILITY_UNVERIFIED`, not a proven PX4 root cause.

### 3 m/s development stress envelope

The mission request remains `5.0 m/s`, but the effective nominal MAIN profile
used for this development branch is:

```text
V/A/J = 3.0 / 2.0 / 4.0
```

This is a source-owned stress profile, not evidence that the vehicle is
qualified at 3 m/s or 5 m/s. The physical trajectory boundary limits remain
independent and unchanged. No speed-envelope promotion was made.

### SUPER comparison

Pinned SUPER `2ad3419c127a617c6d7df6925e81a14175a9c096` uses numerical time
normalization: endpoint velocity, acceleration and jerk are scaled for the
normalized solve and restored after denormalization. This preserves physical
endpoint PVAJ mathematically and is compatible with an immutable execution
boundary as a numerical technique.

SUPER does not establish the project’s deterministic all-Bernstein-control
corridor seed requirement and was not copied as a safety semantic.

## 4. Q1 complex-mission evidence

Frozen scenario fingerprint:

```text
profile/world: long_three_pillars_multiwaypoint /
               long_three_pillars_speed
seed:          0
route length:  approximately 171.23 m
waypoints:     9  (8 PASS_THROUGH, 1 STOP)
route pillars: 3
truth objects: 30
requested speed: 5.0 m/s
map/inflation resolution: 0.2 / 0.2 m
unknown policy: allow_unknown
```

The Q1 run after the route-order repairs accepted waypoints `0..6` but did
not complete. It had zero collision and zero PVA command failure. The first
new planning failure was:

```text
solve_generation: 11
cycle:           62
stage:           main_minco
elapsed:         11.601 ms
hard deadline:   not exceeded
LBFGS return:    1
seed V/A/J:      6.018 / 23.422 / 781.110
limits V/A/J:    3.0 / 2.0 / 4.0
retry builds:    0 valid out of 4
```

Later corridor-boundary and approximately `80 ms` events are downstream and
were not used as the root cause.

An earlier E1 trace stopped after waypoints `0..3` and recorded:

```text
anchor raw:       0.320630 m
anchor aligned:   0.316453 m
tracking limit:   0.25 m
command age motion: approximately 0.011 m
collision:        0
PVA failures:     0
main_minco decisions: 64
rolling no-complete: 66/77
```

The tracking event remains classified as `REAL_TRACKING_LIMITATION` for that
event: the aligned residual itself exceeded the unchanged budget. This does
not identify controller, estimator or bridge ownership, and tracking was not
reopened while the earlier nominal planner failure remained unresolved.

## 5. Exact nominal snapshot and offline replay

The first future `TARGET_FAILURE_SIGNATURE` was captured from run
`external-mode-check-20260907T040103-63283`. The exact fixture is:

```text
.artifacts/runtime/external-mode-check-20260907T040103-63283/
  diagnostic_nominal_fixture/nominal_problem_snapshot_2_2_0.json
SHA-256:
1a36166219f5f99ff451b8de45a034f8e81966db1f9180163ffdce2f5b9e5372
```

The snapshot records world generation/revision, execution identity, head and
tail PVAJ, guide, durations, spatial variables, corridor planes, mapping,
route gates, timing and retry diagnostics. It does not contain the raw
world/occupancy snapshot needed to rerun the authoritative world certificate.

### Boundary and initial seed

The immutable execution head was:

```text
P = (1.018253377, -0.390430017, 3.003551422)
V = (1.940844524, -0.474166244, 0.090462108)
A = (1.775965349, -0.017305269, -0.028679088)
J = (-1.003498519, 1.437617417, -0.188571034)
```

The local frontier tail was:

```text
P = (17.201438995, 4.026687345, 3.0)
V = (2.776851328, 0.965762238, 0.0)
A = (0, 0, 0)
J = (0, 0, 0)
```

Initial piece lengths and durations were:

| Piece | Length (m) | Duration (s) | Corridor |
|---:|---:|---:|---:|
| 0 | 1.637495966 | 0.533333333 | 0 |
| 1 | 5.616810585 | 1.824788145 | 1 |
| 2 | 5.986802650 | 2.708042704 | 2 |
| 3 | 3.925091997 | 2.270460548 | 3 |

The initial corridor-contained seed was geometrically valid but failed its
dynamic certificate:

```text
V = 3.775321726 m/s
A = 9.836412131 m/s2
J = 102.934815060 m/s3
corridor maximum violation: -0.072129973 m
```

### Retry evidence

The first three retries failed at terminal piece `3`, control `4`, plane `3`:

```text
plane = [-1, 0, 0, 13.787851221]

piece-scaled: s=[3.099964579,1.192822612,1,1.320440096]
  point=(13.633573229, 2.785817879, 3)
  violation=0.154277992 m

uniform s=3.099964579
  point=(8.825248715, 1.113528767, 3)
  violation=4.962602507 m

uniform s=4.649946868
  point=(4.637153574, -0.3430505227, 3)
  violation=9.150697647 m
```

The `s=16` retry failed at first piece `0`, control `3`, plane `0`:

```text
plane = [1, 0, 0, -5.285115511]
point = (14.384177529, 2.039297240, 2.627245352)
violation = 9.099062018 m
```

The failed controls are not endpoints. Their duration-dependent velocity,
acceleration and jerk terms move them outside the convex hull as duration is
increased; immutable endpoint PVAJ was not changed.

### Duration-family analysis

For the uniform duration family only:

```text
corridor-feasible scale:  (0, 1.2670439035373]
dynamics-feasible scale:  [15.4164297228932, 16.8026197827265]
intersection:             empty
```

The dynamics range was measured by rebuilding and certifying each duration
against a diagnostic enlarged box; it was not inferred from a simple scaling
ratio. This proves a limitation of that deterministic seed family only. A
complete arbitrary nonuniform four-duration search was not performed.

### Candidate matrix

| Candidate | Corridor | PVAJ | Dynamics | Flatness | World |
|---|---|---|---|---|---|
| A: initial Bezier | pass | pass, residual `1.0e-12` | fail `3.775/9.836/102.935` | not reached | not replayable |
| B: duration-varied Bezier | no joint duration found | preserved | corridor-valid durations fail dynamics | not reached | not replayable |
| C: immutable MINCO seed | fail, violation `0.385901 m` | pass, residual `<1.1e-13` | fail `6.003/7.584/19.839` | not reached | not replayable |
| D: production MINCO/L-BFGS, 80 ms | pass, `-0.017600 m` | pass, residual `<4e-15` | pass `2.940/1.776/1.991` | pass with actual config | not replayable |
| D: high-effort offline | pass, `-0.000744 m` | pass, residual `<1e-14` | pass `2.940/1.776/2.343` | pass with actual config | not replayable |

The normal D solve used one LBFGS attempt and `352` evaluations without a hard
deadline overrun. The high-effort diagnostic used `2965` evaluations. Both
preserved physical PVAJ and all replayable local hard limits. The missing raw
world prevents calling either result a fully authoritative executable bundle.

## 6. What the evidence concludes

Established:

- the original tracking residual was not explained away by time alignment;
- command transport was not the first Q1 blocker;
- the first captured nominal failure is a dynamics-invalid deterministic seed;
- duration retries fail at specific convex-hull controls, not at an unknown
  generic `kBoundaryControl` condition;
- the uniform deterministic Bezier family has disjoint corridor/dynamics
  duration ranges;
- existing generic MINCO/L-BFGS can find a locally certified candidate inside
  the normal `80 ms` budget on the captured local problem;
- SUPER normalization can preserve physical endpoint PVAJ;
- no evidence justifies changing PVAJ, corridor geometry, thresholds, UNKNOWN
  policy, planner deadline, PX4 source or tracking budget.

Not established:

- a full authoritative world-certificate replay for candidate D;
- arbitrary nonuniform Bezier-family feasibility;
- the final production owner of the Q1 mission stop after the nominal solve;
- 10 Hz stable complex-mission performance;
- faster-yaw execution by PX4;
- E2/E3, T2-B/C, T3/T4, hardware or flight qualification.

## 7. Static gates and artifact status

The latest clean committed baseline was reported with:

```text
planning backend CTest: 8/8
runtime CTest:          11/11
mapping CTest:           3/3
planning contracts:      1/1
execution:               2/2
contracts:               1/1
Python:                253/253
Release build:           23 packages
git diff --check:       PASS
```

The current diagnostic WIP Release build also completed `23` packages and
`git diff --check` passed. Because the current worktree is dirty and the
diagnostic source changed after the clean baseline, this report does not
present the current WIP as clean authoritative qualification.

Relevant artifacts:

```text
.artifacts/runtime/external-mode-check-20260906T224141-1495410
  historical Q1 failure; incomplete optimizer input

.artifacts/runtime/external-mode-check-20260907T040103-63283
  target-signature capture and offline fixture

.../diagnostic_nominal_fixture/replay_stdout.txt
  SHA-256 839f2f973283c38beaf4566758f762b4da0085b70e0da311bfdca425f6ac2451
```

## 8. Next development direction

The next single blocker is to make the captured nominal problem sufficient for
authoritative world-certificate replay, or to capture the next snapshot with
the exact immutable world/occupancy data required by that certificate. Until
then:

```text
no new fallback
no production optimizer patch
no E2/E3
no T2-B/C
no T3/T4
no tracking-threshold change
```

Once that seam is closed, rerun the same Q1 fingerprint once and classify the
first post-patch failure before any further tuning. Only a complete Q1 with
all waypoints, continuous authority, unchanged safety gates, bounded latency,
and valid tracking may open the next cadence or yaw qualification phase.
