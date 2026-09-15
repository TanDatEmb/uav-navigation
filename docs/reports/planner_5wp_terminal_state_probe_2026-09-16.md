# Five-waypoint terminal-state feasibility probe

Date: 2026-09-16 (Asia/Ho_Chi_Minh)

## Verdict

The captured terminal velocity is a material part of the five-waypoint nominal
feasibility failure, but this experiment does not yet identify a safe product
replacement.  For three provenance-bound PlanFromRest snapshots, the exact
captured terminal velocity (scale 1.0) produced no candidate even after the
solve deadline was removed. Holding the initial state, every other terminal
derivative, guide, corridor, world, configuration, physical limits, and planner
entrypoint fixed while reducing only terminal velocity produced at least two
independently certified trajectories per snapshot.

This is `CONDITIONAL` formulation evidence, not an executable-bundle result.
The successful probes have no BACKUP, role schedule, lease, admission,
activation, or PX4 authority.  The experiment does not authorize changing
mission endpoint semantics or relaxing the unchanged 5/5/8 m/s, m/s^2, m/s^3
dynamics envelope.

## Frozen evidence

- Capture session:
  `.artifacts/runtime/external-mode-check-20260915T185456-650597`.
- Snapshot directory:
  `.artifacts/runtime/nominal-writer-finalize-canary-20260916`.
- Captured navigation revision:
  `4511c11b6a2006153f296a9459ac4cacbe654bda`.
- Captured source fingerprint:
  `e8b1e5cfb568a72b81114c85265fcfd81640d1c0323d94f03b5f5ea3acce77d1`.
- Captured Release-manifest SHA-256:
  `b3bd3d6b38f9f2718b0b1be02a639a1ba74af0d2f8f95cf4008f8dcefaea9e34`.
- Replay implementation: the commit containing this report, based on
  `21d6df8f32ed464060cff52c4635324e1590736b`.
- Scenario: `long_three_pillars`, positive case, fast preset, map seed 0,
  five waypoints, requested speed cap 5 m/s.
- External PX4 remained dirty/project-customized and the session remained a
  diagnostic run, not C0 qualification.

The bounded writer's sidecar is complete: 45 records submitted, 15 accepted,
15 written, 30 explicitly dropped, zero write errors, zero pending records,
and 15 observed files.  `capture_complete=true` means every accepted record was
accounted for after process stop; it does not mean full 45/45 capture coverage.

Selected immutable inputs:

| Position | Solve generation / planner cycle / world revision | Snapshot SHA-256 |
| --- | --- | --- |
| Start | 1 / 1 / 171 | `4638287e95fb721ad4cdadcd6274732aaa3405ef715ba60e40e542613495fbd9` |
| Middle | 16 / 16 / 183 | `af3b545dc17c0ea14a116344c45bb2db8c2f5fa6f76d2d4519a44d52b9a0493f` |
| End | 49 / 49 / 209 | `a1325148d763b4b686122d219a654ce15766deffd80fab0d5631b296ca815428` |

## Controlled experiment

The replay restores each captured PVAJ boundary, guide, corridor, immutable
world, planner configuration, limits, and PlanFromRest solve mode.  It disables
the solve deadline and evaluates terminal-velocity scales
`0.0, 0.25, 0.5, 0.75, 1.0`.  No other terminal derivative is changed.

A result is counted as certified only when the existing production
deterministic nominal certificate passes, V/A/J and flatness pass, and the
immutable-world sweep passes.  `candidate_available` by itself is not counted.
Every result remains `complete_executable_bundle=false` by construction.

| Terminal velocity scale | Generation 1 | Generation 16 | Generation 49 |
| --- | --- | --- | --- |
| 0.00 | Certificate PASS; V/A/J 2.459/2.524/6.437 | Certificate PASS; 2.249/2.595/6.911 | Certificate PASS; 2.088/2.760/6.911 |
| 0.25 | Certificate PASS; V/A/J 2.459/2.524/6.127 | Certificate PASS; 2.448/1.648/4.106 | Certificate PASS; 2.203/1.464/3.224 |
| 0.50 | Candidate; PVAJ junction certificate FAIL | Certificate PASS; 4.592/3.903/7.221 | Candidate; PVAJ junction certificate FAIL |
| 0.75 | Certificate PASS; 4.900/4.078/6.217 | Candidate; PVAJ junction certificate FAIL | Candidate; PVAJ junction certificate FAIL |
| 1.00 | No candidate | No candidate | No candidate |

All constructed rows passed the immutable-world sweep.  The scale-0 and
scale-0.25 rows were accepted directly as deterministic certified seeds; the
higher scales used production optimization where a candidate was available.
The feasible set observed here is not monotonic: for example, scale 0.50 failed
the independent PVAJ junction certificate at generation 1, passed at
generation 16, and failed again at generation 49. Therefore a global fixed
scale cannot be inferred from these three probes.

The rejected optimized candidates failed certificate stage 3 (`boundary`),
not stage 4 (`route_boundary`). Their reported failing component residuals
were approximately `6e-14`--`1.6e-13` in the affected derivative units against
computed roundoff bounds of approximately `3e-14`--`1.4e-13`. Production
optimization had reported those trajectories as candidates because its
post-L-BFGS gates check
corridor, route boundary, dynamics, and flatness; it does not reuse the
pre-L-BFGS seed's full PVAJ roundoff certificate. This is a separate numerical
contract question. The table remains fail-closed and does not count these
candidates as certified.

The scale-1.0 result matched the unmodified no-deadline PlanFromRest result in
all three snapshots: status 2, no candidate, three optimizer attempts, and two
retries.  This is the parity control for the experiment.

## Finding chain

- **Claim:** the captured endpoint velocity contributes to the repeated
  five-waypoint nominal failure.
- **Invariant:** changing only that boundary value must distinguish the result
  while all captured geometry, limits, world identity, and solve mode remain
  fixed.
- **Reachability:** the inputs came from real PlanFromRest requests at the
  beginning, middle, and end of one failed integrated run.
- **Strongest counterargument:** the lower-velocity solutions may merely be
  finite optimizer outputs that bypass boundary, corridor, route, or dynamics
  semantics.
- **Distinguishing test:** run the production optimizer without a deadline and
  apply the independent production certificate, dynamics, flatness, and world
  checks to every output.
- **Result:** scale 1.0 failed 3/3; lower scales yielded certificate PASS in
  3/3 snapshots, while several other candidates were excluded by the stricter
  PVAJ junction-roundoff certificate.
- **Minimal response:** next test a typed terminal-state policy that derives a
  dynamically reachable endpoint from route/continuation semantics, without
  changing the execution coordinator or hard gates.  Require MAIN/BACKUP bundle
  construction and targeted SITL evidence before any product behavior change.

## Limits and next discriminator

This evidence comes from one SITL session and three of the 15 captured records.
It does not establish a distribution across estimator states, map seeds,
recorded MID-360 data, or hardware.  Zero terminal velocity also changes route
continuation behavior and may be operationally unacceptable even though its
trajectory certificate passes.

The next bounded experiment should compare candidate endpoint policies against
the same snapshots and an independent reachability/continuation contract.  It
must report complete MAIN/BACKUP readiness and time-to-ready, then run a single
targeted five-waypoint SITL regression.  The full 5 m/s matrix should only be
repeated after that discriminator identifies a policy with no authority or
safety regression.

## Follow-up: direction-transition-derived cap

The follow-up probe replaced the arbitrary fixed scale with a dimensional
guide policy. For the final two guide segments, it computes the velocity-vector
change per unit speed and caps equal-magnitude terminal speed by the existing
jerk/acceleration-bounded velocity-change envelope over the captured two-edge
time window. The helper is pure and is not called by production planning.

Across all 15 captured snapshots:

| Observation | Result |
| --- | --- |
| Captured terminal speed | 4.9 m/s in 15/15 |
| Active route-boundary gates | 0 in 15/15 |
| Derived cap, generations 1--11 | 1.090 m/s; scale 0.222; 0.365 s turn window |
| Derived cap, generations 16--49 | 0.917 m/s; scale 0.187; 0.459 s turn window |
| Production solver candidate | 15/15 |
| Full independent certificate | 12/15 PASS |
| Full-certificate exclusions | Generations 2, 6, 11: PVAJ junction roundoff stage |
| V/A/J, flatness, immutable-world sweep | 15/15 PASS |
| Complete executable bundle | 0/15 by replay scope |

The 12 full-certificate PASS cases used the deterministic seed directly with
zero L-BFGS evaluations. The other three required one L-BFGS attempt and were
not counted as certified even though the production solver, dynamics,
flatness, and world checks accepted them. Their additional certificate failure
is the separate machine-scale PVAJ roundoff evidence gap documented above.

This strengthens the endpoint/tangent-timing formulation hypothesis without
proving a product policy. The derived cap is conservative and can reduce the
future frontier speed substantially; using it unconditionally could trade
availability for stop/go behavior. The next product experiment must therefore
measure command continuity, MAIN/BACKUP readiness, activation rate, and actual
vehicle speed rather than treating 15 nominal candidates as a flight PASS.

Verification commands:

```bash
make build

build/navigation_planning_backend/replay_nominal_problem_snapshot \
  SNAPSHOT.json 2>&1 | \
  grep -aE \
  '^(snapshot_kind=|D_recovery_no_deadline |F_terminal_velocity|G_tail_turn_)'

make test
git diff --check
```
