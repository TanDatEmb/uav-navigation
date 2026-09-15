# Exact nominal replay of the 5 m/s five-waypoint failure

Date: 2026-09-16 (Asia/Ho_Chi_Minh)

## Verdict

The five-waypoint availability failure is not explained by the 80 ms solve
deadline alone. Three captured PlanFromRest problems, selected from the start,
middle, and end of one SITL run, still produced no nominal candidate when the
production optimizer was replayed with no deadline. Four additional
initial-duration scales also failed for each selected problem.

For the first snapshot, the closest production result satisfied the configured
velocity and acceleration bounds but reached 9.724 m/s^3 jerk against the
unchanged 8 m/s^3 hard gate. Longer duration probes either still exceeded the
dynamics envelope or left the corridor. The same runtime run reported 47
`nominal_dynamics` failures with observed candidate jerk from 9.623 to
17.374 m/s^3. This confirms a corridor/dynamics feasibility problem for the
replayed inputs; it does not prove that every five-waypoint failure has the same
root cause.

The snapshot capture itself is not yet complete evidence. Only 16 of 51 solve
generations were serialized, and the standalone files contain
`source_revision=not-provided` and no workspace/build-manifest identity. The
parent SITL session has valid authoritative navigation provenance, so the three
files are usable as session-bound diagnostics, but they must not be presented
as independently reproducible qualification artifacts.

## Frozen run and scope

- Navigation HEAD: `73385688f3f0c56321c334c0e73ebe284bace1c8`, clean.
- Release source fingerprint:
  `1f8849d0380d4232918f893194d13a6c7eaa657e6a80564a0f958d2eac7f8dbf`.
- Release manifest SHA-256:
  `16b227c9e9e164a8dc43ebd2121624546a3e60370908e8734928a11977886632`.
- SITL session:
  `.artifacts/runtime/external-mode-check-20260915T183033-624860`.
- Snapshot directory:
  `.artifacts/runtime/5wp-nominal-snapshots-20260916-r3`.
- Scenario: `long_three_pillars`, positive case, fast motion preset, map seed 0,
  five waypoints, requested speed cap 5 m/s.
- Capture: failure-only, immutable world included, bounded asynchronous writer.
- External PX4 remained dirty/project-customized and
  `tracking_experiment=relaxed` remained active. This run is diagnostic only.
- No physical limit, validator, recovery timeout, or admission gate was changed.

The first launch attempt in session
`.artifacts/runtime/external-mode-check-20260915T182946-622459` failed preflight
because the documentation commit after the earlier matrix no longer matched
the installed Release manifest. A canonical `make build` refreshed the manifest
before the run reported here. No planner snapshot was produced by the failed
preflight attempt.

Command:

```bash
UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR=/home/letandat/Dev/uav-navigation/.artifacts/runtime/5wp-nominal-snapshots-20260916-r3 \
UAV_NAVIGATION_NOMINAL_SNAPSHOT_INCLUDE_WORLD=1 \
UAV_NAVIGATION_NOMINAL_SNAPSHOT_FAILURE_ONLY=1 \
MAP_PROFILE=long_three_pillars TEST_CASE=positive MOTION_PRESET=fast \
MAP_SEED=0 SPEED_CAP_MPS=5 PX4_DIR=/home/letandat/Dev/Autopilot make run
```

## Runtime denominator

The session ended `BLOCKED` in `PAUSED_SAFETY_STOP`. It accepted only waypoint
0, published no executable trajectory, observed no collision, and reported a
minimum ground-truth collision clearance of 4.700 m. Runtime evidence captured
812/812 submitted records with no writer loss; that counter does not include
the separate nominal-problem snapshot writer.

| Runtime observation | Result |
| --- | --- |
| Planner cycles | 51 |
| Complete planner trace records | 51 |
| Executable candidates | 0 |
| Runtime admission attempts | 0 |
| First causal failure | 47 `nominal_dynamics`; 3 `no_complete_bundle_at_deadline`; 1 `invalid_input` |
| Certified-seed failure stage | 50 dynamics-stage failures; 1 no seed-stage failure |
| Hard deadline observed | 3/51 cycles |
| Remaining hard budget at finish | -101 to 51,435 us |
| Rejected candidate jerk | 47 values; min 9.623, median 11.927, max 17.374 m/s^3 |
| Configured jerk hard gate | 8 m/s^3 |
| Serialized nominal snapshots | 16/51 solve generations (31.4%) |

The three deadline-expired cycles are real and remain part of the failure
taxonomy. They cannot explain the other 47 dynamics failures, many of which
finished with substantial hard budget remaining.

## Exact replay probes

The replay binary restores the serialized planner configuration, PVAJ boundary
state, guide, pre/post corridor, initial MINCO state, duration retries, and
immutable world. It deliberately does not reconstruct a MAIN/BACKUP role
schedule, execution lease, or commit authorization. Consequently its world
sweep is non-authoritative and `complete_executable_bundle` must remain false.

Selected snapshots:

| Position in run | Solve generation / world revision | SHA-256 |
| --- | --- | --- |
| Start | 1 / 161 | `29c1f9b23ee6d9ec49cbc46157f65d8a29bc1dd1a240031ed7d9234a0563b4ec` |
| Middle | 20 / 176 | `34ffc54e607caf249342da38429bf699065289be165893706f3c3b034d12cf2a` |
| End | 51 / 201 | `506053c1ceb262f9de61f76f137b70ed2f5832c4c7489dc9d55a362f4f7ca185` |

All three reported exact config restoration, coherent pre/post production
setup, and an available immutable world. The main results were:

| Probe | Generation 1 | Generation 20 | Generation 51 |
| --- | --- | --- | --- |
| Corridor-contained Bezier seed | Constructed; world sweep PASS; dynamics certificate failed | Constructed; world sweep PASS; dynamics certificate failed | Constructed; world sweep PASS; dynamics certificate failed |
| Immutable MINCO | Constructed; world PASS; V/A/J failed | Constructed; corridor and world failed; V/A/J failed | Constructed; world PASS; V/A/J failed |
| Production L-BFGS, no deadline | No candidate | No candidate | No candidate |
| Production L-BFGS, 40/80 ms | No candidate | No candidate | No candidate |
| PlanFromRest, no deadline | No candidate | No candidate | No candidate |
| High-effort scales 0.5/1/2/4 | 0/4 candidates | 0/4 candidates | 0/4 candidates |
| Complete executable bundle | Not available in replay | Not available in replay | Not available in replay |

For generation 1, the initial Bezier was geometrically contained but had
`V=10.935`, `A=36.320`, and `J=393.646` under the 5/5/8 runtime limits. The two
captured duration retries reduced those values to 8.440/24.578/313.550 and
5.627/14.927/133.923, but neither approached a physical certificate. The best
optimized candidate reached `V=4.900`, `A=4.930`, `J=9.724`; increasing time
farther had no corridor-compatible interval. This is a conflict between the
current terminal-state/timing/corridor formulation and the dynamics envelope,
not evidence for relaxing the envelope.

## Finding chain

### CONFIRMED: the solve deadline is not the sole cause

- **Claim:** the five-waypoint initial-availability failure is only an 80 ms
  scheduling problem.
- **Invariant:** a no-deadline replay of the same mathematical problem should
  produce a nominal candidate if budget is the only blocker.
- **Reachability:** the SITL run supplied 51 real PlanFromRest requests; three
  exact captured inputs cover world revisions 161, 176, and 201.
- **Strongest counterargument:** capture overhead or one adverse estimator
  sample may have distorted the online result.
- **Distinguishing test:** replay each immutable problem with the production
  optimizer under no deadline and multiple initial-duration scales.
- **Result:** all no-deadline and high-effort probes still failed.
- **Minimal response:** investigate terminal velocity, guide timing, bounded
  time stretch, and corridor compatibility as a planner problem; do not widen
  the coordinator or the process topology.

### CONFIRMED: numerically constructible is not certified or executable

The deterministic Bezier and immutable MINCO paths can be finite and satisfy
boundary reconstruction while failing production dynamics/flatness or corridor
certificates. No replay result contains BACKUP, a role schedule, a lease, or an
admission decision. These outputs are numerical/certificate diagnostics, not
evidence of a ready executable bundle.

### EVIDENCE_GAP: snapshot provenance and capture-loss accounting

The session report binds the run to the authoritative Release manifest, but the
snapshot files themselves retain the default `not-provided` source/workspace
fields. Missing solve generations are visible, but the writer's submitted,
written, dropped, serialization-error, and shutdown-complete counters are not
persisted. The runtime evidence writer's 812/812 count is a different queue and
must not be used as proof that nominal snapshots were complete.

## Minimal next changes

1. When nominal capture is enabled, bind snapshot source/workspace/build-manifest
   identity from the already validated runtime manifest. Reject conflicting
   caller-supplied identity instead of silently recording it.
2. Persist a nominal-writer sidecar with submitted, written, dropped,
   serialization-error, pending, and capture-complete fields. Treat an absent
   or incomplete sidecar as `NOT_EVALUABLE`; do not increase the queue or move
   world serialization into the planner thread merely to obtain 100% coverage.
3. Add a focused offline formulation experiment that varies terminal-state and
   duration construction while keeping 5/5/8 limits and the captured corridor
   fixed. Report feasibility by stage; do not turn the experiment into product
   tuning.
4. Only after a formulation change has an independently certified candidate,
   run one targeted five-waypoint SITL regression before repeating the full
   matrix.

