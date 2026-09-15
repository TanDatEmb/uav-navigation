# 5 m/s SITL diagnostic matrix after the corrected execution baseline

Date: 2026-09-16 (Asia/Ho_Chi_Minh)

## Verdict

This matrix is **diagnostic evidence only**. It produced zero report-level PASS
results: two `FAIL` and seven `BLOCKED`. One of nine runs completed its mission.
All nine report the safety dimension as PASS and no collision was observed, but
those facts do not make the matrix qualification evidence.

The strongest repeated result is the five-waypoint case: all three runs failed
to produce the first horizontal executable bundle within the five-second
recovery window. The optimizer repeatedly produced candidates whose jerk
exceeded the unchanged 8 m/s^3 hard gate. The runtime then failed closed and
handed authority to PX4 Hold. This is a repeatable planner-availability finding,
not evidence that the gate should be relaxed.

The nine-waypoint case showed that the same build can create and execute
certified MAIN/BACKUP bundles, but cannot sustain successor availability across
the full route. Progress varied from only the takeoff waypoint to four accepted
waypoints. The failures involved finite BACKUP endpoints outside waypoint
acceptance, solve-budget exhaustion, `known_free` rejection, and immutable
candidate revalidation. This variability argues for exact offline replay and
failure taxonomy before any architectural expansion or tuning.

## Frozen inputs and limitations

- Navigation source: `ed694d24fa09d83c3f85108bafd5fa3f08e4ccbc`, clean,
  source fingerprint
  `863e94bcb11e5bd341547afa9e34b83fa904652cd20236d40b951a41f11c86fb`.
- Release build manifest SHA-256:
  `65025ede47bc8a6af6e8247be498decd198171df2321a2425be8c674e31d566f`.
- External PX4 source: `deaff86ee335dd697677bcfc2415a23878e1b895`,
  dirty, source fingerprint
  `25341a3df3acb2e557ef386affdf7a67f070603e924eb8107e8d291d22e2081f`.
- PX4 SITL binary SHA-256:
  `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`.
- Every run used `TEST_CASE=positive`, `MOTION_PRESET=fast`, `MAP_SEED=0`,
  `SPEED_CAP_MPS=5`, and the same external PX4 directory.
- The runner captured and revalidated the dirty PX4 status/diff, binary,
  generated runtime tree, Gazebo plugins, and mutable PX4 inputs. This binds the
  results to the executed artifacts; it does not establish authoritative PX4
  build provenance.
- `tracking_experiment=relaxed` was active with geometric tracking and fresh
  typed-health response suppressed. All evaluation results therefore remain
  ineligible for qualification. No threshold was changed during the matrix.
- Evidence lifecycle attribution, reference lineage, and tracking coverage are
  incomplete. A report-level `FAIL` can therefore coexist with mission and
  safety dimensions that individually report PASS.

Command templates:

```bash
MAP_PROFILE=long_three_pillars_speed TEST_CASE=positive MOTION_PRESET=fast \
  MAP_SEED=0 SPEED_CAP_MPS=5 PX4_DIR=/home/letandat/Dev/Autopilot make run

MAP_PROFILE=long_three_pillars TEST_CASE=positive MOTION_PRESET=fast \
  MAP_SEED=0 SPEED_CAP_MPS=5 PX4_DIR=/home/letandat/Dev/Autopilot make run

MAP_PROFILE=long_three_pillars_multiwaypoint TEST_CASE=positive \
  MOTION_PRESET=fast MAP_SEED=0 SPEED_CAP_MPS=5 \
  PX4_DIR=/home/letandat/Dev/Autopilot make run
```

## Denominator-preserving results

`Accepted` includes waypoint 0, the post-takeoff mission origin. `Clearance` is
the minimum ground-truth collision clearance reported against all configured
obstacles, not a continuous-proof certificate.

| Case | Session | Report | Mission | Accepted | Safety | Collision / clearance | Mapping / writer loss | Primary terminal evidence |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2WP-1 | `external-mode-check-20260915T180441-587893` | `FAIL` | `NOT_EVALUABLE` | 0/2 | PASS | 0 / 0.576 m | 0 / 0 | FAST-LIO lost translational observability; propagated odometry had a 716 ms source gap. Command acceptance observed receive age 214.097 ms, beyond the 200 ms lease, and revoked authority. |
| 2WP-2 | `external-mode-check-20260915T180751-591749` | `BLOCKED` | `NOT_EVALUABLE` | 0/2 | PASS | 0 / 0.983 m | 1 / 0 | Repeated BACKUP `known_free` rejection, then STOPPED_HOLD anchor error 0.911 m exceeded the 0.750 m diagnostic bound. One mapping cloud was replaced, independently making evidence incomplete. |
| 2WP-3 | `external-mode-check-20260915T181002-595272` | `FAIL` | PASS | 0,1/2 | PASS | 0 / 1.291 m | 0 / 0 | Mission completed and handed over to PX4 Hold. Overall evaluation remained non-PASS because evidence, motion-quality, and tracking dimensions were not evaluable. |
| 5WP-1 | `external-mode-check-20260915T181325-598982` | `BLOCKED` | `NOT_EVALUABLE` | 0/5 | PASS | 0 / 4.670 m | 0 / 0 | No horizontal bundle was accepted. Dynamic certification repeatedly rejected roughly 10 m/s^3 or higher jerk; PlanFromRest recovery timed out at 5.079 s. |
| 5WP-2 | `external-mode-check-20260915T181449-602538` | `BLOCKED` | `NOT_EVALUABLE` | 0/5 | PASS | 0 / 4.690 m | 0 / 0 | Same initial-availability failure; observed rejected jerk was commonly 10-15 m/s^3. Recovery timed out at 5.095 s. |
| 5WP-3 | `external-mode-check-20260915T181557-605825` | `BLOCKED` | `NOT_EVALUABLE` | 0/5 | PASS | 0 / 4.683 m | 0 / 0 | Same initial-availability failure; repeated rejected jerk near 13-14 m/s^3 before recovery timed out at 5.061 s. |
| 9WP-1 | `external-mode-check-20260915T181706-609099` | `BLOCKED` | `NOT_EVALUABLE` | 0/9 | PASS | 0 / 6.513 m | 0 / 0 | MAIN/BACKUP moved about halfway to waypoint 1, then stopped at a certified endpoint about 9.8 m short. Continuation failed its 80 ms solve budget and the bounded recovery expired. |
| 9WP-2 | `external-mode-check-20260915T181827-612479` | `BLOCKED` | `NOT_EVALUABLE` | 0,1,2,3/9 | PASS | 0 / 2.438 m | 0 / 0 | Progressed toward waypoint 4. Replans repeatedly reached the 80 ms budget; active generation 13 later failed full immutable revalidation and the command became stale. |
| 9WP-3 | `external-mode-check-20260915T182038-616021` | `BLOCKED` | `NOT_EVALUABLE` | 0,1/9 | PASS | 0 / 2.620 m | 0 / 0 | Progressed toward waypoint 2. Replans exhausted the 80 ms budget; STOPPED_HOLD anchor error reached 0.859 m versus 0.750 m and the active candidate was invalidated. |

Aggregate accounting:

- Report verdict: PASS 0/9, FAIL 2/9, BLOCKED 7/9.
- Mission completion: 1/9; mission dimension PASS 1/9.
- Safety dimension PASS: 9/9; observed collisions: 0/9.
- Evidence-writer loss: 0 records in 9/9 runs.
- Mapping observation loss: one replaced cloud in one run; zero in the other
  eight.

## Timing and runtime evidence

All values below are measured distributions from the individual reports. They
are not claimed upper bounds.

| Metric | Range across nine runs | Interpretation |
| --- | --- | --- |
| Mapping callback p99 / observed max | 58.708-76.887 / 73.072-90.334 ms | Significant CPU work, but it did not by itself explain command revocation. |
| Planning-worker runtime p99 / observed max | 44.245-82.624 / 53.201-85.417 ms | Several cases reach or exceed the configured 80 ms solve deadline. |
| Reported planning-total p99 / observed max | 52.583-84.171 / 52.583-84.171 ms | Confirms deadline pressure in the executed solver path. |
| World snapshot export p99 / observed max | 14.141-15.922 / 14.458-17.330 ms | Material background cost; no evidence yet that moving it across a process boundary would improve authority availability. |
| Propagated odometry rate | 49.428-50.000 Hz | Eight runs had a maximum source gap of 20-28 ms; 2WP-1 had the 716 ms observability-related gap. |
| PX4 local-position setpoint source gap, observed max | 24-40 ms | Below the 100 ms command-stream contract in this matrix; command egress is not the leading observed blocker. |

The descriptive speed data also requires caution. Setpoint maxima were at or
below 4.903 m/s in runs that produced horizontal trajectories, while measured
speed maxima reached 6.013 m/s. Tracking remains `NOT_EVALUABLE`, so these values
are a signal for later closed-loop analysis, not a qualified tracking verdict.
The largest recorded LIO-to-ground-truth position-residual p95 was 0.780 m in
2WP-1, the run with the observability failure.

## Finding status

### CONFIRMED: complete executable proposal availability is the main repeated blocker

The five-waypoint reproducer fails 3/3 before the first horizontal command. The
hard gate correctly rejects excessive jerk, and no evidence supports relaxing
it. The distinguishing test is now an exact-world, exact-state offline replay
that records first numerical iterate, first nominal-feasible iterate, first
complete certified proposal, and first admitted executable bundle separately.

### CONFIRMED: finite certified execution does not guarantee route continuation

All three nine-waypoint runs produced executable bundles, yet none completed.
Existing active/BACKUP authority provided bounded motion and stopping, but
successor generation or revalidation failed before the route was complete.
This supports the proposed separation between proposal readiness, admission,
activation, and exposure; it does not yet support a new process boundary.

### CONDITIONAL: LIO observability can invalidate the command lease

One 2WP run reached the production command boundary with stale odometry after an
observability rejection. The fail-closed response is confirmed. Frequency and
hardware reachability remain unqualified from one SITL occurrence.

### EVIDENCE_GAP: closed-loop tracking and lifecycle attribution

The common blockers `EXPERIMENTAL_TRACKING_MODE`,
`LIFECYCLE_ATTRIBUTION_INCOMPLETE`, `REFERENCE_LINEAGE_UNAVAILABLE`, and
`TRACKING_COVERAGE_POLICY_UNAVAILABLE` prevent a qualified interpretation of
tracking, smoothness, and the complete command lineage. The one completed
mission is therefore not a report PASS.

### REJECTED for this matrix: command egress as the dominant failure source

Except for the LIO-induced source gap, the setpoint and propagated-odometry
streams stayed within their timing contracts. The failures correlate instead
with planner certification, solve budget, candidate revalidation, and one
estimator-observability event. This does not prove egress has a hard real-time
bound; it only rejects it as the leading explanation for these nine runs.

## Minimal next work

1. Replay the three five-waypoint initial failures using the captured state,
   immutable world, limits, and exact planner configuration. Classify whether
   the 8 m/s^3 jerk violation originates in guide timing, MINCO formulation,
   bounded retry, or certificate reconstruction. Preserve every no-solution
   run in the denominator.
2. Replay the nine-waypoint successor transactions around finite BACKUP
   completion and full immutable revalidation. Compare the 80 ms deadline,
   certificate-world identity, and known-free failure position without changing
   planner tuning.
3. Close lifecycle attribution and reference lineage as an observability-only
   change so the same matrix can evaluate tracking without the relaxed
   experiment. Do not combine this with planner changes.
4. Add a controlled LIO observability/odometry-lease reproducer using source and
   steady clocks, then verify the PX4 Hold handover and recovery policy.
5. Repeat the targeted failing cases before rerunning the full 2/5/9-waypoint
   matrix. A process split or larger coordinator extraction remains unjustified
   until timing evidence shows a resource-isolation need that these bounded
   changes cannot address.
