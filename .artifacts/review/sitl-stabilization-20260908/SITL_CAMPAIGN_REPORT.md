# SITL campaign report — tracking experiment, 2026-09-08

## Current source checkpoint and profile split — 2026-09-09

The latest independently committed source checkpoint is `d4e34e5f`.
`c5211c4b` records the GPS-off local-takeoff handoff and `39721dfc` closes
the `SimplifySFC` consumer-contract defect; neither changes the default
GPS+LIO mission semantics. `d4e34e5f` also records the mandatory-renewal
budget correction. The working checkout remains dirty because unrelated
planner and replay WIP is intentionally unstaged.

The campaign rows below are default GPS+LIO diagnostics unless their scenario
snapshot says otherwise. The following two runs are a separate explicit
`GPS-off EV` profile (`gps_off_ev_12mps`) with GPS disabled and EV aiding
enabled. This profile uses control envelope `V/A/J=12/2/4`; cap3 requests and
effectively targets 3 m/s, while cap5 requests and effectively targets 5 m/s.
They are infrastructure-valid diagnostics, not qualification:

| Profile | Artifact | Bundles activated | Waypoints | Collision | PVA commands | Executable trajectories | Measured speed p95/max | Outcome |
|---|---|---:|---:|---:|---|---|---:|---|
| GPS-off EV | [`004629-509425`](../../runtime/external-mode-check-20260909T004629-509425/report.json) | 8 | 1/9 | 0 | 922 success / 0 failure | 922/922 | 3.0044 / 3.0800 m/s | `BLOCKED` |
| GPS-off EV | [`005203-511728`](../../runtime/external-mode-check-20260909T005203-511728/report.json) | 4 | 1/9 | 0 | 369 success / 379 failure | 368/369 | 1.5382 / 2.4411 m/s | `FAIL` |

The cap5 failure is explicitly a lidar timestamp/freshness/validity failure;
it is not evidence for a 5 m/s controller or envelope claim. The GPS-off
profile's unavailable PX4 Hold interval is a diagnostic environment
limitation, not a product bug and not permission to relax Hold validation.

## Current backlog after component closure

The yaw root cause is now a separate bounded planning issue: the current
`YawTrajOpt` solves one target yaw over the whole position-horizon duration,
so a long bundle can hold the old heading through a corner and turn late.
PX4 follows the command; yaw limits are not the diagnosed owner. A path-aware
yaw fix and focused regression remain pending.

The terminal backlog is also separate. The terminal bundle crossed the
existing execution-anchor gate at roughly `0.822/0.870 m` raw/projected error;
the subsequent stopped endpoint was rejected with `known_free=1` and
`near_execution=0`. No safety gate, timeout, lease or Hold validator was
relaxed.

R1/R2/R3 diagnostic regression evidence remains open (snapshot double,
setup-failure and off-timed-solve closure). These are not closed by the
runtime rows below. The historical relaxed `1552559` 9/9 completion remains
`qualification_eligible=false` and its failed repeat remains counter-evidence;
neither is promoted to stable Q1 behavior.

## Latest exact-source retest addendum — 2026-09-09

These runs were executed sequentially from clean Release trees after the
focused `SimplifySFC` closure. They are current-source diagnostics, not
qualification evidence. The mission request was `5.0 m/s`; the effective MAIN
envelope was `3.0/2.0/4.0`, and the observed setpoint speed remained about
`2.9–3.0 m/s`.

| Run | Source / manifest | Mode | Waypoints | Complete | Collision / PVA | Total planning p50/p95/max (ms) | Deadline flags |
|---|---|---|---:|---|---:|---:|---:|
| `external-mode-check-20260908T223501-451718` | `8ea66105` / clean Release | adaptive / 3 m/s | 0–4 | no | 0 / 0 | 40.142 / 41.778 / 43.905 | 0 |
| `external-mode-check-20260908T223719-453312` | `8ea66105` / clean Release | relaxed / 3 m/s | 0 | no | 0 / 0 | 40.245 / 72.950 / 72.950 | 0 |
| `external-mode-check-20260908T223834-454788` | `bb609142` / clean Release | adaptive / 3 m/s | 0–1 | no | 0 / 0 | 25.470 / 42.851 / 43.159 | 0 |
| `external-mode-check-20260908T224002-456318` | `bb609142` / clean Release | relaxed / 3 m/s | 0–7 | no | 0 / 0 | 40.124 / 80.071 / 80.211 | 5 / 82 |

Artifacts:

- [8ea adaptive](../../runtime/external-mode-check-20260908T223501-451718/report.json)
- [8ea relaxed](../../runtime/external-mode-check-20260908T223719-453312/report.json)
- [bb609 adaptive](../../runtime/external-mode-check-20260908T223834-454788/report.json)
- [bb609 relaxed](../../runtime/external-mode-check-20260908T224002-456318/report.json)

Clean Release provenance:

- `8ea66105`: manifest source SHA `74cdc3500fb6b81aa54ab30e16af0aab7a522553b208aaf4c45f0d94cc84d102`, manifest SHA-256 `f367b7fc0906d955959a6415e03eb0c8f4850cac43874cc4d11d133de9c1da84`.
- `bb609142`: manifest source SHA `64caf30cc30d2f78a076367dac9cdba4f13dd80daaa43e85a5a8dbaee50aaa3e`, manifest SHA-256 `5a65bedd3b22cef15ba63bbe644fa58a1e741247306a09222eb68a5b4aad7aff`.
- Both use `px4_msgs=86d8239e962f6939e05c3737784f60c02fa884db`, `px4_ros2_interface_lib=4a3370f084ac6f1ef001a4afa2b007845ffd0837`, and customized PX4 `deaff86e` (dirty external checkout).

The `bb609142` relaxed tail contains a concrete timing-boundary observation.
At cycle `706` / solve generation `83`, the recovery request had no certified
nominal seed (`exp_used_certified_seed=0`, deterministic seed failure stage
`5`). The bounded optimizer consumed `75.352 ms` after `4.193 ms` frontend
work, was cancelled at the absolute `80 ms` deadline, and produced no
candidate (`LBFGS return=2`, `final_duration=NaN`, `retry_count=0`). The active
bundle was still generation `20` MAIN and valid, but no new commit occurred.
At cycle `720` / generation `84`, the corresponding values were `77.096 ms`
optimizer after `2.889 ms` frontend, total `80.211 ms`, again cancelled at the
hard deadline with no candidate. The runtime then correctly reported a
PlanFromRest transient failure and retained the old stopped-recovery endpoint.

The `backup_frontend_us`/`backup_opt_us` fields in those records describe the
currently active/previous backup diagnostics; they do not prove that a fresh
backup transaction ran after the failed nominal solve. The subsequent logs
show no backup stage for generations `83/84`. Generation `85` later produced a
certified PlanFromRest MAIN and backup in `71.170 ms` (nominal optimizer
`59.841 ms`, backup frontend `5.167 ms`). At the same `91.032 s` trace sample,
the active execution bundle was already generation `21`, role MAIN and valid;
waypoint `7` was accepted at `97.420 s`. Thus generations `83/84` are
recoverable hard-deadline tail failures, not the first unrecoverable mission
event and not, by themselves, proof of permanent trajectory infeasibility.

Source review confirms why this occurs: `baseline_only` with no certified seed
promotes the optimizer monitor from the normal `80 ms - 40 ms` refinement
cutoff to the full `80 ms` hard deadline. That preserves mandatory feasibility
search but intentionally gives up the finalization reserve. The evidence
proves **mandatory feasibility hard-deadline exhaustion** and confirms that no
fresh backup stage was reached because no MAIN candidate existed. It does not,
by itself, prove a separate backup/finalization allocation defect: a missing
MAIN candidate prevents the backup stage from running even if reserve time
remains. Any counterfactual benefit from a different reserve policy is
unproven; reverting to the refinement cutoff would fail the same mandatory
solve earlier, so this is not yet a justified one-line patch.
The ledger review condition is triggered; no deadline, certificate, lease or
tracking threshold was changed.

The terminal chain is separate from generations `83/84`: generation `85`
restored a certified MAIN+BACKUP successor, and later generations continued
to commit and activate through generation `35`; waypoint `7` was accepted.
The first observed terminal transition in this artifact is generation `35`
activation at simulation `109.936 s`, followed about `1.7 ms` later by the
execution-anchor gate (`raw anchor=0.822 m`, projected `0.870 m`, limit
`0.250 m`). The runtime committed emergency generation `36`, retained its
bounded recovery endpoint, then rejected the expired `STOPPED_HOLD` endpoint
because `known_free=1` but `near_execution=0`, and handed over to PX4 Hold.
This establishes the chronology and separates the recoverable gen83/gen84
deadline failures from the terminal tracking/recovery chain; it does not yet
attribute the underlying anchor excursion to PX4, LIO, estimator alignment or
physical control.

## Candidate provenance

- Repository: `/home/letandat/Dev/uav-navigation`
- SITL source checkout: `/tmp/uav-navigation-phase-release-1358e834`
- Source commit: `c0f9d3727cb4c64750b448592f4e5a11cd65470a`
- Release manifest: `/tmp/uav-navigation-phase-release-1358e834/install/.uav_navigation_build_manifest.json`
- Manifest SHA-256: `489b31b222e31895f2013b00b0e086f0829dba269d4be1487f22a9e45565f9f4`
- Manifest status: `VALID`, `git_dirty=false`
- PX4 checkout: `/home/letandat/Dev/Autopilot`, `deaff86e`, dirty/project-customized; no PX4 source change in this campaign.
- Scenario: `long_three_pillars_multiwaypoint`, seed `0`, 9 waypoints, 3 pillars, requested speed as shown below, MAIN `3/2/4`, scheduler `10 Hz`.

## Runs

| Run | Mode / requested speed | Verdict / outcome | Waypoints | Collision / clearance | PVA failure | Safety stop | Tracking bypass | Total planning p50/p95/p99/max (ms) |
|---|---|---|---|---:|---:|---:|---:|---:|
| `external-mode-check-20260908T155255-336982` | relaxed / 3 m/s | `PASS` / `COMPLETE` | 0–8 | 0 / 1.5828 m | 0 | no | 3798 | 39.137 / 43.147 / 49.796 / 55.020 |
| `external-mode-check-20260908T160007-339478` | adaptive / 3 m/s | `BLOCKED` / `PAUSED_SAFETY_STOP` | 0 | 0 / 6.9338 m | 0 | yes | 0 | 40.166 / 94.871 / 94.871 / 94.871 |
| `external-mode-check-20260908T160124-341043` | off / 3 m/s | `BLOCKED` / `PAUSED_SAFETY_STOP` | 0 | 0 / 7.2807 m | 0 | yes | 0 | 40.105 / 40.255 / 41.694 / 41.694 |
| `external-mode-check-20260908T160251-342652` | relaxed / 4 m/s | `BLOCKED` / `PAUSED_SAFETY_STOP` | 0 | 0 / 7.1494 m | 0 | yes | 0* | 40.150 / 40.393 / 93.250 / 93.250 |
| `external-mode-check-20260908T160924-345641` | relaxed / 3 m/s repeat | `BLOCKED` / `PAUSED_SAFETY_STOP` | 0–5 | 0 / 0.9368 m | 277 | yes | 3497 | 40.109 / 42.728 / 44.816 / 48.861 |

`*` The 4 m/s report records the relaxed configuration, but no completed tracking-bypass counter was observed before the early stop; it is not evidence that the relaxed allowance was effective for the stopping event.

### Artifact paths

- [relaxed 3 m/s report](../../runtime/external-mode-check-20260908T155255-336982/report.json)
- [adaptive 3 m/s report](../../runtime/external-mode-check-20260908T160007-339478/report.json)
- [off 3 m/s report](../../runtime/external-mode-check-20260908T160124-341043/report.json)
- [relaxed 4 m/s report](../../runtime/external-mode-check-20260908T160251-342652/report.json)
- [relaxed 3 m/s repeat report](../../runtime/external-mode-check-20260908T160924-345641/report.json)

Each artifact also contains `metadata.json`, generated config snapshots, raw node logs, `planning_timeline.jsonl`, `execution_timeline.jsonl`, `scenario.json`, and `rosbag/`.

## Interpretation

The relaxed 3 m/s run completed all nine waypoints with zero observed collision and zero PVA command failures, but it is explicitly diagnostic-only: `qualification_eligible=false` and finite MAIN tracking gates were suppressed. The paired adaptive and off runs both stopped fail-closed before waypoint 1; neither had collision or PVA transport failure. Therefore the run does not establish Q1 stability or planner-only success. It establishes that the current 3 m/s mission remains dependent on the experimental tracking allowance under this candidate.

The 4 m/s relaxed run stopped after waypoint 0 and did not achieve the requested-speed criterion (`p95=2.8868 m/s`), so no 4 m/s capability claim is made. No 5 m/s run was started after the 4 m/s result.

The relaxed 3 m/s repeat did not reproduce the earlier completion. It accepted waypoints 0–5, then stopped fail-closed with 277 PVA command failures and minimum observed clearance 0.9368 m. The final tracking-envelope evidence is in the strict BACKUP path: `reverse=0.770/0.750 m`, message `generation=16`, `role=BACKUP`, followed by `PVA command anchor is not near vehicle` and PX4 Hold handover. The experimental allowance suppresses eligible MAIN tracking gates only; it does not suppress BACKUP/EMERGENCY/safety-suffix tracking validation. The one relaxed completion is therefore non-repeatable diagnostic evidence, not Q1 stability.

The paired adaptive/relaxed recovery traces also show a separate recovery-feasibility frontier. The first post-initial `main_minco` failures retained command authority and/or a certified BACKUP selection, so they were recoverable. Later PlanFromRest backup attempts were rejected at `aligned_sfc` before visibility-hull/aligned-hull/known-free checks (`visibility_hull_pass_count=0`, `aligned_sfc_built_count=0`, `known_free_check_count=0`). The adaptive run was not hard-deadline limited at that first recovery rejection; the relaxed 4 m/s run had a later deadline overrun. This is currently a recovery backup-feasibility owner candidate, not yet a production root cause.

The relaxed 3 m/s run's planning record had 76 complete records, 0 observed hard-deadline flags in the rolling trace, and total planning max 55.020 ms. The failed paired runs are not latency qualification evidence: adaptive had a 94.871 ms tail and one rolling hard-deadline record; off stopped before useful mission coverage; relaxed 4 m/s had one rolling hard-deadline record. The repeat relaxed 3 m/s run had 59 records, 209 retry attempts and 51 valid retry builds; its final `experimental_tracking_suppressed=3497` is a cumulative metric, not a sum of rolling snapshots.

## Qualification status

`Q1 = NOT_READY`. No run in this campaign is authoritative qualification evidence. The next production investigation remains the first non-experimental cause of the tracking/safety-stop chain; no PX4, LIO, threshold, planner-rate, or recovery behavior was changed by this campaign.

### Exact terminal episode in the repeat

Generation `16` was activated at simulation time `162.212 s` and remained
MAIN until the normal analytic boundary at `166.932 s`. The setpoint role
changed from MAIN to BACKUP on the next 16 ms sample; the active backup was
available, state source age was `0`, receive age was about `1.5 ms`, and no
lease failure was recorded. The generation-72..78 nominal renewal failures
occurred while this generation remained executable and are therefore
recoverable, not the first terminal event.

The first strict BACKUP tracking-envelope rejection was at `167.524 s`,
generation `16`, role `BACKUP`, message `9121`, with reverse error
`0.770/0.750 m`. The measured position was `[112.940,5.336,2.996]` versus
command `[112.193,5.095,3.002]`; odometry header/receive age was `12 ms`.
Messages `9122..9135` continued the strict failure. The subsequent PVA command
failures were one contiguous stream, samples `9136..9412` at `167.780..172.196
s`, exactly 277 rejected samples. Runtime then reported generation-16
immutable revalidation failure and published rejected commands with no active
bundle before PX4 Hold.

This makes the first observed fatal boundary `strict BACKUP tracking-envelope
exhaustion`, followed by fail-closed command invalidation. It is not yet a
root-cause attribution for the underlying BACKUP residual; synchronized
command/setpoint/estimator/truth analysis is still required.
