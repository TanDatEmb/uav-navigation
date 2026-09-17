# Certified-iterate checkpoint and terminal STOP: 5 m/s diagnostic matrices

Date: 2026-09-17, Asia/Ho_Chi_Minh.

Latest follow-up: clean commit `14e7d367` completed a second nine-run matrix
after the terminal STOP correction. Mission completion was 2/9 and report
PASS 0/9. The product target remains unmet. The original frozen round below
is preserved separately; current evidence is in the post-fix section.

## Original checkpoint-round verdict

All nine requested simulations and their reports completed. Mission completion
was 1/9; report PASS was 0/9 (two FAIL, seven BLOCKED). Recorded collisions,
PX4 failsafes, mapping cloud loss and scenario-writer loss were zero in all
nine runs. None of this constitutes qualification or collision-free stopping
proof. The product target of stable, smooth 5 m/s movement remains unmet.

The checkpoint is reachable through the production MAIN/BACKUP admission
path, but nominal feasibility alone does not solve successor availability.
The matrix also separated a terminal mission-completion defect from planner
failure: 5WP repetition 2 reached the last goal and held it, yet timed out.

## Frozen inputs

- Navigation HEAD: `0cd68221b1de5c82028793d746c57daa037e7ad2`, dirty.
  Complete source fingerprint:
  `384664991067f9ef46106e508b47be27c0203657e964de323f36b3b10216abcc`.
  The dirty source includes the checkpoint, ownership-evidence correction and
  numerical replay diagnostics; results must not be attributed to HEAD alone.
- Authoritative Release manifest SHA-256:
  `dd769e7cd77acd62508086355f7cf9a7e0dbf8e8dac96ebc2bfc2c9936418ab6`.
- PX4 HEAD: `deaff86ee335dd697677bcfc2415a23878e1b895`, dirty;
  source fingerprint
  `25341a3df3acb2e557ef386affdf7a67f070603e924eb8107e8d291d22e2081f`;
  SITL binary SHA-256
  `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`.
  Each session separately captures mutable PX4 parameter/dataman inputs.
- Legacy positive profiles, motion preset `nominal`, seed 0, requested cruise
  speed override 5 m/s, V/A/J 5/5/8, visibility 40 m / 4,096 endpoints,
  ROS domain 42 and XRCE port 8892. No fault injection or tuning between runs.
- Tracking gates remain 0/0/0, the existing relaxed diagnostic policy. Its
  suppression of tracking, typed-health response and stopped-hold rejection
  prevents qualification. Lifecycle, reference lineage and acceptance-policy
  completeness also remain blockers, including in the completed mission.
- Simulations ran sequentially without concurrent builds/tests. Nominal
  snapshot capture was OFF. The sequencer was briefly held during postflight
  reporting for an isolated mission reproducer; no simulation was active then.
- The older 2026-09-16 matrix used motion preset `fast`, not `nominal`.
  It is historical context, not a paired performance A/B comparator.

Command template (repeat 1--3 for each profile):

```bash
env -u UAV_NAVIGATION_NOMINAL_SNAPSHOT_DIR \
  -u UAV_NAVIGATION_NOMINAL_SNAPSHOT_INCLUDE_WORLD \
  -u UAV_NAVIGATION_NOMINAL_SNAPSHOT_FAILURE_ONLY \
  python3 tools/runtime/runner.py external-mode-check \
  --map-profile PROFILE --speed-cap-mps 5.0 --map-seed 0 \
  --visibility-max-endpoints 4096 --visibility-range-max-m 40.0 \
  --ros-domain-id 42 --xrce-port 8892 --experiment-id RUN_ID
```

2WP uses `long_three_pillars_speed` (140 m straight terminal goal); 5WP uses
`long_three_pillars` (48 m outbound, orthogonal return to [41,0,3]); 9WP uses
`long_three_pillars_multiwaypoint` (alternating detours to [140,0,3]).

## Denominator-preserving results

All sessions are under `.artifacts/runtime/`. Accepted indices include the
takeoff origin WP0. Planning distributions cover recorded decision samples,
not every timer trigger. Certificate time is the maximum per-record aggregate,
not one validator call. Observed maxima and p99 are not deadline upper bounds.

| Case | Session | Report / outcome | Accepted | Planning p99/max ms (n) | Checkpoint selections / current records | Certificate aggregate max ms |
|---|---|---|---|---|---|---|
| 2WP-1 | `external-mode-check-20260917T011411-29321` | FAIL / COMPLETE | 0,1 | 50.488/50.488 (25) | 3/25 | 0.261 |
| 2WP-2 | `external-mode-check-20260917T011617-33729` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 80.689/80.689 (8) | 0/6 | 15.008 |
| 2WP-3 | `external-mode-check-20260917T011748-37356` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 69.060/69.060 (10) | 1/9 | 10.486 |
| 5WP-1 | `external-mode-check-20260917T011903-40949` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 66.473/66.473 (10) | 0/7 | 4.060 |
| 5WP-2 | `external-mode-check-20260917T012031-44350` | FAIL / WALL_TIMEOUT | 0,1,2,3 | 86.635/86.635 (25) | 7/23 | 13.969 |
| 5WP-3 | `external-mode-check-20260917T013622-52473` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 70.725/70.725 (1) | 0/1 | 10.134 |
| 9WP-1 | `external-mode-check-20260917T013728-56165` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 80.275/80.275 (7) | 3/7 | 19.881 |
| 9WP-2 | `external-mode-check-20260917T013834-59591` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 80.525/80.525 (6) | 2/6 | 17.180 |
| 9WP-3 | `external-mode-check-20260917T013936-62944` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 80.261/80.261 (7) | 4/7 | 15.953 |

Completion by route: 2WP 1/3, 5WP 0/3, 9WP 0/3. Non-completion is retained as
2/3, 3/3 and 3/3 respectively; it is not removed from timing denominators.
The 91 current diagnostic records contain 20 checkpoint selections, 2,009
certificate calls and 274,121 us of certificate work. Sparse records are not a
complete optimizer-call census or a complete latency distribution.

| Metric | Per-run p99 range | Largest observed max |
|---|---|---|
| Mapping callback | 69.183--88.055 ms | 126.142 ms |
| Planning worker | 49.458--82.973 ms | 88.947 ms |
| World snapshot export | 15.577--18.340 ms | 22.027 ms |
| PX4 local-position setpoint source gap | Not reported as p99 | 32 ms; zero stale events |
| Propagated odometry source gap | Not reported as p99 | 28 ms; zero stale events |

Measured speed reached 6.004 m/s, while the largest setpoint speed was
4.967 m/s. These descriptive maxima include the entire captured run, including
braking/hover; tracking remains NOT_EVALUABLE. They are not a qualified
closed-loop envelope or evidence attributing error to LIO or PX4.

## Findings and next minimum changes

1. **CONFIRMED, integrated reachability:** In 2WP-1, checkpoint cycles 1, 73
   and 206 passed BACKUP and reached immediate admission or staging. The
   checkpoint is not an alternate authority path. In 9WP-3, cycle 27 selected
   a checkpoint but was rejected for `backup_dynamics`; cycle 28 subsequently
   staged generation 2, which activated at source 25.552 s. Cycles 25/26 had
   rejected at `no_complete_bundle_at_deadline`. Downstream gates still
   distinguish nominal feasibility from an executable proposal.
2. **CONFIRMED, successor availability remains open:** All three 9WP runs
   stopped before WP1 despite selected nominal checkpoints. Deadline and
   BACKUP failures must be classified separately. Do not enlarge the solver
   budget, weaken jerk limits or replace execution ownership from this result.
3. **CONFIRMED, terminal STOP predicate:** 5WP-2 accepted WP0--WP3, then held
   the final endpoint without COMPLETE. At source 243.968 s, LIO position was
   approximately [41.1963,-0.0178,2.8478] with speed about 0.0076 m/s. A
   deterministic test against the exact Release mission library reproduced
   non-completion at 0.249 m goal error inside its unchanged 0.8 m radius.
   `ExecutingWaypoint` applies `passThroughAcceptance()` to STOP as well as
   PASS_THROUGH; ordered projection selected earlier segment 0 and rejected
   STOP. An isolated one-predicate correction used geometric current-position
   acceptance for STOP, preserved measured-speed/confirmation/hold gates, and
   passed the reproducer plus all 43 existing mission tests. Canonical
   integration and fresh SITL evidence are separate follow-up work.
4. **CONDITIONAL, postflight report scaling:** 5WP-2 report generation consumed
   minutes of CPU after simulation cleanup. Its JSON is 168,320,320 bytes.
   Lifecycle reduction currently scans all authorizations for every publish;
   this is a quadratic candidate for a measured, parity-tested indexing change,
   not proof that it explains all report time. This cost is outside the command
   path. Profiling attachment was unavailable; the live worker was allowed to
   finish rather than restarted.

The opt-in snapshot discriminator
`external-mode-check-20260917T010955-24315` is separate from the nine-run
denominator: PAUSED_SAFETY_STOP, 0 selected checkpoints in 10 current records,
429 certificate calls / 50,760 us. Its preflight LiDAR source gap (10.8 to
11.4 s) does not establish causality for the final handover around 45 s.

After the matrix, canonical `make test` passed all 81 currently selected CTest
entries. The Python check ran 380 tests successfully with one explicit
artifact-dependent skip (379 executed passes), not 380 executed passes.

Architecture remains option A plus bounded ownership completion: retain one
timeline store and the existing execution episode. Fix the mission predicate
at its current External Mode owner and close complete-bundle availability and
evidence lineage with focused tests/replay. A new large coordinator or process
split is not justified by this matrix.

## Post-fix terminal STOP round

All nine fresh simulations **and** reports are terminal. Completion by route
is 2WP 1/3, 5WP 1/3 and 9WP 0/3. Non-completion is 2/3, 2/3 and 3/3,
respectively (66.7%, 66.7%, 100%; three repetitions are a small sample).
Report PASS is 0/9: three FAIL, six BLOCKED. There were two COMPLETE outcomes,
six PAUSED_SAFETY_STOP outcomes and one FAILED_COMPONENT outcome. The latter
is an odometry-lease rejection, not a permission failure. No failed run was
removed or replaced. This is not qualification or proof of stable/smooth
5 m/s operation.

### Frozen follow-up inputs

- Navigation: clean `14e7d367552004ca161420531824b4e6a49f1711`, source
  fingerprint `18e02b423b7ea8728aecd8a36902245201bfe9e1dadd85ef61d9cc3c64b08333`.
- Authoritative full Release manifest SHA-256:
  `74edfb7bcce8c97bcb9f7b4d7a44d975b28e79c97febd03c21efdc631769b7d3`.
  Each report's `provenance.manifest.source` and manifest digest match these
  values. Canonical build passed all 23 packages; mission tests passed 44/44,
  selected CTest entries 81/81, Python 379 executed passes plus one explicit
  artifact-dependent skip, and seven auxiliary tests passed.
- External PX4 remains the separately captured dirty source/binary described
  above: HEAD `deaff86ee335dd697677bcfc2415a23878e1b895`, source fingerprint
  `25341a3df3acb2e557ef386affdf7a67f070603e924eb8107e8d291d22e2081f`, binary
  `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`.
  Mutable parameter/dataman snapshots remain session-specific.
- Same profiles, positive case, `nominal` preset, seed 0, requested cruise
  5 m/s, MAIN V/A/J 5/5/8, visibility 40 m / 4,096 endpoints, DDS domain 42,
  XRCE 8892, no injected fault, nominal-problem capture OFF. BACKUP physical
  limits remain separate (12/12/30), not the MAIN control envelope.
- Sequential order: three 5WP, three 2WP, three 9WP. No concurrent build/test
  or behavior/configuration change. Small read-only artifact/source inspections
  occurred; this is not a claim of an otherwise workload-free machine.
- Tracking remains the existing relaxed diagnostic policy, including its
  documented suppressed gates. Every report assessment is NOT_EVALUABLE,
  evidence INCOMPLETE and qualification ineligible, with the same six blocking
  codes listed in the original round. Provenance VALID is not qualification.

The command template above applies with explicit `--test-case positive
--motion-preset nominal` and experiment IDs
`terminal-stop-matrix-{2wp,5wp,9wp}-r{1,2,3}-20260917`.

### Full follow-up denominator

Accepted indices below come from
`scenario.json.waypoint_acceptance_events[].accepted_waypoint_index`.
The event's `waypoint_index` is the **next** index; the ephemeral batch wrapper
used that field in its printed `accepted_waypoints`, so that printed list is
not the acceptance witness. No product recorder/reducer data was changed.

| Case | Session under `.artifacts/runtime/` | Report / outcome | Accepted | Planning p99/max ms (n) | Checkpoint selections / current records | Certificate aggregate max ms |
|---|---|---|---|---|---|---|
| 2WP-1 | `external-mode-check-20260917T015652-94541` | FAIL / FAILED_COMPONENT | 0 | 84.242/84.242 (33) | 5/29 | 18.507 |
| 2WP-2 | `external-mode-check-20260917T015841-98513` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 81.768/81.768 (17) | 1/16 | 23.219 |
| 2WP-3 | `external-mode-check-20260917T020007-102032` | FAIL / COMPLETE | 0,1 | 80.195/80.195 (31) | 2/29 | 13.227 |
| 5WP-1 | `external-mode-check-20260917T015138-83125` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 73.719/73.719 (11) | 0/10 | 2.921 |
| 5WP-2 | `external-mode-check-20260917T015251-86803` | FAIL / COMPLETE | 0,1,2,3,4 | 80.764/80.764 (24) | 2/24 | 16.963 |
| 5WP-3 | `external-mode-check-20260917T015438-90782` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 80.351/80.351 (9) | 0/9 | 21.217 |
| 9WP-1 | `external-mode-check-20260917T020158-105668` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 80.306/80.306 (7) | 3/7 | 23.265 |
| 9WP-2 | `external-mode-check-20260917T020304-108922` | BLOCKED / PAUSED_SAFETY_STOP | 0 | 80.479/80.479 (8) | 2/8 | 22.082 |
| 9WP-3 | `external-mode-check-20260917T020411-112209` | BLOCKED / PAUSED_SAFETY_STOP | 0,1,2 | 80.598/80.598 (22) | 6/22 | 18.907 |

The 162 decision-trace records include 154 current checkpoint-diagnostic
records, 21 checkpoint selections, 3,207 certificate calls and 408,806 us of
summed certificate work. These remain sparse captured records, not a census
of all solver jobs. Max certificate time is a per-record aggregate, not one
call. The nine separate `navigation_mapping.timing_distributions` report:

| Metric | Per-run p99 range | Largest observed max |
|---|---|---|
| Mapping callback | 61.332--108.986 ms | 144.086 ms |
| Planning worker | 37.467--82.683 ms | 87.199 ms |
| World snapshot export | 14.432--20.762 ms | 24.405 ms |

These maxima are not upper bounds. No paired ON/OFF overhead or causal
performance improvement is established by comparing the two rounds.

### Adversarial findings and next direction

1. **CONFIRMED, integrated STOP reachability:** 5WP-2 accepted WP0--WP4.
   Its final measured STOP acceptance was 0.182362 m error and 0.049766 m/s.
   `logs/external_mode.log:159`--`:171` show terminal hold, above-limit speed
   samples and eventual measured acceptance. The deterministic regression
   isolates the predicate change; this one integrated completion does not
   establish 3/3 availability or attribute all round differences to that fix.
2. **CONFIRMED, clearance rejection:** 5WP-3 reports minimum ground-truth
   collision clearance 0.078432 m at `long_three_pillar_03`, below its frozen
   `scenario.minimum_collision_clearance_m=0.1`. Recorded collisions, PX4
   failsafes, scenario-writer drops and mapping cloud drops are zero in all
   nine runs; zero collision does not erase this clearance failure. Root
   cause/which layer violated its assumptions remains INCONCLUSIVE without
   synchronized state/command/truth/world residuals. The relaxed policy's
   known suppression of tracking/health responses is not a qualified envelope.
3. **CONFIRMED, receive-age containment:** 2WP-1
   `logs/external_mode.log:131`--`:132` rejects generation 22 with
   `RECEIVE_STALE`, source age 164 ms and receive age 207.942 ms, then hands
   over to PX4 Hold. The report also records one active propagated-odometry
   source gap of 512 ms and wall-arrival gap of 637.962446 ms around the same
   interval (source 52.116 to 52.628 s; arrival wall
   1789610287480217380 to 1789610288118179826 ns). The external-odometry bridge
   separately records `DT_TOO_LARGE`. This supports a missing-update interval,
   not a proven LIO/DDS/PX4 root cause. PX4 setpoint source-gap max remains
   28 ms for this run, showing why output continuity alone is insufficient.
   External Mode already has a separate state-input thread; adding threads
   without dispatch/lock/resource attribution is not the selected fix.
4. **CONFIRMED, mixed availability failures:** In 9WP-1, accepted nominal
   candidates are followed by BACKUP swept-tube failures and MAIN deadline
   failures. `logs/mapping.log:306` records seed V/A/J approximately
   4.890/9.176/28.771, feasible under BACKUP 12/12/30, followed by
   `last_reject_stage=known_free`, failure 8 (`kCertificateTubeBlocked`), cell
   1 (`kUnknown`), role 1 (`BACKUP`). The coarse outcome `backup_dynamics`
   is not causal proof of a dynamics violation. Likewise optimized MAIN
   certificate stage 4 is `kDynamics`, not continuity/corridor failure merely
   because a finite corridor residual is logged. In 9WP-3, progress reaches
   WP2 before an actual later corner corridor rejection (0.01977 > 0.01),
   then invalid-window revalidation (failure 1, zero samples). The code alone
   does not establish the expiry cause; do not classify every stop as the same
   first-waypoint error.
5. **CONFIRMED, limited legacy freshness metric:**
   `NavigationMode::last_state_age_s_` is reset to -1 and read by periodic
   logging, but has no producer assignment in the current source. It is not
   measured decision-age evidence. The explicit freshness rejection and
   source/arrival witnesses above remain distinct authoritative observations.
   9WP-2 also has raw stream stale-event counts despite small source gaps;
   source-stale counts are zero there. Phase/observer-dispatch attribution is
   required before calling those sensor interruptions or dropping the events.

Architecture remains A plus bounded B. The next experiment must close the
complete MAIN/BACKUP/world readiness contract and measured corner/stop
tracking, alongside a source-to-use discriminator for the odometry interval.
Replay must separate geometric/numerical dynamics, UNKNOWN swept tube,
deadline and invalid-window admission. Preserve the 0.1 m clearance gate, the
5/5/8 MAIN and 12/12/30 BACKUP limits, stop gates and all failed runs. No large
coordinator, process split, budget increase, or additional fallback is
justified by this round. Evidence lineage/eligibility and representative
recorded-data/hardware validation remain open.

The temporary terminal-overlap source copies and binaries were removed after
canonical regression and integrated evidence were available. The committed
regression can reproduce the case; all runtime artifacts are retained. No new
parallel execution path or cleanup of user files was introduced.

## Completion-focused discriminator and architecture choice

The post-STOP denominator remains 2/9 complete, with report PASS 0/9.
Rechecking each terminal chain classifies six of the seven non-completions as
loss of a complete successor before the active execution ends, and one as an
odometry lease rejection. This is a terminal-mechanism census, not proof that
all six share one numerical root cause. The low clearance in 5WP-3 is a
separate safety failure, not the causal reason for its mission stop. The
completed 5WP-2 itself contains eight deadline-failure trace records, so a
single failed solve or sparse failure frequency cannot explain completion.

For 9WP, eleven trace records have an observed hard deadline, ten before
BACKUP begins. Durations below are differences of the same transaction's
steady stage stamps, not the cached `time_consuming_` fields:

| 9WP repetition | MAIN n / median / max ms | A* n / median / max ms | BACKUP n / median / max ms | Deadline records / before BACKUP |
|---|---|---|---|---|
| 1 | 7 / 43.930 / 77.740 | 7 / 2.360 / 12.360 | 4 / 2.340 / 4.270 | 3 / 3 |
| 2 | 8 / 61.870 / 77.840 | 8 / 2.290 / 16.140 | 4 / 3.760 / 7.390 | 4 / 4 |
| 3 | 22 / 36.020 / 76.580 | 22 / 2.670 / 22.760 | 11 / 8.700 / 14.300 | 4 / 3 |

This supports prioritizing complete-bundle readiness and the problem sent to
MAIN. It does not justify an executor/process rewrite, making BACKUP optional,
raising budgets, or treating A* as universally negligible: 2/5WP A* tails reach
about 66 ms. BACKUP also fails strict known-free swept tubes even when its
V/A/J is within its separate physical limits.

### Exact-current 9WP capture

Clean observational commit `ab5b4c58b9a6061a73c802ce84e68cc7bcebcdb3`, Release
manifest `7b950aba1b595f1b064b3f4afb336ba6ee43ad7bbf49bf7fa9ca248545e88514`,
source fingerprint
`dd7a9c37a716dd161510e946db2fa3f2d0a288fd6edcea52f277ba6832259eb6`
ran `completion-readiness-9wp-discriminator-20260917` under the same nominal
5 m/s/seed-0 scenario. World capture was ON, unlike the nine-run baseline;
therefore this is not a matched performance/completion comparison. Session
`external-mode-check-20260917T022857-134431` was BLOCKED /
PAUSED_SAFETY_STOP, accepted only WP0 and stopped near [8.71,1.35,3] before
WP1 [20,5,3]. It does not demonstrate improved progress or completion.

The snapshot directory
`.artifacts/runtime/completion-readiness-9wp-discriminator-20260917`
contains ten written snapshots from eleven submissions, one dropped, zero
pending/write errors. The sidecar's accounting COMPLETE / capture_complete
means accounting was finalized; its writer_capture_complete_at_process_stop
is false. **The capture is not lossless.** Replay findings apply to identified
written problems, not a complete solve census.

The source-to-use observation is integrated: 534/534 PX4-update records are
usable, 427 distinct states, zero missing/invalid records or epoch mismatch.
Observed receiver-mutex wait max is 0.000916 ms and receive-to-snapshot max
25.387104 ms. This rejects a receiver-mutex bottleneck **in this run only**;
it does not explain the older 512 ms odometry interval or prove DDS/producer
latency/PX4 acceptance. Instrumentation overhead has no paired OFF/ON bound.

### Selected change: ready-first mandatory feasibility

Claim: when no certified seed exists, waiting until the optional-refinement
cutoff before preserving a fully certified accepted iterate can exhaust the
budget needed for BACKUP/world finalization. Strong counterargument: checking
every iterate early can itself consume the deadline. The old frozen 5WP
snapshots 13--15 distinguish these conditions:

| Frozen problem | Old 40/80 result / cert calls / aggregate us | Naive 0/80 result / calls / us | Weight-independent screen, 40/80 result / calls / us |
|---|---|---|---|
| 13, cycle 161 | candidate / 2 / 344 | candidate / 197 / 17322 | candidate / 9 / 984 |
| 14, cycle 162 | candidate / 78 / 6409 | deadline / 241 / 19774 | deadline / 0 / 0 |
| 15, cycle 163 | candidate / 68 / 5674 | deadline / 281 / 21407 | candidate / 1 / 257 |

These are serial individual probes, not statistical timing bounds. Preserve
the unfavorable problem 14: the new screen removes wasted certification but
the first search attempt can still exhaust 80 ms before reaching its retry.

In current 9WP snapshot 1 / cycle 21, the existing 40/80 path fails after
about 1,880 evaluations and 118 certificates (11,175 us aggregate). The new
40/80 path selects attempt 1 / iteration 8 after eleven evaluations, one
certificate (681 us). Snapshot 6 / cycle 27 likewise selects iteration 9
after eleven evaluations and one certificate (754 us). Both pass optimized
nominal corridor/route/V/A/J/flatness and offline world checks, with unchanged
boundaries/limits. Their MAIN durations are about 16.09 and 17.74 s: readiness
does not establish smooth sustained 5 m/s. Recovery snapshot 7 / cycle 83
still exhausts 80 ms, zero certificate calls, no candidate. Every replay has
`complete_executable_bundle=0`; no BACKUP/admission/lease is invented.

The implementation keeps objective cost/gradient/weights unchanged. It
records raw acceleration/jerk maxima from the already computed samples and
uses product limits plus the existing numerical boundary policy to reject
definitely invalid iterates cheaply. Only the independent full nominal
certificate can select a checkpoint; all downstream gates remain mandatory.
The future-cutoff regression fails on old source and passes after this change,
along with mandatory-feasibility, cancellation and expired-hard-deadline cases.
Canonical Release build passed (23 packages); `make test` exited zero with
selected component suites, 386 Python tests (one explicit artifact-dependent
skip) and seven runtime auxiliary tests. Adversarial diff review found no
concrete P1/P2 bypass or checkpoint identity incoherence. An earlier build
compiled all packages but provenance rejected a concurrent report edit; it
was rerun on fixed source. Integrated completion verification remains pending.

### Remaining structural work, kept separate

- **CONFIRMED input defects / conditional completion impact:** current capture
  guide lengths reach 25.341 m against a 20 m local-window contract. Some
  guide suffixes fold; some initialized junctions repeat a position with
  nonzero durations. Seed jerk ranges about 7,058--125,434 m/s³. Correct
  ordered guide geometry and coherent point/time allocation before tuning
  solver throughput; do not infer every deadline was caused by the fold.
- **DESIGN_DEBT:** `SimplifySFC` bypasses compaction for the entire chain if any
  route gate exists. Gate-aware compaction must preserve marker order,
  metadata, the incoming boundary-point overlap and representable transitions,
  not merely pairwise overlap. Exploratory regression tests confirmed the
  blanket behavior but were removed when this separate implementation was
  deferred; no new dead tests, coordinator, fallback or command path remains.
- **EVIDENCE_GAP:** complete successor readiness must be evaluated jointly
  with strict known-free BACKUP reachability, active remaining horizon,
  state/truth tracking and clearance. Completion is the primary operational
  denominator; faster MAIN or a diagnostic PASS cannot substitute for it.

The selected behavior change must finish with a clean Release repeated
2/5/9WP matrix, three runs each at 5 m/s, snapshots OFF, unchanged gates and
configuration. Recorded-data/hardware validation and qualification eligibility
remain outstanding; no small successful replay closes the product goal.
