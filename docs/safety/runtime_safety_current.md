# Runtime Safety Current Contract

## Purpose and target architecture

This authoritative routine-read safety contract points to lossless history;
it does not replace historical evidence. Ownership remains:
`FAST-LIO estimator -> product-owned ROG-backed WorldModel -> SUPER MAIN plus
certified BACKUP -> immutable committed bundle -> trajectory controller/OMMPC
-> PX4 ROS 2 External Mode`.
Registration is not a planning map; WorldModel owns UNKNOWN/OUT_OF_MAP semantics.
A failed candidate never mutates the committed generation. Core mission-progress ownership is specified in [the authority cut](mission_authority_cut.md).

Source snapshot for this migration: `HEAD 82ef05ca161a18cf4d0e0dfd40ad257ec6a87509`.
The original ledger remains byte-identical in the archive; migration metadata
and the deterministic ID mapping are in the [decision index](runtime_safety_index.md).

## Mandatory reading rule

Before modifying estimation, mapping, planning, control, PX4 integration,
runtime budgets, safety gates, bypasses, or validation thresholds, read this
file.

Read targeted history from the [index](runtime_safety_index.md) and
[archive](archive/runtime_safety_legacy_full.md) when changing a referenced
gate/contract, investigating regression lineage, revisiting a
rejected/reverted design, changing a temporary bypass, or auditing provenance.
Do not full-read the archive as routine context. Do not use this split to
weaken the requirement to retrieve relevant history.

## Contract interpretation

- `IMPLEMENTED` means a source path exists, not qualification.
- `QUALIFIED` requires declared evidence; component/replay/partial integration is not flight acceptance.
- `DIAGNOSTIC_ONLY` and `EXPERIMENT` are not product authority or qualification.
- `BLOCKED`, `INCONCLUSIVE`, `NOT_EVALUABLE`, missing/ambiguous evidence is not PASS;
  safety decisions remain fail-closed.
- History is verbatim; the index uses `UNRESOLVED`, never invented supersession,
  ownership or qualification without source proof.

## Non-negotiable invariants

1. Missing or ambiguous source time, frame, identity, reset epoch, freshness,
   world revision, certificate, or lifecycle witness is not evaluable; it is
   never converted to zero, observer time, partial coverage, or a default PASS.
2. `UNKNOWN` and `OUT_OF_MAP` remain non-traversable for certified BACKUP.
   Planner-only `allow_unknown` and any relaxed visibility profile are
   diagnostic-only and cannot authorize flight or qualification.
3. A candidate is executable only after atomic goal, localization, command
   lease, latest-world, continuous corridor, dynamics/flatness, and swept-world
   checks. Newer world identity invalidates an older candidate.
4. The committed command remains authoritative until a valid successor is
   atomically committed, a certified measured-state brake is activated, or
   PX4 Hold is requested through the existing fail-closed boundary.
5. Waypoint progress is measured and ordered. A planned endpoint does not prove
   waypoint acceptance or terminal completion.
6. External Mode receives finite, frame-correct, temporally continuous P/V/A
   setpoints. PX4 tracking and physical limits remain distinct from planner
   nominal limits; jerk is not an executed PX4 setpoint.
7. Every retained MAIN/BACKUP/emergency command remains bounded by its existing
   lease, tracking, timing, world, and role certificate. A retry or fallback
   may not silently become product authority.
8. Hardware Mid-360 visibility remains blocked until an immutable visibility
   certificate and runtime verifier exist.
9. Threshold changes require distributions from repeated SITL and
   representative recorded data. A single smoke/SITL result, component test,
   partial motion, or dataset shadow result cannot close a hard gate.
10. Documentation cleanup must not change thresholds, planner/controller
    configuration, UNKNOWN policy, command ownership, deadlines, or runtime
    behavior. Any bypass must name owner, scope, impact, evidence, removal
    condition, and verification command in the same change.

## Status model

These dimensions are independent. Historical free-form wording is not rewritten
to fit them; the index records `UNRESOLVED` when a mapping is not proven.

| Dimension | Allowed current values | Meaning |
|---|---|---|
| Lifecycle | `ACTIVE`, `SUPERSEDED`, `REVERTED`, `REMOVED`, `REJECTED`, `UNRESOLVED` | Whether a decision still has current lifecycle authority. |
| Implementation | `NOT_IMPLEMENTED`, `IMPLEMENTED`, `UNRESOLVED` | Source/config implementation only; never qualification. |
| Evidence | `UNVERIFIED`, `UNIT_VERIFIED`, `COMPONENT_VERIFIED`, `INTEGRATION_PARTIAL`, `INTEGRATION_VERIFIED`, `QUALIFIED`, `BLOCKED`, `NOT_EVALUABLE`, `UNRESOLVED` | Evidence boundary, with fail-closed meanings preserved. |
| Authority | `PRODUCT`, `DIAGNOSTIC_ONLY`, `EXPERIMENT`, `UNRESOLVED` | Whether behavior can define the product contract. |

## Active hard gates

The following IDs are active or require closure. Values are concise current
authority; full owner, derivation, false-accept/false-reject analysis, evidence,
and closure commands remain in the archived gate register. `ACTIVE` does not
mean `CERTIFIED`.

| ID | Current contract | Gate state |
|---|---|---|
| HG-001 | SUPER absolute budget: A* attempt 40 ms, A* total 80 ms, solve 180 ms, future-state lead 200 ms. | ACTIVE / PROVISIONAL |
| HG-002 | Independent continuous normalized corridor-plane violation limit: 0.01 m. | ACTIVE / PROVISIONAL |
| HG-004 | Physical/BACKUP envelope 12/12/30; MAIN nominal envelope 5/5/8; body-rate/thrust limits remain separately owned. | ACTIVE / PROVISIONAL |
| HG-005 | Planning radius sum: 0.35 + 0.25 + 0.05 + 0.10 + 0.05 = 0.80 m. | ACTIVE / PROVISIONAL |
| HG-006 | Typed observation freshness maximum age 0.5 s; exact timestamp pairing at ingress. | ACTIVE / PROVISIONAL |
| HG-007 | Retained suffix uses the planner tracking budget and 0.75 m PX4 execution-anchor limit. | ACTIVE / PROVISIONAL |
| HG-008 | Planner watchdog: 1.0 s. | ACTIVE / PROVISIONAL |
| HG-009 | Shared 3-D completion/connectivity tolerance: 0.20 m. | ACTIVE / PROVISIONAL |
| HG-010 | Retained-suffix sweep: spatial step 0.5 inflated-map resolution; time step 2–50 ms. | ACTIVE / PROVISIONAL |
| HG-011 | Hardware Mid-360 visibility is blocked until immutable certification and verification. | ACTIVE / CERTIFIED BLOCK |
| HG-012 | CIRI overlap/seed tolerances and seed clearance remain named safety inputs; no retuning by cleanup. | ACTIVE / PROVISIONAL |
| HG-013 | BACKUP uses the same independent continuous corridor-plane certificate as EXP. | ACTIVE / PROVISIONAL |
| HG-014 | Typed estimator health requires tracking plus navigation/covariance/observability/correction/propagation validity and advancing source time. | ACTIVE / PROVISIONAL |
| HG-015 | Each guide-owned SFC retains its collision-checked vertical min/max envelope and inflated-map voxel. | ACTIVE / PROVISIONAL |
| HG-016 | Boundary overspeed can only reduce inherited speed and must recover on the physics-derived bounded suffix. | ACTIVE / PROVISIONAL |
| HG-017 | Pass-through suffix transfer requires exact goal identity, measured transition, lease, world, and finite end. | ACTIVE / PROVISIONAL |
| HG-018 | Corner route window is stopping/replan/receding distance bounded by certified outgoing route and horizon. | ACTIVE / PROVISIONAL |
| HG-019 | Polyline-aware route regression checks incoming and outgoing arcs at the optimizer-pinned junction. | ACTIVE / PROVISIONAL |
| HG-020 | Cruise pass-through window uses the mission maximum velocity and remains bounded by route/horizon. | ACTIVE / PROVISIONAL |
| HG-021 | Tracking divergence uses bounded hot-stitch/restart rules; no reverse connector is admitted. | ACTIVE / PROVISIONAL |
| HG-022 | Pass-through fillet switches progress at the closest valid junction inside the acceptance ball. | ACTIVE / PROVISIONAL |
| HG-023 | A valid certified command is retained across failed replacement solves while its certificates remain valid. | ACTIVE / PROVISIONAL |
| HG-024 | Every pass-through boundary corridor has an optimizable in-ball junction. | ACTIVE / PROVISIONAL |
| HG-025 | Pass-through boundary corridor is conservatively intersected with the acceptance-region cube. | ACTIVE / PROVISIONAL |
| HG-027 | Deterministic-seed PVAJ equality uses a coefficient-derived representation bound, not a mixed fixed tolerance. | ACTIVE / PROVISIONAL |
| HG-028 | Terminal suppression/hold is STOP-only; PASS_THROUGH restarts from measured PVA. | ACTIVE / PROVISIONAL |
| HG-029 | A* validates one active inflated search layer and only the distinct evidence/probability layer when required. | ACTIVE / PROVISIONAL |
| HG-030 | Remote goals may use only a valid immutable forward route backbone and bounded executable prefix. | ACTIVE / PROVISIONAL |
| HG-031 | A measured-state emergency brake is one-shot per recovery episode; no planner-tick re-arm. | ACTIVE / PROVISIONAL |
| HG-032 | Feasibility-only dynamic penalty activation may repair a violated hard gradient without changing its hard limit. | ACTIVE / PROVISIONAL |
| HG-033 | A* prunes CLOSED/reverse edges and non-traversable endpoints before continuous edge queries. | ACTIVE / PROVISIONAL |
| HG-034 | Local visibility horizon has a 14 m floor and existing 23 m cap; speed-derived expansion remains bounded. | ACTIVE / PROVISIONAL |
| HG-035 | A moving MAIN command requires a certified positive BACKUP suffix. | ACTIVE / PROVISIONAL |
| HG-036 | Repeated guide timestamps are distributed between finite neighboring anchors; invalid timing fails closed. | ACTIVE / PROVISIONAL |

HG-003 (vertical guide envelope) is `REMOVED`; HG-026 (curved MAIN backup
reachability experiment) is `REJECTED/REVERTED`. They remain searchable in the
index/archive and are not active rows here.

## Active temporary bypasses

| ID | Current scope and impact | Removal condition / verification |
|---|---|---|
| TB-003 | Current CIRI setting remains `1`; the two-pass reference is unpromoted. Owner: mapping/planning; a second pass may enlarge the corridor but can change candidate availability and compute tails. Budget/inflation/collision gates unchanged. | Keep assurance debt open until matched dense snapshot/dataset/SITL `1` vs `2` establishes complete-feasible rate and tails. The local two-pass trial failed `PlannerFacade.CruiseFutureAnchorDoesNotReturnToUnacceptedPassBoundary`; do not promote it from this isolated result. |

TB-001, TB-002, TB-004, and TB-005 are removed historical compatibility or
experiment switches. Their evidence and removal conditions are preserved; they
must not be resurrected implicitly.

## Active qualification and safety debt

- MAIN is configured at 5/5/8 (`src/runtime/navigation_runtime/config/planner.yaml`);
  physical/BACKUP limits remain 12/12/30 (`planning_limits.hpp`). This is a
  layer split, not a relaxation of the physical gate.
- The 5 m/s multi-waypoint and completion/recovery matrices remain diagnostic or
  blocked where their reports say so. They do not qualify smooth sustained
  flight, hardware tracking, or full mission acceptance.
- `TB-003` remains open. Do not turn a successful A/B or one SITL run into a
  gate closure.
- Hardware qualification remains blocked by HG-011.
- Full provenance must bind scenario, route, speed, map/profile, fusion,
  source/build identity, and artifact timestamps before a result is evaluable.

## Current experimental and diagnostic modes

- `tracking_experiment.mode=off` disables the SITL experiment regardless of
  simulated time. `relaxed` explicitly retains diagnostic tracking/health
  suppression (`qualification_eligible=false`). Core and adapter effective
  startup witnesses must match the runner request before mission start; see
  [Phase A decision, lineage and verification](runtime_config_truth_20260924.md).
- `backup_allow_unknown`, nominal snapshot capture, terminal-state probes, and
  offline replay are diagnostic/evidence paths. They cannot grant candidate,
  command, world, or PX4 authority; offline world results are non-authoritative
  unless the production role/body-support/commit certificate path is reproduced.

## Recent effective changes

These labeled summaries link to full records; they do not replace the archive.

| ID | Lifecycle | Implementation | Evidence | Authority | Effective summary |
|---|---|---|---|---|---|
| DEC-20260917-007 | ACTIVE | IMPLEMENTED | COMPONENT_VERIFIED | PRODUCT | Couple overlap junction position and time to the continuous guide; retain existing certificates. |
| DEC-20260917-006 | ACTIVE | IMPLEMENTED | INTEGRATION_PARTIAL | DIAGNOSTIC_ONLY | Reassess completion after the normalized-guide matrix; do not claim qualification from the matrix. |
| DEC-20260917-005 | ACTIVE | IMPLEMENTED | COMPONENT_VERIFIED | PRODUCT | Normalize existing guide geometry/time/window boundaries without changing safety gates. |
| DEC-20260917-004 | ACTIVE | IMPLEMENTED | INTEGRATION_PARTIAL | DIAGNOSTIC_ONLY | Reject nominal-only early return after completion-focused SITL; full bundle readiness remains required. |
| DEC-20260917-003 | ACTIVE | IMPLEMENTED | INTEGRATION_PARTIAL | PRODUCT | Separate mandatory nominal readiness from optional objective shaping. |
| DEC-20260917-002 | ACTIVE | IMPLEMENTED | COMPONENT_VERIFIED | DIAGNOSTIC_ONLY | Observe the exact propagated state used at PX4 update for evidence attribution. |
| DEC-20260917-001 | ACTIVE | IMPLEMENTED | COMPONENT_VERIFIED | PRODUCT | Keep terminal STOP acceptance independent of pass-through projection. |
| DEC-20260916-010 | ACTIVE | IMPLEMENTED | COMPONENT_VERIFIED | DIAGNOSTIC_ONLY | Preserve the first fully certified accepted MINCO iterate for replay/evidence. |
| DEC-20260916-009 | ACTIVE | IMPLEMENTED | COMPONENT_VERIFIED | DIAGNOSTIC_ONLY | Resolve command-bundle ownership from export evidence; do not infer it from later telemetry. |
| DEC-20260916-001 | ACTIVE | IMPLEMENTED | INTEGRATION_PARTIAL | PRODUCT | Bound frontier terminal speed by the final guide turn; current SITL evidence remains conditional. |

## Historical lookup

- [Decision index](runtime_safety_index.md): deterministic IDs, source order,
  conservative lifecycle/authority, summaries and links for all706 dated decisions.
- [Lossless archive](archive/runtime_safety_legacy_full.md): byte-identical
  22,542-line,1,495,240-byte ledger, including evidence, rejected alternatives,
  bypass/gate registers and verification. Linked reports own their evidence;
  current summaries do not duplicate historical matrices.

## Migration findings / unresolved inconsistencies

- The legacy schema declared five statuses, but the 706 entries use multiple
  lifecycle, implementation, evidence, diagnostic, and qualification markers.
  The index keeps those dimensions separate and uses `UNRESOLVED` when proof is
  absent; it does not rewrite historical wording.
- The legacy gate table records `HG-004` as 12/12/30, while current source and
  recent evidence distinguish MAIN 5/5/8 from physical/BACKUP 12/12/30. This
  migration records the split in the current contract and preserves the old row
  unchanged in the archive; no runtime value was changed here.
- The ledger is not globally chronological: 706 decisions span 2026-08 through
  2026-09 and contain mixed heading levels and late validation checkpoints. The
  index preserves source order and date rather than inferring authority from
  recency.
- Most legacy decisions do not prove explicit supersession. No supersession or
  qualification was invented during migration; use `UNRESOLVED` and inspect
  the full record when lineage matters.

## Worktree experiment: retain a nominal incumbent without early return

- Owner/status: nominal optimizer and complete-bundle planner; EXPERIMENT,
  not qualified or approved for hardware deployment. This is a behavior
  trial, not an authority-preserving refactor or the withdrawn ready-first
  return policy.
- Scope: retain immutable continuously certified accepted MINCO iterate before the existing refinement cutoff,
  continue optimization, prefer valid final. At cutoff/final rejection select/revalidate only unrevoked incumbent.
  Validator exception/cancellation/hard expiry fail closed; solver-local, not execution authority.
  All yaw/BACKUP/latest-world/admission/activation gates remain.
- Safety: no80ms/400ms, weights, dynamic/corridor/route/freshness/tracking/SAFE-FAST policy change.
  Certificate work consumes complete-bundle time; nominal incumbent may lack viable SAFE BACKUP.
- Evidence/removal: PRE replay332/333 exposes lost early nominal certificate, not completion fix.
  Provisional until regression/full-bundle/SAFE-FAST2/5/9WP evidence; withdraw on completeness/progression/tail regression.
  Keep all failures and qualification blocks. Verify incumbent/final-preference/revocation regressions,
  frozen40/80ms replay, real-facade SAFE/FAST, `make test`, Release and sequential5m/s matrix.
  [Targeted history/outcomes](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#retain-and-continue-incumbent-trial).

## Worktree recovery trial: indeterminate pre-START terminal tracking

- Owner/status: existing runtime execution transaction and planner brake
  validator; IMPLEMENTED in the worktree, EXPERIMENT / COMPONENT_VERIFIED,
  native completion milestone FAILED, not qualified or promoted.
- Scope: PVA-only terminal MAIN monitor, with exact current owner/episode,
  finite dual-clock-fresh same-epoch/frame state, SOURCE before the original
  MAIN START, valid current MAIN sample, raw pressure above the existing
  tracking budget and no greater than the existing execution-anchor cap,
  current body KNOWN_FREE and valid latest-world sweep, before END/lease expiry.
  This may attempt an independently certified measured-state emergency brake;
  it cannot preserve an unsupported MAIN or fabricate actual/projected flags.
- Profile distinction: strict and relaxed PVA may attempt this recovery.
  `suppress_braking` still suppresses finite supported tracking responses;
  indeterminate support is outside that bypass. Velocity-only permission is
  unchanged. Emergency native BACKUP roles retain SAFE known-free / explicit
  FAST UNKNOWN; both forbid OCCUPIED and OUT_OF_MAP.
- Safety impact: a new recovery trigger, not an existing `OTHER_INVALID`
  permission. Dynamics/flatness/yaw/world certificates and factual conditional
  admission/delivery remain mandatory. Failed certification fails closed;
  superseded/post-END results discard without revoking a newer owner. HG-031
  prevents emergency re-arm; measured stop resumes the same waypoint, never
  directly completes mission. Polynomial certification is not a measured
  closed-loop stopping proof.
- Authority cutover: terminal-monitor conditional store admission and Episode /
  executing-goal delivery share the existing owner critical section. Backend
  ACK remains outside and cannot replay delivery or revoke a same-generation
  world copy. This closes the controlled store-H / Episode-G gap only on this
  path. The shared immediate-cutover correction below also covers the generic path.
- Evidence/removal condition: both native SAFE5 bag arms expose this seam;
  three actual-factory controls are RED before implementation and subsequently
  GREEN with adversarial/full Release regression. The frozen18-run matrix
  completes4/18,5WP0/6; it does not justify promotion or a performance tag.
  A separately confirmed exact-START arithmetic defect is corrected before
  the next discriminator. Clock-corrected v3 subsequently completes1/18,
  SAFE5WP0/3 and FAST5WP1/3; system completion remains unachieved. These are
  unpaired snapshots, not proof of a recovery/clock causal regression.
  Keep provisional; withdraw if attributed stopping,
  progression or timing-tail evidence shows regression. Missing tail evidence
  is NOT_EVALUABLE, never a PASS.
- Verification/history: `test_navigation_runtime_terminal_monitor`,
  `test_planner_fsm`, `test_planner_facade`, canonical Release/`make test`,
  serial SAFE/FAST x 2/5/9WP x 3; `python3 tools/validate_runtime_safety_ledger.py`
  and `git diff --check`. [Current evidence and decision](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#indeterminate-pre-start-recovery-trial);
  lineage: DEC-20260909-008 relaxed scope, DEC-20260901-007 terminal projection,
  DEC-20260901-009 measured-stop resume in the decision index/archive.

## Worktree boundary correction: canonical signed MAIN elapsed time

- Owner/status: existing runtime retained-validation transaction; IMPLEMENTED,
  COMPONENT_VERIFIED with v3 Full Release/regression complete; native
  completion milestone FAILED (SAFE5WP0/3, FAST5WP1/3). No
  new bypass, temporary gate, recovery trigger or command authority is added.
- Scope: immutable state is loaded before a shared evaluation timestamp;
  elapsed is checked signed nanosecond evaluation-minus-declared START, then
  converted as a duration. Exact START is zero; genuine pre-START remains
  negative, overflowing/malformed deltas remain unusable, END stays exclusive.
  Independent dual-clock freshness, actual owner/world/lease/END checks and
  source-aligned tracking support remain mandatory.
- Safety impact: remove a false-negative caused by differently rounded
  absolute seconds. No epsilon, zero fallback, grace, anchor retiming, threshold,
  budget or SAFE/FAST UNKNOWN change. It does not invent a G SOURCE sample or
  certify collision-free physical stopping. Backward-clock monitor entry is
  reject-only, not a complete distributed reset/receiver proof.
- Evidence/review condition: frozen SAFE5r2 G13 START/evaluation56092000000ns
  yields legacy elapsed -7.1e-15s while raw .1483748m < .25m and all remaining
  retention checks pass. Real factory controls are3 RED before correction;
  actual Luna v2 monitor37/37 and FSM73/73 GREEN. The all-offset assertion is
  strengthened after v2; final v3 Full Release/regression and monitor37/37,
  FSM73/73, facade40/40 pass. Serial18 closes1 COMPLETE,14 PAUSED and3 component
  failures, all cleanup/provenance valid; no performance tag or qualification.
  Exact per-failure attribution remains a separate census, not implied by the
  clock component PASS. Revisit if declared-time/source semantics change or independent
  identity/reset/END controls fail; no component-only performance claim.
- Verification/history: `test_navigation_runtime_terminal_monitor`,
  `test_planner_fsm`, canonical Release/full regression/`make test`, separately
  frozen serial SAFE/FAST x 2/5/9WP x 3; ledger validation and `git diff --check`.
  DEC-20260909-007 preserves source-aligned tracking semantics. Exact native,
  RED/GREEN provenance and limitations are in the
  [canonical closure](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#exact-start-v3-matrix-closure-and-system-reassessment--2026-09-18).
- Subsequent diagnostic discriminator (no product behavior change): v3 FAST9
  first-loss samples have canonical elapsed0, not recurrence of the repaired
  negative elapsed. G27 raw .9896m is beyond the existing outer cap; another
  G27 raw .3654m attempts H preparation but does not obtain a candidate.
  FAST9r2 actually retries PlanFromRest, not scheduler suppression. Root's
  source-age-only draft controls fail their pressure precondition (.0344m,
  below.25m) and are withdrawn with RED logs retained, not called product
  failures or repaired by retiming/tuning. Existing Full Release/component
  claims above remain bound to their frozen v3 source, not these draft controls.
  Original H-leaf and closed-loop capture attribution remain open; details:
  [terminal/observer discrimination](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#terminal-first-loss-and-observer-cost-discrimination--2026-09-18).

## Worktree boundary correction: canonical planner ACK START ordering

- Owner/status: existing backend `CmdTraj` ACK/history boundary; IMPLEMENTED,
  COMPONENT_VERIFIED, Full Release/regression and frozen18-run native comparison
  CLOSED; completion/performance acceptance remains NOT_MET.
  The execution timeline remains the sole command authority; no bypass or
  recovery permission is added by this correction.
- Scope: precheck and commit use the same checked nanosecond START conversion
  as executable export. Metadata must match the positional origin actually
  stored. Same canonical START may differ in absolute-double rounding; a
  genuinely older origin, malformed/negative/overflowing START and regressed
  generation remain reject-only, before history mutation. Legacy unit API0
  remains representable; yaw structural tolerance and original trajectory
  origins are unchanged. Lost double precision at large epochs is not restored.
- Safety impact: remove a false-negative at exact activation without epsilon,
  clamp, rebase, grace, anchor retiming, budget/gate or SAFE/FAST UNKNOWN change.
  Generation, epoch/request/world/role/dynamics/admission gates still apply.
- Evidence/review condition: actual-factory G at83716000000ns with controlled
  pressure has exact-START H preparation RED and matched+20ms GREEN. Four
  direct guard controls are RED before correction. Actual Luna after correct
  backend install/relink passes trajectory152/152 and monitor39/39; final relaxed/
  observer-OFF controls pass monitor41/41. Full Release23 packages, full test14
  packages,87 CTest targets and final FSM73/facade40 pass. Frozen native18/18 is
  5 COMPLETE/12 PAUSED/1 FAILED_COMPONENT, SAFE5WP0/3 and FAST5WP2/3, all terminal
  cleanup/provenance valid,0 report PASS and no qualification. Native buffered
  cancellation reason is not unique, so original FAST9 H-leaf causality and
  completion/performance improvement remain unproven. No performance tag or
  experiment promotion. Revisit if declared START or history semantics change.
- Verification/history: trajectory Ack/ordering/generation tests, actual
  runtime monitor H preparation/admission/ACK/Episode receipts, canonical Full
  Release/`make test`, frozen SAFE/FAST x2/5/9WP x3, ledger validation and
  `git diff --check`. DEC-20260825-021, DEC-20260829-061,
  DEC-20260831-021 and DEC-20260916-005 are the targeted archived lineage.
  [RED/GREEN and provenance](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#planner-ack-start-correction--2026-09-18).
  [Native closure and evidence limits](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#backend-start-guard-v1-closure-and-fix-versus-performance-review--2026-09-18).

## Worktree boundary trial: BACKUP construction preconditions

- Owner/status: existing planner BACKUP construction; EXPERIMENT, focused
  two sequential SAFE facade REDs followed by focused2/full facade42 GREEN;
  strengthened same-candidate policy/blocked-world recertification also GREEN;
  Full Release/regression and frozen18-run native verification CLOSED;
  completion/performance acceptance NOT_MET; not promoted or qualified.
  No new execution authority or parallel implementation is added.
- Scope: remove the unused robot-radius visibility-seed retreat and its
  minimum-chord rejection. The checked seed was overwritten before the switch
  search; the visibility SFC no longer constructs the actual braking corridor.
  Keep the existing finite visibility window, required MAIN reserve, absolute
  deadline/cancellation, actual braking-SFC/full Bezier hull, continuous
  corridor, dynamics/flatness, swept-world and final authorization boundaries.
- Safety impact: admit construction attempts, not an uncertified command. SAFE
  BACKUP remains KNOWN_FREE; explicit FAST may use UNKNOWN; both still reject
  OCCUPIED and OUT_OF_MAP. Robot radius, clearance/inflation, budgets, leases,
  sampling, anchors and waypoint acceptance values are unchanged. A second
  controlled stage removes the actual-braking minimum net-chord precondition:
  the existing geometry API owns finite short/point seeds and the entire
  polynomial still requires its own corridor, hull and world certificates.
- Evidence/removal condition: exact typed-core selected MAIN at requested
  5m/s has a positive short stop with all existing offline certificates; SAFE
  facade first fails before enumeration while explicit FAST passes. After
  removing only that obsolete gate, ten seeds are feasible; seven longer
  seeds reach world checks and fail, while the final short seed is rejected
  at corridor construction before its mandatory checks. Last-known-free
  diagnostics belong to earlier longer seeds, not that untested final seed.
  The same MAIN's final short stop passes the independent offline checks.
  This is a
  controlled component RED, not attribution for the prior native failures.
  Withdraw if full certificates, active retention, cancellation or timing tails
  regress. Native matrix completes5/18 (6 component failures,7 safety pauses),
  SAFE5WP2/3 and FAST5WP1/3: the proposed milestone is not met in both policies.
  All18 remain qualification-ineligible; native completion improvement and
  physical stopping/capture/cruise/tracking quality remain unproven. Six receiver
  stale witnesses are not a MINCO/CIRI solver failure rate. Do not restore the
  removed automatic mode re-entry as a completion workaround (DEC-20260909-004).
- Verification/history: focused ShortFrontier facade tests, full Release and
  regression/adversarial controls, then frozen SAFE/FAST x2/5/9WP x3 with every
  failure retained; ledger validator and `git diff --check`. Targeted lineage:
  DEC-20260828-005 actual braking hull; DEC-20260909-002 valid point seed;
  DEC-20260902-041 complete baseline before optional refinement. Artifacts:
  `.artifacts/diagnostics/safe-short-frontier-admission-red-v1-*` and
  `.artifacts/diagnostics/subvoxel-backup-corridor-probe-v1-*`.
  [Native closure, provenance and failure discrimination](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#short-backup-native-closure-and-planner-failure-discrimination--2026-09-18).

## Worktree correction: stop authorization requires a concrete polynomial

- Owner/scope: planner BACKUP/EMERGENCY builder and speed governor; source bugfix, component evidence only, not flight-qualified.
- Contract/behavior: the shared PVAJ scalar is a search estimate, not stop authorization. Require a concrete minimum-snap stop, bounded extrema and polynomial support; corridor/world remain independent. Measured PVAJ and physical limits govern; nominal cruise cannot erase derivatives. Steady-cruise proposals use the existing polynomial's closed-form duration/support cap, then the concrete extrema validator. If conversion to the representable duration rounds below the exact A/J boundary, allow at most 32 upward representable corrections; the existing numerical-extrema tolerance does not relax the analytic physical A/J comparison. At a support boundary, allow at most eight downward representable candidate-speed corrections until the concrete stop support fits; this is not a widened feasibility or speed policy. Nonzero measured PVAJ retains bounded synthesis. Scalar support never replaces world/corridor checks.
- Evidence/removal: native `PlannerBackupBraking`/`PlannerSpeedGovernor` counterexamples and positive-state tests, including a feasible speed below the old 1/16 grid floor and bounded-abort checks. Keep full-state stop, support and independent world authorization; extrema work is uninterruptible and no WCET is claimed. Integration/recorded-data evidence remains open; physical/nominal limits, tolerances, UNKNOWN policy and deadlines are unchanged. Lineage: DEC-20260903-001.

## Diagnostic only: complete-baseline refinement opportunity

- Owner/scope: planner-core diagnostic fixtures only, not execution authority. Finite all-known-free SAFE/FAST requests retain 80ms/400ms and every validator;
  no production scheduling, policy, budget, bypass or observer change.
- Evidence: actual Luna typed-core v3 has 12/12 complete initial baselines; early SAFE3/3 and FAST3/3 successors complete with matching STOP/rest and shorter planned
  remaining bundle time. Six late requests fail before MINCO, not in the optimizer or native mission; attribution was root-reviewed.
- Safety/removal: diagnostic only, not deployment/qualification evidence. Retain failures/pins; withdraw if native completion, quality or CPU/latency tails regress. Product change
  needs controlled RED/GREEN, full Release/regression and frozen SAFE/FAST matrix. [Scope, results and retention](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#complete-baseline-refinement-opportunity--2026-09-18).

## Experimental scheduling: one initial-baseline refinement after actual ACK

- Owner/scope: existing serial navigation runtime planning worker. Unpromoted
  working-tree experiment, not native acceptance. Only the first admitted
  initial-stopped certified-seed MAIN+BACKUP for a localization/goal/request
  tuple can open one otherwise-deferred quality attempt, after canonical active,
  Episode and actual backend history generations agree. A queued ACK watermark
  alone is insufficient. Route/dynamics/world-generation must match; same-G
  recertification, seed successors and stopped recovery cannot rearm it.
- Safety: preserve post-admission quiet tick and ordinary renewal/urgent rules;
  never label quality a forced safety transition. Require healthy, exposed,
  current-world MAIN and request/current-world revision agreement, no
  pending/hot/restart/terminal hold. Consume only at a
  valid future-anchor typed-request backend entry; failed/cancelled attempts
  do not refund. Existing admission, retained validation, expiry, fail-closed,
  SAFE known-free and FAST explicit-UNKNOWN policies remain authoritative.
  No budget,400ms anchor, certificate, gate, publisher or observer change.
- Evidence/status: matched legacy-gate RED v4 reaches actual initial runtime
  admission/queued ACK and a valid future anchor, but never starts the early
  successor. GREEN v2 passes four scheduling and two real-runtime synthetic
  tests, including full successor staging/HEAD parity and post-solve failure
  retaining validated MAIN without quality retry. Earlier compile, body-prefix
  and insufficient future-lease fixture blockers are not product findings.
  Full Release/regression and frozen18-run native execution are CLOSED;
  completion/performance acceptance is NOT_MET (SAFE5WP2/3, FAST5WP1/3).
  The18 cases close8 COMPLETE/9 PAUSED/1 component failure, all cleanup/provenance
  valid and qualification-ineligible. Only two cases have a reason6 quality
  opportunity; this is not an18-case successful-refinement ablation. Tracking
  diagnostics exist, but reference-lineage/coverage eligibility and CPU tails
  remain unproven. No performance tag or promotion is earned.
- Removal/verification: withdraw if full-candidate readiness, completion,
  tracking/settling or resource tails regress; no performance tag or promotion
  from component evidence. Run focused BaselineRefinement FSM and real-runtime
  tests, full Release/regression/adversarial controls, then frozen serial
  SAFE/FAST x2/5/9WP x3; ledger validator and `git diff --check`.
  Targeted lineage: DEC-20260901-053/054/055, DEC-20260902-041,
  DEC-20260906-001 and DEC-20260909-021. Existing diagnostic failure injection
  is used only in the test control and stays default OFF.
  [Canonical lifecycle evidence](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#bounded-initial-baseline-refinement-lifecycle--2026-09-18).
  [Full gates, native closure and limitations](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#initial-baseline-refinement-v1-native-closure--2026-09-18).

## Worktree boundary correction: captured planning request coherence

- Owner/status: existing serial runtime request builder; IMPLEMENTED,
  COMPONENT_VERIFIED, full Release/regression/native18 CLOSED, completion NOT_MET;
  unqualified; no new owner/trigger/bypass.
- Scope: refresh only SOURCE/revision to match captured immutable inputs; never
  rebase goal/epoch/request/route/dynamics/active-G/world-generation. Reject changed
  owner/frame/pending/stale inputs before backend; anchor PVAJ/activation/world factual.
- Safety:80ms/400ms, validators/leases/exposure limits, SAFE known-free/FAST explicit UNKNOWN unchanged; worker provenance, owned-key/latest-world admission retained.
- Evidence:3 actual-runtime REDs + positive PASS -> initial6/6, monitor48/48,
  existing FSM77/77, worker12/12 GREEN;7 tampered-key negatives do not solve,
  mutate active/pending or latch failure. Moving4/4, monitor52/52, Release23 packages,
 85 fresh CTests, Python389 PASS/1 SKIP. Native18:5 COMPLETE/7 PAUSED/6 component,
  SAFE5WP1/3, FAST5WP2/3; cleanup/provenance valid,0 report PASS. FAST5r2 conditional;
  no performance/qualification claim; retention and pins in the report.
- Review/verification: revisit request/history/promotion/producer semantics; moving/initial controls, full Release/regression/adversarial, frozen SAFE/FAST
  x2/5/9WP x3, ledger validator/diff check. DEC-20260902-017 lineage.
  [Canonical RED/GREEN and provenance](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#captured-planning-request-coherence--2026-09-18).

## Worktree: shared immediate cutover and CIRI reference restoration

- Owner/scope: IMPLEMENTED one immediate store/Episode/executing-goal delivery under loc->input->command;
  prepared goal/validation outside, exact predecessor/final owner-freshness checks inside. ACK never replays; future stage unchanged.
- Evidence: factory2 RED/1 positive ->3 focused/54 monitor GREEN (CIRI1); final CIRI2 Release23 packages/85 fresh CTests PASS. Native18 closes5 COMPLETE/9 PAUSED/4 component; SAFE5WP1/3, FAST5WP2/3,9WP0/3 each. No performance/qualification claim.
- Planning: CIRI current1 (TB-003 two-pass reference not promoted), loader/config agree;80ms/400ms/dynamics/certificates/leases unchanged; SAFE known-free/FAST explicit UNKNOWN, both forbid OCCUPIED/OOM.
- Removal/review: no new bypass; revert trial if complete readiness, progression/clearance or tails regress.
  Verify immediate/CIRI/supersession, Release/`make test`, SAFE/FAST2/5/9WPx3, dense dataset1/2 A/B; [history/receipts](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#shared-immediate-cutover-and-ciri-reference-restoration--2026-09-18).

## Worktree correction: nonuniform corridor junction velocity

- Owner/scope: existing deterministic seed builder; IMPLEMENTED, COMPONENT_VERIFIED.
  Opposite-duration normalized secant weights reproduce quadratic motion across unequal T.
  A/J construction, interior speed cap, hull damping and immutable endpoint PVAJ remain.
- Safety: no retiming, new bypass/owner, budget, gate, dynamics or SAFE/FAST change;
  full continuous nominal/world/BACKUP/admission certificates remain mandatory.
- Evidence: actual production-header analytic fixture RED; certificate stage5 is
  DeterministicNominalSeedFailureStage::kDynamics, not builder coefficient failure.
  GREEN/14 seed tests/optimizer31/Release23/85 fresh CTests/Python389 PASS (one GUI skip); failed oracle/namespace runs retained. Native readiness pending; no completion/performance claim.
- Verify unequal/reversed/multiple-junction oracle, cap/corner negatives, backend/full
  regression and Release. Lineage DEC-20260828-089; [current action/evidence](../reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md#nonuniform-corridor-junction-correction--2026-09-18).<br>**2026-09-23 Core mission handoff repair:** owner Core `NavigationRuntimeNode` for mission/publication and PX4 adapter for local admission; scope first mission-authority cut; safety impact: retain only the exact certified executing predecessor across desired PASS gate advance, use 200 ms ModeStatus freshness only to establish activation, continue exact finite adapter admission and unchanged 100 ms command/200 ms adapter/500 ms runtime leases, world/certificate/localization checks and Hold; evidence: pre-fix H4/H5 plus `publishCommand()` stack, component tests and focused diagnostic SITL in `artifacts/repair_core_mission_liveness/`; removal condition: revert if exact execution identity, adapter-local admission, or finite predecessor lease cannot be proven; verify Release build, focused package/Python tests, mission-authority guard, SITL parity/lease fence, `python3 tools/validate_runtime_safety_ledger.py` and `git diff --check`. [Targeted history and limitations](runtime_safety_index.md#2026-09-23-core-mission-handoff-repair).<br>**2026-09-24 diagnostic state transport timing trace:** owner FastLIO publisher, PX4 adapter ingress and SITL evidence monitor; scope default-OFF, explicit simulated-time SITL/test sideband sequence/source/steady/callback/lock/accepted-receive witnesses, separate >50 ms `/clock` arrival diagnostics and read-only rosbag clock-gap recovery. Safety impact: no control or health decision consumes the best-effort trace; monitor 500 ms stale assessment, 200 ms state boundary, command lease and Hold unchanged; missing trace is missing evidence. Evidence: pinned A2, natural pilot 3 and ten-run cohort in `artifacts/qualification_gate_recovery/20260924T082427Z-4d184896/`; remove when causal attribution closes or enabled-trace load perturbs control. Verify trace scope guard, Release/component/SITL, ledger validator and `git diff --check`. [Lineage](runtime_safety_index.md#2026-09-24-diagnostic-state-transport-timing-trace).
