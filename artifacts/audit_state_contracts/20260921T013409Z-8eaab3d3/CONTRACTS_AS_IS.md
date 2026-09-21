# AS-IS contracts: PASS_THROUGH, world suspension, stopped recovery

## 1. PASS_THROUGH desired-to-active handoff

### Request ownership

Runtime accepts newer goal only after route snapshot/identity checks and serialized ownership decisions. For a same logical goal transition from a previous PASS_THROUGH waypoint, hot retarget can retain the committed bundle if command is available, no failure is latched, no safety suffix is active and store retention succeeds. `ExecutionEpisode::beginGoal` advances desired epoch/request but preserves active command epoch/request/generation. Planner can solve for successor while timeline samples predecessor; planning and execution overlap.

If active BACKUP/EMERGENCY owns the episode, `canHotRetargetAtWaypointTransition` denies rebinding. `PendingGoalHandoffOwner` accepts strictly newer same-mission goal ordered by request ID, waypoint, then route revision; newest replaces older. It is peeked for stopped recovery and consumed by exact pointer only after replacement command commits. Foreign mission handling follows a separate defer/fail-closed path.

### Commit and late result

Candidate admission checks desired identity, route revision, localization/goal/request key, predecessor generation, world identity/revision, valid interval, measured state/frame/freshness, dynamics/contract and lease. Runtime transaction updates store and active episode identity. Late result with wrong desired key cannot overwrite successor. `returned`, `staged`, `committed`, `activated`, `published`, `received`, and PX4-applied remain different lifecycle points.

### Mission crossing

Adapter validates and caches accepted command. An admitted current continuation command triggers `updateMission()` immediately; mission timer and command subscription share a mutually exclusive callback group. `updateMission()` rechecks exact mission/waypoint/request, command validity/lease, state/health source and receive freshness before measured update.

PASS_THROUGH measured acceptance requires current measured position inside acceptance ball or a segment from previous point to current within acceptance geometry, forward tangent/route order, and finite sample gap `[0, 0.25] s`. STOP acceptance requires current position and settling; it does not use crossing segment. `previous_position_` and adapter clock time update on every finite mission update, so a non-witness update can overwrite the crossing baseline. This is a local counterexample mechanism, not proof the exact input is admitted by a product candidate. Immediate update on accepted continuation is counterevidence to blanket “wait until next timer” claim.

**Enforced:** desired identity can advance independently; active command identity remains until successor activation; suffix is not rebound; pending slot is monotonic/exact-consumed; stale-key result is rejected; measured acceptance is separately guarded.

**Unknown:** frequency of near-boundary measurement; whether every deployment profile can produce the timing/corridor/reserve combination; actual missed crossing; PX4 acceptance.

## 2. World freshness suspension and command authority

At runtime sample tick, latest world snapshot is checked against configured freshness using command ROS time. If not valid, planner worker is cancelled, exact active generation is remembered if identity still matches, episode command availability becomes false, and the callback returns without publishing. Stored bundle and active identity remain; suspension itself does not issue replacement command.

Fresh mapping update can resume only exact suspended generation after immutable bundle validation against latest world, matching localization/goal identity, valid-until, no failure latch and executing goal. Conditions are checked again at commit. Successful recertification restores availability and clears suspension witness; only a later command tick may publish.

Adapter is an independent receiver. Cached command remains eligible only under receive age, header/valid-until, odometry/health and state freshness guards. Expiry invalidates cache, latches stop/handover, deactivates mission, publishes status and asks executor for PX4 Hold. Exact completed endpoint has a bounded recovery exception. For an ordinary command, remaining command-age allowance at suspension is bounded by `min(last_receive + 0.10 s, command_header + 0.10 s, valid_until) - suspend_time`, subject to the receiver ROS clock and validity predicate; if already expired it is zero. It is not reset to a full 0.10 s.

**Enforced:** exact-generation recertification/recheck, no publish from stale-world tick, adapter fail-closed expiry. Runtime recovery does not imply receiver got command or PX4 applied it.

**Unknown:** deployed world freshness profile, adapter remaining lease at suspension, map worker order/timing, integrated recoverable interval and PX4 result. No integrated map callback→adapter receiver/handover test.

## 3. Stopped recovery and terminal semantics

`plan_from_rest_first_failure_steady_ns_` begins at first failed current PlanFromRest result; it is not started by measured stop or first attempt. It uses steady clock; resets on new goal/lifecycle and eligible success. Fail closed only when state is InitialHold/StoppedRecovery, measured speed finite and `<=0.15 m/s`, and elapsed `>=5 s`. Current failure before expiry retries; stale identity result is discarded.

Adapter deadline starts in ROS clock when a completed endpoint requiring recovery is accepted and stores mission/waypoint/request/generation. It is exact 5 s validated by constructor. It can allow holding that exact completed endpoint; a READY replacement, terminal acceptance, goal/lifecycle change or expiry clears it. It does not share runtime clock or start event; no relation guarantees recovery fits both. Same numeric five seconds is not evidence of conflict.

STOP requires measured STOP acceptance, finite slow velocity, terminal hold confirmation window and then waypoint dwell. Mission completion occurs downstream. Keep separate: trajectory ended; safety stop observed; waypoint accepted; mission complete; Hold requested; Hold confirmed. Local audit does not establish firmware acceptance.

**Enforced:** stationary-gated runtime retry timeout, adapter identity/deadline lease, measured STOP settling/confirmation/dwell.

**Specification gap:** required relationship between runtime retry budget and endpoint hold deadline; clock pause/reset policy across profiles; required liveness postcondition after repeated recovery failure.

## Contract classification and source relationship

| Class | Current evidence | Status |
|---|---|---|
| `ENFORCED_BY_CODE` | Exact hot-retarget/active identity guard; pending exact consume; exact-generation world recertification; runtime stationary timeout; adapter command and STOP gates. | Source predicates and related tests are listed above. |
| `DOCUMENTED_REQUIREMENT` | Current safety contract HG-017 requires exact goal identity, measured transition, lease, world and finite endpoint for pass-through suffix transfer; HG-028 scopes terminal hold/suppression to STOP and requires PASS_THROUGH restart from measured PVA; TB-003 keeps CIRI at 1 and the two-pass reference unpromoted. | These documents provide context, not proof each runtime path satisfies every aspect; source refs `ev-safety-current` and `ev-safety-index`. |
| `TEST_ASSERTED` | Selected tests assert exact predecessor retention, per-update witness, measured crossing and stationary timeout. | Assertion tested in this build; the test does not by itself define a broader liveness contract. |
| `DECISION_REQUIRED` | Relation among continuation reserve and tracking/transport delay; liveness target across world suspension and receiver lease; cross-clock retry/endpoint budget; post-success Hold status relation. | No explicit shared requirement identified in reviewed safety entries/source. |
| `CONFLICT` | No direct code-vs-current-safety-contract conflict established in the reviewed subset. | Does not imply whole-repository consistency. |

The prior audit's H6 ACK-only subclaim remains `REFUTED`: pinned `scheduleMode` completion waits for matching `ModeCompleted`, and dependency/consumer did not change A→TARGET. This audit does not re-audit that dependency. It does preserve the current source split: a successful/deactivated Hold callback clears pending/in-flight, whereas only `VehicleStatus.AUTO_LOITER` sets `px4_hold_confirmed_`; target ordering/firmware observation remains absent.
