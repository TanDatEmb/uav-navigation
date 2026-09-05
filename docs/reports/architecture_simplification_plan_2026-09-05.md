# Architecture simplification plan — 2026-09-05

## Baseline

- Baseline audit snapshot was `14c3f78b` on
  `codex/runtime-evidence-for-analysis`; R15 was checkpointed as `05ac49c8`.
  `1642823e` is an ancestor and is present in history. The original audit also
  contained unrelated runtime-evidence WIP; this plan does not treat that WIP
  as acceptance evidence.
- This document is a staged engineering plan. It makes no SITL or flight
  acceptance claim.

## CODE — R15 execution predecessor

`ExecutionTimelineStore` is the sole command pointer owner. `reserveAnchor()`
already records bundle generation, epochs, request, activation/main/end times,
role and world identity. The missing invariant was revalidation that the same
predecessor still owns the command at both pending staging and activation.

The R15 patch revalidates the current predecessor under the store mutex while
staging and rejects a same-world/same-goal newer generation. Activation checks
that both active and pending bundles remain valid under the current world
identity. Existing semantic identity-changing operations clear pending, while
world-only recertification may copy the same trajectory identity and retain a
validated pending successor; rollback restores active and pending together, so
no second pending witness field is needed. No new FSM, gate, timing value or
mutable mirror is introduced.

Regression scope: stale predecessor for `stagePending`, stale predecessor for
`stagePendingAndFinalize`, unchanged valid staging, same-generation active and
pending world recertification followed by activation, and active invalidation
with retained pending rejection. A failed pending finalizer must restore the
previous staged successor and its activation boundary.

## COMPONENT — mapping and route contracts

The current mapping implementation keeps bounded `TraversedFreeSpace`, explicit
sensor origin and OCCUPIED/inflated precedence. `CurrentBodySupport` is absent
at this HEAD. The historical simplification proposal may replace history only
after a typed current-body contract proves: current-body UNKNOWN handling,
outside-body UNKNOWN rejection, OCCUPIED contradiction, no ROG mutation,
normal BACKUP evidence, arbitrary moving entry, and complete MAIN+BACKUP
admission. Until then, mapping history remains the current authority.

PASS progress must remain measured and route ordered. Planner corner handling
already computes a speed cap from acceptance radius and acceleration, preserves
incoming tangent, and checks corridor boundary geometry. Before further route
simplification, make `RouteBoundaryEvent` and a continuation witness explicit
contracts, then derive continuous speed from curvature/corridor/STOP and
time-parameterize it under existing V/A/J certificates. Do not clamp or smooth
commands downstream of the optimizer.

R13/R14 analysis tooling is now corrected in the bounded characterization and
root-cause analyzers, but retained artifact qualification remains open. The
historical characterization and E5 numbers are unqualified until re-analysis
uses those tools with raw source-time/frame/reset provenance; they must not
support controller or estimator conclusions before that evidence exists.

## COMPONENT — execution and retry ownership

`ExecutionEpisode` is the intended lifecycle policy source, while planning
worker activity is orthogonal to physical execution. R15 is complete. The
bounded runtime cleanup removes three write-only compatibility atomics and
the physical `kPlanning` episode phase while preserving the historical numeric
ordinals of all remaining telemetry phases; worker progress remains identified
by `active_planner_solve_generation_`. Paired safety-suffix/restart decisions
use one episode snapshot. The solve-generation-exhausted and rejected-route
branches remain separately audited owner work; their bounded identity-guarded
cleanup keeps the store/timeline transaction boundary and does not add a new
latch or fail-close a possibly stale solve callback. Unify retry/deadline reporting only after
distinguishing optimizer feasibility retries from runtime PlanFromRest retries
and the single terminal deadline.

The bounded early-failure owner fix now uses the same execution timeline
watermark: route-snapshot rejection, solve-generation exhaustion, route-setter
rejection and immediate-commit activation-queue failure may revoke only the
goal/localization identity and `{timeline_version, active pointer}` captured by
that callback. Store recertification or activation wins the race and preserves
the newer command; recertified immediate commits still run planner-history
synchronization before any queue-failure revoke decision. During a valid
pending-goal promotion, the old callback's identity check deliberately
no-ops: the goal-epoch transition clears pending, and the next planner cycle
consumes a refreshed key, so no old bundle is combined with the promoted goal.
This remains a bounded transaction, with no retry threshold, FSM, latch or
global invalidation added.

The preflight artifact `.artifacts/runtime/external-mode-check-20260905T012101-305346`
is legacy occlusion evidence (`REPORT` blocked, no successful planner PVA), not
clutter evidence. `C0` is only a case label here; the separate C0 clutter
scenario is `MAP_SCENE=clutter MAP_SEED=11 TEST_CASE=positive`. `O0-moving`
is not automated by the current harness. Do not combine the 1.402 m occlusion
clearance with clutter results.

## RUNTIME — PX4 and evidence

PX4 receiver admission, odometry/health freshness, command validity and Hold
handover remain required. A continuous velocity profile can replace discrete
approach speed profiles, but cannot replace stale/foreign command rejection,
measured stop/acceptance, asynchronous handoff, or PX4 recovery gating.

Validation order after R15 and runtime cleanup: focused execution/runtime
CTest/build; then
R13/R14 evidence re-analysis; then exact repeated O0-HANDOVER,
C0-NEAR-OBSTACLE and RESET scenarios after a clean provenance-matching build;
finally repeated open/pass-through/obstacle External Mode runs with generation
activation, PVA continuity, clearance, PX4 state/health, waypoint completion
and fail-closed negative-path evidence. No gate is to be relaxed from a single
run, and no scenario is accepted from focused tests alone.

The R15/runtime cleanup integrated build passed for `navigation_execution` and
`navigation_runtime`; direct affected binaries passed execution-episode 5/5,
committed-bundle-store 32/32 and execution-state 10/10. With both ROS Jazzy
and the workspace overlay sourced in the same shell, CTest passed runtime 8/8
and execution 2/2. The earlier unsourced wrapper failure was an environment
path issue, not a test assertion. No SITL was run.
