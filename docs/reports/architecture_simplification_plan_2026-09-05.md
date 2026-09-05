# Architecture simplification plan — 2026-09-05

## Baseline

- HEAD: `14c3f78b` on `codex/runtime-evidence-for-analysis`; worktree was
  clean at audit start. `1642823e` is an ancestor and is present in history.
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

R13 and R14 remain evidence blockers: historical characterization had a
body-frame/ENU-NED velocity basis issue and stale-source handling problem; the
E5 analyzer also had hard-coded causal scope/verdict. Their historical numbers
must be re-analyzed before controller/estimator conclusions.

## COMPONENT — execution and retry ownership

`ExecutionEpisode` is the intended lifecycle policy source, while planning
worker phase is orthogonal to physical execution. Complete migration of
legacy atomics is a later step. Keep the store/timeline transaction boundary.
Unify retry/deadline reporting only after distinguishing optimizer feasibility
retries from runtime PlanFromRest retries and the single terminal deadline.

## RUNTIME — PX4 and evidence

PX4 receiver admission, odometry/health freshness, command validity and Hold
handover remain required. A continuous velocity profile can replace discrete
approach speed profiles, but cannot replace stale/foreign command rejection,
measured stop/acceptance, asynchronous handoff, or PX4 recovery gating.

Validation order after R15 is complete: focused execution CTest/build; then
R13/R14 evidence re-analysis; then exact repeated O0-HANDOVER,
C0-NEAR-OBSTACLE and RESET scenarios after a clean provenance-matching build;
finally repeated open/pass-through/obstacle External Mode runs with generation
activation, PVA continuity, clearance, PX4 state/health, waypoint completion
and fail-closed negative-path evidence. No gate is to be relaxed from a single
run, and no scenario is accepted from focused tests alone.
