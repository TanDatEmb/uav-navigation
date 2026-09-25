# Phase A — configuration truth

## Source chain

`run_sim(tracking_experiment_mode="off")` builds a requested payload in
`tools/runtime/runner.py::_tracking_experiment_payload`. The runner writes
metadata and scenario configuration, then generates both node parameter files
using `_apply_tracking_experiment_parameters`. Launch passes those files to
NavigationRuntimeNode and PX4 External Mode with `use_sim_time:=true`. The
shared C++ loader in `tracking_experiment.hpp` forms the effective policy.
`report.py::_tracking_experiment` reconciles metadata, scenario and generated
parameter snapshots; the evaluator consumes the result.

## First causal mismatch at base

The runner's `off` payload had `enabled=false`, `suppress_braking=false`, and
`suppress_estimator_health_response=false`. Generated parameter files omitted
those three fields and contained only zero coefficients. The C++ loader set
`enabled=use_sim_time=true`, then set both suppression fields true because the
coefficients were zero. The report/evaluator correctly marked the claimed
tracking-off run `config_mismatch`/`NOT_EVALUABLE`. A completed mission under
that bypass is not evidence of true tracking-off behavior.

The lineage is the 2026-09-09 zero-disabled tracking experiment decision in
the archived safety ledger. It explicitly records increased collision risk and
diagnostic-only status. The new requested/effective contract does not erase
that decision; it makes its bypass an explicit `relaxed` selection.

## Phase A invariant

`requested mode and suppression == effective Core == effective adapter` before
the mission begins. `use_sim_time` constrains where an experiment is allowed;
it does not request one. Missing or mismatched live node witnesses are setup
failure `CONFIGURATION_MISMATCH`.

The updated C++ loader reads explicit `tracking_experiment.mode` and optional
`tracking_gate_relaxed`. Both nodes emit one structured startup witness. The
runner compares the witness against the request and records requested,
effective, and source in metadata. The report continues to reconcile generated
configuration and retains legacy artifact interpretation separately.
The witness also includes all tracking/velocity-only numeric bounds. RuntimeNode
emits its parsed planner fault-injection selectors and the actual PlannerFacade
control envelope. The runner compares these with the requested profile and
records the three source-backed configuration domains before launching the
scenario process.

## Runtime result and phase verdict

Three sequential `long_featured`, seed 0, requested `tracking=off` runs used
the same clean source HEAD `1c75f3b4`. Both live node witnesses reported
`mode=off`, `enabled=false`, `suppress_braking=false`, and
`suppress_estimator_health_response=false` in all three. The report marked
configuration status `OK`, without a mismatch or experimental bypass.

| Run | Mission outcome | Accepted | First relevant failure | Evaluator |
| --- | --- | --- | --- | --- |
| A1 | COMPLETE | `[0,1,2,3,4]` | none observed | FAIL / NOT_EVALUABLE |
| A2 | FAILED_COMPONENT | `[0,1,2]` | adapter odometry receive age 208.583 ms; Hold | FAIL / NOT_EVALUABLE |
| A3 | COMPLETE | `[0,1,2,3,4]` | none observed | FAIL / NOT_EVALUABLE |

A2's source age was 8.000 ms, so the logged failure was the independent
steady-clock receive-age gate, not source-time age. The adapter rejected that
command, then requested PX4 Hold. This is the intended fail-closed response
to stale received odometry. The trace does not establish why the receive
sample was late. No lease or tracking threshold was changed.

The evaluator's unrelated lifecycle attribution, motion policy, reference
lineage, and tracking coverage requirements remain NOT_EVALUABLE in all three
runs. A2/A3 additionally report `SOURCE_TIMESTAMP_DUPLICATE`. We retain the
runner's `FAIL` and do not count mission completion as qualification.

**Phase A verdict: PARTIAL.** Requested/effective configuration truth is
closed; the required three consecutive clean tracking-off missions are not.
The campaign stops before Phase B under the nominal-regression gate. See
`SITL_RESULTS.md` and `OPEN_FINDINGS.md`.
