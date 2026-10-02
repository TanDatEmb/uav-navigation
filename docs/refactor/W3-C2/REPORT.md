# W3-C2 — P4-0 characterization checkpoint

## Verdict

`BLOCKED / REVIEW REQUEST`: B3 is now merged at `main@4dc769c`, but the
runtime tree has no characterization trace implementation yet. The required
six K3 sessions were **not run** because C1 sessions without instrumentation
are explicitly not valid K3 input.

## Precondition audit

The current tree has no `NAV_CHARACTERIZATION_TRACE`, `NAV_TRACE`, decision
record sink, `replay_decisions.py`, or `check_trace_corpus.py`. Existing
`execution_trace_snapshot` and runtime diagnostics are not a substitute for
the P4-0 record: they do not serialize full `pre_state`, trigger payload,
predicate arguments/results, effects, and `post_state`.

## Safety boundary

No product Release behavior, threshold, authority, or timing gate was
changed. No uninstrumented SITL run is labeled K3. The timing impact of trace
ON remains unmeasured until the instrumented binary exists.

## Required implementation before K3

1. Add an OFF-by-default, process-local JSONL decision recorder with fixed
   ring-buffer/drop accounting and integer ROS/steady timestamps.
2. Instrument all shell/execution/policy/mission effect sites and the T1a
   tracking/brake records; map every site to `RT-*` or an explicit `RT-N*`.
3. Add K1/K2 fixture capture plus `replay_decisions.py` and
   `check_trace_corpus.py`, then prove OFF binary parity.
4. Build the ON binary in the root install and run exactly six K3 sessions on
   the selected M2 scene/speed cells; K3 is coverage/invariant evidence only,
   not timing comparison.

## Verification

`git diff --check` is the applicable check for this docs-only checkpoint. No
build or SITL was run.
