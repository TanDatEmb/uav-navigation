# Phase A — parallel branch convergence

## Result

**Blocked.** All canonical milestone heads are ancestors of the pinned candidate, and all seven divergent audit branches contain only artifacts/ and/or docs additions. However, origin/codex/close-findings-implementation has two source-level behavior properties absent from the canonical candidate:

1. bounded ULP correction with strict acceleration/jerk checks for steady-stop synthesis (12c8352b);
2. immutable request-owned predecessor bundle used in candidate authorization (bcff7c96).

Therefore MISSING_FIX = 0 is not satisfied. Phase B World tests and Phase C readiness work were not started.

## Remote census

- Remote heads after fetch: 27, symbolic origin/HEAD excluded.
- Canonical ancestor heads: 17, including the 13 named milestone heads, main, both close-findings predecessor branches, and the current World candidate.
- Audit archival-only: 7. Their unique path inventory consists only of artifacts/ and docs paths.
- Experiment-only: 2.
- Diverged product implementation: 1, origin/codex/close-findings-implementation.
- Detailed heads, merge bases, ahead/behind counts and ancestry are in REMOTE_BRANCH_INVENTORY.csv.

All 13 specifically required architecture/evidence milestone heads were individually verified as ancestors. main is also an ancestor: candidate is 83 commits ahead and main has 0 unique commits relative to the candidate.

## Audit and experiment branches

Seven audit branches are archival-only by path inspection; no product source/config changes were found.

The two experiment branches are not merge candidates:
- experiment-runtime-observability-instrumentation adds a diagnostic AuditEvent channel, sinks, normalizers, record/replay scripts and fault tooling. Current C0-SW infrastructure already carries producer-owned witness identities and loss accounting for its registered streams. Exact world/PX4 event tracing remains potentially useful tooling for Phase B, but is not a missing flight-authority safety capability.
- shadow-reducer-diagnostic layers a diagnostic-only reducer/replay model over that event experiment. Current authority owners remain product source of truth; reducer state must not be promoted to authority.

No experiment branch was merged or cherry-picked.

## Per-commit close-findings decision

See CLOSE_FINDINGS_IMPLEMENTATION_DELTA.md and PARALLEL_BRANCH_CONVERGENCE.csv. The full lineage is not safe to declare converged until the two missing current behaviors are either canonically repaired with tests or disproved with equivalent source-level contracts and race evidence.
