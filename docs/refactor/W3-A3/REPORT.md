# W3-A3 — Salvage P1A checkpoint

Checkpoint stopped at step 1 because the designated P1A patch does not apply to the requested `origin/main` base. No runtime/config/safety behavior was changed; no verdict is assigned.

## Baseline and deliverable

- Branch: `refactor/W3-A3`; worktree: `/home/letandat/Dev/uav-navigation-w3-a3`.
- Baseline: `origin/main` at `24ec0fc8718bb8606e4e9e4a7eaa84773d384854`.
- Requested source: `P1A_legacy_cd7a7d8a.patch`; it applies cleanly on the owner workspace at `432dc94630fbc76ca670138228f2f616f6840bb0` but not on this requested base.
- Deliverable in this checkpoint: blocker report and open questions only.

## Verification evidence

| Command | Result |
|---|---|
| `git apply --check <P1A_legacy_cd7a7d8a.patch>` on W3-A3 | **FAIL / STOP** at `px4_navigation_external_mode/CMakeLists.txt`, `px4_navigation_external_mode/package.xml`, `navigation_runtime_node.cpp` |
| same `git apply --check` on owner workspace `432dc94` | **PASS** |
| `git diff --check` | **PENDING final checkpoint commit** |
| `tools/gate.sh static` | **PENDING final checkpoint commit** |
| `tools/gate.sh python` | **NOT_RUN**; step 1 blocker, no Python write-set |
| `tools/gate.sh ros` | **NOT_RUN**; step 1 blocker, no ROS write-set |

## Scope and safety

- Prompt steps 1–4: **BLOCKED at step 1**; steps 2, 3, 3b and gate execution are not claimed.
- Prompt step 5 SITL: **NOT_RUN / WAITING_FOR_C1**. It must be performed only after coordinator C1 completion, with matched scene/speed and `tracking off`; this branch does not issue an A/B verdict.
- No safety gate, threshold, deadline, lease, UNKNOWN policy, or authority was modified. Existing safety invariants remain authoritative; no new ledger entry is warranted for this docs-only blocker record.
- Cleanup: **DEFERRED**. No consumed docs were removed because the salvage replay did not start; do not delete owner-tree salvage inputs or the listed legacy docs until an applicable replay completes.

## Findings

| Finding | Status | Commit / owner |
|---|---|---|
| P1A patch context is stale relative to requested `origin/main` | OPEN / BLOCKED | coordinator/architect decision required |
| A6 oracle rows and witness replay | NOT_MEASURED | resume after B-01 |
| C1-gated SITL A/B comparison | NOT_RUN | coordinator-owned sequencing |

## Commit table

Source: `git log --format='%h %s' origin/main..HEAD` immediately before push.

| SHA | Message |
|---|---|
| *(to be filled immediately before push)* | *(checkpoint commit)* |

## Handoff

**REVIEW REQUEST — coordinator / architect:** please decide B-01 and provide an applicable patch/base. Do not merge or assign a verdict from this checkpoint. Once unblocked, the agent can resume P1A replay, A6 oracle move/check, gates, cleanup, and the C1-gated SITL handoff.
