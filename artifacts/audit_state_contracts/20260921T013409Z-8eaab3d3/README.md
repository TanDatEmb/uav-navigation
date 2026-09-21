# State contract audit — `8eaab3d33db3`

**Verdict:** conditional source findings; no real-workload bottleneck established. This is an AS-IS source/test audit, not a safety qualification.

- `TARGET_SHA`: `8eaab3d33db36e9e636ad4005ed91b8f055f4d68`; branch `codex/close-proven-findings` fetched once and pinned.
- A artifact: `f2bd3f46f9936d622377ea4761f733f988273b66`; source manifest SHA-256 `f9481b2f7312556ae91a8bff590cbc412de8d7e3e068fc98d4c85bb34c730413`; 596 files captured from a dirty tree at HEAD `9534d8dc15920c8b3e80c8fa12f28ec972b8c6a2`.
- Claim-validation artifact: `f2ed429bef80b2c7a2964b3b32d3c00e8089e55f`.
- Audit worktree: `/home/letandat/Dev/uav-navigation-audit-20260921T013409Z-8eaab3d3` on `codex/audit-state-contracts-20260921T013409Z-8eaab3d3`.
- No production source/config/test was changed; build/install outputs are in `/tmp/uav-state-contracts-20260921T013409Z-8eaab3d3/`.

## Start reading

Read [AUDIT_VERDICT.md](AUDIT_VERDICT.md), [CONTRACTS_AS_IS.md](CONTRACTS_AS_IS.md), [STATE_AUTHORITY_MATRIX.md](STATE_AUTHORITY_MATRIX.md), then [DECISIONS_REQUIRED.md](DECISIONS_REQUIRED.md). `index.html` is offline navigation. The machine-readable source-backed index is `model/architecture.json`; exact excerpts and blob hashes are in `evidence/refs.jsonl`; field IDs link into [evidence/index.md](evidence/index.md) with commit-pinned source lines.

## Reproduce / check

Build and test commands are recorded in `validation/commands.json`; selected groups and scope in `tests/selection.json`. Run `python3 tools/regenerate.py --repo . --target-sha 8eaab3d33db36e9e636ad4005ed91b8f055f4d68` and `python3 tools/verify_artifacts.py --repo . --target-sha 8eaab3d33db36e9e636ad4005ed91b8f055f4d68` from the exact audit commit in a clean worktree. These checks establish artifact/source integrity, not semantic completeness.

## Scope

Deep review covers PASS_THROUGH desired-to-active ownership/handoff, stale-world suspension/recertification versus adapter lease, stopped recovery timers, adjacent measured waypoint progress and terminal STOP semantics. The previous H0–H7 audit is summarized; it is not repeated wholesale. No live parameter dump, target workload trace, PX4 applied-mode observation, SITL, or aircraft run was available or performed.
