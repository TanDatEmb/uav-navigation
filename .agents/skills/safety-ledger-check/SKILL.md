---
name: safety-ledger-check
description: Use when changing safety docs, adding or removing a bypass, relaxed gate, fallback-only path, or test-specific behavior, or before touching estimation, mapping, planning, control, PX4 integration, budgets, or validation thresholds
allowed-tools: Bash Read Glob Grep Edit
---

# Safety Ledger Check

`docs/safety/runtime_safety_current.md` is the only safety record. Legacy
`DEC-*`/`HG-*`/`TB-*` labels have no retrievable history; use `git log`.

## Before changing guarded areas

Read the contract's invariants in full; this skill deliberately does not
restate them, so the contract is the only source. Cite invariants by number.

## Recording a workaround

Any temporary bypass, relaxed gate, disabled validation, fallback-only path, or
test-specific behavior must be added to the current contract and targeted
history **in the same change**, with: owner, scope, safety impact, evidence,
removal condition, verification command. Never silently turn a workaround into
product behavior.

## When the safety-document set changes

```bash
python3 tools/validate_runtime_safety_ledger.py
git diff --check
```

Both must pass; report output verbatim. Also run any `tools/check_*.py` that
AGENTS.md or the contract currently names as active (`ls tools/check_*.py`);
if a named check no longer exists, say so rather than skipping silently.

## Three-level review for any correctness/performance change

1. System ownership and end-to-end safety contract.
2. Source, units, runtime artifacts, test evidence.
3. Adversarial: local optimization, hidden bypasses, latency tails, regressions
   in another layer.
