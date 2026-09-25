# PR #2 Codex P1 repair

## Finding

P1 `Use a reachable baseline for the scope guards` identified that both branch-scope scripts hard-coded the incoming World milestone SHA. That object is not guaranteed to exist after squash merge or in a shallow checkout.

## Repair

- Both scope scripts now require an explicit `--base` argument.
- Each validates the commit object and requires it to be an ancestor of `HEAD`; missing objects produce `SCOPE_BASE_UNAVAILABLE`, while non-ancestor commits produce `SCOPE_BASE_INVALID`.
- The checks remain available for historical pre-merge evidence, but were removed from `pre_main_gate.py` because they are branch-delta checks, not permanent product invariants.
- Permanent authority, command-contract, runtime-configuration, evidence-scope, and safety-ledger checks remain in the canonical gate.
- Tests cover explicit reachable baselines, changed protected source, absent/invalid/unreachable baselines, missing arguments, and a one-commit checkout where the historic SHA is absent.

## Current review status

The fix is implemented locally; the existing P1 thread is not marked resolved. Resolution requires the repair to be pushed and verified, followed by a new review of the current PR head. No merge has been performed.
