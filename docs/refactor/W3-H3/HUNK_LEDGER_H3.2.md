# H3.2 hunk ledger

- `docs/safety/runtime_safety_current.md`: update only the H3.2 ledger row.
- `fast_lio_pipeline.cpp`: keep the recovered group epoch local until the
  prediction succeeds; do not advance committed `state_time_` on rebase alone.

Safety scope: prediction epoch transactionality only; existing retry limit,
gap policy, thresholds, command authority, lease, and PX4 behavior are
unchanged. Verification: `test_fast_lio_pipeline`, full backend CTest,
`python3 tools/validate_runtime_safety_ledger.py`, and `git diff --check`.
