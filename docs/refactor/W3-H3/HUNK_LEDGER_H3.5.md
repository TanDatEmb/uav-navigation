# H3.5 hunk ledger

- `docs/safety/runtime_safety_current.md`: update only the H3.5 ledger row.
- `execution_authority.hpp`: determine whether the active bundle is actually
  retainable before applying the equal-epoch admission rejection.

Safety scope: preserve the failed admission latch against equal-epoch replay;
no new authority, lease, threshold, UNKNOWN policy, or PX4 path is introduced.
Verification: focused execution-authority tests, runtime tests, full Release
build/CTest, `python3 tools/validate_runtime_safety_ledger.py`, and
`git diff --check`.
