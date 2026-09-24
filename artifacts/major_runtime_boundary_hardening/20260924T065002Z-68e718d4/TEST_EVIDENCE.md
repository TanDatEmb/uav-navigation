# Phase A deterministic evidence

- Clean authoritative Release build: 23 packages passed at source HEAD
  `1c75f3b47f4b302c5442bcf1fe4f0dcb8934f622`; the installed build
  manifest matched that clean checkout before all three SITLs.
- `python3 tools/runtime/build.py --mode release test` followed by
  `python3 tools/runtime/build.py --mode release check`: 90 CTest cases,
  zero errors/failures/skips. The check command is authoritative for aggregate
  status; a test wrapper exit code alone is insufficient.
- Runtime Python suite: 397 tests passed, one skipped.
- New tracking loader tests: 6/6 passed, including explicit `off` and
  invalid mode/parameter combinations.
- Static checks passed: mission authority, execution authority, desired
  intent, failclosed ownership, command contract, exact OptimizationFailed
  injection, and runtime configuration truth.
- Safety ledger validator and `git diff --check`: passed after evidence docs.

These are build/component checks. The SITL outcome is reported separately and
does not become qualification evidence because these checks passed.
