# W4-H2 quality gates

## Verdict

**REVIEW REQUEST — tooling implementation ready for review; no product code or
SITL behavior changed.** `quality` is an explicit gate and is not included in
`all` until `WAVE3_CLOSED=1`. `tsan` remains explicit and is only added to
`all` with `GATE_TSAN=1`.

## Deliverables

- `tools/gate.sh`: `quality`, `tsan`, architecture-doc lint, and guarded `all` integration.
- `tools/quality/check_quality.py`: selected clang-tidy, timer allow-list, doc/lizard ratchets.
- `tools/quality/lizard_baseline.json`: 1,196 product functions.
- `tools/quality/create_wall_timer_allowlist.tsv`: current timer findings with removal IDs/WPs.
- `tools/refactor/check_architecture_docs.py`: package/layout parity check.
- `tools/tests/test_quality.py` and gate fixtures: pass/fail coverage for every new gate.

## Verification

- `python3 -m unittest discover -s tools/tests -p 'test_*.py'`: PASS, 40 tests.
- `tools/gate.sh quality`: PASS.
- `tools/gate.sh static`: PASS, including architecture-doc lint.
- `git diff --check`: PASS.
- Real TSan build/test: `NOT_MEASURED`; fixture pass/fail coverage is present,
  but this WP does not claim a race-free result without a package-wide run.

The current machine has no system `lizard`; the checked-in baseline was
generated with lizard 1.24.0 in an isolated temporary virtual environment.
The runtime gate intentionally fails closed if lizard is needed for a changed
C++ file and the command is unavailable.

