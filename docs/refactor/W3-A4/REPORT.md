# W3-A4 — Báo cáo salvage J1 + J1b

## Tóm tắt

- Baseline `origin/main`: `432dc94630fbc76ca670138228f2f616f6840bb0`.
- Replay phần runtime của salvage patch sạch; loại artifact `docs/refactor/WP-J1/**` theo write-set người dùng.
- J1b.1 áp cùng `evaluation_window` cho position/velocity statistics và coverage.
- J1b.3 gắn tracking/cross-track là diagnostic, `frame_status=diagnostic_frame_unverified`, và không cho cross-track đổi verdict.
- Guard test import module đã bị baseline gỡ được xóa riêng; không chạm `src/`, `config/`, `Makefile`, hay `flight_profile`.
- Python/static gate PASS; rejudge legacy 16 session, 0 lỗi, không đổi verdict.
- SITL/qualification evidence cho W3-A4: `NOT_EVALUABLE`; không tự rejudge/bịa session P0.2 mới.

## Deliverables

- `tools/runtime/**`: salvage runtime, shared evaluation-window selector, diagnostic metadata contract.
- `tools/runtime/tests/**`: replay tests, retired-guard removal, RED→GREEN tests cho J1b.1/J1b.3.
- `docs/safety/runtime_safety_current.md`: hai ledger rows W3-A4.
- `docs/refactor/W3-A4/OPEN_QUESTIONS.md`: R7-25/R7-26 và Q-TRK còn mở.

## Verification thật

```text
python3 tools/validate_runtime_safety_ledger.py
runtime safety ledger validation: PASS (current=484 lines, gates=34, bypasses=1)

git diff --check
GIT_DIFF_CHECK: PASS

python3 tools/check_mission_authority_cut.py
MISSION_AUTHORITY_STATIC_CHECK: PASS

python3 tools/refactor/check_citations.py . docs/refactor
checked=989 out_of_range=0 ambiguous_basenames=['config.hpp', 'execution_anchor.hpp', 'execution_recovery_state.hpp', 'findings.md', 'main.cpp', 'mission.hpp', 'planner.hpp']

python3 -m unittest discover -s tools/tests -p 'test*.py'
Ran 7 tests in 0.206s — OK
python3 -m unittest discover -s tools/runtime/tests -p 'test*.py'
Ran 434 tests in 3.515s — OK (skipped=2)
python3 -m compileall -q tools/runtime
PY_COMPILE: PASS

python3 tools/runtime/rejudge_all.py --workspace /home/letandat/Dev/uav-navigation-w3-a4 --output /tmp/w3-a4-rejudge.csv --roots /home/letandat/.codex/worktrees/wave2-h2/runtime_evidence
sessions=16 errors=0 output=/tmp/w3-a4-rejudge.csv
rejudge distribution: 13 BLOCKED, 3 FAIL; changed=0 versus saved report.json verdicts
```

The rejudge ran on temporary copies and did not mutate source evidence. The
legacy sessions remain evidence-limited and do not establish qualification.

## RED → GREEN

| Behavior | RED | GREEN | Commit |
|---|---|---|---|
| J1b.1 shared evaluation window | 100 m / 100 m/s out-of-window sample changed p95/max | focused evaluation tests passed | `506222c` |
| J1b.3 diagnostic tracking/cross-track | extreme cross-track changed legacy display verdict; metadata helper absent | verdict invariant, metadata/report/HTML tests passed | `78bde4e` |
| Retired publisher guard | `ModuleNotFoundError: tools.check_world_evidence_non_authority` | stale test removed; runtime suite PASS | `c09010c` |

## Deviations and scope

- Salvage patch was applied only with `--include='tools/runtime/**'`; legacy WP-J1
  docs/benchmark/rejudge CSV hunks were not copied or committed.
- J1b.2 and J1b.4 were not changed; both are assigned to W3-F1/ADR-019.
- No threshold, frame conversion, C++ waypoint mirror, `src/`, `config/`,
  `Makefile`, or merge was performed. Branch `refactor/W3-A4` is pushed for
  architecture review; no merge was performed.

## Findings

| Finding | Status | Commit / ownership |
|---|---|---|
| Retired AST publisher-guard test imported a removed baseline module | FIXED | `c09010c` |
| R7-27 window mismatch between statistics and coverage | FIXED | `506222c` |
| Q-XTRK tracking/cross-track must remain report-only | FIXED | `78bde4e` |
| R7-25/R7-26 typed frame authority | PARTIAL / OPEN by owner direction | `78bde4e`; later typed product evidence required |
| Q-TRK runner default | NOT_FIXED here; F1-owned | W3-F1 / ADR-019 |

## Commit table

Source: `git log --format='%h %s' origin/main..HEAD` immediately before adding
this report.

| SHA | Message |
|---|---|
| `720189d` | `refactor(runtime-tools): replay J1 salvage` |
| `c09010c` | `test(runtime-tools): remove retired publisher guard` |
| `506222c` | `fix(runtime-tools): scope tracking stats to evaluation window` |
| `78bde4e` | `fix(runtime-tools): mark tracking metrics diagnostic` |
| `950c0b7` | `docs(runtime-tools): report W3-A4 salvage` |
| `03f6ccc` | `docs(wave3): record A4 remote handoff` |

Branch is pushed as `refactor/W3-A4`; no merge was created.
