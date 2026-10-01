# W3-A4 — Báo cáo salvage J1 + J1b

## Tóm tắt

- Baseline sau rebase: `origin/main=24ec0fc8718bb8606e4e9e4a7eaa84773d384854`.
- Replay J1 sạch; phạm vi runtime Python, không đổi `src/`, `config/`, ngưỡng, frame conversion hay authority sản phẩm.
- J1b.1 dùng chung `evaluation_window` cho statistics, coverage và `matched_sample_ratio` theo `len(evaluation_reference)`.
- J1b.3 chỉ gắn diagnostic cho cross-track/non-gate; truth-tracking p95/max vẫn giữ `tracking_status`/C0-IFP gate.
- Annotation chạy trên bản copy; `verdict`/`assessment_status` không đổi với cross-track cực đoan.
- Rejudge 16 session: 0 lỗi, phân bố 13 `BLOCKED`/3 `FAIL`, changed=0; acceptance vẫn `NOT_EVALUABLE`.
- Dừng tại checkpoint review; branch đã rebase, chưa merge và chưa tự cấp `MERGE-READY`.

## Deliverables

- `tools/runtime/evaluation.py`: mẫu số ratio theo cửa sổ đánh giá.
- `tools/runtime/tracking_diagnostics.py`: phạm vi diagnostic thu hẹp, deep-copy trước annotate.
- `tools/runtime/tests/**`: RED→GREEN cho ratio, phạm vi nhãn, copy semantics và verdict/assessment invariant.
- `docs/safety/runtime_safety_current.md`: ledger W3-A4 cập nhật đúng scope/evidence/rejudge.
- `docs/refactor/W3-A4/OPEN_QUESTIONS.md`: R7-25/R7-26, Q-TRK và evidence boundary.

## Verification thật

```text
tools/gate.sh static
runtime safety ledger validation: PASS (current=485 lines, gates=34, bypasses=1)
MISSION_AUTHORITY_STATIC_CHECK: PASS
checked=920 out_of_range=0 ambiguous_basenames=['config.hpp', 'execution_anchor.hpp', 'findings.md', 'main.cpp', 'planner.hpp']
DEPENDENCY_DIRECTION: PASS (packages=21, allowed_baseline_violations=4)
GATE_V3_RESULT=PASS

tools/gate.sh python
Python 3.12.3
Ran 19 tests in 0.854s — OK
Ran 437 tests in 2.962s — OK (skipped=2)
GATE_V3_RESULT=PASS

/usr/bin/python3 tools/runtime/rejudge_all.py --workspace /home/letandat/Dev/uav-navigation-w3-a4 --output /tmp/w3-a4-rejudge-r1.csv --roots /home/letandat/.codex/worktrees/wave2-h2/runtime_evidence
sessions=16 errors=0 output=/tmp/w3-a4-rejudge-r1.csv
distribution=13 BLOCKED, 3 FAIL; changed=0 versus saved report.json verdicts
```

`git diff --check` và targeted RED→GREEN đều PASS. Không chạy ROS build/CTest vì write-set chỉ `tools/runtime/**` và docs; không có ROS package bị ảnh hưởng. Artifact rejudge chỉ ở `/tmp`, không sửa evidence nguồn.

## RED → GREEN

| Behavior | RED | GREEN |
|---|---|---|
| J1b.1 window-local ratio | 2/3 matched samples báo `0.666…` thay vì `1.0` | targeted evaluation + full Python gate PASS |
| J1b.3 scope/copy | truth metric bị annotate và `qualification_checks` bị mutate | targeted metadata/verdict tests + full Python gate PASS |
| Retired publisher guard | import `tools.check_world_evidence_non_authority` lỗi | test stale guard đã xoá; full Python gate PASS |

## Deviations and scope

- J1b.2 và J1b.4 không đổi; chuyển W3-F1/ADR-019 theo prompt.
- R7-25/R7-26 không đóng: không có typed world-frame evidence nên không thêm Python frame conversion.
- Không chạy SITL mới; kết quả runtime/qualification của W3-A4 là `NOT_EVALUABLE`.
- Không merge, không sửa main, không thay đổi `Makefile`/`flight_profile`.

## Findings

| Finding | Status | Ownership |
|---|---|---|
| Retired AST publisher-guard test imported removed module | FIXED | W3-A4 |
| R7-27 statistics/coverage/ratio window mismatch | FIXED | W3-A4 |
| Q-XTRK cross-track must remain report-only | FIXED | W3-A4 |
| R7-25/R7-26 typed frame authority | PARTIAL / OPEN | typed product evidence path |
| Q-TRK runner default | NOT_FIXED here | W3-F1 / ADR-019 |

## Commit table

Source: `git log --format='%h %s' origin/main..HEAD` immediately before the final cleanup commit.

| SHA | Message |
|---|---|
| `1eabbc9` | `fix(runtime-tools): narrow diagnostic tracking scope` |
| `5679286` | `test(runtime-tools): expose A4 review regressions` |
| `96e6f7b` | `docs(wave3): complete A4 commit inventory` |
| `6ac15d6` | `docs(wave3): record A4 remote handoff` |
| `2e4e2ab` | `docs(runtime-tools): report W3-A4 salvage` |
| `6757aa8` | `fix(runtime-tools): mark tracking metrics diagnostic` |
| `099daba` | `fix(runtime-tools): scope tracking stats to evaluation window` |
| `0ba9d1d` | `test(runtime-tools): remove retired publisher guard` |
| `06a8a15` | `refactor(runtime-tools): replay J1 salvage` |

Checkpoint: push branch and request architecture re-review; no self-issued merge verdict.
