# WP-H1 — Hotfix trọng tài R-01

> **Ghi chú baseline (2026-10-01):** các đường dẫn `artifacts/`, `runtime_evidence/`, `.artifacts/`, `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, safety index/archive và CI workflow được nhắc trong tài liệu này đã được gỡ khỏi repo. Coi chúng là nguồn gốc lịch sử, không còn kiểm chứng lại được. Xem mục "Ghi chú baseline" trong `docs/refactor/README.md`.

## 1. Tóm tắt

- Đã sửa judge Python, không đổi product C++ và không đổi ngưỡng stale.
- `terminal_handover_wall_ns` nay được scenario ghi theo wall-clock ns tại mốc terminal đầu tiên và các event mode-exit/fail-closed liên quan.
- Judge chỉ miễn stale event của `external_odometry` và `propagated_odometry` khi `event_time_ns >= terminal_handover_wall_ns` hợp lệ.
- COMPLETE không còn xoá stale violation của toàn bộ active window; stall giữa mission vẫn FAIL.
- Session cũ thiếu hoặc có marker hỏng được xử lý fail-closed: không miễn stale nào.
- Regression tests (a)–(e) và repro R-01 đã thêm, focused suite PASS 7/7.
- Corpus `artifacts/baseline_20260928/` không tồn tại trong clone này, nên không có verdict delta và không dựng dữ liệu thay thế.
- Kết quả này là sửa tooling/judge component, không phải flight qualification.

## 2. Deliverables

- [tools/runtime/report.py](../../../tools/runtime/report.py): `handover_stale_exemption()` và bounded exemption trong `_sim_report`.
- [tools/runtime/external_mode_scenario.py](../../../tools/runtime/external_mode_scenario.py): ghi marker wall-ns vào scenario summary và handover events.
- `tools/runtime/tests/test_report_handover.py` (not part of this baseline): regression/unit tests (a)–(e), R-01 và scenario marker.
- [docs/safety/runtime_safety_current.md](../../../docs/safety/runtime_safety_current.md): entry `WP-H1-R01` trong Recent effective changes.
- `docs/refactor/WP-H1/verdict_delta.csv`: NOT_CREATED — baseline corpus không tồn tại.

## 3. Bằng chứng baseline và line anchors

Baseline là `main @ 7e0b8508781f68ecbd18d15d40129e019108f3e7`.

- R-01 hiện hữu trên baseline tại `tools/runtime/report.py:3511-3541`: `terminal_handover` gồm `COMPLETE` và đặt stale violation về zero cho hai stream, không giới hạn theo thời điểm event.
- Baseline scenario recorder là `tools/runtime/external_mode_scenario.py:574-588`; mode exit event là `:1472-1487`; fail-closed mode event là `:1943-1968`; chưa có terminal handover wall-ns marker.
- Architecture/risk context được đọc read-only từ `origin/refactor/WP-D0`: `docs/refactor/ARCHITECTURE_REVIEW.md`, ADR-013..016 và `docs/refactor/risk_register_20260928.md` (R-01).

## 4. Lệnh verify và output thật

Focused WP-H1 suite:

```text
python3 -m unittest discover -s tools/runtime/tests -p 'test_report_handover.py' -v
Ran 7 tests in 0.102s
OK
```

Full Python suite, không skip:

```text
python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py'
Ran 467 tests in 5.275s
FAILED (failures=1, skipped=1)
Failure: test_package_and_workspace_runtime_profiles_have_one_navigation_contract
  install/navigation_runtime/share/navigation_runtime/config/planner.yaml is absent
```

Full Python suite với đúng test install-tree được skip vì P0.1 chưa có trong clone:

```text
full_with_install_tree_skip_summary: testsRun=467 failures=0 errors=0 skipped=2
Ran 467 tests in 5.479s
OK (skipped=2)
```

Safety ledger and syntax/whitespace:

```text
python3 tools/validate_runtime_safety_ledger.py
runtime safety ledger validation: PASS (current=500 lines, decisions=706, gates=36, active_gates=34, bypasses=5, active_bypasses=1)

git diff --check
PASS (exit 0; no output)

python3 -m py_compile tools/runtime/report.py tools/runtime/external_mode_scenario.py tools/runtime/tests/test_report_handover.py
PASS (exit 0; no output)
```

Revert proof using the same new test file with only `tools/runtime/report.py` loaded from baseline `7e0b850`:

```text
Ran 7 tests in 0.108s
FAILED (failures=5, errors=1)
baseline_revert_summary: testsRun=7 failures=5 errors=1
```

The failures are the expected proof that baseline report logic still hides the COMPLETE mid-mission stall, does not apply bounded post-handover filtering, and lacks `handover_stale_exemption`.

Original R-01 reproduction on baseline report:

```text
outcome=''           source_stale_events=1  odometry reasons=['propagated_odometry timestamp/freshness/validity violation', 'external_odometry timestamp/freshness/validity violation']
outcome='COMPLETE'   source_stale_events=1  odometry reasons=[]
```

Baseline corpus check:

```text
test -d artifacts/baseline_20260928
NOT_FOUND (exit 1)
```

The standard report entry point identified for a future corpus rerun is:

```text
python3 tools/runtime/report.py --session <session> --workflow external-mode --config <config> --workspace <workspace>
```

## 5. Lệch khỏi prompt

- Không tạo `verdict_delta.csv` vì `artifacts/baseline_20260928/` không tồn tại; không có session hợp lệ để build report read-only.
- Full suite chưa build install-tree nên test contract cũ fail; kết quả có skip-install-tree đã PASS 467/467 executions với 2 skips.
- Chưa chạy SITL/PX4: WP-H1 là judge-only và không thay đổi product C++.

## 6. Open questions

Không có câu hỏi mở hoặc mâu thuẫn cần tự quyết. Việc thay thế hotfix bằng typed P2 judge vẫn là removal condition của ledger entry.

## 7. Commit SHA

- `31531dc7` — `fix(runtime-judge): bound stale exemption to handover`.
