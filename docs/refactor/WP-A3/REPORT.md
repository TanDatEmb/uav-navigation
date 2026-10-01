# WP-A3-R1 — Bổ sung kịch bản, khai thác số đo, chốt câu hỏi lock

## 1. Tóm tắt

WP-A3-R1 là audit tĩnh read-only trên baseline `main@7e0b8508781f68ecbd18d15d40129e019108f3e7`; không sửa `src/`.
Pipeline sensor → FAST-LIO → mapping → planner → command → adapter → PX4 và odometry → EKF2 đã được truy vết theo thread, clock, identity, deadline và nhánh lỗi.
Kết luận AS-IS: **CONDITIONAL / NOT QUALIFIED**.
Các điểm đã xác nhận gồm `MultiThreadedExecutor(2)`, các worker runtime, wall timer trong simulated-time, divergence A*/solve giữa safety và runtime, và outcall/blocking trong lock scope.
R1 bổ sung S13–S17 và bảng map 12 kịch bản gốc → S-id; các đường F-04/F-05/F-06 có verdict static `NO_CYCLE`.
Tám dòng timing có số đo diagnostic; mười dòng còn lại là `NOT_MEASURED` và đều có `searched` không rỗng.
Mọi số đo không có artifact đúng provenance được ghi `NOT_MEASURED`; artifact khác SHA chỉ được dùng như diagnostic evidence.
Không có thay đổi hành vi, threshold, lease, bypass hoặc safety ledger.
Các câu hỏi authority/provenance còn mở nằm trong `OPEN_QUESTIONS.md`.

## 2. Deliverables

- [`threads.csv`](threads.csv): process, executor/thread, callback group, timer/worker, clock, locks và shared state.
- [`lock_order.md`](lock_order.md): lock graph runtime/Planner, thứ tự ngược, outcall-under-lock và unlock giữa chừng.
- [`pipeline.md`](pipeline.md): S1–S12 và bổ sung S13–S17, mỗi scenario có Mermaid diagram và bảng bước; có bảng map 12 scenario gốc.
- [`timing_budget.csv`](timing_budget.csv): budget, enforcement, đo p50/p95/p99/max/n hoặc `NOT_MEASURED`, kèm `measurement_source`, `measurement_condition`, `searched`.
- [`clock_domains.md`](clock_domains.md): ROS/wall/steady comparisons và wall-timer dưới `use_sim_time=true`.
- [`findings.md`](findings.md): bất thường với verdict `CONFIRMED`/`CONDITIONAL`/`SPECULATIVE`.
- [`OPEN_QUESTIONS.md`](OPEN_QUESTIONS.md): input WP-D0/ADR/risk còn thiếu và authority cần chốt.

## 3. Phạm vi và bằng chứng

- Safety contract/index đã đọc tại `docs/safety/runtime_safety_current.md:1-149` và `docs/safety/runtime_safety_index.md:1-60` (index đã gỡ khỏi repo ở baseline 2026-10-01).
- Baseline có `docs/refactor/WP-A3/*`, nhưng thiếu các input bắt buộc `docs/refactor/ARCHITECTURE_REVIEW.md`, `docs/refactor/adr/ADR-013..016` và `docs/refactor/risk_register_20260928.md`; không dùng branch/commit khác để thay thế provenance. Chi tiết: `OPEN_QUESTIONS.md` OQ-A3-01.
- A* 40/80 ms trong safety ledger khác runtime 30/60 ms tại `docs/safety/runtime_safety_current.md:91` và `src/runtime/navigation_runtime/config/planner.yaml:196-203`; typed solve budget 80 ms tại `src/planning/navigation_planning/include/navigation_planning/planning_timing.hpp:9-15`.
- Mapping/certificate aggregate và product WCET không có đo đúng baseline; telemetry chỉ được ghi như diagnostic tại `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:913-957,1008-1052,1218-1305,1447-1458`.

## 4. Validation đã chạy

Các lệnh sau chạy trong `<repo>`:

```text
$ git merge-base --is-ancestor 7e0b8508781f68ecbd18d15d40129e019108f3e7 HEAD
exit code: 0

$ git diff --name-only 7e0b8508781f68ecbd18d15d40129e019108f3e7 HEAD
docs/refactor/WP-A3/OPEN_QUESTIONS.md
docs/refactor/WP-A3/REPORT.md
docs/refactor/WP-A3/clock_domains.md
docs/refactor/WP-A3/findings.md
docs/refactor/WP-A3/lock_order.md
docs/refactor/WP-A3/pipeline.md
docs/refactor/WP-A3/threads.csv
docs/refactor/WP-A3/timing_budget.csv

$ python3 tools/validate_runtime_safety_ledger.py
runtime safety ledger validation: PASS (current=500 lines, decisions=706, gates=36, active_gates=34, bypasses=5, active_bypasses=1)
exit code: 0

$ git diff --check
exit code: 0

$ git diff -- src/
<no output>
exit code: 0

$ structural audit check
threads_rows=33; timing_rows=18; required_scenarios=12; mermaid_diagrams=17; created_at_missing=0; defined_at_missing=0
exit code: 0
```

Không chạy test/runtime qualification vì đây là deliverable read-only; artifact đo được không đại diện cho baseline `7e0b850` nếu provenance ghi SHA khác.

## 5. R1 changes — số đo và lock-chain

Các dòng đã khai thác được, không gộp khác điều kiện:

| budget_id | p50 / p95 / p99 / max | n | nguồn và điều kiện |
|---|---:|---:|---|
| `SOLVE`, `HG-001-SOLVE` | 8.532 / 13.691 / 24.041 / 24.041 ms | 27 | E10 `planning_total_us`; open map, target/requested 3.0 m/s, commit `cf5dcf0`, diagnostic BLOCKED |
| `PLANNER-PERIOD` | 0 / 0.081 / 0.148 / 0.148 ms | 32 | E10 `planning_scheduling_gap_us`, cùng điều kiện; đây là scheduling gap, không phải cadence |
| `COMMAND-PERIOD` | 20 / 20 / 20 / 20 ms | 3795 | `NOMINAL_TIMING.csv`, long_featured, seed0, External Mode, source commit `96ed8d0`, cohort artifact |
| `SNAPSHOT` | 9.897 / 13.339 / 15.020 / 16.480 ms | 302 | E10 `world_snapshot_export_us`, cùng điều kiện E10 |
| `MAPPING-UPDATE` | 20.267 / 29.850 / 39.572 / 56.590 ms | 302 | E10 `mapping_total_update_us`, cùng điều kiện E10 |
| `PROPAGATED-PUBLISH` | 0.061544 / 0.086799 / 0.106822 / 0.781728 ms | 3939 | `NOMINAL_TIMING.csv`, long_featured, seed0, External Mode, source commit `96ed8d0` |
| `PRODUCER-PUBLISH` | 25.001005 / 25.405100 / 25.677297 / 27.805225 ms | 3938 | `NOMINAL_TIMING.csv`, cùng cohort |

`CERTIFICATE-AGGREGATE`, state age, accepted gap, clock gap, lease, watchdog,
stitch và commit guard không có phép đo đúng metric/điều kiện nên giữ
`NOT_MEASURED`; các đường tìm kiếm cụ thể nằm trong cột `searched` của CSV.
Các số E10/cohort là diagnostic, khác baseline và không phải flight acceptance.

F-04/F-05/F-06 đều **NO_CYCLE (static source audit)**: đã liệt kê full forward
chain và tìm reverse edge trong `findings.md` và `lock_order.md`. Kết luận này
không chứng minh outcall không block, không thay thế runtime contention trace.

HG-001 không thay đổi. Bảng tại `findings.md` đối chiếu chính xác A* attempt,
A* total, solve và future-state lead; authority còn mở để chủ dự án quyết định
giữa SUPER absolute và typed/runtime product budget.

## 6. Lệch prompt và open questions

- Các file `ARCHITECTURE_REVIEW.md`, ADR-013..016 và risk register được yêu cầu đọc nhưng không tồn tại trong target baseline; D0 được đọc read-only từ `origin/refactor/WP-D0`, không merge/cherry-pick. Chi tiết: `OPEN_QUESTIONS.md` OQ-A3-01.
- `timing_budget.csv` không suy diễn số đo cho những metric chỉ có proxy hoặc khác provenance; đó là lý do còn 10 dòng `NOT_MEASURED`.
- Static `NO_CYCLE` không phải runtime proof; project owner vẫn cần quyết định authority HG-001 và có thể yêu cầu trace/benchmark riêng.

## 7. Verify R1 — output thật

Các lệnh chạy tại `<repo>`:

```text
$ git diff --check
exit code: 0

$ git diff --name-only 7e0b850 -- src | wc -l
0
exit code: 0

$ python3 tools/validate_runtime_safety_ledger.py
runtime safety ledger validation: PASS (current=500 lines, decisions=706, gates=36, active_gates=34, bypasses=5, active_bypasses=1)
exit code: 0

$ python3 - <<'PY'
import csv
p='docs/refactor/WP-A3/timing_budget.csv'
with open(p, newline='') as f:
    rows = list(csv.DictReader(f))
assert len(rows) == 18
assert all(row['searched'].strip() for row in rows if row['measured_p50'] == 'NOT_MEASURED')
print(f"timing_rows={len(rows)}; not_measured_rows={sum(row['measured_p50'] == 'NOT_MEASURED' for row in rows)}; empty_searched=0")
PY
timing_rows=18; not_measured_rows=10; empty_searched=0
exit code: 0
```

## 8. Commit và publication

- Base commit: `7e0b8508781f68ecbd18d15d40129e019108f3e7`.
- Audit commits trước R1 đã có: `842e646cee1efa4e250e32c6261232583bba32cd`, `0b0131c0ba1fc40f53e195944dcac06cdf092405`, `39ac417b62324acb365e5355c6383a8510243fec`, `e050dc66762ab5df6c18a22bb39e61fc1fe4019e`.
- R1 verification commit (REPORT + timing correction): `aa824a6d8b5e57c1d63533324d0a4e91a8262106`; metadata follow-up: `fe2178b15e83d0c5b40f950803adf076138cb646`.
- Branch: `refactor/WP-A3`, push fast-forward lên remote; PR #6 giữ trạng thái draft, base `main`; không tự merge.

WP-A3-R1 không phải qualification, không đóng open questions, không tuyên bố
WCET/p99 cho dòng `NOT_MEASURED`, và không thay đổi product behavior.
