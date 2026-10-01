# WP-A4-R1 — Runtime decision-table oracle

## Tóm tắt

1. R1 chuyển deliverable sang schema chung và commit thêm
   `docs/refactor/SCHEMA_decision_table.md`, `var_id`, `event_id`,
   `predicates.csv`, `exit_site` và effects vocabulary đóng.
2. `decision_table.csv` được sinh từ source baseline `7e0b850`; mỗi `return`
   trong 19 function scope có rule riêng, và mỗi side-effect call site có rule
   effect-only riêng.
3. Checker tự parse balanced function spans, tự đếm `return` và call site; nó
   không nhận denominator do bảng truyền vào.
4. Kết quả source coverage: `194/194` return và `81/81` side-effect call site;
   `runCycle` là `75` return theo lexical source span, không phải 74 như ghi
   chú review trước.
5. Có `327` rule, thấp hơn ngưỡng 400 nên giữ một file `decision_table.csv`.
6. Các fault-injection branch được tách rule, guard tham chiếu state var có
   `partition=fault_injection`; planner result được biểu diễn qua event payload.
7. Không sửa `src/`, không chạy SITL/PX4/runtime qualification; các test chưa
   có trong deliverable này giữ `NONE`/`NOT_MEASURED`.

## Deliverables

- [SCHEMA_decision_table.md](../SCHEMA_decision_table.md)
- [state_vars.csv](state_vars.csv)
- [events.csv](events.csv)
- [predicates.csv](predicates.csv)
- [decision_table.csv](decision_table.csv)
- [runcycle_blocks.md](runcycle_blocks.md)
- [coverage_gaps.md](coverage_gaps.md)
- [check_tables.py](check_tables.py)
- [lifecycle_enums.md](lifecycle_enums.md)
- [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md)

## Lệnh verify và output thật

```text
$ python3 docs/refactor/WP-A4/check_tables.py
PASS: rules=327 state_vars=59 events=24 predicates=45
SOURCE_RETURNS total=194 covered=194 gaps=0
SIDE_EFFECT_CALLS total=81 covered=81 gaps=0
RETURNS_BY_FUNCTION runCycle=75;validateRetainedCommand=9;publishCommand=23;commitPlannerCandidate=22;onPropagatedOdometry=5;onRegisteredScan=6;onEstimatorHealth=2;onGoal=3;onModeStatus=8;onCommandAdmission=10;tickMissionProgress=6;applyMissionDecisionLocked=3;resetForLocalizationEpochLocked=2;failClosedLocked=0;schedulePlanningCycle=6;scheduleHeadingRebind=9;consumeHeadingRebind=3;applyQueuedExecutionTimelineActivations=0;suspendCommandForWorldFreshness=2
CALLS_BY_FUNCTION runCycle=25;validateRetainedCommand=6;publishCommand=18;commitPlannerCandidate=19;onPropagatedOdometry=0;onRegisteredScan=0;onEstimatorHealth=0;onGoal=1;onModeStatus=1;onCommandAdmission=0;tickMissionProgress=1;applyMissionDecisionLocked=4;resetForLocalizationEpochLocked=2;schedulePlanningCycle=1;scheduleHeadingRebind=1;consumeHeadingRebind=2;applyQueuedExecutionTimelineActivations=0;suspendCommandForWorldFreshness=0
RUN_CYCLE_SPAN 4010-7548 contiguous
exit_code=0

$ git diff --check
exit_code=0

$ git diff --name-only -- src/
<empty>
exit_code=0

$ python3 tools/validate_runtime_safety_ledger.py
runtime safety ledger validation: PASS (current=500 lines, decisions=706, gates=36, active_gates=34, bypasses=5, active_bypasses=1)
exit_code=0
```

Ledger validator được chạy thêm như một read-only sanity check; R1 không thay
đổi file dưới `docs/safety/`. Safety contract hiện tại và targeted D0
references đã được đọc read-only.

## R1 changes

- Bảng cũ 32 integrated rules được thay bằng 327 rule source-pinned; 194 rule
  có `exit_site`, 81 rule effect-only, 7 fault-injection rule và 45
  planner_fsm predicate rule.
- `check_tables.py` kiểm tra schema, identifier trong guard, payload fields,
  vocabulary effect, predicate map, source return coverage và side-effect call
  coverage.
- `coverage_gaps.md` được xếp lại theo `FAIL_CLOSED` /
  `REQUEST_PX4_HOLD` / `EMERGENCY_BRAKE_*` trước.
- `OPEN_QUESTIONS.md` ghi rõ khác biệt 74/75, denominator planner_fsm và
  trạng thái remote branch divergent.

## Những chỗ lệch khỏi prompt và lý do

- Source baseline thực tế có `runCycle=75` return theo lexical function span;
  checker giữ toàn bộ nested lambda return thay vì loại bỏ một site bằng tay.
- `origin/refactor/WP-A4` ban đầu ở `102964e0`, còn local branch có lịch sử
  WP-A4 riêng tới `a9f0a00`; đã tạo merge commit cùng branch `867337c6` để
  push fast-forward vào PR #5 mà không force-push. Chi tiết ở OQ-04.
- `planner_fsm.hpp` map 45 predicate/helper entries (40 free decision helpers
  plus five public pending-goal methods); private storage helpers không được
  tính vào denominator.
- Không có runtime/PX4/SITL measurement mới; mọi claim qualification là
  `NOT_MEASURED`.

## Open questions

- OQ-01: architecture/ADR/risk docs không có trên baseline, chỉ đọc từ D0.
- OQ-03: lifecycle combinations cần event-log/PX4 trace nếu muốn nâng evidence.
- OQ-04: đã reconcile local WP-A4 history với remote PR #5 ở local; cần xác
  nhận remote SHA sau push.
- OQ-05: có giữ lexical nested-lambda return coverage khi tách reducer hay
  cần AST ownership rule riêng.

## Commit SHA

- Baseline: `7e0b8508781f68ecbd18d15d40129e019108f3e7`.
- Existing WP-A4 commits preserved: `ff8cc083`, `84fa3722`, `a9f0a00e`.
- R1 commit SHA: `94d3a175` (`docs(refactor): refine WP-A4 decision oracle`).
- Report metadata commit: `6ab0b687` (`docs(refactor): record WP-A4-R1 provenance`).
- Same-branch reconciliation merge: `867337c6` (`Merge existing WP-A4 review
  history`); conflict resolution kept the R1 artifacts and touched no `src/`.
- The final metadata commit is created after this update; its exact SHA and
  remote SHA are reported by the final handoff.
