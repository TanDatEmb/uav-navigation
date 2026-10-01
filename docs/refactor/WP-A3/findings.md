# Findings

Chỉ các bất thường/evidence gap có verdict được ghi ở đây. `CONFIRMED` nghĩa là
đường code/giá trị tồn tại trên baseline; không đồng nghĩa đã chứng minh lỗi
flight. `CONDITIONAL` nghĩa là cần giải quyết provenance/config hoặc runtime
reachability trước khi kết luận rộng hơn.

## F-A3-01 — solve budget lệch giữa safety contract và executable contract

**Verdict: CONFIRMED / OPEN CONTRACT CONFLICT.** `PlanningTimingContract` và
`planner.yaml` đặt solve deadline 80 ms (`planning_timing.hpp:9-15`,
`planner.yaml:40-44`), và runtime từ chối planner nếu giá trị không khớp
(`navigation_runtime_node.cpp:1691-1697`). Safety ledger lại ghi HG-001 solve
180 ms (`docs/safety/runtime_safety_current.md:89-97`). A* cũng có current config
30/60 ms (`planner.yaml:196-203`) trong khi ledger mô tả A* attempt/total 40/80 ms.
Không tự chọn giá trị; ADR/safety owner phải xác nhận authority trước khi gán lane
hoặc chốt certifier.

## F-A3-02 — wall timers chạy trong mô phỏng với quyết định dùng ROS time

**Verdict: CONFIRMED / DESIGN RISK.** Runtime có ba `create_wall_timer`
(`navigation_runtime_node.cpp:1773-1783`) và adapter có hai
(`navigation_mode_node.cpp:299-300`, `2599-2600`). Launch truyền `use_sim_time`
(`navigation_runtime.launch.py:17-28`, `px4_external_mode.launch.py:14-24`),
nhưng timer cadence không chờ `/clock`; command/world/lease checks lại dùng ROS
và receive checks dùng steady (`navigation_runtime_node.cpp:8891-8951`,
`navigation_mode_node.cpp:2422-2444`). Đây là mixed scheduling có chủ ý/hiện hữu,
nhưng chưa có artifact chứng minh worst-case khi `/clock` pause.

## F-A3-03 — measured emergency brake chạy trên planning decision thread

**Verdict: CONFIRMED / LATENCY EVIDENCE GAP.** `commitEmergencyBrake` được gọi
từ `validateRetainedCommand` tại `navigation_runtime_node.cpp:8103-8145`; hàm này
được gọi từ planning job tại `navigation_runtime_node.cpp:6307-6318`, và job chạy
trên `PlanningWorker` jthread (`planning_worker.hpp:293-300`). Vì vậy measured
brake không chạy trên command timer riêng mà trên thread đang quyết định/solve.
Source không đặt WCET riêng; artifact đã đọc không có duration distribution cho
brake (`timing_budget.csv`). Không được suy ra “mất 0/80 ms” từ solve budget.

## F-A3-04 — localization reset có đường chờ không bounded

**Verdict: CONFIRMED / FAIL-PATH BUDGET GAP.** Reset nhả lifecycle lock để drain,
nhưng `MappingWorker::reset()` đợi vô hạn tới khi `in_flight_` false
(`mapping_worker.hpp:122-134`; helper unlock/relock tại
`localization_epoch_reset.hpp:15-28`). Nếu `process_` hoặc callback publication
không kết thúc, reset không có timeout sản phẩm. Đây không phải deadlock đã tái
hiện; đây là thiếu giới hạn thời gian trên đường fail/reset.

## F-A3-05 — backend cancellation giữ worker mutex và không có timeout

**Verdict: CONFIRMED / NO_CYCLE (static); LATENCY RISK.** `PlanningWorker::submit` và
`cancelActive` giữ `mutex_` khi gọi `planner_->cancelActiveSolve()`
(`planning_worker.hpp:119-131`, `175-184`). Runtime còn gọi `cancelActive()` trong
localization reset (`navigation_runtime_node.cpp:1903-1908`). Watchdog 1 s chỉ
phát hiện solve age và xử lý phía runtime (`navigation_runtime_node.cpp:8749-8799`),
không đặt timeout cho chính cancellation/worker critical section. Chuỗi lock
không quay ngược vào runtime lifecycle lock hoặc `PlanningWorker::mutex_` từ
`solve_commit_mutex_`; chi tiết chứng minh ở mục R1 bên dưới.

## F-A3-06 — replan lock bao trùm thao tác tốn thời gian và gọi ra ngoài

**Verdict: CONFIRMED / NO_CYCLE (static); LATENCY TAIL RISK.** `replan_lock_` được giữ từ đầu
`planInitialFromStoppedStateImpl`/successor (`planner.cpp:1853-1861`,
`2083-2092`) qua A*, EXP, BACKUP, visualization, logging và commit authorizer
(`planner.cpp:1953-1976`, `2002-2021`, `2126-2182`, `2254-2272`). Lock graph không
cho thấy thứ tự ngược tới `drone_state_mutex_`, `solve_commit_mutex_`, runtime
lock hoặc worker mutex, nhưng lock duration và callback duration chưa có
p50/p95/p99 trong artifact WP-A3. Đây là risk timing, không phải kết luận deadlock.

## F-A3-07 — available measurements không đủ cho acceptance timing

**Verdict: CONFIRMED / NOT_EVALUABLE FOR QUALIFICATION.** Artifact
`artifacts/qualification_gate_recovery/20260924T082427Z-4d184896/` có các phân
phối producer→adapter, nhưng provenance ghi base SHA khác target; không có phân
phối cho A*, solve, stitch, commit guard, mapping update, certificate aggregate,
brake hoặc watchdog. Các số đã chép nguyên dạng trong `timing_budget.csv` chỉ là
transport diagnostics và không được dùng làm flight acceptance.

## HG-001 — đối chiếu authority và scope

WP-A3-R1 không đổi bất kỳ giá trị nào. Bảng dưới chỉ ghi nhận số đang tồn tại
trên các authority khác nhau; số đo trong `timing_budget.csv` không được dùng để
chọn lại threshold.

| metric | HG-001 hiện hành | code/config đang chạy | scope/units | authority decision |
|---|---:|---:|---|---|
| A* attempt | 40 ms | runtime YAML `search_time_limit_s=0.03` = 30 ms | HG-001: SUPER absolute; YAML: runtime retry-attempt budget | **OPEN — project owner chọn authority** |
| A* total | 80 ms | runtime YAML `total_time_limit_s=0.06` = 60 ms; typed config yêu cầu total < solve | HG-001: SUPER absolute; YAML/config: bounded A* stage | **OPEN — project owner chọn authority** |
| solve | 180 ms | typed product `kSolveDeadlineS=0.08` = 80 ms | HG-001: SUPER absolute; typed timing: product planner solve deadline | **OPEN — project owner chọn authority** |
| future-state lead | 200 ms | typed `kStitchDurationS=0.40` = 400 ms; runtime reserve dùng giá trị này | HG-001: SUPER future-state lead; typed/runtime: committed-future stitch/activation lead | **OPEN — project owner chọn authority** |

Evidence exact: HG-001 `docs/safety/runtime_safety_current.md:89-92`; runtime
YAML `src/runtime/navigation_runtime/config/planner.yaml:196-203`; typed product
contract `src/planning/navigation_planning/include/navigation_planning/planning_timing.hpp:9-15`;
future-anchor arithmetic `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:5623-5645`;
planner A* ordering `src/planning/navigation_planning_backend/include/planner_core/config.hpp:300-312`.
Lineage/owner history was indexed at `docs/safety/runtime_safety_index.md` and
`docs/safety/archive/runtime_safety_legacy_full.md` (both removed at the
2026-10-01 baseline); no authority is selected in this read-only revision.

## R1 — F-04/F-05/F-06 complete lock-chain closure

Trong yêu cầu này, F-04 là reset/cancel dưới localization lifecycle lock,
F-05 là terminal mode-status/cancel dưới bộ lifecycle locks, F-06 là planner
replan/outcall và worker cancellation. Verdict `NO_CYCLE` nghĩa là không tìm thấy
directed cycle trong graph acquisition/call đã kiểm trên baseline; không có nghĩa
outcall là non-blocking hoặc đã được chứng minh bằng runtime trace.

| finding | full forward chain checked | reverse edge checked | verdict |
|---|---|---|---|
| F-04 | `localization_transition_mutex_` (`navigation_runtime_node.cpp:1899-1907`) → `PlanningWorker::mutex_` → `PlannerFacade::cancelActiveSolve()` → `Planner::solve_commit_mutex_` (`planning_worker.hpp:177-185`; `planner.hpp:603-606`) | planner/backend cancellation, staging and activation paths were searched for runtime `localization_transition_mutex_`, `input_mutex_` and command transition acquisition; none found | **NO_CYCLE** |
| F-05 | `localization_transition_mutex_` → `input_mutex_` → command transition (`navigation_runtime_node.cpp:2526-2529`) → `cancelActive()` (`:2623-2628`) → worker mutex → `solve_commit_mutex_` | `solve_commit_mutex_` holders only update planner/warm-start/world-authorizer state (`planner.cpp:818-841,876-915`); no return edge to runtime lifecycle locks | **NO_CYCLE** |
| F-06 | PlanningWorker lifecycle `mutex_` held by `submit/cancel/shutdown` → `planner_->cancelActiveSolve()` (`planning_worker.hpp:105-170,177-244`) → `solve_commit_mutex_`; normal `run()` releases worker mutex before `backend_access_mutex_` and job (`:293-300`) | searched all `solve_commit_mutex_` holders for worker mutex/backend access and all planner calls into runtime; no reverse worker/runtime edge | **NO_CYCLE** |

Các đường PlanningWorker thread gọi ngược tới runtime/planner đã kiểm: job gọi
`applyQueuedExecutionTimelineActivations()` rồi
`planner.onExecutionTimelineActivated()` (`navigation_runtime_node.cpp:3991-4007`)
sau khi worker lifecycle mutex đã nhả; `runCycle()` gọi
`validateRetainedCommand()` (`:6317-6318`), nhưng `commitEmergencyBrake()` chỉ
giữ `solve_commit_mutex_` trong callback stage rồi trả về trước
`commitPlannerCandidate()` (`:8143-8164`); `authorizeAndStage()` có thứ tự
`publication_gate_ -> solve_commit_mutex_` (`planner.cpp:818-841`;
`world_snapshot_store.hpp:137-153`). Không có planner→runtime callback nào trong
các đoạn này lấy lại runtime localization/input/command lock. `replan_lock_ ->
drone_state_mutex_` và logging/visualization outcall vẫn là latency risk
(`planner.cpp:1850-1877,2078-2091,2267-2276`), không tạo reverse cycle trong
graph đã kiểm.
