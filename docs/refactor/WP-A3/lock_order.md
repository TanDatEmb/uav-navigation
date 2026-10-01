# Lock order audit

Phạm vi là source tại `7e0b850`. Tôi quét các khai báo lock/unlock của
`NavigationRuntimeNode` và các acquisition của `Planner`; đây là đồ thị thực tế,
không phải đề xuất refactor. Các mutex của worker/backend được ghi riêng khi một
lệnh gọi từ node/planner đi qua chúng.

## `navigation_runtime_node`

### Đồ thị

```text
localization_epoch_ingress_mutex_
        -> localization_transition_mutex_
                -> input_mutex_
                        -> command_execution_lease_failure_latch_.transitionMutex()
                -> propagated_derivative_mutex_
```

Evidence:

| Cạnh | Nơi mutex thứ hai được lấy khi mutex thứ nhất đang giữ |
|---|---|
| `localization_epoch_ingress_mutex_ -> localization_transition_mutex_` | `navigation_runtime_node.cpp:2049-2050`, `2078-2079`, `2100-2101` |
| `localization_transition_mutex_ -> input_mutex_` | `navigation_runtime_node.cpp:1922-1924`, `4049-4052`, `5692-5695`, `7599-7602` |
| `input_mutex_ -> command_execution_lease_failure_latch_.transitionMutex()` | `navigation_runtime_node.cpp:1922-1924`, `2262-2264`; header contract `navigation_runtime_node.hpp:280-286` |
| `localization_transition_mutex_ -> propagated_derivative_mutex_` | `navigation_runtime_node.cpp:2139-2150` (derivative update is inside the localization-transition critical section) |

`navigation_runtime_node.hpp:467-471` declares the canonical order as
`ingress -> localization -> input -> command`. The source scan found no reverse
acquisition of any pair above in the baseline. The comment at
`navigation_runtime_node.cpp:4507-4510` calls the semantic transition
`input_mutex_ -> execution transition`; the actual enclosing acquisition remains
`localization -> input -> command` at `4511-4514`, so this is not a reverse lock
edge.

Các mutex độc lập trong node: `planner_solve_activity_mutex_` and
`planner_timeline_activation_mutex_` (`navigation_runtime_node.hpp:581-588`),
`heading_rebind_mutex_` (`navigation_runtime_node.hpp:638-640`), and the
`propagated_derivative_mutex_` above. Không có cạnh source-confirmed nối chúng
vào canonical transition graph, ngoại trừ việc callback có thể gọi worker API
đang có mutex riêng.

### Thứ tự ngược và gọi ra ngoài khi đang giữ lock

- Không có thứ tự ngược chiều nào được tìm thấy giữa bốn mutex canonical.
  Đây là kết quả kiểm tra source, không phải chứng minh runtime absence of a
  future path.
- `NavigationRuntimeNode::resetForLocalizationEpochLocked` gọi
  `planning_worker_->cancelActive()` khi đang giữ localization lock
  (`navigation_runtime_node.cpp:1903-1908`). API này lấy worker mutex và gọi
  `PlannerFacade::cancelActiveSolve()` trong worker mutex
  (`planning_worker.hpp:177-184`). Đây là gọi ra ngoài qua lock boundary và có
  thể chặn reset theo thời gian backend cancellation.
- Mapping publication thực hiện `store->publishAndFinalizeDecision(...)` trong
  transaction đang giữ localization/input/command locks
  (`navigation_runtime_node.cpp:1222-1252`). Đây là external store/authority
  call, không phải một ROS publish thuần túy.
- Fatal planning worker handler lấy ba canonical locks rồi gọi
  `failClosedLocked()` và log lỗi (`navigation_runtime_node.cpp:1701-1713`).
- Mission path gọi ROS publishers khi transition locks còn giữ: mission-complete
  publish tại `navigation_runtime_node.cpp:2746-2755`, và các receipt/status
  publish trong `applyMissionDecisionLocked` được gọi từ các critical sections
  (`navigation_runtime_node.cpp:2709-2757`, `2789-2862`).
- Có log throttled trong critical section, ví dụ
  `navigation_runtime_node.cpp:4493-4497` và `4694-4699`; các log ở
  `1707-1713` cũng nằm dưới ba lock. Đây là logging call, không phải chứng cứ
  về thời gian log.
- `publishCommand` thực hiện `failClosedLocked()` dưới canonical locks, nhưng
  chủ động nhả lock trước khi cancel worker và log terminal failure
  (`navigation_runtime_node.cpp:8952-8971`); không gọi planner/ROS publish ở
  giữa critical section đó.

### Unlock giữa chừng

| Đường chạy | Bằng chứng | Ý nghĩa |
|---|---|---|
| Localization reset / mapping drain | `localization_epoch_reset.hpp:15-28`, gọi tại `navigation_runtime_node.cpp:1966-1970` | Nhả `localization_transition_mutex_`, chạy `mapping_worker_->reset()`, rồi lấy lại; nếu exception thì relock trước khi throw. Các lock khác không được giữ qua drain theo contract header. |
| World freshness suspension | `navigation_runtime_node.cpp:3471-3482` | Nhả command latch trước `publishWorldTransactionWitness`; chỉ witness sau khi mutation đã xong. |
| Foreign mission sau stop | `navigation_runtime_node.cpp:4524-4535` và `4713-4721` | Nhả command, input, localization theo thứ tự ngược rồi mới `planning_worker_->cancelActive()`. |
| Command lease failure | `navigation_runtime_node.cpp:8967-8989` | Nhả cả ba lock để cancel/log; sau đó relock theo canonical order để recheck identity. |
| Planner watchdog | `navigation_runtime_node.cpp:8840-8860` | Nhả `planner_solve_activity_mutex_` trước log; cuối path chỉ unlock nếu còn sở hữu. |

Điểm đáng chú ý: `MappingWorker::reset()` đợi `in_flight_ == false`
(`mapping_worker.hpp:122-134`) nhưng không có timeout sản phẩm; vì vậy unlock
giữa chừng tránh deadlock với mapping callback nhưng không tạo bounded reset.

## `Planner` — năm mutex

Năm mutex được khai báo tại `src/planning/navigation_planning_backend/include/planner_core/planner.hpp:100-104`:

1. `drone_state_mutex_`
2. `replan_lock_`
3. `solve_commit_mutex_`
4. `command_identity_mutex_`
5. `planner_timeline_mutex_`

### Đồ thị source-confirmed

```text
replan_lock_
    -> drone_state_mutex_
    -> planner_timeline_mutex_
    -> solve_commit_mutex_
    -> command_identity_mutex_
```

| Cạnh | B được lấy khi A đang giữ | Đường gọi chứng minh A còn giữ |
|---|---|---|
| `replan_lock_ -> drone_state_mutex_` | `planner.cpp:1853-1857` và `2084-2089` | `planInitialFromStoppedStateImpl` / `planSuccessorFromExecutionAnchorImpl` giữ `replan_lock_` trong toàn solve |
| `replan_lock_ -> planner_timeline_mutex_` | `planner.cpp:2428-2431` (`plan` được gọi trong worker; replan path dùng cùng planner transaction) và `planner.cpp:68-94` (`setPlannerStage`) | các stage `setPlannerStage` được gọi bên trong replan tại `1863`, `1968`, `2094`, `2172`, `3258`, `3726`, `4404` |
| `replan_lock_ -> solve_commit_mutex_` | `planner.cpp:818-841` (`authorizeAndStage`), `865-873` (`stageCommandHistoryForCandidate`) | call sites trong replan: `2002-2012`, `2033-2044`, `2254-2264`, `2283-2293`, `2313-2323` |
| `replan_lock_ -> command_identity_mutex_` | `planner.hpp:486-495` | `Planner::plan` đặt identity tại `planner.cpp:2490-2498` trước khi gọi initial/successor implementation giữ `replan_lock_` |

`solve_commit_mutex_` cũng được lấy độc lập trong
`onExecutionTimelineActivated`, `discardCommandCandidate` và
`discardRetainedPositionHeadingCandidate` (`planner.cpp:876-933`); các path này
không tạo thêm cạnh source-confirmed với bốn mutex còn lại. `command_identity_mutex_`
chỉ xuất hiện trong `setCommandIdentity` (`planner.hpp:486-495`), và
`planner_timeline_mutex_` trong reset/stage/finish timeline
(`planner.cpp:55-109`).

### Thứ tự ngược và gọi ra ngoài

- Không tìm thấy acquisition reverse của các cạnh trên trong source baseline.
  Đặc biệt không thấy `drone_state_mutex_ -> replan_lock_` hay
  `solve_commit_mutex_ -> replan_lock_`.
- `replan_lock_` bao trùm các thao tác tốn thời gian và gọi ra ngoài: map
  classify/nearest, visualization (`planner.cpp:1886-1889`, `2018-2023`,
  `2163-2166`, `2267-2272`), A*/corridor/MINCO qua
  `generateExpTraj`/`generateBackupTrajectory` (`planner.cpp:1953-1976`,
  `2126-2182`), `authorizeAndStage` (`2002-2005`, `2254-2257`), and planner
  context logging. Đây là CONFIRMED lock hold; thời gian thực tế chưa đo trong
  các artifact được dùng cho WP-A3.
- `authorizeAndStage` gọi commit authorizer/world transaction rồi mới lấy
  `solve_commit_mutex_` trong callback (`planner.cpp:818-841`). Vì caller đang
  giữ `replan_lock_`, đây là external call và nested lock edge cùng một đường.
- `stageCommandHistoryForCandidate` lấy `solve_commit_mutex_` sau khi đã chạy
  solve và vẫn trong replan (`planner.cpp:865-873`, call sites ở trên).
- `planner_context_->viz...` và `planner_context_->warn/info/error` là external
  callbacks/logging dưới `replan_lock_`; không có timeout quanh chúng.

### Kết luận lock

Đồ thị hiện tại có thứ tự nhất quán ở các acquisition đã tìm thấy, nhưng
`replan_lock_` là một lock thô bao quanh solve và I/O-like callbacks, còn
`MappingWorker::reset` và backend cancellation không có timeout. Đây là evidence
cho rủi ro latency/blocking, chưa phải chứng minh deadlock hoặc violation trong
runtime.

## R1 — F-04/F-05/F-06 cycle audit

### Forward chains

| finding | chain while the initiating lock is held | checked source |
|---|---|---|
| F-04 | `localization_transition_mutex_` → `PlanningWorker::mutex_` → `PlannerFacade::cancelActiveSolve()` → `Planner::solve_commit_mutex_` | `navigation_runtime_node.cpp:1899-1907`; `planning_worker.hpp:177-185`; `planner_facade.cpp:221-223`; `planner.hpp:603-606` |
| F-05 | `localization_transition_mutex_` → `input_mutex_` → command transition mutex → `cancelActive()` → worker mutex → `solve_commit_mutex_` | `navigation_runtime_node.cpp:2526-2529,2623-2628`; `planning_worker.hpp:177-185`; `planner.hpp:603-606` |
| F-06 | worker lifecycle `mutex_` in `submit/cancel/shutdown` → `planner_->cancelActiveSolve()` → `solve_commit_mutex_`; ordinary worker job: short worker lock → release → `backend_access_mutex_` → job | `planning_worker.hpp:105-170,177-244,268-331` |

### Reverse-edge audit and PlanningWorker-thread outcalls

- Không tìm thấy `solve_commit_mutex_ -> localization_transition_mutex_`,
  `solve_commit_mutex_ -> input_mutex_`, `solve_commit_mutex_ -> command
  transition`, hoặc `solve_commit_mutex_ -> PlanningWorker::mutex_`. Các holder
  chính là `authorizeAndStage` callback, timeline activation và candidate
  discard (`planner.cpp:818-841,876-915`); chúng không gọi lại runtime lifecycle.
- Không tìm thấy `drone_state_mutex_ -> replan_lock_`; chiều quan sát được là
  `replan_lock_ -> drone_state_mutex_` ở initial/successor
  (`planner.cpp:1850-1860,2078-2091`).
- PlanningWorker job gọi runtime helper
  `applyQueuedExecutionTimelineActivations()` rồi planner activation
  (`navigation_runtime_node.cpp:3991-4007`) sau khi worker lifecycle mutex đã
  nhả. Đây là runtime→planner, không phải planner→runtime callback.
- `runCycle()` gọi `validateRetainedCommand()` (`navigation_runtime_node.cpp:6317-6318`);
  emergency path gọi `commitEmergencyBrake()` (`:8143-8145`), giữ
  `solve_commit_mutex_` chỉ trong callback stage của `authorizeAndStage()` rồi
  trả về trước `commitPlannerCandidate()` (`:8160-8164`).
- `WorldSnapshotStore::commitIfCurrentOrUnaffected()` giữ
  `publication_gate_` rồi invoke callback; callback mới lấy
  `solve_commit_mutex_` (`world_snapshot_store.hpp:137-153`; `planner.cpp:818-841`).
  Không có runtime localization/input/command lock trong callback.
- Vì vậy verdict source-static cho cả F-04/F-05/F-06 là **NO_CYCLE**. Các cạnh
  outcall/logging/visualization vẫn có thể gây blocking/latency tail; verdict này
  không thay thế contention trace hoặc runtime deadlock test.
