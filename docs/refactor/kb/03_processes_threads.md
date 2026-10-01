# KB-03: Process, thread, timer, lock, clock

## 1. Bảng process × thread
| Process | Executor | Thread | Timer (clock) | Callback group |
|---|---|---|---|---|
| **fast_lio** | SingleThreaded (`main.cpp:15`) | (1) executor: tất cả subscription và timer; (2) `fast_lio_main`: owner duy nhất của pipeline; (3) `lio_propagated`; (4) OpenMP ×3; (5) ikd-tree async rebuild | 10 ms prior match; 0.5 s diagnostics; 1 s transport snapshot (**wall**); supervise LiDAR timeout theo **wall** (R4-22) | default (R4-20) |
| **px4_external_odometry_bridge** | SingleThreaded | 1 | — | default |
| **navigation_runtime** | MultiThreaded(**2**) (`navigation_runtime_main.cpp:6`) | executor ×2 + MappingWorker + PlanningWorker + HeadingRebindWorker = **5** | planning 10 Hz, command 50 Hz, mission 20 Hz: **wall timer** (`navigation_runtime_node.cpp:1773-1783`) | default ME (scan, health, goal, mode, admission); odom Reentrant; planning ME (planning + mission); command Reentrant |
| **px4_navigation_external_mode** | spin(mode node) + node thứ hai cho state-input | main: setpoint 50 Hz (px4_ros2) + command + mission progress; state-input: odom, health, PX4 local pos; trace worker 5 ms | boundary 50 ms **wall** (`navigation_mode_node.cpp:299`); retry Hold 250 ms (steady) | 1 `trajectory_mutex_` chung cho cả 3 thread |
| **px4_odometry_bridge** (ingress) | SingleThreaded | 1 | diagnostics 500 ms **wall** | default |
| **judge** (Python) | rclpy đơn luồng (monitor, scenario) | scenario tick 50 ms; decode `/lidar/points` bằng Python trên cùng executor | stale tính theo **wall arrival** (R7-39) | — |

## 2. Lock inventory của runtime (L1–L14, lấy từ R3)
Thứ tự đã ghi trong tài liệu: ingress → localization → input → command-transition.

| # | Mutex | Bảo vệ | Việc nặng bị giữ trong lock (xấu) |
|---|---|---|---|
| L1 | `localization_epoch_ingress_mutex_` | chuyển epoch | drain mapping khi reset epoch (R3-07) |
| L2 | `localization_transition_mutex_` | epoch đang active, transaction lifecycle | publish state store trong odom callback |
| L3 | `input_mutex_` | intent, goal pending, mission | — |
| L4 | `transitionMutex` (lease latch) | mutation của ExecutionAuthority | **publish DDS NavigationCommand** (R3-14); copy 2 NavigationGoal mỗi tick (R3-13); `cancelActive` (R3-16) |
| L5 | mutex nội bộ ExecutionAuthority | timeline, pending | huỷ bundle / evaluator trong lock (R5-27) |
| L6 | `propagated_derivative_mutex_` | estimator A/J | — |
| L7 | `planner_solve_activity_mutex_` | watchdog | thứ tự L7→L2→L3→L4 trong publishCommand |
| L8 | `planner_timeline_activation_mutex_` | deque activation | — |
| L9 | `heading_rebind_mutex_` | pending heading | — |
| L10 | PlanningWorker (+ recursive backend) | job | gọi `cancelActiveSolve` ngay trong lock |
| L11–L14 | HeadingRebindWorker, PendingGoalHandoffOwner (thừa), MappingTelemetry, các store | — | — |
| ext | `Planner::solve_commit_mutex_` | staged / retained candidate | copy trajectory lúc export |

**Triple lock (L2+L3+L4)** bị mọi họ callback lấy: odom, publication mapping, khoảng 20 transaction trong mỗi solve, 3–6 lần mỗi tick command. Vì vậy **latency của command bằng thời gian giữ lock lâu nhất** trong số các bên đó.

**State có nhiều writer:**
- `trajectory_completion_witness_`, `terminal_bundle_generation_`: 4 thread;
- `last_execution_boundary_rejection_`: worker và command;
- `last_planning_timer_*`: int64 thường, không phải atomic (R3-06);
- PlannerFacade: 4 thread (R3-17).

## 3. Lock ở các process khác
- **fast_lio:** `input_mutex_`, trong đó 1 Hz sort tới 65k mẫu (R4-19); `MeasurementBuffer::mutex_`; `initial_prior_mutex_`; mutex của worker; 3 mutex của output publisher; mutex của vendor.
- **adapter:** một `trajectory_mutex_` duy nhất, được giữ cả khi publish DDS ở trạng thái terminal (R5-19). `isArmed()` được đọc chéo thread (R5-23).
- **mapping** (bên trong runtime): worker `mutex_` → `ObservationAccounting::mutex_`; `validate_` chạy **trong** mutex của worker lúc submit.

## 4. Ma trận clock domain (hiện trạng)
| Đại lượng | Clock hiện tại | Chỗ trộn clock (finding) |
|---|---|---|
| Stamp sensor / state / world | ROS time (sim) int64 ns | — |
| Timer quyết định của runtime và adapter | **wall** | freshness lại so theo ROS time; khi sim chạy chậm hoặc nhanh hơn real-time thì chu kỳ lệch |
| Timeout LiDAR của LIO | **wall steady** | R4-22 |
| Deadline solve | steady | R1-03: deadline sim được tổng hợp lại bằng sim_now + steady_remaining |
| Trajectory `start_WT` / emergency | **double giây** sim | N3, R3-09, R1 (clock domain) |
| PX4 `timestamp` / `timestamp_sample` | µs | bridge đổi ns → µs, không tự áp offset (tốt) |
| Lease adapter | ROS source + steady receive | R5-22: chỗ check trong run chỉ dùng ROS receive |
| Stale của judge | **wall arrival** | R7-39 |

## 5. Điểm nghẽn theo thread (tóm tắt; chi tiết ở KB-07 §BOTTLENECK)
1. `fast_lio_main` chạy tuần tự mọi thứ: tối đa 4 lần k-NN, chèn map inline, chờ visibility 10 ms, serialize. Correction đến propagator chậm bao nhiêu thì re-anchor chậm bấy nhiêu.
2. Mapping thread: full export khoảng 6.8 MB mỗi scan; patch gần như không bao giờ thắng (R6-09). Không có budget nào (N11).
3. Planning worker:
   - solve tối đa 80 ms;
   - sau mỗi solve dựng khoảng 400 KeyValue và JSON trước khi nhận job kế;
   - emergency và recert chạy **chung** thread này (N7);
   - SimplifySFC không có bound (R2-24).
4. Command thread: triple lock + publish DDS trong lock (R3-14) + drop tick (R3-15).
5. Judge: `_bracket` O(N²); một run 5 phút mất khoảng 100 s để đánh giá (R7-24).
