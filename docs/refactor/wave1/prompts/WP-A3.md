# WP-A3 — Pipeline end-to-end, timing budget, thread model và các nhánh lỗi

> **Ghi chú baseline (2026-10-01):** các đường dẫn `artifacts/`, `runtime_evidence/`, `.artifacts/`, `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, safety index/archive và CI workflow được nhắc trong tài liệu này đã được gỡ khỏi repo. Coi chúng là nguồn gốc lịch sử, không còn kiểm chứng lại được. Xem mục "Ghi chú baseline" trong `docs/refactor/README.md`.

**Loại:** phân tích read-only. Đọc code và khai thác số đo có sẵn trong `artifacts/`, `runtime_evidence/`, `docs/reports/`. **Phụ thuộc:** WP-D0. Nên có WP-A2 để tham chiếu topic, nhưng có thể chạy song song.

## Mục tiêu
Mô tả chính xác từ sensor tới setpoint PX4: ai làm gì, trên thread nào, clock domain nào, deadline nào (định nghĩa ở `file:line`), kiểm identity gì, và khi lỗi thì đi nhánh nào. Đây là đầu vào để chốt lane assignment trong ADR-013 và vị trí certifier trong ADR-014.

## Deliverable
### 1. `docs/refactor/WP-A3/threads.csv`
Mỗi thread, executor, callback group, timer, worker trong 4 process product:
`process, name, kind (executor|callback_group|ros_timer|wall_timer|std_thread|jthread), period_or_trigger, clock (ros|wall|steady), created_at file:line, callbacks_run (list), locks_taken (list theo thứ tự lấy), shared_state_touched (members)`.
Phải có đủ: MultiThreadedExecutor(2) của runtime (`navigation_runtime_main.cpp:6`), planning/command/mission timer, PlanningWorker, MappingWorker, HeadingRebindWorker, propagated worker của LIO, timer 50 ms của adapter, và mọi timer của bridge.

### 2. `docs/refactor/WP-A3/lock_order.md`
Đồ thị thứ tự khoá thực tế của `navigation_runtime_node` và của `Planner` (5 mutex): mỗi cạnh `A → B` kèm `file:line` nơi B được lấy khi đang giữ A. Liệt kê mọi chỗ thứ tự ngược chiều (nếu có), mọi chỗ gọi ra ngoài (planner, ROS publish, log) trong lúc giữ khoá, và mọi chỗ unlock giữa chừng (ví dụ `localization_epoch_reset.hpp:15-27`).

### 3. `docs/refactor/WP-A3/pipeline.md`
Mỗi kịch bản là một sequence diagram Mermaid, kèm bảng các bước: `step, actor, thread, input identity checked, output, deadline/budget (value + file:line), failure disposition (file:line)`. Các kịch bản bắt buộc:
1. Chu kỳ nominal: `/lidar/points` + IMU → FAST-LIO → `/lio/mapping_observation` → MappingWorker → world snapshot → planning tick → `PlanningRequest` → solve → export → admission/commit → command sampling 50 Hz → `/navigation/navigation_command` → adapter → `/fmu/in/trajectory_setpoint`.
2. Initial plan from rest.
3. Successor renewal (committed future anchor, 400 ms stitch).
4. World revision mới → revalidate retained command (`validateRetainedCommand`, `navigation_runtime_node.cpp:7564`).
5. MAIN → BACKUP activation.
6. Measured-state emergency brake (`commitEmergencyBrake`, `navigation_runtime_node.cpp:8143`): nó chạy trên thread nào và mất bao lâu.
7. Fail-closed → PX4 Hold (cả phía runtime và phía adapter).
8. Localization epoch reset (`navigation_runtime_node.cpp:1905-1970`) và cách nó lan sang bridge/adapter.
9. Waypoint accept STOP / PASS_THROUGH, mission complete, handover.
10. External Mode deactivate → reactivate (activation id).
11. Heading rebind.
12. Luồng odometry PX4: `/lio/odometry_propagated` → bridge → `/fmu/in/vehicle_visual_odometry` → EKF2, gồm gate diagnostics và jump latch.

### 4. `docs/refactor/WP-A3/timing_budget.csv`
`stage, budget_value, unit, defined_at (file:line), enforced_at (file:line), measured_p50, p95, p99, max, n, measurement_source (đường dẫn artifact/report)`. Số nào không có trong artifact thì ghi `NOT_MEASURED`, không được ước lượng. Tối thiểu có: A* attempt/total, solve 80 ms, stitch 400 ms, commit guard, command period, command lease 100 ms, state age 200 ms, freshness 500 ms, watchdog 1 s, snapshot period, mapping update, certificate aggregate, `/clock` gap.

### 5. `docs/refactor/WP-A3/clock_domains.md`
Với mỗi so sánh thời gian trong product (freshness, lease, deadline): hai vế thuộc clock nào, và có phép trộn wall/steady với ROS time nào không. Đánh dấu mọi `create_wall_timer` trong node chạy `use_sim_time=true` (đã biết: 3 cái trong runtime, 2 cái trong adapter).

### 6. `docs/refactor/WP-A3/findings.md`
Chỉ ghi bất thường, có verdict. Tập trung vào: deadline định nghĩa ở chỗ này nhưng được enforce ở chỗ khác với giá trị khác; bước tốn thời gian chạy trên thread quyết định; đường fail không có timeout.

## Nghiệm thu
- Mọi kịch bản 1–12 đều có diagram và bảng bước.
- Mọi dòng trong `timing_budget.csv` có `defined_at`.
- Mọi thread trong `threads.csv` có `created_at`.
- Không sửa `src/`.

---

## HỢP ĐỒNG CHUNG (bắt buộc, áp dụng cho mọi work package)

**Repo:** `github.com/TanDatEmb/uav-navigation`. ROS 2 Jazzy, FAST-LIO, PX4 External Mode, beta chỉ SITL.
**Baseline:** `main @ 7e0b850`. Tạo branch `refactor/<WP-ID>` từ đúng commit này. Các nhánh khác (`codex/*`, `feat/*`) chỉ để đọc tham khảo: KHÔNG merge, KHÔNG cherry-pick.

**Đọc trước khi làm:**
1. `AGENTS.md`.
2. `docs/refactor/ARCHITECTURE_REVIEW.md`: kiến trúc hiện tại, V1–V7, RC1–RC6, tên module đích, 8 quy tắc cứng, các phase.
3. `docs/refactor/adr/ADR-013..016`.
4. `docs/refactor/risk_register_20260928.md` (R-01…R-11).
5. Chỉ khi WP đụng estimation/mapping/planning/control/PX4/threshold: đọc thêm `docs/safety/runtime_safety_current.md`.

**Ràng buộc không thương lượng:**
- KHÔNG đổi giá trị ngưỡng, deadline, lease, budget, UNKNOWN policy, tolerance, trừ khi WP ghi rõ là được phép.
- Refactor và thay đổi hành vi không bao giờ nằm chung một commit.
- Mọi khẳng định trong deliverable phải có `file:line` trên commit baseline. Khi phân loại thì dùng CONFIRMED (đã tái hiện bằng test/trace), CONDITIONAL (đường code có thật nhưng chưa chứng minh được là tới được), SPECULATIVE.
- Không suy đoán hành vi runtime khi chưa đo. Số đo nào chưa có thì ghi `NOT_MEASURED`, tuyệt đối không bịa.
- Không sửa code product nếu WP là loại phân tích (read-only).
- Khi prompt mâu thuẫn với code hoặc với tài liệu an toàn: DỪNG phần đó, ghi vào `docs/refactor/<WP-ID>/OPEN_QUESTIONS.md` (câu hỏi, bằng chứng, các phương án), làm tiếp phần còn lại, không tự quyết.

**Nơi đặt kết quả:** `docs/refactor/<WP-ID>/`. Bắt buộc có `REPORT.md` gồm:
1. Tóm tắt 5–10 dòng.
2. Danh sách deliverable kèm đường dẫn.
3. Lệnh verify đã chạy, kèm output thật (exit code, số test pass/fail).
4. Những chỗ lệch khỏi prompt và lý do.
5. Open questions.
6. Commit SHA của từng commit.

Báo cáo viết tiếng Việt; identifier, tên file và tên cột giữ tiếng Anh.

**Hoàn tất:** push branch `refactor/<WP-ID>`, mở PR vào `main` ở trạng thái **draft**, không tự merge. Tổng công trình sư sẽ review.
