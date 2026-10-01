# WP-A4 — Bóc logic quyết định của `navigation_runtime` thành bảng State / Event / Guard / Effect

**Loại:** phân tích read-only. Đây là WP khó và quan trọng nhất trước P4. **Phụ thuộc:** WP-D0. Nên chạy sau hoặc song song với WP-A3.

## Mục tiêu
Viết đặc tả hành vi **hiện tại** của phần quyết định trong `NavigationRuntimeNode` dưới dạng bảng có thể test được. Bảng này là oracle cho P4: reducer mới phải cho cùng Effect với cùng chuỗi Event. Nhiệm vụ là mô tả trung thực, KHÔNG cải tiến. Hành vi nào trông sai thì vẫn ghi đúng như nó đang chạy, rồi đánh dấu vào cột `suspect`.

## Phạm vi code
- `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp`: tất cả callback và method. Trọng tâm là `runCycle` (4010–7548, CCN 776), `validateRetainedCommand` (7564), `publishCommand`, `onPropagatedOdometry`, `onModeStatus`, `onCommandAdmission`, `onGoal`, `commitPlannerCandidate`, `resetForLocalizationEpochLocked`, `failClosedLocked`.
- `include/navigation_runtime/planner_fsm.hpp` (38 hàm), `desired_planning_intent.hpp`, `planning_supervisor.hpp`, `baseline_refinement.hpp`, `mission_progress.*`, `trajectory_completion.hpp`, `world_temporal_assessment.hpp`, `execution_lifecycle_view.hpp`.
- `src/execution/navigation_execution/include/navigation_execution/execution_authority.hpp`, `execution_recovery_state.hpp`, `execution_lifecycle.hpp`.

## Deliverable
### 1. `docs/refactor/WP-A4/state_vars.csv`
Mọi member (của node và của các owner nó gọi) có ảnh hưởng tới một quyết định:
`name, type, owner (class), writers (file:line list), readers (file:line list), threads, guarding_lock, reset_on (epoch reset / new goal / mode exit / never), proposed_partition (execution | mission | planning_policy | ingress | world | diagnostics_only | fault_injection)`.
`proposed_partition` chỉ là đề xuất quan sát, không mang tính ràng buộc.

### 2. `docs/refactor/WP-A4/events.csv`
Mọi nguồn kích hoạt quyết định: callback của subscription, tick của timer, worker hoàn thành (planning result, mapping publication, heading rebind), deadline/timeout. Cột: `event, source (file:line), thread, payload identity fields`.

### 3. `docs/refactor/WP-A4/runcycle_blocks.md`
Chia `runCycle` thành các block liên tục, đánh số `B01..Bnn`, mỗi block ≤ ~150 dòng và có một mục đích. Mỗi block ghi: `line range`, mục đích (1 câu), state vars đọc/ghi, các early return và điều kiện của chúng, lời gọi ra ngoài (planner_, worker, publish), khoá đang giữ. Tổng các range phải phủ đúng 4010–7548, không hở, không chồng lấn.

### 4. `docs/refactor/WP-A4/decision_table.csv`
Phần chính của WP. Mỗi dòng là một quy tắc:
`rule_id, event, guard (biểu thức theo state_vars và payload, viết dạng boolean rõ ràng), effects (danh sách có thứ tự: PUBLISH_COMMAND / SUBMIT_SOLVE / CANCEL_SOLVE / COMMIT / DISCARD_CANDIDATE / FAIL_CLOSED / REQUEST_PX4_HOLD / EMERGENCY_BRAKE / ADVANCE_WAYPOINT / EMIT_DIAGNOSTIC / ... ), state_updates (var := value), source (file:line range), planner_fsm_predicates_used, covered_by_test (tên test hoặc NONE), suspect (trống hoặc mô tả)`.
- Mỗi hàm trong `planner_fsm.hpp` phải xuất hiện trong ít nhất một rule, hoặc được ghi là unused kèm bằng chứng.
- Các nhánh do fault injection (`inject_*`, 45 tham chiếu) tách riêng thành rule có `event` hoặc guard mang cờ `FAULT_INJECTION`.

### 5. `docs/refactor/WP-A4/lifecycle_enums.md`
So sánh `ExecutionPhase`, `ExecutionRecoveryState`, `ExecutionExposure`, `ExecutionSafetyOwnership`, `ExecutionRestartRequest`: liệt kê tổ hợp nào thực sự đạt được (kèm chỗ code set chúng đồng thời) và tổ hợp nào không bao giờ xảy ra. Nêu các cặp enum trùng nghĩa.

### 6. `docs/refactor/WP-A4/coverage_gaps.md`
Liệt kê rule có `covered_by_test = NONE`, xếp theo mức ảnh hưởng an toàn (rule dẫn tới FAIL_CLOSED, HOLD hay EMERGENCY xếp trước).

## Cách làm
- Đọc code; có thể dùng lizard/clang để cắt block, nhưng guard phải viết lại bằng tay từ code.
- Tên test lấy từ `src/runtime/navigation_runtime/test/*.cpp` và `src/execution/navigation_execution/test/*.cpp`. Nếu một test chỉ phủ predicate của `planner_fsm` mà không đi qua node thì ghi `predicate_only:<test>`.

## Nghiệm thu
- `runcycle_blocks.md` phủ 100% dải dòng.
- 38/38 hàm `planner_fsm` đã được map.
- Mọi rule có `source`.
- Không sửa `src/`.
- Một script nhỏ `check_tables.py` kiểm: các cột bắt buộc không rỗng, `rule_id` duy nhất, dải block không chồng.

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
