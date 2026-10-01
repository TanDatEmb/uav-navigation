# WP-A5 — Bóc logic quyết định của PX4 adapter và odometry bridge

**Loại:** phân tích read-only. **Phụ thuộc:** WP-D0. Chạy song song với WP-A4, dùng đúng format của WP-A4.

## Phạm vi code
- `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp` (2 703 dòng). Trọng tâm: `NavigationMode::updateSetpoint` (2063–2582, CCN 119), `onNavigationCommand`, `onMissionProgress` (1375), activation/deactivation, planner recovery episode, velocity hold, handover.
- Header: `px4_tracking_adapter.hpp` (`adapt` CCN 74), `velocity_only_continuity.hpp` (`limit` CCN 47), `command_admission_assessment.hpp`, `certified_command_handoff.hpp`, `mission_command_identity.hpp`, `navigation_mode.hpp`.
- `src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp` (`on_lio`, `on_lio_diagnostics`), `geometric_jump_continuity.cpp`, `external_odometry_gate.cpp`, `timestamp_conversion.cpp`, `frame_generation_policy.hpp`.
- KHÔNG phân tích `mission_controller.*`: đây là dead code, sẽ bị xoá ở WP-P0.3. Chỉ cần xác nhận lại là 0 call site product.

## Deliverable (dưới `docs/refactor/WP-A5/`)
- `state_vars.csv`, `events.csv`, `decision_table.csv`, `coverage_gaps.md`: format y hệt WP-A4.
- `update_setpoint_blocks.md`: chia `updateSetpoint` thành các block giống cách làm `runCycle` ở WP-A4.
- `setpoint_guard_inventory.csv`: mọi kiểm tra mà adapter làm trước khi gửi setpoint sang PX4. Cột: `check, input, limit_value, limit_defined_at (file:line), pinned_literal (bool), failure_effect`. Đây là đầu vào cho `SetpointGuard` của ADR-014.
- `bridge_continuity.md`: mô tả state machine continuity/jump latch/high-water của bridge. Kèm kịch bản R-02 (jump sau gap >0.5 s, jump sau source_invalid) chỉ ra đúng các dòng cho phép pose nhảy lọt qua. Kèm kịch bản LIO restart (epoch quay về 1) ở `strictly_newer_source_identity` (`frame_generation_policy.hpp:13-20`).

## Nghiệm thu
Giống WP-A4, cộng thêm: mọi literal thời gian/khoảng cách trong `updateSetpoint` và bridge đều có mặt trong `setpoint_guard_inventory.csv` hoặc trong `state_vars.csv`.

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
