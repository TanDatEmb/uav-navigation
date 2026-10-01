# WP-A4-R1 — Làm bảng quyết định của runtime đủ mịn để dùng làm oracle cho reducer

**Loại:** phân tích read-only, làm tiếp trên branch `refactor/WP-A4` (cập nhật PR #5). **Môi trường:** clone, Python 3.12. **Kèm theo:** `SCHEMA_decision_table.md` do người giao việc đính kèm; commit nó vào `docs/refactor/SCHEMA_decision_table.md`.

## Kết quả review PR #5
Cấu trúc tốt: phủ `runCycle` liên tục 4010–7548 bằng 30 block, 40/40 hàm của `planner_fsm` đã map (kiến trúc sư xác nhận 40 là đúng; con số 38 trong prompt gốc là sai), `check_tables.py` PASS. Nhưng **bảng này chưa dùng làm oracle cho P4 được**:
1. Chỉ có 32 rule tích hợp cho `runCycle` và các callback, trong khi riêng `runCycle` có 74 câu `return`, 163 nhánh `if/else if` và 25 call site có tác dụng phụ (đếm bằng grep trên `7e0b850`; checker phải tự đếm lại).
2. `guard` viết bằng văn xuôi, ví dụ "goal non-null && mission/request/route contract valid". Không tham chiếu `var_id`, nên không test được.
3. `effects` không thuộc một bộ từ vựng đóng.
4. REPORT không có output thật của lệnh verify. (Kiến trúc sư đã chạy lại: `check_tables.py` PASS.)

## Việc cần làm
1. Chuyển toàn bộ deliverable sang schema chung: thêm `var_id`, `event_id`, `predicates.csv`, cột `exit_site`, effect lấy từ bộ từ vựng đóng.
2. **Độ mịn:** mỗi `return` trong các hàm sau là ít nhất một rule, và mỗi call site có tác dụng phụ nằm trong `source` của ít nhất một rule:
   - `runCycle`
   - `validateRetainedCommand`
   - `publishCommand`
   - `commitPlannerCandidate`
   - `onPropagatedOdometry`
   - `onRegisteredScan`
   - `onEstimatorHealth`
   - `onGoal`
   - `onModeStatus`
   - `onCommandAdmission`
   - `tickMissionProgress`
   - `applyMissionDecisionLocked`
   - `resetForLocalizationEpochLocked`
   - `failClosedLocked`
   - `schedulePlanningCycle`
   - `scheduleHeadingRebind`
   - `consumeHeadingRebind`
   - `applyQueuedExecutionTimelineActivations`
   - `suspendCommandForWorldFreshness`
3. **Guard** viết lại bằng biểu thức theo `var_id` và `payload.*`. Khi guard phụ thuộc vào kết quả của lời gọi planner/authority, mô hình hoá kết quả đó thành `payload` của một event (ví dụ `PLANNING_RESULT.payload.outcome`), không để nó thành một biểu thức ẩn.
4. Các nhánh fault injection (`inject_*`) giữ thành rule riêng. Trong guard, ghi `var_id` với `partition=fault_injection`.
5. **Ước lượng trước:** nếu tổng số rule vượt quá 400, chia file theo từng hàm (`decision_table_<function>.csv`) và gộp bằng checker.
6. Mở rộng `check_tables.py`:
   - Parse source để tìm mọi `return` và mọi call site có tác dụng phụ trong các hàm ở mục 2.
   - FAIL nếu có cái nào không được phủ.
   - FAIL nếu guard chứa identifier không có trong `state_vars.csv` / `payload` / `predicates.csv`.
   - FAIL nếu effect nằm ngoài bộ từ vựng.

## Nghiệm thu
- Checker PASS. REPORT dán output thật, gồm số `return` / call site đếm được và số đã phủ.
- `coverage_gaps.md` được cập nhật theo các rule mới, xếp theo effect `FAIL_CLOSED` / `REQUEST_PX4_HOLD` / `EMERGENCY_BRAKE_*` trước.
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

**Lưu ý cho revision:** làm tiếp trên branch hiện có của WP, commit mới (không force-push, không rewrite history), cập nhật `REPORT.md` thêm mục "R1 changes".
