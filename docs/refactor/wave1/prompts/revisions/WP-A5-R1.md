# WP-A5-R1 — Chuẩn hoá bảng quyết định adapter/bridge theo schema chung

**Loại:** phân tích read-only, làm tiếp trên branch `refactor/WP-A5` (cập nhật PR #7). **Phụ thuộc:** file `SCHEMA_decision_table.md` (người giao việc đính kèm, hoặc lấy từ nhánh WP-A4-R1).

## Kết quả review PR #7
Chấp nhận về nội dung:
- Finding continuity gap (CONFIRMED ở component, CONDITIONAL ở tích hợp) khớp R-02.
- Xác nhận MissionController có 0 call site product.
- `setpoint_guard_inventory.csv` có ích cho `SetpointGuard` (ví dụ literal `z > 0.5 m` đã được đánh dấu pinned).

Lệch format so với WP-A4 là lỗi của prompt: hai WP chạy song song nên WP-A5 không có format của WP-A4 để theo. Giờ cần chuẩn hoá.

## Việc cần làm
1. Chuyển `state_vars.csv`, `events.csv`, `decision_table.csv` sang schema chung. Rule id dùng tiền tố `AD-` cho adapter, `BR-` cho bridge. Thêm `predicates.csv`.
2. Độ mịn: mọi `return` và mọi call site publish/latch/reseed trong các hàm sau:
   - `NavigationMode::updateSetpoint` (23 `return` trên baseline; checker phải tự đếm)
   - `onNavigationCommand`
   - `onMissionProgress`
   - callback activation/deactivation
   - `Px4ExternalOdometryBridgeNode::on_lio`
   - `on_lio_diagnostics`
   - `observe_geometric_jump_continuity`
   - `evaluate_external_odometry_gate`
3. Viết `check_tables.py` (có thể dùng lại checker của WP-A4-R1, chỉ cần tham số hoá danh sách hàm).
4. Chạy `git diff --check` và checker. Dán output thật vào REPORT; phần "before final publication" trong REPORT hiện tại phải được thay bằng output thật.
5. Giữ nguyên `setpoint_guard_inventory.csv`. Thêm cột `guard_id` để rule có thể tham chiếu tới nó.

## Nghiệm thu
Checker PASS, `git diff --check` sạch, 0 file dưới `src/`.

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
