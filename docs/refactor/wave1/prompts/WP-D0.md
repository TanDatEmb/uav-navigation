# WP-D0 — Đưa tài liệu kiến trúc vào repo

**Loại:** docs-only. **Phụ thuộc:** không. **Làm đầu tiên:** mọi WP khác đều trỏ tới các file này.

## Mục tiêu
Commit nguyên văn bộ tài liệu kiến trúc do tổng công trình sư cung cấp vào `docs/refactor/`.

## Input (người giao việc sẽ đính kèm)
- `ARCHITECTURE_REVIEW.md`
- `risk_register_20260928.md`
- `adr/ADR-013-nav-core-process-topology.md`, `ADR-014-independent-certifier.md`, `ADR-015-typed-evidence-rosbag.md`, `ADR-016-beta-scope-sitl.md`

## Việc cần làm
1. Branch `refactor/WP-D0` từ `7e0b850`.
2. Copy các file vào `docs/refactor/`, giữ nguyên nội dung. ADR đặt ở `docs/refactor/adr/`, KHÔNG đặt vào `docs/adr/`: đây là ADR của chương trình refactor, và sẽ được chuyển vào `docs/adr/` khi phase tương ứng đóng.
3. Thêm `docs/refactor/README.md` (≤ 30 dòng): mục đích thư mục, danh sách WP, quy ước `docs/refactor/<WP-ID>/`.
4. Chạy `python3 tools/validate_runtime_safety_ledger.py` và `git diff --check`. Nếu validator bắt thư mục mới, KHÔNG sửa validator: ghi vào OPEN_QUESTIONS.

## Nghiệm thu
- Diff chỉ gồm file mới dưới `docs/refactor/`.
- 2 lệnh ở trên exit 0.
- `REPORT.md` có SHA.

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
