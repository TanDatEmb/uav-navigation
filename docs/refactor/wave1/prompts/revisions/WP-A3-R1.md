# WP-A3-R1 — Bổ sung kịch bản thiếu, khai thác số đo có sẵn, chốt câu hỏi về khoá

> **Ghi chú baseline (2026-10-01):** các đường dẫn `artifacts/`, `runtime_evidence/`, `.artifacts/`, `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, safety index/archive và CI workflow được nhắc trong tài liệu này đã được gỡ khỏi repo. Coi chúng là nguồn gốc lịch sử, không còn kiểm chứng lại được. Xem mục "Ghi chú baseline" trong `docs/refactor/README.md`.

**Loại:** phân tích read-only, làm tiếp trên branch `refactor/WP-A3` (cập nhật PR #6). **Môi trường:** clone, Python 3.12.

## Kết quả review PR #6
Chấp nhận có điều kiện. Các finding F-01 (HG-001 40/80/180 trong ledger so với 30/60/80 trong code), F-02 (wall timer), F-04/F-05 (cancel khi đang giữ lifecycle lock) và F-10 (emergency brake chạy trên thread PlanningWorker) có giá trị và được giữ lại. Còn thiếu:
1. 12 kịch bản đã bị định nghĩa lại (S1–S12). Có 4 kịch bản bắt buộc mà `pipeline.md` không có: initial plan from rest (0 lần nhắc), measured-state emergency brake (0 lần trong `pipeline.md`), External Mode deactivate → reactivate / `mode_activation_id` (0 lần), MAIN → BACKUP activation (chỉ nhắc lướt).
2. `timing_budget.csv`: 15/18 dòng là `NOT_MEASURED`, trong khi repo có số đo. Ví dụ bảng planning p99/max và certificate aggregate max ở `docs/reports/feasible_checkpoint_5mps_diagnostic_2026-09-17.md` (khoảng dòng 200–215 trở đi), `docs/reports/execution_architecture_5mps_diagnostic_2026-09-16.md`, `artifacts/qualification_gate_recovery/*/`, `runtime_evidence/2026-09-0*/`.
3. REPORT ghi các lệnh verify là "cần chạy" chứ không có output thật. (Kiến trúc sư đã tự chạy: ledger PASS, `git diff --check` sạch, 0 file dưới `src/`. Agent vẫn phải tự chạy lại và dán output.)

## Việc cần làm
1. **Thêm 5 kịch bản** vào `pipeline.md`, dùng cùng format sequence diagram + bảng bước:
   - S13 Initial plan from rest (`kStoppedMeasuredState`).
   - S14 MAIN → BACKUP activation.
   - S15 Measured-state emergency brake: tính từ trigger trong `validateRetainedCommand` (`navigation_runtime_node.cpp:7564`) tới `commitEmergencyBrake` (`:8143`) rồi tới command publish. Ghi rõ thread, khoá đang giữ, thời gian tối đa quan sát được nếu có.
   - S16 External Mode deactivate → reactivate, gồm `mode_activation_id` và việc từ chối command của activation cũ.
   - S17 Successor renewal với committed future anchor (400 ms stitch), tách riêng khỏi S5.
   - Thêm bảng map `12 kịch bản trong prompt gốc → S-id`.
2. **Khai thác số đo.** Với mỗi dòng trong `timing_budget.csv`, tìm trong `docs/reports/**`, `artifacts/**`, `runtime_evidence/**` (grep theo tên metric: `planning`, `p99`, `certificate`, `state age`, `accepted gap`, `clock`, `mapping_total_update_us`…). Có số thì điền p50/p95/p99/max/n kèm đường dẫn nguồn và điều kiện chạy (profile, speed, commit). Không có thì ghi `NOT_MEASURED` và cột `searched` liệt kê các nguồn đã tìm. Không tính gộp các run khác điều kiện.
3. **Chốt F-04/F-05/F-06.** Với mỗi outcall-under-lock, dựng chuỗi đầy đủ để trả lời: có tồn tại một thread đang giữ khoá B (worker mutex / `solve_commit_mutex_` / `replan_lock_`) rồi cố lấy khoá A (localization / input / command transition) không? Liệt kê mọi đường planning-worker thread gọi ngược vào runtime (callback, commit, activation) trong lúc giữ khoá planner. Kết luận: `NO_CYCLE` (chứng minh bằng danh sách đường đã kiểm) hoặc `CYCLE` kèm chuỗi `file:line` cụ thể.
4. **HG-001:** không đổi gì. Chỉ bổ sung vào `findings.md` một bảng so sánh chính xác từng số giữa HG-001 và code (A* attempt, A* total, solve, future-state lead), ghi rõ scope của từng số (ví dụ "SUPER absolute" so với typed product budget) theo lineage trong `docs/safety/runtime_safety_index.md` / archive. Quyết định xem số nào là authority thuộc về chủ dự án.

## Nghiệm thu
- Có đủ S13–S17 và bảng map.
- Mỗi dòng `NOT_MEASURED` đều có cột `searched` không rỗng.
- F-04/F-05/F-06 có verdict `NO_CYCLE` hoặc `CYCLE` kèm bằng chứng.
- REPORT dán output thật của `git diff --check`, `git diff --name-only 7e0b850 -- src | wc -l` (= 0), và `validate_runtime_safety_ledger.py`.

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
