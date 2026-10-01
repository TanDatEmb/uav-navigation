# WP-H1 — Hotfix trọng tài R-01: miễn staleness chỉ trong khoảng handover

> **Ghi chú baseline (2026-10-01):** các đường dẫn `artifacts/`, `runtime_evidence/`, `.artifacts/`, `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, safety index/archive và CI workflow được nhắc trong tài liệu này đã được gỡ khỏi repo. Coi chúng là nguồn gốc lịch sử, không còn kiểm chứng lại được. Xem mục "Ghi chú baseline" trong `docs/refactor/README.md`.

**Loại:** sửa judge (tooling). KHÔNG đổi product C++. Đây là một trong hai lỗi duy nhất được phép sửa trên kiến trúc cũ, vì judge đang cho verdict sai. **Phụ thuộc:** WP-D0. **Môi trường:** Python 3.12, không cần ROS.

## Lỗi (CONFIRMED)
`tools/runtime/report.py:3513-3541`: `terminal_handover` gồm cả outcome `COMPLETE`, dẫn tới `stale_violation = 0` cho `external_odometry` và `propagated_odometry` trên **toàn bộ** active window (từ `navigation_start_wall_ns`/TRACKING tới `observation_finished_wall_ns`), chứ không chỉ đoạn handover sau khi mission kết thúc. Hậu quả: một lần mất source-time 3 s giữa mission biến mất khỏi runtime verdict. `evaluation.py` cũng không bắt được (`evaluate_localization` chỉ bỏ qua sample thiếu).

Repro gốc: `docs/refactor/risk_register_20260928.md` R-01, script `r01_report_handover_stale.py` (người giao việc đính kèm). Kết quả: outcome `''` → 2 violation; outcome `COMPLETE` → `reasons=[]`.

## Việc cần làm
1. **Tạo mốc thời gian handover cùng clock với stale event.** `stale_event_times_ns` do monitor ghi theo wall ns (`report.py:_active_stale_times`). Scenario hiện chỉ ghi `sim_time_ns` và steady (`external_mode_scenario.py:574-588`).
   - Trong `external_mode_scenario.py`: ngay tại thời điểm `self.terminal_outcome` được gán lần đầu, và tại thời điểm quan sát được mode exit / fail-closed handover, ghi `terminal_handover_wall_ns = time.time_ns()` vào event và vào `scenario.json`.
   - Tìm đúng chỗ gán; không chỉ đặt ở `finish()`, vì `finish()` có thể muộn hơn handover.
2. **`report.py`:** chỉ miễn stale event của hai stream đó khi `event_time_ns >= terminal_handover_wall_ns`. Không có mốc (session cũ, hoặc mốc không parse được) → KHÔNG miễn gì. Đây là fail-closed, không tự dùng mốc thay thế nào khác. Làm tương tự cho `fail_closed_handover`.
   - Tách logic thành hàm thuần `handover_stale_exemption(times_ns, handover_wall_ns) -> list[int]` để test được riêng.
3. **Test** (`tools/runtime/tests/test_report_handover.py`):
   - (a) COMPLETE + stall giữa mission → violation vẫn còn;
   - (b) COMPLETE + stale event sau mốc handover → được miễn;
   - (c) COMPLETE, thiếu mốc → không miễn;
   - (d) PAUSED_SAFETY_STOP / fail_closed tương tự;
   - (e) repro gốc R-01 thành regression test.
4. **Ledger:** đây là thay đổi ngữ nghĩa của judge qualification. Thêm entry vào `docs/safety/runtime_safety_current.md`, phần Recent effective changes, có đủ owner, scope, safety impact ("judge không còn che stall giữa mission; một số run COMPLETE trước đây PASS có thể thành FAIL, và đó là đúng"), evidence, removal condition (thay bằng judge P2), lệnh verify. Chạy `validate_runtime_safety_ledger.py`.
5. **Đánh giá lại dữ liệu cũ (read-only):** nếu `artifacts/baseline_20260928/` (WP-P0.2) đã tồn tại, build lại report cho các session baseline bằng cách build report chuẩn có sẵn (`tools/runtime/build_runtime_report.py` hoặc `report.py build`, xác định đúng entry point). Lập bảng `verdict_before, verdict_after` cho mỗi session vào `docs/refactor/WP-H1/verdict_delta.csv`. Không sửa session gốc: output ghi ra thư mục riêng.

## Ngoài phạm vi
R-03, R-05, R-07 (để sang judge mới ở P2). Mọi thay đổi ngưỡng stale (`stale_after_s`).

## Nghiệm thu
- Toàn bộ Python unittest pass trên clone sạch (sau WP-P0.1, hoặc chạy với skip install-tree nếu P0.1 chưa merge; ghi rõ trong REPORT).
- Test (a)–(e) pass. Cũng chính các test đó phải FAIL khi revert riêng `report.py`: chứng minh bằng cách chạy trên commit trước fix và dán output.
- Ledger validator và `git diff --check` pass.

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
