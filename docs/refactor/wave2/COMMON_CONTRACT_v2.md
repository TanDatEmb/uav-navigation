# HỢP ĐỒNG CHUNG v2 (áp dụng cho mọi work package từ đợt 2)

Bản này thay "HỢP ĐỒNG CHUNG" ở cuối các prompt đợt 1. Những điểm khác biệt được đánh dấu **[v2]**.

**Repo:** `github.com/TanDatEmb/uav-navigation`. ROS 2 Jazzy, FAST-LIO, PX4 External Mode. Phạm vi beta: chỉ SITL.

**Baseline [v2]:**
- Tạo branch `refactor/<WP-ID>` từ `origin/main` **sau khi** các PR sau đã merge:
  - #3 D0, #10 P0.1, #4 H1;
  - #9 A1, #12 A2, #6 A3, #5 A4, #7 A5, #8 A6;
  - commit docs `docs/kb-wave2` (KB, design, ADR-017 rev 1, wave2).
- Ghi SHA baseline vào REPORT.
- Bắt buộc kiểm `git diff 7e0b850 <baseline> -- src config` và dán output vào REPORT. Output này phải rỗng; nếu không rỗng thì DỪNG và hỏi.
- Các nhánh khác (`codex/*`, `feat/*`, `refactor/WP-A4-codex`) chỉ để đọc tham khảo. KHÔNG merge, KHÔNG cherry-pick.

**Đọc trước khi làm [v2]:**
1. `AGENTS.md`.
2. `docs/refactor/kb/00_INDEX.md`, `08_project_rules.md` (51 quy tắc; PR phải nêu quy tắc nào áp dụng), và phần của KB liên quan tới WP.
3. `docs/refactor/adr/ADR-013..017`. Riêng ADR-017 đọc bản rev 1, bao gồm §6 Amendment 1.
4. `docs/refactor/design/A1..A5` phần liên quan.
5. `docs/refactor/kb/data/findings.csv`: đọc các ID mà WP được giao. Khi đọc phải kiểm lại `location` và `excerpt` trên baseline. Nếu code không còn giống excerpt thì DỪNG hạng mục đó.
6. Nếu WP chạm estimation, mapping, planning, control, PX4 hoặc threshold: đọc `docs/safety/runtime_safety_current.md`.

**Ràng buộc không thương lượng:**
- KHÔNG đổi giá trị ngưỡng, deadline, lease, budget, UNKNOWN policy hay tolerance, trừ khi WP ghi rõ là được phép. Các mục trong ADR-017 §6.3 là quyết định của chủ dự án; agent không tự chọn.
- Refactor và behavior change không bao giờ nằm chung một commit.
- Mỗi commit behavior phải có:
  - (a) test RED chạy trước khi sửa, có output dán vào REPORT;
  - (b) entry ledger trong `docs/safety/runtime_safety_current.md` mục "Recent effective changes" (Lifecycle / Implementation / Evidence / Authority);
  - (c) kết quả `python3 tools/validate_runtime_safety_ledger.py`.
- Mọi khẳng định phải có `file:line` trên baseline, gắn nhãn CONFIRMED / CONDITIONAL / SPECULATIVE.
- Số đo nào chưa có thì ghi `NOT_MEASURED`; không bịa số.
- Prompt mâu thuẫn với code hoặc tài liệu an toàn: DỪNG phần đó, ghi `docs/refactor/<WP-ID>/OPEN_QUESTIONS.md` (câu hỏi, bằng chứng, các phương án), làm tiếp phần còn lại.

**Gate verify [v2]** (thay `make ci-local` khi không có container runtime hoặc không pull được image). Chạy **native trên đúng head SHA**, dán log vào REPORT và PR:
1. **static:**
   - `git diff --check -- . ':!artifacts/**'`
   - 10 guard `tools/check_*.py`, theo danh sách trong `tools/ci/local_ci.sh`
   - `python3 tools/validate_runtime_safety_ledger.py`
2. **python:** `python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py'`, ghi rõ phiên bản Python.
3. **ros:** chỉ khi WP chạm `src/`, CMake hoặc msg. Chạy colcon build Release cho các package bị ảnh hưởng và toàn bộ reverse dependency, rồi chạy CTest của chúng. Nếu có máy chạy đủ 23 package thì chạy đủ.
Nếu có Docker và pull được image thì chạy `make ci-local` và coi đó là bằng chứng chính.

**Nơi đặt kết quả:** `docs/refactor/<WP-ID>/REPORT.md` gồm:
1. Tóm tắt 5–10 dòng.
2. Deliverable kèm đường dẫn.
3. Lệnh verify kèm output thật.
4. Chỗ lệch khỏi prompt và lý do.
5. Open questions.
6. SHA từng commit.
7. **[v2]** Bảng `finding ID → trạng thái (FIXED / PARTIAL / NOT_FIXED + lý do) → commit`.

Báo cáo viết tiếng Việt; identifier, tên file và tên cột giữ tiếng Anh.

**Hoàn tất:** push `refactor/<WP-ID>`, mở PR draft vào `main`, không tự merge.
