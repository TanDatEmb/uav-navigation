# R2-01: Hoàn tất WP-P0.3 (PR #16)

**Branch:** `refactor/WP-P0.3` (đã có). Không tạo commit code mới.
**Đọc:** `00_README.md` (contract v2.1), `OWNER_DECISIONS_20260930.md` (P03-I2, GATE-G1).

## Việc
1. `git fetch origin && git rebase origin/main` (main đã có H2 `74054ac`; kiểm tra trước bằng `git merge-tree` là sạch).
2. Cập nhật `docs/refactor/WP-P0.3/REPORT.md` và `OPEN_QUESTIONS.md`:
   - I2 → `DEFERRED_TO_P3` theo owner decision 2026-09-30 P03-I2 (`OWNER_DECISIONS_20260930.md`). Giữ nguyên bằng chứng I2 hiện có; không thêm API.
   - Full CTest: áp GATE-G1. Ghi kết quả 22 package PASS là gate chính; log `px4_ros2_cpp` đính kèm, nêu nguyên nhân `waitForFMU` timeout (cần FMU đang chạy, submodule pinned `4a3370f0`).
   - Bỏ câu "Do not merge until I2 is resolved".
3. Chạy lại gate v2 (static + python + ros cho package bị ảnh hưởng: `px4_navigation_external_mode`, `navigation_runtime`, `fast_lio_ros`, `navigation_planning_backend` và reverse deps) **trên head sau rebase**. Dán log.
4. Commit docs riêng (`docs(p0.3): ...`), push `--force-with-lease`, chuyển PR #16 sang **Ready for review**, PR body ghi head SHA + gate.

## Nghiệm thu
- Gate v2 PASS trên head mới (trừ `px4_ros2_cpp` theo G1).
- Bảng finding trong REPORT: I1, I3, I4 FIXED; I5 D-01/D-03 PARTIAL (docs); I2 DEFERRED_TO_P3.
- Không có thay đổi `src/` mới so với trước rebase (`git range-diff` dán vào REPORT).

Không tự merge. Báo chủ dự án khi ready.
