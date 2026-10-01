# R2-04: Hoàn tất WP-P1 PR-A (PR #17)

**Bắt đầu khi:** #16 đã merge. **Branch:** `refactor/WP-P1`.
**Đọc:** `WP-P1.md`, `OWNER_DECISIONS_20260930.md` (A6-THRUST, A6-YAWACC, GATE-G1), contract v2.1.

## Việc
1. **Rebase** lên `origin/main` (đã có H2 + P0.3).
   - Conflict `src/px4/px4_navigation_external_mode/CMakeLists.txt` với P0.3 (P0.3 xoá `mission_controller.cpp` và test; P1 thêm dep `nav_safety_profile`): giữ cả hai — xoá các dòng mission_controller, giữ dep/link profile.
   - `px4_external_odometry_bridge_node.cpp`: giữ cả dòng H2 (`geometric_sample_admissible`) và phần profile/witness của P1.
   - Không cherry-pick; không đổi logic.
2. **A6 oracle** (commit docs/oracle riêng, không đổi giá trị hiệu lực):
   - A6-THRUST: đổi 2 hàng thành `max_thrust_acceleration_m_s2` = 25.0, `min_thrust_acceleration_m_s2` = 6.0 (nguồn `planner.yaml:103-104`), thêm cột/ghi chú `derived_n = value × mass`. Profile đưa hai key này ra khỏi `open_questions`; witness in thêm giá trị N suy ra.
   - A6-YAWACC: hàng `max_yaw_acceleration_rad_s2` = 2.0, nguồn `planner.yaml:73` (loader `config.hpp:203-204`). Ghi `planning_limits.hpp:21` (0.3) là default không hiệu lực, sẽ xoá ở PR-B.
   - `python3 tools/safety_profile/check_against_a6.py` → `open_questions=0`, `failures=0`.
3. **Gate v2** đủ 3 mức trên head sau rebase (ros: 5 process + reverse deps, G1 cho `px4_ros2_cpp`).
4. **Witness run + A/B chống regression** (máy SITL, tuần tự với P0.2):
   - Nhánh A: 3 run trên `2543b0b4`, dùng lại 3 run M2 của P0.2 nếu trùng đúng scene/tốc độ/tracking `off`; nếu không trùng thì tự chạy 3 run.
   - Nhánh B: 3 run trên head P1 (sau rebase), cùng scene/tốc độ/tracking `off`.
   - Bảng: run → verdict, terminal outcome, waypoint accepted, `infrastructure_invalid`, lỗi capture. Đạt nếu số run PASS của B ≥ A và không có lớp lỗi mới chỉ xuất hiện ở B. Không đạt → DỪNG, phân tích (systematic-debugging), không sửa ngưỡng.
   - Cả 5 startup witness `mismatches=[]` trên mỗi run B.
5. Cập nhật REPORT (bảng finding, SHA, A/B), push `--force-with-lease`, PR #17 → **Ready for review**.

## Nghiệm thu
Gate v2 PASS; `check_against_a6.py` PASS; A/B đạt; không có thay đổi effective value (so witness hash trước/sau, giải thích nếu hash đổi do thêm key thrust).
