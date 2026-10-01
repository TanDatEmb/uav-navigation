# WP-A6-R1 — Bổ sung các hằng số an toàn còn thiếu, sửa định nghĩa "pinned", thêm phân loại literal toàn phần

**Loại:** phân tích read-only, làm tiếp trên branch `refactor/WP-A6` (cập nhật PR #8). **Môi trường:** clone, Python 3.12.

## Kết quả review PR #8
Có giá trị: phát hiện HG-001 lệch với code (A* 30/60 ms, solve 80 ms trong code so với 40/80/180 trong ledger), và draft profile giữ đúng các conflict. Nhưng **thiếu những mục bắt buộc mà prompt đã liệt kê**, và một cột bị hiểu sai:
1. **Thiếu `0.15 m/s` stationary ×6**, đây là ví dụ đầu tiên trong prompt. Các vị trí trên baseline: `planning_timing.hpp:25` (`kStationarySpeedMps`), `mission.hpp:38` (`acceptance_speed_mps`, đọc từ YAML), `mission_controller.hpp:100` (`kSafetyStopSpeedMps`), `navigation_mode_node.cpp:1526-1527` (literal), `navigation_runtime_node.cpp:4499` và `:4617` (literal). Hiện tại 0.15 chỉ xuất hiện dưới row `evaluation/stop_enter_mps`, với ngữ nghĩa khác.
2. Thiếu: tốc độ trong jump gate của bridge `maximum_expected_speed_mps 10.0` (`px4_external_odometry_bridge_node.cpp:56`, `sim.yaml`) so với physical 12 m/s; `position_jump_m 0.75` và `orientation_jump_rad 0.35` của bridge phải là **các row riêng** (khác ngữ nghĩa với HG-007); `maximum_continuity_dt_s 0.5`; gate airborne `z > 0.5 m` của adapter (WP-A5 đã tìm ra); `kVisibilityAssociationMaximumAgeNs` 500 ms (`ros_output_publisher.cpp:23`); `propagated_odometry.maximum_correction_age_s 0.50`; `initial_prior.maximum_topic_prior_age_s 0.5`; `time_validator` 200 ms của `px4_odometry_bridge`; tolerance khớp anchor PVAJ (`1e-5, 1e-5, 1e-4, 1e-3, 1e-6, 1e-5`) ở cả `candidate_bundle.hpp:398-403` và `execution_anchor.hpp:47-52`; `kCommandClockToleranceS 1e-9` (`planner_fsm.hpp:459`).
3. Thiếu hằng số của judge/harness: `stale_after_s` theo từng stream trong `config/runtime/common.yaml`; `vehicle_collision_radius_m 0.35` (`runner.py:3161`, `external_mode_scenario.py:1118`); ngưỡng clearance `0.05 m` và residual `0.35 m` / `0.5 m` watchdog (`external_mode_scenario.py`); `pillar_clearance_m 0.8`; tập fault duration `{420, 430, 520, 700}` (`runner.py:2009, 3029, 3132`).
4. **`pinned` bị hiểu sai: 51/70 row đang ghi `true`.** Định nghĩa bắt buộc: `pinned=true` chỉ khi giá trị **được khai báo là cấu hình được** (ROS param / YAML) **nhưng code ép nó phải bằng một literal**, và sẽ throw hoặc reject nếu khác. Ví dụ duy nhất đã biết: `navigation_mode_node.cpp:255-258`. Tìm thêm các chỗ khác cùng dạng (so sánh `std::abs(param - literal) > eps` rồi throw) và tính lại cột này.
5. Chỉ có 70 row trên 6 292 candidate, và không có cách nào biết phần bị loại có hợp lý không.

## Việc cần làm
1. Bổ sung mọi mục ở điểm 1–3. Mỗi ý nghĩa một row, occurrence ghi đầy đủ.
2. Tính lại `pinned` theo định nghĩa ở điểm 4. REPORT liệt kê mọi row `pinned=true` kèm `file:line` của câu lệnh ép.
3. **Phân loại literal toàn phần** trên các file quyết định: `navigation_runtime_node.cpp`, `planner_fsm.hpp`, `mission_progress.cpp`, `execution_authority.hpp`, `navigation_mode_node.cpp`, `px4_tracking_adapter.hpp`, `velocity_only_continuity.hpp`, `px4_external_odometry_bridge_node.cpp`, `geometric_jump_continuity.cpp`, `timestamp_conversion.cpp`, `external_mode_scenario.py`, `report.py`, `evaluation.py`.
   - Với **mọi** literal số xuất hiện trong biểu thức so sánh, làm vế trong `std::min/max/clamp`, hoặc làm default param, xuất một dòng vào `literal_classification.csv` với các cột: `file, line, literal, context (trích 1 dòng code), decision (INCLUDED:<proposed_name> | EXCLUDED:<reason>)`.
   - `EXCLUDED` chỉ được dùng một trong các lý do: `index/size`, `unit_conversion`, `numeric_epsilon<=1e-6`, `enum_or_bitmask`, `test_only`, `diagnostic_format`.
4. Cập nhật `safety_profile_draft.yaml` và `inconsistencies.md` theo các row mới. Riêng 0.15 m/s phải có kịch bản: đổi `acceptance_speed_mps` trong mission YAML thì 5 chỗ còn lại vẫn giữ 0.15, dẫn tới hệ quả gì.

## Nghiệm thu
- Checker `check_constants.py`:
  - (a) mọi mục bắt buộc ở điểm 1–3 có row, kiểm theo danh sách `file:line` ghi trong prompt này;
  - (b) `literal_classification.csv` phủ 100% literal mà scanner tìm được trong các file ở mục 3, không dòng nào có `decision` rỗng;
  - (c) mọi `INCLUDED:<name>` trỏ tới một row có trong `constants.csv`.
- REPORT dán output thật. Không sửa `src/` và `config/`.

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
