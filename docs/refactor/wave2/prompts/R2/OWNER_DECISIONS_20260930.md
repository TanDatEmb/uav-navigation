# Quyết định chủ dự án — 2026-09-30

**Hiệu lực:** có hiệu lực từ khi chủ dự án giao prompt R2 cho agent. File nằm trực tiếp trong thư mục dự án trên máy (không qua git/PR). Agent coi các mục dưới là quyết định có thẩm quyền và trích dẫn tự đủ theo `00_README.md` mục 0: `owner decision 2026-09-30 <ID>: <nội dung>`. Mục nào chủ dự án sửa trong file này thì theo bản sửa.

Nguồn phân tích lịch sử đã được tiêu thụ; các quyết định dưới đây là authority tự đủ cho R2.

| ID | Quyết định | WP thực hiện |
|---|---|---|
| Q-HG001 | Chấp nhận giá trị đang chạy: A* 30/60 ms, solve 80 ms; ghi vào profile + ledger | P1 PR-B |
| Q-ENV | Giữ envelope box (`√2·L`). Xét lại norm (`L`) chỉ khi có SITL evidence (P2 PR-B trở đi) | — (không đổi) |
| Q-VOX | Certify theo khoảng cách tới **voxel box**; tolerance lượng tử hoá tính trong budget 0.10 m | P3 (wave 3) |
| Q-UNK | Một seed unknown-space duy nhất, giá trị **midpoint** | P7 (wave 3) |
| Q-TRK | Runner default tracking = **`off`** (khớp YAML/product); tracking chỉ là chẩn đoán | J1b |
| Q-XTRK | Cross-track **không** là gate verdict; chỉ chẩn đoán | — (giữ nguyên) |
| P03-I2 | Giữ API ambient; I2 rời P0.3. Wave 3 (P3) thêm `planning_goal_world`, `goal_acceptance_radius_m` tường minh vào `PlanningRequest` (commit contract tách commit behavior) | P3 |
| A6-THRUST | Profile giữ scalar cấu hình `min_acc_thr`/`max_acc_thr` (m/s², đúng đơn vị loader); lực N = value × mass chỉ xuất hiện trong witness. Hàng A6 `maximum_thrust_n`/`minimum_thrust_n` đổi thành `max_thrust_acceleration_m_s2`/`min_thrust_acceleration_m_s2` kèm `derived_n` | P1 PR-A fixup |
| A6-YAWACC | Giá trị hiệu lực 2.0 rad/s² (`planner.yaml:73`). Hàng A6 trỏ về YAML. Default C++ 0.3 (`planning_limits.hpp:21`) bị xoá ở P1 PR-B | P1 PR-A fixup + P1 PR-B |
| P02-OQ01 | Phân loại SAFE/FAST theo **policy BACKUP hiệu lực** ghi trong session (`backup_evidence_experiment`), không theo tốc độ yêu cầu | P0.2-R2 |
| P02-OQ02 | Bỏ `structured_corner`; M2 báo partial (2 scene). Thêm scene = WP riêng | P0.2-R2 |
| R7-27 | `evaluation_window` áp cho **cả** error statistics và coverage | J1b |
| R7-25/26 | Giữ OPEN. Vì Q-TRK/Q-XTRK làm tracking thành chẩn đoán, số liệu tracking trong report gắn nhãn `diagnostic_frame_unverified` (J1b). Đóng khi evidence typed có vận tốc world-frame do product phát (P2 PR-B) | J1b, P2 |
| R7-17 | Giữ OPEN. Product phát mission đã resolve (behavior từng waypoint) trong evidence typed; runner/judge đọc, không tự suy | P2 PR-A (phát), P2 PR-B (đọc) |
| GATE-G1 | CTest `px4_ros2_cpp` (integration cần FMU) không chặn gate | mọi WP |
