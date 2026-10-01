# WP-A5 — updateSetpoint block map

Phạm vi chính là NavigationMode::updateSetpoint() tại
src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp:2063-2582.

Đây là inventory tĩnh trên baseline 7e0b850; không phải trace runtime và
không đo được tần suất, latency hoặc reachability thực tế.

| Block | Dòng baseline | Vai trò | Điều kiện/guard chính | Hiệu ứng khi fail |
|---|---:|---|---|---|
| B01 | 2063–2125 | Chụp snapshot dưới lock | mode state, command, odometry, health, PX4 local position, reset counters và velocity-only context | dùng snapshot nhất quán; không tự tạo setpoint |
| B02 | 2126–2146 | Chuẩn bị yaw và transform | PX4 local yaw hợp lệ; helper ENU→local NED | giữ yaw hiện tại hoặc trả failure cho nhánh cần transform |
| B03 | 2147–2200 | publishStationary() | position alignment nếu có; position/velocity/yaw finite và biểu diễn được bằng float | không gửi giá trị không hợp lệ; caller giữ safety hold |
| B04 | 2201–2240 | publishPositionHold() | position NED finite/float-representable; yaw tùy chọn | trả false, caller chuyển sang fail-closed |
| B05 | 2241–2251 | Metrics | chỉ tăng counter và ghi callback gap | observability, không mở gate |
| B06 | 2252–2292 | Terminal ownership | failure_reported_, completion receipt hoặc handover_requested_ | stationary safety hold; không đọc moving command |
| B07 | 2293–2301 | Airborne/acquisition gate | armed, odometry hiện diện, z > 0.5 m | stationary velocity hold và return |
| B08 | 2303–2323 | Odometry freshness | source stamp và receive age cùng <= state_stale_after_s_ | failNavigation() và handover PX4 Hold |
| B09 | 2324–2405 | Typed estimator health | health đã thấy; epoch khớp; source/receive fresh; healthy; chờ acquisition tối đa min(0.5 s, trajectory_wait_timeout_s_) | thiếu health trong cửa sổ thì hold; unhealthy/stale thì fail, trừ bypass SITL diagnostic được phân loại riêng |
| B10 | 2407–2424 | Command/recovery selection | command snapshot, terminal recovery identity và recovery deadline | không chọn command moving nếu không khớp |
| B11 | 2425–2445 | Command lease | receive age <= stale_after_s_; header stamp không future/invalid; commandValidAt(now) ngoài recovery | safety stop hoặc stationary hold tùy nhánh |
| B12 | 2447–2511 | Status và terminal command | REJECTED, COMPLETED, endpoint anchor 0.75 m, velocity-only terminal, bounded recovery | reject/safety hold; chỉ cho recovery cùng identity |
| B13 | 2513–2524 | Velocity-only boundary | role/status, adapter result và native hold request | không gửi setpoint; yêu cầu velocity-only Hold |
| B14 | 2526–2557 | Legacy PVA output | ENU→NED position/velocity/acceleration, yaw/yaw-rate finite và float-representable | safety stop; nếu hợp lệ mới update PX4 trajectory |
| B15 | 2559–2582 | No-command acquisition timeout | sau airborne start, command phải xuất hiện trong trajectory_wait_timeout_s_ | stationary hold trong cửa sổ; hết hạn thì safety stop |

## Liên kết với guard ngoài updateSetpoint

- publishVelocityOnlySetpoint() tại navigation_mode_node.cpp:1691-1944
  dựng tracking_adapter::AdapterData, kiểm tra identity/timestamp/frame/PX4
  reset rồi gọi tracking_adapter::adapt() (px4_tracking_adapter.hpp:351-505).
- velocity_only::limit() tại velocity_only_continuity.hpp:122-367 áp
  continuity owner, dt, giới hạn V/A/J và viability trước khi lưu previous
  state (navigation_mode_node.cpp:1917-1928).
- tryAlignPx4LocalFrameLocked() tại navigation_mode_node.cpp:1516-1541
  dùng speed 0.15 m/s, translation 2.0 m và airborne gate 0.5 m; đây là
  prerequisite cho các nhánh position hold nhưng không nằm trong thân hàm
  updateSetpoint().
- NavigationModeExecutor chạy callback định kỳ bằng wall timer tại
  navigation_mode_node.cpp:2599-2600; updateSetpoint() được gọi trong
  lifecycle này nhưng tần suất runtime không được đo ở WP-A5.

## Phân loại

Các dòng và hiệu ứng điều khiển ở trên là CONFIRMED theo source baseline.
Không có claim PASS về runtime/PX4/SITL; các cửa sổ và threshold là static
contract evidence, với giá trị NOT_MEASURED cho latency và phân phối.
