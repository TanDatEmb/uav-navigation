# Hợp đồng tham số sản phẩm

Tài liệu này là sổ kiểm kê tham số đang được khai báo, load và sử dụng trong
stack navigation. Một tham số chỉ được giữ lại khi có đúng một owner, một
consumer có thể truy vết, đơn vị rõ ràng, kiểm tra miền giá trị và test tương
ứng. Tham số không có đủ bốn điều kiện đó phải bị xóa hoặc chuyển thành giá
trị dẫn xuất.

## Quy ước đơn vị

- Cấu hình sản phẩm dùng giây (`_s`), mét (`_m`), m/s (`_mps`), m/s²
  (`_mps2`), m/s³ (`_mps3`), rad và phần trăm/phân số có tên thể hiện rõ.
- Nanosecond/microsecond chỉ tồn tại ở biên ROS, PX4 hoặc timestamp nội bộ
  cần số nguyên. Loader phải nhận đơn vị trực quan rồi chuyển một lần ở biên.
- Không dùng số âm làm sentinel cho một tham số vật lý. Chọn `std::optional`,
  enum policy hoặc một field `enabled` riêng.
- Weight chỉ là quality objective nếu tên nằm trong `route_reference` hoặc
  `objective`; không được dùng lại để quyết định safety certificate.

## Kết quả audit hiện tại

| Khu vực | Trạng thái | Kết luận |
|---|---|---|
| `navigation_runtime` topics, frames, rates, `data_freshness_window_s` | Đang dùng | Giữ; frame và thời gian phải kiểm tra ở boundary. Đây là cửa sổ freshness chung của runtime và command lease, không phải bản sao của các ngưỡng PX4/bridge. Nếu không có mission contract, planner mặc định `RequireKnownFree`; chỉ mission hợp lệ mới có thể cấp policy MAIN `allow_unknown`. |
| `input_pair_max_skew_s` | Đã xóa | Pairing legacy thực tế yêu cầu timestamp tuyệt đối, không dùng skew cấu hình. |
| `deployment_profile` | Chọn môi trường triển khai | `sitl` là profile duy nhất hiện được phép khởi động. `hardware` bị chặn cho tới khi có artifact visibility bất biến và verifier runtime; không có boolean nào thay thế được certificate. |
| `max_safety_suffix_anchor_error_m` và PX4 `command_anchor_max_error_m` | Đã xóa tham số trùng ownership | Cả runtime và PX4 dùng `navigation_contracts::kCommandAnchorErrorLimitM`; gate hình học vẫn giữ, không còn hai bản sao hiệu chỉnh. |
| `planner_watchdog_timeout_s` | Đang dùng | Đây là watchdog runtime sau khi một solve đã bắt đầu; khác `solve_deadline_s` nội bộ là budget cấp cho planner. Tên đã thể hiện đúng vai trò, giá trị không đổi. |
| `robot_r` | Đã chuyển thành derived | Backend tính từ `vehicle_radius_m + tracking_error_budget_m + localization_error_budget_m + mapping_error_budget_m + planning_margin_m`; không còn key độc lập. Binding world geometry cũng từ chối map không đủ lớn cho planning horizon hoặc có inflation nhỏ hơn safety envelope. |
| `planning_horizon_m`, `receding_distance_m`, `corridor_bound_distance_m`, `corridor_segment_max_length_m` | Đang dùng | Một owner ở planner; hậu tố `_m` thể hiện đây là khoảng cách. Loader kiểm tra quan hệ `0 < receding_distance_m < planning_horizon_m`, corridor bao phủ safety envelope và map có đủ extent. |
| `replan_forward_dt_s`, `sample_traj_dt_s`, `yaw_rate_max_rad_s` | Đang dùng/derived | Hậu tố thể hiện giây hoặc rad/s; `sample_traj_dt_s` chỉ là giá trị dẫn xuất từ world resolution và vận tốc planner, không phải núm hiệu chỉnh độc lập. |
| `rog_map/resolution` trong planner và mapping | Trùng ownership | Chỉ mapping/world-model sở hữu resolution; planner nhận từ immutable geometry. |
| `astar/search_time_limit_s` | Trùng ownership | Planner budget là authority; `PathSearchConfig` chỉ nhận budget đã resolve, không load lại YAML. |
| `visibility_horizon_cap_m` / `visibility_horizon_floor_m` / `visibility_horizon_m` | Cap, floor và effective value | Cap/floor là cấu hình; effective horizon là derived từ braking/replan envelope và không ghi đè ngược lên cap. |
| `astar/map_voxel_num` | Đã xóa | Search window và local index stride được derive từ `WorldGeometry` của snapshot đang pin; A* không còn một bản sao kích thước map trong YAML. |
| `traj_opt/boundary/max_*` | Safety envelope | Giữ với đơn vị vật lý rõ: m/s, m/s², m/s³, rad/s và N. Mission chỉ được hạ, không được nâng; thiếu limit dùng `0` rồi bị reject, không dùng sentinel âm. |
| `flatness_gate_margin_fraction` | Đã loại bỏ | Dynamic/flatness envelope là hard certificate chính xác; objective không được phép nới giới hạn vật lý. |
| `route_reference/{lateral,vertical}_{weight,deadband_m}` | Đang dùng | Bốn field độc lập, quality-only; không làm gate và không suy ra từ `objective/position_penalty_weight`. Giá trị hiện tại là baseline provisional, chưa phải tuning evidence. |
| `corridor_plane_tolerance_m` | Safety certificate | Độc lập với penalty; certificate đánh giá cực trị liên tục của đa thức theo từng mặt phẳng. Vẫn cần evidence về conditioning/độ chính xác số và swept world certificate. |
| `vertical_guide_tolerance_m` | Đã xóa | Route reference là quality objective; corridor, dynamic, flatness và world certificate mới là các gate độc lập. Không giữ thêm một sampled guide gate có thể false-reject nghiệm vẫn nằm trong corridor. |
| `objective/{time,terminal_time,position,velocity,acceleration,jerk,waypoint_attraction,angular_rate,thrust}_*` | Đang dùng | Các weight được gom dưới `objective`, tên phản ánh đúng đại lượng và không còn scale chung. Đây là search-quality terms; hard geometric/dynamic certificates không đọc objective để authorize. Giá trị `0` là cách tắt objective duy nhất; giá trị âm/không hữu hạn bị từ chối. |
| YAML `energy_cost_type`, `scale_factor` | Đã xóa | Loader sản phẩm không đọc các key này. Code waypoint optimizer cũ phải được bóc khỏi product build hoặc chuyển vendor quarantine riêng. |
| `planner/detailed_log_en`, `backup_traj_en`, `mpc_horizon`, `yaw_mode` | Đã xóa | Không có consumer hành vi trong planner hiện tại. |
| `DUAL_PLANNING` / `--dual-planning` | Đã xóa | Runner từng nhận rồi loại bỏ cờ này trước khi load config; MAIN plus BACKUP là một product contract duy nhất, không có chế độ runtime thứ hai. |
| Mission `planning.replan_rate_hz`, `control.pass_through_lookahead_m` | Đã xóa | Parser chỉ đọc nhưng không có consumer hành vi; planner cadence do runtime sở hữu, pass-through acceptance do mission controller sở hữu. |
| Mapping ESDF/frontier/ROS callback/visualization/raycasting/virtual plane | Capability config | Nhiều khối đang disabled nhưng vẫn nằm trong product YAML; cần tách product profile khỏi capability/vendor profile, không để disabled key giả như thể là behavior contract. |
| PX4 odometry `simulation_clock`, `xrce_synchronized` | Đã xóa | Hai cờ này không điều khiển clock hay mapping. Chọn ROS clock bằng `use_sim_time`; external hardware chỉ nhận `timing.clock_domain=ros_time/system_time`, còn offset do uXRCE-DDS/PX4 sở hữu. Không áp dụng offset cục bộ lần thứ hai. |
| Estimator/PX4 profile durations | Đã chuẩn hóa ở biên | YAML/ROS profile dùng `_s` cho duration (`maximum_*_s`, `*_gap_s`, `*_age_s`, `*_history_duration_s`, `maximum_processing_lag_s`); loader dùng `navigation_common::secondsToNanoseconds()` một lần rồi core vẫn giữ ns. PX4 bridge reset/external-age keys cũng dùng `_s`; timestamp nội bộ vẫn ns. |
| Mission schema/planning limits | Đã hợp nhất owner C++ | `navigation_mission` là owner duy nhất của schema, validation, frame, waypoint, planning limits và UNKNOWN policy. Policy được parse sang enum của world-model contract, không truyền raw string qua planner. PX4 External Mode và runtime chỉ consume `navigation_mission::Mission`; Python runner/report vẫn là tooling reader, không phải runtime authority. |

## Các cặp tên gần nghĩa cần giữ một owner

| Cặp hiện tại | Phân loại | Quy tắc xử lý |
|---|---|---|
| `vehicle_radius_m` / `robot_r` | Cùng safety envelope | `vehicle_radius_m` là nguồn vật lý; `robot_r` chỉ là giá trị dẫn xuất trong planner, không có YAML key riêng. |
| `rog_map/resolution` / world-model `evidence_resolution_m` và `inflated_resolution_m` | Cùng dữ liệu map nhưng khác boundary | Mapping/world model sở hữu resolution thực tế; planner không còn load bản sao từ YAML mà bind `evidence_resolution_m` từ immutable snapshot. `inflated_resolution_m` chỉ phục vụ lớp occupancy tương ứng. |
| `visibility_horizon_cap_m` / `visibility_horizon_floor_m` | Một cap và một floor | `visibility_horizon_m` là derived; không có field cấu hình thứ ba cho effective value. |
| planner `max_vel` / mission `max_velocity_mps` | Envelope và giới hạn nhiệm vụ | Planner boundary là trần sản phẩm; mission chỉ được hạ trần và phải truyền qua `DynamicLimits`, không tạo thêm param PX4 tương đương. |
| runtime `data_freshness_window_s` / PX4 `state_stale_after_s` / bridge `external_odometry.maximum_age_s` | Không cùng clock/điểm đo | Runtime dùng một cửa sổ duy nhất cho world source, execution state và command lease; PX4 kiểm tra receive/health lease, bridge kiểm tra conversion timestamp. Không gộp các boundary này bằng cách sao chép một con số; mọi thay đổi cần phân phối đo được. |
| PX4/runtime mission parser / runner mission parser | Khác boundary, cùng schema | C++ product paths dùng `navigation_mission::loadMission()`. Python chỉ đọc YAML để orchestration/report và phải reject schema drift qua contract tests; nó không được cấp quyền planner/flight. |
| estimator `propagated_odometry.publish_rate_hz` / runtime `command_rate_hz` | Producer rate và consumer tick | Giữ độc lập; producer rate là contract dữ liệu, command rate là scheduler. Không dùng một rate để suy ra rate kia. |
| bridge `maximum_expected_speed_mps` / planner dynamic limit | Continuity-jump budget và trajectory limit | Không phải cùng safety gate; bridge value hiện có nguy cơ thấp hơn product envelope, cần chuyển sang budget continuity có provenance hoặc derive từ shared vehicle envelope trước hardware. |

## Không được thêm tham số mới nếu

1. Hành vi có thể suy ra từ state/world geometry/mission contract hiện có.
2. Tham số chỉ che một lỗi ownership, timestamp, frame, copy hoặc budget.
3. Tham số là bản sao của một gate ở lớp khác.
4. Chưa có test chứng minh nó thay đổi đúng một hành vi và không làm yếu
   certificate.

Các ngưỡng safety chỉ được hiệu chỉnh sau khi có phân phối từ SITL lặp lại và
recorded sensor data đại diện. Không dùng route-reference weight, penalty
weight hoặc một boolean enable để thay thế known-free evidence.

`config/runtime/mapping.yaml` là nguồn chuẩn duy nhất của runtime profile.
Package CMake chỉ copy file này vào build/install tree với tên
`navigation_runtime.yaml` để launch mặc định hoạt động; không duy trì thêm một
bản YAML hand-edited trong package.
