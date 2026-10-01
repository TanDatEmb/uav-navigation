# WP-A6 — các điểm không nhất quán và seams về authority

Phạm vi là các numeric candidate có `all_equal=false`, hoặc có đồng thời
YAML-loadable và literal/default cùng semantic, hoặc là parameter bị pin trong
code. Đây là inventory AS-IS; không có giá trị nào được sửa trong WP-A6.

## I-01 — A* attempt/total lệch HG-001

- Hiện trạng: `planner.yaml:199` đặt `search_time_limit_s=0.03 s` và
  `planner.yaml:203` đặt `total_time_limit_s=0.06 s`; HG-001 lại ghi lần lượt
  `0.04 s` và `0.08 s` trong `runtime_safety_current.md:91`.
- Tác động: thay YAML từ `0.03` thành `0.04` chỉ đổi planner profile; nó không
  làm runner, report hay safety table chứng minh được cùng ngân sách. Giữ
  `0.03/0.06` thì runtime nhỏ hơn contract được ghi nhưng provenance PASS vẫn
  mơ hồ.
- Scenario YAML cụ thể: một run SITL nạp `astar.search_time_limit_s: 0.04`
  nhưng `total_time_limit_s` vẫn `0.06`; attempt budget và total budget không
  còn cùng tỷ lệ, trong khi HG-001 vẫn yêu cầu `40/80 ms`.
- Cần quyết định owner của HG-001 rồi cập nhật đồng thời YAML, source/runner và
  bằng chứng phân phối; không tune từ một run.

## I-02 — solve deadline YAML/source lệch HG-001

- Hiện trạng: typed source, `planner.yaml:44` và runner đều dùng `0.08 s`,
  nhưng HG-001 ghi solve `0.18 s`.
- Tác động: nếu đổi YAML `0.08 -> 0.18`, `minimum_main_reserve_s` vẫn được
  tính từ typed `0.08`, nên có hai reserve contract trong một process.
- Scenario YAML cụ thể: deploy file có `solve_deadline_s: 0.18`; planner
  nhận 180 ms nhưng `planning_timing.hpp` vẫn suy ra reserve 0.60 s từ 80 ms,
  tạo một gate cấu hình và một gate typed khác nhau.
- Đây là conflict authority, không phải đề xuất tăng ngân sách.

## I-03 — vehicle radius có judge fallback literal (R-08)

- Hiện trạng: YAML `planner.vehicle_radius_m=0.35` (`planner.yaml:64`), C++
  default cũng `0.35` (`planner_core/config.hpp:196`), nhưng runner và
  `external_mode_scenario.py` có fallback literal `0.35`.
- Tác động: đổi YAML `0.35 -> 0.40` chỉ đổi planner/corridor; collision judge
  vẫn dùng `0.35`, có thể báo PASS cho geometry không còn tương ứng.
- Scenario YAML cụ thể: chạy `planner.vehicle_radius_m: 0.40` với cùng artifact
  runner hiện tại; bundle planner inflate thêm 5 cm nhưng judge fallback không
  đổi, nên clearance/vehicle collision là cross-layer false agreement.
- Đây là pinned parameter cần một typed profile hoặc provenance assertion trong
  runner; không đổi giá trị trong WP-A6.

## I-04 — planner route yaw-rate default và SITL override khác nhau

- Hiện trạng: backend default `1.5 rad/s` (`planner_core/config.hpp:202`),
  SITL YAML `yaw_rate_max_rad_s=2.0` (`planner.yaml:72`).
- Tác động: chạy không nạp YAML dùng envelope hẹp hơn; chạy SITL nạp YAML dùng
  envelope rộng hơn, trong khi report có thể chỉ lưu file YAML.
- Scenario YAML cụ thể: xóa key `yaw_rate_max_rad_s` khỏi profile; runtime rơi
  về `1.5` nhưng artifact vẫn ghi profile trước đó là `2.0`, làm replay không
  tái tạo được route yaw/feasibility.
- Cần ghi effective parameter dump và xác định profile nào là qualification
  authority.

## I-05 — optimizer defaults bị YAML override nhưng chưa có manifest hiệu lực

- Hiện trạng: `exp_opt_accuracy` là `1e-5` ở `traj_opt/config.hpp:187` và
  `5e-6` ở YAML `planner.yaml:112`; `integral_reso` là default `10` nhưng
  YAML là `15` (`planner.yaml:120`); `backup` lại có `12`.
- Tác động: artifact chỉ lưu source hoặc chỉ lưu YAML không đủ tái tạo số vòng
  solve và tail latency.
- Scenario YAML cụ thể: đổi `exp_traj.opt_accuracy: 5e-6 -> 1e-4`; nếu fallback
  loader silently bỏ key, backend vẫn chạy `1e-5` mà report tưởng đang chạy
  profile mới.
- Đây là solver-tuning provenance gap; chưa phải safety gate relaxation.

## I-06 — lidar queue default và profile override khác nhau

- Hiện trạng: C++ default `8 scans` (`parameter_loader.cpp:288`), cả `sim.yaml`
  và `dataset.yaml` override `16`.
- Tác động: cùng node binary có backpressure khác nhau tùy config; mất mẫu có
  thể thay đổi coverage mà không xuất hiện như threshold change.
- Scenario YAML cụ thể: chạy profile thiếu `lidar_queue_capacity`; queue giảm
  16 -> 8, trong khi report vẫn gắn cùng scenario/seed, làm denominator và
  dropped-stream evidence không so sánh ngang được.
- Cần đưa effective queue capacity vào runtime manifest; không coi capacity là
  safety PASS.

## I-07 — min range estimator default và SIM override khác nhau

- Hiện trạng: C++ default `0.10 m` (`parameter_loader.cpp:259`), SIM YAML đặt
  `0.50 m` (`sim.yaml:44`).
- Tác động: returns gần sensor bị loại ở SIM nhưng không bị loại khi dùng
  default; geometry map và CIRI evidence thay đổi.
- Scenario YAML cụ thể: đổi `min_range: 0.50 -> 0.10` để “giữ thêm điểm”; nếu
  không đổi dataset/profile và denominator, clearance/regression không còn
  cùng sensor contract.
- Đây là profile-specific estimator input, cần tách khỏi vehicle clearance.

## I-08 — scan/registration voxel và local-map extent phụ thuộc profile

- Hiện trạng: registration voxel default `0.20 m`, SIM và dataset `0.30 m`;
  scan voxel SIM `0.20 m` nhưng dataset `0.90 m`; half extent default
  `50/50/25 m`, SIM `30/30/15 m`, dataset `50/50/25 m`.
- Tác động: cùng tên mission không có cùng evidence resolution hoặc map support;
  không thể gộp phân phối latency/coverage giữa SIM và recorded dataset.
- Scenario YAML cụ thể: copy `dataset.yaml` sang SITL rồi đổi
  `scan_voxel_size_m: 0.90 -> 0.20` mà giữ report label; A* visibility và
  dropped/occupied evidence thay đổi, nhưng artifact không nói profile geometry
  mới.
- Cần profile identity trong manifest trước khi so sánh qualification.

## I-09 — thrust source expression và YAML scalar cần xác nhận đơn vị

- Hiện trạng: `planning_limits.hpp:22-23` khởi tạo `6.0*1.64` và `25.0*1.64`,
  còn YAML `planner.yaml:103-104` là scalar `25.0/6.0`.
- Tác động: nếu loader gán trực tiếp scalar vào Newton thì source và YAML khác
  nhau một hệ số mass; nếu YAML là normalized thrust thì tên/unit đang gây hiểu
  nhầm.
- Scenario YAML cụ thể: đổi `max_acc_thr: 25.0 -> 20.0`; cần biết effective
  value là `20 N` hay `20*1.64 N` trước khi đánh giá physical certificate.
- Đây là unit/effective-value question, để `NOT_EVALUABLE` cho đến khi trace
  loader và runtime dump xác nhận.

## I-10 — cùng numeric value 0.75 nhưng hai authority semantics

- Hiện trạng: command anchor error limit là `0.75 m` (`command_safety_contract.hpp:10`);
  odometry bridge position-jump default cũng `0.75 m` (`px4_external_odometry_bridge_node.cpp:53`).
- Tác động: đổi một key YAML liên quan odometry không được phép đổi command
  anchor, dù hai literal hiện bằng nhau.
- Scenario YAML cụ thể: đặt `max_position_jump_m: 0.50`; bridge sẽ reject reset
  jump ở 0.50 nhưng command consumer vẫn phải giữ anchor contract 0.75. Nếu
  refactor gộp chúng thành một key, một subsystem có thể vô tình nới gate còn lại.
- Giữ hai tên/owner riêng trong profile hợp nhất.

## I-11 — trajectory/state stale là pinned ROS parameters

- Hiện trạng: YAML external mode đặt `.10/.20 s`; node defaults lặp lại và
  `navigation_mode_node.cpp:255-256` pin chính xác hai giá trị bằng kiểm tra
  `1e-9`.
- Tác động: YAML scenario thay `trajectory_stale_after_s: 0.20` nhưng node
  vẫn reject parameter hoặc deploy fail; đây là pin có chủ đích nhưng hiện có
  nhiều authority copies.
- Scenario YAML cụ thể: đổi `.10 -> .15`; node sẽ không accept nếu check pin còn
  nguyên, trong khi report có thể chỉ ghi file YAML. Cần manifest ghi rejected
  override và effective value.
- Đây là pinned parameter, không tự ý bỏ pin trong WP-A6.

## I-12 — goal completion/connection/near-goal là aliases bị tách tên

- Hiện trạng: ba contract C++ cùng `0.20 m` (`goal_contract.hpp:12-14`), YAML
  mission có acceptance radius `0.5–1.5 m`, còn shadow planner dùng `0.20 m`.
- Tác động: đổi acceptance radius của mission không được phép đổi completion
  contract, nhưng tên “acceptance” trong tooling dễ làm người đọc gộp chúng.
- Scenario YAML cụ thể: đặt mission `acceptance_radius_m: 1.0`; mission judge có
  thể chấp nhận waypoint ở 1 m trong khi near-goal shortcut vẫn chỉ dùng 0.20 m.
- Cần giữ nhóm semantics riêng và ghi rõ effective owner trong profile.

## I-13 — safety table ghi spatial step factor, không phải map resolution

- Hiện trạng: source validator clamp temporal `0.002–0.05 s` và spatial step
  `0.5 * resolution`; YAML map resolution hiện `0.20 m`, còn HG-010 ghi `0.5`
  như factor.
- Tác động: người vận hành có thể đổi `rog_map.resolution` thành `0.5 m` do
  đọc nhầm HG-010, làm spatial validation thưa hơn bốn lần.
- Scenario YAML cụ thể: đổi `resolution: 0.2 -> 0.5`; effective spatial step
  thành `0.25 m`, không phải contract `0.5 m`. Cần tên factor/unit rõ ràng.

## I-14 — finalization reserve là YAML value nhưng qualification yêu cầu p99

- Hiện trạng: runtime profile và qualification `initial_s` cùng `0.04 s`, nhưng
  qualification ghi update `max(0.04, measured_p99_s + 0.005)` và bắt buộc SITL+
  recorded distribution.
- Tác động: sửa YAML `0.04 -> 0.02` để giảm latency không thể coi là hợp lệ nếu
  p99 vẫn lớn hơn; sửa lên một giá trị lớn hơn cũng là behavior/threshold change.
- Scenario YAML cụ thể: `finalization_reserve_s: 0.02`; planner vẫn có thể hoàn
  tất một run đơn, nhưng qualification phải giữ `NOT_EVALUABLE` cho đến khi có
  phân phối mới.

## I-15 — sáu occurrence product của `0.15 m/s` không cùng authority

- Hiện trạng: planning timing có `planning_timing.hpp:25`; mission acceptance đọc
  `mission.hpp:38`; mission controller có safety-stop ở `mission_controller.hpp:100`;
  adapter lặp literal tại `navigation_mode_node.cpp:1526-1527`; runtime recovery
  lặp tại `navigation_runtime_node.cpp:4499` và `:4617`.
- Tác động: đổi `acceptance_speed_mps` trong mission YAML chỉ đổi authority
  acceptance. Năm occurrence còn lại vẫn là `0.15 m/s`; acceptance có thể chờ/
  công nhận ở tốc độ khác trong khi adapter, safety-stop hoặc recovery vẫn dùng
  gate cũ.
- Scenario YAML cụ thể: đổi `acceptance_speed_mps: 0.15 -> 0.25`; source seam
  cho thấy mission có thể nhận giá trị mới còn năm gate kia không đổi. Hiệu ứng
  runtime thực tế `NOT_MEASURED`; verdict `CONDITIONAL`, không được gộp một key.
- Các copy harness `external_mode_scenario.yaml:13` và
  `external_mode_scenario.py:2689` là semantics SITL riêng; evaluator
  `evaluation.py:28` dùng `stop_enter_mps=0.10` cũng là semantics riêng.

## I-16 — bridge continuity có các gate riêng dù numeric gần nhau

- Hiện trạng: bridge đặt `position_jump_m=0.75` tại
  `px4_external_odometry_bridge_node.cpp:53`, `orientation_jump_rad=0.35` tại
  `:55`, `maximum_expected_speed_mps=10.0` tại `:57`, và
  `maximum_continuity_dt_s=0.5` tại `:63`; các override tương ứng nằm ở
  `config/runtime/sim.yaml:85-87` và `:90`. Physical planner envelope vẫn là
  `12 m/s` tại `planning_limits.hpp:16`/`planner.yaml:99`.
- Tác động: đây là continuity/freshness guards của bridge, không phải HG-007
  command-anchor limit `0.75 m` hay physical envelope `12 m/s`. Gộp tên/profile
  sẽ tạo đường bypass hoặc nới sai owner.
- Phân loại: `CONFIRMED` về source ownership; runtime impact `NOT_MEASURED`.

## I-17 — freshness và judge/harness không phải flight acceptance

- Hiện trạng: visibility association `500 ms` ở `ros_output_publisher.cpp:23`;
  propagated correction age `0.50 s` ở `parameter_loader.cpp:304` và
  `sim.yaml:69`; initial prior age `0.5 s` ở `parameter_loader.cpp:200`;
  PX4 time-validator stale/future `200 ms` ở `time_validator.hpp:15-16` và
  bridge node `:607`. Judge có `stale_after_s` theo từng stream ở
  `config/runtime/common.yaml:11-23`, fallback `1.0 s` ở `:6`, vehicle radius
  `0.35 m` ở `runner.py:3161`/`external_mode_scenario.py:1118`, clearance
  `0.05 m` ở `:3112`, residual p95 `0.35 m` ở `:2923`, watchdog `0.5 m` và
  `0.75 m/s` ở `:1395`, pillar clearance `0.8 m` ở `:3110`, và fault-duration
  choices `{420,430,520,700}` ở `runner.py:2009`, `:3029`, `:3132`.
- Tác động: đây là estimator freshness hoặc evaluator/harness evidence gates;
  chúng không chứng minh product runtime/PX4 safety khi đứng một mình.
- Phân loại: `CONFIRMED` về source rows; flight qualification `NOT_EVALUABLE`.

## Quy tắc xử lý

Các dòng trên không phải danh sách đề xuất tuning. Trước mọi thay đổi numeric:
giữ nguyên contract hiện tại, xác định authority, ghi effective value vào
runtime manifest, chạy targeted tests rồi repeated representative SITL và
recorded-data distribution. Nếu chưa có source-time/frame/coverage/lifecycle
witness thì kết luận an toàn là `NOT_EVALUABLE`.
