# WP-A6 — Open questions / provenance gates

## OQ-01 — Baseline thiếu bộ tài liệu hợp đồng chung

- Classification: `CONFIRMED` (đã kiểm tra trên commit `7e0b8508781f68ecbd18d15d40129e019108f3e7`).
- Evidence: `docs/refactor/ARCHITECTURE_REVIEW.md`, `docs/refactor/adr/ADR-013.md` đến `ADR-016.md` và `docs/refactor/risk_register_20260928.md` không tồn tại trên baseline; các bản đọc tham khảo chỉ có trên `refactor/WP-D0`.
- Impact: WP-A6 không thể gắn các kết luận kiến trúc/risk vào đúng snapshot baseline mà không copy tài liệu từ branch khác.
- Options: (A) land bộ tài liệu vào `main` rồi rerun WP-A6; hoặc (B) chấp nhận chúng là context ngoài-baseline và không dùng làm source authority cho CSV/profile.
- Current handling: chỉ dùng source/YAML/Python và `runtime_safety_current.md` từ baseline; không merge/copy/cherry-pick D0.

## OQ-02 — HG-001 và profile YAML có hai authority timing

- Classification: `CONFIRMED` textual conflict; runtime effective value `NOT_MEASURED`.
- Evidence: `docs/safety/runtime_safety_current.md:91` ghi A* `40/80 ms`, solve `180 ms`, future lead `200 ms`; `src/runtime/navigation_runtime/config/planner.yaml:44,199,203` và typed source dùng `0.08/0.03/0.06 s`.
- Impact: đổi YAML X không chứng minh được gate HG-001 Y đã đổi; reserve nội bộ có thể tiếp tục dùng typed `0.08 s`.
- Options: chọn owner/units; sau đó cần effective-parameter manifest, targeted test và phân phối SITL + recorded data trước mọi thay đổi.

## OQ-03 — Thrust scalar có đúng đơn vị với source expression không

- Classification: `CONDITIONAL`.
- Evidence: `src/planning/navigation_planning/include/navigation_planning/planning_limits.hpp:22-23` dùng `6.0*1.64` và `25.0*1.64`; `src/runtime/navigation_runtime/config/planner.yaml:103-104` dùng scalar `25.0/6.0`.
- Impact: loader có thể nhận normalized thrust hoặc Newton; chưa đủ trace để chọn effective value.
- Current handling: draft giữ `CONFLICT`, không tự chọn/đổi giá trị.

## OQ-04 — Profile hiệu lực của SIM, dataset và judge

- Classification: `CONDITIONAL`; runtime identity `NOT_MEASURED`.
- Evidence: `config/runtime/sim.yaml:44-90`, `config/runtime/dataset.yaml:51-73`, `tools/runtime/runner.py:3161` và `tools/runtime/external_mode_scenario.py:1118` có override/fallback khác nhau; estimator queue/range/voxel và `vehicle_radius_m` chưa có một manifest hiệu lực chung.
- Impact: đổi YAML X có thể để judge hoặc một profile Y giữ giá trị cũ, làm coverage/clearance/denominator không so sánh được.
- Required closure: lưu effective parameter dump cùng map/profile/route/seed/build identity; thiếu dump thì kết luận qualification là `NOT_EVALUABLE`.

## OQ-05 — Phạm vi epsilon numeric dưới `1e-6`

- Classification: `CONDITIONAL`.
- Evidence: `docs/refactor/WP-A6/scan.py` phát hiện raw candidates; CSV chỉ giữ các epsilon được phân loại là roundoff, loại unit-conversion (`1e-9` seconds-to-nanoseconds), index/size và diagnostic formatting.
- Impact: nếu đưa mọi numeric literal vào profile sẽ biến false positive thành safety key; nếu lọc quá rộng sẽ bỏ sót epsilon thuộc contract.
- Required closure: owner review các occurrence nhóm `numerical_roundoff_epsilons` trước khi P1 gom thành một profile key.

## OQ-06 — Line anchor bridge `maximum_expected_speed_mps`

- Classification: `CONFIRMED` source-line split; không phải khác biệt giá trị.
- Evidence: declaration bắt đầu tại
  `src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp:56`,
  còn literal `10.0` nằm tại `:57` trong cùng `declare_parameter` statement.
- Impact: prompt nêu `:56`, trong khi occurrence witness chính xác phải ghi
  `:57`; checker nhận anchor `{56,57}` và không tạo occurrence giả tại `:56`.
- Current handling: `constants.csv` giữ `:57`; report ghi rõ line split để
  reviewer có thể đối chiếu baseline.

## OQ-07 — Phân loại static literal toàn phần

- Classification: `CONFIRMED` static coverage; runtime reachability `NOT_MEASURED`.
- Evidence: `check_constants.py` trên 13 file quyết định tìm 1.229 candidate;
  `literal_classification.csv` có 1.229 row, 20 `INCLUDED` và 1.209
  `EXCLUDED` bằng đúng các reason được R1 cho phép; `required_file_line_occurrences=47/47`.
- Impact: phân loại không chứng minh execution path, effective parameter hay
  runtime qualification; các `test_only`, `index/size` và epsilon cần owner
  review trước khi P1 đưa vào `nav_safety_profile`.
- Current handling: không sửa `src/`/`config/`; checker chỉ tạo artifact và
  fail nếu coverage, reason hoặc tên `INCLUDED` không hợp lệ.
