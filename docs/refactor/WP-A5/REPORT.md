# WP-A5 — REPORT

## Tóm tắt

1. Review tĩnh trên source baseline `main @ 7e0b8508781f68ecbd18d15d40129e019108f3e7`; branch tiếp tục là `refactor/WP-A5`.
2. R1 chuẩn hoá `state_vars.csv`, `events.csv`, `decision_table.csv` theo schema chung; rule adapter dùng tiền tố `AD-`, bridge dùng `BR-`.
3. Bổ sung `predicates.csv` với predicate pure và liên kết 48 `guard_id` của `setpoint_guard_inventory.csv`; các giá trị guard cũ được giữ nguyên.
4. `check_tables.py` tự quét thân 11 target (9 hàm chính và 2 callback executor activation/deactivation), không hard-code số `return`; riêng `updateSetpoint` được đếm là 23 return trên baseline.
5. Các return và call site publish/latch/reseed/hold/fail-closed trong adapter/bridge đều được phủ bởi decision rules.
6. Không sửa product source; checker xác nhận `source_changes=0`, chưa có runtime/PX4/SITL qualification claim.
7. Finding continuity gap sau source-invalid/gap vẫn là CONFIRMED ở component và CONDITIONAL ở tích hợp, khớp R-02 (`geometric_jump_continuity.cpp:80-100,130-135`).

## Phạm vi và provenance

- Source baseline: `7e0b8508781f68ecbd18d15d40129e019108f3e7`.
- Nhánh: `refactor/WP-A5`; HEAD trước R1 là `0032d9f10773b1b0ce9e900dc97d40dde9671550`.
- D0 architecture/ADR/risk documents được đọc read-only từ `origin/refactor/WP-D0` tại
  `6e531de29bcfe8efd7ba62f29b8ca4ff136fd6e4`; không merge hoặc cherry-pick.
- Schema chung được lấy đúng từ nội dung SCHEMA trong prompt vì file
  `docs/refactor/SCHEMA_decision_table.md` không có trong các ref WP-A4 hiện có.
- Source anchors chính: adapter `navigation_mode_node.cpp:821-905,1143-1423,2063-2582`;
  bridge `px4_external_odometry_bridge_node.cpp:156-373`;
  continuity `geometric_jump_continuity.cpp:30-169`; gate
  `external_odometry_gate.cpp:5-49`.

## R1 changes

- Đổi header và nội dung ba bảng sang schema chung:
  `state_vars.csv`, `events.csv`, `decision_table.csv`.
- Thêm `predicates.csv`, 94 predicate rows; mọi guard trong decision table chỉ dùng
  state var, constant hoặc `pred:<name>()`.
- Giữ 48 dòng guard hiện hữu trong `setpoint_guard_inventory.csv`, thêm duy nhất
  cột `guard_id` với mã `AD_G001`–`AD_G048`.
- Thêm `check_tables.py` với danh sách hàm mục tiêu được tham số hoá trong
  `TARGETS`; checker kiểm tra rule/effect/event/source/predicate và coverage
  return/call-site.
- Cập nhật REPORT và thay placeholder before-final-publication bằng output kiểm tra
  thực tế.

## Deliverables

- `docs/refactor/WP-A5/state_vars.csv`
- `docs/refactor/WP-A5/events.csv`
- `docs/refactor/WP-A5/decision_table.csv`
- `docs/refactor/WP-A5/predicates.csv`
- `docs/refactor/WP-A5/check_tables.py`
- `docs/refactor/WP-A5/setpoint_guard_inventory.csv`
- `docs/refactor/WP-A5/update_setpoint_blocks.md`
- `docs/refactor/WP-A5/bridge_continuity.md`
- `docs/refactor/WP-A5/coverage_gaps.md`
- `docs/refactor/WP-A5/OPEN_QUESTIONS.md`
- `docs/refactor/WP-A5/REPORT.md`

## Verdict

`PARTIAL_AS_IS`: inventory và coverage static đã chuẩn hoá và machine-auditable;
đây không phải runtime, SITL, PX4 hay flight qualification evidence.

- CONFIRMED: high-water source identity và gate ordering
  (`px4_external_odometry_bridge_node.cpp:215-339`), conversion/covariance gates
  (`px4_external_odometry_bridge_node.cpp:245-283`), jump latch/gate
  (`external_odometry_gate.cpp:18-24`), adapter admission/setpoint guards
  (`navigation_mode_node.cpp:1143-1352,2063-2582`), và MissionController không có
  product call site (CMake/test references còn ở `CMakeLists.txt:25`,
  `test_mission.cpp:189+`).
- CONDITIONAL: continuity recovery seam và epoch restart policy
  (`geometric_jump_continuity.cpp:80-100,130-135`;
  `frame_generation_policy.hpp:13-20`); cần integration/producer contract.
- NOT_MEASURED: latency tails, repeated representative SITL, PX4/HIL, publication
  reachability dưới runtime timing, recorded-sensor distribution và flight acceptance.

Không thay đổi threshold, deadline, lease, budget hoặc safety gate.

## Lệnh verify và output thật

Các lệnh dưới đây được chạy trong worktree WP-A5; checker là static checker, không
phải test runtime.

    $ git rev-parse HEAD
    650138332c072afff6ccd5a7db6e3fae6c103c92

    $ git branch --show-current
    refactor/WP-A5

    $ python3 docs/refactor/WP-A5/check_tables.py
    PASS: state_vars=38 events=9 predicates=94 guards=48 rules=99 returns=54 returns_by_target={'onActivate': 0, 'onDeactivate': 0, 'executor_onActivate': 0, 'executor_onDeactivate': 0, 'onNavigationCommand': 5, 'onMissionProgress': 3, 'updateSetpoint': 23, 'on_lio_diagnostics': 2, 'on_lio': 5, 'observe_geometric_jump_continuity': 15, 'evaluate_external_odometry_gate': 1} side_effect_calls=51 side_effects_by_target={'onActivate': 1, 'onDeactivate': 1, 'executor_onActivate': 0, 'executor_onDeactivate': 0, 'onNavigationCommand': 7, 'onMissionProgress': 2, 'updateSetpoint': 24, 'on_lio_diagnostics': 1, 'on_lio': 4, 'observe_geometric_jump_continuity': 11, 'evaluate_external_odometry_gate': 0} source_changes=0
    exit code: 0

    $ git diff --check
    (no output)
    exit code: 0

    $ git diff --name-only -- src/
    (no output)
    exit code: 0

    $ git diff --cached --check
    (no output)
    exit code: 0

    $ git diff HEAD --name-only -- src/
    (no output)
    exit code: 0

    $ targeted C++/runtime tests
    NOT_RUN: no usable targeted build/test executable was present; this deliverable is read-only static analysis.

## Deviations and limits

- Schema file WP-A4 được tham chiếu trong prompt nhưng không tồn tại trong
  `origin/refactor/WP-A4`; dùng nội dung schema được cung cấp trực tiếp, không
  tự đổi format và không copy tài liệu D0 vào branch.
- Không thêm schema root file vì đây là shared input thuộc WP-A4; artifact WP-A5
  chỉ đọc schema và thực hiện revision.
- Không sửa file dưới `src/`, CMake, safety ledger, threshold, test hoặc runtime behavior.
- Không có runtime log, PX4 publication trace, SITL distribution hoặc recorded-sensor
  measurement; các khẳng định tương ứng giữ `NOT_MEASURED`.
- Các predicate là static descriptions để kiểm tra coverage; không được hiểu là
  runtime function instrumentation.

## Open questions

- OQ-02: producer/source-instance contract khi localization epoch quay về 1.
- OQ-03: emergency velocity-only handoff có chủ ý luôn handover PX4 Hold hay không.
- OQ-04: test chuỗi gap/source-invalid rồi large recovery pose để quyết định giữ
  trusted anchor hay bắt buộc generation mới.
- OQ-01 được giải quyết cho R1 bằng schema do prompt cung cấp; việc WP-A4 publish
  shared schema root vẫn là dependency quản trị, không phải quyết định hành vi.

Chi tiết bằng chứng và phương án vẫn nằm trong `OPEN_QUESTIONS.md`,
`bridge_continuity.md` và `coverage_gaps.md`.

## Commit SHA

- Source baseline: `7e0b8508781f68ecbd18d15d40129e019108f3e7`.
- Documentation snapshot trước R1: `292c8ed5`.
- Report-finalization trước R1: `0032d9f10773b1b0ce9e900dc97d40dde9671550`.
- R1 artifact commit: `650138332c072afff6ccd5a7db6e3fae6c103c92`.
- R1 report-finalization commit: final `git rev-parse HEAD` được trả trong
  handoff/PR; commit này chỉ cập nhật report provenance/output, không chứa
  product source.
