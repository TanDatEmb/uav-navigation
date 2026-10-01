# WP-A1-R1 — REPORT

## 1. Tóm tắt

- R1 thay caller grep bằng reference graph compiler-backed từ CMake
  `compile_commands.json` và libclang; không sửa `src/`.
- Inventory vẫn bao phủ 344/344 file product và 801 symbol rows.
- Caller columns mới là `prod_callers_other_tu`, `prod_callers_same_tu` và
  `test_callers`; reference trong definition scope/member body bị loại.
- Responsibility đã được viết lại theo vai trò dữ liệu/quyết định/ownership,
  không dùng câu lặp tên symbol; checker semantic hiện PASS.
- `MissionController` là `0/0` production callers và `DELETE`; các oracle
  module khác khớp yêu cầu R1.
- OQ-01 đã đóng: 38 node method definitions; 20 free helper rows tách riêng.
- `nav_plan_contract` nhận polynomial/piece và certificate data; module graph
  sau mapping có `cycle_status=none`.
- Không thay đổi threshold, deadline, lease, budget, UNKNOWN policy, tolerance,
  safety gate hoặc product behavior. Không có runtime qualification claim.

## 2. Deliverables

- [`class_inventory.csv`](class_inventory.csv): 801 semantic rows, compiler
  caller counts, responsibilities, action/rationale và target modules.
- [`reference_graph.py`](reference_graph.py): libclang AST graph builder;
  `merge_reference_graph.py` ghép các package-scoped runs.
- [`reference_graph.csv`](reference_graph.csv): graph đã merge, 801 symbol IDs.
- [`semantic_enrichment.py`](semantic_enrichment.py): semantic/action mapping
  và reviewed role overrides cho các boundary safety-critical.
- [`module_dependency_graph.py`](module_dependency_graph.py) và
  [`module_dependency_graph.md`](module_dependency_graph.md): downward-only
  target-module graph và cycle check.
- [`method_split_navigation_runtime_node.csv`](method_split_navigation_runtime_node.csv):
  38 `NavigationRuntimeNode` methods + 100 `Planner` metrics; không còn
  `free_helper::` rows.
- [`method_split_navigation_runtime_helpers.csv`](method_split_navigation_runtime_helpers.csv):
  20 helper metrics tách riêng.
- [`coverage_check.py`](coverage_check.py): file coverage, semantic, oracle,
  graph-row và cycle-status checker.
- [`inventory_builder.py`](inventory_builder.py): source census/CCN builder;
  caller columns của builder chỉ là placeholder zero cho tới khi enrichment
  bằng `reference_graph.py`.
- [`summary.md`](summary.md) và [`OPEN_QUESTIONS.md`](OPEN_QUESTIONS.md):
  verdict, limits và evidence gaps.

## 3. Phương pháp và evidence anchors

- Scope là mọi `.h/.hpp/.cpp/.cc/.cxx` dưới `src/`, bỏ external/vendor/test.
  File coverage được kiểm bằng `coverage_check.py`; definition anchor là
  `file:line` trong inventory.
- CMake Release compile database được bật bởi
  `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`. `reference_graph.py` parse từng TU
  bằng libclang và đếm `DeclRefExpr`, `TypeRef`, `MemberRefExpr`, `CallExpr`
  và `TemplateRef`; origin trong test tách sang `test_callers`.
- Definition scope được loại bằng AST extent; member body của owner class
  cũng bị loại, nên reference nội bộ không biến thành caller của chính class.
- Tổng graph sau merge: `5426` `prod_callers_other_tu`, `5697`
  `prod_callers_same_tu`, `15102` `test_callers`; AST definition witness
  `534/801` rows. `confidence=high` chỉ cho row có witness.
- Evidence anchors baseline: `MissionController` declaration
  `src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/mission_controller.hpp:51`;
  `ExecutionAuthority` declaration
  `src/execution/navigation_execution/include/navigation_execution/execution_authority.hpp:124`;
  `CandidateBundle` declaration
  `src/planning/navigation_planning/include/navigation_planning/candidate_bundle.hpp:122`;
  `NavigationRuntimeNode` declaration
  `src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp:261`.
- Certificate/piece anchors: `CertificateTubeFailure` và
  `SweptValidationResult` tại
  `src/planning/navigation_planning_backend/include/planner_core/trajectory_world_validator.hpp:10-45`;
  `TrajectoryPieceLocation` tại cùng file `:471`; `Piece` tại
  `src/planning/navigation_planning_backend/include/data_structure/base/piece.h:52`;
  corridor validation tại
  `src/planning/navigation_planning_backend/include/planner_core/corridor_plane_validation.hpp:12-77`.

## 4. Verify đã chạy — output thật

### Build/compile database

Command chính:

```text
source /opt/ros/jazzy/setup.bash && source <repo>/install/setup.bash && colcon build --symlink-install --parallel-workers 1 --packages-select navigation_mapping navigation_execution navigation_planning_backend fast_lio_ros fast_lio_tools px4_navigation_external_mode px4_odometry_bridge navigation_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers console_cohesion+
```

Output/status thật: `navigation_mapping`, `navigation_execution`,
`navigation_planning_backend`, `fast_lio_ros` và
`px4_navigation_external_mode` build được; `px4_odometry_bridge` configure
fail với CMake message `px4_msgs package not found; initialize submodule`.
Package runtime được build riêng bằng cùng `--cmake-args` và output thật là
`Summary: 1 package finished [1min 22s]`. Đây là giới hạn môi trường, không
phải product source change. Fast-LIO isolated graph output thật là
`compile_commands=58`, `translation_units_parsed=58`,
`translation_units_failed=1`, `clang_diagnostics=52`,
`references_resolved_outside_definition_scope=7327`; failed TU là
`src/estimation/fast_lio_core/test/test_base_link_state_converter.cpp` với
`ValueError: Unknown template argument kind 154`.

### Reference graph

Representative package run:

```text
/tmp/wp-a1-venv/bin/python docs/refactor/WP-A1/reference_graph.py --repo . --compile-commands build --source-prefix src/planning/navigation_planning_backend --output docs/refactor/WP-A1/.reference_graph_src_planning_navigation_planning_backend.csv --libclang /usr/lib/x86_64-linux-gnu/libclang-18.so.1
compile_commands=34
translation_units_parsed=34
translation_units_failed=0
references_resolved_outside_definition_scope=9724
```

Merge output:

```text
graphs_merged=17 symbols=801 output=docs/refactor/WP-A1/reference_graph.csv
inventory_updated=docs/refactor/WP-A1/class_inventory.csv
```

Post-merge counts: `rows=801 total_other=5426 same=5697 tests=15102 defs=534`.
Oracle output includes `MissionController: 0/0/39`,
`ExecutionAuthority: 0/0/8`, `CandidateBundle: 0/0/11` and
`NavigationRuntimeNode: 1/0/84`, where the tuple order is
`other_tu/same_tu/test`.

### Checkers and static checks

```text
modules=23 edges=21 excluded=125
cycle_status=none
product_files=344 covered_files=344 expected_symbols=801 inventory_rows=801 delta=+0.00%
semantic_rows=801
reference_graph_rows=801
semantic_failures=0
```

Additional checks: `py_compile=PASS`; method split check
`rows=138 navigation_runtime_node=38 planner=100 helpers=0`; no product source
files changed (`git diff --name-only -- src | wc -l` = `0`). `git diff --check`
was run after artifact generation and must remain empty before push. C++ unit,
ROS integration, SITL, recorded-sensor and PX4 tests: `NOT_RUN` because this
WP is static/read-only.

## 5. R1 changes

1. Added compiler reference graph and replaced legacy `prod_callers` with the
   three split columns.
2. Rewrote responsibility/action/rationale layer and expanded checker with
   forbidden generic sentence, zero-caller mechanism, oracle and cycle checks.
3. Applied contract ownership decision: polynomial/piece and certificate
   result/error data target `nav_plan_contract`; validator/corridor logic targets
   `nav_certifier`.
4. Corrected OQ-01 method split and preserved helper metrics in a separate file.
5. Added explicit build/AST evidence gaps; no product behavior was changed.

## 6. Chỗ lệch prompt và lý do

1. `px4_odometry_bridge` không có compile database usable vì checkout thiếu
   `px4_msgs` submodule; rows không có AST witness không được gán high.
2. Toàn bộ aggregate AST run được thực hiện thành 17 package-scoped processes
   để giải phóng memory libclang; `reference_graph.csv` vẫn là một graph merge
   duy nhất trên 801 IDs.
3. `method_split_navigation_runtime_node.csv` còn 100 `Planner` metrics vì
   file này vốn là method-split artifact chung; riêng 38 node definitions và
   20 helper rows đã được tách đúng theo OQ-01.

## 7. Open questions

Xem [`OPEN_QUESTIONS.md`](OPEN_QUESTIONS.md). Còn hai evidence gap cần WP sau
đóng: một TU fast-lio parse failure và runtime/plugin/linker reachability.
Không có cycle architectural nào còn lại ngoài `OPEN_QUESTIONS`.

## 8. Commit SHA

- Baseline ancestor: `7e0b8508781f68ecbd18d15d40129e019108f3e7`.
- R0 artifact commit: `a192e189` (`docs(refactor): add WP-A1 class inventory`).
- R1 artifact commit: `f3eda624` (`docs(refactor): revise WP-A1 semantic inventory`).
- R1 provenance-report follow-up: commit ngay sau artifact commit; SHA cuối cùng
  được xác nhận bằng `git rev-parse HEAD` trước khi push, không rewrite hoặc
  force-push lịch sử.
