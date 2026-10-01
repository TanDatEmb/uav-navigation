# WP-A1-R1 — Open questions và evidence gaps

## OQ-01 — 38 definition versus prompt count 58 — RESOLVED

- Baseline `7e0b850` có 38 definition của `NavigationRuntimeNode::` trong
  `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp`.
- 20 free helper cùng translation unit không phải member method; chúng được
  tách sang `method_split_navigation_runtime_helpers.csv`.
- File method chính giữ 38 node methods và 100 `Planner` metrics. Không dùng
  58 như một definition count.

## OQ-02 — Compiler graph completeness — CONDITIONAL

- `reference_graph.py` đã chạy bằng libclang từ compile database Release, theo
  17 package-scoped runs: 172 compile commands, 171 TU parse được, 1 TU
  `fast_lio_core` parse/visit lỗi (`test/test_base_link_state_converter.cpp`,
  `ValueError: Unknown template argument kind 154`, 52 clang diagnostics).
- `px4_odometry_bridge` không có compile command vì CMake dừng ở thiếu
  `px4_msgs` submodule. Các row tương ứng giữ `definition_seen_in_ast=false`
  khi không có witness; không tự động gán PASS hay `confidence=high`.
- Macro/plugin/linker registration không phải reference expression trong graph.
  WP sau phải xác nhận các entry point đó bằng link map hoặc registration trace
  trước khi thực thi một `DELETE`.

## OQ-03 — Production count bằng zero không tự chứng minh dead code — OPEN

- R1 đã thay grep bằng `DeclRefExpr`, `TypeRef`, `MemberRefExpr`, `CallExpr` và
  `TemplateRef`, đồng thời loại reference trong definition scope và member body.
- `DELETE` vẫn chỉ là action phân tích. Các row giữ lại production-zero có
  rationale nêu rõ public header contract, aggregate initialization,
  serialization, template instantiation hoặc entry point mechanism.
- Không có product code nào bị xóa trong WP-A1-R1.

## OQ-04 — `nav_planner`/`nav_certifier` cycle — RESOLVED IN MAPPING

- `Piece`, `TrajectoryPieceLocation`, polynomial trajectory representations và
  certificate result/error data (`StopFailureReason`, `CertificateTubeFailure`,
  `SweptValidationResult::Failure` và tương tự) được map về
  `nav_plan_contract` như data.
- Validator/corridor free-function groups được map về `nav_certifier`.
- `module_dependency_graph.md` được sinh lại sau mapping và có
  `cycle_status=none`. Việc di chuyển source thật là scope của WP tiếp theo;
  graph này không tuyên bố product đã migrate.

## OQ-05 — Runtime qualification — NOT_EVALUATED

- WP này read-only, không chạy ROS integration, SITL, recorded sensor replay
  hay PX4 flight acceptance.
- CCN/NLOC, compiler reference graph và module mapping chỉ là static/diagnostic
  evidence; không thay thế phân phối SITL, source-time/frame witness,
  certificate lifecycle hay runtime safety ledger.
