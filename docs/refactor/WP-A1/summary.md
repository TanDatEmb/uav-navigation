# WP-A1-R1 — Tóm tắt inventory và map kiến trúc

Baseline được khóa tại `main@7e0b8508781f68ecbd18d15d40129e019108f3e7`; mọi
artifact là phân tích read-only trên branch `refactor/WP-A1`. Inventory bao phủ
344/344 file product và 801 symbol rows. `reference_graph.py` dùng compile
database và libclang, không dùng caller grep.

## Kết quả chính

- Graph đã đếm riêng `prod_callers_other_tu`, `prod_callers_same_tu` và
  `test_callers`; merge từ 17 package-scoped runs có 172 compile commands,
  171 TU parse được, 1 TU fast-lio lỗi parse.
- Tổng reference resolve ngoài definition scope: `5426` other-TU,
  `5697` same-TU và `15102` test references; AST thấy definition của 534/801
  rows. `confidence=high` chỉ gán cho rows có AST definition witness.
- Oracle: `MissionController` có `0/0` production callers và `DELETE`;
  `ExecutionAuthority -> nav_execution`; `CandidateBundle -> nav_plan_contract`;
  `NavigationRuntimeNode -> nav_core_node`, `SPLIT`.
- OQ-01 được đóng: file method split có 38 `NavigationRuntimeNode::` methods;
  20 free helper rows chuyển sang
  `method_split_navigation_runtime_helpers.csv`; 100 `Planner` metrics vẫn
  được giữ trong file chính.
- Polynomial/piece data và certificate result/error data được map về
  `nav_plan_contract`; validator/corridor free-function groups map về
  `nav_certifier`. `module_dependency_graph.md` báo `cycle_status=none`.
- Không sửa `src/`, không thay đổi threshold, deadline, lease, budget, UNKNOWN
  policy hay safety gate. Không có runtime/PX4/SITL qualification claim.

## Action distribution

| action | rows |
|---|---:|
| `DELETE` | 260 |
| `MOVE` | 491 |
| `KEEP` | 48 |
| `SPLIT` | 2 |
| **total** | **801** |

`DELETE` ở đây là phân loại inventory theo compiler graph và contract, không
phải lệnh xóa product. Với production count bằng zero nhưng giữ lại, rationale
ghi rõ public header contract, aggregate initialization/serialization, template
instantiation hoặc entry-point mechanism.

## Method split

`method_split_navigation_runtime_node.csv` có 138 rows: 38 method definition
thực tế của `NavigationRuntimeNode` và 100 method/inline metrics của
`navigation_planning_backend::Planner` (44 out-of-line + 56 inline/header).
20 free helper cùng translation unit nằm riêng trong
`method_split_navigation_runtime_helpers.csv`; không còn dòng `free_helper::`
trong file method chính.

## Static limitations

- Một TU của `fast_lio_core` không visit được (`test/test_base_link_state_converter.cpp`,
  `ValueError: Unknown template argument kind 154`) và `px4_odometry_bridge`
  không có compile command do CMake dừng ở thiếu `px4_msgs` submodule; các
  điểm này được giữ là evidence gap, không mặc định thành caller hay PASS.
- Graph không chứng minh plugin/linker registration hoặc runtime reachability;
  việc thực thi `DELETE` phải có review/link-map hoặc test riêng ở WP sau.
- Đây là static/diagnostic evidence; không dùng để thay thế repeated SITL,
  recorded-sensor evidence hoặc PX4 qualification.
