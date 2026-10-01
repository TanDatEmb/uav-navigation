# WP-A1 — Class inventory và bảng map sang kiến trúc đích

**Loại:** phân tích read-only (không sửa `src/`). **Môi trường:** không cần ROS, chỉ cần clone + Python 3.12, `lizard`. **Phụ thuộc:** WP-D0.

## Mục tiêu
Liệt kê **mọi** class/struct/enum class và mọi nhóm free function có trạng thái hoặc logic trong code product (bỏ qua `src/external/**`, `*_vendor/**`, `**/test/**`). Với mỗi mục: vai trò hiện tại, state nó sở hữu, ai dùng nó, và nó thuộc module đích nào. Đây là đầu vào để tổng công trình sư chốt vai trò từng class trong kiến trúc đích.

Quy mô tham chiếu (đếm thô bằng regex; con số chính xác do bạn đo): backend ≈146, fast_lio_core ≈114, navigation_runtime ≈73, fast_lio_ros ≈36, navigation_planning ≈34, px4_odometry_bridge ≈29, px4_navigation_external_mode ≈29, navigation_mapping ≈21, execution ≈14, world_model ≈12, mission ≈11.

## Deliverable
### 1. `docs/refactor/WP-A1/class_inventory.csv`
Mỗi dòng một symbol. Các cột, đúng thứ tự:

| cột | nội dung |
|---|---|
| `id` | `<package>::<qualified_name>` |
| `package`, `file`, `line` | nơi định nghĩa |
| `kind` | `class` \| `struct_pod` \| `struct_logic` \| `enum` \| `free_fn_group` \| `template` |
| `installed_public` | `true` nếu header nằm dưới `include/` của package và được install |
| `responsibility` | 1 câu, động từ + đối tượng |
| `state_fields` | số field thành viên có hậu tố `_` (0 cho POD) |
| `sync_primitives` | danh sách mutex/atomic/condvar |
| `threads` | các thread/callback chạm vào nó (tên callback, worker), hoặc `single` |
| `depends_on` | các id khác mà nó dùng trực tiếp (`;`-sep) |
| `prod_callers` | số call site ngoài test (grep + đọc code) |
| `test_callers` | số file test dùng nó |
| `max_ccn` | CCN lớn nhất trong các method của nó (lizard) |
| `current_tier` | L0..L4 theo bảng §1 trong ARCHITECTURE_REVIEW |
| `target_module` | một trong: `nav_core_types, nav_safety_profile, nav_world_contract, nav_plan_contract, nav_mission_contract, navigation_contracts, nav_evidence_msgs, lio_core, nav_world, nav_planner, nav_certifier, nav_execution, nav_mission, nav_planning_policy, px4_setpoint_core, odom_bridge_core, lio_node, nav_core_node, px4_adapter_node, odom_bridge_node, sitl_harness, nav_judge, vendor, DELETE` |
| `action` | `KEEP` \| `MOVE` \| `SPLIT` \| `MERGE` \| `DELETE` \| `REWRITE` |
| `rationale` | ≤ 1 câu; với SPLIT thì ghi rõ tách thành các phần nào |
| `evidence` | `file:line` cho các khẳng định chính |
| `confidence` | `high` \| `medium` \| `low` |

Quy tắc gán:
- `DELETE` khi `prod_callers == 0` và symbol không phải entry point (main, node, plugin). Kèm lệnh grep đã dùng.
- Validator an toàn (world/corridor/dynamics/swept/stop) đang nằm trong backend → `nav_certifier`, action `MOVE` hoặc `SPLIT`.
- Logic quyết định trong `NavigationRuntimeNode`/`planner_fsm.hpp` → `nav_execution` / `nav_mission` / `nav_planning_policy`, action `SPLIT`. Việc tách chi tiết thuộc WP-A4; ở đây chỉ gán module.
- Các ca không chắc chắn: `confidence=low`, đồng thời ghi vào OPEN_QUESTIONS.

### 2. `docs/refactor/WP-A1/method_split_navigation_runtime_node.csv`
Mỗi method của `NavigationRuntimeNode` (58 method) và của `Planner` (backend): `method, line_start, line_end, nloc, ccn, members_written (list), members_read (list), target_module, notes`. Lấy members read/written bằng cách đọc code: tìm các identifier có hậu tố `_` thuộc class.

### 3. `docs/refactor/WP-A1/summary.md`
- Bảng đếm theo `target_module × action`.
- Top 20 symbol cần SPLIT, sắp theo `max_ccn × state_fields`.
- Danh sách DELETE kèm bằng chứng 0 caller.
- Các cycle phụ thuộc giữa `target_module` mà mapping hiện tại sẽ tạo ra (vi phạm quy tắc "dependency chỉ đi xuống"), mỗi cycle kèm cặp symbol gây ra nó.

## Cách làm gợi ý
- `lizard -l cpp --csv <package>` để lấy CCN và NLOC.
- Liệt kê symbol bằng clang (`clang++ -Xclang -ast-dump=json -fsyntax-only`) nếu dựng được include path; nếu không thì regex + đọc tay. Ghi rõ phương pháp đã dùng trong REPORT.
- `prod_callers`: `git grep -nw <Name>` loại `/test/`, rồi đọc ngữ cảnh để bỏ các trường hợp chỉ nằm trong comment.

## Nghiệm thu
- Mọi file header/cpp product đều có ít nhất một dòng trong CSV. Kèm script `docs/refactor/WP-A1/coverage_check.py` in ra những file chưa có dòng nào; kết quả phải rỗng.
- Tổng số dòng CSV nằm trong ±10% so với số đếm của script coverage.
- Không có thay đổi nào dưới `src/`.

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
