# WP-A1-R1 — Sửa class inventory: từ phủ cơ học sang phân tích vai trò thật

**Loại:** phân tích read-only, làm tiếp trên branch `refactor/WP-A1` (cập nhật PR #9). **Môi trường:** máy có ROS 2 Jazzy và build Release của WP-P0.1, vì cần `compile_commands.json`. **Phụ thuộc:** WP-P0.1 đã merge, hoặc checkout được nhánh đó để build.

## Kết quả review PR #9 (tổng công trình sư)
Phần phủ file đạt yêu cầu: 344/344 file, 801 row, `coverage_check` PASS. Nhưng **nội dung ngữ nghĩa chưa dùng được**:
1. `responsibility` được sinh theo khuôn, ví dụ "Stores MissionController.", "Generates Planner.", "Coordinates MappingActor.". Câu chỉ lặp lại tên symbol nên không mô tả được vai trò.
2. `prod_callers` sai phương pháp: đếm cả tham chiếu trong chính file định nghĩa. Oracle đã biết: `MissionController` có **0 call site product** (WP-A5 xác nhận, và `docs/safety/mission_authority_cut.md` cũng ghi vậy), nhưng inventory ghi `prod_callers=26, action=MOVE → nav_mission`.
3. Có 201 row `prod_callers=0` mà không phải DELETE và không có lý do.
4. 753/801 row là `MOVE` với rationale chung chung. `confidence=high` cho 579 row trong khi phương pháp chỉ là regex.
5. OQ-01 (38 vs 58): kiến trúc sư xác nhận **38 definition là đúng**. Con số 58 đếm cả khai báo trong header. Bỏ 20 dòng `free_helper::` khỏi file method-split, hoặc để ở một file riêng.
6. OQ-04 cycle `nav_planner ↔ nav_certifier` qua `Piece` / `StopFailureReason`: **quyết định:** kiểu biểu diễn đa thức và piece của trajectory (`Piece`, `TrajectoryPieceLocation` và tương tự), cùng mọi enum kết quả/lỗi của certificate (`StopFailureReason`, `CertificateTubeFailure`, `SweptValidationResult::Failure`…), đều chuyển vào `nav_plan_contract` dưới dạng dữ liệu. Cả `nav_planner` và `nav_certifier` phụ thuộc vào đó, nên cycle biến mất. Áp quyết định này vào mapping.

## Việc cần làm
1. **Reference graph bằng compiler.** Build với `--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`. Dùng libclang (Python `clang.cindex`) hoặc `clang-query` để đếm, cho mỗi symbol, số tham chiếu (`DeclRefExpr`, `TypeRef`, `MemberRefExpr`, `CallExpr`) **ngoài phạm vi định nghĩa của chính nó** (ngoài thân class và các method của class đó), tách thành: `prod_callers_other_tu`, `prod_callers_same_tu`, `test_callers`. Dùng các cột này thay cho `prod_callers` cũ. Ghi phương pháp vào REPORT, kèm script `reference_graph.py`.
2. **Responsibility viết tay** cho mọi row có `kind ∈ {class, struct_logic, free_fn_group, template}` (khoảng vài trăm row). Yêu cầu: đọc code, viết 1 câu nêu *nó quyết định / tính / sở hữu cái gì, cho ai*, KHÔNG nhắc lại tên symbol. Với `struct_pod` / `enum`: nêu ≥ 2 field hoặc giá trị chính và ai tạo ra nó.
3. **Action/rationale:**
   - `prod_callers_other_tu + prod_callers_same_tu == 0`, không phải entry point → `DELETE`; hoặc giữ lại với rationale nêu cơ chế cụ thể (template instantiation, macro registration, ADL…) và bằng chứng.
   - `SPLIT`: liệt kê các phần tách ra, và mỗi phần đi về `target_module` nào. Ví dụ `NavigationRuntimeNode` → `nav_core_node` (shell) + `nav_execution` + `nav_mission` + `nav_planning_policy` + `nav_world` (service) + `sitl_harness` (phần fault injection).
   - Mọi `target_module` của `NavigationRuntimeNode` phải là `nav_core_node`, action `SPLIT`.
   - `confidence=high` chỉ dùng khi đã có compiler reference graph và đã đọc code.
4. **Cycle check lại** sau khi áp quyết định ở mục 6 phần review, cho ra `module_dependency_graph.md`: các cạnh giữa `target_module`, đã loại bỏ mọi vi phạm "dependency chỉ đi xuống" theo ARCHITECTURE_REVIEW §2. Cycle nào còn lại thì liệt kê kèm symbol gây ra.

## Nghiệm thu (checker `coverage_check.py` phải mở rộng để kiểm tất cả điều sau)
- Oracle: `MissionController` → `prod_callers_other_tu=0`, `action=DELETE`. `ExecutionAuthority` → `nav_execution`. File `trajectory_world_validator.hpp` và `corridor_plane_validation.hpp` → `nav_certifier`. `CandidateBundle` → `nav_plan_contract`.
- Không row nào có `responsibility` chứa nguyên văn tên symbol ở cuối câu, hoặc khớp regex `^(Stores|Generates|Coordinates|Classifies|Carries|Implements stateful or decision logic in) \S+\.?$`.
- Không còn row `prod_callers == 0` nào mà không phải DELETE và không có rationale nêu cơ chế.
- `module_dependency_graph.md` không có cycle, hoặc mọi cycle đều nằm trong OPEN_QUESTIONS.
- REPORT dán output thật của checker và của `reference_graph.py`.

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

**Lưu ý cho revision:** làm tiếp trên branch hiện có của WP, commit mới (không force-push, không rewrite history), cập nhật `REPORT.md` thêm mục "R1 changes".
