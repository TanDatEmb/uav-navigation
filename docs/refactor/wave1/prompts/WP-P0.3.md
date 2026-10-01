# WP-P0.3 — Xoá dead code và sửa doc drift (không đổi hành vi)

**Loại:** refactor thuần. Hành vi runtime phải giữ nguyên. **Phụ thuộc:** WP-P0.1: phải có `make ci-local` xanh trước khi bắt đầu. **Môi trường:** ROS 2 Jazzy để build/test.

## Nguyên tắc
- Mỗi hạng mục dưới đây là **một commit riêng**, message dạng `refactor(p0.3/<item>): ...`.
- Trước khi xoá bất kỳ symbol nào: chạy lại `git grep -nw <symbol> -- src tools` và dán output vào REPORT. Kết quả phải chứng minh 0 call site product. Nếu kết quả khác với prompt thì DỪNG hạng mục đó và ghi OPEN_QUESTIONS.
- Nếu một test được port mà cho kết quả khác test gốc: KHÔNG chỉnh assertion cho khớp. Revert phần port đó và ghi vào OPEN_QUESTIONS (đó là dấu hiệu legacy API có hành vi khác `plan(request)`, cần tổng công trình sư xem).

## Hạng mục
### I1. Xoá `MissionController` legacy
- File: `src/px4/px4_navigation_external_mode/src/mission_controller.cpp` (752 dòng), `include/.../mission_controller.hpp`, target `${PROJECT_NAME}_contract` trong `CMakeLists.txt:24-31, 53`, export liên quan.
- `test/test_mission.cpp` có 45 test. Các test `MissionLoader.*` kiểm `navigation_mission::loadMission`, nên **chuyển** chúng sang `src/contracts/navigation_mission/test/` (giữ nguyên nội dung assertion). Chỉ xoá các test đang kiểm `MissionController`.
- Comment nhắc tới MissionController trong `certified_command_handoff.hpp`, `mission_command_identity.hpp`, `planner_fsm.hpp:605`, `planner.cpp:3510`, `navigation_runtime_node.cpp:5330`: sửa cho đúng owner hiện tại (`navigation_runtime::MissionProgress`, theo `docs/safety/mission_authority_cut.md`).
- `tools/check_mission_authority_cut.py` hiện cấm `MissionController` xuất hiện lại. Giữ lệnh cấm đó, và cập nhật những đường dẫn file không còn tồn tại.
- Cập nhật `docs/safety/mission_authority_cut.md` ("The legacy MissionController library/API remains compiled for compatibility…") thành "removed in WP-P0.3". Thêm entry trong phần Recent effective changes của `runtime_safety_current.md`: Lifecycle ACTIVE, Implementation IMPLEMENTED, Evidence UNIT_VERIFIED, Authority PRODUCT, mô tả "legacy library removed; no call path change". Sau đó chạy `validate_runtime_safety_ledger.py`.

### I2. Thu gọn API ambient của `PlannerFacade` / `Planner`
- Xoá khỏi public API của `planner_facade.hpp`: `setCommandIdentity`, `setWorldModelView`, `setGoalAcceptanceRadius`, `setRouteSnapshot`, `setMissionStartPosition`, `stageImmediateHeadingRebind`, `setState`, `planInitialFromStoppedState`, `planSuccessorFromExecutionAnchor` (đã xác nhận 0 call site trong `src/runtime` và `src/px4`).
- Trong `planner_core/planner.hpp`: chuyển các method tương ứng sang `private`, vì `Planner::plan()` vẫn dùng chúng nội bộ (`planner.cpp:2349-2480`). Xoá hẳn method nào không còn caller nào, kể cả nội bộ (ví dụ `resetExpOptimizationDiagnostics` nếu chỉ còn một chỗ gọi thì inline hoặc giữ private, ghi rõ trong REPORT).
- Port mọi test trong `navigation_planning_backend/test/test_planner_facade.cpp` đang dùng các API trên (51 lời gọi) sang dùng `PlannerFacade::plan(PlanningRequest)`. Đặc biệt là 5 test sau: `ExpiredPlanReportsLatestTimeoutAtProductBoundary`, `ImmediateHeadingRebindRetainsPositionAndUsesNewActiveLeg`, `PassThroughEntryAfterBackupDoesNotAdvertiseBoundaryEvent`, `PassThroughLookaheadExportsRouteBoundaryEvent`, `PassThroughLookaheadPrefixWithoutBoundaryEntryStaysValid`. Viết một helper builder `PlanningRequest` cho test, dùng chung. Có thể tham khảo cách làm của nhánh `codex/close-findings-implementation` (commit `c1c04a8`), nhưng KHÔNG cherry-pick. Nhánh đó chậm 104 commit và conflict 19/21 file.
- `test_planner_config.cpp` và các test khác nếu có dùng những API này: xử lý tương tự.

### I3. QoS profile không dùng
- `fast_lio_ros/include/fast_lio_ros/qos_profiles.hpp` và `src/qos_profiles.cpp`: xoá `mappingObservation()` và `mapOutput()` (0 consumer).
- Sửa comment/doc cho đúng QoS thực tế của `/lio/mapping_observation` (`estimatorOutput()`: reliable, keep_last 10; `ros_output_publisher.cpp:120`) trong `qos_profiles.hpp` và `docs/architecture/navigation_layers.md`. KHÔNG đổi QoS publisher: đổi QoS là thay đổi hành vi, để sang P2.

### I4. Shim header
- `navigation_runtime/include/navigation_runtime/execution_recovery_state.hpp` chỉ re-export `navigation_execution`. Đổi 4 chỗ include (`planner_fsm.hpp:15`, `same_identity_renewal_injection.hpp:8`, `navigation_runtime_node.hpp:43`, `test/execution_authority_lifecycle_fixture.hpp:7`) sang header gốc. Xoá shim. Chỉnh `using` nếu cần.

### I5. Doc drift
- `docs/architecture/navigation_layers.md` nhắc `navigation_execution::ExecutionTimelineStore`, là class không tồn tại. Sửa cho đúng owner thực tế (`ExecutionAuthority`, `execution_authority.hpp`).
- Quét `docs/architecture/*.md`: mọi tên class/file được nhắc tới mà `git grep` không tìm thấy đều liệt kê trong REPORT. Chỉ sửa khi chắc chắn; không chắc thì ghi OPEN_QUESTIONS.

## Ngoài phạm vi
Không đụng `runCycle`, không đổi logic, QoS, ngưỡng. Không xoá bất cứ thứ gì chưa có trong danh sách trên, kể cả khi thấy nó cũng là dead code: ghi lại để làm WP sau.

## Nghiệm thu
- `make ci-local` xanh sau **mỗi** commit (REPORT ghi kết quả từng commit).
- Số test: tổng CTest sau WP ≥ trước WP trừ số test MissionController đã xoá. Số test MissionLoader được chuyển phải khớp 1:1.
- Toàn bộ 9 static guard, cộng `check_ci_contract.py`, pass.
- `git grep -nw MissionController -- src` chỉ còn trong guard/comment lịch sử (liệt kê cụ thể).
- `git diff --stat` không có file nào dưới `src/runtime/navigation_runtime/src/`, ngoại trừ các chỉnh comment ở I1.

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
