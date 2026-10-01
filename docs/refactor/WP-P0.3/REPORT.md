# WP-P0.3 — REPORT sau W3-A1

## Tóm tắt

- Baseline B3: `origin/main=432dc94630fbc76ca670138228f2f616f6840bb0`; branch cục bộ `refactor/WP-P0.3` đã có baseline này làm ancestor.
- W3-A1 sửa đúng I4: xoá re-export trong runtime headers và qualify trực tiếp symbol sở hữu bởi `navigation_execution`.
- I2 không chạm, giữ `DEFERRED → W3-B5`.
- Không đổi hành vi, ngưỡng, lease, UNKNOWN policy, authority hay safety ledger.
- Compile runtime thành công; các CTest package bắt buộc và Python đều PASS.
- Full build chưa đạt acceptance vì wrapper provenance fail-closed sau build; `make test` có một failure TB-003 không liên quan, CTest lặp package đã PASS.
- Theo yêu cầu owner: chưa push, chưa mở PR, chưa merge.

## Deliverable và write-set

- Runtime headers: `planner_fsm.hpp`, `same_identity_renewal_injection.hpp`, `baseline_refinement.hpp`, `navigation_runtime_node.hpp`, `execution_lifecycle_view.hpp`.
- Runtime TU/test compile plumbing: `navigation_runtime_node.cpp` và các test TU cần local `using`; không tạo lại public header shim.
- Báo cáo này; không sửa/copy/commit prompt coordination docs.
- I4(a): `planner_fsm.hpp` không còn 5 alias cũ; I4(b): `same_identity_renewal_injection.hpp` dùng kiểu fully-qualified và compile độc lập.

## Finding → trạng thái → commit

| Finding | Trạng thái | Commit |
|---|---|---|
| I4(a) — recovery-state shim bị dời vào `planner_fsm.hpp` | FIXED | `0f72686` |
| I4(b) — `same_identity_renewal_injection.hpp` thiếu qualifier | FIXED | `0f72686` |
| I2 — ambient planner API | DEFERRED → W3-B5 | — |

## Verification evidence

| Lệnh | Kết quả thực tế |
|---|---|
| `grep -rn 'using navigation_execution::' src/runtime/navigation_runtime/include` | PASS; trước sửa 8 dòng alias, sau sửa 0 dòng |
| `git diff --check` | PASS |
| `python3 tools/check_mission_authority_cut.py` | PASS: `MISSION_AUTHORITY_STATIC_CHECK: PASS` |
| `python3 tools/validate_runtime_safety_ledger.py` | PASS: `current=483 lines, gates=34, bypasses=1` |
| `python3 tools/refactor/check_citations.py . docs/refactor` | PASS: `checked=983 out_of_range=0`; còn ambiguous basename đã biết |
| `make build` Release | Lần đầu: `21 packages finished [25min 27s]`, dừng tại `test_planner_fsm.cpp:547` do thiếu local `ExecutionPhase`; sau khi sửa, runtime compile PASS |
| `make build COLCON_FLAGS='--packages-skip-build-finished'` | `navigation_runtime` và `navigation_bringup` compile PASS; wrapper fail-closed khi thiếu `install/px4_navigation_external_mode/lib/libpx4_navigation_external_mode_contract.a` trong bước provenance |
| CTest `navigation_runtime` với `-E integration_tests` | PASS: `19/19` |
| CTest package set với ROS Jazzy sourced: `navigation_execution`, `navigation_mission`, `px4_navigation_external_mode`, `fast_lio_ros`, `navigation_planning_backend` | PASS lần lượt `2/2`, `2/2`, `8/8`, `12/12`, `9/9` |
| `/usr/bin/python3 -m unittest discover -s tools/runtime/tests -p 'test_*.py' -v` | PASS: Python `3.12.3`, `420 tests`, `1 skipped` |
| `make test` | NOT PASS: `93 tests`, `1 failure` tại `PlannerFacade.CruiseFutureAnchorDoesNotReturnToUnacceptedPassBoundary` (TB-003/known contract trial); CTest package lặp lại sau đó PASS `9/9` |

## Deviations, blockers và tiêu chí chưa đạt

- Prompt yêu cầu push/draft PR, nhưng owner yêu cầu rõ `không push/merge`; branch upstream hiện `[gone]`, nên giữ commit local và không tạo PR.
- Acceptance “build đủ workspace Release + authoritative manifest” chưa đạt: compile đã đi tới runtime, nhưng build wrapper không tạo manifest do artifact path `libpx4_navigation_external_mode_contract.a` không tồn tại trong target hiện tại. Đây là blocker provenance độc lập với I4.
- Acceptance “CTest toàn bộ gate ổn định” chưa đóng hoàn toàn vì `make test` có failure TB-003; lần CTest sourced trực tiếp trên package set đều PASS, cần owner/kiến trúc sư quyết định cách xử lý nondeterministic/known trial trước merge.
- SITL/replay không chạy; đây là refactor namespace/header, không tạo bằng chứng runtime qualification.
- I2 chưa làm theo chỉ thị W3-A1; chuyển nguyên trạng sang W3-B5.

## Commit map

Bảng này lấy ngay trước khi cập nhật REPORT bằng `git log --format='%h %s' origin/main..HEAD`.

| SHA | Message |
|---|---|
| `0f72686` | `refactor(runtime): qualify execution recovery symbols` |
| `d7cfe8c` | `docs(p0.3/report): record current baseline evidence` |
| `68cbfdd` | `docs(p0.3/i5): record frame and timing limitations` |
| `cbb1ef0` | `refactor(p0.3/i4): remove recovery state shim` |
| `14c40d1` | `refactor(p0.3/i3): remove unused QoS profiles` |
| `c9bbd90` | `refactor(p0.3/i1): remove legacy mission controller` |
