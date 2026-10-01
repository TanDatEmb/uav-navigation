# WP-P0.3 — Báo cáo refactor trên baseline hiện tại

## 1. Tóm tắt

- Baseline thực tế sau rebase là `432dc94` (`chore(skills): add agent skills adapted from PX4`); các object `7e0b850`, `2543b0b4` và `74054ac` không tồn tại nên không thể thực hiện range-diff lịch sử R2.
- Làm xong bốn hạng mục refactor độc lập: I1, I3, I4 và I5; không thay đổi ngưỡng, QoS publisher, ownership runtime hay đường điều khiển.
- I1 xóa library/header/source/test `MissionController` legacy; 6 test loader được chuyển sang package `navigation_mission`, còn test controller legacy được loại bỏ theo owner mission-progress hiện tại.
- I3 xóa `mapOutput()` và `mappingObservation()` không có consumer; `/lio/mapping_observation` vẫn dùng `estimatorOutput()` reliable/keep-last 10.
- I4 xóa runtime recovery-state shim và chuyển các include/test sang `navigation_execution` owner.
- I5 ghi nhận giả định heading SITL chưa được kiểm runtime và known violation N3; đồng thời sửa hai citation stale sau I3.
- Không chạy SITL. Build branch sau refactor đã được thử bằng `make build` nhưng dừng có chủ đích ở `px4_msgs` sau 5 package/7m40s để tránh lặp generator tốn thời gian; build 23 package anh cung cấp là baseline tham khảo, không phải bằng chứng cho head này.

## 2. Deliverables

- I1: xóa `src/px4/px4_navigation_external_mode/{src/mission_controller.cpp,include/px4_navigation_external_mode/{mission.hpp,mission_controller.hpp},test/test_mission.cpp}`; thêm `src/contracts/navigation_mission/test/test_mission_loader.cpp`; cập nhật CMake/package ownership.
- I3: cập nhật `src/estimation/fast_lio_ros/include/fast_lio_ros/qos_profiles.hpp` và `src/estimation/fast_lio_ros/src/qos_profiles.cpp`.
- I4: xóa `src/runtime/navigation_runtime/include/navigation_runtime/execution_recovery_state.hpp`; cập nhật owner includes và enum assertions.
- I5: cập nhật `docs/architecture/frame_conventions.md`, `docs/architecture/timing_contract.md`, `docs/refactor/WP-A2/findings.md`, `docs/refactor/risk_register_20260928.md`.
- Safety record: cập nhật `docs/safety/mission_authority_cut.md` và thêm entry `WP-P0.3-I1` trong `docs/safety/runtime_safety_current.md`.

## 3. Finding → trạng thái → commit

| Finding | Trạng thái | Commit |
|---|---|---|
| I1 — `MissionController` legacy | FIXED; static authority guard PASS | `f523d21` |
| I2 — ambient planner API | DEFERRED; không đụng trong gói nhanh này, cần quyết định/contract P3 | — |
| I3 — QoS helper không dùng | FIXED; không có consumer còn lại | `364a36f` |
| I4 — recovery-state shim | FIXED; owner namespace là `navigation_execution` | `0179850` |
| I5 / D-01 / N3 | PARTIAL; ghi nhận known violation, chưa đổi runtime representation | `3c7b496` |
| I5 / R5-17 | PARTIAL; ghi nhận heading assumption, chưa thêm runtime alignment | `3c7b496` |

## 4. Verification evidence

| Kiểm tra | Kết quả |
|---|---|
| `python3 tools/check_mission_authority_cut.py` | PASS: `MISSION_AUTHORITY_STATIC_CHECK: PASS` |
| `python3 tools/validate_runtime_safety_ledger.py` | PASS: `current=483 lines, gates=34, bypasses=1` |
| `python3 tools/refactor/check_citations.py . docs/refactor` | PASS: `checked=983 out_of_range=0`; còn ambiguous basename legacy như `config.hpp`, không phải out-of-range |
| `git diff --check` trước các commit | PASS |
| `make build COLCON_FLAGS='--packages-select navigation_mission'` | C++ package compile PASS (`navigation_mission` finished 16.3s), wrapper provenance FAIL-CLOSED vì partial install thiếu `install/navigation_contracts` |
| `./build/navigation_mission/test_mission_loader` | PASS: 6/6 |
| `./build/navigation_mission/test_mission_contract` | PASS: 19/19 |
| CTest wrapper for the two mission tests | NOT_RUNNABLE in partial install: `ament_cmake_test` missing from overlay; direct gtest binaries above pass |
| Python baseline trước refactor | PASS: Python 3.12.3, `420 tests`, `2 skipped`, `0 failures` |
| `make build` trên head refactor | NOT_EVALUABLE/INTERRUPTED: `5 packages finished`, `px4_msgs` bị dừng sau `7min 40s`, `17 packages not processed`; không diễn giải là build PASS |
| SITL/replay | NOT_RUN; ngoài gói refactor nhanh |

Build 23 package do chủ dự án cung cấp (`23 packages finished [4min 49s]`, 4 package stderr) được giữ làm baseline build context; manifest bind nó vào `ea0e00e` với dirty worktree, không phải SHA head `89c80cd`, nên không dùng làm acceptance evidence của branch này.

## 5. Deviations và open questions

- Không thể dùng contract baseline cũ hoặc thực hiện rebase/R2 first-parent audit vì lịch sử không có trong git hiện tại; cost là branch này cần công trình sư đánh giá như một gói refactor độc lập trên baseline squashed.
- Không triển khai J1/P0.2/P1/P2 trong lượt này. Các worktree cũ có code/report nhưng git metadata đã mất và baseline mới ghi rõ artifact/history cũ không còn kiểm chứng; không phục hồi mù.
- P0.3-I2 giữ nguyên API ambient theo owner decision 2026-09-30 P03-I2; cần wave mới bám source hiện tại để thêm `PlanningRequest` field tường minh và port test có kiểm chứng.
- Build source-level đầy đủ cần chạy lại sau khi công trình sư chốt baseline/dependency strategy; blocker hiện tại là thời gian generate `px4_msgs`, không phải compile error của bốn commit refactor.

## 6. Commit map

| Item | SHA | Message |
|---|---|---|
| I1 | `c9bbd90` | `refactor(p0.3/i1): remove legacy mission controller` |
| I3 | `14c40d1` | `refactor(p0.3/i3): remove unused QoS profiles` |
| I4 | `cbb1ef0` | `refactor(p0.3/i4): remove recovery state shim` |
| I5 | `68cbfdd` | `docs(p0.3/i5): record frame and timing limitations` |
