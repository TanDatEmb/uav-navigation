# WP-P0.3 — REPORT sau W3-A1

## Verdict

W3-A1 đã hoàn tất trên branch đã rebase với `origin/main=599dc057f4162878022854fa5e15a7baffb91fdc`. Đây là refactor namespace/header và tooling provenance; không đổi thân hàm, chữ ký, ngưỡng, authority, UNKNOWN policy hay safety ledger. Chưa merge; chờ review cấp `MERGE-READY`.

## Deliverable và write-set

- Runtime headers/TU/test compile plumbing trong `src/runtime/navigation_runtime/`: xoá public-header shim và qualify symbol sở hữu bởi `navigation_execution`; I2 giữ `DEFERRED → W3-B5`.
- `tools/runtime/build_provenance.py:16-24`: chỉ giữ artifact thực sự được CMake install.
- `tools/runtime/tests/test_runtime_contract.py:80-99,1120-1132`: kiểm tra mọi `CRITICAL_ARTIFACTS` ánh xạ tới `install(TARGETS ...)` của package tương ứng.
- `src/runtime/navigation_runtime/test/execution_authority_lifecycle_fixture.hpp:63-65`: qualify `ExecutionRecoveryEvent`; không còn `using` trong runtime header.
- Không chạm B1 polynomial files, B6 adapter, H3 fixes, A2 gate/dependency implementation.

## Finding → trạng thái → commit

| Finding | Trạng thái | Commit |
|---|---|---|
| I4(a) — recovery-state shim trong `planner_fsm.hpp` | FIXED; header guard còn 0 alias | `066881e` |
| I4(b) — `same_identity_renewal_injection.hpp` thiếu qualifier | FIXED; header compile độc lập | `066881e` |
| I4(c) — fixture test header còn alias cục bộ | FIXED; chuyển sang fully-qualified type | `2a5312d` |
| Provenance trỏ artifact static không còn install | FIXED; bỏ `libpx4_navigation_external_mode_contract.a` | `2a5312d` |
| Provenance/install drift không có test | FIXED; thêm mapping guard cho 7 artifact critical | `2a5312d` |
| I2 — ambient planner API | DEFERRED → W3-B5 | — |

## Verification evidence trên HEAD `2a5312d864367787c08669564723fac04b90f3cd`

| Lệnh/evidence | Kết quả |
|---|---|
| `git submodule update --init --recursive` | PASS; `px4_msgs` và `px4_ros2_interface_lib` đã initialized |
| `PARALLEL_WORKERS=2 MAKE_JOBS=2 make build` | PASS: 23 packages, 4m29s; authoritative manifest được ghi tại `install/.uav_navigation_build_manifest.json` |
| `PARALLEL_WORKERS=2 MAKE_JOBS=2 make test` | PASS; CTest và Python hoàn tất, Python `422 tests`, `OK (skipped=1)` |
| `source /opt/ros/jazzy/setup.bash && tools/gate.sh ros` | PASS: blocking `9 packages`, `1475 tests`, `0 errors, 0 failures, 0 skipped`; `px4_ros2_cpp` attachment rc=0 |
| `tools/gate.sh static` (trong lần `all`) | PASS: ledger `current=483 lines, gates=34, bypasses=1`; mission authority PASS; citations `checked=918 out_of_range=0`; dependency `35 packages`, 4 allowed baseline violations |
| `tools/gate.sh python` (trong lần `all`) | PASS: 19 tool tests và runtime `422 tests`, `OK (skipped=1)` |
| `grep -rn 'using navigation_execution::' src/runtime/navigation_runtime/include` | PASS; trước sửa 8 dòng alias, sau sửa 0 |
| `git diff --check` | PASS |

Lần gọi `tools/gate.sh all` không source Jazzy fail ở G1 vì `/usr/bin/python3` không thấy `ament_package`; đây là lỗi môi trường invocation. Gate ROS đã được chạy lại sau `source /opt/ros/jazzy/setup.bash` và PASS như bảng trên. Log attachment nằm ở `artifacts/gate/px4_ros2_cpp-ctest.log`; failure lint nội bộ của dependency là attachment-only và không chặn theo contract v3 §G1.

## Rebase và cleanup

- Đã rebase thành công lên merge commit A2 `599dc057`; không phát sinh conflict.
- `docs/architecture/planner_receding_horizon_contract.md` đã absent ở `origin/main` sau A2, nên không có modify/delete conflict để xử lý.
- `docs/architecture/px4_tracking_adapter_design_20260909.md` còn tồn tại trên A1 branch và được xóa trong commit cleanup cuối sau REPORT này.
- Chưa push lại head mới trong bản REPORT này; force-push bằng `--force-with-lease` sau khi tạo cleanup commit.

## Commit map

Bảng dưới đây được lấy ngay trước commit cập nhật REPORT bằng `git log --format='%h %s' origin/main..HEAD`; không dùng các SHA cũ trước rebase.

| SHA | Message |
|---|---|
| `2a5312d` | `fix(tools): align provenance artifacts with installed targets` |
| `77afe7c` | `docs(wave3): complete A1 commit inventory` |
| `4f37646` | `docs(wave3): finalize A1 gate report` |
| `6805cfe` | `docs(p0.3): record W3-A1 evidence` |
| `066881e` | `refactor(runtime): qualify execution recovery symbols` |
| `eb7ceaa` | `docs(p0.3/report): record current baseline evidence` |
| `47cf477` | `docs(p0.3/i5): record frame and timing limitations` |
| `643a421` | `refactor(p0.3/i4): remove recovery state shim` |
| `9f2de39` | `refactor(p0.3/i3): remove unused QoS profiles` |
| `e398c5a` | `refactor(p0.3/i1): remove legacy mission controller` |

## Handoff

- A1 head sau cleanup sẽ được push lên `refactor/WP-P0.3` bằng `--force-with-lease`; không merge.
- Các PR còn lại `#3, #1, #5, #6, #4` phải rebase lên `origin/main=599dc057` trước review tiếp theo.
- SITL/replay không chạy; gói này không được dùng để claim runtime qualification.
