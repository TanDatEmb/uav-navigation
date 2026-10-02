# W3-A2b report: `gate.sh ros` excludes `px4_ros2_cpp` from the verdict (G1)

Baseline `origin/main` = `24ec0fc8718bb8606e4e9e4a7eaa84773d384854`. Branch `refactor/W3-A2b`. Gate-rule change (tooling): architect review required, not layer L.

## Tóm tắt
- Lỗi (CONFIRMED, REPORT #3 B1): `colcon test-result --verbose` không có `--test-result-base` quét toàn bộ `build/` nên failure attachment của `px4_ros2_cpp` (G1) làm gate FAIL dù CTest package được chọn PASS.
- Sửa: áp đúng `gate_ros_attachment_fix.patch` (sha256 `af548c0b54c413d110bbd0d9aded8dbdb5fb3627e8bc8fd1a9d32200657ee79d`, đã kiểm khớp REVIEW_R2 §2): `colcon test-result --test-result-base build/<pkg>/test_results` cho từng package được chọn, bỏ `px4_ros2_cpp`; có failure ⇒ `return 1` ⇒ `GATE_V3_RESULT=FAIL`.
- Không đổi gì khác trong `tools/gate.sh` (diff +12/-1, một hunk). Attachment `px4_ros2_cpp` vẫn chạy và ghi log `artifacts/gate/px4_ros2_cpp-ctest.log`.
- Ledger: không có entry mới. Không đổi ngưỡng/bypass runtime; G1 (attachment-only) đã là chính sách ADR-018 E3 và dòng `printf` + `--packages-skip px4_ros2_cpp` đã có sẵn, patch chỉ làm kết quả khớp chính sách đó. Gate vẫn chặn mọi failure của package được chọn (test thứ hai). Không có WP an toàn nào bị đổi.

## Verify
| Lệnh | Kết quả |
|---|---|
| `python3 -m unittest tools.tests.test_gate` trước fix (head RED `687eed7`) | `test_ros_gate_ignores_attachment_only_px4_ros2_cpp_failures` FAIL (stub in `Summary: 5 errors, 5 failures (px4_ros2_cpp)`, `GATE_V3_RESULT=FAIL`); test selected-failure PASS vì gate đã FAIL |
| cùng lệnh sau fix | 6/6 OK |
| `tools/gate.sh static` | `GATE_V3_RESULT=PASS` (citations 920 checked, out_of_range=0; dependency PASS, 4 baseline exceptions) |
| `tools/gate.sh python` | Python 3.12.3; tools/tests 21/21; runtime/tests 422 OK (2 skips); `GATE_V3_RESULT=PASS` |
| `tools/gate.sh ros` trên head #3 B1 `ce907cf` + 2 commit A2b (cherry-pick tạm, `2d9ec95`, nhánh local `tmp/a2b-b1-verify`, không push) | `GATE_V3_RESULT=PASS` |

`gate.sh ros` thực tế (flock `/tmp/uavnav-build.lock`, `--parallel-workers 2`, `MAKE_JOBS=2`):
```
gate: head=2d9ec9556025bda8a0b4d4abf709bcab476a5064
gate: ros packages=navigation_bringup navigation_execution navigation_planning navigation_planning_backend navigation_runtime
Summary: 5 packages finished [17.4s]
Summary: 85 tests, 0 errors, 0 failures, 0 skipped
Summary: 26 tests, 0 errors, 0 failures, 0 skipped
Summary: 380 tests, 0 errors, 0 failures, 0 skipped
Summary: 290 tests, 0 errors, 0 failures, 0 skipped
gate: ros: px4_ros2_cpp attachment rc=0 log=artifacts/gate/px4_ros2_cpp-ctest.log
GATE_V3_RESULT=PASS
```
Đối chứng trên cùng cây sau gate (`build/px4_ros2_cpp/test_results` có mặt):
`colcon test-result` không base: `Summary: 4051 tests, 2 errors, 3037 failures, 121 skipped` (hành vi cũ, sẽ FAIL); `--test-result-base build/px4_ros2_cpp/test_results`: `3231 tests, 2 errors, 3033 failures` ⇒ toàn bộ failure đến từ `px4_ros2_cpp` (CONFIRMED); gate mới không đếm chúng.

## Lệch prompt
- Worktree tạm mới không có `install/` của package upstream, nên trước gate đã chạy `colcon build --packages-up-to <5 package> px4_ros2_cpp` Release (21 package, PREBUILD_RC=0) trong cùng khoá flock; sau đó `gate.sh ros` tự build lại + test 5 package được chọn. Không đổi gate.
- Tổng 781 test (4 package có kết quả; `navigation_bringup` không có CTest) khác con số 1483 trong REPORT #3 vì REPORT đó gồm kết quả quét toàn `build/` (CONDITIONAL; không đối chiếu thêm).
- Phụ thuộc: `px4_ros2_cpp` attachment rc=0 trong lần chạy này; failure xuất hiện trong kết quả `colcon test-result` do kết quả cũ/attachment trong `build/` (nguồn chính xác NOT_MEASURED).

## Open questions
none

## Commit
| SHA | Message |
|---|---|
| 687eed7 | test(gate): ros result must ignore px4_ros2_cpp attachment failures (RED) |
| 1a81868 | fix(gate): count only selected packages' CTest results in ros gate (G1) |
| (head) | docs(W3-A2b): report (SHA of this commit = PR head, not self-referenced) |

## Finding
| Finding | Status | Commit |
|---|---|---|
| G1: `gate.sh ros` đếm failure `px4_ros2_cpp` | FIXED | `fix(gate)` |
| Cleanup `salvage/gate_ros_attachment_fix.patch` | Sau merge: `rm` trên cây chính (contract §6.1) | n/a |
