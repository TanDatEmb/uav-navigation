# W3-B1 — Piece/Trajectory polynomial contract

## Tóm tắt

CONFIRMED: branch `refactor/W3-B1` đã rebase sạch lên `origin/main=24ec0fc8718bb8606e4e9e4a7eaa84773d384854`; không merge cây chính.
CONFIRMED: W3-B1 vẫn là `refactor(move)` duy nhất; `Piece`, `Trajectory`, `RootFinder` và implementation ở `navigation_planning/polynomial`, giữ `geometry_utils`/`math_utils`.
CONFIRMED: REVIEW_R1 §3 #3 đã xử lý: `color_text` là bản chép nguyên văn định nghĩa gốc `static const std::string`; target `polynomial` bật `POSITION_INDEPENDENT_CODE ON`.
CONFIRMED: không thêm dependency ROS/`rog_map_vendor`; không còn include path cũ trong source backend.
CONDITIONAL: static/python gate PASS; Release build PASS; ROS gate đã chạy nhưng FAIL vì 5 attachment checks của dependency `px4_ros2_cpp`.
NOT_APPLICABLE: SITL/replay không chạy vì đây là MOVE-only, không đổi runtime behavior, threshold, authority hoặc safety policy.

## Deliverables và write-set

- Contract headers: `src/planning/navigation_planning/include/navigation_planning/polynomial/{piece,trajectory,root_finder,types}.hpp`.
- Implementations: `src/planning/navigation_planning/src/polynomial/{piece,trajectory,root_finder}.cpp`.
- CMake: `src/planning/navigation_planning/CMakeLists.txt`, `src/planning/navigation_planning_backend/CMakeLists.txt`.
- Backend callers: include path mới cho các caller/test; header cũ không còn shim.
- `type_utils.hpp` giữ include ngược duy nhất tới `polynomial/types.hpp` theo prompt; `trajectory.hpp` bỏ `color_msg_utils.hpp` không dùng.
- Test `test_trajectory.cpp` giữ ở backend vì dùng backend/corridor/planner, không phải test thuần của contract.

## Rebase / MOVE evidence

| Hạng mục | Kết quả |
|---|---|
| Base yêu cầu | `24ec0fc8718bb8606e4e9e4a7eaa84773d384854` |
| `git merge-base HEAD origin/main` | `24ec0fc8718bb8606e4e9e4a7eaa84773d384854` |
| Rebase | PASS, không conflict |
| Diff move | `git diff -M --color-moved=zebra --color-moved-ws=allow-indentation-change --stat origin/main..HEAD` giữ các file polynomial ở mức `R*`; logic thân hàm không đổi |
| Hunk không-moved | include path/`<string>`: include; qualifier Eigen: namespace; target/install/link: cmake; local `color_text` copy thay alias: alias; bỏ `color_msg_utils.hpp`: unused-include |
| Dependency | `navigation_planning` không link `rog_map_vendor`; không có cycle trong graph |

## Gate và kiểm chứng

- `tools/gate.sh static`: PASS — ledger `current=483 lines, gates=34, bypasses=1`; mission authority PASS; citations `checked=927 out_of_range=0`; dependency direction PASS với 4 baseline exceptions.
- `tools/gate.sh python`: PASS — Python 3.12.3; `tools/tests` 19/19; runtime `422 tests`, `OK (skipped=2)`.
- `flock /tmp/uavnav-build.lock env PARALLEL_WORKERS=2 MAKE_JOBS=2 make build`: PASS — 23 packages, 18m43s; authoritative build manifest được ghi trong `install/`.
- `flock /tmp/uavnav-build.lock env PARALLEL_WORKERS=2 MAKE_JOBS=2 make test`: PASS — exit 0; runtime `422 tests`, `OK (skipped=1)`; CTest affected/reverse đã chạy trên build Release.
- `flock /tmp/uavnav-build.lock bash -lc 'source /opt/ros/jazzy/setup.bash && env PARALLEL_WORKERS=2 MAKE_JOBS=2 ./tools/gate.sh ros'`: FAIL — rerun sau khi B6 nhả lock, đúng HEAD `ae7216ae32cbbc0a1ed03959451642b678d906cd`; `1483 tests, 0 errors, 5 failures, 0 skipped`. Các failure là attachment-only dependency checks của `px4_ros2_cpp`, cụ thể cpplint/uncrustify; gate không đạt `GATE_V3_RESULT=PASS`.
- SITL/replay: NOT_APPLICABLE cho `refactor(move)`; không dùng thiếu SITL để claim qualification runtime.

## REVIEW_R1 finding → status → commit

| Finding | Status | Commit |
|---|---|---|
| Piece/Trajectory/RootFinder còn ở backend contract | FIXED | `1866788` |
| Backend còn include path cũ | FIXED | `1866788` |
| `trajectory.hpp` kéo `color_msg_utils.hpp` không dùng | FIXED | `1866788` |
| Closure type thuần không được kéo `rog_map_vendor` vào target mới | FIXED | `1866788`, `a1a8d68` |
| REVIEW_R1 §3 #3: copy nguyên văn `color_text` gốc | FIXED | `50e008f` |
| REVIEW_R1 §3 #3: `POSITION_INDEPENDENT_CODE ON` cho `polynomial` | FIXED | `50e008f` |
| Release build / affected + reverse CTest | PARTIAL — build và `make test` PASS; ROS gate FAIL vì 5 `px4_ros2_cpp` attachment checks (cpplint/uncrustify) | `50e008f`, checkpoint evidence |
| SITL qualification | NOT_APPLICABLE — MOVE-only | — |

## Open questions / deviations

- OQ-01/OQ-02/OQ-03 cũ đã được cập nhật theo baseline v3; OQ-02 đóng bằng `50e008f`.
- Legacy lineage SHA không truy hồi theo COMMON_CONTRACT_v3; không dùng làm bằng chứng.
- Prompt W3-B1 không liệt kê path ở mục “Dọn dẹp và đóng”; vì vậy không tự `git rm` file nào và không tạo cleanup commit giả. Nếu kiến trúc sư chỉ định path cụ thể, ghi vào OPEN_QUESTIONS và xử lý ở commit `docs(cleanup)` riêng.

## Commit inventory

Bảng này lấy từ `git log --format='%h %s' origin/main..HEAD` sau report carrier cuối; final HEAD và remote push được xác nhận trong REVIEW REQUEST.

| SHA | Message |
|---|---|
| `b1cdfe2` | `docs(wave3): record B1 ROS gate result` |
| `fd2634a` | `docs(wave3): update B1 contract v3 review report` |
| `50e008f` | `refactor(planning): preserve moved color contract and enable PIC` |
| `9a05722` | `docs(wave3): complete B1 commit inventory` |
| `0f489e4` | `docs(wave3): complete B1 commit inventory` |
| `25b2702` | `docs(wave3): record B1 review fix` |
| `a1a8d68` | `fix(planning): restore direct type includes` |
| `55e7671` | `docs(wave3): record B1 remote handoff` |
| `b9d87d5` | `docs(planning): report W3-B1 move evidence` |
| `1866788` | `refactor(planning): move polynomial contract into navigation_planning` |

## Handoff

Branch đã được push bằng `--force-with-lease` sau khi report ghi đủ bằng chứng gate; gate vẫn FAIL do dependency attachment checks, nên REVIEW REQUEST chỉ báo cáo trạng thái và không tự cấp verdict. Không merge.
