# W3-B6 — `px4_setpoint_core`

## Tóm tắt

- Baseline: `origin/main @ 432dc94630fbc76ca670138228f2f616f6840bb0`; branch `refactor/W3-B6`.
- Deliverable: target STATIC `px4_setpoint_core`, không ROS/PX4 runtime dependency; adapter và test thuần link target này.
- `navigation_mode_node.cpp` giữ nguyên 2 703 dòng; không tách `updateSetpoint`, không chạm `flight_profile`.
- Trạng thái: **CONDITIONALLY VERIFIED** ở package scope; full workspace `make build` chưa chạy do shared build lock của WP-P0.3. Branch được push để review; không merge.

## Verification

| Lệnh | Kết quả thật |
|---|---|
| `python3 src/px4/px4_navigation_external_mode/test/test_setpoint_core_layout.py .../CMakeLists.txt` | RED trước patch; GREEN sau patch: `PASS` |
| `cmake -S src/px4/px4_navigation_external_mode -B /tmp/uav-navigation-w3b6-cmake -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release` | configure/generate PASS |
| `cmake --build /tmp/uav-navigation-w3b6-cmake --target px4_setpoint_core --parallel 1` | `Built target px4_setpoint_core` |
| `cmake --build /tmp/uav-navigation-w3b6-tests --target test_navigation_command test_tracking_envelope test_local_frame_alignment test_px4_tracking_adapter test_velocity_only_continuity px4_navigation_external_mode_adapter --parallel 1` | tất cả target compile/link PASS |
| `ctest ... -R '^(test_setpoint_core_layout\|test_navigation_command\|test_tracking_envelope\|test_local_frame_alignment\|test_px4_tracking_adapter\|test_velocity_only_continuity)$'` với ROS Jazzy environment | `100% tests passed, 0 tests failed out of 6` |
| W3 dependency guard | `NOT_MEASURED` on this HEAD; A2 implementation is on separate branch `refactor/W3-A2` and must be rerun after integration |
| `python3 tools/check_mission_authority_cut.py` | `MISSION_AUTHORITY_STATIC_CHECK: PASS` |
| `python3 tools/validate_runtime_safety_ledger.py` | `PASS (current=482 lines, gates=34, bypasses=1)` |
| `python3 tools/refactor/check_citations.py . docs/refactor` | `checked=989 out_of_range=0` |
| `git diff --check` | PASS |

`make build` đã được thử nhưng bị từ chối bởi canonical lock: `pid=17616`,
`mode=exclusive-build`, đang chạy từ `/home/letandat/Dev/uav-navigation-r2`.
Không giết hoặc chạy song song build dùng chung.

## Moved-only evidence

- `git diff -M --color-moved=zebra --color-moved-ws=allow-indentation-change --stat`: chỉ có CMake/test; không có hunk logic sản phẩm.
- Header giữ nguyên include path và được đưa vào `PX4_SETPOINT_CORE_HEADERS`; không có đổi thân hàm/signature.
- `src/px4/px4_navigation_external_mode/include/**`: không thay đổi.
- `src/px4/px4_navigation_external_mode/src/**`: không thay đổi.
- `navigation_mode_node.cpp`: 2 703 dòng trước/sau.
- A5-R1 có các nhãn predicate `AD_*`, nhưng B3 hiện không định nghĩa free function `pred:<name>()` tương ứng trong `navigation_mode_node.cpp`; không tự ý tách điều kiện của `updateSetpoint`.

| Hunk không-moved | Lý do |
|---|---|
| `CMakeLists.txt` target/install/test link | `cmake` |
| `test_setpoint_core_layout.py` | verification-only, không phải product logic |

## Finding → trạng thái → commit

| Finding / yêu cầu | Trạng thái | Commit |
|---|---|---|
| Target STATIC `px4_setpoint_core` cho 11 header thuần | FIXED | `c08d832` |
| Core không link/include `rclcpp`, `px4_ros2_cpp` | FIXED | `c08d832` |
| Test thuần link `px4_setpoint_core`; adapter link target | FIXED | `c08d832` |
| Predicate A5-R1 trong node | NOT_APPLICABLE — không có free predicate tương ứng trên B3 | `c08d832` |
| `updateSetpoint`/`flight_profile` ngoài phạm vi | NOT_TOUCHED | `c08d832` |

## Commit

| SHA | Message |
|---|---|
| `c08d832` | `refactor(px4): isolate pure setpoint core` |
| `42b4c18` | `docs: report W3-B6 package gate` |

## Open questions

1. **CONDITIONAL:** Full workspace `make build` và Make-driven CTest cần chạy lại sau khi lock P0.3 được giải phóng; targeted package build/CTest đã PASS.
2. **CONDITIONAL:** `px4_setpoint_core` là target STATIC với generated anchor vì write-set cấm thêm product `.cpp`; nếu kiến trúc sư yêu cầu archive có translation unit thật, cần quyết định mở rộng write-set ở wave sau.
3. **CONDITIONAL:** Target core hiện là carrier cho header-only implementation; owner cần xác nhận mô hình này phù hợp với mục tiêu MOVE trước khi merge. Không tự chuyển inline logic sang `.cpp` trong WP này.
