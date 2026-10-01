# W3-B6 — `px4_setpoint_core`

## Tóm tắt

- Baseline sau rebase: `origin/main @ 24ec0fc8718bb8606e4e9e4a7eaa84773d384854`; branch `refactor/W3-B6`.
- Loại thay đổi: `refactor(move)`. Không đổi threshold, safety ledger, command ownership, deadline, planner/controller config hay runtime behavior.
- Đưa 11 pure header vào target STATIC nội bộ `px4_setpoint_core`; anchor `.cpp` generated include đủ cả 11 header để compile độc lập với ROS/PX4.
- Adapter và năm pure test link target; target không install/export theo cách giải conflict CMake của W3-A1 vì không có downstream consumer.
- `navigation_mode_node.cpp`: 2 703 dòng ở baseline → 2 678 dòng sau MOVE. Không tách `updateSetpoint`; không chạm `flight_profile`.
- Không tự cấp verdict qualification. Đây là handoff để coordinator/architect review.

## Rebase và phạm vi

Đã rebase lên `origin/main` đúng SHA yêu cầu. Conflict trong
`src/px4/px4_navigation_external_mode/CMakeLists.txt` được xử lý theo A1:
giữ target adapter/node và test hiện hành, thêm `px4_setpoint_core`, bỏ
`install(EXPORT ...)`/`ament_export_targets` vì core là target package-private.

Các header trong core:

`command_admission_assessment.hpp`, `command_acceptance_gate.hpp`,
`tracking_envelope.hpp`, `velocity_only_continuity.hpp`,
`px4_tracking_adapter.hpp`, `certified_command_handoff.hpp`,
`mission_command_identity.hpp`, `planner_recovery.hpp`,
`local_frame_alignment.hpp`, `navigation_input_validation.hpp`,
`runtime_metrics_policy.hpp`.

Ba helper được MOVE nguyên thân/signature, không đổi call site hoặc semantics:

| Helper | File sở hữu sau MOVE |
|---|---|
| `floatRepresentable` | `include/px4_navigation_external_mode/navigation_input_validation.hpp` |
| `checkedEnuToNed` | `include/px4_navigation_external_mode/local_frame_alignment.hpp` |
| `checkedTimestampAdd` | `include/px4_navigation_external_mode/planner_recovery.hpp` |

## Predicate → file

Các `pred:<name>()` trong WP-A5 là nhãn inventory/decision-table; B6 không
tạo free function mới cho chúng. Bảng dưới đây ghi file sở hữu của guard theo
`docs/refactor/WP-A5/setpoint_guard_inventory.csv` và giữ nguyên exit-site
ownership:

| Predicate/guard IDs | File sở hữu |
|---|---|
| `AD_G001–AD_G015`, `AD_G018–AD_G026`, `AD_G041–AD_G048` | `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp` |
| `AD_G016–AD_G017` | `src/contracts/navigation_contracts/include/navigation_contracts/command_safety_contract.hpp` |
| `AD_G027–AD_G034` | `src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/px4_tracking_adapter.hpp` |
| `AD_G035–AD_G040` | `src/px4/px4_navigation_external_mode/include/px4_navigation_external_mode/velocity_only_continuity.hpp` |
| Named labels `AD_MESSAGE_INVALID`, `AD_TERMINAL_CLOSED`, `AD_RETAIN_REJECTION`, `AD_STALE_ODOMETRY`, `AD_ANCHOR_INVALID`, `AD_COMMAND_ACCEPTED`, `AD_PROGRESS_*`, `AD_TERMINAL_*`, `AD_HEALTH_*`, `AD_RECEIVE_STALE`, `AD_HEADER_INVALID`, `AD_VALIDITY_EXPIRED`, `AD_PLANNER_REJECTED`, `AD_COMPLETED_*`, `AD_RECOVERY_DEADLINE_INVALID`, `AD_VELOCITY_ONLY_PATH`, `AD_PVA_UNREPRESENTABLE`, `AD_ACQUISITION_OPEN` | `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp` |
| Moved helper predicates used by those sites | `navigation_input_validation.hpp`, `local_frame_alignment.hpp`, `planner_recovery.hpp` as listed above |

Không có free function `pred:<name>()` tương ứng để tiếp tục tách khỏi node;
không tách `updateSetpoint` vì hàm phụ thuộc ROS/member state và không phải
MOVE thuần.

## Verification

| Lệnh | Kết quả |
|---|---|
| Structural test trước MOVE helper | RED: 6 lỗi đúng cho ba helper thiếu/đang còn ở node |
| Structural test sau MOVE helper và sau include-root regression assertion | `PASS: px4_setpoint_core target and pure test links are structurally present` |
| `tools/gate.sh static` | `GATE_V3_RESULT=PASS`; safety ledger `PASS`, mission authority `PASS`, dependency direction `PASS` với warning baseline đã cho phép |
| `tools/gate.sh python` | tools `19/19`; runtime `422` tests OK, `skipped=2`; `GATE_V3_RESULT=PASS` |
| Initial `make build` reproduction | FAIL at generated anchor: missing `navigation_contracts/navigation_command_contract.hpp`; root cause was missing `${navigation_contracts_INCLUDE_DIRS}` on core, fixed in `af471cc` |
| `git diff --check` | PASS tại code checkpoint; chạy lại sau khi chốt REPORT |
| `make build` dưới `flock /tmp/uavnav-build.lock` với `PARALLEL_WORKERS=2 MAKE_JOBS=2` | PASS, exit `0`; `23 packages finished [6min 1s]`; build manifest được ghi |
| `make test` dưới cùng lock | exit `0`; Python runtime `Ran 422 tests ... OK (skipped=1)`; recipe còn ghi `STOPPED` và `Runtime report: FAIL (cleanup succeeded)` cho fixture runtime, không phải qualification evidence |
| Targeted CTest dưới cùng lock | `100% tests passed, 0 tests failed out of 6`; tổng thời gian `0.30 sec` |

`tools/gate.sh static` cũng đã chạy `python3 tools/validate_runtime_safety_ledger.py`;
không có safety-document change trong B6. Không dùng single SITL run để suy ra
qualification; B6 không phải runtime/SITL qualification.

## Moved-only evidence

`git diff -M --color-moved=zebra --color-moved-ws=allow-indentation-change --stat`
được dùng để kiểm tra diff theo MOVE. Các hunk không phải moved logic sản phẩm:

| Hunk | Lý do |
|---|---|
| `CMakeLists.txt` target, generated anchor, link graph, package-private decision | build ownership / A1 conflict resolution |
| `test_setpoint_core_layout.py` | verification-only structural contract |
| `docs/refactor/kb/areas/R5_layer.md` citation range | cập nhật line anchor do xoá 25 dòng helper khỏi node |
| `docs/refactor/W3-B6/REPORT.md` | evidence/handoff only |

Không xoá file hay dọn `WP-A5`; prompt yêu cầu giữ oracle cho follow-up N15.

## Finding → trạng thái → commit

| Finding / yêu cầu | Trạng thái | Commit |
|---|---|---|
| Rebase đúng A1 baseline `24ec0fc...` | DONE | `722db27` |
| Target STATIC `px4_setpoint_core` cho 11 header | DONE | `722db27` |
| Anchor `.cpp` include đủ 11 header | DONE | `722db27` |
| Core không dùng/export ROS/PX4; không export target không cần thiết | DONE | `722db27` |
| Adapter và pure tests link core | DONE | `722db27` |
| Core có include root cho `navigation_contracts` sau build reproduction | DONE | `af471cc` |
| Ba helper MOVE vào header phù hợp | DONE | `1ff734a` |
| Predicate → file table và non-move rationale | RECORDED HERE | report commit |
| Không tách `updateSetpoint`, không đổi behavior | DONE | `722db27`, `1ff734a` |

## Open questions / review request

1. Cần coordinator/architect xác nhận mô hình header carrier + generated anchor
   phù hợp mục tiêu W3-MOVE; B6 không thêm product `.cpp` ngoài write-set.
2. N15 (unrepresentable input follow-up) vẫn là work riêng; B6 chỉ MOVE helper,
   không đổi gate hoặc hành vi.
3. Full workspace build, `make test` và targeted CTest đã có kết quả thật ở trên;
   full gate cuối vẫn phải chạy sau REPORT commit dưới shared lock.
4. Không có claim runtime/SITL qualification trong báo cáo này.

## Commit inventory

Bảng cuối phải được regenerate ngay trước push từ:

```text
git log --format='%h %s' origin/main..HEAD
```

Kết quả tại code checkpoint trước commit REPORT:

| SHA | Message |
|---|---|
| `9f0317c` | `docs(wave3): complete B6 commit inventory` |
| `427e624` | `docs(wave3): complete B6 commit inventory` |
| `22d2a73` | `docs(wave3): qualify B6 gate evidence` |
| `9d39095` | `docs(wave3): record B6 remote handoff` |
| `6a907be` | `docs(px4): record W3-B6 evidence` |
| `722db27` | `refactor(px4): isolate pure setpoint core` |
| `1ff734a` | `refactor(px4): move setpoint validation helpers` |
| `af471cc` | `fix(px4): expose contract includes to setpoint core` |
