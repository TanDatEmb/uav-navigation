# Kết luận audit state contracts

## Kết luận chính

- **PASS_THROUGH:** nhận xét “nhiều state/identity đồng nghĩa authority bị trùng” không được xác nhận. `goal_epoch/request_id` là desired identity; `active_command_goal_epoch/request_id/generation` là identity của lệnh còn được sample. Khi guard hot-retarget qua, runtime giữ predecessor đến khi successor commit/activate. Nếu safety suffix đang sở hữu thực thi, goal mới vào pending slot đơn điệu và chỉ bị consume sau commit. Đây là khác lifetime/owner, có test production class. Full reachability qua mọi producer/admission gate chưa được dựng trong một test.
- **World suspension:** code stale-world gate ngừng publish ở tick đó, lưu exact bundle generation và đặt episode unavailable. Chỉ exact bundle sau fresh-world validation và commit recheck mới resume. Adapter cache/lease là owner riêng; hết hạn có thể gọi fail-closed stop/Hold request. Không có atomic handoff giữa hai tầng và chưa có integrated liveness test/trace.
- **Stopped recovery:** runtime 5 s dùng steady clock, bắt đầu tại first failed current `PlanFromRest`; adapter 5 s dùng ROS clock, bắt đầu khi nhận completed endpoint cần recovery. Owner, consumer, clock và start event khác nhau. Bằng nhau về số giây không chứng minh xung đột. Quan hệ ngân sách liên tầng chưa được đặc tả/enforce.
- **Bottleneck:** **KHÔNG ĐỦ DỮ LIỆU XẾP HẠNG BOTTLENECK THỰC TẾ**: không có target workload hoặc latency distribution. Không kết luận state count, MINCO, callback hay lock là nguyên nhân chiếm ưu thế.
- Stale world, expired command, stale health/state/frame, stopped retry timeout và PX4 handover là fail-closed branches trong code; việc dừng mission tự nó không phải defect.

## Khóa baseline và delta

- `TARGET_SHA`: `8eaab3d33db36e9e636ad4005ed91b8f055f4d68`; tree `e82a074eee5703f0fde2acafd22381fa53020d39`; branch product `codex/close-proven-findings`, fetch một lần.
- Frozen A artifact commit `f2bd3f46f9936d622377ea4761f733f988273b66`; A ghi source HEAD `9534d8dc15920c8b3e80c8fa12f28ec972b8c6a2`, dirty status. Manifest có 596 file, SHA-256 `f9481b2f7312556ae91a8bff590cbc412de8d7e3e068fc98d4c85bb34c730413`. Vì vậy A không phải clean tree của HEAD đó. So sánh từng hash manifest với source blob tại TARGET, không dùng source ở root checkout.
- Delta manifest ghi 17 đường dẫn đổi/thiếu: 15 content paths và 2 generated build metadata. Governor/braking, planner CIRI, safety ledger và episode identity đã đổi; adapter PASS_THROUGH/mission/command/PX4 paths sâu không đổi. Chi tiết: [DELTA_REVIEW.md](DELTA_REVIEW.md).
- Dependency pins: `px4_msgs` `86d8239e962f6939e05c3737784f60c02fa884db`; `px4_ros2_interface_lib` `4a3370f084ac6f1ef001a4afa2b007845ffd0837`. H6 dependency claim không được mở lại vì pin/consumer không đổi A→TARGET; state writers của request/in-flight/confirmed vẫn được nối vào matrix, kể cả callback Success/Deactivated xóa pending riêng với VehicleStatus.

## Verdict theo claim

| Claim | Verdict | Source reachability và test | Phản chứng / phần chưa biết |
|---|---|---|---|
| C1 PASS_THROUGH handoff/ownership | `CONDITIONAL` | Source call path; 5 `ExecutionEpisode`, 8 FSM, 23 adapter progression tests pass. Active identity giữ đến commit; suffix pending goal exact-consume. | Chưa test producer tạo candidate qua toàn bộ admission/world gates; chưa có target trace. Lost-crossing local sequence không chứng minh product reachability. |
| C2 world suspension → resume/adapter handover | `CONDITIONAL` | Runtime no-publish gate, exact-generation resume/recheck và adapter lease được đọc; episode/store/world/node fixture tests pass. | Không có test nối map worker callback đến receiver deadline/handover; live world freshness, remaining lease, PX4 acceptance chưa biết. |
| C3 stopped recovery timers | `CONDITIONAL` | Writer/reset/start events và guards được truy; runtime helper/handler retry tests pass. | Chưa có joint fake-clock test cho runtime steady clock + adapter ROS clock; relation cần thiết chưa có spec. |
| H0 effective deployment profile | `UNRESOLVED` | Config/constructor/consumer liên quan được xem trong phạm vi. | Không có parameter dump/profile của workload mục tiêu. |
| H1 continuation crossing mất witness | `CONDITIONAL` | Mission measured crossing và immediate update khi continuation command được nhận có trong code/test. | Một local sequence có thể overwrite previous sample khi chưa có witness; không chứng minh candidate đó admit được trong product. |
| H3 publication-under-lock là bottleneck | `NOT_REPRODUCED` | Structural source observation không được nâng thành latency claim. | Không có target RMW/workload timing hoặc distribution. |
| H4 hai recovery timer xung đột | `REFUTED` đối với suy luận “cùng 5 s ⇒ conflict”; `SPECIFICATION_GAP` với quan hệ ngân sách cần thiết | Khác owner, clock, start event, scope. | Một ràng buộc liên tầng có thể tồn tại nếu được đặc tả; chưa tìm thấy/enforce. |
| H5 old 16-sample governor miss | `REFUTED` cho tiền đề TARGET | Delta đã bỏ speed-grid mechanism cũ; builder/helper tests pass. | Không bác bỏ mọi false negative/search cost khác; không benchmark workload. |
| H6 Hold API success chỉ là ACK request | `REFUTED` theo pinned dependency/consumer trong prior audit, A→TARGET không đổi | Pin và consumer giữ nguyên. | Chưa quan sát firmware mode apply; không phải field runtime verdict. |
| H7 bottleneck thực tế | `UNRESOLVED` | Funnel gates được xác định source. | Không có matched workload counts/reasons/latencies. |

H1 không gộp vào C1: adapter gọi `updateMission()` ngay cho admitted current continuation; điều này bác bỏ câu chuyện tổng quát “chỉ chờ timer sau”. Tuy nhiên current witness/crossing/callback-order relation chưa được chứng minh end-to-end.

## Build, tests và giới hạn

Build Release tạo fresh binaries từ TARGET (13 packages, exit 0). 12 nhóm test chọn lọc, 106 case tổng cộng, pass; xem [tests/selection.json](tests/selection.json), logs nén có hash tại `validation/log_manifest.json`. Đây là production classes/helpers với fake clocks, synthetic immutable worlds và local transport fixtures; không phải runtime observations/PX4 acknowledgements/qualification.

ROS tests dùng `ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=218`, `rmw_fastrtps_cpp`. Trước test không thấy FC/MicroXRCE/runtime/Gazebo/rosbag processes hoặc ports liên quan. `unshare -n` bị chặn, nên isolation evidence là local-only middleware/process check, không phải network namespace. Không dùng flight stack, SITL, thiết bị, arm, mode switch hay thiết bị ROS domain.

## Trạng thái

- `AUDIT_DELIVERY: PARTIAL` — ba contract được truy source và có test chọn lọc; các gap writer/reader liên tầng nêu dưới đây chưa khép kín.
- `DESIGN_EVIDENCE: PARTIAL_AS_IS` — đủ để thảo luận có giới hạn về identity/handoff và fail-closed behavior; chưa đủ để chốt liveness theo workload/profile.
- `PERFORMANCE_NOT_CHARACTERIZED` — không có matched workload timing.

`PARTIAL_AS_IS` cho toàn hệ thống; focused three-contract slice có source refs và selected tests. Còn thiếu: deployment parameter dump, target workload trace, joint world-to-receiver lease fixture, joint two-clock recovery test, full production-created handoff candidate/admission scenario và PX4 acceptance observation. Không sửa product source/config/tests.
