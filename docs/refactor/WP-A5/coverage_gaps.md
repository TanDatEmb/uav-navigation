# WP-A5 — coverage gaps và rủi ro còn mở

## Phạm vi và provenance

- Baseline của WP-A5 là main commit 7e0b850; không merge hoặc cherry-pick
  branch WP-D0/WP-A4.
- Bộ ADR-013..016 và risk register được đọc từ refactor/WP-D0 để hiểu contract,
  nhưng không phải file baseline và không được dùng làm source line evidence cho
  code. Chi tiết thiếu format WP-A4 được ghi ở OPEN_QUESTIONS.md: OQ-01.
- Không có product code change. Mọi kết luận ở đây là static/source evidence;
  runtime, PX4, SITL distribution và latency đều NOT_MEASURED.

## Gaps đã xác nhận

### G-01 — updateSetpoint là mega-function, ownership chưa tách

NavigationMode::updateSetpoint() dài 2063-2582, còn executor và setpoint
publication lifecycle nằm trong cùng class. Các guard terminal, freshness,
health, command lease, terminal status, velocity-only và legacy PVA nối tiếp
nhau bằng nhiều return branch. Đây là CONFIRMED về cấu trúc source; chưa phải
kết luận rằng hành vi runtime sai. Bản đồ chi tiết nằm trong
update_setpoint_blocks.md và setpoint_guard_inventory.csv.

### G-02 — literal/timing contract chưa tập trung thành typed contract

Constructor/validation giữ các cửa sổ 0.10 s, 0.20 s và 5.0 s
(navigation_mode_node.cpp:171-190,243-271), trong khi updateSetpoint dùng
chúng ở nhiều guard (:2303-2445,2559-2582). Health wait còn có cap literal
0.5 s (:2346-2348). Các giá trị đã inventory, nhưng owner/contract giữa
planner, adapter và executor chưa là một typed object. Không đề xuất tuning
threshold trong WP-A5.

### G-03 — continuity recovery có seam sau gap/source invalid

R-02 trong risk register phù hợp với source hiện tại:
geometric_jump_continuity.cpp:75-100,130-135 làm mất trusted anchor sau
source invalid hoặc dt > 0.5 s; frame kế tiếp có thể reseed trusted qua
:95-100. Gate external chỉ nhìn trusted hiện tại
(external_odometry_gate.cpp:18-24). Đây là CONFIRMED component-level,
CONDITIONAL integrated-level. Cần test chuỗi gap/source-invalid -> jump ->
recovery trước khi quyết định sửa behavior; không sửa threshold 0.75/10/0.5
trong WP-A5.

### G-04 — external bridge high-water không thể hiện source restart identity

External bridge reject epoch thấp hơn high-water ở
px4_external_odometry_bridge_node.cpp:215-234. Helper restart policy trong
frame_generation_policy.hpp:26-34 được call ở executable khác, không có
bằng chứng source-instance recovery cho external bridge. Đây là CONDITIONAL
design/evidence gap, xem OQ-02; không suy diễn rằng mọi process restart đều
kẹt vì object restart sẽ reset high-water.

### G-05 — typed health ở NavigationMode và string diagnostics ở bridge là hai hợp đồng

NavigationMode yêu cầu typed health/epoch/freshness tại
navigation_mode_node.cpp:2324-2405. External bridge lại subscribe
/lio/diagnostics best-effort và parse string key/value tại
px4_external_odometry_bridge_node.cpp:106-110,156-203. Không có evidence
trong WP-A5 rằng hai stream có cùng producer epoch, timestamp semantics hoặc
lifecycle. Đây là CONDITIONAL integration gap.

### G-06 — wall timer không khớp ADR-016

NavigationMode tạo wall timer/worker tại navigation_mode_node.cpp:299-308
và executor tạo wall timer tại :2599-2600; ADR-016 trên WP-D0 yêu cầu
SITL use_sim_time và cấm create_wall_timer cho loop safety-critical. Đây là
documentation/architecture coverage gap được CONFIRMED ở source baseline,
không phải runtime qualification và không được sửa trong analysis-only WP.

### G-07 — velocity-only emergency boundary chưa được chứng minh

publishVelocityOnlySetpoint() kiểm tra role emergency và authorization tại
navigation_mode_node.cpp:1702-1713 nhưng ngay sau đó yêu cầu STATUS_READY
:1714-1716; OQ-03 ghi nhận cần quyết định behavior riêng. Không claim
runtime reachability.

### G-08 — adapter pure helper không đồng nghĩa product path đầy đủ

tracking_adapter::adapt() là pure helper, không temporal state và không tự
tạo default setpoint (px4_tracking_adapter.hpp:17-20,351-505). WP-A5 thấy
nó được gọi trong velocity-only path (navigation_mode_node.cpp:1858-1862);
legacy PVA vẫn có đường transform trực tiếp :2526-2557. Đây là ownership gap
cho refactor, không phải bằng chứng adapter đã bảo vệ toàn bộ product output.

### G-09 — diagnostics freshness dài hơn measurement freshness

External bridge giữ max odometry age 0.5 s nhưng diagnostics age 2.0 s
(px4_external_odometry_bridge_node.cpp:47-78,206-213). Đây có thể là chủ ý
tách health lease khỏi sample lease, nhưng chưa có contract/evidence ở baseline
chứng minh phù hợp với source restart và jump latch. Phân loại CONDITIONAL.

### G-10 — MissionController không còn product call site nhưng chưa bị xóa

Search product source excluding tests/build cho 0 nơi instantiate/call
MissionController; CMake vẫn compile src/mission_controller.cpp
(src/px4/px4_navigation_external_mode/CMakeLists.txt:24-27) và
test/test_mission.cpp vẫn exercise nó. Vì vậy claim đúng là “0 product call
sites, legacy source/test còn tồn tại”, không phải “đã xóa hoàn toàn”.

## Coverage test còn thiếu

- Gap > 0.5 s với pose jump lớn ở frame recovery kế tiếp.
- source_valid false với pose jump lớn ở frame recovery kế tiếp.
- External bridge producer restart epoch 1 sau high-water epoch > 1.
- Đồng bộ lifecycle/epoch giữa /lio/diagnostics và propagated odometry.
- Adapter negative tests cho từng timing tuple/reset/float guard trên product
  call path; hiện có static contracts nhưng chưa có runtime coverage mapping.
- Repeated representative SITL và PX4/HIL evidence: NOT_MEASURED; không được
  thay bằng component test hoặc một run đơn.

## Classification rule

CONFIRMED = source hoặc test chứng minh trực tiếp. CONDITIONAL = cần điều kiện
tích hợp/runtime hoặc producer contract chưa hiện diện. SPECULATIVE = chưa dùng
cho finding chính; WP-A5 không trình bày speculative claim như defect.
