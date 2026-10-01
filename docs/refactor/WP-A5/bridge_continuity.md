# WP-A5 — bridge continuity

## Kết luận

Trong external odometry bridge, source identity được lọc trước conversion và
publication; geometric continuity và latch chặn frame nhảy khi có baseline tin
cậy. Tuy nhiên, gap lớn hơn 0.5 s hoặc source invalid sẽ reseed frame hiện tại
với continuity_trusted=false, và frame kế tiếp có thể trở thành baseline trusted
mà không được so với anchor cũ. Đây là CONFIRMED ở mức component, CONDITIONAL ở
mức tích hợp vì các input diagnostics, timestamp, covariance, generation và
transport còn phải đồng thời hợp lệ. Không có runtime/PX4 measurement ở WP-A5.

## Chuỗi xử lý tĩnh

1. on_lio() kiểm tra epoch/sequence bằng strictly_newer_source_identity trước
   conversion (px4_external_odometry_bridge_node.cpp:215-226) và chỉ sau đó
   cập nhật high-water (:228-234). Sample cũ hoặc trùng bị drop.
2. Khi epoch mới được chấp nhận, node xóa last_received_, last_published_ và
   continuity state, đồng thời thông báo epoch cho latch
   (px4_external_odometry_bridge_node.cpp:235-244).
3. Conversion kiểm tra frame lio_odom/base_link, timestamp, quaternion và
   covariance PSD/finite; lỗi đóng publication (external_odometry_conversion.cpp:105-174,
   px4_external_odometry_bridge_node.cpp:245-283).
4. TimestampConverter áp mapping generation, tuổi sample tối đa 0.5 s và các
   high-water timestamp (timestamp_conversion.cpp:49-118,
   px4_external_odometry_bridge_node.cpp:286-295).
5. Source continuity hợp lệ chỉ khi LIO valid, frame valid và public generation
   hợp lệ (px4_external_odometry_bridge_node.cpp:300-303). Geometric helper
   sau đó cập nhật baseline/trust/jump (geometric_jump_continuity.cpp:75-168).
6. Gate yêu cầu node/transport ready, timestamp/covariance/LIO/generation/frame
   fresh, continuity trusted và không có jump latch
   (external_odometry_gate.cpp:5-48). Chỉ gate pass mới tạo và publish
   VehicleOdometry (px4_external_odometry_bridge_node.cpp:314-372).

## Các nhánh continuity

### Jump trong cùng generation và dt hợp lệ

Với baseline hợp lệ, cùng public generation, min_dt <= dt <= max_dt, helper
tính delta position/orientation và bound theo speed/angular-rate
(geometric_jump_continuity.cpp:102-168). Delta vượt margin cộng rate*dt làm
jump=true; latch chỉ được set một lần tại geometric_jump_latch.cpp:5-12.
Gate sau đó đóng dù các input khác đúng. Đây là CONFIRMED từ source và test
component hiện có (test_geometric_jump_continuity.cpp:23-103).

### Gap lớn hơn max dt

Giả sử baseline tại t0 là p0, generation G, trusted=true; frame kế tiếp ở
t0+0.51 s có p10. Nhánh dt > max_dt tại geometric_jump_continuity.cpp:130-135
reseed p10 và đặt trusted=false, không đặt jump latch. Gate chặn chính frame
này qua continuity input (external_odometry_gate.cpp:18-24, node gate assembly
px4_external_odometry_bridge_node.cpp:314-339).

Frame kế tiếp cùng generation, nếu source valid và không có lỗi khác, đi vào
nhánh no trusted baseline tại geometric_jump_continuity.cpp:95-100: p10+epsilon
trở thành baseline trusted mà không so với p0. Vì vậy một pose jump có thể
được chấp nhận ở frame recovery tiếp theo. Đây là finding CONFIRMED ở mức
component và CONDITIONAL ở mức tích hợp; chưa có runtime measurement, và test
hiện tại chưa có chuỗi gap -> jump -> recovery.

### Source invalid

source_continuity_valid false khi lio_valid_, frame_valid hoặc generation
valid false (px4_external_odometry_bridge_node.cpp:300-303). Nhánh invalid
tại geometric_jump_continuity.cpp:75-83 cũng reseed frame hiện tại với
trusted=false. Frame valid kế tiếp lại dùng 95-100, trở thành baseline trusted
không so với anchor trước đó. Đây là cùng một continuity seam như gap; source
invalid còn bị các freshness/conversion gate chi phối nên kết luận tích hợp
là CONDITIONAL.

### Generation mới và latch

Diagnostics parse public frame generation tại
px4_external_odometry_bridge_node.cpp:156-203. Generation mới hợp lệ được
truyền vào latch (:171-190); observePublicFrameGeneration() chỉ clear latch
khi generation strictly newer (geometric_jump_latch.cpp:14-27). Đây là reset
theo identity công khai, không phải reset theo mỗi frame.

## Restart và high-water

strictly_newer_source_identity() chỉ chấp nhận epoch lớn hơn high-water hoặc
sequence lớn hơn trong cùng epoch (frame_generation_policy.hpp:13-20). Do
external bridge gọi nó trước mọi gate, nếu high-water là epoch 4 mà process
producer restart gửi epoch 1/sequence 1, sample bị reject trước conversion và
public generation không tiến (px4_external_odometry_bridge_node.cpp:215-234).
Trạng thái high-water được khởi tạo lại khi chính bridge process restart; còn
trong cùng process không có source-instance identity riêng để phân biệt restart.

Một helper khác, frame_generation_after_source_restart(), có semantics tăng
public generation khi đã có public frame (frame_generation_policy.hpp:26-34),
nhưng call site được rà thấy thuộc executable bridge khác
(px4_odometry_bridge_node.cpp:307-330), không phải on_lio() của external
bridge. Vì vậy chính sách recovery ở external bridge là CONDITIONAL/OPEN
QUESTION, không được nâng thành claim runtime chung. Xem OQ-02.

## Evidence status

- Static source evidence: CONFIRMED cho thứ tự high-water, conversion, continuity,
  latch và gate.
- Component test evidence: có test same-generation jump và generation change;
  chưa có test gap/source-invalid rồi jump ở frame recovery.
- Runtime/PX4/SITL: NOT_MEASURED; không dùng finding này làm flight acceptance.
