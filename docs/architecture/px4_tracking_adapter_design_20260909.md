# Px4TrackingAdapter — thiết kế Level A cho PX4 1.17

Ngày: 2026-09-09. Trạng thái: DESIGN REVIEW, chưa triển khai adapter,
chưa có kết quả SITL chứng minh hiệu quả. Chủ sở hữu implementation duy nhất:
task `01a083d6-eaf2-7770-9988-04965ae8c29f`. Root phụ trách review.
Task `01a07215-a562-7a02-b359-70d35135c5d0` đã được yêu cầu dừng việc được
root giao và bàn giao WIP; không phân công implementation song song.

## 1. Quyết định và baseline

Chọn Level A để kiểm chứng việc dùng sai số vị trí LIO trong stock PX4 cascade.
Không sửa LIO, EKF fusion/trust/covariance, không thêm estimator translation,
không velocity-first, không custom firmware. Level B/C không nằm trong patch A.
Đây là thay đổi feedback control, không chỉ là đổi hệ tọa độ.

Source đã đối chiếu:

- Navigation HEAD khi kết thúc đọc: `4be263b145310a8e14f56d598e5069129da8b9b0`.
  Các file PX4 External Mode được đọc không có diff trong checkout.
- `/home/letandat/Dev/Autopilot`: `deaff86ee335dd697677bcfc2415a23878e1b895`,
  describe `v1.17.0-14-gdeaff86ee3-dirty`; tag local `v1.17.0` trỏ đến
  `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`. Có dirty DDS topic config và
  các thành phần khác. Đây là identity source, chưa xác nhận binary đang chạy.
- px4_msgs: `86d8239e962f6939e05c3737784f60c02fa884db`.
- px4_ros2_interface_lib: `4a3370f084ac6f1ef001a4afa2b007845ffd0837`.
- Checkout này CÓ `src/px4/px4_navigation_external_mode`; không suy từ mô tả
  một nhánh main khác rằng package đã bị xóa.

Tham khảo upstream chính thức, phiên bản 1.17:
[PositionControl](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/mc_pos_control/PositionControl/PositionControl.cpp),
[reset/controller](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/mc_pos_control/MulticopterPositionControl.cpp),
[EV position fusion](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/EKF/aid_sources/external_vision/ev_pos_control.cpp).
Đối với runtime, SHA local, diff, build manifest và ULog có quyền quyết định;
không dùng nhãn phiên bản thay cho build identity.

## 2. Những kết luận cần sửa trong đề xuất

1. Đẳng thức loại translation đúng tại cùng thời điểm/state; không đúng tuyệt
   đối qua transport và khoảng giữ setpoint. Khi FC dùng state mới ở t_c:
   `e_FC(t_c) = R_k e_L(t_k) + p_E(t_k) - p_E(t_c)` trước reset adjustment.
   Pairing LIO/PX4 tốt vẫn chưa loại được tuổi raw anchor khi FC tiêu thụ lệnh.
2. Giữ stock cascade không tự bảo toàn certificate PVAJ/corridor. Với R thay
   đổi, `d(p_sp)/dt = v_E + R(v_d-v_L) + dR/dt*(p_d-p_L)`, thường khác
   `v_ff = R*v_d`. Setpoint là reference có feedback, không phải một trajectory
   mới đã được planner certify. Giữ nguyên planner certificate và kiểm chứng
   tracking tube, dynamics, clearance của chuyển động thực riêng biệt.
3. PX4 reset compensation không phải handshake frame. Library hiện gửi
   `TrajectorySetpoint.timestamp=0`; UCDR deserialize thay bằng thời gian nhận
   PX4. Controller chỉ điều chỉnh khi timestamp/counter phù hợp. Command tạo
   từ state trước reset nhưng tới sau reset có thể không được sửa, đặc biệt
   khi controller đã tiêu thụ counter mới. Đây là test bắt buộc, không phải
   bằng chứng failure cũ đã do reset. Đổi timestamp một mình cũng không sửa
   được mọi thứ tự sự kiện vì controller không lưu lịch sử reset theo command.
4. Level A XYZ chuyển outer-loop height reference sang LIO. GPS vẫn hỗ trợ
   EKF/velocity/global status nhưng không tự cứu LIO height drift bị adapter
   triệt khỏi position error. Chưa giải quyết bay ra ngoài vùng lidar hữu ích.
5. Level B dùng raw velocity không tái tạo chính xác feedback nội bộ: local
   controller có velocity filtering và nhánh xử lý z; saturation nằm trước
   velocity error. Thay velocity error tác động cả P và I, D vẫn từ PX4.
   Không quảng bá lambda=1 là thay toàn bộ feedback bằng LIO.

EV position bias có nhánh phụ thuộc GNSS/EV frame/yaw; không đồng nhất nó với
accelerometer bias, gyro bias hoặc reset. Không truyền bias IMU FC sang IMU
lidar. Chưa quy nguyên nhân các mission fail trước đây cho PX4.

## 3. Boundary và API dự kiến

Đặt pure helper ở `px4_navigation_external_mode/px4_tracking_adapter.hpp`;
runtime state pairing thuộc NavigationMode, gọi sau command admission và ngay
trước tạo TrajectorySetpoint. Không đặt trong odometry bridge/planner.

`adapt(reference, lio_state, raw_px4_state, timing_witness, policy) -> result`

| Input | Contract |
| --- | --- |
| Reference | P,V,A,yaw,yaw-rate SI, thời điểm sample có nghĩa, frame, localization/mission/goal/request/bundle/sample identity, role và lease |
| LIO | PropagatedOdometry + typed health, source/receive time, sequence/epoch, P,V,orientation, covariance/validity |
| Raw PX4 | VehicleLocalPosition P,V,heading, heading validity/variance, position/velocity validity, dead-reckoning status, timestamp và timestamp_sample, reset counters/deltas; cùng message snapshot |
| Timing | Clock mapping generation và uncertainty, thời gian dự kiến dùng reference, pair skew, state ages, dự báo tuổi anchor tại FC |
| Policy | off/shadow/level_a, explicit experiment identity; lambda velocity luôn bằng 0 ở Level A |

Output success: raw-local-NED P,V,A,yaw,yaw-rate và immutable witness (input
IDs/timestamps, R, error LIO, error adapter, reset tuple, timing bound, mode).
Output failure: typed reason, không trả zero/default setpoint có vẻ hợp lệ.
Pure helper không giữ translation drift, không gọi ROS/I/O, không solve.
Runtime giữ bounded buffers và lifecycle/clock/reset generations.

Không dùng các trường diagnostic-only trong NavigationCommand làm authority.
Hiện header.stamp là thời điểm command và state_source_stamp là thời điểm
state; cần kiểm tra/bổ sung semantic reference evaluation time trong contract
trước khi dùng. Không lấy valid_until làm giấy phép coi sample cũ là sample mới.

## 4. ENU, NED, body origin và yaw

L: navigation ENU, body FLU. E: PX4 raw local NED, body FRD. Trước hết xác nhận
cùng điểm vật lý trên UAV. Nếu khác sensor origin, dùng extrinsic đã cấu hình
và lever-arm kinematics đúng một lần; không ước lượng offset động để che nó.

Đặt `C = [[0,1,0],[1,0,0],[0,0,-1]]` (ENU -> NED).
Với cùng heading vật lý, đã xử lý body extrinsic:

```
psi_E_enu = wrap(pi/2 - psi_E_ned)
delta     = wrap(psi_E_enu - psi_L_enu)
R         = C * Rz(delta)
e_L       = p_d_L - p_L
p_sp_E    = p_E + R * e_L
v_ff_E    = R * v_d_L
a_ff_E    = R * a_d_L
yaw_sp_E  = wrap(psi_E_ned - wrap(psi_d_L - psi_L_enu))
yaw_ff_E  = -yaw_rate_d_L
```

Yaw-rate là physical feedforward từ heading tracker, không lấy đạo hàm nhiễu
của relative estimator yaw để bù thêm. Không áp dụng giới hạn slew trực tiếp
lên raw yaw qua EKF heading reset; giới hạn chuyển động yaw trong LIO frame.
R là relative-heading measurement tức thời, không continuous FrameAlignment
estimator; nó vẫn nhạy với noise/time skew và phải có uncertainty witness.
Không suy gravity-aligned chỉ từ tên frame; tilt/extrinsic không hợp lệ phải
reject, không dùng yaw-only rotation ngoài giả thiết.

Kiểm tra identity: ENU east -> NED east; north -> north; up -> down;
delta=90 độ; wrap quanh +/-pi; vehicle quay nhưng delta giữa frame không đổi.

## 5. Timestamp và budget

Tách watchdog freshness khỏi control synchronization. Theo dõi source age,
receive age bằng steady clock, pair skew, output queue/transport age và PX4
consume age. DDS có thể đã dịch timestamp sang clock agent; không trừ offset
lần hai. Không mặc định timestamp_sample là thời điểm chính xác của predicted
output P: kiểm tra EKF publisher/output predictor của binary được dùng.

Không chọn ngưỡng từ một run. Requirement đơn giản chưa có prediction:
`T_error <= e_timing_budget / v_bound`; 0.10 m tại 5 m/s là 20 ms, tại
12 m/s là 8.33 ms. Đó là ví dụ phân bổ budget, chưa phải threshold production.
Raw VLP DDS config hiện rate_limit 50 Hz: chỉ riêng giữ mẫu có thể tới 20 ms,
tương đương 0.24 m ở 12 m/s. Vì vậy chỉ ghép nearest samples không đủ cho
budget 0.10 m tại 12 m/s trong trường hợp xấu nhất.

Dùng tổng bound có clock uncertainty, pair skew, anchor age/transport, sai số
vận tốc/prediction và `0.5*a_bound*T^2`; thêm sai số góc tác động lên e_L và
feedforward. Không cộng p95 độc lập để gọi là worst-case bound.

Giai đoạn đầu shadow đo distribution p50/p95/p99/max và gap dưới tải. Nếu
budget không đạt, sửa scheduling/transport trước. Bounded state prediction
hoặc tăng publication rate là thay đổi riêng có test và ledger, không tăng
watchdog timeout. Không extrapolate qua reset, health loss, role boundary hay
lease expiry. Không Taylor-extrapolate command vượt đoạn trajectory được cấp
quyền; nếu sample không phù hợp thì cần exact producer sample/contract riêng.

## 6. Health, resets và chuyển vai trò

Reuse typed LIO TRACKING/navigation/covariance/jump/epoch gates. Raw PX4 phải
có valid P/V trên các trục điều khiển, finite heading và heading_good_for_control;
check source/receive ages và clock mapping. Callback hiện chủ yếu lưu raw
position/velocity/counters, chưa giữ heading đầy đủ cho adapter. Invalid packet
không được refresh freshness hoặc để cached valid state sống vô hạn.

PX4 reset: không sửa LIO/ResetCompensator, không cộng delta sau khi đã anchor
raw mới. Invalidate pair cũ khi thấy counter đổi, tạo pair mới trong một epoch;
không interpolate xuyên reset. Counter wrap một bước phải phân biệt với jump
hoặc reboot. Vì reset ở FC có thể chưa tới ROS, local guard không đảm bảo
atomicity; phải đo reset-in-flight residual ở controller trước active trials.

LIO epoch/public-frame reset: invalid reference/pair cũ, yêu cầu reference được
producer cấp quyền cho epoch mới. Không làm smooth biến event thành drift.
Mode/clock restart: clear temporal caches, giữ nguồn reset riêng biệt.

Không bật/tắt adapter tự động giữa chuyến hoặc theo role. MAIN, BACKUP,
EMERGENCY và position hold của navigation phải qua cùng semantics khi
Level A active và input còn hợp lệ; nếu chỉ đổi MAIN, handoff có thể nhảy
position error về fixed-origin cũ. Giữ certificate/role/lease gates ở LIO.
Gặp input invalid: chuyển supervisor theo contract lỗi hiện có, không tự
fallback về absolute P hoặc âm thầm giữ adapter từ state cũ. Hold PX4 native
là handover sang một controller reference khác; đo discontinuity riêng.
GPS-off Hold unavailable vẫn là giới hạn test user đã chấp nhận.

Nếu chưa đóng được reset transport race, không claim reset continuity; Level A
chưa đủ điều kiện bật cho campaign obstacle tốc độ cao. Không sửa firmware
chỉ dựa trên giả thuyết: fault injection trước để xác định bound/limitation.

## 7. Instrumentation và A/B test

Reuse PX4_INPUT_SETPOINT trace nhưng đưa serialize/I/O ra ngoài hot callback;
bounded queue, drop counter và NOT_RECORDED cho thiếu dữ liệu. Log reference
LIO trước adapter, raw input/state pair, output sau adapter, setpoint FC nhận
và controller feedback/setpoint sau reset/saturation, health/reset/bias,
LIO-GT/PX4-GT/vehicle-GT, timing và version/config provenance. ROS publish
trace một mình không chứng minh FC đã dùng đúng error.

| Test | Điều phải chứng minh |
| --- | --- |
| Pure algebra | Translation 0/5/10 m triệt trong paired error; rotation/basis/yaw đúng; float overflow/NaN reject |
| Time skew/hold | Sweep 0/5/10/20/100 ms; transport và load gaps; measured residual phù hợp timing bound, không refresh sample giả |
| PX4 reset | XY/Z/heading/velocity reset trước/sau pair, enqueue, DDS receive, FC consume; old command tới muộn sau counter đã consume; không double/missed correction vượt bound |
| LIO reset | Epoch/jump/health transition, queued command cũ, counter wrap/reboot; không dùng reference sai epoch |
| Yaw | WP2->3->4 ngắn, wrap, quay tại chỗ, estimator yaw drift/noise, identity updates không dùng heading cũ |
| Role/terminal | MAIN->BACKUP->EMERGENCY->hold, stop/dwell và toàn bộ WP; không đổi frame policy giữa handoff |
| Quality | LIO dropout/degeneracy/height drift, GPS reacquisition, disagreement common-mode; không coi estimator khớp là success |
| Load | Planner bận, diagnostic queue đầy, delayed/reordered message; không blocking hay thêm latency tails không giới hạn |

Thứ tự commit/test:

1. Instrumentation + clock/reference contract, zero behavior change; ghi ledger.
2. Pure adapter + unit/integration fault tests, shadow only, cùng config EKF.
3. Level A opt-in SITL sau khi timing/reset evidence có ý nghĩa; ledger ghi
   owner/scope/impact/removal/verification cùng behavior commit.
4. So sánh off/shadow/Level A trên cùng yaw/planner build, map/route/seed,
   request/config/effective speed, adaptive policy và fusion config. Không
   sửa yaw/planner/gates/EKF trong cùng cặp A/B. GPS-on/off thành strata riêng.
   Đi từ open-map 3/5 m/s đến 8/12 và obstacle maps khi gate trước đạt.
5. Chỉ xét Level B nếu velocity disagreement persistent liên quan tracking
   sau khi loại timing/saturation; cap/slew và controller filtering phải được
   mô hình hóa. EKF trust test là campaign khác.

Acceptance gồm full mission/đủ waypoint/terminal dwell, truth tracking và
clearance, saturation/dynamics, brake/solve-fail attribution, latency tails,
reset transient và health. Repeated runs theo từng map/speed, không gộp PASS
map dễ để che map khó. 12 m/s là ceiling đã cấu hình, không chứng minh vehicle
đạt 12 trên leg ngắn. Không dùng PX4≈LIO làm flight acceptance.

## 8. Kết luận review

Level A đáng thử trước velocity-first và EKF trust changes. Đã xác định seam,
phương trình, ownership và bộ test. Ba điều phải đóng bằng evidence trước
active high-speed obstacle campaign: tuổi raw anchor tới FC, thứ tự reset
qua DDS, và continuity/certificate của mọi role. Thiết kế không tự giải quyết
nominal solve failure hoặc LIO height mất quan sát. Không có production code,
parameter hay safety gate nào được thay đổi bởi tài liệu này.
