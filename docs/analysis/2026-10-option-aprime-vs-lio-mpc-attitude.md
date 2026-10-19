# So sánh A′ với B′ (LIO + MPC qua giao diện attitude, EKF2 chạy độc lập) (2026-10-19)

Câu hỏi của owner: dùng LIO cho toàn bộ trạng thái, một bộ điều khiển MPC gửi **attitude + thrust** cho PX4, còn EKF2 và
PX4 vẫn chạy **độc lập**. Theo lời owner, đây là hướng an toàn, nhưng nó không giải được trường hợp LIO hỏng: khi đó quyền
được trả lại cho PX4, nên mode né vật cản chỉ dùng được ở môi trường LIO có dữ liệu ổn định. Báo cáo so B′ với A′ hiện tại
(đã cải tiến theo O39) trên **mọi** tiêu chí đã nêu. Hai báo cáo trước đã so A′ với B ở dạng chung:
[2026-10-t-sync-vs-lio-frame-control.md](2026-10-t-sync-vs-lio-frame-control.md) (O36) và
[2026-10-t-consistency-math-review.md](2026-10-t-consistency-math-review.md) (O38, O39). Báo cáo này không lặp lại hai bản
đó. Phần mới gồm: hai biến thể của "EKF2 độc lập", việc bàn giao lại cho PX4 có số đo, và hành vi PX4 v1.17 với một external
mode chỉ có attitude.

Phạm vi: chỉ đọc. Không chạy SITL, không đổi `src/`, `tools/`. Mọi số có nguồn (`file:dòng` trong PX4 v1.17.0-14 ở
`~/Dev/Autopilot`, px4_ros2 ở `src/external/px4_ros2_interface_lib/px4_ros2_cpp`, run id dưới `log/sitl/`, hoặc script ở
mục 9). Điều không kiểm được ghi **UNVERIFIED**. Số của mục 4 và mục 6 là **mô hình**.

## 0. Kết luận ngắn

1. **Giữ A′ bây giờ** (D10, O36). B′-E1 bị **loại**: EKF2 vẫn đi theo LIO, nên B′-E1 giữ mọi ghép nối của A′ (O38, `T`
   cho mission và BACKUP) rồi đặt thêm một vòng bay flight-critical trên companion. Không có tiêu chí nào B′-E1 hơn A′ mà
   A′ không đạt được rẻ hơn.
2. **B′-E2 chỉ có nghĩa ở profile gps.** Ở đó nó là bố cục chuẩn REP-105: điều khiển trên odom (LIO), GNSS chỉ dùng cho
   goal. Nó cũng đưa GNSS, lệch đồng hồ ghép cặp và sàn σ EV ra khỏi vòng điều khiển, và để lại cho PX4 một bộ ước lượng
   thứ hai không bị LIO làm bẩn. Ở no_gps, B′-E2 **không chấp nhận được**: PX4 không bao giờ có vị trí ngang, nên khi bàn
   giao chỉ còn Descend. Descend giữ xe nằm ngang mà không phanh, nên xe trôi theo quán tính: 14.7 m tới khi chạm đất nếu
   đang bay 5 m/s ở 2.5 m (drag của mô hình SITL).
3. **B′ không bỏ được giới hạn D46** và **thu hẹp** vùng dùng được của mode. A′ bay tiếp qua những lần mất LIO ngắn
   (BACKUP qua `T` đóng băng; gps tới 10 s, no_gps tới khoảng 5 s). Với B′ như owner mô tả, mỗi lần LOST là hết mode.
   Cái B′ đổi được là sai số khi LIO khoẻ: K1 = 0 trên đường lệnh. Nó cũng làm sự phụ thuộc thành tường minh: chất lượng
   điều khiển bằng đúng chất lượng LIO.
4. **Bàn giao lại cho PX4 lúc đang bay nhanh là chỗ yếu nhất của B′.**
   - Lúc bàn giao, PX4 chỉ phanh theo đường thẳng, không có chứng nhận. Ở 5 m/s, quãng từ lúc LIO hỏng tới lúc dừng là
     8.8–14.4 m, so với ≤ 9.2 m theo BACKUP đã chứng nhận của A′.
   - PX4 không có timeout cho setpoint của external mode. Khi luồng attitude bị treo, xe tăng tốc vòng hở trong
     khoảng 0.9–1.2 s rồi PX4 mới coi mode là "unresponsive".
5. **Một phát hiện chung cho cả hai phương án (đã kiểm trên ULog):** ở no_gps, AUTO_LOITER không chạy được vì
   `global_position_invalid` = 1 suốt chuyến bay. Như vậy quy tắc §4.2 "PX4 còn vị trí → AUTO_LOITER" không áp được ở
   no_gps.
6. **Hybrid tốt hơn hẳn chưa cần xây.** Lấy giá trị "bộ phát hiện" của B′-E2 trong A′ bằng một con số sẵn có: độ trôi
   của `T` tại xe kể từ INIT (gps, DC gain 1). Không thêm trạng thái. B′-E2 (chỉ ở gps) chỉ mở theo trigger đo được ở
   mục 7.

## 1. Định nghĩa các phương án

| | A′ (hiện tại + O39) | B′-E1 | B′-E2 |
|---|---|---|---|
| Vòng vị trí/vận tốc | PX4 (`mc_pos_control`) trên EKF2, setpoint P/V/A biến đổi qua `T` | MPC trên `/lio/state` (companion) | như E1 |
| Giao diện PX4 | `TrajectorySetpoint` | `VehicleAttitudeSetpoint` (q_d, thrust_body ∈ [−1, 1]) | như E1 |
| EKF2 | fuse EV (D42, `EKF2_EV_CTRL=15`) | fuse EV như hôm nay, **không độc lập** | không EV; gps: GNSS/baro/mag/IMU; no_gps: chỉ IMU/baro/mag |
| `T` | trên đường lệnh 50 Hz, mission (D19), BACKUP (D11) | mission, BACKUP, và lệch yaw cho q_d | gps: mission + bộ phát hiện; no_gps: không có `T` nào đo được |
| LIO hỏng | timeline tự chạy hết BACKUP qua `T` FROZEN, PX4 thực thi | trả quyền cho PX4 (lời owner) | như E1; no_gps chỉ còn Descend |

**Ghép nối của E1.** E1 không phải "EKF2 độc lập":
- ở no_gps, EKF2 ≡ LIO tới vài mm (σ EV mà EKF2 dùng là 0.100 m, LIO gửi 1.30 mm; `T` ≈ phép đồng nhất; O36);
- ở gps, EKF2 là trung bình có trọng số GNSS–EV (σ 0.5 m mỗi nguồn), bias EV kéo DC về GNSS (H_G DC = 1, math review §1);
- EV yaw được fuse, nên compass không được fuse (`mag_control.cpp`, §4.1). Khi LIO mất, EKF2 reset heading về compass
  (O38, −20.47° ở `20261014T062206Z-no_gps-hover-yaw1.2`);
- epoch LIO mới làm vị trí EKF2 nhảy ở no_gps (F33c).

Một lỗi LIO lọt qua cổng EV vào EKF2 trước khi bị phát hiện. Cổng là `EKF2_EVP_GATE` = 5 σ innovation, tức ≥ 2.5 m ở gps
và ≥ 0.5 m ở no_gps (`ekf2/params_external_vision.yaml:60-75`). Vì vậy **bộ ước lượng dự phòng bị nhiễm cùng lỗi với
bộ điều khiển**.

## 2. Bảng so sánh

Đánh giá bằng chữ: **tốt**, **chấp nhận**, **yếu**, **không đạt**. Lý do một dòng; bằng chứng ở mục 3.

| # | Tiêu chí | A′ | B′-E1 | B′-E2 |
|---|---|---|---|---|
| 1a | D44.1 ca xấu (LIO hỏng lúc bay nhanh) | **chấp nhận**: BACKUP đã commit tự chạy, PX4 phanh bằng vị trí của nó | **yếu**: phải phát hiện rồi đổi mode, PX4 phanh thẳng không chứng nhận | gps **yếu** như E1; no_gps **không đạt**: Descend, xe trôi 9–15 m |
| 1b | D46 (7 hàng) | không giải hàng nào (đúng bản chất); hàng 6 dùng cổng nhị phân | không giải hàng nào; hàng 7 (sim lạc quan) nặng hơn | hàng 6 biến mất (không EV); hàng 3 phát hiện sạch hơn ở gps; hàng 1 tệ hơn ở no_gps |
| 1c | D44.3 trung thực thông tin | **chấp nhận**: `T` là hiệu chỉnh map→odom; VALID còn là đếm cặp (O39) | **yếu**: gọi EKF2 là "độc lập" trong khi nó đi theo LIO | **tốt** ở gps; no_gps trung thực nhưng không còn gì để bàn giao |
| 1d | D44.4 không lan liên tầng | **yếu**: cổng → STALE → INVALID → bàn giao (L2/I7), O38 | **yếu**: lỗi LIO vào thẳng vòng bay và cả bộ dự phòng | **chấp nhận**: lỗi LIO vào vòng bay, không vào bộ dự phòng |
| 1e | D44.5/6/7 ít cờ, phạm vi, không thích nghi | **tốt**: `T` 4 trạng thái/0 cờ; mở rộng có budget O39 | **không đạt**: thêm MPC, chuyển mode, ước lượng hover thrust (thích nghi) mà vẫn giữ `T` | **yếu**: như E1, đổi thêm D42/D19 |
| 2 | Toán và logic | **chấp nhận**: I1 xấp xỉ (K1 + K2), L1–L4 đã biết, ổn định vòng `T` chỉ là mô hình | **chấp nhận**: I1 đúng khi LIO khoẻ; hai nguồn attitude trong một vòng; vẫn cần lệch yaw | **tốt** ở gps (REP-105 chuẩn); no_gps D19 không định nghĩa được |
| 3 | Tiêu chí đã có (O36–O39, DV1/DV2) | **tốt**: K1–K4 và budget O39 viết cho A′; O38 cần quyết | **yếu**: O37, DV1 thành flight-critical; O38 vẫn còn | gps: bỏ O38, K1 trên đường lệnh; O37/DV1 vẫn flight-critical |
| 4 | Xử lý lỗi | **chấp nhận**: LOST/DEGRADED, reset PX4 hấp thụ; GNSS nhảy → INVALID (fail-closed, có thể quá tay) | **yếu**: lỗi bị che khỏi PX4 hoặc lan vào nó | gps **chấp nhận**: phát hiện tốt, bàn giao kém; no_gps **không đạt** |
| 5 | Kiến trúc an toàn | **tốt**: vòng flight-critical trên FC; failsafe vị trí của PX4 còn bảo vệ mode | **yếu**: vòng ngoài trên Linux qua DDS; PX4 không kiểm vị trí cho mode attitude; setpoint treo → tăng tốc vòng hở ~1 s | **yếu** (gps), **không đạt** (no_gps) |
| 6 | Hiệu năng bám | **chấp nhận**: mặc định 0.37 m p95 ở 5 m/s; sau chỉnh gain và feedforward drag/trễ 0.09 m (mô hình) | **tốt** về lý thuyết (0.07 m, mô hình) nhưng cần nhận dạng mô hình | như E1 |
| 7 | Công sức, kiểm chứng | **tốt**: S2 đã có kế hoạch; `px4_mode` chưa xây ở cả hai | **yếu**: ≥ một lát cỡ S1b + AR L3; A′ vẫn phải xây làm dự phòng | **yếu**: như E1 + đổi profile và D19 |
| 8 | Mission, frame, BACKUP | **tốt**: D19 qua `T`; BACKUP do PX4 thực thi (D11) | **chấp nhận**: cần `T` như A′; BACKUP phải chọn IMU-only hay `px4_mode` thứ hai | gps **chấp nhận**; no_gps **không đạt**: không có frame mission, không có ai thực thi BACKUP ngoài MPC chạy IMU |
| 9 | Đảo ngược, phân lát | **tốt**: `/nav/command` ở frame LIO; đổi bộ thực thi chỉ chạm `px4_mode` | **chấp nhận**: quay về A′ rẻ nếu đã có setpoint trajectory | **chấp nhận**: E2 là tham số profile (đặt trước khi arm) |

## 3. Bằng chứng theo tiêu chí

### 3.1 D44 và D46

**Ca xấu (D44.1).** Xem mục 4. A′ không cần phát hiện lỗi để bắt đầu phanh. Timeline đã commit có MAIN ≥ 1.0 s rồi
BACKUP tới trạng thái đứng yên (§2.1). Khi LIO LOST, Supervisor chỉ ngừng submit, và timeline tự chạy qua `T` FROZEN
(§2.3). B′ cần chuỗi: phát hiện, quyết định, đổi mode, rồi PX4 mới phanh.

**D46 từng hàng:**

| Hàng D46 | A′ | B′-E1 | B′-E2 |
|---|---|---|---|
| 1 no_gps + mất hình học | BACKUP qua `T` (nếu O38 (a)), PX4 dead-reckon ≤ 5 s rồi failsafe khi xe gần đứng yên | như A′ nếu bàn giao ngay (EKF2 có vận tốc đã học từ EV) | Descend ngay ở tốc độ bay, trừ khi MPC tự phanh bằng IMU |
| 2 LIO trôi chậm, no_gps | không thấy | không thấy | vị trí: không thấy. **Yaw: thấy được** nhờ compass độc lập (EV yaw không fuse); bị nhiễu từ trường gần kết cấu thép (UNVERIFIED) |
| 3 khớp sai tự tin | gps: cổng `T` 0.5 m → STALE → bàn giao; EKF2 loại EV ở 2.5 m | gps: test ratio EV của EKF2; điều khiển bỏ qua | gps: residual GNSS−LIO đầy đủ băng thông, ngưỡng theo σ GNSS (thật 1–3 m, UNVERIFIED) |
| 4 liên tục frame qua restart | `T` INIT theo epoch (D47) | như A′ | gps như A′; no_gps không có tham chiếu ngoài IMU (như A′ về thông tin) |
| 5 mép hình học | như nhau | như nhau | như nhau |
| 6 covariance EV | cổng nhị phân TRACKING | như A′ | **không áp dụng** (không có EV) |
| 7 sim lạc quan | ước lượng lạc quan | **thêm**: mô hình động lực (hover thrust 0.785, drag k ≈ 0.11 1/s, motor 12.5/25 ms) lý tưởng nằm trong vòng | như E1 |

**Trung thực thông tin.** B′ không tạo thông tin giả, nhưng E1 gọi một bộ ước lượng đi theo LIO là "độc lập". Ở no_gps,
E2 trung thực tới mức không còn gì: PX4 không có vị trí ngang. Đó đúng là hàng 1 của D46, chỉ xảy ra **sớm hơn** (ngay lúc
LIO hỏng) và ở **tốc độ cao hơn**.

**Lan liên tầng.** Ở A′, chuỗi cổng → STALE → INVALID → bàn giao đi vòng qua Supervisor (math review L2, I7), và O38 đã
được ghi. Ở B′, sai số LIO đi thẳng vào tầng flight-critical. Riêng E1 còn nhiễm vào bộ dự phòng (common-mode).

**Ít cờ, phạm vi, không thích nghi.** B′ thêm:
- thư viện MPC;
- state machine bàn giao và quay lại (ít nhất MPC_ACTIVE / BRAKING / HANDED_BACK);
- hai kiểu setpoint trong `px4_mode`;
- lệch yaw cho q_d.

`T` vẫn còn. Lực đẩy chuẩn hoá cần hover thrust, mà PX4 ước lượng nội bộ nhưng không xuất qua DDS:
`uxrce_dds_client/dds_topics.yaml` không có `hover_thrust_estimate`. Có hai cách lấy nó: tự ước lượng online, đó là cơ chế
thích nghi (trái D44.7); hoặc hằng số đã nhận dạng cộng tích phân. Beta là 1–5 m/s, và D10 đã định B chỉ mở khi A′ trượt
O4. Vì vậy B′ lúc này vượt nhu cầu (D44.6).

### 3.2 Toán và logic

- **Một nguồn cho mỗi đại lượng.**
  - A′: vị trí điều khiển là EKF2, vị trí kế hoạch là LIO; `T` nối hai nguồn một cách tường minh.
  - B′: vị trí do LIO. **Attitude có hai nguồn:** MPC tính hướng lực đẩy trong frame LIO, PX4 thực hiện q_d trên
    attitude EKF2. Nghiêng khác nhau ε ≈ b_acc/g thành gia tốc ngang sai g·ε (b = 0.05 m/s² cho 0.05 m/s²), và tích phân
    của MPC phải hấp thụ phần này.
  - Yaw của q_d phải ở heading EKF2. Đây là một `T` 1-DoF trong vòng 100 Hz, kèm cuộc đua reset heading giống L3:
    PX4 chỉ chỉnh setpoint cũ hơn ước lượng (`mc_att_control_main.cpp:321-338`).
- **Bất biến I1.** B′ đạt chính xác khi LIO khoẻ (K1 = 0). A′ đạt xấp xỉ (math review §3). I2 (danh tính frame) vẫn cần ở
  B′ cho mission và BACKUP.
- **Cơ chế ẩn.** `T` hấp thụ lệch đồng hồ ghép cặp (8–15 ms × v đo ở gps; 40–75 mm ở 5 m/s là ngoại suy): B′ bỏ nó
  khỏi đường lệnh. Sàn σ EV: E2 bỏ. FROZEN: B′ chỉ còn dùng cho BACKUP qua PX4. Cổng cố định: E2 thay bằng bộ phát hiện,
  có thể dùng Mahalanobis (hướng của O39).
- **REP-105.** Math review §4 đã ghi điểm khác thường của A′: bộ điều khiển chạy trên "map" (EKF2, có GNSS kéo). B′-E2 ở
  gps là bố cục chuẩn: điều khiển trên odom, global cho goal. Đây là ưu điểm khái niệm thật của B′-E2. Ở no_gps, A′ cũng
  đã chạy trên odom về thực chất, vì EKF2 ≡ LIO và `T` ≈ I.
- **Ổn định vòng kín.** Vòng `T` của A′ có sup|L| ≤ 0.29 trong mô hình. B′ ổn định trong mô hình mục 6 với trễ DDS tới
  50 ms. Cả hai chỉ là mô hình.

### 3.3 Tiêu chí đã định

- **O36.** K1–K4 viết cho A′.
  - Mô hình mục 6 cho biết nhánh "K2 trội" có một bước rẻ chưa được ghi trước khi mở B: thêm feedforward drag (k·v)
    và bù trễ attitude (lấy mẫu gia tốc sớm khoảng 0.1 s) trong sampler, cùng một vòng chỉnh `MPC_XY_P`/`VEL_*`.
  - K3 (≤ 0.227 m p95) áp như nhau cho B′.
- **O39.** Budget của `T` là 4 trạng thái, 8 cạnh, 5 tham số, 800 dòng. B′ không thay `T` mà cộng thêm một hệ con lớn hơn
  budget đó. B′ là "phương án mức 2" của O39, nên chỉ mở theo quy tắc dừng (b) hoặc (c).
- **O38.**
  - E1 vẫn mắc lỗi này: reset heading khi LIO mất làm hỏng `T` FROZEN dùng cho BACKUP qua PX4, và làm nhảy lệch yaw của
    q_d đúng lúc bàn giao.
  - E2 không mắc: không fuse EV yaw nên compass luôn là nguồn heading.
- **O37 và DV1.** Ở B′, DEGRADED nghĩa là MPC đang chạy trên predictor chỉ có IMU. Vì vậy "DEGRADED không có lối ra"
  (O37) và dao động TRACKING↔DEGRADED (DV1) trở thành lỗi **flight-critical**: chuyển mode liên tục, trái P10.
- **DV2.** Ở B′, DV2 ra khỏi đường lệnh nhưng vẫn ở `T` của mission.

### 3.4 Xử lý lỗi

| Lỗi | A′ | B′-E1 | B′-E2 |
|---|---|---|---|
| LIO DEGRADED/LOST | phát hiện; BACKUP qua `T` FROZEN | phát hiện; MPC chạy IMU-only 0.5–1.3 s rồi bàn giao | như E1; no_gps Descend |
| Restart LIO (D38, D41, D47) | `T` INIT, Supervisor REACQUIRING | MPC chờ hay quay lại mode sau Loiter (hai lần đổi mode mỗi episode) | như E1; no_gps: frame mission theo predictor |
| GNSS nhảy hoặc trôi (gps) | **lan** vào vòng PX4, `T` bám ≤ 0.25 m/s; nhanh hơn thì INVALID → bàn giao | **bị che** khỏi điều khiển; EKF2 lệch | **phát hiện** qua residual; không vào điều khiển |
| Reset EKF2 | hấp thụ theo G; FROZEN → INVALID (O38) | lệch yaw q_d phải đổi; BACKUP qua PX4 như A′ | gps: reset GNSS chỉ đổi `T` mission |
| Lệch đồng hồ | `T` hấp thụ (sai) | không vào vòng | không vào vòng |
| Bias IMU, rung | EKF2 và LIO đều ước lượng bias; sim không có | MPC dùng LIO trực tiếp; nghiêng ε vào vòng | như E1 |
| Cấu trúc lặp | xem D46 hàng 3 | như trên | như trên |
| Vật động | như nhau (LIO, map) | như nhau | như nhau |
| Mất scan qua Ethernet | = gap LiDAR | = gap, vòng chạy IMU-only tới khi phát hiện | như E1 |
| Process `navigation`/`px4_mode` chết | PX4 giữ setpoint cuối: vòng vị trí vẫn đóng, dừng ở khoảng P + V/`MPC_XY_P`; unresponsive sau ~1 s | PX4 giữ attitude cuối: **vòng hở**, gia tốc g·tan(tilt) tới khi unresponsive | như E1 |

Ghi chú cho dòng cuối. PX4 không có timeout setpoint cho external mode: tín hiệu offboard chỉ gắn với
`NAVIGATION_STATE_OFFBOARD` (`commander/ModeUtil/mode_requirements.cpp:158`); `mc_pos_control` không kiểm tuổi setpoint
(`MulticopterPositionControl.cpp:443-515`), `mc_att_control` chỉ cần setpoint mới hơn (`mc_att_control_main.cpp:310-319`).
Bộ phát hiện duy nhất là arming-check reply: hỏi mỗi 300 ms, 3 lần không trả lời thì unresponsive
(`HealthAndArmingChecks/checks/externalChecks.hpp:69-75`, `.cpp:253-283`), tức khoảng 0.9–1.2 s. Nếu reply vẫn chạy
trên executor còn sống trong khi luồng MPC treo, PX4 không bao giờ phát hiện (UNVERIFIED với cách chia thread sẽ xây).

| Tilt khi treo | 10° | 20° | 30° |
|---|---|---|---|
| Sau 1.0 s vòng hở: Δv / Δp | 1.7 m/s / 0.87 m | 3.6 m/s / 1.79 m | 5.7 m/s / 2.83 m |

### 3.5 Kiến trúc an toàn

- **Ai đóng vòng flight-critical.**
  - A′: rate, attitude, vận tốc và vị trí đều trên FC (PX4, RTOS); companion chỉ gửi tham chiếu 50 Hz với lease 100 ms.
  - B′: vòng vị trí và vận tốc trên Linux không realtime, qua hai chặng DDS (LIO → MPC → uXRCE-DDS → PX4), 100 Hz
    (`experimental/attitude.hpp:30`).
  - Số đo có trong log: `/lio/state` có dt wall p99 15.3 ms, max 16 ms ở hover (RTF 0.80; `jitter.py` trên
    `20261014T061438Z`, `20261014T062206Z`, `20261013T152418Z`). Đuôi khi CPU tải ICP 20–50 ms và planner 80 ms:
    **UNVERIFIED**. Log lưu theo sim time 4 ms nên không tách được trễ DDS.
- **Failsafe PX4 còn bảo vệ gì.**
  - Mode trajectory đòi `local_position` (`trajectory.cpp:88-100` → `mode.cpp:257-273`). Mất vị trí thì mode không
    chạy được và PX4 tự fallback RTL → Land → Descend (`failsafe.cpp:698-702`, `framework.cpp:576-605`), không cần
    process của mình.
  - Mode attitude chỉ đòi attitude và angular velocity (`attitude.cpp:73-82`). PX4 **không** failsafe khi mất vị trí,
    nên B′ phải tự bàn giao.
  - Gộp hai kiểu setpoint trong một mode thì yêu cầu là OR (`mode.cpp:257-273`). E2-no_gps không arm được mode có kiểu
    trajectory, trừ khi đặt `local_position_is_optional` (`trajectory.cpp:98`).
- **Quyền và bàn giao (D11, Reason).** A′ đã có Reason (`LIO_RECOVERY_TIMEOUT`, `PX4_POSITION_LOST`,
  `ALIGNMENT_INVALIDATED`, `COMMAND_STREAM_LOST`). B′ thêm một Reason mới cho mỗi lần LIO hỏng (`LIO_LOST_HANDBACK`).
  Đó là bàn giao **thường xuyên**, trái tinh thần D11 ("chỉ khi hết timeout hoặc mất khả năng tiếp tục").
- **Vùng dùng được của mode, định nghĩa đo được.** Gọi `E_LIO` là tập đoạn bay có:
  - (i) LIO TRACKING liên tục, không LOST;
  - (ii) trị riêng nhỏ nhất ≥ 1.5 × ngưỡng (1.65e5 tịnh tiến, 4.2e6 xoay; biên hover đo được là 2.5× và 2×, D36)
    trên ≥ 99% scan;
  - (iii) gap scan p99 < 0.15 s.

  Với định nghĩa này:
  - **B′** = `E_LIO`, hoặc `E_LIO` cộng các lỗ ≤ thời gian phanh IMU-only 1.3–2.7 s nếu MPC tự phanh.
  - **A′** = `E_LIO` ∪ các lỗ có LOST ≤ ngân sách bỏ cuộc (gps 10 s, D11; no_gps ≤ `EKF2_NOAID_TOUT` 5 s) và BACKUP còn
    hiệu lực (D3, D4). Ở gps còn thêm điều kiện bất đồng GNSS–LIO tại xe < 0.25 m/s (L2).

  Vậy B′ có vùng dùng được **nhỏ hơn hẳn** ở gps, và nhỏ hơn hoặc bằng ở no_gps.

### 3.6 Hiệu năng

Mô hình một trục (`tracking.py`, `tracking2.py`):
- tham chiếu tăng tốc 3 m/s², jerk 4;
- drag k = 0.112 1/s, tính từ x500 SITL (`rotorDragCoefficient` 8.06428e-05, `motorConstant` 8.54858e-06, khối lượng
  2.33 kg);
- trễ attitude 0.1 s **giả định**;
- A′ dùng gain PX4 v1.17 mặc định (`MPC_XY_P` 0.95, `VEL` 1.8/0.4/0.2). Sai số p95 (m):

| v (m/s) | A′ mặc định | A′ + drag ff + bù trễ 0.1 s | A′ chỉnh gain (2/3/1) + drag + bù trễ | B′ (trễ DDS 20 ms) | B′ drag sai 50% |
|---|---|---|---|---|---|
| 1 | 0.090 | 0.071 | 0.038 | 0.036 | 0.036 |
| 3 | 0.248 | 0.184 | 0.083 | 0.065 | 0.066 |
| 5 | 0.372 | 0.262 | 0.093 | 0.070 | 0.064 |
| 10 | 0.540 | 0.329 | 0.104 | 0.076 | 0.061 |

- **Đọc bảng.**
  - Với gain mặc định, A′ trượt K3 (0.227 m) từ khoảng 3 m/s trong mô hình.
  - Gần hết khoảng cách tới B′ là gain và feedforward, không phải kiến trúc. Cả hai thứ này làm được trong A′, ở sampler
    và trong `MPC_*` đặt trước khi arm.
  - Có gust 1 m/s² trong 2 s: A′ đã chỉnh cho 0.165 m ở 5 m/s, B′ cho 0.202 m. Tích phân của B′ trong mô hình yếu hơn,
    nên đây là so sánh tuning, không phải kết luận.
  - Với k = 0.3 và trễ 0.15 s ở 10 m/s: A′ mặc định 1.13 m, A′ + drag 0.43 m, A′ chỉnh gain + drag + bù trễ 0.15 s
    0.109 m, B′ 0.113 m. Mô hình này không có bão hoà tilt hay thrust, nên lợi thế thật của MPC (ràng buộc, preview) ở
    8–10 m/s không hiện ra; phần đó chỉ đo được.
- **MPC cần những gì:**
  - nhận dạng hover thrust (0.785 ở SITL, `hover_thrust_estimate` trong ULog `20261014T061438Z`);
  - đường cong thrust và sụt áp pin;
  - drag và đáp ứng attitude;
  - solver realtime trên companion;
  - giới hạn tilt và thrust khớp với certifier (D23).

  Nhận dạng làm trong sim không chuyển sang được phần cứng (D46 hàng 7).
- **Giới hạn của PX4 (P4).** `PositionControl.cpp:124-152` là P → PID cộng feedforward V/A, không có mô hình drag và
  không có preview. P4 trên `main` xảy ra với velocity-only (O36 §1), chưa bao giờ với P/V/A qua `T`.

### 3.7 Công sức và kiểm chứng

- **A′:** S2 đã lên kế hoạch (`px4_mode` trajectory, Supervisor tối thiểu, sampler, O38). `px4_mode` chưa xây ở cả hai
  phương án (`docs/architecture/AS_BUILT.md:77`), nên không có chi phí chìm. `/nav/command` đã ở frame LIO (D17), nên
  B′ dùng thẳng được.
- **B′ thêm (ước lượng, UNVERIFIED), ≥ một lát cỡ S1b:** thư viện điều khiển thuần và test; công cụ nhận dạng từ ULog;
  kiểu setpoint attitude (namespace `experimental`) cùng lệch yaw và xử lý reset heading; state machine bàn giao, quay
  lại và BACKUP; watchdog tự viết (PX4 không có); khớp certifier với giới hạn của MPC; với E2 thêm profile
  `EKF2_EV_CTRL=0`, bộ phát hiện, sửa D42 và D19; AR mức L3 (D10, D11, §4.2); khoảng 20–30 lần SITL.
- A′ **vẫn phải xây** làm bộ thực thi BACKUP ở E1 và E2-gps. Đây là cùng kết luận "B = A′ + một vòng mới" của O36.
- **SITL kiểm được:** bám 1/3/5 m/s, lidar_cut (S1c Task 5), giết process, treo luồng setpoint. **Không kiểm được:**
  GNSS thật (PX4 bỏ qua lệnh tiêm GNSS, O36), bias IMU, rung, sụt áp pin, đuôi DDS trên link thật, nhiễu từ.
  Rủi ro đường dài của B′ cao: phần lớn giá trị của nó (GNSS ngoài vòng, 10 m/s) chỉ kiểm được trên phần cứng.

### 3.8 Mission, frame, MAIN/BACKUP

- **gps (D19).** Waypoint GPS đổi sang LIO qua `T` ở cả ba phương án. Ở E2, `T` là gauge chậm giữa LIO và EKF2 chỉ có
  GNSS: nhiễu mét, nhưng sai số waypoint hội tụ khi xe tới gần (I5).
- **no_gps.** Frame mission là local PX4 (D19). Ở E2 frame này không tồn tại, nên phải đổi D19 sang frame LIO epoch 0;
  sau restart, liên tục chỉ còn theo predictor (D46 hàng 4).
- **BACKUP sau khi LIO hỏng (D2, D11, §2.1).** B′ phải chọn một trong ba:
  - (i) MPC thực thi BACKUP trên predictor chỉ có IMU. Sai số 0.12/0.27/0.48 m ở 1/3/5 m/s với bias 0.05 m/s² và lỗi
    nghiêng 0.5° (`handback.py`). Trung thực khi giới hạn đúng bằng thời gian phanh, nhưng sim không có bias;
  - (ii) PX4 thực thi qua `T` FROZEN: đó chính là A′, cần kiểu setpoint thứ hai; ở E2-no_gps không làm được;
  - (iii) bàn giao ngay: phanh thẳng không chứng nhận, trái D11.

  Lời owner ("trả mode lại cho PX4") là (iii). Đề xuất tối thiểu nếu B′ được mở là (i) rồi mới bàn giao khi đứng yên.
- **MAIN** không đổi nghĩa. Supervisor (§2) vẫn là nơi ghi duy nhất; `phase()` vẫn là hàm dẫn xuất.

### 3.9 Đảo ngược và phân lát

- **A′ → B′:** rẻ ở mức kiến trúc. `/nav/command` ở frame LIO, chỉ `px4_mode` đổi. E2 là tham số profile đặt trước khi
  arm (không đổi khi bay).
- **B′ → A′:** rẻ nếu `px4_mode` đã có kiểu trajectory.
- **Phải đo trước (S2):** K1–K4 theo O36; K2 tách gain/feedforward; thí nghiệm BACKUP ba hành vi (math review §7);
  lidar_cut no_gps (O38).

## 4. Bàn giao lại cho PX4: số

Mô hình `handback.py`, tham số PX4 v1.17 mặc định:
- `MPC_ACC_HOR` 3, `MPC_JERK_AUTO` 4 (`multicopter_autonomous_params.c:88,103`);
- điểm dừng chiếu trước `v·(v/2a + 0.6)` (`navigator_main.cpp:1586-1599`, `TrajMath.hpp:107`);
- Loiter của multicopter là "current position with braking" (`loiter.cpp:111-113`).

Thời gian phát hiện theo bảng ngân sách §3.4: gap 0.5 s, suy biến 0.3 + 1.0 s. Thời gian đổi mode 50–150 ms giả định,
gồm tick, DDS và vòng Commander 10 ms (`Commander.hpp:217`); UNVERIFIED.

| v (m/s) | B′ → Loiter (gps), gap / suy biến | A′ BACKUP đã commit (xấu nhất / BACKUP đã bắt đầu) | Descend ở 2.5 m AGL, k = 0.11 / 0.30 / 0.50 |
|---|---|---|---|
| 1 | 1.1–1.4 m / 1.9–2.2 m, 1.6 / 2.5 s | 1.2 m / 0.2 m, đã chứng nhận | 2.9 / 2.2 / 1.7 m |
| 3 | 4.3–5.2 m / 6.7–7.7 m, 2.4 / 3.2 s | 4.5 m / 1.5 m | 8.8 / 6.6 / 5.0 m |
| 5 | 8.8–10.4 m / 12.8–14.4 m, 3.1 / 3.9 s | 9.2 m / 4.2 m | 14.7 / 11.0 / 8.3 m; chạm đất ở 3.35 / 1.71 / 0.84 m/s |

**Đọc bảng.**
- **Quãng đường gần bằng nhau, nhưng chất khác hẳn.**
  - Quãng của A′ là phần còn lại của một quỹ đạo **đã chứng nhận known-free** trong map LIO (có thể cong quanh vật cản).
    Nó được thực thi bằng ước lượng PX4 qua `T` đóng băng, với ngân sách sai số 0.15–0.76 m (math review §7, mô hình).
  - Quãng của B′ là đường thẳng theo hướng vận tốc, không ai kiểm va chạm. Ở 5 m/s, 9–14 m đường thẳng gần vật cản
    là va chạm khi quỹ đạo đang rẽ.
- **Tính khả dụng của PX4 khi đang nhận bàn giao.**
  - gps: Loiter chạy được.
  - **no_gps:** AUTO_LOITER đòi global position ở multicopter (`mode_requirements.cpp:119-120`), mà ULog
    `20261014T062206Z-no_gps-hover-yaw1.2` có `failsafe_flags.global_position_invalid` = 1 suốt chuyến và không có topic
    `vehicle_global_position`. Bit `mode_req_global_position` = 56 gồm mission, loiter và RTL. **Đã kiểm.**
  - Ở no_gps, thứ còn lại là Land, đòi `local_alt` và `local_position_relaxed` (`mode_requirements.cpp:175-180`). Land
    chiếu điểm dừng bằng lat/lon (`land.cpp:63`), nên hành vi không có global origin là UNVERIFIED. Sau 5 s
    (`EKF2_NOAID_TOUT`, `ekf2/module.yaml:76-86`) chỉ còn Descend.
  - **Phát hiện này áp cho cả A′:** §4.2 "PX4 còn vị trí: AUTO_LOITER qua mode executor" sẽ bị từ chối ở no_gps.
- **Descend không phanh ngang.** `FlightTaskDescend.cpp:72` đặt gia tốc ngang 0 ("stay level"), còn trục dọc dùng
  `MPC_LAND_SPEED` 0.7 m/s (dòng 52) hoặc 0.3 m/s² khi không có vận tốc dọc (dòng 58).
  - Ở B′-E2-no_gps, xe trôi bằng quán tính ở tốc độ bay.
  - Ở A′ và B′-E1, Descend chỉ tới sau khoảng 5 s dead-reckoning, lúc BACKUP đã đưa xe về gần đứng yên.
- **Ngân sách thời gian.** No_gps có khoảng 5 s trước khi PX4 mất vị trí. A′ dùng chúng để phanh có chứng nhận. B′-E1
  dùng 0.5–1.3 s để phát hiện; B′-E2 không có ngân sách nào.
- **Giá trị "kiểm chéo".**
  - Ở E2-gps, residual `r = p_px4 − T_slow(p_lio)` là một **bộ phát hiện** với nhiễu biết trước (σ GNSS). Ở mức 3σ với
    GNSS thật 1–3 m (UNVERIFIED), nó chỉ bắt được lỗi LIO thô (3–9 m): khớp sai, trôi dài. Nó không bắt được trôi 1 m
    trong khe hở 1 m.
  - A′-gps đã có cùng thông tin. `T` tại xe trôi so với lúc INIT đúng bằng bất đồng GNSS − LIO tích luỹ, với DC gain 1
    (math review §1), và giá trị đó đã ở `/alignment`.
  - Khác biệt duy nhất: ở A′ cùng con số đó còn được dùng làm **bộ sửa** (lệnh bám nó), còn E2 chỉ dùng làm bộ phát hiện.
  - Ở no_gps không có kiểm chéo vị trí trong mọi phương án. E2 thêm kiểm chéo yaw (compass).

## 5. Điều chỉ là mô hình hoặc chưa kiểm

- **Mô hình:** toàn bộ mục 4 (thời gian đổi mode, quãng phanh thật của Loiter và Land, drag thật); bảng mục 3.6 (một
  trục, trễ attitude 0.1 s giả định, gain của B′ chọn tay); Δv khi treo setpoint; ngân sách phanh IMU-only; ổn định vòng
  của cả hai.
- **UNVERIFIED:** hành vi Land ở no_gps không có global origin; PX4 có phát hiện luồng MPC treo khi executor còn sống
  không (phụ thuộc reply arming-check và MPC có dùng chung executor); đuôi trễ DDS khi CPU tải; σ GNSS thật và multipath;
  nhiễu từ cho kiểm chéo yaw; công sức của B′; xuất `hover_thrust_estimate` qua DDS bằng cách sửa `dds_topics.yaml`
  (đổi firmware).
- **Đã kiểm:** AUTO_LOITER không chạy được ở no_gps (ULog `failsafe_flags`); yêu cầu của mode attitude và mode
  trajectory (code px4_ros2); không có timeout setpoint cho external mode và Descend không phanh ngang (code PX4); hover
  thrust 0.785 và drag của x500 (ULog, SDF); jitter `/lio/state` (log).

## 6. Phán quyết

**(a) Bây giờ: A′.** Làm S2 như kế hoạch, cùng O38 (a) và O39. Thêm vào nhánh K2 của O36 bước rẻ đặt **trước** khi mở B:
feedforward drag và bù trễ attitude trong sampler, cùng một vòng `MPC_*` có ghi lại. Đây là chính sách của sampler theo
quy tắc chống phình §2.2, không phải trạng thái mới.

**(b) B′-E1: loại.** Nó bị A′ trội: cùng ghép nối EKF2–LIO, cùng `T`, cùng O38, cộng thêm một vòng bay trên Linux và các
lần bàn giao không chứng nhận.

**(c) B′-E2 chỉ cho gps, là phương án mức 2 của O39.** Chỉ mở khi có ít nhất một trigger đo được:
- **T1:** S2 cho K3 > 0.227 m p95 ở 3–5 m/s, và K2 vẫn trội sau bước (a);
- **T2:** trên phần cứng gps, K4 > 0 mỗi giờ do GNSS (không do DV2), hoặc K1 > 0.05 m p95 do GNSS trôi > 0.25 m/s;
- **T3:** chuyển sang profile khéo léo hoặc 8–10 m/s (O7), và K2 đo được vượt nửa khe hở certificate khi tilt hoặc
  thrust bão hoà, thứ mà mô hình mục 3.6 không có.

Khi mở, phải kèm: BACKUP do MPC thực thi trên predictor IMU-only, giới hạn bằng thời gian phanh, bàn giao khi đứng yên;
watchdog setpoint của chính mình (PX4 không có); `px4_mode` giữ kiểu trajectory làm dự phòng (A′). **No_gps không bao
giờ dùng E2.**

**(d) B′ không đổi giới hạn "chỉ dùng ở nơi LIO có dữ liệu ổn định".** Giới hạn đó là D46 hàng 1–3, không phụ thuộc bộ
điều khiển, và B′ còn **thu hẹp** vùng dùng được (mục 3.5). B′ đổi được: sai số khi LIO khoẻ (K1 = 0, không lệch đồng hồ
ghép cặp); GNSS ra khỏi vòng và một bộ dự phòng không nhiễm lỗi LIO (cả hai chỉ ở E2-gps); sự phụ thuộc thành tường minh.

**(e) Hybrid.** A′ cho mode né vật cản ở cả hai profile. Ở gps, dùng `T` tại xe trôi so với lúc INIT (đã có trong
`/alignment`) làm **chỉ số giám sát** trong KPI S2. Không thêm trạng thái hay ngưỡng cho tới khi có số đo phần cứng (O39 d).
Hybrid này lấy được phần "phát hiện" của E2 mà không trả giá phần "bàn giao".

**(f) Thí nghiệm rẻ nhất cho từng điểm mở:**

| Điểm mở | Thí nghiệm | Chi phí |
|---|---|---|
| K2 có sửa được bằng gain và feedforward không | fit k và trễ attitude từ ULog S1c Task 6 (POSCTL 1/3/5 m/s), chạy lại `tracking2.py` với số thật | offline sau Task 6 |
| Bàn giao ở no_gps (Loiter bị chặn, Land không có global) | thêm vào lidar_cut no_gps của Task 5: đọc `failsafe_flags`, `vehicle_status.nav_state` sau khi EV dừng; lệnh Hold thử khi đang hover | SITL, đã có kế hoạch |
| Sàn nhiễu bộ phát hiện E2-gps | một lần hover gps với `EKF2_EV_CTRL=0` (đặt trước khi arm), đo `‖p_px4 − T_slow(p_lio)‖` | 1 lần SITL (lạc quan vì GNSS sim) |
| PX4 phát hiện mode treo | treo luồng setpoint (không treo executor) của một mode hover thử ở S2, đo thời gian tới failsafe | S2, sau khi có `px4_mode` |
| BACKUP: tiếp tục, bàn giao hay phanh | thí nghiệm ba hành vi của math review §7 | S2/S4 |

## 7. Rủi ro của phán quyết

1. **Mô hình bám đánh giá thấp lợi thế của MPC** khi tilt bão hoà ở tốc độ cao, hoặc khi gain PX4 không tăng được trên
   phần cứng vì rung. Khi đó T1 hoặc T3 bật muộn, sau S2. Đường lui: `/nav/command` ở frame LIO giữ chi phí đổi ở mức cục
   bộ.
2. **GNSS thật ở gps làm `T` hỏng thường xuyên** (rủi ro chính của A′, O36). SITL không tái hiện được, nên T2 chỉ bật được
   trên phần cứng. Trước đó A′ có thể bàn giao quá tay (fail-closed).
3. **Bàn giao ở no_gps của chính A′ chưa chạy được như spec.** AUTO_LOITER bị chặn; Land không có global origin chưa rõ.
   Nếu Land cũng không chạy, mọi bàn giao no_gps thành Descend. A′ chỉ an toàn nếu BACKUP đã đưa xe về gần đứng yên
   trước 5 s (O38 phải quyết).
4. **Đánh giá sai công sức của B′** (UNVERIFIED) theo cả hai chiều: một geometric controller đơn giản có thể rẻ hơn MPC.

## 8. Đối chiếu nhanh với lời owner

| Lời owner | Đánh giá |
|---|---|
| "Đây là hướng an toàn" | **Một phần.** An toàn hơn ở chỗ GNSS và `T` ra khỏi vòng khi LIO khoẻ (E2-gps). Kém an toàn hơn ở chỗ vòng bay chạy trên Linux qua DDS, PX4 không kiểm vị trí cho mode attitude, setpoint treo thành vòng hở, và bàn giao thường xuyên không chứng nhận |
| "Không giải trường hợp LIO hỏng, khi đó trả mode cho PX4" | **Đúng.** Thêm: ở no_gps-E2 PX4 không có gì để nhận; ở no_gps-E1 và A′, Loiter bị chặn. BACKUP của D11 cần ai đó thực thi |
| "Chỉ dùng ở môi trường đủ dữ liệu ổn định cho LIO" | **Đúng, và chặt hơn A′.** Định nghĩa `E_LIO` ở mục 3.5 |
| "EKF2 và PX4 chạy độc lập" | E1 **không** độc lập (mục 1). E2 độc lập thật, nhưng chỉ hữu ích ở gps |

## 9. Phương pháp và script

Script trong scratchpad của phiên (`.../scratchpad/bprime/`, không commit), chạy bằng `python3 -I` (riêng
`ulog_probe.py` cần `pyulog` từ user site): `ulog_probe.py <ulg>` (`failsafe_flags`, `vehicle_global_position`,
`hover_thrust_estimate`); `handback.py` (quãng phanh Loiter, BACKUP, quán tính Descend, phanh IMU-only, drag x500 từ SDF);
`tracking.py`, `tracking2.py` (mô hình bám một trục A′ và B′); `jitter.py <run>` (dt wall-clock và sensor của `/lio/state`).
