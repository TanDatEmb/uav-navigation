# Phản biện: đồng bộ `T` giữa LIO và PX4, hay điều khiển riêng trong frame LIO (2026-10-19)

Câu hỏi của owner: cơ chế `T_px4←lio` có đang chứng tỏ là chạy được không; thiết kế có đủ ổn định để duy trì không;
hay nên chuyển sang một bộ điều khiển riêng (thay vòng vị trí, vận tốc, gia tốc của PX4) dùng thẳng toạ độ LIO (MPC,
phương án B của D10). Người phản biện không bảo vệ thiết kế cũ và không ưu tiên ý mới. Nguyên tắc áp dụng: D44, D46
(SYSTEM_DESIGN §0 "Giới hạn thông tin"), D40–D48. Mọi con số có nguồn (run, commit, `file:dòng`); điều không kiểm được
ghi **UNVERIFIED**. Owner chưa xác nhận; đề xuất tiêu chí nằm ở O36 trong [DECISIONS.md](../architecture/DECISIONS.md).

## 0. Kết luận ngắn

1. **`T` chạy đúng ở mọi chỗ đã đo, nhưng mới chỉ đo ở hover.** 12 lần hover SITL: `T` VALID 100% cửa sổ ở 10 lần;
   sai số khung tại xe p95 2.7–15 mm; 0 cặp bị cổng nhảy; 10/10 reset PX4 sau VALID được hấp thụ (VALID→VALID).
   Một lần rơi STALE giữa cửa sổ, do lỗi ghép cặp (DV2), không phải do khái niệm. Chưa có chuyến bay nào có chuyển
   động có lệnh, mất GNSS, khởi động lại LIO hay reset heading lớn sau VALID.
2. **Không chuyển sang B lúc này.** B không bỏ được `T`: frame mission ở gps (D19) và đường BACKUP khi LIO mất (D11)
   vẫn cần nó. Vậy B = A′ + một vòng điều khiển bay mới. B cũng không giải điểm nghẽn thông tin nào của D46. Phần B
   loại bỏ được (sai số khung trên đường lệnh) đo được ở hover là ≤ 15 mm p95.
3. **Có thể mở B sau**, chỉ theo một trigger đo được ở S2 (O36): sai số bám tổng vượt O4 S3 **và** phần do vòng của PX4
   chiếm ưu thế sau một vòng chỉnh tham số `MPC_*`. Không bao giờ mở B chỉ để chữa P3: P3 đã có lời giải rẻ hơn là `T`.
4. **Đơn giản hoá `T` thì vẫn làm, dù chọn hướng nào:** làm xong D47/D48 (Task 4), sửa DV2 bằng cách chờ mẫu PX4 kế
   tiếp thay vì bỏ cặp, và không thêm M1/M2.
5. **Phép đo rẻ nhất để quyết:** chạy Task 6 của S1c (benchmark drift 1/3/5 m/s, đã có kế hoạch), báo cáo thêm residual
   của `T` theo tốc độ và sai số bám của PX4 trong frame của chính nó (đọc từ ULog). Không cần đổi gì trong sản phẩm.

## 1. Bối cảnh: `T` sinh ra để chữa gì

- P3 ("LIO né đúng nhưng PX4 drift và đâm vật cản") và P4 ("velocity-only bám kém; P/V/A bám theo vị trí đã lệch")
  được ghi trên baseline `main`.
- Trên `main`, frame transform **chưa bao giờ nằm trên đường lệnh**. `main:docs/architecture/SYSTEM_DESIGN.md` §4 ghi:
  "Yaw/bias witness và pure frame-transform contract hiện là DIAGNOSTIC_ONLY". Biên PX4 là velocity-only (W5-CP07 trong
  `main:docs/safety/runtime_safety_current.md`). EV lại mang nhãn NED và fuse EV yaw, nên frame `lio_odom` chỉ đúng khi
  xe khởi động hướng đông (F34).
- Vậy triệu chứng P3/P4 xuất hiện khi **không có `T`**, với một lỗi frame đã biết (F34, đã sửa bằng FRD ở `b64e0a9`).
  Không có bằng chứng nào nói "`T` không đủ". A′ (P/V/A qua `T`) chưa từng bay có chuyển động; B cũng chưa.

## 2. Hiệu quả của `T` trong các lần chạy SITL

**Cách đo** (script offline chỉ đọc `log/sitl/<run>/`, không chạy SITL):
- Tỉ lệ trạng thái: tính theo thời gian, trên `alignment.jsonl`.
- Residual: `residual_position_m` của các output VALID trong cửa sổ hover có cặp (`raw_*` khác null).
- "`T` cố định": `T₀` = `T` VALID đầu cửa sổ. Với mỗi mẫu EV (`vehicle_visual_odometry`, FRD), lấy
  `|p_px4(t) − T₀(p_lio(t))|`, so với cùng phép đo dùng `T` động lúc đó. Hiệu hai số là phần `T` động thật sự bù.
- Drift ngang: so dịch chuyển từ đầu cửa sổ với ground truth (ENU→NED).
- `NO_PX4_SAMPLE`: tổng số đếm thật của các đợt bị gộp (bản ghi `summary`), trên số odometry nhận được.

| Run | Profile | VALID trong cửa sổ | Độ trải `yaw_T` | Biên độ `t` x/y (mm) | Residual p95 / max (mm) | `T` cố định p95 (mm) | Drift PX4 / LIO (mm) | Reset sau VALID | `NO_PX4_SAMPLE` / odom |
|---|---|---|---|---|---|---|---|---|---|
| `20261013T151158Z-gps-hover` (harness mất cloud) | gps | 87.9% (FROZEN 12.1%, 15 lần) | 0.161° | 24 / 28 | 15.1 / 23.3 | 57.0 | 31.5 / 4.7 | 1 | 23 / 665 |
| `20261013T152418Z-gps-hover` | gps | 100% | 0.090° | 41 / 19 | 10.7 / 15.5 | 44.3 | 32.1 / 2.9 | 1 | 264 / 838 |
| `20261013T154709Z-gps-hover` | gps | 100% | 0.076° | 42 / 18 | 8.8 / 10.7 | 42.1 | 34.0 / 5.7 | 1 | 11 / 842 |
| `20261013T154923Z-gps-hover-yaw1.2` | gps | 100% | 0.040° | 26 / 17 | 7.8 / 9.4 | 35.6 | 34.4 / 3.3 | 1 | 24 / 829 |
| `20261013T155144Z-no_gps-hover` | no_gps | 100% | 0.032° | 4 / 4 | 5.1 / 15.8 | 9.4 | 25.5 / 30.8 | 0 | 143 / 832 |
| `20261013T155429Z-no_gps-hover-yaw1.2` | no_gps | 100% | 0.035° | 7 / 2 | 5.3 / 17.4 | 8.8 | 9.9 / 7.6 | 0 | 2 / 832 |
| `20261013T155805Z-gps-hover` (Hold) | gps | 100% | 0.053° | 41 / 19 | 9.5 / 11.6 | 64.3 | 34.0 / 5.7 | 1 | 14 / 844 |
| `20261013T163717Z-gps-hover` | gps | 100% | 0.045° | 36 / 27 | 9.7 / 11.4 | 50.1 | 35.5 / 2.8 | 1 | 10 / 842 |
| `20261013T232211Z-gps-hover` | gps | 100% | 0.138° | 18 / 44 | 9.3 / 20.1 | 51.5 | 36.9 / 3.3 | 1 | 262 / 844 |
| `20261014T061438Z-gps-hover-yaw1.2` (EV 15) | gps | 100% | 0.075° | 33 / 9 | 8.1 / 11.6 | 31.8 | 28.3 / 7.5 | 1 | 0 / 826 |
| `20261014T062206Z-no_gps-hover-yaw1.2` (EV 15) | no_gps | 100% | 0.054° | 6 / 2 | 2.7 / 6.5 | 7.1 | 6.7 / 6.6 | 0 | 202 / 830 |
| `20261014T062427Z-gps-hover-yaw1.2-gnssout10` | gps | 92.3% (INIT 7.7%, sau STALE) | 0.040° | 26 / 10 | 8.3 / 10.4 | 29.4 | 26.8 / 6.2 | 1 | 356 / 831 |

Đường lỗi duy nhất đã ép: `20261013T162711Z-gps-altitude_sweep`. LIO LOST ở 36.2 s, `T` VALID→FROZEN, rồi 10.4 s sau
INVALID (`FROZEN_TOO_LONG`). Đây đúng là đồng hồ mà D48 bỏ.

**Đọc bảng.**
- **gps:** PX4 trôi theo GNSS (sim Gauss-Markov) 27–37 mm trong 60 s, trong khi LIO chỉ trôi 3–8 mm. `T` động kéo sai số
  khung tại xe từ 29–64 mm (nếu giữ `T` cố định) xuống 8–15 mm p95. Phần dư là độ trễ của bộ lọc τ = 2 s. `T` đổi chậm:
  tịnh tiến biến thiên 9–44 mm trong 60 s (< 1 mm/s), yaw ≤ 0.16°. **Đây là việc thật của `T`:** mang độ lệch
  GNSS–LIO sang đường lệnh.
- **no_gps:** `yaw_T` = 0.000–0.006°, tịnh tiến biến thiên ≤ 7 mm. `T` cố định chỉ kém `T` động vài mm. PX4 **đi theo
  LIO**: ở `20261013T155144Z` LIO trôi 30.8 mm và PX4 trôi 25.5 mm theo nó (đúng hàng 2 của bảng D46). Trạng thái dừng
  của `T` là phép đồng nhất. `T` chỉ mang thông tin sau một reset heading hoặc vị trí của EKF2, hay sau khi LIO khởi
  động lại; chưa lần chạy nào có các sự kiện đó sau VALID.
- **Reset:** 10 reset PX4 sau VALID (9 hover gps và chuyến quét; heading 0.002–0.249°), cả 10 đều VALID→VALID. Reset lớn (88° ở yaw spawn 0, ±20°
  khi EV yaw bắt đầu) đều rơi vào INIT, trước VALID. Reset lớn sau VALID chỉ có test thành phần
  (`Px4Bridge.LargeHeadingResetAfterReinitKeepsTheVehiclePointContinuous`, Task 4). **UNVERIFIED** trong SITL.
- **DV2:** 0–43% odometry không ghép được cặp (`NO_PX4_SAMPLE`); một lần làm `T` STALE trong cửa sổ (32.90 s, 4.6 s
  không có `T`). Nguyên nhân chắc chắn nằm ở code: cặp chỉ được xét một lần, lúc odometry tới. Nếu chưa có mẫu PX4 sau
  stamp scan thì cặp bị bỏ ngay (`src/px4/uavnav_px4_bridge/src/alignment.cpp:163`). Còn vì sao mẫu PX4 tới sau thì
  vẫn mở ([báo cáo EV đầy đủ](2026-10-s1c-ev-full.md) §6).

### 2a. EV fuse đầy đủ: PX4 thật sự tin gì

Đọc ULog của các lần chạy (`estimator_aid_src_ev_pos`, cửa sổ hover):

| Đại lượng | gps | no_gps | Nguồn |
|---|---|---|---|
| σ vị trí EV do LIO gửi (p50) | 1.33 mm | 1.30 mm | [báo cáo EV đầy đủ](2026-10-s1c-ev-full.md) §3 |
| σ quan sát EV mà EKF2 dùng (p50) | 0.521 m | 0.100 m | ULog `20261014T061438Z`, `20261014T062206Z`, `20261013T152418Z`, `20261013T155429Z` |
| Test ratio EV p95 | 0.0000 | ≤ 0.0002 | như trên |
| EV được fuse | 100% | 100% | như trên |
| Innovation GNSS p95 | 0.054–0.067 m | — | ULog gps |
| Sai số bám của PX4 trong frame của nó, POSCTL (p95) | 8.7–10.2 mm | 4.8–7.3 mm | ULog `vehicle_local_position_setpoint` so với `vehicle_local_position` |

- Sàn của EKF2 (PX4 v1.17 `ekf2/EKF/aid_sources/external_vision/ev_pos_control.cpp:129-146`) chặn σ EV dưới 0.1 m
  (`EKF2_EVP_NOISE`) và dưới 0.5 m khi GNSS đang fuse. Bias estimator chạy khi có GNSS (dòng 111–116).
- **Kết quả ở no_gps:** EKF2 = LIO trong vài mm. EV đã làm hết việc của `T`; `T` thêm ≤ 5 mm.
- **Kết quả ở gps:** EKF2 là trung bình có trọng số giữa GNSS và EV (cùng cỡ σ 0.5 m), còn LIO giữ frame cục bộ. Hai
  frame lệch nhau đúng bằng độ lệch GNSS, và chỉ `T` (hoặc B) đưa được độ lệch này vào đường lệnh.

## 3. Chi phí và độ ổn định của `T`

### 3a. Lỗi và vòng sửa có liên quan tới `T` hay bridge từ S1a

| # | Lỗi | Phát hiện | Sửa | Loại |
|---|---|---|---|---|
| 1 | Residual, cổng và bước lọc đo tại gốc LIO: ở 300 m, 1.7 mrad đã thành 0.5 m | O12 (S1a) | `f2faa1e`, `29241ce` (D30) | Vốn có của hai frame; cài đặt S1a sai chỗ đo |
| 2 | Reset heading của PX4: cộng thẳng delta, không áp phép đổi hệ G | O11 | sửa câu chữ spec (code đã đúng) | Vốn có (hai bộ ước lượng) |
| 3 | Pose 1e300 làm residual không hữu hạn | review D30 vòng 1 | `29241ce` | Sơ suất (kiểm input) |
| 4 | Cạnh không có trong bảng vẫn được áp | review S1a vòng 1 | `2267786` | Sơ suất |
| 5 | Bộ giới hạn tốc độ riêng không bao giờ kích hoạt | O12 | gỡ ở `f2faa1e` | Làm thừa |
| 6 | Phương sai hướng EV tính theo trục world | S1a | `6f555e8` | Đường EV, có ở mọi phương án |
| 7 | Bộ giới hạn event của bridge nuốt các đợt lỗi sau | review S1b T9 I1 | `ee9fb2f` | Hạ tầng log |
| 8 | FROZEN→VALID nhận mọi epoch; `go(VALID)` đặt lại mốc stale | phản biện S1c C2 | D47 (Task 4, chưa code) | Vốn có (hai frame × LIO restart); thiết kế sơ suất |
| 9 | Hai đồng hồ 10 s cho cùng một lỗi | O35 | D48 (Task 4, chưa code) | Thiết kế sơ suất |
| 10 | Offer seed và cửa sổ đồng bộ lại (O29, O33) | AR4 | cắt trước khi code | Làm thừa |
| 11 | DV2: cặp bị bỏ khi mẫu PX4 tới sau odometry | review cuối S1b #7, S1c T1 I5 | mở (Task 4 Step 5) | Sơ suất cài đặt |
| 12 | Luật F34 đánh FAIL một reset được hấp thụ đúng | review S1c T1 I2 | `52fbf46` | Sơ suất ở bộ chấm |
| 13 | Mã health lạ không đóng băng `T` | review cuối S1b | carry-over S2 | Sơ suất |
| 14 | Hai reset trong một cửa sổ mẫu bị mất chỉ áp delta cuối | review S1b T10 | chấp nhận; cổng nhảy chặn | Vốn có (bộ đếm PX4, F13) |
| 15 | Reset heading khoảng 20° khi EV yaw bắt đầu | D35, O26 | đo được, nhưng trước VALID | Vốn có (PX4 tự chọn nguồn heading) |

- **Tổng:** 5 lỗi vốn có của việc có hai frame (1, 2, 8, 14, 15), 7 sơ suất (3, 4, 7, 9, 11, 12, 13), 2 chỗ làm thừa
  (5, 10), 1 lỗi của đường EV (6).
- **Ở B:** 2, 14, 15 vẫn còn, vì EKF2 vẫn chạy và `T` vẫn cần cho frame mission (D19) và cho BACKUP khi LIO mất (D11).
  1 và 8 cũng còn ở `T` dùng cho mission. B không xoá được loại lỗi vốn có nào. Nó chỉ đưa hậu quả của chúng ra khỏi
  vòng 50 Hz.
- **So sánh:** `uavnav_px4_bridge` có 5 commit fix trên 13 commit, trong đó 3 commit riêng của alignment. `uavnav_lio_core`
  có 12 commit fix trên 27 (`git log`). `T` không phải điểm nóng bất ổn; lifecycle của LIO mới là điểm nóng (AR1).
- **Chi phí thiết kế:** 26 dòng của DECISIONS nhắc `T`. Có 11 mục O/D về `T` (O11, O12, O22, O29, O31, O33, O34, O35,
  D35, D47, D48). Ba trong số đó (O29, O31, O33) bị bỏ mà chưa code. Đây là giá của việc dựng tính năng giả vờ có thông
  tin (D44.3), và AR4 đã cắt.

### 3b. Những gì `T` sở hữu (code tại `7fcca0c`)

| Hạng mục | Hiện tại | Sau D47/D48 | Nguồn |
|---|---|---|---|
| Trạng thái | 4 (INIT, VALID, FROZEN, INVALID) | 4 | `src/px4/uavnav_px4_bridge/include/uavnav/px4bridge/alignment.hpp:89` |
| Cạnh | 7 | 8 (−`FROZEN_TOO_LONG`, +2 `NEW_LIO_EPOCH`) | `src/px4/uavnav_px4_bridge/include/uavnav/px4bridge/alignment.hpp:228-236` |
| Reason | 12 | 12 (−1, +1) | `src/px4/uavnav_px4_bridge/include/uavnav/px4bridge/alignment.hpp:98-115` |
| Đồng hồ | `valid_stale` 1 s, `frozen_max` 10 s | `valid_stale` | `src/px4/uavnav_px4_bridge/src/alignment.cpp:114-125` |
| Tham số tầng (b) | 6 | 5 | `src/px4/uavnav_px4_bridge/include/uavnav/px4bridge/config.hpp:20-25` |
| Hằng tầng (a) | 9 (cửa sổ ghép 50 ms, buffer 1 s/256, dt lọc, 1e6 m, 200 cặp, tick, stamp tương lai) | 9 | `src/px4/uavnav_px4_bridge/include/uavnav/px4bridge/limits.hpp:19-71` |
| Event | 4 (`AlignmentTransition`, `AlignmentPairRejected`, `AlignmentAccumulationRestarted`, `Px4ResetApplied`) | 4 | AS_BUILT `uavnav_px4_bridge` |
| Cờ trạng thái bool | 0 (`cfg_valid_` là hằng) | 0 | `src/px4/uavnav_px4_bridge/include/uavnav/px4bridge/alignment.hpp:321-323` |
| Dòng code / test | 730 / 1633 | ≈ | `wc -l` |

- Theo D44.5, `T` gọn: một enum, một bảng cạnh, không cờ.
- Theo D46, `T` không giả vờ có thông tin. Liên tục frame qua restart thì để INIT (hàng 4 của bảng); covariance EV thì
  để cổng nhị phân (hàng 6).
- Phần còn nợ (8, 9, 11) đã có lời giải nhỏ.

## 4. Phương án B, trình bày ở dạng mạnh nhất

**B:** bộ điều khiển riêng (MPC hoặc geometric) chạy trên trạng thái LIO, gửi attitude + thrust (hoặc body rate) cho
PX4, thay vòng P/V/A của PX4.

| Khía cạnh | B cần gì | Bằng chứng |
|---|---|---|
| Giao diện PX4 | `OffboardControlMode` có position/velocity/acceleration/attitude/body_rate/thrust_and_torque/direct_actuator (PX4 `msg/OffboardControlMode.msg`). Trong px4_ros2 release/1.17, attitude và rate nằm ở namespace `experimental`, 100 Hz | `src/external/px4_ros2_interface_lib/px4_ros2_cpp/src/control/setpoint_types/experimental/attitude.cpp:73-82` |
| Yêu cầu vị trí | Setpoint attitude không đòi `local_position`, nên PX4 **không** failsafe mode khi mất vị trí; B tự lo | `src/external/px4_ros2_interface_lib/px4_ros2_cpp/src/components/mode.cpp:264-272` |
| Nguồn trạng thái | `/lio/state` 100 Hz, p99 12 ms (P7). Thêm hai chặng DDS (LIO → điều khiển → PX4) trên Linux không realtime | TRACEABILITY P7, `run:20261013T152418Z-gps-hover` |
| Mô hình | thrust chuẩn hoá [−1, 1] cần hover thrust (PX4 ước lượng nội bộ, không xuất qua DDS mặc định), drag, giới hạn tilt; nhận dạng trong sim không chuyển sang phần cứng | PX4 `uxrce_dds_client/dds_topics.yaml` (không có `hover_thrust_estimate`) |
| EKF2 | vẫn chạy (attitude cho vòng trong, failsafe, bàn giao D4); vẫn phải gửi EV để PX4 có vị trí khi bàn giao | D4, D11, SYSTEM_DESIGN §3.4 |
| LIO LOST | B mất phản hồi vị trí. Hai lựa chọn: tích phân IMU (trôi theo t², sim không có bias nên lạc quan), hoặc quay về PX4 qua `T` đóng băng, tức là A′ | D11, §4.2 |
| Bỏ được | `T` trên đường lệnh 50 Hz; ảnh hưởng của GNSS lên vị trí khi LIO khoẻ; trạng thái `ALIGNMENT_INVALID` của `px4_mode` | |
| Không bỏ được | `T` cho frame mission gps (D19); `T` đóng băng cho BACKUP (D11); frame mới sau restart (D46 hàng 4: planner/map reset như nhau); drift no_gps (D46 hàng 2); bàn giao cần vị trí PX4 (no_gps mất sau `EKF2_NOAID_TOUT` 5 s) | D46 |
| Thêm lỗi mới | một bộ ước lượng duy nhất cho điều khiển; LIO hỏng = điều khiển hỏng nếu không có A′ dự phòng; độ trễ đuôi và jitter của DDS trong vòng bay; phần mềm tự viết trở thành flight-critical; mất bộ điều khiển PX4 đã được kiểm nhiều | |
| Công sức (ước lượng) | ≥ một lát cỡ S1b (≈ 14 task, ≈ 20 lần SITL): thư viện điều khiển và test, nhận dạng mô hình, `px4_mode` attitude, thiết kế lại bàn giao, certifier khớp mô hình động lực mới, AR mức L3 (đổi D10, §4.2) | ước lượng, UNVERIFIED |

**Phần mạnh nhất của B:**
1. Khi LIO khoẻ, nhiễu, trôi và nhảy của GNSS **không vào** vòng vị trí. A′ phải bám chúng qua `T`, với độ trễ τ và giới
  hạn 0.25 m/s tại xe. Ở gps, một bước nhảy GNSS > 0.5 m mà EKF2 không báo reset sẽ làm `T` từ chối cặp → STALE →
  `ALIGNMENT_INVALID` → bàn giao.
2. Bay khéo léo hay 10 m/s có thể vượt khả năng của vòng tầng P–PID + feedforward của PX4
  (`mc_pos_control/PositionControl/PositionControl.cpp:124-152`).

Cả hai điểm là **giả thuyết phần cứng**: chưa có số đo (sim GNSS 27–37 mm trong 60 s; chưa bay > 0 m/s có lệnh).

**Các phương án giữa:**
- **M1, lệnh tương đối:** `p_sp = p_px4(t) + R(ψ)(p_cmd − p_lio(t))`. Đây là `T` với τ → 0 và không có cổng. Vẫn cần ghép
  thời gian hai luồng (cùng loại lỗi DV2), vẫn cần `T` đóng băng khi LIO mất, và đưa nhiễu tức thời vào setpoint. Nó là
  một **giá trị của τ**, không phải kiến trúc mới. Không xây; chọn τ bằng số đo S2.
- **M2, bộ bám vòng ngoài trong frame LIO sinh P/V/A:** A′ đã làm phần feedforward. Đóng thêm một vòng phản hồi trên sai
  số LIO thì hai vòng vị trí đánh nhau khi `T` trễ (D44.7). Không làm.
- **M3, PX4 chỉ làm vòng attitude, vòng P/V của mình:** cùng chi phí và rủi ro như B, ít lợi hơn MPC. Chỉ đáng xét như
  bước đầu của B nếu trigger O36 bật.

**Kiến trúc đã giữ đường lui:** `/nav/command` ở frame LIO (D17). Đổi bộ thực thi sau này chỉ chạm `px4_mode`, không
chạm planner hay Supervisor.

## 5. Điều gì quyết định: tiêu chí đo được (đề xuất O36)

Tách sai số bám tại S2 thành các thành phần đo được, ở mỗi tốc độ 1, 3, 5 m/s (đoạn thẳng và đoạn rẽ, hai profile,
≥ 3 lần mỗi ô):

| Mã | Đại lượng | Cách đo | Ngưỡng đề xuất |
|---|---|---|---|
| K1 | Sai số khung tại xe do `T` (cả trễ τ) | `residual_position_m` khi VALID (`/alignment`) | p95 ≤ 0.05 m (một nửa O4 S4) |
| K2 | PX4 bám setpoint trong frame của chính nó | ULog `vehicle_local_position_setpoint` so với `vehicle_local_position` | báo cáo; so với K1 |
| K3 | Lệch planner–thật (O4 S3) | ground truth so với lệnh `/nav/command` đổi sang world | p95 ≤ 0.227 m |
| K4 | `T` rời VALID khi LIO vẫn TRACKING và không có reset PX4 | event `AlignmentTransition` | 0 mỗi giờ bay |
| K5 | Số bàn giao `ALIGNMENT_INVALID` | event của `px4_mode` | 0 trên scene tĩnh |

**Quy tắc:**
- K3 đạt: giữ A′, không mở B (D10).
- K3 trượt và K1 chiếm ưu thế (K1 p95 ≥ ½ K3 p95): sửa `T` (τ, ghép cặp). Không mở B.
- K3 trượt và K2 chiếm ưu thế: một vòng chỉnh `MPC_*` có ghi lại, và kiểm feedforward gia tốc tới PX4. Nếu vẫn trượt
  thì mở B (hoặc M3) bằng một AR mức L3.
- K4 > 0: đó là lỗi cài đặt (loại DV2), sửa ở bridge.

**S2 phải ghi:** setpoint `px4_mode` đã gửi (JSONL, kèm `T` và epoch đã dùng); `/nav/command`; phần còn lại đã có
(`/alignment`, `vehicle_local_position`, ULog, ground truth).

**Trả lời được bây giờ** (hover):
- K1 ≤ 15 mm p95 (max 23 mm);
- K2 ≤ 10 mm p95 (POSCTL);
- K4 = 1 lần trong 12 cửa sổ 60 s, do DV2.

**Chưa trả lời được:** K1, K2, K3 khi có chuyển động; mất GNSS; khởi động lại LIO (Task 5); reset lớn sau VALID; độ
trôi GNSS thật.

**Phép đo rẻ nhất:** Task 6 của S1c (sáu lần POSCTL, 1/3/5 m/s, đã có kế hoạch) cho K1, K4 và một xấp xỉ của K2 khi có
chuyển động. Chỉ cần thêm hai mục vào báo cáo; không đổi sản phẩm. Xấp xỉ, vì POSCTL bám vận tốc từ stick, không phải
P/V/A feedforward như A′.

## 6. Ảnh hưởng của giới hạn mô phỏng (D46 hàng 7)

| Thiếu trong sim | Ảnh hưởng tới kết luận |
|---|---|
| Bias IMU | LIO và EKF2 đều lạc quan. B chịu nặng hơn khi LIO mất (tích phân IMU trong vòng điều khiển); A′ có `T` đóng băng cộng EKF2 |
| Thời gian từng điểm (méo tới 0.5 m ở 5 m/s) | Sai số LIO tăng như nhau ở A′ và B. B đưa nó thẳng vào vòng điều khiển; ở A′, EKF2 làm mượt EV với sàn σ 0.1/0.5 m |
| Nhiễu góc LiDAR (0.15°) | Tăng nhiễu `yaw_T`; residual tại xe giữ ảnh hưởng nhỏ (D30). K1 trên phần cứng phải đo lại |
| GNSS sim (trôi 27–37 mm/60 s) | Phần động của `T` ở gps bị **đánh giá thấp**; độ trôi và multipath thật quyết định K1, K4 ở gps. Đây là rủi ro chính của A′ |
| Vòng PX4 trên mô hình x500 lý tưởng | K2 trong sim lạc quan cho cả PX4 và cho mô hình của B |

## 7. Phán quyết

**(a) `T` đủ hiệu quả và ổn định để giữ, ở mức đã đo.**
- Ở gps nó bù đúng thứ cần bù: 29–64 mm → 8–15 mm p95. Ở no_gps nó là đồng nhất, đúng bản chất (EV đã làm EKF2 = LIO).
- Lỗi cài đặt duy nhất còn mở (DV2) là lỗi cục bộ.
- Chi phí bảo trì thấp hơn LIO, và phần thừa đã cắt (AR4).
- Không có bằng chứng `T` hỏng; cũng chưa có bằng chứng nó đủ khi có chuyển động.

**(b) Không chuyển B bây giờ.**
- B không bỏ được `T`, không giải điểm nghẽn nào của D46, và thêm một vòng bay flight-critical qua DDS.
- Mở B **sau** chỉ khi trigger của O36 bật ở S2, nghĩa là bộ điều khiển PX4, không phải `T`, là nút thắt.
- "Không bao giờ" không nói được, vì profile khéo léo và 10 m/s (O7) có thể cần B. Đó là câu hỏi hiệu năng điều khiển,
  không phải câu hỏi frame.

**(c) Đơn giản hoá ngay, dù chọn hướng nào:**
1. D47/D48 (Task 4): `T` theo epoch, bỏ `frozen_max`.
2. Sửa DV2 không thêm trạng thái: odometry chưa có mẫu PX4 bao quanh thì **giữ một cặp chờ**, xét lại ở mẫu PX4 kế tiếp
   trong `kPairingWindow`, thay vì bỏ ngay (`src/px4/uavnav_px4_bridge/src/alignment.cpp:153-170`).
3. `px4_mode` biến đổi lệnh bằng `T` cùng epoch với lệnh (phản biện S1c C5).
4. Không xây M1, M2, không thêm cơ chế "bù trễ τ" trước khi có K1 khi chuyển động.

   Giữ: 4 trạng thái, cổng tại xe, INIT 20 cặp, phép đổi hệ G cho reset.

**(d) Ba rủi ro lớn nhất của phán quyết:**
1. **GNSS thật ở gps:** trôi nhanh hơn 0.25 m/s tại xe, hoặc nhảy > 0.5 m không kèm reset, sẽ làm `T` STALE →
   `ALIGNMENT_INVALID` giữa mission. SITL không tạo được tình huống này (lệnh tiêm GNSS bị PX4 bỏ qua). Chỉ phần cứng
   trả lời được.
2. **Hiệu năng vòng PX4 khi bay nhanh:** nếu K2 là nút thắt, quyết định B đến muộn (sau S2). Đường lui `/nav/command` ở
   frame LIO giữ chi phí đổi ở mức cục bộ.
3. **DV2 có thể mang tính cấu trúc** (thứ tự DDS của hai luồng). Khi đó ghép theo thời gian cần thiết kế lại phần đệm.
   Rủi ro trung bình: mọi phương án có `T` đều mang nó, kể cả B.

## 8. UNVERIFIED

- Mọi hành vi khi có chuyển động có lệnh (A′ và B đều chưa bay).
- Hành vi của `T` khi mất GNSS, khi LIO khởi động lại, và khi có reset heading lớn sau VALID.
- Ước lượng công sức của B.
- Việc PX4 có xuất `hover_thrust_estimate` qua DDS nếu đổi `dds_topics.yaml`: chưa thử.
- Độ trễ đuôi của đường DDS lệnh attitude: chưa đo.
- Nguyên nhân gốc của DV2.
