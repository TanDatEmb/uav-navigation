# Phản biện nền tảng của `T_px4←lio`: toán, tính nhất quán, ổn định vòng kín (2026-10-19)

Câu hỏi của owner: giữ và bảo trì `T` (4-DoF, ước lượng online từ cặp pose) có chấp nhận được không, hay là một điểm
thiết kế làm hỏng logic, phá tính nhất quán của robot. Phản biện trước ([2026-10-t-sync-vs-lio-frame-control.md](2026-10-t-sync-vs-lio-frame-control.md),
O36) so A′ với B chủ yếu theo số đo; báo cáo này không lặp lại phần đó mà tấn công **nền tảng** của `T`. Đầu vào thêm:
bản phản biện v2 owner dán vào (coi là giả thuyết cần kiểm, mục 8). Không chạy SITL, không đổi `src/`, `tools/`.
Mọi số có nguồn: script trong scratchpad (mục 13), `file:dòng`, hoặc run id. Điều không kiểm được ghi **UNVERIFIED**.
Kết quả mô hình (mục 5, 7) là **mô hình**, chưa kiểm trên PX4 thật hay trong SITL có chuyển động.

## 0. Kết luận ngắn

1. **`T` không hỏng logic.** Nó là một hiệu chỉnh kiểu `map→odom` của REP-105: LIO là "odom" (liên tục, sau D47 không
   tiêu thụ gì của PX4), EKF2 là "map" (fuse GNSS và EV của LIO). Hướng phụ thuộc này là hướng chuẩn.
2. **Nhưng `T` không phải một gauge độc lập.** Ở no_gps, `T` ≈ phép đồng nhất cộng sai số bám EV của EKF2. Ở gps, `T` ≈
   lọc thấp của (sai số GNSS − sai số LIO) nhìn qua EKF2. Residual nhỏ chỉ nói hai bộ ước lượng **đồng ý tại xe**, không
   nói xe đúng so với thế giới.
3. **Phần đúng về toán:** đo tại xe (D30), 4-DoF (khi `z_T` được phép trôi), phép đổi hệ G cho reset PX4, `T` theo epoch
   (D47). **Phần là vá:** giá trị `τ_T` = 2 s, cổng nhảy dẫn tới STALE → INVALID → bàn giao, và quy tắc "reset PX4 khi
   FROZEN → INVALID".
4. **Một lỗi logic thật, không phải sơ suất cài đặt.** Ở no_gps, khi LIO mất, EV yaw dừng và EKF2 reset heading về
   compass. Đó là reset trong lúc FROZEN, nên `T` → INVALID và BACKUP của D11 không chạy được. Đây là kết luận đọc code
   PX4, chưa đo (mục 3, O38).
5. **C (`T` cố định mỗi epoch) bị bác** ở profile gps (ví dụ phản chứng ở mục 6). Phương án S, L và R cũng không thay
   được A′ lúc này. R thực chất là `T` với `τ` nhỏ, nên nó là một hướng chỉnh của A′, không phải kiến trúc mới.
6. **Vòng kín qua chuyển động ổn định** trong mô hình: `sup|L|` ≤ 0.29 ở mọi `τ` (kể cả 0), trễ tới 200 ms. Rủi ro thật
   là lệch thời gian ghép cặp (`T` hấp thụ `v·Δ`, hover không đo được) và quy tắc bỏ cuộc gắn vào bộ lọc.

## 1. `T` là gì về mặt toán

**Đối tượng.** Gọi x(t) là pose thật (vị trí p, yaw ψ) trong frame thế giới W.
- Frame L của LIO (một epoch) và frame P (local NED của EKF2) liên hệ với W bằng hai phép cứng cố định A_L, A_P (gauge).
  "Gauge thật" là `T* = A_P⁻¹ ∘ A_L`.
- Hai bộ ước lượng cho p̂_L(t) = A_L⁻¹ x(t) + e_L(t) và p̂_P(t) = A_P⁻¹ x(t) + e_P(t), với e_L, e_P là sai số theo
  thời gian (trôi, nhiễu, trễ).
- Cặp cùng stamp cho `T_inst(t)`, phép cứng duy nhất đưa p̂_L(t) tới p̂_P(t). Đo tại xe (D30):
  `T_inst ⊖ T*` = (e_P − R*·e_L, e_ψP − e_ψL) + lệch thời gian ghép · v.

**Nhóm.** SE(3) thu về 4-DoF (R³ ⋊ SO(2)), vì cả hai frame đều căn theo trọng trường: roll và pitch quan sát được từ
gia tốc kế ở cả hai, còn yaw của LIO là gauge không quan sát được.
- Chỗ giả thiết hỏng: frame LIO nghiêng ε ≈ b_acc/g so với phương thẳng đứng thật. 4-DoF bỏ ε, nên sai số z ở khoảng
  cách ngang d từ điểm neo là ε·d (`geometry_checks.py` mục 4): 0.20 m ở 100 m với b = 0.02 m/s²; 0.51 m (100 m) và
  1.53 m (300 m) với b = 0.05 m/s².
- `T` động bám thành phần này như độ trôi chậm của `z_T` tại xe; `T` cố định thì không. SITL không có bias IMU (ε = 0).

**Gauge hay hiệu sai số?** `T_inst` chỉ là gauge khi e_P − R*e_L là hằng số. Thực tế nó thay đổi theo thời gian và theo
quãng đường (trôi LIO tích luỹ theo đường đi, GNSS lang thang). Vì vậy `T` là **gauge + phần dư phi cứng, biến thiên
theo thời gian**, chỉ tuyến tính hoá đúng tại xe. Đưa một điểm xa xe qua `T` hiện tại là ngoại suy. Sai số yaw của
`T` nhân cánh tay đòn, 2·d·|sin(Δψ/2)| (bảng v2, đã tính lại, khớp):

| d \ Δψ | 0.5° | 1° | 2° | 5° |
|---|---|---|---|---|
| 10 m | 0.09 | 0.17 | 0.35 | 0.87 |
| 50 m | 0.44 | 0.87 | 1.75 | 4.36 |
| 100 m | 0.87 | 1.75 | 3.49 | 8.72 |
| 300 m | 2.62 | 5.24 | 10.47 | 26.17 |

**`T` hội tụ về đâu khi EKF2 fuse EV.** Mô hình KF tuyến tính một trục, `ekf_transfer.py`. Mô hình xấp xỉ: bias EV
được gộp vào cùng KF, còn PX4 chạy nó như một bộ ước lượng riêng. Các tham số:
- EV: σ 0.5 m khi có GNSS, 0.1 m khi không (sàn `EKF2_GPS_P_NOISE`/`EKF2_EVP_NOISE`); EV vận tốc 0.1 m/s;
- GNSS: 0.5 m, 0.3 m/s;
- bias EV nsd 0.13 (Autopilot `modules/ekf2/EKF/common.h` dòng 444).

Với x̂_P = H_G·e_G + H_E·e_L, ta có `T_inst = H_G·(e_G − e_L) + (H_G + H_E − 1)·e_L`.

| Profile | \|H_G\| ở 0.001 / 0.1 / 1 rad/s | \|H_G+H_E−1\| ở 1 / 3 rad/s | Nghĩa của `T` |
|---|---|---|---|
| gps (bias EV bật) | 1.000 / 0.881 / 0.212; = 0.5 ở 0.34 rad/s (≈ 3 s) | 0.056 / 0.154 | DC: đúng độ lệch GNSS − LIO; tần số cao: bị EV kéo về 0 |
| gps, EV không bias (khi `ev_yaw` bật) | 0.500 / 0.470 / 0.167 | 0.059 / 0.159 | chỉ còn **một nửa** độ lệch: PX4 frame là trung bình hai nguồn |
| no_gps | 0 / 0 / 0 | 0.063 / 0.208 | `T` ≈ đồng nhất + lỗi bám EV của EKF2; không phải gauge |

Nhánh không bias: EV FRD có `ev_yaw` thì tắt bộ ước lượng bias (Autopilot
`modules/ekf2/EKF/aid_sources/external_vision/ev_pos_control.cpp` dòng 88–95, 112–117).
- Ở gps, `ev_yaw` không bật khi GNSS đang fuse (S1c Task 1), nên trường hợp 0.5 chỉ xảy ra khi mất GNSS. UNVERIFIED.

**Đếm hai lần.** `T` lấy hiệu giữa một ước lượng đã chứa LIO (EKF2) và chính LIO. Như vậy không sai, vì `T` không bị
fuse lại như một phép đo độc lập ở đâu cả. Hệ quả chỉ là băng thông: `T` thấy độ lệch GNSS − LIO qua H_G(s).
- Thứ **sẽ** làm hỏng ước lượng là một vòng ngược "PX4 → LIO" (seed `T⁻¹ × pose PX4`). D47 đã cắt vòng đó.
- Vậy đếm hai lần chỉ gây lệch băng thông, không làm ước lượng vô hiệu.

**Thành phần tỉ lệ vận tốc (đo được).** `lag_regression.py` hồi quy sai số theo vz thật (ground truth), trên các run
có cất cánh hoặc quét độ cao (|vz| p95 0.65–0.93 m/s; ngang ≤ 0.012 m/s):

| Run | D_LIO so GT (ms) | D_PX4 so GT (ms) | độ dốc e_z của cặp theo vz (ms) | \|e_z\| p95 |
|---|---|---|---|---|
| `20261013T162711Z-gps-altitude_sweep` | 15.6 ± 3.0 | −18.2 ± 0.7 | 8.2 ± 1.4 (r 0.35) | 21.3 mm |
| `20261014T061438Z-gps-hover-yaw1.2` | 3.5 ± 3.2 | −9.1 ± 1.3 | 11.8 ± 1.7 (r 0.24) | 17.0 mm |
| `20261013T152418Z-gps-hover` | 4.1 ± 1.8 | −25.5 ± 0.6 | 14.9 ± 1.1 (r 0.49) | 14.3 mm |
| `20261014T062206Z-no_gps-hover-yaw1.2` | 4.6 ± 1.8 | 4.7 ± 1.3 | −2.6 ± 1.0 (r −0.11) | 10.6 mm |

- Ở gps, cặp mang một độ lệch ≈ 8–15 ms × v; ở no_gps ≈ 0 vì EKF2 bám EV. Ngoại suy sang trục ngang ở 5 m/s:
  40–75 mm (UNVERIFIED: kênh dọc có baro/GNSS riêng).
- Một cặp không phân biệt (i) trễ thật của ước lượng PX4, mà bộ điều khiển PX4 hành động theo (bù là đúng), với (ii)
  lệch đồng hồ ghép cặp (bù là sai). Chỉ hồi quy theo v trên dữ liệu có chuyển động mới tách được; hover thì không.

## 2. Ý nghĩa thật của các giá trị

| Giá trị | Nó làm gì | Nó chứng minh | Nó **không** chứng minh |
|---|---|---|---|
| VALID | `px4_mode` được dùng `T` | có cặp được nhận trong `valid_stale` (1 s), mọi cặp trong cổng 0.5 m/5° | xe đúng so với thế giới; điểm xa xe hay map đúng; ở no_gps nó gần như luôn đúng theo cấu trúc (EKF2 ≈ LIO) |
| `residual_position_m` | độ bất đồng tại xe, trước bước lọc | hai bộ ước lượng, cùng chia sẻ LIO, đồng ý tới cỡ đó | độ chính xác tuyệt đối; nó gồm cả nhiễu ghép thời gian `v·Δ` |
| K1 = `T_inst − T_used` | sai số tầng alignment thêm vào lệnh, trong frame LIO | phần lệnh lệch so với ý định tương đối của planner (mục 3) | lỗi của chính LIO, lỗi bám của PX4 (K2) |
| INIT | gom 20 cặp nhất quán (≈ 2 s) | không có gì để dùng; `px4_mode` không biến đổi lệnh | — |
| FROZEN | giữ `T` cuối khi LIO không TRACKING | quan hệ L→P **lúc đóng băng** | quan hệ đó còn đúng: PX4 vẫn trôi (no_gps: chỉ IMU) hoặc bị GNSS kéo mà không reset |
| INVALID | `px4_mode` bàn giao `ALIGNMENT_INVALID` | không có `T` dùng được theo luật | rằng một trong hai bộ ước lượng hỏng (DV2 làm INVALID khi cả hai luồng đều tốt) |

## 3. Bất biến và lỗi logic

| # | Bất biến | Thiết kế hiện tại |
|---|---|---|
| I1 | Lệnh `p` trong frame LIO được thực thi sao cho LIO thấy xe ở `p` | **xấp xỉ**: sai số = K1 + K2; K1 ≈ r·τ (r = tốc độ bất đồng) + `Δ·Δv` khi tăng/giảm tốc |
| I2 | `T` dùng cho một lệnh có cùng danh tính frame với lệnh: epoch LIO (miền) và thế hệ reset PX4 (đích) | **vi phạm ở interface**: `NavCommand` có `lio_epoch` (`src/core/uavnav_interfaces/msg/NavCommand.msg:10`), `Alignment.msg` không có epoch hay bộ đếm reset (`src/core/uavnav_interfaces/msg/Alignment.msg:1-24`); D47 (C5) đòi `px4_mode` dùng `T` cùng epoch là không làm được với message này |
| I3 | Setpoint liên tục khi `T` đổi | **bảo đảm** ở bước lọc (≤ α·0.5 m = 25 mm mỗi cặp); ở reset PX4 xem L3 |
| I4 | Không ghép `T` qua hai epoch | **bảo đảm** sau D47 (INIT); D46 hàng 4 thừa nhận mất liên tục |
| I5 | Waypoint mission (gps, D19) và goal của planner chỉ một điểm vật lý | **xấp xỉ**: đổi lại ở mỗi request (`docs/architecture/SYSTEM_DESIGN.md:170`), sai số Δψ_T·d_còn lại hội tụ khi tới gần; **chưa định nghĩa** kiểm "đã tới waypoint" dùng phép đổi nào (§2.6) |
| I6 | FROZEN chỉ dùng trong khi sai số tích luỹ còn trong khe hở | **không chứng minh** (mục 7) |
| I7 | Mỗi quyết định bỏ cuộc có một chủ | **vi phạm mềm**: cổng → STALE (1 s) → INVALID → `px4_mode` bàn giao, đi vòng qua Supervisor (cùng họ với O35) |

**L1 (lỗi logic thật). no_gps: mất LIO thì `T` thành INVALID ngay, BACKUP của D11 không chạy được.**
Chuỗi sự kiện, theo đọc code PX4 v1.17, chưa đo sau VALID:
1. LIO không TRACKING, nên bridge dừng EV (§4.1).
2. Sau 2·`EV_MAX_INTERVAL` = 0.4 s, EKF2 dừng EV yaw (`ev_control.cpp` dòng 102–107).
3. `yaw_align` đã bị xoá khi EV yaw bắt đầu (S1c Task 1). Nhánh `!ev_yaw && !yaw_align` cho compass bắt đầu, kèm reset
   heading (`mag_control.cpp` dòng 179–183, 286–297).
4. Độ lớn của reset bằng độ lệch giữa heading khởi động của LIO và từ bắc: −20.47° ở run
   `20261014T062206Z-no_gps-hover-yaw1.2`. Về nguyên tắc nó có thể tới ±180°.
5. Reset khi FROZEN đưa `T` → INVALID (`src/px4/uavnav_px4_bridge/src/alignment.cpp:205-210` tại `86eaa28`), rồi guard
   `ALIGNMENT_INVALIDATED` cho bàn giao (`docs/architecture/SYSTEM_DESIGN.md:135`).

Phép đổi G của reset heading là **chính xác**: vị trí liên tục, delta có trong mẫu. Quy tắc INVALID bỏ đi một thông tin
có sẵn (trái D44.3). Lý do duy nhất của quy tắc là một delta có thể mất khi rơi message (F13). Ba quyết định đã chốt
(D11, D42, §4.1) cùng nhau loại BACKUP ở no_gps. Ghi thành O38.

**L2 (vá trở thành chính sách). Cổng nhảy + `τ_T` quyết định bàn giao.**
- Khi EKF2 dịch so với LIO nhanh hơn jump/τ = 0.25 m/s mà không reset, `T` từ chối cặp. Ví dụ: hội tụ về GNSS sau một
  lần mất, hay GNSS nhảy được fuse dần.
- Khi đó xe (bám EKF2) trôi so với map LIO, rồi INVALID và bàn giao. `gate_cascade.py`:

| Bất đồng | τ 2 s (beta) | τ 0.3 s | τ 0.3 s, không cổng |
|---|---|---|---|
| 0.3 m/s, 1 m | K1 0.47 m, VALID | 0.06 m, VALID | 0.06 m |
| 0.5 m/s, 2 m | K1 1.01 m, **INVALID 2.4 s sau khi bắt đầu** | 0.10 m, VALID | 0.10 m |
| 1.0 m/s, 1 m | K1 0.93 m, **INVALID** | 0.20 m, VALID | 0.20 m |
| 5 m/s, 1 m (bước) | K1 1.0 m, INVALID | 1.0 m, INVALID | 0.56 m |

- Ở cả hai nguồn của bất đồng, bám theo bất đồng giữ được ý định tương đối của planner:
  - PX4 dịch: bám thì xe không dịch theo PX4;
  - LIO trượt khi vẫn TRACKING: bám thì xe nhất quán với map cục bộ mà LIO vừa dựng.
- Cổng chỉ có lý khi dùng để loại **một cặp hỏng** (lỗi ghép, rác). Bất đồng kéo dài là một tín hiệu health, và quyết
  định trên nó thuộc về Supervisor.

**L3 (đua nhỏ, cross-layer). Reset PX4 trong lúc `px4_mode` còn `T` cũ.** PX4 tự cộng delta reset vào setpoint, chỉ
cho setpoint có stamp trước mẫu reset (Autopilot `modules/mc_pos_control/MulticopterPositionControl.cpp` dòng 687–712);
px4_ros2 để PX4 tự đóng stamp (`experimental/trajectory.cpp` dòng 31). Setpoint tính bằng `T` cũ mà tới sau reset lệch
đúng một delta trong một vòng PX4 → bridge → `/alignment` → `px4_mode` (ước 10–40 ms, UNVERIFIED). Không bị bù hai
lần; bước nhảy ngắn này biến mất nếu `/alignment` mang bộ đếm reset (I2).

**L4 (giới hạn thông tin). Một cặp không tách được gauge, sai số và lệch thời gian** (mục 1). Không cơ chế nào trong
`T` giải được điều này; chỉ hiệu chỉnh đồng hồ và hồi quy theo v trên dữ liệu có chuyển động (S2) mới tách được.

## 4. So với thực hành chuẩn

- **REP-105:** `map→odom` = ước lượng map∘(odom→base)⁻¹ tại cùng stamp, odom liên tục, map đúng toàn cục; hiệu chỉnh
  chỉ đúng tại robot. `T` đúng là cấu trúc này (L = odom, P = map). Điểm **khác thường**: bộ điều khiển (PX4) chạy trên
  frame "map" có GNSS kéo, còn planner chạy trên "odom". Trong ROS bộ điều khiển thường chạy trên odom chính vì lý do
  này. `T` phải trừ mọi dịch chuyển của map khỏi lệnh, trong băng thông của nó. Đây là lý do K1 và `τ` là trọng tâm.
- **robot_localization (dual EKF):** quy tắc là EKF local không bao giờ nhận đo toàn cục; EKF global được nhận odom.
  Hệ này tuân theo sau D47 (LIO không nhận PX4). EKF2 nhận EV là chuẩn. Cách chữa chuẩn khi **odom** bị **map** nuôi là
  cắt đường đó, và D47 đã làm.
- **Cộng đồng PX4 (FAST-LIO → EV → EKF2, mọi thứ trong frame local PX4):** đúng ở no_gps, vì EKF2 ≡ LIO (bảng mục 1,
  `T` ≈ đồng nhất đo được). Ở gps + EV hiếm khi được làm. Cái giá là map bị GNSS kéo (phương án S, mục 6).

## 5. Ổn định của vòng kín do `T` tạo ra

**Mô hình** (`loop_model.py`, một trục ngang): `T_used` → offset setpoint → vòng PX4, với tham số v1.17 mặc định
`MPC_XY_P` 0.95, `VEL_P/I/D` 1.8/0.4/0.2, trễ attitude 0.1 s **giả định**. Bộ điều khiển PX4 tác động lên ước lượng trễ
d_E. Cặp lấy tại stamp có lệch đồng hồ δ. Sau đó bộ lọc τ chạy ở 10 Hz và qua trễ truyền λ.
- Vì hai bộ ước lượng cùng ước lượng x, x triệt tiêu trong `T_inst`, chỉ còn qua D = d_E − δ − d_L.
- Hàm vòng: `L(s) = M(s)·F(s)·e^{−sλ}·(e^{−s(d_E−δ)} − e^{−s·d_L})`, nên |L| ≲ |D|·ω·|M F|.

| τ | λ | D | sup\|L\| | min\|1−L\| |
|---|---|---|---|---|
| 0 (lệnh tương đối) | 0.05–0.2 s | ±200 ms | 0.214–0.289 | ≥ 0.787 |
| 0.3 s | 0.05–0.2 s | ±200 ms | 0.200–0.261 | ≥ 0.803 |
| 2 s | 0.05–0.2 s | ±200 ms | 0.088–0.097 | ≥ 0.917 |

- **Kết luận:** ổn định theo định lý độ lợi nhỏ, biên ≥ 3.4× ở mọi τ, kể cả τ = 0, ở cả gps và no_gps (khác nhau chỉ ở D).
- Vòng trong của PX4 theo d_E: biên pha 59° / 54° / 48° / 37° ở 0 / 50 / 100 / 200 ms; trễ là chuyện của PX4, không
  của `T`. Đường ước lượng LIO → EV → EKF2 → `T` là chuỗi hở sau D47, không phải vòng.

**Sai số thêm vào khi bay** (cruise 5 m/s rồi phanh 3 m/s², đã trừ nền bám riêng của PX4, đơn vị m, cruise/sau phanh):

| Nguồn | τ 0 | τ 0.3 | τ 2 | C (không cập nhật) |
|---|---|---|---|---|
| trễ thật của PX4, 50 ms | 0.005/0.18 | 0.006/0.21 | 0.032/0.29 | 0.25/0.25 |
| trễ thật của PX4, 100 ms | 0.011/0.37 | 0.015/0.43 | 0.076/0.60 | 0.51/0.50 |
| lệch đồng hồ cặp, 50 ms | 0.25/0.25 | 0.25/0.25 | 0.24/0.25 | 0.00/0.00 |
| lệch đồng hồ cặp, 100 ms | 0.50/0.50 | 0.50/0.50 | 0.49/0.49 | 0.00/0.00 |

- `T` bù đúng trễ thật; `τ` chậm làm xấu pha chuyển tiếp (0.60 m so với 0.37 m). `T` hấp thụ **sai** lệch đồng hồ
  ghép cặp, dù τ là bao nhiêu; C miễn nhiễm với nó, và đó là ưu điểm duy nhất của C.

**SITL hiện có chống đỡ được gì.** Chỉ hover: D ngang không đo được (v ≈ 0), kênh dọc cho 8–15 ms ở gps (mục 1),
không có tăng/giảm tốc ngang, không có lệnh P/V/A.

Mọi kết luận về ổn định và K1 khi bay ở trên là **mô hình**. Con số "2.32° / −11.75°" mà v2 nhắc tới không có trong repo
(grep `docs/`, `.superpowers/`). UNVERIFIED, không dùng.

## 6. Các phương án mà phản biện trước chưa cân

| | S: một frame = local PX4 | L: một frame = LIO, PX4 chỉ vòng trong | C: `T` hằng mỗi epoch + giám sát | R: lệnh tương đối tại xe, goal tuyệt đối |
|---|---|---|---|---|
| Mất đi | `T`, cặp, DV2 | `T` trên đường 50 Hz | lọc, cổng, DV2 trên đường lệnh | lọc chậm (`τ` → nhỏ) |
| Sinh ra | gps: map đăng ký bằng pose EKF2, nên mỗi lần GNSS kéo hay reset là map nhoè hoặc nhảy; LIO phải nhận pose PX4 (vòng odom←map, trái chuẩn) | vòng bay flight-critical qua DDS, vẫn cần `T` cho D19/D11 (phản biện trước) | sai số = bất đồng tích luỹ từ đầu epoch | nhiễu ghép vào setpoint (đo được: mm ở hover) |
| Ca xấu | GNSS nhảy: map hỏng ⇒ va chạm | LIO hỏng ⇒ điều khiển hỏng | ví dụ dưới | lệch đồng hồ cặp vào thẳng lệnh (như mọi τ) |
| no_gps | ≡ hiện trạng (`T` ≈ I) | như gps | ≡ hiện trạng (`T` ≈ I) | ≡ hiện trạng |
| Công sức | lớn (Mapping + LIO + D19) | ≥ một lát cỡ S1b | nhỏ | rất nhỏ (một giá trị `τ` + vai trò cổng) |
| Bằng chứng S2 sẽ quyết | không cần | K2 trội (O36) | — | K1 theo v, độ dốc cặp theo v |

**C có chấp nhận được không: không, ở gps.**
- Ví dụ của v2 (đã tính lại, `geometry_checks.py` mục 2): LIO x = 300, PX4 x = 302, goal 310.
  - C cho setpoint 310: PX4 thấy sai số 8 m, trong khi planner muốn 10 m, tức lệch tương đối 2 m.
  - `T` tức thời cho 312: lệch 0 m.
- Thêm bất đồng heading tích luỹ Δψ từ gốc epoch (mục 3 của script): C lệch 3.36 / 5.75 / 10.97 m ở Δψ = 0.5 / 1 / 2°
  tại 300 m. `T` tức thời lệch 0, chỉ còn Δψ_nhiễu × lookahead (0.1° × 10 m = 17 mm). Cộng thêm độ nghiêng ε·d (mục 1).
- C chỉ "chấp nhận được" ở no_gps, nơi nó trùng với hiện trạng, nên không mang lại gì. Ưu điểm thật của C (miễn nhiễm
  lệch đồng hồ) đạt được rẻ hơn bằng cách hiệu chỉnh đồng hồ.
- **R** là dạng đúng về toán cho đường lệnh: `sp_P = p_P + R_ψ(g_L − p_L)`, tức là `T` với `τ` → 0 (đại số của v2). Nó
  không phải kiến trúc mới, mà là một giá trị `τ` cộng với việc tách vai trò của cổng.

## 7. BACKUP sau khi LIO mất: có an toàn không, trong frame nào, ngân sách sai số

- **Frame.** BACKUP chứng nhận trong map LIO, thực thi qua `T` FROZEN trong frame PX4: xe bám ước lượng PX4. An toàn
  đòi tổng sai số trên cả quãng thực thi nhỏ hơn khe hở của certificate.
- Inflation của config baseline (kế thừa từ `main`) là 5 × 0.2 = 1.0 m (`config/runtime/planner.yaml:221-227`); world
  của v2 chưa xây.
- **Ngân sách** (`backup_budget.py`, đầu vào là **giả định**): t = 1 + v/3 s, d = v + v²/6; thành phần: K1 lúc đóng
  băng (0.05 m/s × τ), lỗi vận tốc EKF2, bias gia tốc 0.03 m/s², heading (gps 0.5°, no_gps 2° sau reset compass), LIO
  trôi khi DEGRADED 0.1 m.

| Profile | v | RSS τ 2 / 0.3 (m) | tổng xấu nhất τ 2 / 0.3 (m) |
|---|---|---|---|
| gps | 1 / 3 / 5 m/s | 0.15 / 0.17 / 0.21; 0.11 / 0.14 / 0.19 | 0.28 / 0.36 / 0.47; 0.19 / 0.27 / 0.38 |
| no_gps | 1 / 3 / 5 m/s | 0.16 / 0.24 / 0.39; 0.13 / 0.22 / 0.38 | 0.33 / 0.52 / 0.76; 0.25 / 0.43 / 0.67 |

- Có vẻ trong khe hở 1.0 m nếu giả định đúng, nhưng không có chứng minh: phần LIO trôi trước LOST không quan sát được
  (D46), và ở no_gps L1 làm BACKUP không chạy được. Không thể nói "tiếp BACKUP an toàn hơn bàn giao ngay", cũng không
  thể nói ngược lại (đồng ý v2).
- **Thí nghiệm đề xuất (S2/S4).** Cùng kịch bản, cùng thời điểm cắt LiDAR (lidar_cut có seed), 1/3/5 m/s, vật cản ở
  khe hở 1–2 m, hai profile, mỗi ô ≥ 5 lần. So: (A) tiếp BACKUP qua `T` FROZEN; (B) bàn giao ngay (AUTO_LOITER);
  (C) commit BRAKE/HOLD ngay.
- Đo: khoảng cách nhỏ nhất tới vật cản (ground truth), quãng dừng, sai số so với quỹ đạo đã chứng nhận, hành vi PX4 sau đó.
- **Phải có trước khi chạy:** `navigation` (Supervisor tối thiểu có BACKUP/BRAKE) và `px4_mode` gửi P/V/A, cả hai
  **chưa xây** (`docs/architecture/AS_BUILT.md:77`); scene có vật cản; lidar_cut (plan S1c Task 5); bộ đo khoảng cách tới
  vật cản từ ground truth; quyết định O38 (nếu không, (A) ở no_gps chỉ đo được L1).

## 8. Bản phản biện v2: nhận, bác, xung đột

| Nhận định của v2 | Kiểm | Kết luận |
|---|---|---|
| Rút C | ví dụ 300/302/310 tính lại khớp; mở rộng với Δψ (mục 6) | **Nhận**; bổ sung: ở no_gps C ≡ hiện trạng, ưu điểm duy nhất là miễn nhiễm lệch đồng hồ |
| Residual nhỏ tại xe không chứng minh điểm xa, map, vị trí thật | mục 1, 2 | **Nhận**; mạnh hơn: ở no_gps residual ≈ 0 theo cấu trúc, không phải độ đo chính xác |
| FROZEN không chứng minh BACKUP an toàn | mục 7 | **Nhận**; mạnh hơn: ở no_gps quy tắc hiện tại loại luôn BACKUP (L1) |
| `navigation`, `px4_mode`, bộ phát TrajectorySetpoint chưa xây | `docs/architecture/AS_BUILT.md:77`; không có package `px4_mode` | **Nhận** (đã kiểm) |
| Đại số `sp_P = p_P + R(g_L − p_L)` | đúng; đó là `τ` → 0 | **Nhận**; nó là R, định nghĩa K1 |
| Bảng cánh tay đòn yaw | tính lại khớp mọi ô | **Nhận** |
| `f2faa1e` sửa hình học đúng, chọn điểm neo là vốn có | đọc commit | **Nhận** |
| `29241ce` (1e300, NaN) là kiểm input | đọc commit | **Nhận**: sơ suất |
| `6f555e8` lỗi ngữ nghĩa dữ liệu nghiêm trọng ở biên ước lượng | đọc commit: phương sai hướng theo trục world thay vì body | **Nhận**, nhưng nó thuộc đường EV, có ở mọi phương án, không nói gì về `T` |
| D47: VALID phải mang danh tính + epoch | đúng; còn thiếu ở interface (I2) | **Nhận và mở rộng** |
| DV2 là lỗi đồng bộ thật; 32.90 s → 37.53 s | `log/sitl/20261014T062427Z-gps-hover-yaw1.2-gnssout10/events_px4_bridge.jsonl` dòng 58 (VALID→INVALID STALE 32.90), 59 (INIT 32.92), 83 (VALID 37.53) | **Nhận**: 4.6 s không có `T` khi cả hai luồng tốt |
| Biên "2.32° / −11.75°" là của mô hình | không tìm thấy nguồn | **Không kiểm được**; mô hình của báo cáo này cho số khác (mục 5), cũng chỉ là mô hình |
| Không xoá `T` vì phức tạp, không giữ vì residual nhỏ | | **Nhận** |
| (ngầm) mọi chức năng của `T` cần chứng minh như nhau | | **Bác một phần**: I3, I4, G và đo tại xe đã chứng minh được bằng đại số; phần cần đo chỉ là K1 khi bay, BACKUP và đồng hồ |

**Xung đột với v2.** v2 giữ A′ "chờ kiểm" mà không chỉ ra lỗi logic cụ thể. Báo cáo này chỉ ra L1 (cần quyết định),
L2 (`τ` và cổng nên đổi theo trigger) và I2 (interface).

## 9. Lịch sử lỗi của `T`: triệu chứng khái niệm hay sơ suất

| Lỗi | Loại | Lý do |
|---|---|---|
| Cánh tay đòn ở 300 m (O12, `f2faa1e`) | khái niệm, đã sửa đúng | `T` chỉ tuyến tính hoá đúng tại xe; S1a đo sai điểm |
| Reset heading cộng thẳng (O11, G) | khái niệm, đã đúng trong code | reset PX4 là phép đổi hệ quanh xe |
| Pose 1e300 (`29241ce`) | sơ suất | kiểm input |
| Phương sai hướng EV sai frame (`6f555e8`) | sơ suất ở đường EV | không thuộc `T` |
| FROZEN → VALID mọi epoch (O34/D47) | khái niệm: danh tính frame | còn ở interface (I2) |
| Hai đồng hồ 10 s (O35/D48) | thiết kế: hai chủ | cùng họ với I7 (cổng → bàn giao) |
| DV2 | sơ suất cài đặt, nhưng là triệu chứng của L4 | cặp bỏ sớm (`src/px4/uavnav_px4_bridge/src/alignment.cpp:163` tại `86eaa28`); yêu cầu thời gian là vốn có |
| Reset khi FROZEN → INVALID loại BACKUP no_gps (L1) | khái niệm, mới | giữ fail-closed trong khi thông tin (delta) có sẵn |

## 10. S2 phải đo gì

1. Chuyển động có lệnh 1 / 3 / 5 m/s, đoạn thẳng, rẽ, tăng/giảm tốc 1–3 m/s², hai profile, ≥ 3 lần mỗi ô (O36).
2. Xoay yaw lớn (Y3a, ≥ 90°) khi đang bay: `residual_yaw`, K1 và sai số vị trí theo lookahead.
3. Quãng 100–600 m: sai số so với ground truth của LIO, của PX4, của xe so với lệnh; sai số yaw theo khoảng cách lookahead.
4. K1 = `T_inst − T_used` theo v và a; hồi quy residual của cặp theo v để tách lệch đồng hồ khỏi trễ thật (mục 1).
5. Tính nhất quán P/V/A: V và A đã xoay bằng `yaw_T`, so với đạo hàm của P đã biến đổi; `dT/dt` có đáng kể không.
6. K2 (PX4 bám setpoint trong frame của nó) từ ULog, K3, K4, K5 như O36; ghi setpoint đã gửi kèm `T`, epoch, bộ đếm reset.
7. Kết quả BACKUP khi LIO mất: thí nghiệm ba hành vi ở mục 7.
8. no_gps lidar_cut (S1c Task 5): có reset heading về compass khi EV dừng hay không, độ lớn bao nhiêu, `T` đi đâu (L1).

Kết quả **chỉ có từ mô hình**, chưa kiểm trên PX4: ổn định vòng `T` (mục 5), biên pha PX4 theo trễ, bảng sai số
cruise/phanh, H_G/H_E của EKF2, ngân sách BACKUP.

## 11. Phán quyết

**(a)** `T` chấp nhận được. Nó là hiệu chỉnh `map→odom` chuẩn, không mâu thuẫn logic, với **năm điều kiện**:
1. chỉ dùng tại xe và horizon ngắn, không chuyển hình học xa như phép cứng;
2. lệch đồng hồ ghép cặp ≪ sai số cho phép / v;
3. mang danh tính frame (I2);
4. odom không nhận map (D47);
5. `z_T` được phép trôi (4-DoF + độ nghiêng).

Không điều kiện nào là "tính năng thêm". Tất cả là ràng buộc dùng.

**(b)** Từng đặc điểm:

| Đặc điểm | Giữ / đơn giản / bỏ |
|---|---|
| đo tại xe, 4-DoF, G, `T` theo epoch, 4 trạng thái, INIT 20 cặp | **giữ**: có cơ sở toán |
| `τ_T` = 2 s | **đơn giản hoá**: chọn bằng τ* = (σ²_cặp·dt/(4r²))^{1/3}, tức 0.1 s (r = 0.05 m/s) tới 1.4 s (r = 1 mm/s, SITL), σ = 10 mm, dt = 0.1 s; giá trị đặt từ K1 của S2 |
| cổng nhảy | **đổi vai**: chỉ loại cặp đơn lẻ; bất đồng kéo dài thành event health cho Supervisor; không tự dẫn tới INVALID |
| FROZEN | **giữ**, nhưng đổi nghĩa thành "quan hệ lúc đóng băng" (mục 2) |
| reset PX4 khi FROZEN → INVALID | **đổi** theo O38 |
| cơ chế ghép cặp | **giữ**, sửa DV2 bằng cặp chờ (O36) |
| `/alignment` | **thêm** epoch LIO và ba bộ đếm reset PX4 (yêu cầu cho `px4_mode` ở S2) |

**(c)** Chọn **A′ với `T`** bây giờ; không chọn S, L hay C. Trigger cho từng chỉnh sửa:
- **R (`τ` nhỏ, cổng thành giám sát)** khi S2 cho K1 p95 > 0.05 m ở 3–5 m/s, hoặc K4 > 0 không do DV2.
- **Hiệu chỉnh đồng hồ** trước mọi thứ khác, nếu độ dốc residual theo v ngang > 10 ms.
- **L/B** chỉ theo trigger K2 của O36.
- **S** không bao giờ ở gps: map bị GNSS kéo.

**(d)** Ba rủi ro lớn nhất, mỗi rủi ro một phép đo rẻ:
1. **L1 có thật** (BACKUP no_gps mất), hoặc ngược lại EKF2 không reset. Phép đo: run lidar_cut no_gps của S1c Task 5,
   đọc `heading_reset_counter` và `/alignment`. Chỉ SITL, đã có kế hoạch.
2. **Lệch đồng hồ ghép cặp khi bay ngang lớn hơn mức thấy ở kênh dọc.** Phép đo: Task 6 S1c (POSCTL 1/3/5 m/s), chạy
   `lag_regression.py` thêm trục ngang trên log. Offline.
3. **GNSS thật lang thang > 0.25 m/s** (L2), không tái hiện được trong SITL. Phép đo: phát lại offline `T` trên một log
   GNSS thật công khai hoặc log của owner, đo tần suất từ chối cặp ở τ 2 s và 0.3 s. Offline, chưa có dữ liệu.

## 12. UNVERIFIED

- Toàn bộ mục 5 và mục 7 (mô hình); hành vi PX4 ở L1 và cửa sổ đua L3 (đọc code, chưa đo).
- Thành phần `v·Δ` ở trục ngang.
- Trường hợp H_G = 0.5 khi mất GNSS.
- Bias IMU và độ nghiêng trên phần cứng.
- Nguồn của số "2.32° / −11.75°".

## 13. Phương pháp và script

Script trong scratchpad của phiên (`.../scratchpad/tmath/`, không commit). Chạy bằng `python3 -I`, chỉ đọc
`log/sitl/`, numpy 1.26:
- `geometry_checks.py`: bảng cánh tay đòn, ví dụ 300/302/310 kể cả có Δψ, độ nghiêng;
- `ekf_transfer.py`: KF ổn định ba trạng thái, đáp ứng tần số H_G, H_E;
- `lag_regression.py <run>`: hồi quy hiệu với ground truth theo vz (mục 1);
- `loop_model.py`: sup|L|, min|1−L|, biên pha vòng trong, mô phỏng thời gian 1 ms;
- `gate_cascade.py`: luật VALID của `alignment.cpp` (chỉ tịnh tiến) với một rampe bất đồng;
- `backup_budget.py`: ngân sách mục 7.
