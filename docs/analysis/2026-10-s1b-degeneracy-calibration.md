# Hiệu chỉnh ngưỡng suy biến LIO từ chuyến quét độ cao trong SITL (S1b Task 13, 2026-10-13)

> **Cập nhật 2026-10-14 (D36):** owner chốt rằng suy biến khi bay trong map trống là bình thường và không cố khắc phục. Ngưỡng giữ giá trị beta, ba phương án hiệu chỉnh ở mục 7 không còn là việc phải làm; công cụ và pattern `altitude_sweep` được giữ làm chẩn đoán.

Báo cáo này ghi chuyến quét độ cao SITL dùng để đặt ngưỡng suy biến của LIO (spec §3.2: "lấy từ log trị riêng trên
scene mở và scene suy biến (bay cao) trong SITL"; D20, D28, F36, O6). Artifact nằm local ở
`log/sitl/20261013T162711Z-gps-altitude_sweep/` (không commit): `summary.json`, `calibration.json`, bản ghi JSONL,
event log, `harness.log`.

**Kết quả:** chuyến bay chạy đủ tám bậc 2.5..20 m và **PASS** ba kiểm dữ liệu cùng `phases_ok`. Nhưng LIO vào LOST lúc
37.1 s, ở độ cao ≈ 4.76 m, trên đường leo từ bậc 2.5 m lên bậc 5 m. Trước LOST có 31 cửa sổ 1 s, và drift ≤ 0.05 m ở
**cả 31**. Dữ liệu vì vậy **không có vùng suy biến** để đặt ngưỡng: quy tắc của plan trả về "insufficient support"
cho cả hai khối. **Cặp beta được giữ nguyên**: `degeneracy_translation_min_info = 1.1e5`,
`degeneracy_rotation_min_info = 2.8e6`. `config/lio/sim.yaml` không đổi. Chỉ chú thích dẫn xuất trong `config.hpp`
được sửa để trỏ tới báo cáo này. Mục 7 nêu chuyến bay nào sẽ cho đủ dữ liệu. Chọn cách nào là việc của owner.

## 1. Phương pháp (plan S1b Task 13)

- **Chuyến bay:** `make sitl-hover PROFILE=gps PATTERN=altitude_sweep`. Chỉ bay profile gps, vì EV mất khi LIO suy
  biến và PX4 cần GNSS để giữ vị trí. Xe leo từng bậc 2.5 m tới 20 m trên độ cao mặt đất đo lúc `wait_ready`, giữ mỗi
  bậc 10 s `/clock` với stick giữa ở mode Position, rồi `NAV_LAND`.
- **Lý do thông tin giảm:** Mid-360 không thấy gì dưới −7° (F36). Bộ lọc preprocess bỏ điểm xa hơn 40 m
  (`preprocess_max_range_m`). Vì vậy mặt đất ra khỏi tầm ở h ≈ 40 · tan 7° ≈ 4.9 m. Đỉnh các tường 4–5 m của world
  `px4_lio_smoke` ra khỏi tầm nhìn ở h ≈ 5.1–5.9 m. Trên khoảng 6 m, scan không còn điểm nào trong tầm.
- **Dữ liệu:**
  - mỗi scan: trị riêng nhỏ nhất của khối tịnh tiến và khối xoay, lấy từ event `LioScan` của node `lio`;
  - mỗi cửa sổ 1 s: drift |Δp_lio − Δp_gt| của `/lio/state` so với ground truth Gazebo, sau `T`. Yaw của `T` là trung
    bình vòng của `/alignment` VALID trước LOST: 1.5402 rad, độ trải 0.0005 rad. Chuỗi biến đổi là FLU → FRD → NED
    → ENU.
  - Mỗi cửa sổ mang trị riêng nhỏ nhất của các scan nằm trong nó.
- **Quy tắc ngưỡng** (`tools/uavnav/lio_calibration.py`), cho từng khối:
  - Ngưỡng là mức nhỏ nhất L trên lưới log (20 mức mỗi decade) sao cho các cửa sổ có trị riêng ≥ L drift ≤ 0.05 m
    trong ≥ 95% số cửa sổ.
  - Mọi mức cao hơn còn ≥ 50 cửa sổ phía trên cũng phải giữ điều đó.
  - Một ngưỡng chỉ được đề xuất khi có ≥ 50 cửa sổ mỗi phía **và** tập toàn bộ cửa sổ không giữ được 95%, tức là có
    vùng suy biến thật. Nếu không, công cụ báo "insufficient support" và không bịa ra ngưỡng.
  - Scan và cửa sổ từ LOST đầu tiên trở đi bị loại. Ở S1b, LIO đã LOST thì ở lại RESTARTING (O25), và `/lio/state` chỉ
    còn tích phân IMU.
- **Kiểm dữ liệu** (`sitl_checks.py --sweep`; không chấm tiêu chí hover):
  - `sweep_steps`: đủ tám bậc, mỗi bậc giữ ≥ 10 s, chiều cao PX4 lệch ≤ 0.5 m;
  - `calibration_inputs`: mỗi bậc trước LOST được phủ bởi trị riêng `LioScan` và `/lio/state`, mọi bậc được phủ bởi
    ground truth, và `T` đã biết;
  - `events`.

  Thiếu dữ liệu thì kiểm FAIL.

## 2. Chuyến bay `20261013T162711Z-gps-altitude_sweep`

- **Trước khi bay:** máy rảnh (load 0.57, còn 8 GB, không có tiến trình px4/gz/XRCE/colcon, UDP 8888 trống). Lệnh
  chạy qua script bọc.
- **Kết quả harness:** `make` rc 0. Driver thoát sau 206 s wall. Teardown xong cả 4 session. Sau đó không còn tiến
  trình SITL nào và UDP 8888 trống. `SITL_SWEEP_RESULT=PASS`.
- **Các pha:** `wait_ready` 11.86 s (mặt đất z −0.011 m). `posctl` (PX4 đã ở POSCTL). `arm`. `sweep` 11.90–138.91 s.
  `land` 138.91–161.32 s (`nav_state` 18), disarm xong.
- **Các bậc:** mỗi bậc giữ 10.02 s, chiều cao PX4 lệch ≤ 0.158 m. Leo một bậc mất 4.7–5.4 s; bậc đầu mất 11.3 s vì
  tính cả cất cánh.

| Bậc (m) | Giữ từ–đến (s) | z ground truth (m) | Trạng thái LIO lúc giữ |
|---|---|---|---|
| 2.5 | 23.23–33.25 | 2.33 | TRACKING |
| 5.0 | 37.98–48.01 | 4.83 | LOST (từ 37.1 s) |
| 7.5 | 53.37–63.39 | 7.29 | LOST |
| 10.0 | 68.34–78.36 | 9.83 | LOST |
| 12.5 | 83.50–93.52 | 12.31 | LOST |
| 15.0 | 98.83–108.85 | 14.82 | LOST |
| 17.5 | 113.99–124.01 | 17.33 | LOST |
| 20.0 | 128.89–138.91 | 19.83 | LOST |

**Timeline LIO, `T` và EKF2** (thời gian sensor = `/clock`, D29):

- LIO: TRACKING lúc 5.8 s. TRACKING→DEGRADED (`DEGENERATE_SCANS`) lúc 36.2 s. DEGRADED→LOST
  (`DEGENERACY_PERSISTED`) lúc 37.1 s.
- Alignment: INIT→VALID lúc 9.8 s. VALID→FROZEN (`LIO_LOST`, do health không TRACKING) lúc 36.21 s. FROZEN→INVALID
  (`FROZEN_TOO_LONG`) lúc 46.3 s. Có hai `Px4ResetApplied`: lúc 7.78 s khi còn INIT, và lúc 19.12 s VALID→VALID. Cả
  hai đúng thiết kế (O22, O25).
- EV và EKF2: `cs_ev_pos` từ 6.92 s tới 35.78 s. Bridge gửi 302 mẫu EV. GNSS được fuse suốt chuyến (163/169 mẫu
  `estimator_status_flags`). PX4 giữ độ cao mọi bậc, và chuyến bay không bị ảnh hưởng khi mất EV.
- Event: không có `LioQueueOverflow`, `LioBackendFailed`, `ImuRateTooHigh`, `EventsDropped`, `LioScanDropped` hay
  `ImuGap`. Node lio nhận 1619 scan (10.00 Hz) và 32367 mẫu IMU.

## 3. Phân bố trị riêng theo bậc độ cao

Bảng tính trên mọi event `LioScan` có trị riêng. Từ bậc 5 m trở đi, LIO đã LOST và ESKF chỉ còn IMU nên đã trôi.
Trị riêng ở các bậc đó **chỉ để tham khảo** và không vào quy tắc ngưỡng.

| Đoạn | z (m) | Scan (lý do) | λ_t p5 / p50 / p95 | λ_r p5 / p50 / p95 | Điểm vào p50 |
|---|---|---|---|---|---|
| Mặt đất, trước arm | −0.01 | 65 SCAN_GOOD (cộng khởi động) | 4.85e5 / 6.47e5 / 6.70e5 | 2.58e7 / 3.38e7 / 3.54e7 | 8140 |
| Giữ 2.5 m | 2.33 | 100 SCAN_GOOD | 4.88e5 / 5.05e5 / 5.15e5 | 2.50e7 / 2.58e7 / 2.73e7 | 4790 |
| Giữ 5 m (sau LOST) | 4.83 | 101 SCAN_DEGENERATE | 678 / 758 / 1.51e4 | 7.95e4 / 1.05e5 / 2.54e6 | 1460 |
| Giữ 7.5–20 m (sau LOST) | 7.3–19.8 | 100% SCAN_EMPTY (603) | – | – | 717–752 |
| Hạ cánh (sau LOST) | 19.8 → 0 | 106 SCAN_EMPTY, 119 SCAN_DEGENERATE (0/0) | 0 | 0 | 866 |

**Đoạn leo 2.5 → 5 m** (33.25–37.98 s) cho thấy thông tin giảm liên tục. Các mẫu sau là scan đơn lẻ.

| t (s) | z (m) | Lý do | λ_t | λ_r | Điểm sau preprocess | quality |
|---|---|---|---|---|---|---|
| 33.2 | 2.33 | SCAN_GOOD | 5.02e5 | 2.56e7 | 2050 | 100 |
| 34.2 | 2.97 | SCAN_GOOD | 3.95e5 | 2.03e7 | 1773 | 100 |
| 35.0 | 3.71 | SCAN_GOOD | 2.48e5 | 1.52e7 | 1101 | 100 |
| 35.5 | 4.18 | SCAN_GOOD | 2.01e5 | 1.13e7 | 918 | 92 |
| 35.9 | 4.51 | SCAN_GOOD | 1.22e5 | 8.53e6 | 627 | 55 |
| 36.0 | 4.55 | SCAN_DEGENERATE | 1.08e5 | 8.41e6 | 555 | 49 |
| 36.5 | 4.66 | SCAN_DEGENERATE | 4.19e4 | 7.06e6 | 439 | 19 |
| 36.8 | 4.72 | SCAN_DEGENERATE | 1.26e4 | 2.07e6 | 384 | 6 |
| 37.0 | 4.74 | SCAN_DEGENERATE | 924 | 1.09e5 | 367 | 0 |

- Khối **tịnh tiến** là khối ràng buộc: nó xuống dưới 1.1e5 ở z ≈ 4.55 m. Lúc đó khối xoay còn 8.4e6, gấp 3 lần
  ngưỡng xoay.
- Khối xoay xuống dưới 2.8e6 muộn hơn 0.8 s, ở z ≈ 4.72 m.
- Toàn bộ khoảng 5e5 → 1e3 của λ_t diễn ra trong chưa tới 0.6 m độ cao (4.2 → 4.75 m), tức khoảng hai cửa sổ 1 s.
- Từ 50.0 s (z 6.31 m) mọi scan đều `SCAN_EMPTY`. Còn khoảng 740 điểm hữu hạn mỗi scan, nhưng không điểm nào qua
  được bộ lọc tầm 0.5–40 m.

**Biên của cặp beta ở hover:** đã xét bảy hover của Task 11/12, gồm 5,806 scan SCAN_GOOD trên toàn bộ các run (mặt
đất, cất cánh, hover 2.5 m).
- λ_t nhỏ nhất là 2.77e5, gấp 2.5 lần 1.1e5.
- λ_r nhỏ nhất là 5.59e6, gấp 2.0 lần 2.8e6.

## 4. Drift và quy tắc ngưỡng

- **Trước LOST:** có 156 cửa sổ 1 s có cả LIO và ground truth. 31 cửa sổ trong đó có scan và kết thúc trước LOST.
- **Drift của 31 cửa sổ này:** p95 0.0255 m, max 0.0445 m. Cửa sổ max kết thúc lúc 7.22 s, khi xe còn đứng trên
  đất, ngay sau khi vào TRACKING lúc 5.8 s.
- **Giữ 2.5 m (9 cửa sổ trọn trong lúc giữ):** p50 0.0022 m, p95 0.0053 m.
- **Đoạn leo:**

  | Cửa sổ kết thúc (s) | λ_t nhỏ nhất | drift (m) | Ghi chú |
  |---|---|---|---|
  | 34.22 | 3.95e5 | 0.0126 | |
  | 35.22 | 2.16e5 | 0.0111 | |
  | 36.22 | 8.13e4 | 0.0075 | các scan cuối của cửa sổ không được áp dụng (D31) |

  Cửa sổ 37.22 kết thúc sau LOST nên bị loại.
- **Sau LOST:** 125 cửa sổ chỉ có IMU, drift p50 2.10 m mỗi giây, max 6.92 m. Các cửa sổ này bị loại.
- **Kết quả quy tắc** (`python3 -m tools.uavnav.lio_calibration log/sitl/20261013T162711Z-gps-altitude_sweep`, exit
  3):

  ```text
  LIO_CALIBRATION=UNSUPPORTED (translation: insufficient support: no degenerate region (drift <= 0.05 m in 100.0%
  of all 31 windows, so every level keeps 95% and the data cannot place a threshold)); the beta pair stays
  ```

  Khối xoay cho cùng kết quả (`calibration.json`, `blocks.rotation.unsupported`). Cả hai phía đều thiếu hỗ trợ: phía
  "suy biến" có 0 cửa sổ drift > 0.05 m, và tổng số cửa sổ (31) dưới 50.

## 5. Kết luận và cặp ngưỡng

- **Giữ cặp beta** `1.1e5 / 2.8e6`. Dữ liệu không ủng hộ một cặp khác. Không có cửa sổ nào phía trên ngưỡng beta drift
  quá 0.05 m (31/31), nên không có bằng chứng ngưỡng beta quá thấp. Dữ liệu cũng không thể cho thấy một ngưỡng thấp
  hơn là an toàn (mục 7).
- Cặp beta tách rõ hover khỏi scene suy biến. Hover có biên 2.5× (tịnh tiến) và 2× (xoay). Scene chỉ có tường (mục 6)
  thấp hơn ngưỡng tịnh tiến khoảng 120× (và ngưỡng xoay khoảng 26×).
- `config/lio/sim.yaml` không đổi, nên không cần build lại hay bay hover lại để xác nhận. Chú thích dẫn xuất trong
  `src/estimation/uavnav_lio_core/include/uavnav/lio/config.hpp` (chỉ chú thích) nay trỏ tới báo cáo này, thay cho
  "S1b recalibrates both".

## 6. Trị riêng của scene chỉ có sàn và chỉ có tường

- **Chỉ có sàn (floor-only):** không xuất hiện trong world `px4_lio_smoke`. Mặt đất ra khỏi tầm 40 m ở h ≈ 4.9 m,
  sớm hơn đỉnh tường (5.1–5.9 m). Giá trị tổng hợp của S1a (test
  `LioPipeline.PlaneOnlyScansNeverReachTrackingAndBlocksAreNotSwapped`, RecordProperty) là λ_t = 1722 (1.6% ngưỡng)
  và λ_r = 1.50e5 (5.4% ngưỡng). Hành lang tổng hợp có λ_t = 4406.
- **Chỉ có tường** (SITL, scan 37.0 s ở z 4.74 m, ngay trước LOST, khi ESKF còn bám): λ_t = 924 (0.8% ngưỡng), λ_r =
  1.09e5 (3.9% ngưỡng). Lúc giữ 5 m, sau LOST, λ_t p50 là 758. Khi không còn sàn, phương thẳng đứng gần như không
  được ràng buộc, nên khối tịnh tiến sụp trước.
- Cả hai trường hợp đều thấp hơn ngưỡng tịnh tiến beta 60–120 lần (và ngưỡng xoay 19–26 lần), nên quy tắc suy biến
  bắt được chúng chắc chắn.

## 7. Giới hạn của phương pháp, và chuyến bay nào sẽ cho đủ dữ liệu

Chuyến quét theo plan không thể cho ≥ 50 cửa sổ ở phía suy biến, vì ba lý do:

1. **LOST sau 1.0 s suy biến** (`lifecycle_degeneracy_lost_s`), và S1b không có bên gọi restart (O25). Mỗi chuyến
   bay vì vậy chỉ có khoảng một cửa sổ dưới cặp đang chạy.
2. **Scan bị đánh giá suy biến không được áp dụng vào predictor** (D31). Drift của cửa sổ dưới cặp đang chạy vì vậy là
   drift chỉ-IMU do chính thiết kế tạo ra, không phải chất lượng khớp scan. Một chuyến quét có thể cho thấy cặp đang
   chạy quá **thấp** (drift xấu ở phía trên nó), nhưng không thể cho thấy một cặp thấp hơn là an toàn.
3. **Trong world này thông tin sụp trong chưa tới 0.6 m độ cao**, và hai khối giảm cùng nhau. Chuyến quét không tách
   được hai khối: `block_rank_correlation` của công cụ đo điều này.

Các cách có thể cho đủ dữ liệu. Mỗi cách cần owner quyết định, vì đổi harness hoặc dùng config riêng cho lần hiệu
chỉnh:

- **(a) Replay "shadow":**
  1. Ghi `/lidar/points` và `/lidar/imu` của một chuyến quét.
  2. Chạy lại qua driver offline `LioPipeline` với cặp ngưỡng đặt ở cận dưới (1.0), chỉ cho lần replay đó, để mọi
     correction thành công đều được áp dụng.
  3. Đo drift theo trị riêng trên cùng dữ liệu, có thể lặp lại.

  Cách này cần thêm bước ghi bag vào harness.
- **(b) Chuyến bay dải hẹp:** giữ mỗi 0.1 m từ 3.8 m tới 4.8 m, 10 s mỗi bậc. Như vậy có khoảng 110 cửa sổ trải λ_t từ
  1e3 tới 5e5. Chuyến này dùng một file config LIO riêng cho lần hiệu chỉnh, với ngưỡng thấp và `degeneracy_lost` dài,
  truyền qua harness. `config/lio/sim.yaml` không đổi.
- **(c) World suy biến một phần** (`corridor.sdf`, `tunnel_*.sdf`): suy biến tịnh tiến theo một trục trong khi khối
  xoay vẫn tốt. Cách này tách được hai khối, điều mà chuyến quét độ cao không làm được.

Cho tới khi có một trong các cách trên, cặp beta là giá trị khởi đầu. Biên của nó ở mục 3 và mục 6.

## 8. `ImuGap` (chính sách "một khe IMU làm scan suy biến")

- Chuyến này có 0 `ImuGap` trên 32,367 mẫu IMU. Bảy hover của Task 11/12 cũng có 0, với khoảng 1.8 × 10⁴ mẫu IMU mỗi
  lần ở 200 Hz.
- Trong SITL, bước IMU là 4/8 ms (Task 3), dưới `imu_max_gap_s` 0.02 s. Chính sách "khe IMU > `imu_max_gap_s` làm scan
  đó suy biến và ESKF tiến thời gian không có correction" **chưa từng được kích hoạt** trong SITL.
- Việc xác nhận chính sách cần một lần chạy có tiêm khe IMU (S4 fault injection) hoặc phần cứng. Mục này vẫn là
  UNVERIFIED.

## 9. Ghi chú D28: `quality` và `EKF2_EV_QMIN`

- `/lio/odometry` chỉ được phát cho kết quả `kScanGood` khi TRACKING. Bất biến của `evaluate_degeneracy` là
  `quality >= 50 ⇔ !degenerate` (`degeneracy.hpp`). Vì vậy mọi odometry được phát ra có `quality` ≥ 50.
- Số đo khớp với điều đó:
  - bảy hover có `quality` = 100 ở toàn bộ 5,682 odometry;
  - chuyến quét có `quality` nhỏ nhất 55, ở 302 odometry (mẫu cuối lúc 35.9 s, khi λ_t = 1.22e5, ngay trên ngưỡng);
  - EV mà bridge gửi PX4 có `quality` nhỏ nhất 55.
- Với `EKF2_EV_QMIN = 0` (beta, O23), tham số này **không thể có tác dụng**. Mọi giá trị `EKF2_EV_QMIN` ≤ 50 đều trơ
  như nhau. Chỉ giá trị > 50 mới thêm một cổng lọc, và đó là một quyết định thiết kế (D28), không phải việc hiệu chỉnh.

## 10. UNVERIFIED

- Drift khi λ_t ở khoảng (1.1e5, 2e5) mà correction vẫn được áp dụng: chỉ có khoảng một cửa sổ (35.22 s, drift 0.011
  m), chưa đủ để kết luận.
- Trị riêng sau LOST (bậc 5 m trở lên) được tính với một ESKF đã trôi. Chúng chỉ dùng để tham khảo.
- Scene chỉ có sàn không được quan sát trong SITL. Chỉ có giá trị tổng hợp.
- Chính sách `ImuGap` (mục 8).
- Nhiễu và mẫu quét thật của Mid-360 khác mô hình Gazebo. Cặp ngưỡng cần kiểm lại trên phần cứng (P8).
- Mỗi cấu hình chỉ bay một lần, nên chưa biết độ lặp lại của độ cao LOST (≈ 4.76 m).

## 11. Tài liệu cần cập nhật (dành cho architect; báo cáo này không sửa chúng)

- `docs/TRACEABILITY.md`: dòng O6/§3.2 (ngưỡng suy biến) giữ trạng thái beta, chưa hiệu chỉnh. Bằng chứng: run
  `20261013T162711Z-gps-altitude_sweep`, báo cáo này, và các commit của S1b Task 13.
- `docs/architecture/SYSTEM_DESIGN.md` §3.2: câu "Giá trị khởi đầu của ngưỡng là một việc của lát S1" cần ghi rằng
  chuyến quét S1 không đủ hỗ trợ, và việc hiệu chỉnh chuyển sang cách (a), (b) hoặc (c) ở mục 7. §3.1: LOST sau 1.0 s
  suy biến cộng với không restart (O25) khiến một chuyến quét không thể thu dữ liệu dưới ngưỡng.
- `docs/architecture/DECISIONS.md`: cần một mục O mới để owner chọn cách hiệu chỉnh (a), (b) hoặc (c). Ghi chú D28:
  `EKF2_EV_QMIN` ≤ 50 trơ do cấu trúc.
- Plan S1b: Task 13 Step 3 "Set the pair" không thực hiện vì dữ liệu không đủ. Mục "Carry-over to S2" của Task 14 cần
  thêm việc hiệu chỉnh ngưỡng.

## 12. Tái tạo

```bash
make sitl-hover PROFILE=gps PATTERN=altitude_sweep      # bounded 300 s; summary.json (data checks)
python3 -m tools.uavnav.lio_calibration log/sitl/<run>  # bảng theo bậc, calibration.json; exit 0 / 3 / 2
```
