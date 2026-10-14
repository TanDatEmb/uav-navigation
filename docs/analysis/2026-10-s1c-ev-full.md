# EV fuse đầy đủ ở cả hai profile: đo trong SITL (S1c Task 1, D42, 2026-10-14)

Báo cáo này ghi lần đo SITL của cấu hình `EKF2_EV_CTRL=15` (EV vị trí, vận tốc, độ cao và yaw) ở cả hai profile, theo
D42 và plan [S1c v2](../plans/2026-10-14-s1c-recovery-heading-drift.md) Task 1. Thiết kế: SYSTEM_DESIGN §0 (profile),
§4.1 (tham số EKF2). Sổ các lần chạy: [2026-10-s1c-sitl-runs.md](2026-10-s1c-sitl-runs.md).

**Kết quả.**
- Lần 1 (gps) và lần 2 (no_gps) **PASS** đủ mọi kiểm cùng `phases_ok`.
- Lần 3 (gps, mất GNSS 10 s) **không thực hiện được**: PX4 v1.17 với Gazebo không xử lý lệnh tiêm lỗi GNSS. Không có
  ack, và GNSS vẫn được fuse suốt cửa sổ. Theo plan, lần 3 bị bỏ khỏi quy tắc quyết định.
- **Cấu hình EV đầy đủ được giữ** ở cả hai profile. Không có ngoại lệ theo profile, và không cần lần so sánh
  `EXPERIMENT_EV_CTRL=7`. Hành vi khi mất GNSS ở profile gps **chưa kiểm chứng**.
- Một phát hiện ngoài phạm vi: ở lần 3, `T` rơi VALID→INVALID (STALE) trong cửa sổ do chuỗi `NO_PX4_SAMPLE` dài hơn
  1 s (DV2, mục 6).

## 1. Thay đổi

- `config/px4/profiles/gps.yaml`: `EKF2_EV_CTRL` 7 → 15. `tools/uavnav/px4_profile.py` chốt 15 ở cả hai profile (D42).
- Bộ chấm `tools/uavnav/sitl_checks.py`:
  - **`f34_values` (D42):** PASS khi độ trải `yaw_T` trong cửa sổ ≤ 1°, và `T` ở VALID sau mọi `Px4ResetApplied`
    kể từ lần VALID đầu tiên. Giá trị `yaw_T` tuyệt đối và độ lệch so với π/2 − yaw spawn chỉ được báo cáo.
  - **`ekf2_heading`** (thay `ekf2_ev_yaw` của D20): chỉ báo cáo, không chấm. Gồm tỉ lệ thời gian theo từng nguồn
    heading (`cs_mag_hdg`, `cs_mag_3d`, `cs_ev_yaw`), dòng thời gian của nguồn, và mọi lần reset heading (thời
    điểm, độ lớn, trước hay sau VALID đầu tiên).
  - **`gnss_consistency`** (chỉ gps): `cs_gnss_pos` ≥ 95% thời gian cửa sổ, trừ khoảng mất GNSS được tiêm. Một yêu
    cầu tiêm mà PX4 không ack ACCEPTED thì FAIL.
  - **`ev_variance`**: chỉ báo cáo σ EV đã gửi, so với sàn của PX4.
  - **`events`** FAIL thêm khi có `LioNodeFailed`, `BridgeNodeFailed`, `LioPublishFailed`, `BridgePublishFailed`,
    `LioIngestReentered`, hoặc `LioNodeSummary.publish_failed` khác 0.
  - **Minor của S1b:** T11 M2 (báo thêm khoảng cách đến theo thời gian thực bên cạnh thời gian `/clock`); T11 M3 (tỉ
    lệ cờ tính theo thời gian, mỗi mẫu giữ tới mẫu sau, không theo số mẫu).
- Harness:
  - `GNSS_OUTAGE_S=<s>` (gps, hover): ở giây 20 của hover gửi `VEHICLE_CMD_INJECT_FAILURE` (GPS, OFF), `s` giây sau
    gửi OK, ghi ack vào `hover_meta.json`. `SYS_FAILURE_EN=1` chỉ đặt cho lần chạy này.
  - Recorder ghi thêm `cs_mag_hdg`, `cs_mag_3d`, `cs_yaw_align`, `delta_heading` và phương sai EV.
- Commit `a693bcf`. Test: `Px4ProfileTest.test_d42_ev_ctrl_is_full_in_both_profiles`, `SitlChecksTest.test_f34_passes_on_any_stable_offset`,
  `test_f34_fails_on_unstable_t`, `test_heading_sources_are_reported`, `test_gnss_dropout_outside_the_outage_fails_gps`,
  `test_ev_variance_is_reported`, `test_events_fail_on_publish_failures`. Kiểm đột biến: 16/16 mutant bị bắt.

## 2. Chấm lại bốn lần F34 của S1b (không bay)

Bốn lần này chạy với `EKF2_EV_CTRL=7` ở gps. Chấm trên bản sao symlink nên `summary.json` của S1b giữ nguyên.

| Run | `f34_values` | Độ trải `yaw_T` | `yaw_T` trung bình | Reset sau VALID |
|---|---|---|---|---|
| `20261013T154709Z-gps-hover` | PASS | 0.076° | 88.28° | 1 (VALID→VALID) |
| `20261013T154923Z-gps-hover-yaw1.2` | PASS | 0.039° | 19.53° | 1 (VALID→VALID) |
| `20261013T155144Z-no_gps-hover` | PASS | 0.031° | 0.006° | 0 |
| `20261013T155429Z-no_gps-hover-yaw1.2` | PASS | 0.035° | 0.005° | 0 |

Kết quả toàn lần là FAIL chỉ vì hai kiểm báo cáo mới thiếu dữ liệu và fail-closed: bản ghi S1b chưa có
`cs_mag_hdg` và `position_variance`. `gnss_consistency` PASS ở cả hai lần gps.

## 3. Ba lần bay

Cả ba lần: commit `a693bcf`, yaw spawn 1.2 rad (π/2 − yaw spawn = 21.25°), hover 60 s ở 2.5 m. Chạy từng lần một qua
script bọc. Load 1 phút lúc bắt đầu lần lượt là 0.78, 0.10 và 1.12. Sau mỗi lần không còn tiến trình SITL nào.

### Lần 1: gps (`20261014T061438Z-gps-hover-yaw1.2`): PASS

- **Nguồn heading theo thời gian** (thời gian `/clock`):
  - 7.86 s: căn yaw;
  - 8.06 s: `mag_hdg`;
  - 20.67 s: `mag_3d` tới hết lần chạy.
  - `cs_ev_yaw` **không bao giờ bật**. Cửa sổ: 100% `mag_3d`, `cs_yaw_align` 100%.
- **Reset heading:**
  - 7.88 s, +19.56°: căn la bàn, trước VALID đầu tiên (11.33 s);
  - 20.62 s, +0.014°: sau VALID; `T` đi VALID→VALID (`Px4ResetApplied`), ngoài cửa sổ (24.84–84.85 s).
- **`T`:** VALID suốt cửa sổ, độ trải 0.075°, `yaw_T` 19.53° (lệch −1.72° so với 21.25°: declination của mô
  phỏng, như S1b).
- **Drift** PX4 so với ground truth: 0.028 m. `cs_gnss_pos` 100%. `cs_ev_pos` 100%.
- **σ EV đã gửi:** vị trí p50 1.33 mm, max 1.47 mm; vận tốc p50 4.7 mm/s; yaw p50 0.16 mrad. Sàn của PX4 là
  `EKF2_EVP_NOISE` 0.1 m, và 0.5 m (`EKF2_GPS_P_NOISE`) khi GNSS đang fuse. 100% mẫu nằm dưới sàn.

### Lần 2: no_gps (`20261014T062206Z-no_gps-hover-yaw1.2`): PASS

- **Nguồn heading theo thời gian:**
  - 8.00 s: căn yaw;
  - 8.07 s: `mag_hdg`;
  - 9.76 s: EV yaw bắt đầu (một mẫu `mag_hdg+ev_yaw`), từ 9.96 s chỉ còn `ev_yaw`.
  - `cs_yaw_align` về false từ 9.76 s: với EV FRD, EKF2 xoá `yaw_align` khi bắt đầu EV yaw, đúng như đọc
    `ev_yaw_control.cpp`. Cửa sổ: 100% `ev_yaw`.
- **Reset heading:** 8.01 s, +20.49° (căn la bàn); 9.77 s, −20.47° (về heading của EV). Cả hai trước VALID đầu
  tiên (11.93 s), nên `T` không phải áp reset nào.
- **`T`:** VALID suốt cửa sổ, độ trải 0.053°, `yaw_T` 0.000°. Frame local của PX4 là frame khởi động của LIO, như S1b.
- **Drift** 0.007 m. `cs_ev_pos` 100%.
- **σ EV:** vị trí p50 1.30 mm, max 1.34 mm; vận tốc p50 4.6 mm/s; 100% dưới sàn 0.1 m.

### Lần 3: gps, mất GNSS 10 s (`20261014T062427Z-gps-hover-yaw1.2-gnssout10`): PX4 từ chối việc tiêm lỗi

- `SYS_FAILURE_EN=1` đã được áp: `px4_gz.log` ghi `SYS_FAILURE_EN: curr: 0 -> new: 1`.
- OFF gửi ở 45.512 s, OK ở 55.536 s. Không có `VehicleCommandAck` nào cho lệnh 420, và log PX4 không có dòng nào về
  việc này.
- `cs_gnss_pos` đúng ở cả 10 mẫu cờ trong khoảng tiêm, và đúng suốt cửa sổ.
- Lý do (đọc PX4 v1.17.0-14):
  - lệnh 420 cho GPS chỉ được `simulator_mavlink` xử lý (`SimulatorMavlink.cpp:1263`);
  - harness Gazebo lấy GNSS từ `gz_bridge` (`GZBridge.cpp:93`, navsat), mà `gz_bridge` không xử lý lỗi;
  - `FailureInjector` của commander chỉ xử lý motor, và không ack các unit khác;
  - Commander bỏ qua lệnh 420 (`Commander.cpp:1530`).
- Bộ chấm FAIL `gnss_consistency` ("injection was not accepted") như thiết kế. Theo plan, **lần 3 bị bỏ** khỏi quy
  tắc quyết định, và không có vòng sửa harness: đây là giới hạn của PX4, không phải lỗi harness.
- Các kiểm còn lại của lần 3 (heading, σ EV, drift 0.027 m) giống lần 1. Riêng `alignment_valid` và `f34_values`
  FAIL vì sự cố DV2 ở mục 6.

## 4. Tổng hợp theo yêu cầu của plan

| | Lần 1 gps | Lần 2 no_gps | Lần 3 gps + outage |
|---|---|---|---|
| Nguồn heading trong cửa sổ | `mag_3d` 100% | `ev_yaw` 100% | `mag_3d` 100% |
| Reset heading (sau VALID) | 1 (+0.014°, ngoài cửa sổ) | 0 | 1 (+0.019°, ngoài cửa sổ) |
| Trạng thái `T` qua các reset | VALID→VALID | không có reset | VALID→VALID |
| Độ trải `yaw_T` | 0.075° | 0.053° | 19.54° (INIT/INVALID trong cửa sổ, mục 6) |
| Drift ngang so với ground truth | 0.028 m | 0.007 m | 0.027 m |
| σ vị trí EV p50 / sàn PX4 | 1.33 mm / 0.1 m | 1.30 mm / 0.1 m | 1.35 mm / 0.1 m |
| Kết quả | PASS | PASS | bỏ (tiêm lỗi bị từ chối) |

## 5. Quyết định theo quy tắc của plan

- **Quy tắc:** cấu hình EV đầy đủ được giữ khi mọi lần chạy PASS, và ở lần 3 `T` vẫn VALID với drift ngang
  ≤ 0.5 m. Nếu không, không đổi profile mà bay một lần so sánh `EXPERIMENT_EV_CTRL=7`, rồi mở mục O cho owner.
- **Áp dụng:**
  - lần 1 và lần 2 PASS;
  - lần 3 bị bỏ, vì PX4 từ chối việc tiêm lỗi.
- **Kết luận:** `EKF2_EV_CTRL=15` được giữ ở cả hai profile.
  - Không cần lần so sánh, nên tuỳ chọn `EXPERIMENT_EV_CTRL` không được xây (D44.6).
  - Ở profile gps, PX4 tự chọn la bàn và không bao giờ bắt đầu EV yaw. Vì vậy, khi GNSS còn, EV đầy đủ cho đúng
    hành vi của D20 cũ.
- **Chưa kiểm chứng:** khi mất GNSS, EKF2 có chuyển sang EV yaw không (kèm một reset heading cỡ 20°), và `T` có
  hấp thụ reset đó khi đang VALID không. Phần `T` có test thành phần ở Task 4
  (`Px4Bridge.LargeHeadingResetAfterReinitKeepsTheVehiclePointContinuous`).
- **Cách khác để tạo mất GNSS** (chưa làm, cần controller quyết):
  - ngắt topic navsat phía Gazebo (ví dụ một bridge chặn có điều khiển);
  - hoặc dùng `simulator_mavlink` thay `gz_bridge` cho GNSS.

## 6. Phát hiện: `T` rơi STALE trong cửa sổ hover (DV2)

Ở lần 3, trước khi tiêm lỗi:

| `/clock` | Sự kiện |
|---|---|
| 30.0–31.9 s | `AlignmentPairRejected NO_PX4_SAMPLE` dày lên (mỗi 0.3–0.5 s một bản ghi đầu đợt) |
| 32.90 s | `AlignmentTransition` VALID→INVALID `STALE`, tuổi 1.1 s |
| 32.92 s | INVALID→INIT `LIO_TRACKING` |
| 33.0–37.4 s | INIT, `NO_PX4_SAMPLE` vẫn tiếp tục |
| 37.53 s | INIT→VALID `PAIRS_CONSISTENT` |

- **LIO:** TRACKING suốt lần chạy, không có transition nào.
- **Luồng dữ liệu đều:** `/lio/odometry` đến đều (khoảng cách lớn nhất 120 ms); `vehicle_local_position` đến đều
  (lớn nhất 24 ms, mẫu `timestamp_sample` cách nhau tối đa 24 ms).
- **Số đo gợi ý nguyên nhân:** stamp của odometry LIO **mới hơn** `timestamp_sample` mới nhất của PX4 tại lúc
  odometry đến.

  | Lần | Cặp có stamp LIO mới hơn mẫu PX4 mới nhất (trong 600 cặp của cửa sổ) | Bản ghi đầu đợt `NO_PX4_SAMPLE` trong cửa sổ |
  |---|---|---|
  | 1 | 0 | 0 |
  | 2 | 131 | 153 |
  | 3 | 316 | 164 |

  Mức chênh tối đa chỉ 4–8 ms, tức một tới hai bước mô phỏng. Khi cặp rơi vào phía "chưa có mẫu PX4" liên tục hơn
  1 s, `T` hết hạn (`alignment_valid_stale_s` 1.0 s).
- **Độ trải 19.54°** của lần 3 không phải `T` trôi: đó là các output INIT/INVALID trong cửa sổ, có yaw 0.0 khi `T`
  chưa VALID.
- **Xếp loại:** hành vi của alignment và bridge, không phải của harness, của EV đầy đủ hay của GNSS. Đây chính là DV2
  (plan Task 4 Step 5: đo stamp của cặp so với mẫu buffer mới nhất). Ở S1b nó chỉ làm rơi cặp; lần này nó lần đầu
  đưa `T` ra khỏi VALID trong cửa sổ hover. Không sửa ở Task 1.

## 7. Lệch so với plan

- Chữ ký `_check_gnss_consistency(data)` không có tham số `profile`. Kiểm này chỉ được tạo ở profile gps
  (`evaluate_hover`), nên no_gps không có khoá `gnss_consistency`.
- `ekf2_ev_yaw` (kỳ vọng của D20: gps không EV yaw, no_gps ≥ 95%) được thay bằng `ekf2_heading` chỉ báo cáo. Theo
  D42, PX4 tự chọn nguồn heading.
- Ngoài danh sách file của plan:
  - `Makefile` (biến `GNSS_OUTAGE_S`, cần cho lệnh của plan);
  - comment của `config/px4/profiles/no_gps.yaml` (bỏ trích dẫn D20).
- Độ trải F34 tính trên mọi mẫu `/alignment` trong cửa sổ, kể cả khi `T` không VALID. Như luật của plan, một reset
  heading lớn trong cửa sổ sẽ làm độ trải vượt 1° dù `T` hấp thụ đúng. Vì vậy bộ chấm báo thêm
  `max_spread_between_resets_rad` (độ trải trong từng đoạn giữa các reset), chỉ để báo cáo. Ba lần bay không có
  reset nào trong cửa sổ.

## 8. Chưa kiểm chứng (UNVERIFIED)

- Hành vi của EV đầy đủ khi mất GNSS ở profile gps (nguồn heading, reset, `T`, drift): chưa tiêm được lỗi (mục 3).
- Mỗi cấu hình chỉ bay một lần với yaw spawn 1.2. Thứ tự la bàn rồi EV yaw ở no_gps giống lần 4 của S1b.
- Cơ chế chính xác của DV2 (bridge tra buffer PX4 thế nào khi stamp LIO mới hơn mẫu PX4 mới nhất): mới đo ở mức lúc
  dữ liệu đến tại recorder, chưa đọc từ trạng thái buffer của bridge.
