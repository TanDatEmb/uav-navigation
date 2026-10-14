# Phản biện thiết kế S1c theo D44 (2026-10-14)

Báo cáo này xét lại thiết kế S1c (plan [2026-10-14-s1c-recovery-heading-drift.md](../plans/2026-10-14-s1c-recovery-heading-drift.md)
bản 14 task, SYSTEM_DESIGN §3.1/§3.4/§4.1, AR3, O29, O33) theo nguyên tắc D44 của owner. Người phản biện không bảo vệ
thiết kế cũ. Mỗi kết luận có nguồn: code (`file:dòng`), mã PX4 v1.17.0-14 (`~/Dev/Autopilot`), hoặc log SITL của S1b.
Đề xuất thay thế nằm ở AR4 trong [DECISIONS.md](../architecture/DECISIONS.md) và ở plan S1c v2. Owner chưa xác nhận.

## 0. Kết luận ngắn

- Phần lớn độ phức tạp của S1c v1 **dựng ra thông tin không có**: offer seed từ PX4, "thông tin tự thân", cổng innovation,
  cửa sổ đồng bộ lại. Mỗi thứ là một tầng phương pháp đặt lên một câu hỏi mà dữ liệu không trả lời được (D44.3).
- Các điểm nghẽn thật (mục 5) không phụ thuộc vào những tính năng đó: thêm chúng không đổi kết quả khi bay thật.
- Bản v2 giữ đúng phần owner đã chốt (D40–D43) ở mức tối thiểu: 7 task thay 14, 0 message mới thay 1, 1 tham số tầng (b)
  mới và bỏ 1 tham số thay 4 tham số mới, 3 cạnh mới thay 4, không cờ trạng thái mới.

## 1. Kiểm thông tin: mỗi tín hiệu nói gì, và có đúng bản chất không

| Tín hiệu | Thông tin vật lý thật có | Tín hiệu khẳng định | Trung thực? | Kết luận |
|---|---|---|---|---|
| Trạng thái LIO (TRACKING…) | Trị riêng thông tin của scan tại ước lượng hiện tại; LiDAR gap theo sensor time; σ từ P của ESKF | TRACKING = "ước lượng tin được" | Một phần. TRACKING chỉ có nghĩa "bị ràng buộc và hiệu chỉnh được áp", không có nghĩa "đúng". Khớp sai ở cấu trúc lặp vẫn có trị riêng lớn | Giữ; đổi cách đọc: TRACKING = có ràng buộc |
| Cờ suy biến (λ_min, 3/5 scan, 1 s) | Thật: tính từ Jacobian residual của chính scan | "Scan ràng buộc đủ 6 bậc" | Đúng khi ước lượng còn bám. Ở LOST, ICP với map đóng băng và prior đã trôi cho trị riêng **vô nghĩa** | Giữ; ở LOST không dùng kết quả ICP |
| σ vị trí > 0.5 m | P của ESKF, mô hình điểm độc lập σ 0.03 m | "Đủ bất định để LOST" | Lạc quan theo cấu trúc (không chứa bias, khớp sai, deskew) | Giữ cạnh có sẵn, không dựa vào nó |
| "Thông tin tự thân" (AR3) | Hình học của scan với mặt phẳng của chính nó | "Restart bây giờ sẽ thành công" | **Không**: dùng lại ngưỡng của phép đăng ký cho một đại lượng khác thang (không nhiễu đăng ký, không hạ mẫu như ICP) mà không có số đo (D36 cấm hiệu chỉnh). Câu hỏi "restart có thành công không" được trả lời miễn phí và đúng hơn bởi chính lần thử: INITIALIZING đánh giá suy biến thật trên map mới | **Bỏ**. Thay bằng: scan tới đều và không rỗng liên tục một thời gian chờ cố định |
| Cổng innovation (1.0 m / 10°) | Hiệu giữa output predictor và snapshot ESKF tại stamp scan | "Hiệu chỉnh này có hại" | **Không**: ESKF đã nhận phép đăng ký; chặn ở predictor làm predictor và ESKF lệch nhau mãi, mọi hiệu chỉnh sau bị chặn, rồi LOST. Ngưỡng là đoán, "chỉnh lại từ log" là quy tắc tự thích nghi (D44.7); sim không có khớp sai nên không có dữ liệu. EKF2 đã có cổng innovation EV (`EKF2_EVP_GATE` 5σ) và D42 giao PX4 phân xử | **Bỏ** |
| Kết luận "áp dụng nhưng không tươi" | Tầm tích phân IMU sau hiệu chỉnh (now − scan end) | "Output đã hiệu chỉnh nhưng không đủ để xác nhận TRACKING" | Đúng bản chất (tầm tích phân là hậu quả thật lên sai số output) | **Giữ ý, bỏ loại kết luận**: thành guard trên cạnh xác nhận sẵn có của lifecycle |
| Seed `SYNC` (`T⁻¹ × pose PX4`) | gps: GNSS ~0.5–1 m; no_gps: PX4 chỉ có IMU sau khi mất EV, tức chính LIO trôi đi một vòng qua PX4 | "Frame mới liên tục với frame cũ" | Seed của một map mới là **chọn gauge**, không phải thông tin cho LIO. Thông tin GNSS tới hệ qua bước INIT của `T` từ các cặp, như nhau với mọi seed. Ở no_gps là vòng phản hồi LIO → EV → PX4 → seed → LIO (D44.4) | **Bỏ** offer và message; LIO seed bằng pose predictor của chính nó |
| Cửa sổ đồng bộ lại + cạnh FROZEN → INIT sau 2 s | Cặp epoch mới bị cổng nhảy từ chối | "Frame mới không liên tục" | Vá triệu chứng của một quy tắc không trung thực: FROZEN → VALID nhận **bất kỳ** health TRACKING, kể cả của epoch khác (`alignment.cpp:356`). Đọc code: `go(kValid)` đặt `stale_since_` (`alignment.cpp:106`) nên `T` "VALID" sai tới 1 s rồi INVALID → INIT; O33 ghi "giữ FROZEN tới 10 s" là không khớp code | **Bỏ**. Quy tắc thật: `T` gắn với một epoch LIO; cặp của epoch mới hơn → INIT |
| `frozen_max` 10 s của `T` | Không: `T` đóng băng không "già" đi khi frame PX4 không đổi; ở no_gps PX4 tự báo mất vị trí sau 5 s | "T đóng băng hết tin cậy" | **Không**: đây là ngân sách bỏ cuộc 10 s của Supervisor chép sang một chỗ thứ hai (comment `alignment.hpp`: "= lio_recovery_timeout_s") | Đề xuất bỏ (O35) |
| `valid_stale` 1 s | Cặp tới 10 Hz khi TRACKING | "VALID mà không còn được kiểm" | Đúng | Giữ |
| `quality` EV | λ_min / ngưỡng, kẹp; mọi EV phát ra ≥ 50 | "Chất lượng EV" | Trơ: `EKF2_EV_QMIN=0` | Giữ làm log (D28) |

### 1a. EV có làm PX4 trôi theo không?

- **Sự thật về kênh bất định.** EKF2 không dùng thẳng covariance của EV. Trong `ev_pos_control.cpp`:
  - phương sai đo bị kẹp dưới bởi `max(var, EKF2_EVP_NOISE², 0.01²)`, với `EKF2_EVP_NOISE` mặc định 0.1 m (dòng 144–147);
  - khi GNSS đang fuse, kẹp dưới thêm `EKF2_GPS_P_NOISE²` (0.5 m, dòng 131–135);
  - FRD không có EV yaw và có GNSS thì EV vị trí đi qua bộ ước lượng bias (dòng 112–116): drift chậm của LIO bị hấp thụ vào bias, GNSS thắng về lâu dài.
- **Thứ LIO gửi** là P của ESKF (`backend.cpp:244`), mô hình điểm độc lập. Log S1b không ghi phương sai này
  (`vehicle_visual_odometry.jsonl` chỉ có vị trí, quality): **chưa đo**. Nếu nó dưới 0.1 m như thường gặp ở FAST-LIO, kênh
  covariance trơ hoàn toàn và độ bất định EKF2 dùng là **tham số cố định do ta đặt**.
- **Khi LIO suy biến:** không có EV (cổng nhị phân: chỉ TRACKING mới phát odometry, `frontend.cpp:256`). Không có đường
  "EV quá tự tin lúc suy biến".
- **Khi LIO trôi hoặc lệch mà vẫn TRACKING:**
  - ở gps, GNSS kéo lại (bias EV). Riêng EV **vận tốc** được fuse không có bias estimator, nên một lệch vận tốc của LIO đi thẳng vào EKF2;
  - ở no_gps, EV là nguồn vị trí duy nhất: EKF2 = LIO, covariance chỉ đổi cổng innovation và tốc độ bám, không đổi drift.
- **Kênh trung thực rẻ nhất:** không có. Nghịch đảo ma trận thông tin (cận Cramér-Rao) cho cỡ mm, vẫn dưới sàn 0.1 m.
  Sai số thật là lỗi mô hình (bias, khớp sai, deskew), và LIO không quan sát được nó. Thêm phép "bơm covariance" sẽ là một
  hệ số đoán rồi chỉnh (D44.7).
- **Đề xuất:**
  - giữ cổng nhị phân TRACKING;
  - ghi rõ trong spec rằng độ chính xác EV mà PX4 giả định là `EKF2_EVP_NOISE`/`EKF2_EVV_NOISE`, cố định theo profile;
  - ghi phương sai EV thật một lần trong SITL S1c để xác nhận kênh trơ;
  - số drift của benchmark D39 là căn cứ cho giá trị cố định đó ở phần cứng.

### 1b. Không còn hình học: nguồn thông tin còn lại theo profile

| Profile | Còn lại | PX4 | Không thể làm (về mặt thông tin) | Hệ phải làm |
|---|---|---|---|---|
| gps | GNSS (sim: Gauss-Markov 0.8 m), baro, compass, IMU | Giữ vị trí cỡ GNSS, heading theo bắc | Giữ frame LIO liên tục; liên tục sau restart tốt hơn độ chính xác GNSS | Chạy hết timeline qua `T` đóng băng rồi giữ (D11); bỏ cuộc ở 10 s → AUTO_LOITER; LIO chờ và tự restart, `T` INIT lại từ cặp |
| no_gps | IMU, baro, compass | Mất vị trí ngang sau `EKF2_NOAID_TOUT` 5 s → failsafe PX4 | Giữ vị trí quá vài giây; seed có nghĩa (PX4 chỉ có IMU); `T` có nghĩa sau khi PX4 mất vị trí | Bàn giao cho failsafe PX4 (D4: Descend); LIO vẫn chờ nhưng xe đã thuộc PX4. Phạm vi bay no_gps phải nằm trong vùng có hình học (D3) |

## 2. Kiểm lan lỗi xuyên tầng

```text
LIO health ──(không TRACKING)──► EV dừng ──► EKF2: hết EV sau 0.2 s ──(no_gps: 5 s)──► PX4 mất vị trí ──► failsafe PX4
     │                                                                                         ▲
     ├──► T: VALID → FROZEN ──(10 s, frozen_max)──► INVALID ──► px4_mode: ALIGNMENT_INVALID ──┤  (đua nhau)
     │                                                                                         │
     └──► Supervisor: LOST(since) ──(10 s, lio_recovery_timeout_s)──► LIO_RECOVERY_TIMEOUT ────┘
LIO epoch mới ──► EV epoch mới ──(no_gps)──► EKF2 reset vị trí theo EV (F33c) ──► T: reset PX4 khi FROZEN → INVALID
```

Điểm lan lỗi đã thấy:

- **C1. Hai bộ đếm 10 s cho cùng một lỗi.** `frozen_max` của `T` và `lio_recovery_timeout_s` của Supervisor cùng hết hạn,
  nên `Reason` bàn giao phụ thuộc thứ tự tới (`ALIGNMENT_INVALID` hay `LIO_RECOVERY_TIMEOUT`). Đề xuất: một chủ (Supervisor);
  bỏ cạnh FROZEN → INVALID theo thời gian (O35).
- **C2. Epoch mới qua cửa FROZEN → VALID.** Như mục 1: `T` báo VALID cho một frame khác tới 1 s, rồi STALE, rồi INIT.
  Ba lần đổi trạng thái cho một nguyên nhân. Đề xuất: epoch mới → INIT một lần.
- **C3. Seed `SYNC`.** Thêm một vòng `px4_bridge` → `lio`, biến lỗi của PX4 (reset, mất vị trí) thành lỗi của LIO. Bỏ.
- **C4. Backend chậm.** Theo quy tắc tuổi O15, tải CPU thành LOST, epoch mới, reset map navigation và `T` INIT lại:
  một lỗi tài nguyên lan ra bốn tầng. D40 (áp mọi hiệu chỉnh predictor đặt được) giữ LIO ở DEGRADED với output vẫn được
  hiệu chỉnh. **Giữ** hướng D40.
- **C5. (S2) `T` INIT sau restart.** Spec §4.2 không nói `px4_mode` làm gì khi `T` là INIT. Lệnh epoch cũ có thể dẫn tới
  bàn giao. Đường `SELF` của v1 cũng có vấn đề này. Ghi vào carry-over S2 kèm đề xuất: `px4_mode` biến đổi lệnh bằng `T`
  cùng epoch với lệnh, và giữ giá trị đóng băng cho lệnh epoch cũ.

**Bảng ngân sách, một chủ cho mỗi quyết định** (đưa vào §3.4 của spec):

| Quyết định | Chủ duy nhất | Giới hạn | Giá trị beta |
|---|---|---|---|
| TRACKING → DEGRADED | `LioLifecycle` | gap; 3 scan xấu | 0.25 s; 3 |
| DEGRADED → LOST | `LioLifecycle` | gap; suy biến kéo dài; σ | 0.5 s; 1.0 s; 0.5 m |
| LOST → RESTARTING | `LioLifecycle` | scan tới đều, không rỗng, liên tục | `restart_dwell_s` 2.0 s |
| INITIALIZING → LOST | `LioLifecycle` | map sẵn mà chưa xác nhận | `restart_dwell_s` (cùng khoá) |
| EV bật/tắt | `px4_bridge` | theo health, không có đồng hồ riêng | — |
| EKF2 ngừng fuse EV; PX4 mất vị trí (no_gps) | PX4 | `EV_MAX_INTERVAL` 0.2 s; `EKF2_NOAID_TOUT` 5 s | PX4 |
| `T` VALID → INVALID | `AlignmentEstimator` | không có cặp được nhận khi đang TRACKING | `valid_stale` 1.0 s |
| `T` → INIT | `AlignmentEstimator` | cặp của epoch LIO mới hơn (sự kiện, không đồng hồ) | — |
| `T` FROZEN → INVALID | `AlignmentEstimator` | chỉ reset PX4 (bỏ đồng hồ, O35) | — |
| Bỏ cuộc, bàn giao | Supervisor (S2) | từ LOST đầu tiên của episode | 10 s (gps); no_gps theo PX4 |

## 3. Kiểm độ phức tạp: v1 so với v2

| Hạng mục | v1 thêm | v2 thêm | Lý do cắt hoặc gộp |
|---|---|---|---|
| Trạng thái | 0 | 0 | |
| Cạnh `LioLifecycle` | +3 (INITIALIZING → LOST, 3 reason), 1 đổi tên | +1 (`kReinitNotConfirmed`), 1 đổi tên | gap/suy biến trong lần thử đã nằm trong "chưa xác nhận sau dwell" |
| Cạnh `T` | +1 (FROZEN → INIT sau cửa sổ), quy tắc reset trong cửa sổ | +2 (VALID/FROZEN → INIT `kNewLioEpoch`), −1 (`kFrozenTooLong`) | epoch là thông tin thật, cửa sổ là đoán |
| Loại input lifecycle | +3 (`kScanAppliedNotFresh`, self-info đủ/thiếu) | 0 | guard dùng mốc IMU lifecycle đã thấy; "không rỗng" là loại sẵn có |
| Enum kết luận | 6 giá trị + 4 struct + API `innovation_at` | 0: kết quả có kiểu sẵn có `on_correction` → `Result<void, PredictorReason>` là kết luận duy nhất | bỏ tuổi thì còn đúng một chỗ quyết định |
| Dữ liệu trạng thái | `stable_since_`, `reinit_since_`, dữ liệu cửa sổ (2) | `stable_since_`, `confirming_since_`, `last_imu_t_`, epoch của `T` | không cờ bool; mốc thời gian là dữ liệu |
| Tham số tầng (b) | +4 (innovation ×2, `restart_stable_s`, `alignment_resync_window_s`) | +1 (`lifecycle_restart_dwell_s`), −1 (`alignment_frozen_max_s`) | không còn ngưỡng đoán |
| Hằng tầng (a) | `kClockMode`, `kSelfInfoNeighbours`, `kMaxStampAhead` | `kMaxStampAhead` 0.5 → 0.05 s, có `static_assert` theo cận dưới mới 0.2 s của `gap_degraded` | `kClockMode` của lio thuộc phần cứng (P8) |
| Message | +1 (`LioRestartSeed`, 8 trường) + QoS + pub/sub | 0 | không có offer |
| Toán backend mới | `self_information` (k-NN, khớp mặt phẳng) | 0 | |
| Event mới | `CorrectionVerdict`, `LioStabilityReset`, từ chối offer, … | 0 tên mới; reason mới trên `StateTransition`/`AlignmentTransition` | `LioScan` đã log scan rỗng |
| Bỏ hạn chờ 100 ms của scan | có | không | ở 10 Hz scan kế tiếp tới sau ~100 ms: "bị thay" và "quá 100 ms" trùng nhau; chỉ ghi hậu quả vào spec |
| Task | 14 | 7 | gộp đo và báo cáo vào task tạo ra dữ liệu |
| Cổng owner | AR3, O29, O33 | AR4, O34, O35 | O34 thay O29 + O33 |
| Lần chạy SITL | 22–24 (~190 phút) | 12 (~95 phút) | xem mục 4 và plan |

**Quyết định viết theo nghĩa đen mà làm tăng độ phức tạp:**

- **D38 "seed `T⁻¹ × pose PX4`".** Đây là nguồn của O29, message mới và hai đường seed. Phương án nhỏ nhất: LIO tự seed,
  `T` INIT theo epoch (O34). Đổi chữ của D38, giữ mục tiêu "khôi phục rồi đồng bộ lại".
- **D40 "gom vào một hàm kết luận có kiểu".**
  - Khi bỏ quy tắc tuổi, chỉ còn hai từ chối kỹ thuật: (1) predictor không đặt được stamp vào buffer (`covers()`, đã trả `Result` có kiểu); (2) sai epoch, áp cho mọi loại kết quả.
  - Một hàm sáu kết luận là thừa. Đề xuất: coi `on_correction` là hàm kết luận (AR4).
- **D41 "ổn định đủ lâu".** Hiểu theo thông tin có thật: scan tới đều và không rỗng. Không suy ra "sẽ thành công".

## 4. Lỗi thực tế có khả năng nhất

| # | Lỗi | Thiết kế hiện làm gì | Nên làm gì | Sim kiểm được? |
|---|---|---|---|---|
| 1 | Bias IMU, rung | IKFoM ước lượng bias; khoảng chỉ IMU trôi theo t² | Ngân sách LOST ngắn; seed vận tốc từ predictor sẽ sai trên phần cứng (ghi rủi ro) | Không (sim không có bias): phần cứng |
| 2 | Thời gian từng điểm, méo chuyển động | Deskew FAST-LIO có, chưa chạy (scan sim một thời điểm, O20) | Benchmark nêu rõ là cận lạc quan | Không: phần cứng |
| 3 | Khớp sai ở cấu trúc lặp (hành lang, mặt tiền) | Không phát hiện được (trị riêng vẫn lớn) | gps: EKF2 (cổng EV, bias) phân xử; no_gps: giới hạn vận hành | Một phần (world hành lang có sẵn), ngoài S1c |
| 4 | Vật động | Không xử lý riêng; outlier ICP | Không làm ở beta | Không: phần cứng |
| 5 | Reset EKF2 (GNSS glitch, EV bắt đầu, heading reset) | `T` áp delta tất định khi VALID; FROZEN → INVALID | Như cũ, cộng epoch → INIT | **Có, rẻ**: tiêm lỗi GNSS và cắt LiDAR trong S1c |
| 6 | Mất scan qua Ethernet, backend chậm | Quy tắc gap; D40 giữ DEGRADED khi chậm | Như v2 | **Có, rẻ**: dừng bridge LiDAR (`SIGSTOP`); bão hoà backend bằng unit test |
| 7 | Bước nhảy đồng hồ, stamp đi trước | Ngưỡng 0.5 s ≥ `gap_lost` (review I1) | 0.05 s có `static_assert`; `kClockMode` của lio ở phần cứng | Chỉ unit test (sim một đồng hồ) |
| 8 | Compass nhiễu trong nhà | no_gps: EV yaw được fuse thì compass không fuse (`mag_control.cpp`); gps ở ngoài trời | Không thêm gì; `T` hấp thụ reset heading | Có thể tiêm lỗi mag, ngoài S1c |

## 5. Điểm nghẽn thật, thêm tính năng không giải được

1. **no_gps mất hình học:** chỉ còn IMU, PX4 mất vị trí sau 5 s. Không cơ chế nào giữ được vị trí; phải bàn giao (D4).
2. **Drift chậm của LIO khi TRACKING ở no_gps:** không có tham chiếu độc lập, EKF2 đi theo LIO. Chỉ giới hạn vận hành (D3)
   và số đo phần cứng trả lời được.
3. **Khớp sai mà tự tin:** thông tin của chính scan không phân biệt được. Chỉ một sensor độc lập (GNSS) mới phân biệt.
4. **Liên tục frame qua một lần restart:** bị chặn bởi tham chiếu lúc restart (gps: nhiễu GNSS; no_gps: drift IMU trong lúc
   mất). Không cách seed nào vượt được, kể cả `SYNC`.
5. **Biên hình học:** đại lượng dao động quanh ngưỡng; quy tắc chỉ giãn được các lần thử. Xe phải đổi vị trí, và đó là
   quyết định của navigation, không phải của LIO.
6. **Độ trung thực của sim:** không bias IMU, không thời gian từng điểm, không nhiễu góc. Mọi số drift và khôi phục của S1c
   là cận lạc quan; thêm SITL không tạo ra sự thật của phần cứng.

## 6. Đã cắt và lý do

| Cắt | Lý do (D44) |
|---|---|
| Offer seed, `LioRestartSeed`, `choose_seed`, `SeedSource`, O29 | gauge, không phải thông tin; vòng phản hồi (3, 4) |
| Cửa sổ đồng bộ lại, `alignment_resync_window_s`, reset trong cửa sổ, O33 | thay bằng `T` theo epoch (3, 7) |
| `self_information`, `kSelfInfoNeighbours`, hai loại input | proxy không đo được; lần thử là phép kiểm thật (3, 6) |
| Cổng innovation và hai tham số | ngưỡng đoán, tự chỉnh, trùng quyền PX4 (3, 7) |
| `CorrectionVerdict` sáu giá trị, `kScanAppliedNotFresh` | một guard trên cạnh sẵn có là đủ (5, 6) |
| `kClockMode` lio, `StartError`, bỏ hạn chờ scan | phần cứng; thay đổi không đổi hành vi ở 10 Hz (6) |
| Chạy SITL: hai hover yaw 0, cắt 12 s, lặp lại drift, tốc độ 2 và 4 m/s | không thêm thông tin cho quyết định (6) |

Giữ: cấu hình EV đầy đủ và lần đo sớm (D42, 3 lần chạy: gps yaw 1.2, no_gps yaw 1.2, gps mất GNSS 10 s); benchmark drift
(D39, 2 profile × 1/3/5 m/s, đối chiếu datasheet); fault injection cắt LiDAR và leo mất hình học; D40 bỏ quy tắc tuổi; D41 LIO không bỏ cuộc.
