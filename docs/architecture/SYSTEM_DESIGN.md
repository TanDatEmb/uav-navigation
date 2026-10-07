# Thiết kế hệ thống UAV Navigation (rebuild v2)

Trạng thái: **đã được owner duyệt 2026-10-06** (D27), branch `rebuild/v2`.
Đây là bản đồ duy nhất cho việc xây lại hệ thống.

- Lý do và lịch sử của từng quyết định nằm trong [DECISIONS.md](DECISIONS.md). Mỗi mục dưới đây ghi rõ mã quyết định D\*.
- Tiến độ được theo dõi trong [TRACEABILITY.md](../TRACEABILITY.md).
- Quy trình làm việc (dừng lại khi lệch thiết kế, các quy tắc cấu trúc) nằm trong [AGENTS.md](../../AGENTS.md).
- Branch `main` chỉ dùng làm nguồn tham khảo (D13, D15).

## §0. Mục tiêu, phạm vi, profile

### Mục tiêu

Bay mission nhiều waypoint trong môi trường có vật cản. Các thành phần chính:

- định vị bằng LiDAR-inertial (FAST-LIO, Mid-360);
- world model do sản phẩm tự sở hữu;
- quỹ đạo MAIN đi kèm một BACKUP có chứng nhận;
- PX4 External Mode nhận lệnh.

Mục tiêu dài hạn là 10 m/s. Bản **beta** chạy SITL trên PX4 **v1.17** (D5, P8), ở tốc độ 1–5 m/s. Phần cứng nằm ngoài phạm vi beta.

### Profile

Profile là tham số tầng (c) theo D22. Profile được chọn khi chạy; không có trạng thái riêng cho profile.

| Trục | Giá trị | Ảnh hưởng | Mã quyết định |
|---|---|---|---|
| Bay | **mặc định** | MAIN được đi vào UNKNOWN; BACKUP bắt buộc known-free | D1, D2 |
| Bay | **khéo léo** | Cả MAIN và BACKUP được đi vào UNKNOWN. Đây là chế độ **chấp nhận rủi ro do owner phê duyệt**: phải chọn tường minh, ghi vào metadata của mỗi lần chạy, không bao giờ là mặc định | D1, D2 |
| Định vị | **có GPS** | EKF2 luôn fuse GNSS. EV gửi FRD, tắt EV yaw (`EKF2_EV_CTRL=7`). Frame mission là toạ độ GPS | D3, D19, D20 |
| Định vị | **không GPS** | Chỉ dùng ở môi trường LIO duy trì được. EV gửi FRD, bật EV yaw (`EKF2_EV_CTRL=15`). Frame mission là toạ độ local của PX4 | D3, D19, D20 |
| Yaw | **Y1 khoá heading** (mặc định) / **Y3a theo hướng đoạn bay** | Xem §2.5 | D24 |

### Hành vi an toàn không được nới

- **Fail-closed.** Thiếu dữ liệu, hoặc dữ liệu cũ hay mơ hồ về thời gian, frame, identity, epoch, world hay certificate, thì bị từ chối. Không bao giờ coi là đạt.
- **BACKUP known-free** ở profile mặc định.
- **Không bàn giao im lặng.** Mọi lần bàn giao cho PX4 và mọi lần huỷ đều có `Reason` trong event log (D11).
- **Không có bypass ẩn.** Một lối tắt tạm thời phải: nằm trong config, được log, và được liệt kê trong DECISIONS.

## §1. Thành phần và process (D17)

```text
 LiDAR 10Hz + IMU 200Hz
        │
 ┌──────▼───────────┐  /lio/odometry 10Hz (đã correction, stamp = thời điểm scan, epoch)  ┌───────────────────┐
 │ lio              ├────────────────────────────────────────────────────────────────────► px4_bridge        │──EV FRD──► PX4 EKF2
 │ (§3)             │  /lio/state 100Hz (output predictor), /lio/health, /lio/scan        │ (§4.1)            │◄─odometry + reset PX4
 └──────┬───────────┘                                                                     └─────────┬─────────┘
        │                                                                                           │ /alignment (T, trạng thái, tuổi)
 ┌──────▼─────────────────────────────────────────────────────────┐                                 │
 │ navigation (1 process, 3 thread)                                │                                 │
 │  Mapping actor (§5.2) ──WorldView bất biến──► Planner worker (§5.3)                               │
 │        └──revalidate──► Supervisor (§2) ◄──Result<Bundle,Reason>─┘                               │
 │                          sampler 50Hz → /nav/command (P/V/A + yaw, frame LIO)                     │
 └──────┬─────────────────────────────────────────────────────────┘                                 │
 ┌──────▼──────────────────────────┐◄─────────────────────────────────────────────────────────────────┘
 │ px4_mode (§4.2)                  │  P/V/A biến đổi qua T ──► TrajectorySetpoint ──► PX4
 └─────────────────────────────────┘
 Mọi process ──► /events + JSONL (§6.1)
```

Mapping, Planner và Supervisor nằm chung một process. Chúng chia sẻ world và bundle bất biến qua bộ nhớ chung, nên không phải serialize map. Ranh giới giữa ba khối là ranh giới thread và kiểu dữ liệu.

## §2. Supervisor: một nơi ghi duy nhất cho điều khiển bay (D18, D24)

### §2.1 Timeline đã commit

- Mỗi `Bundle` (§5.1) là **một quỹ đạo liên tục**: MAIN prefix dài ≥ 1.0 s, nối sang BACKUP suffix, kết thúc ở trạng thái đứng yên.
- Supervisor chỉ làm một việc với timeline: quyết định **có thay timeline hay không**. Việc thay là commit nguyên tử, nối PVAJ liên tục tại một điểm tương lai trên timeline đang chạy.
- Sampler lấy mẫu timeline theo thời gian. Hết timeline thì giữ điểm cuối; đó chính là "hold".
- `phase(bundle, t) ∈ {MAIN, BACKUP, BRAKE, HOLD}` là hàm **dẫn xuất**, chỉ dùng cho log và KPI. Không có code nào ghi phase.

### §2.2 Trạng thái

```text
Session:          IDLE ──MissionLoaded ∧ Px4Active──► RUNNING ──waypoint cuối đạt (đo)──► COMPLETED
                                                         └──── guard bỏ cuộc (Reason) ────► HANDED_OVER
NavAvailability:  AVAILABLE ──LIO LOST──► LOST(since) ──LIO TRACKING, epoch mới──► REACQUIRING(since)
    (khi RUNNING)     ▲                                                              │ ResetMap; T ổn định 3 s
                      └──────────────────────────────────────────────────────────────┘
```

- Hàm chuyển trạng thái `transition(state, event) → (state, effects)` là hàm thuần.
- Mỗi lần chuyển trạng thái phát một `EventRecord` (§6.1).
- **Quy tắc chống phình.** Mọi yêu cầu mới phải được diễn đạt bằng một trong các cách sau:
  - event, guard hoặc effect mới;
  - thuộc tính của bundle;
  - phase dẫn xuất;
  - chính sách của một thành phần.

  Thêm trạng thái mới bắt buộc phải review thiết kế.

### §2.3 Quy tắc trong RUNNING

| Event | Guard | Effect |
|---|---|---|
| `Tick(t)` | AVAILABLE, world sẵn sàng, worker còn chỗ | `SubmitPlan`; anchor = timeline(t + 200 ms), hoặc trạng thái đứng yên đo được |
| `PlanResult(Ok)` | Cùng epoch và mission; world của bundle ≥ world của request; certificate đạt; nối liền với timeline | `Commit(bundle)` |
| `PlanResult(Err)` | | Giữ timeline cũ. `Emit(reason)` |
| `WorldUpdated` | `revalidate` phần sẽ thực thi trên vùng thay đổi: vẫn sạch | Không làm gì |
| `WorldUpdated` | Va chạm trong quãng cần để dừng, vị trí hiện tại KNOWN_FREE, chưa phanh trong episode này | Bộ tổng hợp phanh (§5.4), rồi `Commit(BRAKE)` |
| `WorldUpdated` | Va chạm, phanh không chứng nhận được | → HANDED_OVER(`EMERGENCY_UNCERTIFIED`) |
| `LioHealth(LOST)` | | → LOST(t). Ngừng submit, bỏ kết quả còn treo. Timeline tự chạy hết qua `T` FROZEN |
| `LioHealth(TRACKING, epoch mới)` | Đang LOST | → REACQUIRING, effect `ResetMap` |
| `Alignment(VALID, ổn định 3 s)` | Đang REACQUIRING | → AVAILABLE, plan lại từ trạng thái đứng yên |
| `WaypointReached(đo)` | | Mission tracker tiến tới waypoint kế. Waypoint cuối thì → COMPLETED |
| `Px4Mode(Inactive)` | | → HANDED_OVER(`OPERATOR` hoặc `PX4_FAILSAFE`) |

**Guard bỏ cuộc**, đánh giá ở mỗi `Tick`, mỗi guard dẫn tới HANDED_OVER với một `Reason` riêng:

| Điều kiện | Reason |
|---|---|
| LOST lâu hơn `lio_recovery_timeout_s` (profile GPS, mặc định 10 s) | `LIO_RECOVERY_TIMEOUT` |
| PX4 báo mất vị trí (profile không GPS) | `PX4_POSITION_LOST` |
| PX4 reset trong lúc `T` đang FROZEN | `ALIGNMENT_INVALIDATED` |
| Đứng yên mà không commit được bundle nào trong `no_path_timeout_s` (mặc định 15 s) | `NO_PATH_TIMEOUT` |

### §2.4 Chống chuyển nhánh liên tục (P10)

| Quy tắc | Thành phần sở hữu |
|---|---|
| R1. MAIN prefix ≥ 1.0 s | Planner, là bất biến của bundle |
| R2. Worker giữ 1 request đang chạy và 1 request chờ (bản mới nhất). Chỉ huỷ khi request cũ hết hợp lệ (đổi epoch, goal hoặc mission) | Worker queue |
| R3. World mới chỉ dẫn tới `revalidate` trên vùng thay đổi. World cũ đơn thuần không thu hồi command khi lease còn hạn | Mapping và Supervisor |
| R4. Tracking error chỉ dẫn tới re-anchor; không bao giờ tự kích hoạt BACKUP hay phanh | Planner |
| R5. Nối PVAJ tại điểm tương lai, dù điểm đó đang ở MAIN hay BACKUP | Planner và Supervisor |
| R6. KPI tính từ event log | Công cụ (§6.1) |

### §2.5 Yaw (D24)

`YawPolicy` là hàm thuần, chạy khi lấy mẫu lệnh. Yaw **không** nằm trong bundle. Certifier kiểm flatness với yaw rate xấu nhất ±ω_max, nên bundle hợp lệ với mọi profile yaw trong giới hạn đó.

- **Y1 (mặc định):** yaw là hằng số trong cả mission. Lấy yaw lúc bắt đầu, hoặc yaw do mission chỉ định.
- **Y3a (option của mission):**
  - Ở mỗi đoạn, yaw đích là `atan2(wp[i+1] − wp[i])`.
  - Xoay trong lúc bay với tốc độ ω (mặc định 45°/s), có giới hạn gia tốc xoay 90°/s².
  - Khi phase là BACKUP hoặc BRAKE: giữ yaw hiện tại và giảm ω về 0.
- ω_max = 90°/s là hằng số trong code. ω vận hành nằm trong YAML, giới hạn trong (0, ω_max].
- **Y3b** (bám hướng vận tốc, perception-aware) để sau beta, chỉ khi có sensor FOV hẹp. Kiến trúc đã có sẵn điểm mở rộng:
  - `yaw(t)` trong Bundle;
  - `yaw_policy` trong request;
  - bước kiểm tầm nhìn trong certifier.

### §2.6 Mission

`MissionDefinition` gồm frame (GPS hoặc local PX4, D19) và danh sách waypoint với behavior `STOP` hoặc `PASS_THROUGH`.

- Mission tracker là thành phần thuần do Supervisor sở hữu, và chỉ Supervisor được gọi nó.
- Tiến độ được tính từ trạng thái **đo được**, theo thứ tự waypoint. Điểm cuối của một quỹ đạo không phải bằng chứng đã tới waypoint.
- Waypoint được đổi từ frame mission sang frame LIO qua `T` hiện tại khi tạo request.

## §3. `lio` (D20)

### Giữ và viết lại

- **Giữ:** IKFoM ESKF, ikd-tree, residual point-to-plane, toán propagation.
- **Viết lại:** lifecycle, health, ingest, output.

### §3.1 State machine

```text
INITIALIZING ──map đủ ∧ 5 scan tốt liên tiếp──► TRACKING ◄──5 scan tốt──┐
     ▲                                              │ suy biến 3 scan, hoặc gap > 0.25 s
     │                                              ▼                    │
     │                                           DEGRADED ───────────────┘
     │ epoch mới, map rỗng, seed                    │ suy biến kéo dài > 1.0 s, gap > 0.5 s,
     │                                              │ hoặc σ vị trí > 0.5 m
 RESTARTING ◄── scan đủ hình học trở lại ──────── LOST
```

- **LiDAR gap** tính theo **sensor time trên mỗi mẫu IMU** (stamp IMU − điểm cuối scan gần nhất).
- Mọi lối vào TRACKING đều qua bước xác nhận.
- Scan rỗng là một event, không phải exception.
- `/lio/scan` chỉ được phát khi TRACKING.
- Các con số trong sơ đồ là giá trị beta, thuộc tầng (b), sẽ chỉnh từ log.

### §3.2 Phát hiện suy biến

- Mỗi scan, tính ma trận 6×6 `HᵀR⁻¹H` và tách thành khối **tịnh tiến** và khối **xoay**. Trị riêng nhỏ nhất của mỗi khối so với ngưỡng (tầng b), có hysteresis.
- Giá trị khởi đầu của ngưỡng là một việc của lát S1: lấy từ log trị riêng trên scene mở và scene suy biến (bay cao) trong SITL.
- Mọi scan đều ghi log các trị riêng này.

### §3.3 Output predictor `/lio/state` 100 Hz

Làm theo đủ cơ chế của EKF2 (F35):

1. Hai mốc thời gian: ESKF ở thời điểm scan, predictor ở thời điểm IMU mới nhất.
2. Ring buffer trạng thái output cùng trục thời gian với IMU, độ trễ tối đa 300 ms. Sai số được tính **tại timestamp của scan**: tra buffer theo thời gian, không lấy mẫu cũ nhất.
3. Attitude hiệu chỉnh qua delta-angle với gain `0.5·dt/delay`.
4. Vel/pos hiệu chỉnh bằng PI (`dt/τ`, tích phân `0.1·gain²`), áp lên **toàn bộ buffer**. τ_vel = τ_pos = 0.25 s (tầng b).
5. Kênh dọc riêng.
6. dt được lấy trung bình và kẹp như PX4: khoảng giữa hai lần hiệu chỉnh kẹp tối đa 0.03 s.

**Nghĩa của τ (D30).** τ ở đây mang cùng nghĩa với `EKF2_TAU_VEL`/`EKF2_TAU_POS` của PX4, không phải hằng thời gian thực tế:
- PX4 hiệu chỉnh ở khoảng 100 Hz nên phép kẹp 0.03 s không có tác dụng.
- Predictor chỉ được hiệu chỉnh theo nhịp scan (10 Hz), nên hằng thời gian hiệu dụng ≈ τ·T_scan/0.03 (≈ 0.83 s với τ = 0.25 s).
- Phép kẹp được giữ có chủ đích: bỏ kẹp thì mỗi scan kéo khoảng 40% sai số, gây nhảy bậc đúng lỗi F23.
- τ được đánh giá lại bằng output tracking error trong log SITL.

**Khác biệt thứ ba có chủ đích so với PX4 (D30):** hiệu chỉnh attitude được giữ trong một **cửa sổ có giới hạn** rồi về 0. PX4 làm mới hiệu chỉnh này ở mỗi bước EKF. Predictor chỉ có hiệu chỉnh theo nhịp scan, nên nếu giữ vô hạn thì attitude sẽ trôi liên tục khi scan ngừng.
7. Reset tường minh: phát `reset_counter` kèm delta pos, vel, yaw.
8. Xuất output tracking error vào `/lio/health`.

### §3.4 Khởi động lại

- Seed = `T⁻¹ × pose PX4`, với `T` đang FROZEN.
- Bắt đầu epoch mới; map được reset ở `navigation`.
- **Luôn** tăng `reset_counter` theo epoch của chính mẫu dữ liệu.
- Phát event `LioRestart` gồm: epoch cũ, epoch mới, seed, `T`.

### §3.5 Thread (D30)

Phần thuần của LIO tách thành hai khối, mỗi khối do đúng một thread sở hữu:

| Khối | Thread | Sở hữu | Không được làm |
|---|---|---|---|
| **Frontend** | ingest | Nhận IMU và LiDAR; `OutputPredictor`; `LioLifecycle` (**nơi ghi duy nhất** của trạng thái LIO); kiểm gap trên từng mẫu IMU; phát `/lio/state`, `/lio/health`, `/lio/odometry` | Chạy ICP hay cập nhật map |
| **Backend** | estimator | `IkfomEstimator`, map (ikd-tree), deskew, ICP, đánh giá suy biến | Ghi trạng thái lifecycle; phát message |

Hai khối trao đổi qua hàng đợi có giới hạn, chỉ chứa dữ liệu bất biến:
- **Frontend → backend:** bản sao mẫu IMU, scan đã nhận, lệnh `restart(seed)`.
- **Backend → frontend:** kết quả scan, gồm `EstimatorSnapshot`, báo cáo suy biến, covariance, và loại kết quả:
  - loại đưa vào lifecycle: `kScanGood`, `kScanDegenerate`, `kScanEmpty`, `kMapReady`, `kRestartSeeded`;
  - loại không đưa vào lifecycle (D31): `kImuInitialized` (căn predictor), `kScanNotProcessed` (đối soát), `kRestartRejected`.

**Chỉ một scan được predictor áp dụng thì mới tính là scan có correction (D31).** Nếu predictor từ chối hiệu chỉnh của một kết quả `kScanGood`, ví dụ vì cũ hơn buffer 300 ms (`kOlderThanBuffer`), thì frontend:
- đưa scan đó vào lifecycle như `kScanDegenerate` với reason riêng;
- **không** đẩy mốc "scan cuối" lên.

Nhờ vậy backend chậm kéo dài sẽ dẫn tới DEGRADED/LOST. Nếu không có quy tắc này, `/lio/state` có thể trôi trong khi health vẫn báo TRACKING.

**Phần thực thi có thread là mã sản phẩm (D31).** Việc ghép frontend, backend và hai hàng đợi trên hai thread nằm trong một lớp của `uavnav_lio_core`, lớp này có test và có chạy TSan. Node ROS ở S1b dùng lại đúng lớp đó. Driver đồng bộ một thread chỉ dùng cho test và replay offline; node sản phẩm không được dùng nó.

Frontend đưa từng kết quả scan vào lifecycle và predictor. Vì vậy thời gian ICP chạy (20–50 ms) không làm `/lio/state` dồn cục và không làm chậm kiểm gap.

Hàng đợi tràn là một event có reason, và không bao giờ chặn thread ingest.

Thread khác:
- **events writer.**
- **Worker OpenMP** bên trong ICP: số thread là hằng số tầng (a) (mặc định 3), và chỉ backend gọi tới chúng. Runtime libgomp giữ pool thread giữa các lời gọi. Bản build TSan đặt số thread về 1, vì libgomp không được TSan instrument (D31).

## §4. PX4 (D21, D7, D20)

### §4.1 `px4_bridge`

**EV gửi EKF2**

- Nguồn là `/lio/odometry` 10 Hz, **chỉ gửi khi LIO ở TRACKING**.
- Nhãn frame `LOCAL_FRAME_FRD`.
- `reset_counter` lấy từ epoch của chính mẫu.
- `timestamp_sample` là thời điểm scan, đổi sang đồng hồ PX4 qua timesync.
- Covariance và quality lấy từ LIO.

**Alignment `T_px4←lio`, 4-DoF (x, y, z, yaw)**

- Mỗi cặp mẫu cùng thời điểm (pose LIO tại thời điểm scan, pose PX4 nội suy tại đúng thời điểm đó) cho một `T` tức thời.
- **Mọi phép đo của `T` đều tính tại vị trí xe**, không tại gốc LIO (D30). Ở xa gốc, một sai số yaw nhỏ nhân với cánh tay đòn sẽ thành sai số tịnh tiến lớn: ở 300 m, 1.7 mrad đã thành 0.5 m. Thứ cần giữ đúng là setpoint tại xe. Cụ thể:
  - residual vị trí = `‖p_px4 − T(p_lio)‖` tại cặp mẫu; residual yaw tính riêng;
  - cổng nhảy: `T` tức thời có residual tại xe quá 0.5 m hoặc 5° (tầng b) thì bị bỏ qua và phát event;
  - bước lọc với `τ_T` = 2 s (tầng b): phần yaw là **phép xoay quanh vị trí xe hiện tại**, phần tịnh tiến là độ lệch vị trí tại xe;
  - bước tích luỹ INIT (20 cặp nhất quán) cũng đo tại vị trí xe. Nếu không, khởi tạo lại ở xa gốc sẽ không bao giờ đạt VALID (D31).
- **Không có bộ giới hạn tốc độ riêng.** Cổng nhảy và `τ_T` đã giới hạn tốc độ biến thiên của `T` tại xe ở jump/τ_T, tức 0.25 m/s và 2.5°/s với giá trị beta.
- Log cả `T` thô và `T` đã lọc.
- **PX4 reset (D30):**
  - Reset `xy` hoặc `z`: cộng tất định delta tương ứng từ `vehicle_local_position` vào `T`.
  - Reset `heading`: EKF2 chỉ xoay quaternion, vị trí local vẫn liên tục. Vì vậy áp **phép đổi hệ quy chiếu G**, tức xoay `T` quanh vị trí xe theo PX4: `yaw_T += dh`, `t ← Rz(dh)·(t − p_xe) + p_xe + d`. Áp cùng G lên các mẫu PX4 đang buffer.
  - Không giả định mỗi lần counter chỉ tăng 1.
- Trạng thái `T`:

```text
INIT ──20 cặp nhất quán liên tiếp──► VALID ──LIO LOST──► FROZEN ──LIO TRACKING──► VALID
                               │                  └──PX4 reset hoặc FROZEN quá lio_recovery_timeout_s──► INVALID
                               ├──PX4 reset: cộng delta (vẫn VALID)
                               └──VALID không có cặp mới > 1.0 s──► INVALID
```

- Xuất `/alignment` gồm: `T`, trạng thái, tuổi, residual.

### §4.2 `px4_mode`

```text
UNREGISTERED ─► IDLE ──PX4 kích hoạt──► ACTIVE ──bàn giao(Reason)──► RELEASING ──► RELEASED
                  ▲                       │
                  └── PX4 tự huỷ (Reason: OPERATOR | PX4_FAILSAFE) ┘
```

**Mỗi tick setpoint**

1. Lấy `/nav/command` mới nhất.
2. Nếu tuổi vượt lease 100 ms thì bàn giao với reason `COMMAND_STREAM_LOST`.
3. Biến đổi P/V/A và yaw qua `T`, chấp nhận khi `T` là VALID hoặc FROZEN.
4. Nếu `T` là INVALID thì bàn giao với reason `ALIGNMENT_INVALID`.
5. Gửi `TrajectorySetpoint`.

**Các quy định khác**

- `px4_mode` không tự quyết định hold; hold là lệnh do Supervisor gửi.
- Trạng thái "đang bay" lấy từ `vehicle_land_detected`.
- **Cách bàn giao:**
  - PX4 còn vị trí: chuyển sang AUTO_LOITER qua mode executor, tối đa 3 lần thử.
  - PX4 không còn vị trí: kết thúc mode với lỗi để failsafe của PX4 tự chọn mode (ví dụ Descend).
- Input đi qua mailbox kiểu latest-value. Không publish khi đang giữ khoá.

## §5. World, Planner, Bundle (D23, D24)

### §5.1 Bundle (bất biến)

```text
Bundle {
  identity:    mission_id, goal_seq, lio_epoch, request_id, world_revision, profile
  time:        start_ns, backup_start_ns, end_ns             // int64 ns
  segments[]:  { role: MAIN | BACKUP | BRAKE, đa thức vị trí P(t) 3D }
  certificate: { checks_passed (bitset đầy đủ), certifier_version, unknown_policy_main, unknown_policy_backup }
}
Bất biến: MAIN prefix ≥ 1.0 s (trừ BRAKE); kết thúc ở trạng thái đứng yên; PVAJ liên tục tại mọi chỗ nối
```

### §5.2 Mapping actor

- Nhận `/lio/scan`, dùng ROG-map backend (vendor).
- Kết quả cập nhật có kiểu: `UPDATED` hoặc `NO_CHANGE(reason)`. **Không bao giờ poison.** Gặp exception thì reset map, tăng `world_generation` và phát event.
- Chỉ có **một** cài đặt WorldView và **một** bộ duyệt voxel (DDA). Luôn tôn trọng `unknown_policy` được truyền vào.
- Bán kính inflation lấy từ một nguồn duy nhất.
- Xuất snapshot bất biến kèm `revision` và vùng thay đổi.
- API: `classify`, `traversable(segment, policy)`, `swept(trajectory, policy)`, `revalidate(timeline, changed_region)`.
- `ResetMap` khi epoch LIO đổi.

### §5.3 Planner (worker, hàm thuần)

```text
PlanningRequest { anchor PVAJ, goal (frame LIO), world snapshot, profile, limits, deadline }
  → A* guide → CIRI corridor → MINCO MAIN (UNKNOWN theo profile)
  → BACKUP: seed phanh tất định → tối ưu (known-free ở mặc định / UNKNOWN ở khéo léo)
  → Certifier ĐỘC LẬP: dynamics và flatness (yaw rate ±ω_max) · corridor liên tục · swept world ·
                       route regression (đóng khi dữ liệu route hỏng) · R1 · nối liên tục
  → Result<Bundle, Reason>
```

- Kết quả optimizer là enum: `Converged`, `Cancelled`, `DeadlineExceeded`, `Infeasible`, …
- Planner không giữ state ẩn giữa các request. Warm-start là input tường minh, có khoá theo identity.
- Không có đường heading rebind.
- Nhịp 10 Hz, deadline solve 80 ms (tầng a).
- Giới hạn động học:
  - MAIN nominal 5/5/8 (tốc độ/gia tốc/jerk) cho beta;
  - vật lý và BACKUP 12/9/30.

  Cả hai lấy từ config. Không ghi đè ngầm.

**Tận dụng và bỏ từ `main`**

| Tận dụng (dọn và sửa khi tách) | Bỏ |
|---|---|
| `astar`, `ciri`, lõi `corridor_generator`, `minco`, `lbfgs`, `sdlp`/`sdqp`, giải nghiệm đa thức (sửa lỗi nghiệm bội), toán phanh BACKUP, flatness map, swept check (gộp lại thành một) | Phần điều phối `planner.cpp`, `planner_facade`, `Config`, `log_utils`, các hook viz, `quickhull`, `yaw_traj_opt` |

### §5.4 Bộ tổng hợp phanh

- Hàm thuần: (trạng thái đo được, world, giới hạn vật lý) → `Result<Bundle(BRAKE), Reason>`.
- Dùng đa thức dừng min-snap cộng swept check.
- Chạy trên thread Supervisor, budget 10 ms (hằng số tầng a). Vượt budget thì coi như không chứng nhận được.

## §6. Quy ước chung (D12, D22, D25)

### §6.1 Event log

```text
EventRecord { t_steady_ns, t_ros_ns, component, event, state_before, state_after, reason,
              identity {mission_id, lio_epoch, request_id, bundle_id, world_revision},
              values[≤16] (key, double) }
```

- Mỗi process có một ring buffer, cộng một thread writer ghi ra `/events` và file JSONL cho mỗi lần chạy.
- Ring đầy thì đếm số bản ghi bị bỏ và phát event `EventsDropped`.
- Script `tools/uavnav` dùng event log để:
  - tính KPI;
  - đếm theo từng reason;
  - dựng lại timeline.

### §6.2 Ba tầng tham số

| Tầng | Ở đâu | Đổi thế nào |
|---|---|---|
| (a) Hằng số | `limits.hpp` của từng thành phần, có comment ghi cách suy ra | Review thiết kế |
| (b) YAML có giới hạn | Struct có kiểu, đơn vị nằm trong tên (`_s`, `_mps`, `_m`), schema có min/max | Sửa YAML. Giá trị ngoài phạm vi thì process từ chối khởi động |
| (c) Profile và mission | File profile, file mission. Runner đặt tham số PX4 theo profile | Chọn khi chạy |

Mỗi giá trị chỉ có **một** nguồn.

### §6.3 Thời gian

- Bốn kiểu riêng: `SensorTime`, `RosTime`, `SteadyTime`, `Px4Time`. Mọi giá trị là int64 ns.
- Mỗi cycle tạo **một** `TimeSnapshot`.
- Budget được đo không tính thời gian chờ khoá.

### §6.4 Thread và nhịp

| Process | Thread | Nhịp |
|---|---|---|
| `lio` | ingest+predictor, estimator, events | IMU 200 Hz, state 100 Hz, scan 10 Hz |
| `px4_bridge` | executor, events | EV 10 Hz |
| `navigation` | mapping, planner, supervisor, events | map 10 Hz, plan ≤ 10 Hz, command 50 Hz |
| `px4_mode` | vòng px4_ros2, events | 50–100 Hz |

- Giữa các thread chỉ truyền dữ liệu bất biến.
- Không publish và không gọi callback khi đang giữ khoá.
- Lỗi của worker luôn có đường phục hồi hoặc fail-closed tường minh.

### §6.5 Message v2

`LioState`, `LioHealth`, `LioOdometry`, `Alignment`, `NavCommand` (P/V/A + yaw, frame LIO, epoch, `bundle_id`, phase, lease), `EventRecord`, `MissionDefinition`.

Quy ước bắt buộc (D28):
- **Miền thời gian của từng trường:**
  - `LioState.stamp`, `LioOdometry.stamp`: SensorTime.
  - `LioHealth.stamp`: SensorTime của mẫu IMU mới nhất được đánh giá.
  - `Alignment.stamp`: SensorTime của mẫu LIO trong cặp mới nhất.
  - `NavCommand.stamp` và `NavCommand.sample_time_ns`: RosTime.
- **`LioOdometry.quality`:** uint8, 0–100, lấy từ biên độ suy biến của LIO. Bridge chép sang `VehicleOdometry.quality`. Beta đặt `EKF2_EV_QMIN=0`, nên giá trị này chỉ được ghi log.
- **`MissionDefinition`:**
  - `yaw_rad` là yaw cho Y1; NaN nghĩa là dùng yaw lúc bắt đầu mission.
  - Với frame GPS, `z` là độ cao so với home của PX4.

### §6.6 Khi một process chết (beta)

Beta không tự khởi động lại process. Các thành phần khác phát hiện qua health stale và đi theo guard tương ứng:
- `lio` chết được coi là LIO LOST;
- `navigation` chết dẫn tới `COMMAND_STREAM_LOST` ở `px4_mode`.

## §7. Bố cục, thứ tự xây, theo dõi (D26, D16)

### §7.1 Package

```text
src/external/  px4_msgs, px4_ros2_interface_lib, livox_ros_driver2
src/vendor/    ikfom_vendor, ikd_tree_vendor, rog_map_vendor
src/sim/       uav_simulation, uav_description
src/core/      uavnav_core, uavnav_interfaces
src/estimation/fast_lio_core (toán FAST-LIO tận dụng, thuần), uavnav_lio_core (thuần), uavnav_lio
src/navigation/uavnav_world, uavnav_planning, uavnav_supervisor (thuần), uavnav_navigation
src/px4/       uavnav_px4_bridge, uavnav_px4_mode
src/uavnav_bringup             tools/uavnav/
```

Logic quyết định nằm trong thư viện thuần, test được không cần ROS. Package ROS chỉ là vỏ.

### §7.2 Lát cắt dọc

| Lát | Nội dung | Kiểm tra |
|---|---|---|
| S0 | `uavnav_core`, `uavnav_interfaces`, khung build, gate tối giản (thay `gate.sh` và các validator của `main`) | Unit test |
| S1 | `uavnav_lio` và `uavnav_px4_bridge`, tham số PX4 theo profile | Hover ở mode Position của PX4, EKF2 fuse EV, log `T`. Kiểm giả thuyết F34 |
| S2 | `uavnav_px4_mode`, Supervisor tối giản, quỹ đạo thẳng, yaw Y1/Y3a, reason bàn giao | Bay tới waypoint trong môi trường trống |
| S3 | `uavnav_world`, `uavnav_planning`, commit timeline, R1–R5 | Scene có vật cản |
| S4 | LIO LOST/REACQUIRING/khởi động lại, phanh, guard bỏ cuộc, cắt LiDAR trong sim | Fault injection |
| S5 | Gate beta (§7.3) | KPI trên các scene cố định |

### §7.3 Gate beta (D14, D18)

| Hạng mục | Ngưỡng |
|---|---|
| Va chạm trên các scene cố định | 0 |
| Mission chạy hết ở 1–5 m/s | Có |
| Bàn giao không có Reason | 0 |
| Unit test cho mọi state machine và mọi hàm quyết định an toàn | Có |
| Thời gian ở phase MAIN trên tổng thời gian di chuyển | ≥ 95% |
| Số lần vào BACKUP | ≤ 1 / mission |
| BRAKE trên scene tĩnh | 0 |
| HOLD không do waypoint STOP | 0 |
| Tốc độ trung bình / cruise trên đoạn thẳng | ≥ 0.85 |

Bảng tiêu chí đầy đủ để đánh giá sau beta là O4 trong DECISIONS.

### §7.4 Theo dõi bám thiết kế

- Mỗi work package trỏ tới mục § của spec mà nó hiện thực.
- [TRACEABILITY.md](../TRACEABILITY.md) ghi: yêu cầu → mục spec → WP → trạng thái → bằng chứng.
- Cuối mỗi WP có bước review so code với spec. Mỗi chỗ lệch thành một mục O\*.
- **Quy tắc dừng:** có ≥ 3 chỗ lệch đang mở, hoặc bất kỳ chỗ lệch nào chạm §2, thì dừng lại sửa thiết kế tổng.

## §8. Hoãn lại sau beta

| Hạng mục | Mã |
|---|---|
| Bảng tiêu chí đo đầy đủ | O4 |
| Chiến lược lên 10 m/s và đo vùng known-free | O7 |
| Y3b | D24 |
| Respawn process | §6.6 |
| Relocalize vào map cũ | D20 |
| Đo lại lựa chọn EV 10 Hz và τ | O2 |
| Ngưỡng suy biến | O6 |
| Lọc `T` | D21 |
