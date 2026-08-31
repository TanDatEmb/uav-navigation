# Báo cáo isolation SITL External Mode — 2026-08-31

## Kết luận ngắn

Run full-stack `sanity_open`, `5 m/s` hiện **không đủ điều kiện qualification**:
`INFRASTRUCTURE_INVALID`, sau đó External Mode fail-closed với
`ODOMETRY_STALE`/`RECOVERY_TIMEOUT`. Không có bằng chứng collision hoặc PX4
failsafe. LIO vẫn báo `TRACKING`; vì vậy không quy lỗi này cho planner, mapping
hay LIO.

Các run isolation độc lập 30 s đã thu hẹp ranh giới. Bảng dưới là phân phối
5 repetitions cho mỗi tầng; `over-500` là số run có max wall-arrival gap của
`/world/open/stats` vượt lease 500 ms:

| Tầng thêm vào | Clock min/median/max | Stats min/median/max | over-500 | Nhận định |
|---|---:|---:|---:|---|
| GZ-only, không PX4/ROS 2 | 100.5 / 102.1 / 105.2 ms | 110.3 / 128.9 / 166.4 ms | 0/5 | baseline không vượt lease |
| GZ + PX4 | 405.6 / 675.2 / 2283.8 ms | 486.0 / 755.7 / 2316.1 ms | 4/5 | PX4–GZ integration/host scheduling là blocker chính |
| GZ + PX4 + PointCloud bridge | 102.6 / 583.8 / 3569.3 ms | 163.7 / 604.1 / 3599.7 ms | 4/5 | PointCloud tăng tail và tương tác xấu với PX4–GZ |

Các bridge control đã được lặp tiếp. Tính cả screening đầu tiên, clock-only có
`/stats` khoảng `528, 1433, 202, 645, 1762 ms` (4/5 vượt lease), IMU-only có
`2080, 2079, 1558, 1936, 803 ms` (5/5 vượt lease). Đây là bằng chứng bridge
làm tail xấu hơn, nhưng vì PX4-only cũng đã 4/5 vượt lease nên chưa được quy
toàn bộ nguyên nhân cho bridge.

Screening bổ sung: ground-truth bridge đơn lẻ có stats gap `1019 ms`; combined
control bridge (clock + IMU + ground truth) và PointCloud bridge có stats gap
`2747 ms`. Các giá trị này là một mẫu mỗi cấu hình, dùng để định hướng chứ
không thay thế repeated distribution.

LIO screening với control + PointCloud bridge và đúng `fast_lio_params.yaml` có
stats gap `1358 ms`; log chỉ xác nhận node khởi động, chưa có đủ monitor/health
để gọi `TRACKING`. Vì hạ tầng native đã fail trước đó, screening này không được
dùng làm bằng chứng chất lượng LIO.

Một GZ-only sample có RTF min `0.039` ở transient khởi động nhưng
không có gap vượt 500 ms; không dùng RTF transient đó để thay đổi gate.

Các số liệu trên là wall-arrival gap của native Gazebo observer; `500 ms` chỉ
là lease chẩn đoán/contract hiện hành, không phải tham số được tune. Mỗi ô mới
là một run 30 s, nên đủ để xác định hướng và chưa đủ để đóng qualification hay
thay đổi hard gate.

## Bằng chứng runtime

- Full run: [025303 report](../../.artifacts/runtime/external-mode-check-20260831T025303-128894/report.json), native diagnostic run: [025613 report](../../.artifacts/runtime/external-mode-check-20260831T025613-131169/report.json).
- Full run có `simulation_clock_active_wall_gap_count=17`, max active gap khoảng
  `6006 ms`; scenario kết thúc `RECOVERY_TIMEOUT`, nguyên nhân trạng thái là
  `ODOMETRY_STALE`.
- Native diagnostic của full run ghi nhận đồng thời `/world/open/clock` và
  `/world/open/stats` bị gián đoạn; RTF có lúc xuống khoảng `0.0067`. Đây là
  bằng chứng tải đã ảnh hưởng tới simulator/transport boundary, không chỉ là
  lỗi hiển thị báo cáo.
- Baseline PX4-only sau rebuild sạch của binary (`f65ddce...`, lockstep bật)
  vẫn có gap `0.91–2.88 s` ở cả 5/5 run. A/B tắt logger vẫn có `1.07–4.71 s`
  ở 5/5 run; A/B tắt PX4 LiDAR subscription vẫn có `0.55–6.77 s` ở 5/5 run.
  Các event không phải terminal: chúng xảy ra giữa source-time đang tăng,
  khoảng simulation time `5–22 s`, sau khi PX4 đã báo `Ready for takeoff`.
  Vì vậy đây là active-runtime stall, không phải chỉ là startup warm-up.
- Đối chứng bằng model/world chuẩn của PX4: chạy `make px4_sitl gz_x500`
  với `GZ_GUI=0`, không ROS 2, không model Mid360 và không LiDAR bridge của dự
  án vẫn ghi nhận 5 native `/world/default/clock` gap từ `537 ms` đến
  `2098 ms`, 5 `/stats` gap từ `538 ms` đến `2163 ms`, với RTF thấp nhất
  `0.00222`. Các gap xảy ra trong source-time khoảng `3.3–23.1 s`, không chỉ
  lúc khởi động. Artifact chẩn đoán là
  [px4 standard gz_x500 profile](/home/letandat/Dev/Autopilot/.artifacts/runtime/px4-standard-gz-x500-profile-115054/gazebo_native_summary.json).
  Bằng chứng này đẩy xác suất lỗi hệ thống PX4–Gazebo/lockstep/host lên cao
  hơn và loại trừ giả thuyết “chỉ do cấu hình LiDAR hoặc world dự án”; nó vẫn
  chưa chỉ ra một source line cụ thể.
  Run đối chứng này có `apt upgrade` chạy nền trên host nên không được xem là
  clean-host distribution; tuy vậy các PX4-only run trước đó cũng tái hiện
  active gap trước thời điểm nhiễu này. Vì vậy nó là bằng chứng boundary, không
  phải bằng chứng đã định lượng ảnh hưởng riêng của host load.
- Cumulative ladder trước đó không được dùng để kết luận nhân quả: các node
  chạy nối tiếp trong cùng process tree và visibility bridge nhận `0` cloud.
  Các bảng trên dùng fresh process cho từng tầng và đã được lặp 5 lần.
- `make build` đã build thành công 23 package; `make test` đã hoàn thành với
  CTest không có failure/skip và Python runtime tests `209` tests passed.
- Thử nghiệm backport lockstep local trên PX4 `release/1.17` (build có thay đổi
  chỉ ở POSIX lockstep scheduler) đã build thành công. Functional test
  `functional-lockstep_scheduler_test` chạy đủ `100` vòng và PASS. Native
  `gz_x500`, không ROS 2, được lặp 5 phiên 30 s; `/clock` có max
  `101–111 ms`, `/stats` có max `104–293 ms`, và cả `5/5` phiên không có event
  vượt lease `500 ms`. Đây là tín hiệu fix đúng boundary, chưa phải
  qualification toàn bộ map/5 m/s; artifact nằm tại
  [px4-lockstep-backport-20260831](/home/letandat/Dev/Autopilot/.artifacts/runtime/px4-lockstep-backport-20260831/).
  RTF transient thấp ở lúc khởi động ở một số phiên nhưng source time vẫn tăng
  và không tạo native gap event; không dùng transient đó để nới gate.

## Phân loại nguyên nhân và ownership

1. **Primary — PX4 SITL ↔ Gazebo systems/plugins/transport và host
   scheduling.** PX4-only đã vượt `500 ms` trong khi chưa có ROS bridge, LIO,
   mapping hay External Mode. Cần tái hiện bằng đúng PX4 tree/build mà người
   dùng đã thử; PX4 tree hiện vẫn dirty, nên provenance của môi trường phải
   được khóa trước khi sửa.
2. **Secondary — `ros_gz_bridge` serialization/scheduling.** PointCloud
   `PointCloudPacked → PointCloud2` tạo tác động lớn nhất khi ghép với PX4;
   IMU cũng có tail vượt lease. Việc tách process hiện tại chưa chứng minh
   tách được tài nguyên simulator/transport.
3. **Not causal for this trigger — LIO, mapping, planner, External Mode.** LIO
   vẫn `TRACKING`; mapping drop/replacement là triệu chứng backpressure. Gọi
   `Hold`/từ chối recovery khi receive-age stale là hành vi fail-closed đúng.
4. **Independent issue — legacy `long_three_pillars`.** Artifact legacy có
   infrastructure hợp lệ và residual/tracking gate vượt `0.25 m`; phải xử lý
   riêng sau khi hạ tầng clock ổn định, không gộp với `ODOMETRY_STALE`.

## Kế hoạch test tiếp theo

### A. Khóa provenance và baseline

Mỗi run phải ghi PX4 commit + dirty diff, Gazebo/ROS/Python versions, GPU/driver,
CPU governor, số core, RAM/swap, process tree, PSI CPU/IO/memory và environment.
Không dùng `SPEED_CAP_MPS=10`; chỉ dùng mission-native `5 m/s` cho acceptance.

### B. Isolation ladder độc lập

Chạy tối thiểu 5 repetitions cho mỗi tầng, 30–60 s/tầng, cùng world/model,
`GZ_GUI=0`, partition/domain riêng; chỉ thêm tầng sau khi tầng trước có phân
phối gap đạt yêu cầu:

`GZ-only → PX4 → clock bridge → IMU bridge → ground-truth bridge →
PointCloud bridge → visibility bridge → MicroXRCEAgent → monitor → LIO →
mapping → px4_ingress → External Mode`.

Đã hoàn tất 5 repetitions cho GZ-only, PX4-only, PX4 + PointCloud bridge,
clock-only và IMU-only; ground-truth/combined bridges và LIO đã có screening.
Bước kế tiếp là lặp ground-truth/combined bridges rồi thêm từng node ROS 2:
visibility, MicroXRCEAgent, monitor, LIO, mapping, px4_ingress và External Mode.

Với mỗi tầng lưu:

- native `/clock` và `/stats`: max/p50/p95 gap, RTF p05/p50/min, source
  regression;
- ROS stream rate và wall/source gap của clock, IMU, PointCloud, corrected và
  propagated odometry;
- CPU/RSS/thread count của `gz sim`, PX4, bridge và node vừa thêm;
- mapping replacement/drop, LIO health, PX4 state, External Mode predicate
  reset/hold/re-entry.

### C. Tiêu chí dừng và acceptance

- Không nới `0.20 s`, `0.50 s`, `0.15 m/s`, `5 s`, `15 s` hay tracking gate
  `0.25 m` để biến run thành PASS.
- Native clock/stats gap vượt lease phải là `INFRASTRUCTURE_INVALID`, không phải
  acceptance evidence.
- Chỉ sau khi ladder hạ tầng ổn định mới chạy `sanity_open` ở `1/3/5 m/s` ít
  nhất 5 repetitions; sau đó structured/long/tunnel/clutter ở `5 m/s` ít nhất
  3 repetitions/seed.
- Qualification vẫn yêu cầu waypoint completion, continuity, speed recovery,
  altitude, clearance, PX4 state, propagated odometry, LIO health, mapping
  accounting và zero collision/failsafe.

## Hướng fix an toàn

Chưa có source line đủ bằng chứng để sửa product navigation. Không sửa planner,
không tăng queue/freshness timeout, không giảm resolution/rate/FOV như một
product fix.

### Trạng thái PX4 1.18

Kiểm tra branch local và `upstream/release/1.18` cho thấy 1.18 đã có nhóm sửa
lockstep trước đó (`162cce35d8`, atomic timeout và tách pha broadcast), nhưng
production code vẫn dùng `pthread_cond_wait()` không có wall-clock recheck và
điều kiện lifetime cũ `!timeout && _setting_time`. Fix signal-loss/lifetime
đầy đủ hơn (`27a21488b2`) chưa là ancestor của `release/1.18`. Issue chính thức
[#27915](https://github.com/PX4/PX4-Autopilot/issues/27915) cũng ghi nhận lỗi
lockstep tái hiện trên `v1.18.0-beta1` với `gz_x500`/Gazebo 8.14. Do đó 1.18
chỉ nên là một nhánh A/B, không phải lời giải mặc định; cần so sánh
`1.17 nguyên bản → 1.18 nguyên bản → 1.18 + backport 27a` trên cùng host,
world, model và observer trước khi chuyển toàn bộ qualification.

1. **Ưu tiên PX4-only:** profiling callback/transport/plugin của PX4–GZ và host
   scheduler; chỉ dùng CPU affinity/priority như thí nghiệm có đo lường, không
   che stall. Nếu xác định blocking I/O/serialization trong simulation path,
   chuyển sang bounded asynchronous handoff và đo lại phân phối.
2. **Sau đó bridge:** tách `/clock` thành bridge tối giản, tách IMU khỏi
   ground-truth; đánh giá latest-only bounded conversion cho PointCloud sao cho
   overload bỏ cloud cũ, không phát delayed stale cloud. Giữ nguyên semantics
   cảm biến; mọi thay đổi behavior phải ghi ledger trước khi merge.
3. **Harness:** bật native observer mặc định cho `external-mode-check`, bổ sung
   gap ở mốc `0.20 s` và log đầy đủ predicate/reset của automatic recovery; đây
   là observability, không thay đổi safety behavior.

## Nghiên cứu cách fix ổn định

### Thứ tự ưu tiên

| Ưu tiên | Hướng | Vì sao | Điều kiện chấp nhận |
|---|---|---|---|
| P0 | Khóa provenance và profile PX4-only | PX4-only đã có tail vượt lease; chưa thể kết luận bridge là nguyên nhân duy nhất | Cùng PX4 build/dirty diff, Gazebo, world, model; có CPU scheduling, native clock/stats và process trace |
| P1 | Sửa đúng lớp PX4–Gazebo lockstep/plugin nếu profile chỉ ra blocking hoặc lỗi lifetime | Đây là lớp duy nhất đã fail trước khi có ROS 2; sửa phải nằm ở owner của stall | Repeated distribution giảm tail, không đổi sensor/time semantics, không có regression PX4 state |
| P2 | Tách và bounded hóa bridge high-bandwidth | `PointCloudPacked → PointCloud2` làm tail xấu thêm khi ghép PX4; bỏ dữ liệu cũ khi quá tải phù hợp hơn phát dữ liệu trễ | Giữ đúng frame/fields/timestamp; đo drop/backlog/age và LIO health; ghi ledger nếu đổi behavior |
| P3 | Thử `ros_gz_point_cloud` native plugin | Có thể tránh generic bridge conversion cho GPU LiDAR | Chỉ dùng nếu build được tương thích với Jazzy/Gazebo hiện tại và A/B chứng minh tốt hơn; package hiện không có trong môi trường local |
| P4 | CPU affinity/priority hoặc render-engine | Chỉ là biện pháp giảm nhiễu host, không phải fix gốc | Chỉ giữ nếu lặp lại được lợi ích trên cùng máy; không che mất stall bằng timeout |

### Các hướng không xem là fix

- Không tắt lockstep: tài liệu PX4 xác nhận Gazebo chạy lockstep và không hỗ trợ
  disable lockstep.
- Không dùng `PX4_SIM_SPEED_FACTOR` để chữa trễ. Biến này chỉ đổi tốc độ mô
  phỏng; issue PX4 gần đây ghi nhận đường `set_physics` hiện có thể làm gravity
  trong physics engine về 0 nếu request không giữ lại các field khác.
- Không tăng freshness/clock lease, không giảm tracking gate, không dùng queue
  lớn để hấp thụ backlog, và không giảm LiDAR resolution/rate/FOV như product
  fix. Các cách này có thể làm báo cáo đẹp hơn nhưng không chứng minh simulator
  và dữ liệu vẫn đúng semantics.

### Cách triển khai fix sau khi có profile

1. Chạy A/B PX4-only với cùng binary: baseline, tracing, và từng thay đổi một.
   Nếu gap nằm trong native Gazebo nhưng PX4 process không bị CPU starvation,
   kiểm tra lockstep callback/transport và version compatibility trước khi sửa
   planner.
2. Chạy lại `PX4 + clock`, `PX4 + IMU`, `PX4 + ground truth`, rồi PointCloud.
   Với bridge, ưu tiên queue bounded/latest-only ở ranh giới conversion; không
   để callback conversion nặng giữ đường clock/IMU.
3. Chạy soak 10–15 phút, lấy RSS/thread count và tail latency để loại memory
   leak hoặc accumulation; sau đó mới chạy full ladder và các map ở mission
   native `5 m/s`.
4. Chỉ gọi là ổn định khi mỗi cấu hình đạt phân phối lặp lại, cleanup sạch,
   native clock/stats không vượt lease, không có stale odometry, và toàn bộ gate
   bay/clearance/PX4/LIO/mapping đều đạt.

## Đối chiếu nguồn bên ngoài

- Tài liệu Gazebo xác nhận `ros_gz_bridge` là network bridge giữa ROS 2 và
  Gazebo Transport, có conversion theo từng message và hỗ trợ `lazy`; lazy chỉ
  tránh subscriber nội bộ khi không có ROS subscriber. Xem
  [Gazebo ROS 2 integration](https://github.com/gazebosim/docs/blob/master/harmonic/ros2_integration.md#ros_gz_bridge).
- Tài liệu PX4 xác nhận Gazebo SITL chạy lockstep, `GZBridge` đồng bộ thời gian
  PX4 trong mỗi bước mô phỏng, và lockstep không thể tắt trên Gazebo; IO/CPU có
  thể tự giới hạn tốc độ. Xem
  [PX4 Gazebo Simulation guide](https://docs.px4.io/v1.17/en/sim_gazebo_gz/index#change-simulation-speed).
- Issue cộng đồng chính thức của `ros_gz` ghi nhận đúng lớp triệu chứng với
  `gpu_lidar`: bật sensor bridge làm giảm ROS topic rate và RTF, còn tắt bridge
  giữ RTF ổn định hơn. Xem
  [ros_gz issue #368](https://github.com/gazebosim/ros_gz/issues/368).
- `ros_gz` có plugin `RosGzPointCloud` để xuất trực tiếp GPU LiDAR thành ROS
  `PointCloud2`, nhưng môi trường local chưa cài package này và demo upstream
  không phải bằng chứng tương thích sẵn với stack hiện tại. Xem
  [GPU LiDAR point-cloud example](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_point_cloud/examples/gpu_lidar.sdf).
- PX4 cũng đang có issue lockstep khác về lifetime của condition variable trong
  POSIX SITL; đây không phải bằng chứng bug hiện tại nhưng là lý do phải khóa
  version/dirty diff và profile lockstep trước khi patch. Xem
  [PX4 lockstep issue #27915](https://github.com/px4/PX4-Autopilot/issues/27915).
- Issue `PX4_SIM_SPEED_FACTOR` cho thấy request `set_physics` chỉ chứa một field
  có thể ghi đè gravity/max-step-size về giá trị mặc định. Vì vậy speed factor
  không được dùng như workaround cho latency. Xem
  [PX4 speed-factor issue #27480](https://github.com/px4/PX4-Autopilot/issues/27480).
- Mã `gz-sim` hiện dùng sensor/render update thread và cơ chế chờ đồng bộ; vì
  vậy cần đo cả native simulator timing, không chỉ `ros2 topic hz`. Xem
  [gz-sim Sensors.cc](https://github.com/gazebosim/gz-sim/blob/main/src/systems/sensors/Sensors.cc).
- Một issue khác của `gz-sensors` cho thấy sensor update rate có thể thấp hơn
  scene refresh/render rate ngay cả khi dùng headless/server rendering; đây là
  lý do GPU configuration riêng lẻ chưa đủ để loại trừ scheduling/serialization.
  Xem [gz-sensors issue #332](https://github.com/gazebosim/gz-sensors/issues/332).

## Trạng thái thay đổi

Không có product source, safety gate, threshold hoặc ledger behavior nào được
thay đổi trong đợt test này. Các thay đổi dirty hiện hữu của worktree được giữ
nguyên. Kết luận hiện tại là **mixed infrastructure blocker**, chưa phải
qualification PASS và chưa nên áp dụng patch cục bộ vào planner/LIO.

## Kế hoạch triển khai fix thử nghiệm

### Track 1 — đưa môi trường về strict ổn định

1. Tạo bản provenance sạch cho PX4 (`commit`, submodule, build manifest, dirty
   diff rỗng), rồi lặp `GZ-only → PX4-only` trên đúng world/model 10 lần.
2. Thu native `/clock` và `/stats`, RTF, `perf sched`, `pidstat -w`, CPU PSI,
   RSS/thread count và process tree. Chỉ tiếp tục khi phân phối PX4-only đã
   được giải thích; không lấy một lần PASS làm bằng chứng.
3. A/B từng thay đổi: PX4/Gazebo version tương thích, lockstep callback/plugin,
   host affinity/priority (diagnostic), rồi bridge control. Mỗi biến thể phải
   giữ semantics thời gian, IMU, pose và actuator.
4. Tối ưu bridge sau cùng: PointCloud bounded/latest-only, tách process/queue,
   và thử direct `ros_gz_point_cloud` nếu build tương thích. Không cho callback
   PointCloud nặng làm ảnh hưởng `/clock`/IMU.
5. Soak 15 phút cho baseline và biến thể tốt nhất; loại mọi biến thể có memory
   tăng không giới hạn, stale source stamp, out-of-order data, cleanup lỗi hoặc
   làm LIO/mapping mất accounting.

### Track 2 — profile cuối cùng để đánh giá logic, không giả danh an toàn

Chỉ triển khai nếu Track 1 chưa đạt nhưng cần chạy toàn bộ logic map. Đây là một
profile tường minh, mặc định `strict`, ví dụ `simulation_evaluation_profile:
bounded_gap`; không sửa các gate product hiện tại (`state 0.20 s`, command
`0.10 s`, clock lease `0.50 s`, tracking `0.25 m`).

Contract bắt buộc của profile bounded-gap:

- Có một `max_wall_gap_s` cố định được chốt trước toàn bộ matrix; không tự động
  tăng theo map hoặc theo kết quả lần chạy. Giá trị chỉ được chọn sau phân phối
  baseline lặp lại và ghi vào ledger cùng owner, scope, safety impact, evidence,
  removal condition và verification command.
- Có thêm `max_gap_events_per_run` và `max_total_gap_s`; thiếu một trong ba,
  gap vượt trần, hoặc native source time không liên tục thì **FAIL**, không
  chuyển thành degraded.
- Khi gap xảy ra, runner chỉ ghi nhận `SIMULATION_PAUSE`; External Mode vẫn
  fail-closed, PX4 Hold, không phát command dựa trên state cũ. Chỉ resume sau
  khi propagated odometry/health đạt lại source-age và receive-age, PX4 báo
  Hold ổn định, vehicle stationary và bundle mới hoàn chỉnh.
- Mọi waypoint/clearance/collision/LIO/mapping/PX4 invariant vẫn bắt buộc. Các
  metric continuity/speed trong đoạn bị pause bị đánh dấu `invalidated`, không
  được nội suy hoặc tính là thành công.
- Report tách hai verdict: `logic_evaluation` có thể là
  `PASS_DEGRADED_BOUNDED`; `qualification_eligible` luôn `false` nếu có gap.
  Vì vậy profile này giúp kiểm tra logic thuật toán trên tất cả map, nhưng
  không chứng minh bay liên tục 5 m/s.

### Thứ tự code/test khi được phép triển khai Track 2

1. Thêm schema/profile và validation fail-closed; chưa thay đổi controller hay
   External Mode acceptance.
2. Bổ sung event ledger cho pause/resume, max/total/count, source-time gap,
   command suppression và PX4 Hold identity.
3. Viết unit tests cho boundary `0`, đúng trần, vượt trần, nhiều gap, tổng gap,
   source-time discontinuity, resume khi đang moving và thiếu health.
4. Chạy `sanity_open` 1/3/5 m/s; sau đó toàn bộ map 5 m/s tối thiểu 5 lần/map.
   Chỉ dùng kết quả để đánh giá logic nếu không có strict PASS.
5. Khi Track 1 đạt strict, xóa hoặc khóa profile bounded-gap khỏi đường
   qualification; không để workaround trở thành behavior mặc định.
