# Thiết kế hệ thống UAV Navigation

Baseline thiết kế ngày 2026-10-05. Đây là bản thiết kế tổng thể duy nhất.
[Safety contract](../safety/runtime_safety_current.md) sở hữu invariant, gate và
bypass; [roadmap](../ROADMAP.md) sở hữu thứ tự công việc tiếp theo. Khi văn bản
khác source/config, ghi sai lệch và giữ fail-closed; không tự đổi behavior.

## 1. Mục tiêu và trạng thái

Sản phẩm dùng FAST-LIO, world model do sản phẩm sở hữu, MAIN kèm certified
BACKUP và PX4 ROS 2 External Mode. Phạm vi được kiểm tra là SITL; hardware
Mid-360 bị chặn cho tới khi có immutable visibility certificate và verifier.
Component test hoặc mission COMPLETE riêng lẻ không phải flight qualification.

Mục tiêu dài hạn kế thừa để xem xét: mission ổn định tới 10 m/s, UAV radius
0.50 m, MAIN có policy known-free hoặc explicit UNKNOWN, BACKUP luôn known-free,
GPS bật và PX4 sở hữu fallback khi LIO không còn hợp lệ. 10 m/s là mục tiêu
chưa nghiệm thu; cấu hình MAIN hiện hành là 5/5/8, không được nâng bằng docs.
Không chấp nhận policy cho BACKUP đi vào UNKNOWN/OUT_OF_MAP.

| Nhãn | Nghĩa |
|---|---|
| IMPLEMENTED | Có đường source; không tự chứng minh an toàn hoặc tốc độ |
| COMPONENT_VERIFIED | Test có phạm vi cụ thể; không thay runtime distribution |
| PROPOSED | Ý tưởng cần review/đo trước khi đổi sản phẩm |
| DIAGNOSTIC_ONLY | Đo/report/replay; không cấp command authority |
| NOT_MEASURED / NOT_EVALUABLE | Thiếu phép đo hoặc evidence hợp lệ; không phải PASS |

## 2. Ownership và layout

```text
LiDAR + IMU -> FAST-LIO
  -> RegisteredScan + propagated odometry + typed health
  -> navigation_runtime (mapping actor + planning worker)
  -> immutable committed MAIN/BACKUP bundle -> command sampler
  -> NavigationCommand -> PX4 External Mode -> TrajectorySetpoint
```

| Boundary | Owner và trách nhiệm |
|---|---|
| Estimation | fast_lio_core/fast_lio_ros: synchronization, initialization, correction, propagation, health, reset epoch; registration map không phải planning map |
| Mapping | navigation_mapping: bounded observation worker, mutable ROG-backed integration, immutable WorldModelView; UNKNOWN/OOM semantics |
| Planning contracts | navigation_planning: ROS/vendor-free C++20 request/outcome, kinematic state, identity và limits |
| Solver | navigation_planning_backend: A*, CIRI corridor, MINCO, yaw, certified stop synthesis; candidate không tự sở hữu command |
| Execution | navigation_execution: identity/lease/commit/exposure, immutable timeline và sampling |
| Composition | navigation_runtime: ingress, worker scheduling, mission and publication wiring; backend history còn là extraction debt |
| Mission | navigation_mission: C++ YAML validation, waypoint identity/frame/limits/policy; Python parser chỉ orchestration/report |
| PX4 adapters | px4_external_odometry_bridge, px4_odometry_bridge, px4_navigation_external_mode: conversion, local admission, bounded Hold handover |
| Tooling | tools/runtime: runner, monitor, report; simulator truth chỉ evidence |

Source được nhóm trong `src/common`, `src/contracts`, `src/estimation`,
`src/mapping`, `src/planning`, `src/execution`, `src/runtime`, `src/px4`.
`src/navigation_bringup` sở hữu launch/RViz, `src/uav_description` sensor frames,
`src/uav_simulation` Gazebo assets. Vendor giữ license/provenance riêng;
`src/external` chứa dependencies pinned, không phải public product API.

Không có MappingWorldNode/PlanningControllerNode hoặc snapshot/bundle ROS
transport riêng trong sản phẩm hiện tại. Không dùng tên package `nav_*` của
thiết kế cũ như bằng chứng đã triển khai. Core mission-progress ownership phải
đọc cùng [mission authority cut](../safety/mission_authority_cut.md).

## 3. Interfaces và pipeline

| Interface | Producer → consumer | Hợp đồng |
|---|---|---|
| /lio/mapping_observation | FAST-LIO → mapping | RegisteredScan ghép corrected pose và cloud nguyên tử theo source time/frame; empty-valid khác absent |
| /lio/odometry_propagated | FAST-LIO → runtime/PX4 bridges | High-rate propagated state, epoch/sequence và source time; không thay bằng callback time |
| /lio/health | FAST-LIO → runtime/bridge/adapter | Typed navigation/covariance/observability/correction/propagation validity; newer-invalid đóng gate |
| /navigation/navigation_command | runtime → adapter | P/V/A/J, yaw/rate, source time, lease, epoch/goal/request/activation/sample, world và bundle identity |
| /navigation/mission_progress | core → adapter | Measured ordered progress, terminal receipt; planned endpoint không phải acceptance |
| /navigation/command_admission | adapter → core | Local admission receipt; không tạo mission owner thứ hai |
| /px4_adapter/mode_status | adapter → core | Mode activation and local state; không tự chứng minh mission complete |
| /fmu/in/vehicle_visual_odometry | external odometry bridge → EKF2 | NED/FRD frame/covariance/time conversion, exact source validity |
| /fmu/in/trajectory_setpoint | External Mode → PX4 | Finite frame-correct continuous P/V/A+yaw/yaw_rate; jerk không gửi cho PX4 |
| /navigation/diagnostics, /px4/diagnostics | runtime/adapters → tools | Observability; chuỗi KeyValue không được tạo quyền bay mới |
| /sim/ground_truth/odometry | Gazebo → monitor/report | Evaluation-only; không input vào FAST-LIO hoặc external vision |

Schema thực nằm trong [navigation contracts](../../src/contracts/navigation_contracts/msg)
được xem là source của field/QoS. Cleanup không thay schema hoặc subscriptions.
Các đề xuất evidence topics và mission-only entrypoint là PROPOSED, không phải
interface đã có. Goal/test và velocity-only diagnostic không được promote.

Nominal pipeline: validate ingress → pin source/epoch/mission/world → submit
bounded request → solve/certify candidate → final identity/latest-world/lease
check → atomic commit → sample committed bundle → adapter-local admission → PX4.
Candidate lỗi không mutate committed generation. Worker result cũ bị discard.
World mới cần validation/recertification trước khi command tiếp tục có quyền.

## 4. Estimation, frames và time

Estimator giữ IKFoM-compatible nominal state:
`(p_odom_imu, R_odom_imu, R_imu_lidar, p_imu_lidar, v, gyro_bias, accel_bias, gravity)`.
Rotations là SO3, gravity là S2; 23-DoF tangent covariance có blocks p(0:3),
R(3:6), extrinsic R(6:9), extrinsic p(9:12), v(12:15), bg(15:18), ba(18:21),
g(21:23). ManifoldState chỉ là output/interchange, không phải filter thứ hai.
Online extrinsic estimation không thuộc baseline runtime.

Pipeline: adapters → scan/IMU synchronization → stationary initialization →
propagation → deskew hoặc declared simultaneous-scan bypass → preprocessing →
correspondences/residuals → iterated correction → corrected outputs → insert
accepted registration points. Init/lost/rejected không phát zero odometry như
state hợp lệ. Corrected odometry chỉ sau correction trong Tracking; propagated
output có gate riêng. Reset do FastLioNode processing owner thực thi, xoá
history/ingress/visibility cũ và đổi epoch; component evidence không chứng minh
lifecycle recovery trong chuyến bay.

TF sở hữu bởi repo: `lio_odom -> base_link -> livox_frame -> livox_imu_frame`.
LIO corrected/propagated: `lio_odom -> base_link`. PX4 ingress ROS dùng
`px4_odom -> base_link`. ROS internal ENU/FLU, PX4 world NED/body FRD:

```text
C_ned_enu = [[0,1,0], [1,0,0], [0,0,-1]]
C_frd_flu = [[1,0,0], [0,-1,0], [0,0,-1]]
```

Basis conversion không phải world-origin/yaw alignment. Bridge rejects unsupported
POSE_FRAME_FRD; quaternion PX4 là body-FRD → world-NED. Pose covariance ở
lio_odom, twist covariance ở base_link; missing/nonpositive variance không được
chấp nhận. Initial prior là LIO-local zero, stationary IMU; PX4/simulator không
là estimator startup input.

SITL launcher tắt SIM_GZ_EN_ODOM, bật normal GNSS/baro/magnetometer/range aiding
và EV fusion. GNSS/EV bias có thể làm PX4 local origin khác lio_odom. Yaw/bias
witness và pure frame-transform contract hiện là DIAGNOSTIC_ONLY; chưa phải
continuous T3 authority. Không suy từ witness rằng setpoint đã được compensate.

PROPOSED T3: transform phải có source/frame/time/epoch/version, bounded bias
và derivatives, bảo toàn identity/order/time, tính cả tác động lên V/A và
certificate. Thiếu/reset/stale/innovation-failed phải reject theo fail-closed
boundary. Không cho transform tự giữ, thay hoặc reorder command. Chọn firmware
bias topic hoặc measured fit chỉ sau matched evidence, không dùng LPF τ như
một safety constant chưa đo.

Absolute timestamp là signed integer nanoseconds kèm clock domain; chỉ local
duration chuyển sang seconds ở calculation boundary. Backend bundle còn double
seconds/closures là debt, không được tuyên bố đã chuyển hết sang pure data.
Sensor time là measurement time; scan header semantics explicit, scan interval
phải được IMU bracket, không đoán relative time từ point index.
SITL/replay dùng ROS /clock, dương/monotonic/fresh, UXRCE_DDS_SYNCT=0.
Realtime dùng ROS system clock với transport synchronization thuộc PX4; không
cộng estimated_offset lần hai hoặc tự đổi clock khi mất timesync. Hardware
vẫn bị chặn. Wall-time budgets và source/sim-time freshness là hai phép đo khác.

## 5. World, planning và certificates

Mutable registration NN map của FAST-LIO không phải WorldModel. Mapping actor
xuất immutable view và pin generation/revision/observation stamp cho request.
UNKNOWN/OCCUPIED/OUT_OF_MAP phải được bảo toàn cả base và inflated layers.
BACKUP/EMERGENCY require known-free; explicit MAIN UNKNOWN policy không áp sang
suffix. Mission chỉ được hạ dynamic limit, không tăng physical limit.

Snapshot hiện là full export hoặc immutable parent/patch; PROPOSED fixed-size
COW chunks cần full-export oracle cho mọi cell state, inflation/virtual planes,
positive/negative slide, boundaries và OOM. Metadata-only successor không làm
mất identity; revision gap hoặc missing dirty history dùng full fallback.
AABB touched region không thay exact dirty-cell/chunk certificate. Worker hóa
snapshot export phải chứng minh supersession/cancellation và không publish stale.

Independent boundaries phải giữ: dynamics, flatness, continuous corridor planes,
swept-world, route regression, certified stop, PVAJ anchor continuity, identity,
lease và latest-world commit. Retained heading rebind không miễn MAIN route
certificate và không vay minimum MAIN reserve. Retained valid command được giữ
qua failed replacement; candidate uncertified không được expose.

Stop authorization cần concrete minimum-snap polynomial và support/extrema,
không chỉ scalar S-curve estimate. Steady a=j=0 và nonzero measured PVAJ là hai
nhánh khác; closed-form `d=0.9375*v²/a` không đại diện mọi braking state.
Interior duration retry family mới đã DEFER khỏi baseline: thiếu proof deadline/
cancellation; existing retries vẫn chưa qualified về latency tails. Paired yaw-stop
phải recertify position V/A/J và executed support/horizon khi đổi duration.
External `/lio/reset` đã DEFER; existing lifecycle helpers không cung cấp service
reset mới. Publication fence/topic-prior rearm cần R0 review trước khi mở lại.

PROPOSED independent certifier extraction: chuyển validator không đổi semantics,
chạy shadow trên pinned world và boundary, so disagreement theo failure kind
trước switch authority. CertificateRecord cần bind bundle digest, complete
required checks, world identity, profile fingerprint và certification version.
Data-only bundle, typed polynomial time và removal of evaluator/world-validator
closures cần parity oracle; không thay bool/callback authority bằng docs.

## 6. Execution và failure behavior

Hiện execution authority giữ committed/staged timeline, lifecycle/exposure,
lease và epoch. Backend private history chỉ phục vụ continuity, không command
publisher thứ hai. Publication phải giữ command-source stamp ≥ state-source
stamp; future source time reject, không clamp evidence.

| Tình huống | Boundary phải giữ |
|---|---|
| Candidate failed/stale | Discard; committed command chỉ giữ khi certificates/lease còn hợp lệ |
| World stale/revised | Suspend/recertify hoặc reject; không dùng old world identity |
| MAIN không còn usable | Certified positive BACKUP suffix, hoặc certified measured-state emergency theo clearance |
| Tracking/anchor pressure | Reject/reanchor theo contract; riêng pressure không authorize emergency |
| Emergency | Clearance witness + current KNOWN_FREE; one-shot recovery episode; failed synthesis không rearm mỗi tick |
| Localization reset | Invalidate old generation/history, cancel stale work, require current valid state |
| Command/state/health lease expired | Fail closed; không refresh lease bằng rejected sample |
| Measured waypoint acceptance | Core ordered progress; adapter receipt không tự advance mission |
| Terminal stop | STOP-only suppression/hold; PASS_THROUGH restart từ measured state |
| Hold handover failure | Bounded AUTO_LOITER attempts; release mode để PX4 native fallback chọn mode runnable |

PROPOSED reducer extraction: pure event→state/effects, single writer, immutable
jobs/results và explicit stale-result rejection. Đích trạng thái gồm Idle,
TrackingMain, SafetySuffix(Backup/Emergency), StoppedHold, Px4Hold; exposure chỉ
có nghĩa khi có command. Đây chưa là type/API hiện hành.
EpochReset, StateSample, WorldRevised, CandidateCertified/Failed, CommandTick,
AdmissionReceipt, StopObserved, DeadlineMissed phải được characterized từ source
trước extraction. Effects commit/discard/publish/suspend/recertify/cancel/Hold
thực thi trong shell sau reducer. Exit-site/line-number coverage không phải
semantic oracle. Preserve predicates, event→effect traces và fault controls.

Fast lane tách recert/emergency khỏi planning lane vẫn PROPOSED: phải đo
trigger→exposure, lock-wait và cancellation trước chọn scheduling. Không đổi
thread/timer chỉ vì thiết kế nói single writer. Dependency guard còn allow-list
cụ thể; roadmap phải đóng từng violation, không mở exemption blanket.

## 7. Budgets và configuration

[Planner configuration](../../src/runtime/navigation_runtime/config/planner.yaml)
và [typed timing](../../src/planning/navigation_planning/include/navigation_planning/planning_timing.hpp)
là source triển khai; safety contract sở hữu interpretation.

| Đại lượng | Hiện hành | Giới hạn bằng chứng |
|---|---|---|
| MAIN nominal V/A/J | 5/5/8 | Không phải PX4 physical capability |
| Physical/BACKUP V/A/J | 12/9/30 | 9 m/s² horizontal configuration bound; thrust/vertical/3-D/ramp chưa được qualify |
| Radius sum | 0.50+0.227+0.05+0.173+0.05=1.00 m | Tracking term derived/provisional; không phải measured margin |
| Corridor-plane violation | 0.01 m | Continuous polynomial certificate; conditioning debt còn mở |
| A* attempt/total | 20/40 ms | Reserve finalization 40 ms trong solve 80 ms |
| Solve/forward stitch | 80 ms / 400 ms | Không lẫn future lead hay wall/source-time clock |
| Planning/command/snapshot | 10 Hz / 50 Hz / 100 ms | Producer rate không chứng minh receive freshness |
| Minimum MAIN reserve | 80+400+100+20=600 ms | Không vay reserve cho heading rebind |
| Command stream/adapter state | 100 ms / 200 ms | Distinct source/receive boundaries |
| Runtime observation freshness | 500 ms | Exact timestamp pairing tại ingress |
| Visibility floor/cap | 14/20 m | Actual stop support + reaction + radius phải fit horizon |
| Watchdog/completion tolerance | 1.0 s / 0.20 m | Không tune từ smoke test |
| World sweep | 0.5 inflated-map resolution; 2–50 ms | Không dùng sampled guide penalty thay world certificate |

Units: s, m, m/s, m/s², m/s³, rad; clock boundary dùng integer ns. Derived robot
radius và search geometry không có YAML owner thứ hai. `config/runtime/mapping.yaml`
là canonical runtime profile; CMake copy vào install, không có hand-edited twin.
Objective/route-reference weights quality-only; không relax hard certificates.
Mission parser product là C++; Python chỉ tooling. Không gộp các age limits
khác clock/producer/consumer bằng cách sao chép một hằng số.

Tracking experiment default `off`; explicit `relaxed` là declared diagnostic
health/tracking suppression và qualification-ineligible. Không dùng default
relaxed mô tả cũ. Hardware deployment remains blocked; configuration fingerprint
không tương đương full cross-process ConfigWitness/SafetyProfile implementation.

## 8. Verification và unresolved debt

Build/test và gate chạy trên system Python/ROS Jazzy, Release và root artifacts.
Ledger/doc/dependency/mission checks bảo vệ cấu trúc/contract, không qualification.
SITL + representative recorded data cần complete provenance: source/build,
scenario/route/speed/policy/fusion, timestamp/clock, cleanup và failure retained.
No-sample, at-rest requested high speed, stale-install hoặc infrastructure-invalid
không được đưa vào distribution như pass. Đo achieved speed, coverage và tails.

Debt chuyển sang roadmap mới:

- Safe forward successor/liveness và full mission completion chưa nghiệm thu.
- Tracking/frame-error decomposition U1, bias/reset U2, physical reaction U4,
  yaw/alignment U6, actual braking/horizon U9 còn thiếu representative distributions.
- T3 continuous product authority, certificate transport, full profile witness,
  independent certifier và reducer replay chưa được triển khai đầy đủ.
- Dependency extraction, private backend history và double-time bundle còn mở.
- Các branch-only safety fixes chưa được chọn vẫn giữ trong local Git bundle,
  không được xem là redundant chỉ vì khác HEAD. Roadmap triage từng counterexample.
- TB-003 CIRI two-pass reference không promoted; HG-011 hardware block còn nguyên.

Runtime artifacts cũ vẫn ở máy nhưng không thuộc code backup theo yêu cầu owner.
Không gắn qualification với file đã bị xoá hoặc với legacy ID không có record.
Chẩn đoán/checkpoint cũ được bảo toàn trong backup, không tiếp tục làm authority.

U1/U2 evaluators trả NOT_EVALUABLE trong baseline, không có trusted summaries:
frame witness identity/time/basis và source-order/bounded-horizon còn thiếu.
Host telemetry background recorder đã hoãn để tránh ghi đè metadata và poll-time
RTF giả freshness; raw artifacts không trở thành distributions. U4 deduplicate
exact event identity, conflicting duplicates invalidate evidence.
