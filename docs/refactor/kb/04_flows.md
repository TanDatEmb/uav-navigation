# KB-04: Các luồng hoạt động (hiện trạng)

Pipeline đích, kèm budget và cách chia lane, nằm ở `design/A3_pipeline_timing_target.md`. Ở đây chỉ mô tả luồng **đang chạy** và nhánh lỗi chính của mỗi luồng.

## F1. Khởi động và readiness (SITL, `runner._run_sim_unlocked`, runner.py:3617-3868)
Thứ tự khởi động các process:
1. `monitor`
2. `px4_gazebo`, rồi chờ `/world/<w>/clock` (tối đa 90 s)
3. `xrce_agent`
4. ros_gz bridge và `gz_lidar_bridge`
5. `rosbag`
6. `visibility_bridge`
7. `px4_ingress`
8. (nếu có) `world_observation_gate`
9. `mapping`: launch runtime, rồi kiểm log witness
10. `lio` (fast_lio + EV bridge + RSP)
11. `rviz`

Sau đó runner chờ lần lượt, mỗi bước có timeout:
- imu và lidar > 0 (90 s)
- LIO `TRACKING` (90 s)
- external_odometry > 0 (45 s)
- mapping_ready (180 s)
- `external_mode`: marker READY (15 s), sau đó kiểm tracking witness
- `external_mode_scenario`

Hết timeout thì FAIL. Log witness lệch với config yêu cầu thì `CONFIGURATION_MISMATCH`.

## F2. Estimator (FAST-LIO)
**Luồng chính:**
1. IMU và LiDAR được adapter convert, rồi đưa vào ingress queue (4096 / 16).
2. `fast_lio_main` đẩy dữ liệu vào `MeasurementBuffer`. `synchronizeNext` trả về một trong bốn kết quả: group, discontinuity, error hoặc wait.
3. `processInternal` chạy lần lượt: validate → predict (`state_time_` = scan end) → deskew → preprocess → correct (tối đa 4 vòng) → lifecycle → chèn điểm vào map.
4. `publish`: corrected odom, RegisteredScan, health (stamp = scan end).
5. Worker propagate nhận correction, re-anchor và replay, rồi publish 50 Hz: `/lio/odometry_propagated`, TF, health (stamp = thời điểm propagate).

**Nhánh lỗi:**

| Lỗi | Kết quả |
|---|---|
| Không có end bracket IMU | chờ (**mãi mãi** nếu IMU bị từ chối, R4-01) |
| IMU gap | discontinuity, giữ state kiểu ZOH (R4-03); scan kế tiếp có thể cũng bị từ chối (R4-02) |
| Predict fail trước lần correct đầu tiên | kẹt vĩnh viễn (R4-12) |
| Correct fail | rollback. Khi ở Tracking thì chuyển Degraded; đủ số lần liên tiếp thì Lost |
| Mất LiDAR (theo wall clock) | 0.2 s → Degraded, 1.0 s → Lost |
| Exception | worker fail, node vẫn chạy nhưng **không còn output** |

## F3. Mapping và world snapshot (thread mapping trong runtime)
**Luồng chính:**
1. `/lio/mapping_observation` → `onRegisteredScan` (kiểm contract, epoch, sequence) → `MappingWorker.submitFromWaiting` (latest-only, validate ngay trong mutex).
2. Worker: validate lại độ tươi 0.5 s → `MappingActor::process`:
   - epoch mới thì rebuild map;
   - `ROGMap::updateMap`: slide, raycast, log-odds, inflation;
   - tính dirty region, cập nhật change history;
   - export full hoặc patch.
3. Revalidate bundle active/pending. Có 5 nhánh:
   1. vùng thay đổi không giao với bundle,
   2. chạy full `validateWorld`,
   3. endpoint terminal,
   4. endpoint recovery đã hết hạn,
   5. reject.
4. Dưới L2–L4: `publishAndFinalizeDecision`, tức publish world + chuyển certificate + (nếu cần) invalidate, rồi resume command đang bị suspend.
5. Handler diagnostics dựng khoảng 100 KeyValue ngay trên thread mapping.

**Nhánh lỗi:**

| Lỗi | Kết quả |
|---|---|
| Decode lỗi, hoặc process ném exception | `mappingFailStop` → `abort()` |
| Epoch không khớp | Superseded |
| Commit fail | invalidate + throw |
| BELOW_GROUND / ABOVE_CEILING | actor bị poison, worker FATAL (R6-04) |

## F4. Chu kỳ planning 10 Hz
**Luồng chính:**
1. Wall timer → `schedulePlanningCycle` → `currentPlanningKey`. Snapshot lấy dưới L2–L4. Trả nullopt nếu thiếu goal / state / world, đang có pending successor, đang PX4Hold, hoặc terminal hold đang pending.
2. (Tuỳ chọn) job heading rebind → `PlanningWorker.submit`.
3. `runCycle` (4010-7548):
   - Gate: epoch; world stale thì cancel + suspend; state stale thì return; BACKUP/EMERGENCY đang bay thì return.
   - Xử lý certified stop / completion / restart-from-rest.
   - `classifyPlannerRenewal`.
   - `PlannerFacade::plan`.
4. Bên trong planner:
   1. resolve goal
   2. A*
   3. corridor
   4. MINCO
   5. yaw
   6. flatness
   7. backup
   8. `authorizeAndStage` (route regression, swept world trên world mới nhất, `commitIfCurrentOrUnaffected`)
   9. export bundle
5. `classifyPlannerResult`:
   - `CommandReady` → `commitPlannerCandidate`;
   - Retain/Validate → `validateRetainedCommand`;
   - RetryFromRest → timeout có thể dẫn tới fail-closed;
   - FailClosed.
6. Dựng decision trace (khoảng 400 field) và JSON path.

## F5. Commit candidate (`commitPlannerCandidate` 2933-3429)
- Có 16 loại reject: export, activation window, anchor khớp, endpoint, route boundary, MAIN reserve, anchor state, …
- Mỗi lần reject đều gọi `planner_->discardCommandCandidate()`. Hàm này có thể xoá mất successor của worker nếu lời gọi đến từ thread command (R3-01).
- Successor đi qua `stagePending` với anchor đã đặt trước. Candidate immediate đi qua `admitImmediateCandidate` (re-check dưới L2–L4).
- ACK queue fail → xoá command của identity hiện tại và fail-closed.

## F6. Retained validation, emergency, BACKUP (`validateRetainedCommand` 7564-8587)
1. Tính anchor error **raw** (command tại `now` so với state tại source) và **time-aligned**; world validation; các bridge phase / path / experimental; projected bound (chỉ nhìn một period, R3-10).
2. Điều kiện emergency (`measuredStateEmergencyMayReplaceCommittedCommand`, chỉ từ TrackMain) → `commitEmergencyBrake`: state lấy ở source, stamp lấy `now` (R3-09) → commit immediate.
3. Transaction cuối cho ra một trong các kết quả: discard, superseded, lease fail → fail-closed, emergency fail → PX4Hold, recovery bridge, MAIN bridge, certified suffix (+ kích hoạt BACKUP), fail-closed.
4. Lệch giữa suffix raw và certificate aligned có thể làm fail-closed ở chặng STOP cuối, dù vehicle vẫn nằm trong tube (R3-11).

## F7. Command 50 Hz (`publishCommand` 8590-9908)
**Luồng chính:**
1. Gate epoch → world stale (cancel + suspend + **không gửi message**) → `consumeHeadingRebind` → activation pending → watchdog.
2. Không có command thì return. Lease bị lease mới thay trong lúc chờ lock thì **return, không publish** (R3-15).
3. Lease invalid → latch + fail-closed + publish `REJECTED/EMERGENCY` với PVAJ = 0.
4. `CommandSampler.sample` → stopped-hold / terminal → transaction exposure → publish `READY / COMPLETED / BRAKING` với `valid_until = now + 100 ms`, **trong lock** (R3-14).

## F8. Adapter: command → PX4 setpoint
**Admission (`onNavigationCommand` 1143-1352):**
- lần lượt: contract → lease → quyền terminal → session identity → tuổi odometry 0.2 s → thứ tự sample_id → tracking envelope (box, R5-16) → commit + gửi `CommandAdmission`.
- Command xấu thì reject và giữ command trước. Envelope vượt ngưỡng thì safety stop. Odometry stale thì `failNavigation`.

**`updateSetpoint` 50 Hz (2063-2582):** xét lần lượt, gặp điều kiện đầu tiên thì dừng:
1. failure / receipt / handover → giữ đứng yên
2. chưa arm hoặc z ≤ 0.5 → vận tốc 0
3. lease odometry fail → `failNavigation`
4. health xấu → `failNavigation`
5. command stale → safety stop
6. `COMPLETED` → giữ vị trí + cửa sổ recovery có giới hạn
7. velocity-only (experiment) → setpoint vận tốc
8. còn lại, PVA: căn frame **chỉ tịnh tiến**, `enuToNed`, `yawEnuToNed` → `TrajectorySetpoint`

Không có command trong 5 s kể từ lúc airborne thì safety stop. Executor thử lại PX4 Hold mỗi 250 ms cho tới khi PX4 xác nhận.

## F9. LIO → PX4 EKF2 (EV bridge `on_lio` 215-373)
**Luồng chính:**
1. Kiểm identity (epoch, sequence phải mới hơn hẳn). Epoch đổi thì reset baseline và latch.
2. `convert_ros_lio_odometry`.
3. Kiểm biểu diễn được bằng float, kiểm variance.
4. `TimestampConverter` đổi sang µs, age ≤ 0.5 s, từ chối µs trùng hoặc lùi.
5. Jump continuity + latch.
6. Gate: diagnostics TRACKING, covariance, generation, fresh ≤ 2 s.
7. Publish `VehicleOdometry` (NED, `reset_counter` = generation lấy từ diagnostics).

**Nhánh nguy hiểm:**
- dt < 1e-4 s hoặc dt ≤ 0 → reseed ở trạng thái trusted mà **không so hình học** (R5-08, S1);
- gap hoặc invalid → reseed (R-02);
- `reset_counter` lấy từ diagnostics cũ ngay sau khi epoch đổi (R5-09).

## F10. Mission và handover
1. `MissionProgress` (tick 20 Hz): ModeStatus (transient_local) + CommandAdmission → xử lý crossing, continuation, stop confirmation → Goal kế tiếp hoặc Complete.
2. Publish `/navigation/mission_progress` và `/navigation/mission_complete`.
3. Adapter nhận receipt thì handover sang PX4 Hold.
4. Scenario xem là `COMPLETE` khi có `mission_complete` và nav_state = AUTO_LOITER. Nếu không, sau `hold_handover_timeout_s` thì kết luận `HOLD_HANDOVER_FAILED`.

## F11. Reset localization epoch
1. Ingress thấy epoch mới hơn → `resetForLocalizationEpochLocked`: gọi tay khoảng 60 bước theo thứ tự (1905-1970), trong đó có unlock để drain mapping.
2. Adapter và bridge reset baseline theo epoch.
3. **Thực tế trong SITL, epoch LIO không bao giờ đổi** (R4, public frame generation). Vì vậy luồng này chỉ được test ở mức component.

## F12. Fail-closed → PX4 Hold
- Runtime có khoảng 25 lời gọi `failClosedLocked`. Chúng đưa lifecycle về `Px4Hold` (absorbing), sau đó runtime publish `REJECTED` hoặc ngừng lộ command.
- Adapter gặp `REJECTED`, stale hoặc health xấu thì gọi `safetyStopNavigation` / `failNavigation` → handover → executor thử lại Hold 250 ms.

## F13. Đánh giá (offline)
1. Nguồn evidence: monitor (`samples.jsonl`, `monitor.json`) + scenario (`scenario.jsonl`) + rosbag.
2. `report.build` → `evaluation.load_evaluation_inputs` (reducer lifecycle và world) → `evaluate_session` (mission, safety, tracking, motion, evidence, C0_SW) → `report.json` → `REPORT.html`.
3. Verdict: PASS khi không có lý do fail nào. Có các override cho PAUSED_SAFETY_STOP, ABORTED, FAILED_COMPONENT.
4. **HTML tự tính một overall khác** (R7-41).
