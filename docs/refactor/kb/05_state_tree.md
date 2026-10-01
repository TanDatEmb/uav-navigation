# KB-05: Cây trạng thái hệ thống

Trạng thái của cả hệ thống là **tích** của các máy trạng thái thành phần dưới đây. Mỗi mục ghi rõ enum hoặc cờ, transition kèm `file:line`, và owner.

```
S0 SYSTEM
├── S1 PX4 (owner: PX4) ─ nav_state {MANUAL, POSCTL, AUTO_TAKEOFF, OFFBOARD, EXTERNALn, AUTO_LOITER, …} × armed × failsafe
├── S2 ADAPTER (px4_navigation_external_mode)
│   ├── S2.1 mode_active_ {INACTIVE, ACTIVE}                    onActivate 821-875 / onDeactivate 877-905
│   │   └── ACTIVE ▸ WAIT_AIRBORNE → WAIT_COMMAND(≤5 s) → TRACKING_PVA | VELOCITY_ONLY(exp)
│   │                → COMPLETED_HOLD(+bounded recovery) | SAFETY_STOP → HANDOVER
│   ├── S2.2 flags: failure_reported_, handover_requested_, mission_completion_receipt_,
│   │         planner_recovery_pending_, px4_local_frame_aligned_  (navigation_mode.hpp:223-231)
│   ├── S2.3 Executor Hold handover {NONE, PENDING, IN_FLIGHT, CONFIRMED}  (2651-2686, retry 250 ms)
│   └── projection → NavigationModeStatus.external_mode_state (355-452)
├── S3 RUNTIME (navigation_runtime)
│   ├── S3.1 Localization {NOT_READY, READY(epoch)}   reset 1905-1970
│   ├── S3.2 Intent: PlanningIntentTransition {NONE, NEW_INTENT, HOT_RETARGET} (desired_planning_intent.hpp:17-21)
│   │         + pending goal (PendingGoalHandoffOwner) + foreign-mission latch (2200 / 2642)
│   ├── S3.3 EXECUTION LIFECYCLE: 5 facet độc lập (N8)
│   │   ├── Phase {InitialHold, TrackingMain, TrackingBackup, StoppedHold, Px4Hold}
│   │   ├── Recovery {InitialHold, TrackMain, TrackBackup, EmergencyBrake, StoppedRecovery, Px4Hold}
│   │   ├── Exposure {Unavailable, Available, Suspended, Failed}
│   │   ├── Safety {Nominal, SafetySuffix}
│   │   └── Restart {None, FromRest}
│   ├── S3.4 Planning: Worker {IDLE, ACTIVE(job), PENDING(latest)} × StartMode {StoppedMeasured, CommittedFuture}
│   │         × PlannerRenewalReason(7) → PlannerResultDisposition(6)
│   ├── S3.5 RetainedDecisionDisposition(8) · StaleCommandPublicationDisposition(3)
│   ├── S3.6 MissionProgress: INACTIVE | ACTIVE × gate(waypoint, request) × {CROSSING, CONTINUATION, STOP_CONFIRM, HOLD} → COMPLETE
│   └── S3.7 latches: lease-failure (8955 / 9763), skip_replan_once_, deferred_terminal_status_, completion witness
├── S4 PLANNER (internal to the solve)
│   ├── solve_stage_ {0 idle, 1 setup, 2 A*, 3 corridor(30+cg), 4 MINCO, 5 backup}: never reset to 0 (R1)
│   ├── candidate slot {EMPTY → STAGED → PROMOTED | DISCARDED} (837 / 876-908 / 910-918)
│   └── RET_CODE {SUCCESS, FAILED, NEW_TRAJ, EMER, NO_NEED, FINISH, OPT_FAILED, …}
├── S5 ESTIMATOR
│   ├── EstimatorStatus: WaitingForSensors → CollectingImu → InitializingImu → InitializingMap → Tracking ⇄ Degraded → Lost
│   │     (Resetting reachable only through reset(), which the node **never calls**)
│   ├── PropagatedOdometryStatus {WaitingForCorrection, Ready, ImuGap, TimestampRegression, MissingBracket,
│   │     InvalidState, QueueOverflow, MainEstimatorInvalid, StaleCorrection}
│   ├── InitialPriorStatus {NotRequired, Waiting, CandidateAvailable, Applied, FallbackApplied, Rejected, Closed}
│   └── public frame generation: **constant for the whole process lifetime**
├── S6 MAPPING
│   ├── MappingActor {READY, POISONED(terminal)}
│   ├── MappingWorker {NOT_STARTED, RUNNING, RESETTING, FATAL(terminal), STOPPING, JOINED}
│   ├── voxel {UNKNOWN, KNOWN_FREE, OCCUPIED} (two different unknown seeds, R6-02)
│   └── export {DEFERRED, FULL(7 reasons), PATCH}
├── S7 EV BRIDGE (LIO→PX4)
│   ├── continuity reason {NO_BASELINE, SOURCE_INVALID, GENERATION_*, *_FRAME_INVALID, TIMESTAMP_NOT_INCREASING,
│   │     DT_TOO_SMALL, DT_TOO_LARGE, THRESHOLD_OVERFLOW, WITHIN_THRESHOLD, JUMP_DETECTED}
│   ├── jump latch {CLEAR, LATCHED}: cleared only by a strictly newer generation
│   └── gate {READY, reason…}
├── S8 INGRESS BRIDGE (PX4→ROS) ─ TimestampEvent · ResetObservationStatus · frame/reset/time generations
└── S9 JUDGE RUN ─ runner lifecycle → scenario FSM (arm / takeoff / activate / mission / terminal) → verdict
```

## 1. Máy trạng thái recovery (owner: `navigation_execution`, `execution_recovery_state.hpp:44-94`)
| Từ | Event | Tới |
|---|---|---|
| InitialHold | MainCommitted / BackupActivated / EmergencyCommitted | TrackMain / TrackBackup / EmergencyBrake |
| TrackMain | BackupActivated / EmergencyCommitted / TerminalStopCompleted | TrackBackup / EmergencyBrake / StoppedRecovery |
| TrackBackup, EmergencyBrake | CertifiedStopObserved | StoppedRecovery |
| StoppedRecovery | MainCommitted | TrackMain. **Bỏ qua** BackupActivated/EmergencyCommitted, trong khi Phase lại chuyển sang TrackingBackup (R5-29) |
| bất kỳ | EmergencyCertificationFailed | Px4Hold |
| Px4Hold | bất kỳ | Px4Hold (absorbing) |

**Chỗ runtime kích hoạt event:**
- BackupActivated: 8488-8489, 9211
- EmergencyCommitted: 3339, 9190-9212
- CertifiedStopObserved: 4521-4522 (điều kiện speed ≤ 0.15)
- TerminalStopCompleted: 4963-4964
- EmergencyCertificationFailed: 8416-8418
- `failClosedLocked`: khoảng 25 chỗ

## 2. Estimator (owner: `fast_lio_pipeline.cpp`)
| Transition | Dòng |
|---|---|
| InitializingMap → Tracking (correction đầu tiên thành công) | 708-711 |
| InitializingMap → Lost (hết số lần thử đăng ký map ban đầu) | 689 |
| Tracking → Degraded: timeout wall / uncorrected / map guard | 254 / 1661 / 798 |
| Degraded → Lost: timeout wall / số lần uncorrected liên tiếp | 259 / 1659 |
| Degraded, Lost → Tracking (đủ số lần confirm). Bị **chặn vĩnh viễn** nếu `map_recovery_required_` | 724 |
| Discontinuity → Lost (gap dài / inflation fail) | 1209 / 1185 |

**Ngõ cụt không có đường thoát:**
- R4-01: IMU buffer bão hoà.
- R4-12: `state_time_` lệch trước lần prediction đầu tiên.
- R4-16: map guard latch.

## 3. Scenario judge (`external_mode_scenario._tick` 2497-2856)
Các bước theo thứ tự:
1. WAIT_SIM_CLOCK
2. DISCOVER_MODE
3. PRE_ARM_ACTIVATE
4. ARM
5. Cất cánh theo một trong hai nhánh:
   - AMSL: PREPARE_HOLD → SETTLE → NAV_TAKEOFF
   - local_ned: PRESTREAM → OFFBOARD → TAKEOFF_WAIT
6. STABLE_WINDOW
7. ACTIVATE_EXTERNAL
8. MISSION
9. Trạng thái kết thúc: {COMPLETE, FAILED_COMPONENT, PAUSED_SAFETY_STOP, ABORTED_OPERATOR, HOLD_HANDOVER_FAILED, *_TIMEOUT}

Toàn bộ máy trạng thái chạy trên cờ boolean, không có enum.

## 4. Bất biến chéo giữa các thành phần (cần cho kiến trúc đích)
Cột "Hiện tại" cho biết bất biến có được enforce hay không.

| # | Bất biến | Hiện tại |
|---|---|---|
| X1 | Adapter ACTIVE ⇒ `mode_activation_id` của mọi command trùng với activation hiện tại | **enforce** (session identity) |
| X2 | Runtime `Px4Hold` ⇒ trong ≤ 1 tick adapter nhận `REJECTED` hoặc stale và handover | enforce, gián tiếp qua lease 0.1 s |
| X3 | Estimator ∉ {Tracking} ⇒ EV bridge đóng gate, và adapter `failNavigation` | có enforce, nhưng **trễ tới 0.2 s**: health DEGRADED phát từ đường scan bị bỏ (R4-06) |
| X4 | Epoch LIO đổi ⇒ runtime, adapter, bridge cùng reset, và `reset_counter` PX4 tăng | **không đồng bộ**: `reset_counter` lấy từ diagnostics (R5-09); trong SITL epoch không bao giờ đổi |
| X5 | Heading của frame LIO = heading NED của PX4 | **không kiểm**; chỉ đúng vì spawn yaw = 0 (R5-17) |
| X6 | Recovery = StoppedRecovery ⇒ Phase ≠ TrackingBackup | **vi phạm được** (R5-29) |
| X7 | Planner chỉ được đọc/ghi từ thread planning | **vi phạm**: 4 thread cùng dùng (R3-17) |
| X8 | Một đại lượng anchor error cho mọi quyết định trong cùng một transaction | **vi phạm** (R3-11) |
| X9 | Mọi latch có đường thoát được định nghĩa rõ | **vi phạm** (R4-01, R4-12, R4-16, R6-04) |

Kiến trúc đích (A4) biến S3.3 thành một sum type, qua đó loại X6 theo cấu trúc. Các bất biến X3–X5 và X7–X9 cần quyết định bổ sung; xem KB-09.
