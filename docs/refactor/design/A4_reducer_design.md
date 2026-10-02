# A4 — Reducer tách từ `runCycle` / `planner_fsm` / `ExecutionAuthority`

Baseline `main @ 7e0b850`. Input: WP-A4-R1 (`decision_table.csv` 327 rule, `predicates.csv` 45, `state_vars.csv` 59, `lifecycle_enums.md`) và W3-B6 predicate coverage (adapter/bridge, 99 rule; xem `docs/refactor/W3-B6/REPORT.md`).

## 0. Trạng thái oracle: A4-R1 **chưa phải** bảng quyết định ngữ nghĩa

Kiến trúc sư tự đo lại `decision_table.csv` của A4-R1:

| Chỉ số | Giá trị |
|---|---|
| Rule có guard `payload.branch_line == N` | 194 / 327 |
| Rule có guard `payload.call_line == N` | 81 / 327 |
| Rule có guard `payload.predicate_line == N` | 45 / 327 |
| Rule có guard ngữ nghĩa thật (chỉ gồm 7 rule fault-injection) | 7 / 327 |
| Effect `NO_OP` | 239 / 327 |
| `covered_by_test = NONE` | 282 / 327 |

**Kết luận:**
- A4-R1 là một **index đầy đủ các exit site** của runtime: mọi return và mọi call site có side effect đều có ID. Checker PASS vì nó chỉ kiểm phủ.
- Guard lại là số dòng. Vì vậy bảng này không trả lời được câu hỏi "trong điều kiện nào thì effect gì", và **không dùng được làm oracle ngữ nghĩa cho P4**.

Ngược lại, W3-B6 đã ghi nhận 99/99 guard adapter/bridge dạng `pred:<name>()` và 94 predicate. Bằng chứng coverage này **dùng được** làm oracle cho P6.

**Quyết định (đi vào D4 của ADR-017):** không mở vòng R2 để viết văn xuôi cho 327 guard. Oracle của P4 gồm ba lớp:
1. **Exit-site index** (A4-R1). Mỗi `RT-*` phải được map tới một transition đích (§5) hoặc được đánh dấu `DELETED` kèm lý do. Checker CI kiểm việc này.
2. **Predicate thuần** (45 hàm trong `planner_fsm.hpp`). Đây mới là ngữ nghĩa thật; ở P4a chúng được **di chuyển nguyên văn** (cùng chữ ký, cùng unit test), không viết lại.
3. **Characterization trace** (WP mới P4-0, làm trước mọi thay đổi code runtime):
   - Instrument baseline để ghi chuỗi `(event, state-digest, effects)` tại các exit site `RT-*`.
   - Thu trace từ toàn bộ test hiện có và các run SITL của P0.2.
   - Reducer mới phải cho **cùng chuỗi effect** khi replay cùng chuỗi event.
   - SITL không deterministic, nên so sánh ở mức event→effect của reducer, không so sánh ở mức quỹ đạo.

## 1. Vấn đề của mô hình hiện tại

- **5 enum độc lập** trong `ExecutionLifecycleState` (`execution_lifecycle.hpp:52-60`): phase × recovery × exposure × safety × restart, tức 5×6×4×2×2 = 480 tổ hợp. Owner chỉ dựng khoảng 8 tổ hợp (`lifecycle_enums.md`). Ít nhất 2 tổ hợp "constructible nhưng chưa rõ reachability":
  - `kTrackingMain + kFromRest`
  - `kTrackingBackup + kTrackMain`
- `applyRecoveryEvent()` chỉ đổi `recovery`, trong khi exposure/phase/safety do các method khác đổi (`execution_authority.hpp:274-284`). Kết quả là một bước chuyển logic bị rải qua nhiều lời gọi, và có trạng thái trung gian nhìn thấy được giữa các lời gọi đó.
- Method `const` lại mutate state (`execution_authority.hpp:717-729`, `952-958`), đi qua các field `mutable` (`:1283-1305`).
- Lifecycle của `ExecutionAuthority` include ROS msg (`:17`).
- Quyết định nằm rải trong khoảng 5 000 dòng của `runCycle` (4010-7548), `validateRetainedCommand` (7564-8587) và `publishCommand` (8590-9908). Các hàm này gọi predicate thuần nhưng tự thực thi side effect ngay trong nhánh, lúc đang giữ khoá (A3 T7).

## 2. `ExecutionReducer` (package `nav_execution`, thuần C++, không ROS)

### 2.1 State: một sum type thay cho 5 enum
```cpp
struct Identity { uint64_t localization_epoch, goal_epoch, request_id, mode_activation_id;
                  uint64_t bundle_generation; WorldIdentity world; };      // nav_core_types
enum class Exposure : uint8_t { Available, Suspended };                   // chỉ có nghĩa khi đang có command
struct Idle        { };                                                   // = kInitialHold
struct TrackingMain{ Identity id; CommittedCommand active; std::optional<CommittedCommand> staged;
                     Exposure exposure; bool restart_from_rest; };        // restart chỉ hợp lệ trước solve kế
struct SafetySuffix{ Identity id; CommittedCommand active; enum Kind{Backup, Emergency} kind;
                     Exposure exposure; };
struct StoppedHold { Identity id; CommittedCommand hold; bool restart_from_rest; };
struct Px4Hold     { FailReason reason; TimestampNs at; };                // hấp thụ tới EpochReset / ModeReactivated
using ExecutionState = std::variant<Idle, TrackingMain, SafetySuffix, StoppedHold, Px4Hold>;
```

Cách map sang baseline:

| Baseline (phase, recovery, exposure, safety, restart) | Đích |
|---|---|
| (InitialHold, InitialHold, Unavailable, Nominal, None) | `Idle` |
| (TrackingMain, TrackMain, Available/Suspended, Nominal, None/FromRest) | `TrackingMain` |
| (TrackingBackup, TrackBackup, *, SafetySuffix, None) | `SafetySuffix{Backup}` |
| (TrackingBackup, EmergencyBrake, *, SafetySuffix, None) | `SafetySuffix{Emergency}` |
| (StoppedHold, StoppedRecovery, Available, Nominal, *) | `StoppedHold` |
| (Px4Hold, Px4Hold, Failed, Nominal, None) | `Px4Hold` |
| (TrackingBackup, TrackMain, …): frozen suffix đang sample MAIN trước thời điểm switch | `TrackingMain` + `active.role_schedule` chứa suffix. Role đang sample được **tính** từ schedule và thời gian, không lưu thành state (lifecycle_enums hàng 8) |

**Không biểu diễn được theo cấu trúc (N8):**
- Px4Hold mà vẫn Available;
- MAIN mang SafetySuffix;
- StoppedHold mang SafetySuffix;
- Idle mà Available;
- Suspended khi không có command.

Unit test đích tương ứng: `static_assert` và test cho `std::visit` cho thấy các kiểu này không có field để biểu diễn tổ hợp đó.

**One-shot emergency (HG-031)**, theo đúng code:
- `measuredStateEmergencyMayReplaceCommittedCommand` yêu cầu `recovery == kTrackMain` và `role != kEmergency` (`planner_fsm.hpp:501-531`).
- Trong mô hình đích, điều này tương đương: chỉ chuyển được `TrackingMain → SafetySuffix{Emergency}`. Từ `SafetySuffix{*}` không có transition nào sang emergency; chỉ `StopObserved` mới thoát ra `StoppedHold` (`execution_recovery_state.hpp` `transitionExecutionRecovery`).
- Ghi chú: ledger legacy (`runtime_safety_legacy_full.md:3392`, đã gỡ ở baseline 2026-10-01; HG-031 còn trong `runtime_safety_current.md`) viết "MAIN/BACKUP", còn code chỉ cho từ MAIN. Reducer encode **theo code**. Ledger được làm rõ bằng một commit docs riêng (N10).

### 2.2 Event
| Event | Nguồn | Payload (data) |
|---|---|---|
| `EpochReset` | ingress (`/lio/health` hoặc discontinuity) | epoch, reason |
| `ModeStatusChanged` | ingress `/px4_adapter/mode_status` | activation_id, active, receipt |
| `CandidateCertified` | planning lane | bundle, CertificateRecord, identity |
| `CertificationFailed` | planning lane | identity, failure codes |
| `RecertifyResult` | lane | identity, per-command {ok, suspend, failure codes}, world |
| `EmergencyPrepared` / `EmergencyFailed` | lane | identity, bundle, record / reason |
| `CommandTick` | ROS timer 50 Hz | now (ROS), state lease |
| `StateSample` | ingress odom | PVAJ, epoch, generation, acceleration/jerk_estimated flags |
| `WorldTemporal` | mapping lane | freshness assessment |
| `StopObserved` | tính từ `StateSample` (speed ≤ `stationary_speed_mps` của profile) | — |
| `AdmissionReceipt` | ingress (adapter) | command id, accepted/rejected, reason |
| `DeadlineMissed` | lane watchdog | job key |

### 2.3 Effect: dùng vocabulary chung (`SCHEMA_decision_table.md`)
`PUBLISH_COMMAND(sample)`, `SUSPEND_COMMAND`, `RESUME_COMMAND`, `COMMIT_CANDIDATE`, `DISCARD_CANDIDATE(reason)`, `RECERTIFY_RETAINED`, `EMERGENCY_BRAKE_PREPARE(boundary, terminal_alt)`, `EMERGENCY_BRAKE_COMMIT`, `FAIL_CLOSED(reason)`, `REQUEST_PX4_HOLD`, `CANCEL_SOLVE(key)`, `EMIT_EVIDENCE(msg)`, `NO_OP`.

Effect chỉ là dữ liệu; shell thực thi chúng **sau** khi reducer đã trả về. Reducer không gọi ROS và không nhận callback.

### 2.4 Bảng transition đích (rút gọn; bảng đầy đủ nằm trong CSV của WP P4-1)
| From | Event | Guard (predicate thuần, di chuyển nguyên văn) | To | Effects |
|---|---|---|---|---|
| Idle / StoppedHold | CandidateCertified(MAIN) | identity hiện hành ∧ commit check O(1) (A5 §1.4) | TrackingMain | COMMIT_CANDIDATE, EMIT_EVIDENCE |
| TrackingMain | CandidateCertified(MAIN) | `supersedingBundleMayRemainAvailable`, activation ≥ now+commit_guard | TrackingMain (staged) | COMMIT_CANDIDATE |
| * | CandidateCertified / CertificationFailed | identity **không** hiện hành | (giữ nguyên) | DISCARD_CANDIDATE(stale) |
| TrackingMain | RecertifyResult(active fail) | `committedSafetySuffixIsUsable` ∧ `retainedSafetyTransitionMayActivateBackup` | SafetySuffix{Backup} | EMIT_EVIDENCE |
| TrackingMain | RecertifyResult / CommandTick | `measuredStateEmergencyMayReplaceCommittedCommand` | TrackingMain (chờ emergency) | EMERGENCY_BRAKE_PREPARE(`makeMeasuredEmergencyBoundary`, `plannerEmergencyTerminalAltitude`) |
| TrackingMain | EmergencyPrepared | identity hiện hành | SafetySuffix{Emergency} | EMERGENCY_BRAKE_COMMIT, PUBLISH_COMMAND |
| TrackingMain | EmergencyFailed / DeadlineMissed | — | Px4Hold | FAIL_CLOSED, REQUEST_PX4_HOLD |
| SafetySuffix | StopObserved | certified stop (HG-035) | StoppedHold | EMIT_EVIDENCE |
| SafetySuffix | RecertifyResult(fail) | — | Px4Hold | FAIL_CLOSED, REQUEST_PX4_HOLD |
| TrackingMain / SafetySuffix | WorldTemporal(stale) | `WorldTemporalAssessment` | (cùng state, exposure = Suspended) | SUSPEND_COMMAND |
| (Suspended) | WorldTemporal(fresh) | `worldFreshnessSuspendedCommandMayResume` | exposure = Available | RESUME_COMMAND |
| StoppedHold | CommandTick | `stoppedPlanningTimeoutMayFailClosed` | Px4Hold | FAIL_CLOSED |
| TrackingMain | CommandTick | `commandAnchorRecoveryDue` ∧ ¬`terminalStopMayDeferAnchorRecovery` | (đi qua nhánh emergency) | … |
| Tracking* | CommandTick | lease còn ∧ exposure = Available | = | PUBLISH_COMMAND(sample) |
| Tracking* | CommandTick | `classifyStaleCommandPublication` ≠ publish | = hoặc Px4Hold | theo disposition |
| any ≠ Px4Hold | ModeStatusChanged(inactive) hoặc activation khác | — | Idle | CANCEL_SOLVE(*), EMIT_EVIDENCE |
| any | EpochReset | — | Idle (hoặc Px4Hold nếu reset bị từ chối) | CANCEL_SOLVE(*), RESET_LANE, EMIT_EVIDENCE |
| Px4Hold | bất kỳ, trừ EpochReset và ModeReactivated | — | Px4Hold | NO_OP (evidence rejection) |

### 2.5 Timestamp
- Mọi thời gian trong state và event là `TimestampNs` (int64 ns, có domain).
- Baseline đang truyền `now().seconds()` dạng double vào `commitEmergencyBrake` (`navigation_runtime_node.cpp:8143` và vùng xung quanh), tức mất độ phân giải ns khi so sánh activation.
- Đích: chỉ còn int ns. Việc đổi kiểu thuần tuý là refactor; nếu có so sánh nào đổi kết quả, trace P4-0 sẽ phát hiện.

## 3. `PlanningPolicyReducer` (package `nav_planning_policy`)
- **State:** pending goal handoff (thay `PendingGoalHandoffOwner`), renewal state, hot-retarget state, watchdog, in-flight `JobKey`.
- **Event:** GoalAdmitted, PlanningTick, CandidateCertified/Failed (để phân loại renewal), ExecutionStateChanged (read-only), MissionAdvanced.
- **Effect:** `SUBMIT_SOLVE(PlanningRequest)`, `CANCEL_SOLVE`, `EMIT_EVIDENCE`.
- **Quyền hạn:** planning policy **không** commit và không fail-closed. Nó chỉ quyết định *có solve hay không, solve gì*. Quyền commit và fail-closed thuộc duy nhất execution.

## 4. `MissionReducer` (package `nav_mission`)
- **State:** waypoint index, acceptance tracker, completion, `safety_profile_hash`.
- **Event:** StateSample, TerminalStopCompleted (từ execution), AdmissionReceipt, ModeStatusChanged.
- **Effect:** `ADVANCE_WAYPOINT`, `COMPLETE_MISSION`, `PUBLISH_MISSION_PROGRESS`.
- **Ranh giới:** mission không đọc world model; đây là lời giải cho V2. Trong A1, `MissionController` là DELETE, và chỉ còn một đường mission (D6).

## 5. Map 45 hàm `planner_fsm.hpp` sang owner đích
| Owner đích | Hàm (dòng trong `planner_fsm.hpp`) |
|---|---|
| `nav_planning_policy` (12) | `PendingGoalHandoffOwner::{enqueueGoal:30, consumeGoal:53, goalMatchesStatus:61, clearGoal:70, clearIfCurrent:75}`, `canHotRetargetAtWaypointTransition:102`, `pendingGoalTerminalStatusMayClear:179`, `clearHotGoalTransitionAfterCommit:193`, `hotRetargetUsesCommittedFutureState:204`, `plannerTerminalStopBrakingDistanceM:347`, `plannerTerminalStopApproachDue:366`, `classifyPlannerRenewal:382` |
| `nav_planning_policy` (3, cần input từ execution) | `watchdogTimeoutMayRetainSafetySuffix:144`, `watchdogTimeoutMayRetainStoppedRecoveryHold:156`, `classifyPlannerResult:703` |
| `nav_execution` (27) | `passThroughTerminalAckMayRetainCommand:133`, `stoppedHoldCommandRole:166`, `commandAnchorRecoveryDue:252`, `terminalStopMayDeferAnchorRecovery:271`, `terminalStopCompletionObserved:291`, `terminalStopEndpointContractValid:309`, `terminalSuccessorHoldMayTransfer:328`, `retainedValidationTransition:465`, `terminalMainHasIndeterminatePreStartPressure:473`, `measuredStateEmergencyMayReplaceCommittedCommand:501`, `makeMeasuredEmergencyBoundary:537`, `plannerEmergencyTerminalAltitude:551`, `terminalHoldIsPending:572`, `committedTerminalBundleHoldIsPending:584`, `backupStopNeedsMeasuredRestart:597`, `stoppedPlanningTimeoutMayFailClosed:623`, `worldFreshnessSuspendedCommandMayResume:636`, `supersedingBundleMayRemainAvailable:661`, `classifyStaleCommandPublication:692`, `committedSafetySuffixIsUsable:742`, `retainedSafetyTransitionMayActivateBackup:771`, `retainedCommandMatchesExecutionIdentity:782`, `retainedCommandTrackingLimit:801`, `assessTimeAlignedRetainedTracking:825`, `assessPhaseExecutionCertificate:874`, `phaseExecutionBridgeMayPreserveMain:959`, `projectedRetainedAnchorErrorUpperBound:984` |
| `nav_mission` (2) | `completedPassThroughRequiresContinuation:608`, `waypointBehaviorContractValid:618` |
| `sitl_harness` (1) | `ordinaryRenewalFailureInjectionMayArm:448` |

Tổng: 12 + 3 + 27 + 2 + 1 = 45 hàm.

Luật: ở P4a mỗi hàm được **di chuyển nguyên văn**, cùng chữ ký và cùng unit test. Nếu cần sửa ngữ nghĩa thì làm thành commit behavior riêng, có ledger entry.

Shim `navigation_runtime/execution_recovery_state.hpp` hiện chỉ include bản ở `navigation_execution`; xoá ở P4.

## 6. Map exit site `RT-*` sang transition (luật cho checker P4-1)
| Nhóm `RT-*` (theo event trong A4-R1) | Số rule | Đích |
|---|---|---|
| `RUNTIME_RUN_CYCLE` | 60 | Tách ra: planning policy (schedule, renewal), execution (recert, emergency), shell (lấy snapshot). Mỗi rule map tới 1 transition, hoặc `SHELL_PLUMBING` nếu chỉ là lấy dữ liệu hay lock |
| `RUNTIME_VALIDATE_RETAINED_COMMAND` | 15 | execution: nhóm RecertifyResult / emergency |
| `RUNTIME_PUBLISH_COMMAND` | 41 | execution: CommandTick |
| `RUNTIME_COMMIT_PLANNER_CANDIDATE` | 41 | execution: CandidateCertified (commit check), cộng certifier nếu rule kiểm certificate |
| `PLANNING_CYCLE_SCHEDULE` | 7 | planning policy: PlanningTick → SUBMIT_SOLVE |
| `PLANNING_RESULT` | 40 | planning policy `classifyPlannerResult` → execution |
| `PREDICATE_EVALUATION` | 45 | §5 (di chuyển nguyên văn) |
| `INGRESS_*` | 40 | IngressGate (identity/epoch/freshness) → event |
| `MISSION_*` | 14 | MissionReducer |
| `HEADING_REBIND_*` | 15 | fast lane job + execution commit |
| `FAULT_INJECTION_*` | 7 | `sitl_harness` (ADR rule 8) |
| `WORLD_FRESHNESS_SUSPEND` | 2 | execution WorldTemporal |
| **Tổng** | **327** | |

Checker P4-1 kiểm ba điều:
- mọi `RT-*` đều có đúng một cột `target_transition` (hoặc `DELETED`/`SHELL_PLUMBING` kèm lý do);
- mọi transition trong bảng đích được ít nhất một `RT-*` trỏ tới, hoặc được đánh dấu `NEW` kèm ledger;
- tập effect trong trace P4-0 bằng tập effect được bảng đích sinh ra.

## 7. Test harness bắt buộc
- **Reducer test:** bảng `(state, event) → (state', effects)`, sinh cho mọi dòng của bảng đích. Không cần ROS.
- **Replay test:** trace P4-0 được đưa vào reducer; so sánh chuỗi effect.
- 18 rule P0 (FAIL_CLOSED/EMERGENCY) hiện **không có test ở node path**: `RT-200, 201, 204, 220, 222, 223, 224, 225, 229, 231, 233, 236, 240, 242, 259, 266, 267, 271` (A4-R1 `coverage_gaps.md`). Mỗi rule phải có characterization trace ở P4-0 và reducer test **trước** khi logic được move.
- Lỗi nhỏ trong A4-R1: `coverage_gaps.md` liệt kê `RT-300`, nhưng `decision_table.csv` không có dòng này (327 dòng, nhảy từ RT-299 sang RT-301). Checker P4-1 phải bắt được loại lệch này.
