# A1 — Bản đồ sở hữu module: hiện tại → đích

Baseline `main @ 7e0b850`. Tài liệu này ở **mức kiến trúc**: chỉ quyết định cho các symbol **đổi module** (MOVE / SPLIT / DELETE). Code nằm yên trong module của nó thì chỉ ghi ở mức package. Inventory đủ 801 symbol là của WP-A1-R1 (`docs/refactor/WP-A1/class_inventory.csv`, đã kiểm lại: coverage PASS, semantic checker PASS, oracle `MissionController → DELETE` đúng). Khi file đó mâu thuẫn với tài liệu này, **tài liệu này thắng**, và file inventory phải được sửa lại.

## 1. Mức package: giữ nguyên

| Package hiện tại | Module đích | Ghi chú |
|---|---|---|
| `fast_lio_core` (8.2k) | `lio_core` | KEEP toàn bộ; chỉ bổ sung discontinuity declaration (§3.7) |
| `ikfom_vendor`, `ikd_tree_vendor` | vendor | KEEP |
| `rog_map_vendor` | vendor | KEEP, trừ `navigation_math/` phải rời vendor (§3.6) |
| `navigation_planning_backend`: `path_search/`, `traj_opt/minco*`, `ciri`, `corridor_generator`, `deterministic_nominal_seed`, `corridor_bezier_seed`, `route_backbone`, `route_yaw_reference`, `evidence_speed_governor` | `nav_planner` | Nằm yên; namespace hoá include root (V3) |
| `navigation_mapping`: `MappingActor`, `MappingWorker`, `WorldSnapshotStore`, `MappingWorldSnapshot`, `ObservationAccounting` | `nav_world` | Nằm yên. `MappingWorker` trở thành *mapping lane* (ADR-013) mà không đổi logic |
| `fast_lio_ros` (node, adapters, output publisher, propagated worker) | `lio_node` | Nằm yên, trừ việc publish generation typed (§3.7) |

## 2. Quy tắc gán (đã dùng cho mọi quyết định dưới đây)

1. Dữ liệu được ≥ 2 module core cùng dùng → tier A (contract), và **chỉ là dữ liệu**.
2. Code quyết định *có được thực thi không* (validator, certificate) → `nav_certifier`, không bao giờ nằm trong `nav_planner`.
3. Code quyết định *chuyển trạng thái* → reducer của đúng khái niệm đó (`nav_execution` / `nav_mission` / `nav_planning_policy`).
4. Code chỉ phục vụ fault injection, experiment hay probe → `sitl_harness` (plugin), không link vào binary product.
5. Kiểu ROS msg không được xuất hiện trong tier A/B. Chuyển đổi msg ↔ domain nằm ở shell.

## 3. Các symbol đổi module

### 3.1 `navigation_runtime` (14.7k) — SPLIT hoàn toàn
| Symbol (file) | Đích | Action | Lý do / cách tách |
|---|---|---|---|
| `NavigationRuntimeNode` (`navigation_runtime_node.hpp/.cpp`, 38 method, 157 field) | `nav_core_node` + các reducer | SPLIT | Shell giữ: subscription, publisher, timer, param load, chuyển đổi msg ↔ domain. Toàn bộ quyết định chuyển sang reducer (A4). `runCycle` (CCN 776) bị xoá, thay bằng các handler theo event |
| `MappingTelemetry`, `MappingLifecycleObserver`, `PendingRegisteredScan` | `nav_core_node` (world service adapter) | MOVE | Glue giữa ingress và mapping lane |
| `PlanningWorker` (`planning_worker.hpp`) | `nav_core_node` (planning lane) | MOVE | Giữ semantics latest-only và cancellation. Thêm bước certify trên cùng lane (ADR-014) |
| `HeadingRebindWorker` | `nav_core_node` (fast lane) | MERGE | Gộp vào fast lane như một loại job |
| `DesiredPlanningIntent`, `PlanningIntentTransition` | `nav_planning_policy` | MOVE | State của reducer planning policy |
| `PendingGoalHandoffOwner` (`planner_fsm.hpp:23`) | `nav_planning_policy` | MOVE | Goal queue hoặc handoff |
| 40 hàm trong `planner_fsm.hpp` | chia 3 reducer | SPLIT | Bảng phân bổ ở A4 §4 |
| `PlanningSupervisor`, `PlanningPriority` | `nav_planning_policy` | MOVE | Ưu tiên / chọn request |
| `BaselineRefinementContext/Opportunity` | `nav_planning_policy` | MOVE | |
| `CertifiedMainContinuationWindow/BoundaryFacts` | `nav_execution` | MOVE | Quyết định giữ MAIN qua waypoint |
| `TrajectoryCompletionWitness` | `nav_mission` | MOVE | Mission completion không được suy ra từ planner (invariant 5) |
| `MissionProgress` + các struct liên quan (`mission_progress.hpp`) | `nav_mission` | MOVE | Đã là writer duy nhất; giữ nguyên logic |
| `mission_goal`, `mission_dynamics` | `nav_mission` | MOVE | |
| `WorldTemporalAssessment` | `nav_execution` | MOVE | Quyết định suspend/resume khi world stale |
| `KinematicDerivativeEstimator` | `nav_core_node` (ingress) | MOVE | Ingress gate cho state |
| `ExecutionTraceSnapshot/Store`, `commit_trace`, `RetainedDecisionObservation` và accounting | `nav_core_node` (evidence emitter) | SPLIT | Dữ liệu đi vào `nav_evidence_msgs` (A5 §4); không còn là KeyValue |
| `PathRelativeTrackingResult`, `ExperimentalTrackingResult` | `nav_execution` (policy) + `sitl_harness` (experiment) | SPLIT | Phần product là tracking certificate (HG-007); phần experiment đi vào plugin |
| `ExactOptimizationFailureInjection`, `SameIdentityRenewalInjectionController`, 12 param `inject_*` | `sitl_harness` | MOVE | Quy tắc 4 |
| `runtime_boundaries.hpp`: `GoalTransitionKind`, `PlannerSolveFailureWitness`, `PlannerSolveActivityScope` | tách theo từng nội dung: goal → `nav_planning_policy`; witness → evidence | SPLIT | File đang gộp nhiều nội dung không liên quan |
| `execution_recovery_state.hpp`, `execution_lifecycle_view.hpp` (shim) | — | DELETE | Chỉ là re-export (WP-P0.3 I4) |

### 3.2 `navigation_execution` (1.8k)
| Symbol | Đích | Action | Lý do |
|---|---|---|---|
| `ExecutionAuthority` (`execution_authority.hpp`, 1 309 dòng) | `nav_execution` | REWRITE thành reducer | Logic định danh và commit được giữ. Sửa: (a) 5 enum lifecycle trở thành một sum type (A4 §2); (b) các method `const` đang ghi state `mutable`, ví dụ `invalidateIfCurrent(...) const` (`:717-729`) và `failClosedLifecycleLocked() const` (`:952-958`), phải trở thành chuyển trạng thái tường minh; (c) bỏ `navigation_contracts::msg::NavigationGoal` khỏi core (`:17, :48-49`), thay bằng domain `GoalIdentity`/`WaypointGoal` |
| `ExecutionLifecycleState` + `ExecutionPhase`/`Exposure`/`SafetyOwnership`/`RestartRequest` | `nav_execution` | REWRITE | Gộp thành `ExecutionState` (A4 §2) |
| `ExecutionRecoveryState` + `transitionExecutionRecovery` | `nav_execution` | KEEP ý tưởng, MERGE vào reducer | Hàm chuyển trạng thái thuần là mẫu đúng; mở rộng ra |
| `CommandSampler` | `nav_execution` | KEEP | |
| `ExecutionStateStore`, `ExecutionStateLease` | `nav_core_node` (ingress) | MOVE | Là ingress state, không phải quyết định execution |
| `ExecutionStateFailureLatch` | `nav_execution` | MERGE vào reducer | Latch trở thành state của reducer, nên không cần mutex riêng |
| `candidateMatchesAnchor` + tolerance (`execution_anchor.hpp:47-52`) | `nav_certifier` | MOVE | Là một certificate: anchor continuity. Tolerance lấy từ `nav_safety_profile`; xoá bản trùng ở `candidate_bundle.hpp:398-403` |
| `timestamp_freshness.hpp` | `nav_core_types` | MOVE | |

### 3.3 `navigation_planning` (contract, 1.7k)
| Symbol | Đích | Action | Lý do |
|---|---|---|---|
| `PlanningRequest`, `PlanningKey`, `GoalIdentity`, `PlanningHistory`, `PlanningOutcome`, `PlanningFailure*`, `KinematicState`, `ExecutionAnchor`, `RouteBoundary*` | `nav_plan_contract` | MOVE | |
| `CandidateBundle` | `nav_plan_contract` | REWRITE | Bỏ `world_validator`, `evaluator` (`std::function`). Bundle mang dữ liệu đa thức (A5 §2). Thời gian chỉ dùng int ns; bỏ `start_wall_time_s`/`duration_s` kiểu double |
| `CompleteBundleCertificates` (4 bool) | `nav_plan_contract` → `CertificateRecord` | REWRITE | Bốn bool này hiện được gán hằng `true` lúc export (`planner.cpp:1057-1059`) nên không mang bằng chứng gì. Thay bằng record do certifier tạo ra (A5 §1.4) |
| `TrajectoryValidationResult` | `nav_plan_contract` (dữ liệu kết quả của certifier) | MOVE | |
| `PlanningTimingContract` (`planning_timing.hpp`) | `nav_safety_profile` | MOVE | Mọi giá trị timing vào profile (A5 §3) |
| `planning_limits.hpp` (`DynamicLimits`, `VehicleControlEnvelope`) | kiểu → `nav_plan_contract`; giá trị → `nav_safety_profile` | SPLIT | |
| `candidate_admission.hpp` | `nav_certifier` | MOVE | Là quyết định admission |
| `planner_diagnostics.hpp` (319 dòng) | `nav_evidence_msgs` + `nav_planner` internal | SPLIT | |

### 3.4 `navigation_planning_backend` — phần rời khỏi `nav_planner`
| Symbol | Đích | Action |
|---|---|---|
| `trajectory_world_validator.hpp` (`validateExecutableCandidate`, `certificateTubeIsSafe`, `locatePieceForSweep`, `SweptValidationResult`, `CertificateTubeFailure`) | `nav_certifier` (logic) + `nav_plan_contract` (kiểu kết quả) | MOVE |
| `corridor_plane_validation.hpp` | `nav_certifier` | MOVE |
| `traj_opt/trajectory_dynamics.hpp` (`evaluateTrajectoryDynamics`, `trajectorySatisfiesFlatnessEnvelope`) | `nav_certifier` | MOVE (planner vẫn được gọi hàm này qua certifier API để tự lọc, nhưng kết quả gọi từ planner không có authority) |
| `route_regression_certificate.hpp` (`certifyMainRouteRegression`) | `nav_certifier` | MOVE |
| `backup_braking.hpp`: phần *certify stop* (`StopReachability`, `StopKinematicEnvelope`, `StopFailureReason`) | `nav_certifier`; phần *synthesis* (`BackupBrakingSeed`, sinh đa thức dừng) giữ ở `nav_planner` | SPLIT |
| `data_structure/base/piece.h`, `trajectory.h` (`Piece`, `Trajectory`) | `nav_plan_contract` (`PolynomialPiece`, `PiecewisePolynomial`, dữ liệu thuần) | MOVE. Giải cycle `nav_planner ↔ nav_certifier` mà WP-A1 đã phát hiện |
| `CmdTraj` (`data_structure/cmd_traj.h`), private command history | `nav_execution` sở hữu lịch sử; planner chỉ nhận `PlanningHistory` trong request | SPLIT |
| `PlannerFacade`: 9 API ambient | — | DELETE (WP-P0.3 I2) |
| `Planner` (69 field, 5 mutex) | `nav_planner` | REWRITE ở P5: `SolveContext` theo request + các stage thuần |
| `planner_runtime_context/` | `nav_planner` | KEEP (logger/clock injection) |

### 3.5 `navigation_world_model` (0.8k) và `navigation_mission` (1.0k)
| Symbol | Đích | Action | Lý do |
|---|---|---|---|
| `WorldModelView` (interface), `WorldSnapshotIdentity`, `CellState`, `UnknownPolicy`, `AxisAlignedBox`, `WorldGeometry`, `CurrentBodySupport` (data) | `nav_world_contract` | MOVE | |
| `directionalSupportToLocalBoundary`, `CurrentBodySupport::contains` (logic trong `world_model_view.hpp:135-345`) | `nav_certifier` | MOVE | Là logic certificate chứ không phải contract |
| `continuous_clearance.hpp` (`observedOccupiedTubeIsClear`) | `nav_certifier` | MOVE | |
| `goal_contract.hpp` (`kGoalCompletionToleranceM` …) | giá trị → `nav_safety_profile`; kiểu → `nav_mission_contract` | SPLIT | Gỡ V2 |
| `WorldCommitAuthorizer`, `WorldValidationLease` | `nav_world` | MOVE | Lease của snapshot store |
| `Mission`, `MissionWaypoint`, `ImmutableRouteSnapshot`, `RouteSegment`, `RouteProjection` | `nav_mission_contract` | MOVE; bỏ include `world_model_view.hpp` (`mission.hpp:10`) | Gỡ V2 |
| `RouteProgress` (logic tiến độ) | `nav_mission` | MOVE | |

### 3.6 `navigation_contracts` (headers có logic), `rog_map_vendor/navigation_math`
| Symbol | Đích | Action |
|---|---|---|
| `execution_state_freshness.hpp` (`evaluateExecutionStateFreshness`) | `nav_core_types` (hàm thuần) | MOVE |
| `navigation_command_contract.hpp` | `nav_core_node` (decode/validate msg) + `px4_setpoint_core` (SetpointGuard) | SPLIT |
| `command_safety_contract.hpp` (0.75 m, HG-007) | `nav_safety_profile` | MOVE |
| `tracking_experiment.hpp` (policy của experiment `relaxed`) | `sitl_harness` | MOVE (quy tắc 4) |
| `odometry_validity.hpp` | `nav_core_types` | MOVE |
| `rog_map_vendor/include/navigation_math/*` | `nav_core_types` (math) | MOVE (V3) |

### 3.7 Estimator, bridge, adapter
| Symbol | Đích | Action | Lý do |
|---|---|---|---|
| `LioPublicFrameGeneration`, `PublicFrameEvent` (`fast_lio_ros/lio_public_frame_generation.hpp`) | `lio_core` (logic) + typed msg field | MOVE | Estimator đã có khái niệm discontinuity, nhưng chỉ publish qua chuỗi diagnostics (`ros_output_publisher.cpp:781`). Đây là nguyên nhân gốc của V5 |
| `GeometricJumpContinuity*`, `GeometricJumpLatch`, `ExternalOdometryGate*`, `TimestampConverter`, `frame_generation_policy.hpp`, `ExternalOdometryFrame` | `odom_bridge_core` | MOVE; sửa continuity ở P6 (R-02) |
| `Px4ExternalOdometryBridgeNode` | `odom_bridge_node` | MOVE; chỉ đọc `/lio/health` + generation trong `PropagatedOdometry` |
| `Px4OdometryBridgeNode` (reverse bridge, chỉ runner dùng) | `sitl_harness` | MOVE |
| `NavigationMode`, `NavigationModeExecutor` (`navigation_mode.hpp`, node 2 703 dòng) | `px4_adapter_node` (shell) + `px4_setpoint_core` | SPLIT: `updateSetpoint` (CCN 119) thành setpoint pipeline typed |
| `CommandAdmissionAssessment`, `CommandAcceptanceGate`, `tracking_envelope`, `velocity_only_continuity`, `px4_tracking_adapter`, `certified_command_handoff`, `mission_command_identity`, `planner_recovery`, `local_frame_alignment` | `px4_setpoint_core` | MOVE |
| `px4_input_trace`, `reject_provenance` | evidence (`nav_evidence_msgs`) | SPLIT |
| `MissionController` (+ `_contract` lib) | — | DELETE (WP-P0.3 I1) |

## 4. Đồ thị dependency đích (phải không có cycle)

```
nav_core_types  ◄── nav_safety_profile
     ▲                   ▲
nav_world_contract   nav_mission_contract   nav_plan_contract   nav_evidence_msgs
     ▲      ▲              ▲      ▲             ▲    ▲    ▲
  nav_world  nav_certifier ─────────────────────┘    │    │
             ▲   nav_mission  nav_planning_policy    │    │
             │        ▲             ▲           nav_planner
        nav_execution ┘─────────────┘                 ▲
             ▲                                        │
        nav_core_node ────────────────────────────────┘ (chỉ qua interface IPlanner)
```

Quy tắc CMake:
- `nav_certifier` KHÔNG được link `nav_planner`.
- `nav_execution` KHÔNG được link `nav_planner` hay `nav_world`; nó chỉ nhận `CertificateRecord` và identity.
- `nav_core_node` là nơi duy nhất thấy mọi module core.

Có một CMake test (P1) đọc `package.xml` / `target_link_libraries` và FAIL khi có cạnh ngược chiều.
