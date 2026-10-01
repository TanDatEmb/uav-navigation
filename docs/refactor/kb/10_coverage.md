# KB-10: File và đối tượng đã duyệt (ma trận phủ)

**Dữ liệu đầy đủ:** `data/coverage_matrix.csv`, mỗi dòng là một file của repo trên `7e0b850` (snapshot trước baseline 2026-10-01; các dòng trỏ tới `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, safety index/archive và `docs/runtime_validation.md` mô tả file đã gỡ). Không tính `runtime_evidence/`, `artifacts/`, `.artifacts/` (evidence lịch sử) và `src/external/livox_ros_driver2` (driver bên ngoài).

Các cột:
- `area`: vùng review. R1–R7 là reviewer theo vùng; `KTS` là kiến trúc sư; `-` là ngoài phạm vi.
- `depth`:
  - FULL: đọc toàn bộ.
  - SKIM: đọc để biết cái gì có test và cái gì không.
  - INTERFACE: chỉ xem cách dự án dùng code vendor.
  - SKIPPED: có lý do ghi trong `notes`.
  - NOT_IN_SCOPE: ngoài phạm vi, có lý do.
- `questions`: các câu Q1–Q12 thực sự đã được trả lời cho file đó.

Q1–Q12 là 12 câu hỏi của checklist, dùng chung cho mọi vùng:

| Q | Câu hỏi |
|---|---|
| Q1 | Vai trò trong kiến trúc |
| Q2 | Thuộc lớp nào |
| Q3 | Process, thread, timer |
| Q4 | Luồng |
| Q5 | Máy trạng thái |
| Q6 | Hành vi theo nhánh |
| Q7 | Nhiệm vụ và nhu cầu thông tin |
| Q8 | Điểm nghẽn |
| Q9 | Xung đột |
| Q10 | Toán |
| Q11 | Hiệu năng |
| Q12 | Logic và quy tắc lập trình |

## Tổng hợp
| Mức độ phủ | Số file |
|---|---|
| FULL (agent) | 405 |
| SKIM (agent; chủ yếu test và fixture) | 228 |
| INTERFACE (vendor: IKFoM, ikd-tree, quickhull, sdlp, sdqp, mvie, root_finder, ESDF/frontier) | 31 |
| SKIPPED (2 file license, 1 benchmark offline không nằm trên đường ra verdict) | 3 |
| KTS: kiến trúc sư tự đọc (script PX4 SITL, 4 tài liệu kiến trúc, ledger, `.gitmodules`) | 8 |
| NOT_IN_SCOPE (docs lịch sử, SDF world/model, asset, submodule, metadata) | 96 |
| **Tổng** | **771** |

## Phủ theo vùng
| Vùng | Phạm vi | File | LOC |
|---|---|---|---|
| R1 | planner_core | 36 | 16.5k |
| R2 | backend ngoài planner_core (traj_opt, facade, test) | 70 | 34.8k |
| R3 | navigation_runtime | 56 | 24.2k |
| R4 | estimation (+vendor ở mức interface) | 186 | 29.4k |
| R5 | PX4 adapter, bridges, execution, contracts, mission, common | 126 | 23.6k |
| R6 | mapping, world model, rog_map_vendor | 58 | 13.4k |
| R7 | tools/runtime, guards, config, launch, simulation, CI | 135 | 47.7k |

### Độ sâu theo vùng (số file / LOC)
| Vùng | FULL | SKIM | INTERFACE | SKIPPED |
|---|---|---|---|---|
| R1 | 33 / 15.5k | 2 / 0.8k | 1 / 0.2k | — |
| R2 | 44 / 13.6k | 15 / 16.7k (test) | 11 / 4.5k (vendor) | — |
| R3 | 34 / 14.7k (toàn bộ node 9.9k) | 22 / 9.5k (test) | — | — |
| R4 | 111 / 13.0k | 59 / 8.6k | 14 / 7.1k (IKFoM, ikd-tree) | 2 (license) |
| R5 | 76 / 11.7k | 50 / 11.9k | — | — |
| R6 | 39 / 9.3k | 14 / 3.2k | 5 / 0.9k (ESDF/frontier) | — |
| R7 | 68 / 19.0k | 66 / 28.4k | — | 1 (benchmark) |
| KTS | 5 / 0.4k | 3 / 23.3k (ledger) | — | — |

### Số file đã trả lời từng câu hỏi (theo tag trong CSV)
| Vùng | Q1 | Q2 | Q3 | Q4 | Q5 | Q6 | Q7 | Q8 | Q9 | Q10 | Q11 | Q12 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| R1 | 5 | — | 5 | 4 | 5 | 9 | 7 | 3 | 8 | 26 | 7 | 24 |
| R2 | 33 | 11 | 3 | 3 | 4 | 11 | 7 | 1 | 9 | 26 | 8 | 30 |
| R3 | 11 | 2 | 9 | 4 | 12 | 9 | 6 | 3 | 7 | 15 | 4 | 37 |
| R4 | 82 | 18 | 21 | 37 | 26 | 27 | 46 | 22 | 35 | 73 | 35 | 83 |
| R5 | 13 | 15 | 11 | 3 | 24 | 33 | 39 | 2 | 12 | 49 | 9 | 30 |
| R6 | 19 | 2 | 6 | 7 | 10 | 11 | 20 | 7 | 12 | 18 | 7 | 28 |
| R7 | 58 | — | 17 | 11 | 5 | 4 | 51 | 2 | 45 | 29 | 6 | 24 |

Cách đọc bảng: số đếm là số file mà reviewer ghi tag câu hỏi đó. Q2, Q3, Q4 và Q8 chủ yếu được trả lời ở **mức lớp**, trong các mục `Processes & threads`, `Flows` và `Bottlenecks` của `areas/R*_layer.md`, chứ không gắn cho từng file. Vì vậy số đếm của chúng thấp mà không có nghĩa là thiếu phủ.

Ma trận chi tiết file × Q1–Q12 nằm trong CSV. Mọi file production được đọc FULL đều trả lời đủ Q1–Q12 ở mức file hoặc ở mức lớp (`areas/R*_layer.md`).

## Những gì chưa phủ, cần làm ở bước sau
| Hạng mục | Lý do chưa làm | Đề xuất |
|---|---|---|
| Hành vi runtime (latency, race) | chỉ phân tích tĩnh | Đo phân phối ở P0.2 (M1–M7); chạy TSan trên PlanningWorker, ExecutionAuthority và bridge |
| Build C++ đầy đủ; chạy lại test C++ | thiếu boost và pcl trong container | CI hoặc máy ROS: `make ci-local` |
| Nội bộ vendor (IKFoM esekfom ngoài vùng patch, ikd-tree, quickhull…) | ngoài phạm vi | Giữ ranh giới vendor; review khi đổi version |
| `evaluation.py` 530–1500 (reduce_lifecycle, reduce_world_transactions) | quá lớn, chỉ đọc lướt | Sẽ bị thay bằng decoder typed ở P2 (D8); không đầu tư review sâu |
| Hình học SDF của world | là data | R-03 đã ghi nhận rpy bị bỏ qua |
| 45 file docs lịch sử | không phải nguồn code | Chuyển sang `docs/history/` (quy tắc D1) |
| Kiểm chứng tay các S2 còn lại (ngoài R4-06, R3-11, R7-25) | giới hạn thời gian | Mỗi S2 khi được giao sửa phải bắt đầu bằng test repro (RED trước) |

## Danh sách đối tượng chính đã duyệt (theo lớp)
Class, hàm và máy trạng thái chính đã được mô tả kèm `file:line` trong `areas/R*_layer.md`, mục `## Components`, `## State machines`, `## Behavior per branch`. Theo vùng:

| Vùng | Đối tượng |
|---|---|
| R1 | Planner (plan, planInitial, planSuccessor, generateExpTraj, generateBackup, authorizeAndStage, export, resolveGoal, PathSearch, commitEmergencyBrake, heading rebind, STOP hold), validator swept, AbsoluteDeadline, command_time, A*, CorridorGenerator, CIRI, backup_braking, evidence_speed_governor, route_regression, Config, replan_contract, FOVChecker, NominalProblemSnapshotWriter |
| R2 | Piece, Trajectory, MINCO S4NU, BandedSystem, ExpTrajOpt, BackupTrajOpt, YawTrajOpt, traj_opt Config, trajectory_dynamics, FlatnessMap, lbfgs, Gcopter, geometry_utils, Polytope / SimplifySFC, ExpTraj / BackupTraj / CmdTraj, PlannerFacade, PlannerRuntimeContext, A* header |
| R3 | NavigationRuntimeNode (toàn bộ), MappingTelemetry, PendingGoalHandoffOwner, 45 hàm planner_fsm, DesiredPlanningIntent, runtime_boundaries, PlanningWorker, HeadingRebindWorker, PlanningSupervisor, KinematicDerivativeEstimator, MissionProgress, trace stores, path_relative / experimental tracking, localization_epoch_reset, mapping_fail_stop |
| R4 | Timestamp / Duration / ClockDomain, Status / Result, sensor types, MeasurementBuffer / Synchronizer, ImuInitializer, prior, IkfomEstimator (+ patch esekfom), ScanDeskewer, preprocess, ResidualBuilder / PlaneEstimator / Gate, IkdTreeRegistrationMap, LocalMapManager, FastLioPipeline, AngularVelocityResolver, BaseLinkStateConverter / CovarianceProjector, ImuStatePropagator, PropagatedOdometryWorker, FastLioNode, adapter ROS, RosOutputPublisher, LioPublicFrameGeneration, ParameterLoader, lio_offline |
| R5 | NavigationMode, NavigationModeExecutor, tracking_adapter, velocity_only, tracking_envelope, local_frame_alignment, admission assessment, MissionController (deprecated), EV bridge node, conversion, jump continuity / latch / gate, TimestampConverter, ingress bridge node, FrameConverter, ResetCompensator, OdometryRingBuffer, TimestampValidator, ExecutionAuthority, CommandSampler, ExecutionStateStore, recovery FSM, CandidateBundle, planning contracts, navigation_contracts headers + msg, Mission / RouteProgress, navigation_common |
| R6 | MappingActor, MappingObservation, mapping_types, MappingWorker, ObservationAccounting, MappingWorldSnapshot, WorldSnapshotStore, current_body_support, RuntimeMappingMap, MappingWorldModelView, ROGMap, ProbMap, InfMap / CounterMap, SlidingMap, RayCaster, Config, world_model_view, continuous_clearance, goal_contract, world_commit_authorizer |
| R7 | runner (resolve, param generation, orchestration, witness), process_group Session, monitor StreamStats, ExternalModeScenario FSM, offboard / closed-loop harness, EvidenceWriter, evaluation (các chiều đánh giá, C0_SW), report (verdict), html / flight_review, world_observation_gate, evidence_contract, data.py, 10 static guard + ledger validator + pre_main_gate, uav_simulation bridges, 4 launch, config/runtime (37 YAML) |
