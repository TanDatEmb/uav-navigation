# A5 — Spec hợp đồng: certifier, bundle, SafetyProfile, evidence

Baseline `main @ 7e0b850`. Tài liệu này là spec cho các phase P1 (SafetyProfile), P2 (evidence) và P3 (certifier shadow). Mọi giá trị số trong tài liệu là **giá trị đang chạy**; không có giá trị nào được tune.

## 1. `nav_certifier`

### 1.1 Vì sao cần
Planner hiện tự dựng bundle rồi tự bật cờ chứng nhận:
- `certificates.dynamics = flatness = world = true` (`planner.cpp:1057-1059`);
- `terminal_stop` được suy ra từ `kind` (`:1064-1066`);
- closure `world_validator` capture `execution_command` và `certificate` của chính planner (`:1137`).

Runtime chỉ kiểm các cờ đó (`CompleteBundleCertificates::completeFor`, `candidate_bundle.hpp:66-76`). Như vậy bên sinh (generator) đang tự chứng nhận cho mình (RC2).

Code validator đã tồn tại và làm đúng việc. Vấn đề chỉ là **nó nằm sai package và chạy dưới quyền của planner**.

### 1.2 Danh mục certificate (chuyển nguyên văn code từ backend)
| Certificate | Nguồn hiện tại | HG | Input (chỉ data) | Output |
|---|---|---|---|---|
| `DYNAMICS` | `traj_opt/trajectory_dynamics.hpp` | HG-004 | `PiecewisePolynomial`, envelope main/backup (profile) | ok, max v/a/j/yaw-rate, margin, failure code |
| `FLATNESS` | idem (thrust và tilt từ flatness) | HG-004 | idem | ok, max tilt/thrust, margin |
| `CORRIDOR` | `corridor_plane_validation.hpp`; `continuous_clearance.hpp:23` | HG-002 / HG-013 | polynomial, corridor planes, `corridor_plane_tolerance_m` | ok, min slack, piece/time của vi phạm |
| `WORLD_SWEPT` | `trajectory_world_validator.hpp` (sampling tạm thời 2–50 ms, spatial factor 0.5) | HG-010 | polynomial, role schedule, **`WorldSnapshot` đã pin (dạng value)**, unknown policy theo role (BACKUP luôn là `kRequireKnownFree`, `:529-533`), radius sum | ok, `first_blocked_{time,position,cell_state}`, `evaluated_generation`, `sample_count`, tube failure |
| `ROUTE_REGRESSION` | `route_regression_certificate.hpp` | — | polynomial, route boundary | ok, failure code |
| `STOP` (BACKUP / terminal / emergency) | phần certify stop trong `backup_braking.hpp` | HG-035 | suffix polynomial, envelope physical_backup | ok, stop time, stop distance |
| `ANCHOR_CONTINUITY` | `execution_anchor.hpp` (**có 2 bản**: execution và planning); tolerance tại `candidate_bundle.hpp:398-403` = `execution_anchor.hpp:47-52` | HG-007 / HG-027 | PVAJ boundary của bundle, PVAJ của command đang chạy tại activation | ok, error từng thành phần |

Bộ certificate bắt buộc theo `kind`:
- `kMainWithBackup`: DYNAMICS, FLATNESS, CORRIDOR, WORLD_SWEPT, STOP(backup), ANCHOR_CONTINUITY, ROUTE_REGRESSION (nếu có route boundary).
- `kTerminalStop`: như trên, nhưng STOP áp cho terminal thay vì backup.
- `kEmergencyBrake`: DYNAMICS, FLATNESS, WORLD_SWEPT, STOP(emergency), ANCHOR_CONTINUITY. Boundary lấy từ measured state, và A/J ước lượng bị bỏ theo `makeMeasuredEmergencyBoundary`.

Điều này **khớp `completeFor()` hiện tại** (dynamics ∧ flatness ∧ world ∧ (terminal ⇒ terminal_stop)), chỉ viết tường minh hơn. Nếu P3 shadow tìm thấy một certificate mà baseline không thực sự chạy cho một `kind` nào đó, trường hợp này được ghi thành finding, **không** tự bổ sung vào gate.

### 1.3 API
```cpp
namespace nav_certifier {
struct CertifyInput {
  const nav_plan_contract::CandidateBundle& bundle;         // pure data (§2)
  const nav_world_contract::WorldSnapshot&  world;          // value/immutable handle, pinned
  const nav_plan_contract::CommandBoundary& boundary;       // PVAJ tại activation
  const nav_safety_profile::Profile&        profile;
};
CertificateRecord certify(const CertifyInput&) noexcept;               // chạy trên lane
CertificateRecord recertify(const CertificateRecord& prior,
                            const nav_plan_contract::CandidateBundle&,
                            const nav_world_contract::WorldSnapshot& new_world,
                            const nav_safety_profile::Profile&) noexcept; // chỉ chạy lại WORLD_SWEPT (+CORRIDOR nếu planes đổi)
}
```
- `nav_certifier` **không link** `nav_planner`. CMake gate: kiểm `get_target_property(LINK_LIBRARIES)`, và thêm một test grep include.
- `certify` là hàm thuần: cùng input thì cùng output. Test property: gọi hai lần, record giống hệt nhau từng byte.

### 1.4 `CertificateRecord`
```cpp
struct CertificateResult { CertificateKind kind; bool ok; uint16_t failure_code;
                           double margin; int64_t violation_time_ns; };   // margin theo đơn vị của kind
struct CertificateRecord {
  uint64_t certificate_id;              // hash(bundle_digest, world identity, profile hash, boundary)
  uint64_t bundle_digest;               // hash của pieces + role schedule + identity (§2)
  WorldIdentity world;                  // epoch, generation, revision, observation_stamp_ns
  uint64_t safety_profile_hash;
  CandidateBundleKind kind;
  std::array<CertificateResult, kCertificateKindCount> results;  // kind không áp dụng ⇒ NOT_APPLICABLE
  bool complete;                        // = mọi kind bắt buộc đều ok
  int64_t certified_at_ns;              // steady, chỉ để diagnostic
  uint32_t certifier_version;
};
```

**Commit check O(1)** trong execution reducer, thay cho `completeFor()`:
`record.complete ∧ record.bundle_digest == digest(bundle) ∧ record.world == bundle.pinned_world ∧ record.safety_profile_hash == profile.hash ∧ identity hiện hành`.

Execution **không** gọi lại validator. Recert khi world đổi đi qua `RECERTIFY_RETAINED` (A4 §2.4).

### 1.5 Nơi chạy và kế hoạch P3 (shadow)
1. **P3a.** Chuyển validator sang `nav_certifier`; planner vẫn gọi chúng qua `nav_certifier` (đổi lời gọi, hành vi giữ nguyên).
2. **P3b (shadow).** Planning lane sau `solve` gọi `certify` độc lập. Kết quả so với cờ planner tự đặt và ghi `CertificateEvidence`. Gate vẫn dùng cờ cũ.
3. **P3c (switch).** Gate đổi sang `CertificateRecord`; cờ trong bundle bị xoá. Đây là commit behavior riêng, chỉ làm khi phân phối shadow trên P0.2 cho **0 bất đồng** ở các trường hợp planner=true/certifier=false. Nếu có bất đồng: phân tích từng case; không nới gate.

## 2. `CandidateBundle` dạng pure data (`nav_plan_contract`)
```cpp
struct PolyPiece { int64_t duration_ns; std::array<Eigen::Vector3d, N+1> pos_coeffs; std::array<double, N+1> yaw_coeffs; };
struct PiecewisePolynomial { int64_t origin_ns; std::vector<PolyPiece> pieces; };   // N = bậc hiện tại của traj_opt
struct RoleInterval { int64_t begin_ns, end_ns; CandidateRole role; };               // offset từ origin_ns
struct CandidateBundle {
  BundleIdentity id;                 // localization_epoch, goal_epoch, request_id, bundle_generation, producer_cycle_id
  WorldIdentity pinned_world;        // world được solve và certify
  int64_t valid_from_ns, valid_until_ns, activation_ns, declared_start_ns, declared_end_ns;
  CandidateRole role; CandidateBundleKind kind; CandidateSource source; bool terminal_stop;
  PiecewisePolynomial trajectory;    // MAIN + suffix trong cùng một polynomial
  std::vector<RoleInterval> role_schedule;
  std::optional<int64_t> backup_start_ns;
  AxisAlignedBox protected_region;
  std::optional<RouteBoundaryConstraint> route_boundary; std::optional<RouteBoundaryEvent> route_event;
  std::optional<CandidateQuality> quality;   // chỉ diagnostic
};
TrajectoryPoint sample(const CandidateBundle&, int64_t t_ns) noexcept;  // hàm tự do, thay closure `evaluator`
```

So với hiện tại:
- Xoá `evaluator` và `world_validator` (hai `std::function`, `candidate_bundle.hpp:164-165`).
- Xoá `CompleteBundleCertificates` (thay bằng `CertificateRecord` đi kèm bundle).
- Xoá mọi field `*_time_s` kiểu double (`start_wall_time_s`, `duration_s`, `backup_start_time_s`, `CandidateRoleInterval::{begin,end}_time_s`). Hiện có cả `declared_start_ns` lẫn `start_wall_time_s`, và phải kiểm chéo giữa chúng (`:256-262`). Đích chỉ còn int ns.
- Bundle **không** include `navigation_world_model` (`:14`). `WorldIdentity` và `AxisAlignedBox` chuyển về `nav_core_types`.

Rủi ro chính: `sample()` phải cho cùng điểm như `evaluator` đang cho. Test: với mọi bundle trong trace P4-0 và P0.2, so `sample` với `evaluator` trên lưới 1 ms. Sai khác cho phép bằng roundoff tolerance hiện có (`candidate_bundle.hpp:398-403`); không đặt tolerance mới.

## 3. `nav_safety_profile`

### 3.1 Luật
1. **Một nguồn:** `config/safety_profile/<name>.yaml`. Code C++, Python và schema ROS param được **sinh** từ file này.
2. **Không default trong C++** cho tham số safety. Thiếu key ⇒ load fail ⇒ process không arm. Trạng thái hiện tại ngược với điều này: ví dụ `config.hpp:202` có yaw-rate 1.5 trong khi `planner.yaml:72` là 2.0 (A6 I-04), và LIO có ba conflict YAML↔loader (A6: `estimator_min_range_m`, `registration_voxel_m`, `lidar_queue_capacity`).
3. **Overlay deployment** (`sim`, `dataset`) chỉ được đổi key có `overlay: allowed`. Mission YAML **không** được chứa key safety. Hiện `mission_acceptance_speed_mps = 0.15` đang nằm trong mission YAML (A6); key này chuyển vào profile.
4. **Hash** = SHA-256 của profile sau khi canonical hoá (key sort, số in bằng repr), cắt còn 64 bit cho `uint64` trong msg. Hash đầy đủ nằm trong `ConfigWitness`.
5. **Mọi process** (`lio_node`, `nav_core_node`, `px4_adapter_node`, `odom_bridge_node`, judge) load cùng profile, publish `ConfigWitness` lúc start. Adapter kiểm hash của `MissionProgress` và `NavigationCommand` (thông qua evidence header); lệch hash ⇒ không arm (rule 6).
6. **Pinned parameter bị bỏ:** `navigation_mode_node.cpp:255-258` đang pin 4 timing literal. Adapter đọc giá trị từ profile, và test assert giá trị bằng profile chứ không bằng literal.
7. **Mọi key có `qualification_status`** (`PROVISIONAL` / `ACTIVE` / `NOT_EVALUABLE`) và `hg` nếu có. Key mới không có evidence thì bắt buộc là `PROVISIONAL`.

### 3.2 Schema (nhóm theo draft A6-R1; các giá trị lấy từ draft đó)
```yaml
schema_version: 1
profile: {name: sitl_current_as_is, qualification_status: NOT_EVALUABLE}
timing:            # planner_period 0.10, solve_deadline 0.08*, stitch 0.40, commit_guard 0.02, command_period 0.02,
                   # planner_watchdog 1.0, finalization_reserve 0.04, initial_hold_timeout 5.0, stopped_recovery_timeout 5.0,
                   # temporal_sample_{min,max} 0.002/0.05, astar_attempt 0.03*, astar_total 0.06*,
                   # emergency_prepare_deadline: null (P0.2), mapping_update_budget: null (P0.2)
speed_gates:       # stationary 0.15 (6 bản sao hiện tại → 1 key, xem 3.3)
freshness:         # observation_max_age 0.5, adapter_trajectory_stale 0.10, adapter_state_stale 0.20, odometry_diag_max_age 2.0 (xoá ở P6),
                   # reset_metadata_max_age 0.10, odometry_association_gap 0.005, visibility_association_max_age 0.5, ...
lease:             # adapter_command 0.10, trajectory_wait 5.0, planner_recovery_wait 5.0
envelope: {main: {...}, physical_backup: {...}, route: {...}}
geometry:          # vehicle_radius 0.35, tracking/localization/mapping/planning budgets 0.25/0.05/0.10/0.05 (sum 0.80 là DERIVED),
                   # command_anchor_error_limit 0.75, goal_completion_tolerance 0.20, corridor_plane_tolerance 0.01,
                   # map_resolution 0.20, spatial_step_factor 0.5, local_window 20, visibility_horizon 14/23, airborne_height 0.5
tolerance_numeric: # anchor_pvaj_roundoff {1e-5,1e-5,1e-4,1e-3,1e-6,1e-5} (xoá bản sao ở execution_anchor.hpp)
retry: {...}
queue: {...}
estimator:         # overlay: allowed — min_range, scan_voxel, registration_voxel, local_map_half_extent (sim/dataset khác nhau là hợp lệ)
```
`*` = chờ D7(a) (HG-001). Các giá trị đã có trong draft A6-R1 được copy nguyên, không tính lại.

Key `DERIVED` (ví dụ `planning_radius_sum_m`, `minimum_main_reserve_s`) được **tính** trong codegen từ các key nguồn và không được ghi tay. Hiện `minimum_main_reserve_s` được suy từ solve deadline dạng typed ở `planning_timing.hpp:14-15`. Nếu YAML đổi solve deadline mà reserve không đổi theo, sẽ sinh ra hai contract khác nhau (A6 I-02). Codegen loại bỏ khả năng đó.

### 3.3 Hợp nhất `0.15 m/s`
Có 6 bản sao cùng giá trị:
- `planning_timing.hpp:25`
- `mission.hpp:38`
- `mission_controller.hpp:100`
- `mission_controller.cpp:376`
- `navigation_mode_node.cpp:1526-1527`
- `navigation_runtime_node.cpp:4499,4617`

Chúng đổi thành một key `speed_gates.stationary_speed_mps` **nếu** cả 6 cùng ngữ nghĩa "đã dừng". A6-R1 tách thành 5 tên (stationary / mission_acceptance / safety_stop / adapter_stationary / runtime_stopped_recovery). Quyết định: giữ **5 key riêng, cùng giá trị** ở P1 (refactor thuần). Chỉ gộp sau khi chủ dự án xác nhận chúng là cùng một ngữ nghĩa (D7(b)).

### 3.4 Codegen
`tools/safety_profile/gen.py` sinh ra ba thứ:
- `nav_safety_profile/generated/profile.hpp`: struct và loader; không có default, `validate()` kiểm miền giá trị;
- `nav_safety_profile_py/profile.py`: dùng cho judge và runner. Thay các literal fallback trong runner, ví dụ `0.35` (R-08, A6 I-03);
- `schema.json`: để kiểm YAML trong CI.

CI kiểm file generated có khớp với nguồn (chạy lại gen rồi `git diff --exit-code`).

## 4. Evidence schema (`nav_evidence_msgs`)
Danh sách msg và EvidenceHeader xem A2 §4. Phần dưới bổ sung định nghĩa chi tiết cho các msg mới hoàn toàn.

```
# CertificateResult.msg
uint8 KIND_DYNAMICS=0
uint8 KIND_FLATNESS=1
uint8 KIND_CORRIDOR=2
uint8 KIND_WORLD_SWEPT=3
uint8 KIND_ROUTE_REGRESSION=4
uint8 KIND_STOP=5
uint8 KIND_ANCHOR_CONTINUITY=6
uint8 kind
uint8 STATUS_OK=0
uint8 STATUS_FAIL=1
uint8 STATUS_NOT_APPLICABLE=2
uint8 STATUS_NOT_RUN=3
uint8 status
uint16 failure_code          # enum per kind, generated table in nav_certifier
float64 margin
int64 violation_time_ns

# CertificateEvidence.msg
EvidenceHeader evidence
uint8 PURPOSE_CERTIFY=0
uint8 PURPOSE_RECERTIFY=1
uint8 PURPOSE_SHADOW=2
uint8 purpose
uint64 certificate_id
uint64 bundle_digest
BundleIdentity bundle
WorldIdentity world
uint8 bundle_kind
CertificateResult[] results
bool complete
bool planner_claimed_complete   # P3b shadow only; removed in P3c
int64 duration_ns

# ConfigWitness.msg  (transient_local, depth 1)
EvidenceHeader evidence
string profile_name
string profile_sha256
uint16 schema_version
string[] overlays
string[] keys
string[] values                 # canonical repr; same order as keys
string build_flavor             # "product" | "sitl"

# DropCounter.msg  (1 Hz per producer)
EvidenceHeader evidence
string[] topics
uint64[] dropped                # publish attempted but queue full / suppressed
```

Luật schema:
- **Enum:** mọi enum trong msg có hằng số tường minh, và C++ `static_assert` từng giá trị bằng enum domain tương ứng (chấm dứt R-05 dạng "ordinal bị hiểu sai").
- **Version:** thêm field là **minor** (decoder cũ vẫn đọc được). Đổi hoặc xoá field là **major**: tăng `schema_version`, và judge từ chối bag có major lạ ⇒ `NOT_EVALUABLE`.
- **Decoder:** sinh từ `.msg` (`rosbags` typestore). Judge không được `import` tên key chuỗi. CI gate: grep `DiagnosticArray` và `KeyValue` trong `nav_judge` ⇒ fail, sau khi P2 xong.
- **Dual emit (P2):** trong suốt P2, product vẫn phát `DiagnosticArray` như cũ. Judge v2 đọc typed; judge v1 được giữ để so sánh. Verdict hai bản phải bằng nhau trên mọi bag của P0.2. Nếu khác, đó là finding; không tự sửa ngưỡng.
