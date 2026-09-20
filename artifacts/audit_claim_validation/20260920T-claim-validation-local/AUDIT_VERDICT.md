# Báo cáo audit giả thuyết nghẽn H0–H7

## Kết luận mở đầu

- **Đúng trong phạm vi source:** use_sim_time=true cùng hệ số tracking bằng 0 bật hai suppression flag trong loader; runtime có thể tạm ngừng command khi world stale rồi chỉ resume bundle đã recertify đúng identity; lệnh ROS được publish trong critical section chia sẻ khóa với đường odometry; runtime và PX4 adapter có timer recovery khác owner/clock; governor dùng grid 16 speed sample.
- **Đúng có điều kiện, chưa phải lỗi vận hành đã xác nhận:** measured crossing có thể không được khôi phục nếu xảy ra lúc continuation witness không hợp lệ và mẫu đo kế tiếp đã rời acceptance region. Candidate admission reserve không ràng buộc trực tiếp tracking/transport delay. Nhưng adapter gọi updateMission() ngay khi nhận continuation hợp lệ và kiểm lại freshness/identity. Chưa tái hiện candidate thật đi qua toàn bộ producer, geometry, certificate và admission.
- **Nhận xét trước nói quá mức:** scheduleMode(AUTO_LOITER) callback Success không chỉ là command ACK; thư viện PX4 pin còn chờ ModeCompleted. Timer runtime và adapter khác điểm bắt đầu/clock nên 5 s bằng nhau không chứng minh xung đột. Publication dưới lock chứng minh khả năng blocking nhưng không chứng minh bottleneck target. use_sim_time + zero coefficients chỉ thay một số response policy, không bỏ typed-health freshness, identity, frame, lease, world/collision và physical-stop gates.
- **Đặc tính fail-closed có chủ đích:** command stale ở PX4 adapter latches safety stop/handover; Deactivated không lập tức retry Hold, để không chiếm lại quyền sau operator/mode takeover. Chưa có bằng chứng cho thấy các kết quả này vi phạm liveness contract được đặc tả.
- **Bottleneck thực tế:** chưa đủ dữ liệu xếp hạng. Không tìm thấy trace workload có source/binary/profile provenance và không đo distribution đồng thời ở funnel.

Kết quả cuối là **PARTIAL_AS_IS**. Đây là kết luận audit, không phải đánh giá an toàn/qualification.

## Baseline và phạm vi

| Baseline | Giá trị |
|---|---|
| Artifact A | Commit f2bd3f46f9936d622377ea4761f733f988273b66; chỉ đọc artifacts/architecture_as_is/20260920T-as-is-local/baseline/source_snapshot/. Snapshot ghi HEAD lúc chụp 9534d8dc15920c8b3e80c8fa12f28ec972b8c6a2 và chứa nội dung từ dirty worktree. Không coi đây là clean tree của SHA đó. |
| Checkout B | HEAD 9534d8dc15920c8b3e80c8fa12f28ec972b8c6a2, branch codex/close-proven-findings, không detached. |
| Dirty state trước audit | 34 porcelain-v2 entries; staged diff 0 byte; unstaged diff 1,778,436 byte; có source/header/test/config/docs chưa commit. Bản chụp đầy đủ ở baseline/status_porcelain_v2.txt; unstaged diff SHA-256 9dc4eb1071b4c8b417be36ff5b8fab9d3f35b3e383fcbdd990143c4284d409d3. |
| A↔B source | Tại thời điểm bắt đầu, 596/596 đường dẫn trong manifest khớp byte giữa A và B. Manifest SHA-256 f9481b2f7312556ae91a8bff590cbc412de8d7e3e068fc98d4c85bb34c730413; final checker xác nhận snapshot A vẫn khớp manifest, nhưng checkout hiện tại C đã drift (xem dưới), nên không có kết luận A↔C. Artifact không chứa tar archive nguồn tách biệt; tái lập dựa trên source snapshot + manifest và artifact commit. |
| Dependency | px4_msgs 86d8239e962f6939e05c3737784f60c02fa884db; px4_ros2_interface_lib 4a3370f084ac6f1ef001a4afa2b007845ffd0837; submodule status sạch. H6 đọc source pin, không dùng documentation mới nhất. |
| Build có trước audit | build/compile_commands.json có 57 entries đều thuộc PX4 interface-library scope, SHA 32e3692ca5c79a3cab76266b502fe9996142e1494b39fdee6506a7e5e40fc2c1; không có build/CMakeCache.txt; install/setup.bash có nhưng shell ban đầu chưa source. Không dùng binary này làm bằng chứng cho A/B. |
| Test build audit | Release, GCC 13.3, ROS Jazzy; hai colcon build biệt lập trong validation/build và validation/install, cùng source A: backend/dependency slice 7 packages và runtime slice 10 packages. Logs và compile database nằm trong output. |
| Live profile | Không có runtime parameter dump, launch argv, mission input, RMW/domain capture hoặc binary-to-run identity. Effective live profile vẫn **UNKNOWN**. |

Trong audit này không reset/checkout/stash/clean, stage/commit/push; không chạy ROS node, paired process, SITL/Gazebo hoặc publish vào thiết bị. Toàn bộ file do audit tạo nằm trong output audit mới. Baseline B ban đầu giữ 34 status entries như đã chụp; tình trạng checkout hiện tại C được báo cáo tách riêng dưới đây.

### Drift phát hiện ở final check

Khi bắt đầu audit, B có HEAD 9534d8dc... và dirty diff như bảng trên; khi chạy final checker, checkout đã chuyển sang HEAD 8eaab3d33db36e9e636ad4005ed91b8f055f4d68 (parent đúng bằng 9534d8dc...), branch vẫn codex/close-proven-findings. Git ghi commit subject “Close braking evidence and lifecycle findings”; working tree ngoài audit output hiện không còn staged/unstaged diff. Audit không thực hiện commit/push và không gán provenance của commit này cho agent.

Final checker phát hiện 15/596 manifest files của current checkout C khác frozen A, gồm runtime node, execution episode, planner governor/caller/config/test và hai safety contract docs. Danh sách/hash đầy đủ ở validation/final_checkout_drift.json. Không ghép code C vào verdict A. H0 và H6 source được trích không nằm trong 15 đường dẫn drift; verdict H1–H5 trong báo cáo chỉ có hiệu lực cho A và B lúc bắt đầu, còn cùng claims trên C được đánh dấu UNVERIFIED_AFTER_CAPTURE_DRIFT cho đến khi đọc lại phần source đổi. H7 vẫn chưa có workload evidence. Checker status cuối là DRIFT_DETECTED, không phải PASS.

## Bảng H0–H7

| ID | Evidence source A | Product reachability | Test/trace | Phản chứng / narrowing | Effective profile | Verdict; phần thiếu |
|---|---|---|---|---|---|---|
| H0 | Loader/config/launch: E-H0-01..08. | Loader branches reachable; process profile của run không biết. | Production loader fixture, 5 tổ hợp: TEST_EXECUTED, exit 0. | sim=false và positive tracking coefficient làm suppression false; velocity-only ngoài sim bị throw. | Live unknown; xem profile_matrix.md. | CONFIRMED_WITH_SCOPE: source mapping đúng; cần param dump/config/mission/binary của workload. |
| H1 | Reserve, producer, immediate consumer, crossing: E-H1-01..08. | Static sequence có; candidate cụ thể qua mọi geometry/certificate/admission chưa chứng minh. | Native MissionController counterexample + positive/identity controls; 19 mission tests + helper boundary. Không runtime trace. | onNavigationCommand gọi updateMission() ngay khi nhận continuation hợp lệ và recheck lease/health/identity. | Source contract biết; live tracking profile unknown. | CONDITIONAL / SPECIFICATION_GAP; cần actual candidate và state/time trace hoặc integrated fixture. |
| H2 | Suspension/resume, adapter expiry/latch: E-H2-01..07. | Statistically reachable nếu không có command recertified hữu ích trước adapter lease, các input khác vẫn hợp lệ. | Lifecycle helper 2 passed; không adapter transport/PX4 run. | Fresh exact-generation recertification trước expiry có thể resume. Completed-recovery exception hẹp theo identity/episode. | Source freshness .5 s, adapter lease .1 s; live unknown. | CONFIRMED_WITH_SCOPE / INTENTIONAL_POLICY; chưa chứng minh liveness violation hoặc PX4 acceptance. |
| H3 | Lock scope, final publish, store finalizer/test: E-H3-01..04. | Odometry shares localization lock; store finalizer invokes publish before unlock. | 2 store tests passed, gồm barrier test; không transport timing. | Executor có 2 threads; API call có thể trả nhanh. Không target distribution/lock wait-hold separation. | RMW/load/binary live unknown. | Structural blocking CONFIRMED_WITH_SCOPE; bottleneck UNRESOLVED. |
| H4 | Runtime timer, adapter timer starts/helper/assertions: E-H4-01..05. | Hai đường source khác lifetime/identity. | 3 production FSM/helper tests passed; chưa chạy adapter timer handler fake clock. | Runtime timeout chỉ trong stopped recovery và measured speed guard; adapter timer chỉ từ completed endpoint. | Steady vs ROS clock; live values unknown. | CONDITIONAL / SPECIFICATION_GAP; thiếu budget relation và same-episode test. |
| H5 | Governor/caller/route support/unit assertions: E-H5-01..04. | Governor fail có thể return trước nominal endpoint solve; production candidate false negative chưa chứng minh. | 3 PlannerSpeedGovernor tests; 24-case helper sweep; không workload timing. | Không sweep input nào grid-reject khi fine sweep có witness; support .401 m vượt >2*resolution với default .2 m nhưng chưa dựng map/route thật. | Planner config không gắn run. | NOT_REPRODUCED; 16-grid là FACT_FROM_SOURCE; false negative/cost unresolved. |
| H6 | App lifecycle + pinned ModeExecutor API: E-H6-01..04. | Pinned client path qua ACK rồi mode activation/ModeCompleted; firmware semantics thiếu. | Test source reviewed; không API/firmware runtime test. | Success không chỉ là ACK. Deactivated không retry có thể bảo vệ operator authority. VehicleStatus stream độc lập. | Submodule revisions biết; firmware/run identity không có. | CONDITIONAL: ACK-only cũ REFUTED; callback/VehicleStatus ordering UNRESOLVED. |
| H7 | Funnel và rejection branches tổng hợp từ E-H1..H6. | Static paths được rà; incidence target không biết. | Bounded artifact scan không thấy trace khớp provenance. | Không cohort, successful control, request/episode grouping hoặc timing distribution. | Workload/source/binary/profile unknown. | UNRESOLVED; KHÔNG ĐỦ DỮ LIỆU XẾP HẠNG BOTTLENECK THỰC TẾ. |

## H1 — continuation và crossing

certifiedMainContinuationBoundaryEligible() đòi pass-through, MAIN role, đúng junction/epochs/request, boundary thuộc MAIN interval và reserve từ analytic boundary đến hết MAIN đạt R. Runtime producer chỉ đặt witness cho sampled MAIN và khi main_end_stamp - command_ros_time >= R. Source không thêm quan hệ R với tracking/transport/source-state age. Admission có thể giữ candidate nếu horizon reserve đạt contract. Xem E-H1-01..03.

Ở consumer có guard bị nhận xét trước bỏ qua: command continuation được chấp nhận gọi updateMission() ngay, ngoài timer 50 ms. updateMission() xác minh command contract/validity, state và health freshness, epoch, mission, waypoint, request; đồng thời không dùng witness của bundle kế cận. Xem E-H1-04..05.

MissionController lưu đè previous sample mỗi update. Nếu measured segment vượt acceptance ball khi witness vắng/không hợp lệ, sau đó vị trí ra ngoài acceptance và witness đến muộn, cặp segment cũ không còn. Harness gọi production MissionController/RouteProgress xác nhận cơ chế cục bộ này và negative control identity. Đây là LOCAL_COUNTEREXAMPLE, không chứng minh runtime planner tạo lịch đó. Kết luận là timing/liveness contract còn thiếu bằng chứng; chưa gán DEFECT.

## H2 — world suspension và authority

Runtime stale-world branch hủy planning worker, lưu suspended bundle generation và suspendCommand(), đồng thời không phát command từ callback stale. Khi world mới về, code recertify rồi kiểm generation, localization/goal epoch, executing identity, validity, failure latch và exposure latch trước khi khôi phục episode. Xem E-H2-01..04.

PX4 adapter giữ command cuối theo header + receive age. Với lệnh thường, khi lease hết, safetyStopNavigation() latch failure/handover, invalidates cached command, deactivate mission controller, publish status/stationary path và yêu cầu Hold. Terminal completed recovery chỉ được giữ trong đúng episode trước deadline. Đây là boundary fail-closed có chủ đích. Chưa chạy paired test, không giả receiver còn đủ 100 ms, và không khẳng định PX4 đã áp dụng Hold.

## H3 — lock và bottleneck

Code xác nhận ROS publish nằm bên trong shared localization/input/command latch locks và store mutex; propagated odometry tranh localization mutex. Source chỉ xác nhận quan hệ chờ có thể xảy ra. Existing store barrier test không đi qua ROS publication. Không đo callback wait/hold trong no-fault workload, RMW, queue delay, guide/corridor/optimizer/recovery cùng một build. Vì vậy không xếp đây là nguyên nhân thực tế.

## H4 — recovery timer

Runtime timer bắt đầu tại lỗi PlanFromRest đầu tiên của identity hiện tại, dùng steady clock; timeout chỉ ở stopped recovery/initial hold khi measured speed trong ngưỡng. PX4 adapter timer bắt đầu từ command Completed nhận được hoặc setpoint completed được quan sát, dùng ROS time và identity mission/waypoint/request/bundle. Hai bộ đếm bảo vệ các chặng khác nhau. Không tìm thấy đặc tả bắt đầu/quan hệ ngân sách; thiếu đặc tả không tự là lỗi.

## H5 — governor và funnel

Source hiện có fixed grid 16 sample; không suy từ audit cũ. Measured PVAJ được kiểm trong physical BACKUP model trước khi hạ desired cruise speed. Caller tạo support từ cửa sổ 20 m, directional AABB, route support và known-free/MAIN policy support. Governor fail return trước nominal endpoint solve ở caller; reason taxonomy tách physical envelope, missing support, synthesis và budget.

Route partial support đòi hơn 2*resolution; do đó ví dụ .02 m không phải chứng cứ product path với default resolution .2 m. Audit sweep cho 24 helper inputs, support .401–5 m, desired speed 1–8 m/s, zero measured derivatives; không thấy grid reject khi fine sweep có witness. Sweep dùng cùng production stop certificate, không phải chứng minh toán học độc lập hay map thật. makeBackupBrakingSeed lấy duration động-feasible đầu tiên rồi evaluateStopReachability so support; chưa có duration counterexample độc lập. Không đo candidate count/cost trong caller.

## H6 — PX4 Hold

Submodule pin: px4_ros2_interface_lib 4a3370f084ac6f1ef001a4afa2b007845ffd0837; px4_msgs 86d8239e962f6939e05c3737784f60c02fa884db. Client ModeExecutor gửi VEHICLE_CMD_SET_NAV_STATE, chờ command ACK, activate scheduled mode, rồi chờ matching ModeCompleted mới gọi callback. Vì vậy nhận xét “Success chỉ là ACK” phải rút lại.

Application px4_hold_confirmed_ chỉ thành true khi nhận VehicleStatus.AUTO_LOITER; API callback có thể clear pending khi Success hoặc Deactivated. Hai stream không có ordering proof trong dependency client. Không có PX4 firmware source/build hoặc runtime status/callback trace để quyết định semantics sâu hơn. Deactivated có thể là operator/mode handover; tự retry có thể chiếm lại quyền.

## Nhận xét cũ cần rút lại hoặc giới hạn

1. H1: Không được khẳng định progress luôn chờ timer 50 ms; accepted continuation gọi updateMission() ngay. Mất crossing chỉ đúng nếu crossing xảy ra khi witness không hợp lệ và sample sau rời vùng/segment có thể chấp nhận.
2. H2: “Recoverable suspension” là runtime intent; downstream adapter có lease độc lập có thể đưa tới safety stop/Hold. Đây là nhánh fail-closed có điều kiện, không phải lỗi liveness đã chứng minh.
3. H3: Lock scope thật, bottleneck target chưa đo.
4. H4: Cùng 5 s không có nghĩa cùng timer. Không ghi “timer conflict” như finding xác nhận khi chưa có budget spec hoặc same-episode reproduction.
5. H5: Giữ source fact 16 mẫu, nhưng bỏ support 0.02 m khỏi product finding nếu không qua route/map resolution gate. Thiếu abort trong loop không tự là deadline miss.
6. H6: API Success không phải ACK-only; status ordering vẫn chưa biết. Không diễn giải giữ quyền operator thành retry bug.
7. H0: Comment YAML nói “disable gates” rộng hơn consumer. Freshness/identity/frame/world/physical certificate vẫn có hiệu lực.

## Test, coverage, renderer

Lệnh, exit code, build provenance và log ở validation/commands.md. Test chạy offline trong output riêng. Không ROS node/publisher, paired process, PX4 firmware, SITL hay thiết bị. Hai isolated colcon builds hoàn tất: backend slice 7 packages (exit 0), runtime slice 10 packages (exit 0). Mermaid CLI không có và máy không có Node/npm; chỉ render Graphviz DOT thành SVG. Không dùng image generation.

Coverage claim-scope:

- H0: loader/2 process loaders/launch/config/build provenance reviewed; 5 loader input combinations executed.
- H1: continuation reserve producer → adapter callback → witness → measured crossing reviewed; integrated candidate/geometry path missing.
- H2: runtime suspend/resume + adapter expiry/latch reviewed; lifecycle helper tests passed; adapter handler/paired transport missing.
- H3: nested lock/publisher/store and odometry competitor reviewed; 2 store tests; ROS publication timing/queue/RMW distribution missing.
- H4: runtime timer writer/reset/read + two adapter starts/expiry/reset reviewed; 3 runtime FSM tests; adapter timer fake-clock test missing.
- H5: governor/caller/reason/route support reviewed; 3 unit tests + 24 helper sweep; immutable map candidate and duration oracle missing.
- H6: app flag writers and pinned dependency callback reviewed; no firmware-level ordering test.
- H7: funnel alternatives listed; matched request/solve/bundle/episode workload cohort unavailable.

Coverage trên đây áp dụng frozen A và B tại thời điểm bắt đầu. Với checkout C sau drift, source bị sửa ở H1–H5 chưa được đọc lại, nên chưa có verdict mới cho các nhánh đó. Chưa gắn READY_FOR_REDESIGN_DISCUSSION vì H1 candidate reachability, H2 transport authority, H3 target contention, H4 budget relation, H5 production witness, H6 firmware order và H7 workload còn thiếu. Tối thiểu để bước thiết kế sau có căn cứ:

1. Run target gắn source SHA/diff, binary/build, launch argv, runtime+adapter parameter dump, mission/map/profile và PX4 firmware/dependency identity.
2. Trace source/receive time, state/frame/epoch, goal/request/bundle/world identities, planner reject reasons, command publish/receive/valid_until, adapter transitions, PX4 status/mode và measured mission progress; correlation theo request/episode.
3. Spec hiện hành về liveness của pass-through crossing, world suspension/recovery và quan hệ budget giữa runtime RetryFromRest với adapter terminal recovery.

Các luận điểm đã bác bỏ không nên mang sang backlog như finding xác nhận.
