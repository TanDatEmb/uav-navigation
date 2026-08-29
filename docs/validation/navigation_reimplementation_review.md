# Đánh giá tái triển khai navigation

Ngày đánh giá/cập nhật: 2026-08-29. Reviewed source HEAD: `b303695e47fe0b306b50b743c2e9052730c9cbee`
trên branch `codex/navigation-stability-checkpoint`; worktree sạch tại thời điểm
checkpoint. Phạm vi gồm estimator, mapping, planner, execution, runtime và PX4
boundary. Đây là đánh giá kỹ thuật, không phải chứng nhận bay.

## Kết luận điều hành

Chưa thể kết luận stack đã hoàn tất. Các lỗi ownership, timestamp, policy,
world snapshot, A*, CIRI, command commit, ROG-map boundary, optimizer boundary
và một số dead parameter đã được sửa ở mức source/unit tại các checkpoint trước
và HEAD này. Tuy nhiên còn ba khoảng trống P0/P1 không được che bằng gate:

1. Chưa có bằng chứng end-to-end lặp lại cho chuỗi mapping → A* → corridor →
   nominal → backup → commit → command → PX4.
2. Chưa có continuous safety certificate đầy đủ cho thân phương tiện cộng sai
   số tracking, localization và mapping; kiểm tra tube hiện tại là kiểm tra
   bảo thủ cho độ cong quỹ đạo trên grid.
3. SITL chưa chạy qua preflight trong phiên này: runner bị
   `PermissionError: [Errno 1] Operation not permitted` khi mở UDP probe cho
   XRCE. Không có artifact SITL nào được tạo, vì vậy không được gọi đây là
   SITL PASS hay dùng để chỉnh threshold.

## Trạng thái theo nhóm kiểm tra

| Nhóm | Trạng thái | Bằng chứng hiện có và phần còn thiếu |
|---|---|---|
| Clock, timestamp, frame | PARTIAL | Đã có conversion dùng chung, mapping clock domain cho SITL và kiểm tra frame/age ở boundary. Cần repeated SITL, realtime recorded data và kiểm chứng offset uXRCE-DDS trên môi trường có UDP. |
| Input LIO và propagated state | PARTIAL | Propagated odometry hiện là stream bắt buộc; FAST-LIO package build và parameter-loader trực tiếp pass (19/19 khi đặt `ROS_LOG_DIR` writable). Chưa có runtime artifact mới chứng minh rate, freshness, reset epoch và PX4 continuity trong closed loop. |
| World snapshot và map evidence | PARTIAL | Snapshot có identity/generation, exact layer bounds, raycasting profile và recertification theo bundle. Chi phí export snapshot, đặc biệt nghi vấn khoảng 71% callback, chưa có p50/p95/p99 mới. |
| A* | PARTIAL | Dùng bounds của immutable world, sparse touched-node workspace, frontier entry bất biến và timeout đúng nghĩa. Chưa có phân phối latency/RSS trên recorded data và SITL. |
| CIRI/corridor | PARTIAL | Đã loại bỏ phép tự-trừ luôn bằng zero trong dựng tiếp tuyến, chặn seed zero-length/non-finite, dùng absolute deadline và giới hạn diagnostic buffer. Chưa có benchmark dense obstacle hoặc chứng minh mọi đường đi hợp lệ không bị cắt quá chặt. |
| Nominal optimizer | PARTIAL | `route_reference` đã tách thành bốn field quality-only; objective không còn là safety certificate và sampled vertical-guide gate đã gỡ. Objective disabled bằng `0`, giá trị âm/không hữu hạn bị từ chối; chưa có conditioning/A-B distribution. |
| Backup và UNKNOWN | PARTIAL | MAIN/backup dùng typed unknown policy; backup yêu cầu known-free evidence và profile bật raycasting. Khi không có mission hợp lệ, planner mặc định `RequireKnownFree`; chưa có integration evidence chứng minh backup luôn là certificate hoàn chỉnh cho toàn bộ đoạn thực thi. |
| Continuous trajectory safety | PARTIAL | Có adaptive subdivision và inflated-cell tube check với budget hữu hạn. Chưa bao phủ hình học thân phương tiện, tracking error, localization error, mapping error và mọi sai số số học theo một certificate độc lập. |
| Commit/execution | PARTIAL | Commit transaction, identity recertification và publish revalidation đã có test. Chưa có race distribution end-to-end khi map thay đổi đúng lúc command publish. |
| Safe stop/FSM/PX4 | PARTIAL | Các nhánh stale/invalid/failed được fail-closed và command provenance được kiểm tra. Chưa có repeated PX4 external-mode run để xác nhận hold/stop, waypoint acceptance, altitude và speed recovery. |
| Performance | OPEN | Chưa đủ dữ liệu để kết luận `Observed duration over simulation time` xấu hơn hay tốt hơn. Không được đổi rate, deadline, raycasting hoặc threshold từ một run. |
| Dataset | OPEN | Các artifact cũ chỉ là health/shadow evidence và không đại diện cho source hiện tại. Cần chạy lại scenario đại diện sau khi build ổn định. |
| SITL | BLOCKED BY ENVIRONMENT | Preflight UDP bị sandbox chặn trước PX4/Gazebo. Giữ nguyên probe và gate; cần môi trường cho phép isolated XRCE UDP. |
| C++20 và vocabulary | PARTIAL | Toàn bộ product package C++ dùng chuẩn C++20 và source/config/tooling không còn tên implementation tham khảo; backend chỉ cài `PlannerFacade`, interface chết và parity checker đã gỡ. C++14/17 còn lại thuộc external dependency, còn provenance/license trong docs là bắt buộc và không phải product API. |

## Ma trận truy vết bộ câu hỏi

| Mục | Đánh giá hiện tại | Bằng chứng hoặc khoảng trống chính |
|---|---|---|
| 0. DNA main/backup/stop | PARTIAL | Candidate có role schedule, backup known-free và commit fail-closed; chưa có chuỗi E2E lặp lại chứng minh toàn bộ bất biến. |
| 1. FAST-LIO state | PARTIAL | State P/V/A/J, timestamp, freshness và propagated stream có typed/unit tests; thiếu recorded-data và SITL lặp lại. |
| 2. LiDAR timing/deskew | PARTIAL | Adapter/point-time contract có test; chưa có dataset vật cản mỏng, ít điểm và occlusion. |
| 3. Local world model | PARTIAL | Map trượt, layer bounds, UNKNOWN/OCCUPIED/FREE và snapshot identity đã có; export vẫn cần benchmark tail. |
| 4. Map/planning horizon | PARTIAL | Geometry bind từ snapshot và validation horizon đã có; chưa chứng minh theo sensor/airframe thực tế. |
| 5. A* | PARTIAL | UNKNOWN policy, exact bounds, frontier immutable và timeout đã có regression; chưa có latency/RSS distribution. |
| 6. Path reduction | OPEN | Đường search được giữ dày để validate seed; cần benchmark/thiết kế line-of-sight reduction trước khi tối ưu tiếp. |
| 7. CIRI | PARTIAL | Guard finite/degenerate, deadline, bounded diagnostics và tube validation đã có; dense-obstacle feasibility còn mở. |
| 8. Exploration/safety corridor | PARTIAL | Mission policy typed; MAIN cho phép UNKNOWN chỉ khi mission hợp lệ khai báo, BACKUP ép known-free; không có mission thì fail-closed known-free; chưa có coverage E2E. |
| 9. Known-free/FOV/occlusion | PARTIAL | Raycasting profile và inflated segment certificate đã bật; chưa chứng minh đầy đủ mount FOV, che khuất và dynamic obstacle. |
| 10. Time-parameterized trajectory | PARTIAL | Piecewise polynomial, MINCO, time optimization và PVAJ boundary có trong source; chưa có closed-loop tracking evidence. |
| 11. Dynamic limits | PARTIAL | V/A/J, body-rate, thrust và flatness gates được kiểm tra; nguồn thông số airframe/controller vẫn chưa được chứng minh. |
| 12. Backup trajectory | PARTIAL | Seed dừng động lực học, continuity, known-free và atomic candidate gate có; thiếu repeated failure/stop scenarios. |
| 13. Switch time | PARTIAL | `t_s` được giới hạn và tìm lùi theo braking hull; cần log distribution và kiểm chứng progress/safety. |
| 14. Commit/FSM | PARTIAL | Immutable committed bundle, recertification, stale/timeout fail-stop và provenance có test; chưa có race E2E. |
| 15. Replan budget | PARTIAL | Absolute deadline dùng xuyên A*/CIRI/optimizer; chưa có p50/p95/p99 theo stage. |
| 16. Hot-path efficiency | OPEN | Đã bỏ eager A* workspace và dead raycast override; snapshot export/raycast/copy/RSS chưa được đo lại. |
| 17. Controller | OPEN | Planner có dynamic gate và PX4 envelope boundary; tracking error/controller equivalence chưa có evidence. |
| 18. PX4 boundary | PARTIAL | Frame/time conversion, propagated odometry và fail-closed bridge có unit tests; SITL bị chặn trước launch. |
| 19. Parameter correctness | PARTIAL | Nhiều duplicate/dead knobs đã gỡ; C++ mission schema và runtime profile đã một owner; runtime dùng `data_freshness_window_s` như một cửa sổ duy nhất trong cùng boundary, còn PX4/bridge giữ các lease khác clock. Boundary limits thiếu giá trị bị reject với default `0`, không còn sentinel âm. |
| 20. Vehicle/sensor feasibility | OPEN | Chưa có hardware hoặc calibration evidence cho sensing, thrust-to-weight, CPU thermal và latency. |
| 21. Adversarial scenarios | OPEN | Có regression cục bộ cho curve/degenerate/unknown/race; chưa chạy đủ thin obstacle, slope, ceiling, blackout và retarget suite. |
| 22. Upstream regressions | PARTIAL | Một số lỗi upstream đã thành unit tests; danh sách scenario đầy đủ vẫn cần dataset/SITL harness. |
| 23. Observability/replay | PARTIAL | Cycle/stage/commit/backup/failure telemetry và error history có; cần artifact provenance E2E. |
| 24. Benchmark matrix | OPEN | Chưa có bộ cảnh chuẩn và phân vị timing/CPU/RSS/tracking. |
| 25. Acceptance criteria | OPEN | Chưa có success/collision/replan/backup/tracking/clearance distribution; không tuyên bố theo tốc độ đơn lẻ. |
| 26. Software architecture | PARTIAL | Product contracts, facade/PImpl và boundary packages đã có; legacy headers và config duplication còn tồn tại. |
| 27. Non-core upstream features | PARTIAL | Dead dual-planning switch và một số knobs đã gỡ; ESDF/frontier/FOV cần quyết định theo consumer thực tế. |
| 28. Performance DNA | PARTIAL | Local map, sparse search, CIRI local query và staged deadlines có; raycast/export chi phí chưa được chứng minh. |
| 29. Final acceptance gate | OPEN | Chỉ đóng sau repeated dataset + SITL + sanitizer/hardware evidence và truy vết đủ mọi lỗi. |
| 30. Review invariant | ADOPTED | Mọi simplification phải chỉ ra invariant, cơ chế thay thế và test/benchmark tương đương trong decision ledger. |

## Đánh giá `route_reference`

Phương án bốn tham số độc lập là hợp lý:

```yaml
route_reference:
  lateral_weight: 1.0
  vertical_weight: 1.0
  lateral_deadband_m: 0.05
  vertical_deadband_m: 0.05
```

Các weight chỉ làm thay đổi chất lượng nghiệm và điều kiện khởi tạo optimizer;
không được dùng để cấp quyền an toàn. Các deadband có đơn vị mét; weight là
thang đo của objective solver, không phải đại lượng vật lý và không nên được
dùng như một ngưỡng safety. Không nên thêm gate cho route-reference: nếu
quỹ đạo vượt corridor, occupied cell, dynamic limit hoặc known-free backup
certificate thì validator độc lập phải từ chối. Dùng gate mới ở đây sẽ biến
quality term thành safety authority và lặp lại lỗi coupling cũ.

Giá trị `1.0/1.0/0.05/0.05` hiện chỉ là baseline dễ đọc, chưa phải giá trị đã
được hiệu chỉnh. Cần ghi nhận objective magnitude, gradient norm, solver
iterations, reject reason và latency distribution trước khi đổi số.

## Kết quả audit tham số

### Đã xử lý

- `robot_r` không còn là YAML knob; được derive từ vehicle, tracking,
  localization, mapping và planning margin.
- Planner không còn tự sở hữu unknown-space boolean; mission policy được
  chuyển thành typed policy. Runner cũng không tái sinh ghost key.
- `astar/map_voxel_num` và planner-side duplicate world geometry đã bỏ; search
  lấy bounds/resolution từ snapshot.
- `astar/search_time_limit_s` không còn bị load như một authority thứ hai;
  planner resolve budget rồi truyền xuống A*.
- visibility horizon đã tách cap, floor, effective value; sensing horizon dùng
  `0.0` để biểu thị disabled thay vì số âm.
- `propagated_odometry.enabled` đã bỏ vì đây là stream bắt buộc của runtime;
  không còn nhánh có thể tắt producer nhưng vẫn báo contract hợp lệ.
- Với mission `allow_unknown`, candidate không có BACKUP suffix không còn bị
  từ chối theo cờ; nó phải vượt qua chứng nhận `KNOWN_FREE` cho toàn bộ quỹ đạo.
  Candidate có BACKUP vẫn dùng UNKNOWN cho MAIN và known-free cho BACKUP.
- Revalidation cũng dùng cùng resolver role-aware; không còn đường giữ lại
  main-only bundle qua UNKNOWN sau khi map đổi revision.
- Duration ở các boundary sản phẩm dùng giây và được chuyển sang integer time
  một lần ở loader/boundary.

### Còn phải xử lý, chưa tự ý thêm parameter

- `data_freshness_window_s` được dùng cho world source, execution state và
  command lease trong runtime. Không tách thành nhiều knob khi chưa có bằng
  chứng hai policy khác nhau; PX4/bridge vẫn là các boundary clock khác và
  không đọc lại tham số runtime.
- `planner_watchdog_timeout_s` và backend `solve_deadline_s` có hai owner hợp lệ
  khác nhau; tên runtime đã thể hiện đây là watchdog, còn backend là solve
  budget. Vẫn cần artifact chứng minh quan hệ hai budget trong repeated runs.
- C++ mission parser đã gom vào `navigation_mission::loadMission()`; PX4 và
  runtime dùng chung schema/validation. Python runner/report vẫn đọc YAML ở
  boundary tooling để orchestration và đánh giá artifact, không phải authority
  bay; UNKNOWN policy được chuyển sang enum dùng chung với world model; cần giữ
  contract test chống schema drift.
- Root `config/runtime/mapping.yaml` là nguồn chuẩn duy nhất của runtime
  profile; CMake sinh bản cài đặt `navigation_runtime.yaml` từ nguồn này.
  `data_freshness_window_s` là tên contract hiện tại; loader duplication ở
  runtime profile đã được loại bỏ. Các boundary PX4/bridge khác clock vẫn cần
  phân phối timing riêng trước khi cân nhắc derive giá trị.
- Các objective quality đã dùng `0` làm giá trị tắt duy nhất; objective âm và
  không hữu hạn bị reject. `waypoint_attraction_weight` chỉ là quality term của
  nominal optimizer, không phải safety gate hay switch bypass.
- Các source-auth topic/frame của bridge là boundary authentication, không
  nên biến thành knob chỉ để tránh hard-code. Nếu đổi phải có contract source
  identity thay thế.

## Verification đã chạy

Trên source sau các thay đổi hiện tại:

- `fast_lio_ros` build thành công; parameter-loader trực tiếp **19/19** pass
  khi source đúng ROS/workspace overlay và đặt
  `ROS_LOG_DIR=/tmp/uav-navigation-ros-log`. Lần chạy không đặt biến này bị
  lỗi môi trường do `/home/letandat/.ros/log` read-only, không phải assertion
  của loader.
- `navigation_planning_backend` CTest **5/5** pass.
- `navigation_mission` CTest **1/1** và PX4 External Mode CTest **3/3** pass
  sau khi chuyển loader mission sang package dùng chung.
- `navigation_runtime` CTest **7/7** pass.
- Python runtime contract và HTML report **171/171** pass.
- Bridge, mapping vendor và execution đã rebuild/recheck sau các thay đổi liên
  quan và pass lần lượt **6/6**, **1/1** và **2/2**.
- Lần chạy trực tiếp sau parameter cleanup xác nhận thêm: planner runtime
  context **4/4**, mission contract **3/3**, map vendor **19/19**, navigation
  planning **85/85**, runtime mapping **39/39**, mapping worker **17/17**,
  PX4 mission **28/28**, timestamp conversion **10/10**, typed contracts
  **5/5**, geometric continuity **5/5**, execution bundle/state **9/9** và
  FAST-LIO parameter loader **19/19**. Các cảnh báo UDP/getifaddrs chỉ là giới
  hạn sandbox.

Các kết quả trên là build/unit/contract evidence. Chúng không thay thế dataset,
SITL hoặc hardware acceptance.

## Lịch sử lỗi và nguyên tắc đóng

`docs/validation/refactor_error_history.md` đã ghi các lỗi phát hiện trong quá
trình refactor: duplicate policy, hidden runner override, world revision
invalidation, CIRI budget, A* workspace/frontier, route-reference coupling,
zero-length CIRI seed, off-tube certificate false-reject, main-only known-free
false-reject, propagated enable switch và SITL preflight. Mỗi thay đổi
safety/runtime tương ứng đã được ghi trong
`docs/architecture/runtime_safety_decision_ledger.md` với owner, impact,
evidence, removal condition và command.

Không được đóng đánh giá khi còn một mục P0/P1 chỉ có unit evidence, chưa có
artifact provenance hoặc chưa truy vết được lỗi về source/test/scenario. Không
được gỡ gate để làm cho SITL hoặc dataset PASS.

## Phase tiếp theo

1. Chạy lại build/test trong workspace sạch tương đối; lưu provenance của
   source, config và binary. Không commit cùng lúc với tuning.
2. Trong môi trường có UDP, chạy một dataset và một SITL smoke; sau đó chạy
   bộ scenario đại diện lặp lại, thu p50/p95/p99 cho snapshot export, mapping,
   planner solve, command publication, observed simulation duration và RSS.
3. So sánh từng stage với baseline tham khảo chỉ ở dạng benchmark, không copy
   behavior hoặc nới safety predicate.
4. Fix ngay các lỗi causal P0/P1; lỗi quan sát/độ rõ cấu trúc ghi vào history,
   không biến thành bypass.
5. Sau khi end-to-end evidence xanh, hoàn tất các contract còn PARTIAL, chạy
   sanitizer/hardware evidence và lập checkpoint Git. Vocabulary/C++20 hygiene
   product đã được thực hiện; provenance/license docs không được xóa. Hiện Git
   index đang read-only nên chưa thể tạo commit an toàn trong workspace này.
