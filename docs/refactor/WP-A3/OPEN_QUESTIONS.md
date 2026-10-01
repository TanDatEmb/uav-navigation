# OPEN QUESTIONS

## OQ-A3-01 — Provenance của tài liệu phụ thuộc WP-D0

- **Câu hỏi:** Có cần đưa `ARCHITECTURE_REVIEW.md`, ADR-013..016 và
  `risk_register_20260928.md` vào baseline `main@7e0b850`, hay giữ chúng là
  target/reference của WP-D0?
- **Bằng chứng:** Các file không có trong tree baseline, nhưng đã được đọc
  read-only từ `origin/refactor/WP-D0` theo orchestration note; không merge,
  cherry-pick hoặc dùng chúng làm source-line evidence cho baseline.
- **Xử lý R1:** Dùng D0 chỉ để kiểm tra ownership/target/lineage context; mọi
  line claim trong artifact vẫn trỏ vào source hoặc safety file trên baseline.
  Project owner cần quyết định provenance đóng gói khi hợp nhất architecture.

## OQ-A3-02 — solve/A* authority nào có hiệu lực?

- **Câu hỏi:** Chốt `solve=80 ms, A*=30/60 ms` theo executable
  `planner.yaml`/`PlanningTimingContract`, hay `solve=180 ms, A*=40/80 ms` theo
  HG-001 trong safety ledger?
- **Bằng chứng:** executable values tại `src/runtime/navigation_runtime/config/planner.yaml:40-44,196-203`
  và `src/planning/navigation_planning/include/navigation_planning/planning_timing.hpp:9-15`;
  ledger tại `docs/safety/runtime_safety_current.md:89-97`.
- **Rủi ro:** Chốt lane/certifier theo một giá trị mà không giải quyết conflict
  sẽ làm sai deadline ownership và acceptance semantics.

## OQ-A3-03 — Bốn process có bao gồm legacy bridge không?

- **Câu hỏi:** “Bridge” trong deliverable là `px4_odometry_bridge_external_node`
  duy nhất của launch mặc định, hay bao gồm thêm executable legacy
  `px4_odometry_bridge_node`?
- **Bằng chứng:** CMake/launch có cả hai executable; external bridge callback-driven
  tại `px4_external_odometry_bridge_node.cpp:538-541`, legacy bridge có diagnostics
  wall timer tại `px4_odometry_bridge_node.cpp:144-155,636-639`.
- **Xử lý tạm thời:** `threads.csv` ghi cả hai và đánh dấu legacy là alternative;
  pipeline scenario 12 dùng external bridge vì đó là đường `/lio/odometry_propagated`
  → `/fmu/in/vehicle_visual_odometry`.

## OQ-A3-04 — Runtime timing artifacts có được phép dùng khi base SHA khác?

- **Câu hỏi:** Có artifact được commit/pin lại từ chính `7e0b850` cho mapping,
  solve, commit, emergency brake và `/clock` không?
- **Bằng chứng:** `NOMINAL_TIMING.csv` và `CLOCK_RELATION.md` có số đo hữu ích
  nhưng `BASE_PROVENANCE.md` ghi base SHA khác target. Vì vậy chỉ transport
  diagnostics được chép nguyên số, còn product budgets là `NOT_MEASURED`.
- **Rủi ro:** Không được dùng các số này để claim runtime qualification hoặc để
  tune threshold.

## R1 disposition

- F-04/F-05/F-06 now have the static verdict **NO_CYCLE** in
  `findings.md`/`lock_order.md`. The remaining question is bounded blocking and
  runtime contention, not an unresolved source-level cycle in the inspected
  graph.
- HG-001 authority remains **OPEN** with the project owner: the exact conflict
  is A* 40/80 vs runtime 30/60, solve 180 vs typed 80, and future-state lead
  200 vs typed/runtime 400 ms. This revision deliberately makes no choice and
  changes no threshold.
