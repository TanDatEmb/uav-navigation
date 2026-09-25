# Lock ownership (pre-mutation baseline and target)

Baseline execution-internal locks: `ExecutionEpisode::mutex_` and `ExecutionTimelineStore::mutex_` (2). `NavigationRuntimeNode` also serializes localization, input and command-lease transitions; these are separate domain/boundary locks and are excluded from the execution-internal count. The command sampler snapshots the timeline under its lock and evaluates the immutable polynomial after releasing it. World recertification prepares bundle copies outside the store lock. These timing properties must survive the refactor.

Target execution-internal locks: one `ExecutionAuthority::mutex_` (1). Its critical section may check identities/versions and swap prepared immutable pointers or bounded lifecycle tags. Planner solve, candidate validation, world sweep, polynomial evaluation, ROS publish and file I/O must remain outside except the already-bounded final publication callback for exact linearization. Any measured lock wait is reported separately from end-to-end publication latency.

No claim of performance improvement follows from reducing lock count alone. Before/after p50/p95/p99/max and handoff gap require a matched runtime run.
