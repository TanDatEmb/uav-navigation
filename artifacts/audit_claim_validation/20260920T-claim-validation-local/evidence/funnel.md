# H7 — source funnel and alternative rejection points

This is the coarse source order for reviewed paths, not a count of observed requests. Each boundary can wait, supersede, cancel, reject with a typed reason, or retain an existing execution bundle.

| Boundary | Outcomes / evidence | Status |
|---|---|---|
| Goal ingress and acceptance | Runtime validates identity, epochs, route and measured state; desired goal and executing goal remain distinct. | Source reviewed; no workload counts. |
| Schedule eligibility / queue | Worker checks freshness, health, world snapshot, cancellation, timing and exact request key before enqueue/start. | Static path; no target trace. |
| Route guide / local support | Search can fail or return a partial route only under its explicit support guard; AABB directional support and guide length feed the governor. | Source reviewed; no matching world capture. |
| Corridor / candidate synthesis | Corridor, route-boundary/PVAJ constraints, MAIN synthesis and BACKUP seed are distinct stages. | Selected helpers tested. |
| Nominal governor | Checks measured PVAJ under BACKUP physical limits, then inspects descending fixed 16-sample speed grid. Caller returns FAILED before nominal endpoint MINCO synthesis if none qualifies; failure taxonomy separates budget exhaustion, physical envelope, stop synthesis and insufficient support. | Code fact; no reachable production false negative or cost reproduced/measured. |
| Candidate validate / commit | Newest-world certificate and identity/generation/deadline commit guards may reject after planner success. | Static path; selected gates tested. |
| Bundle activation / command sample | Desired, pending, committed, executing and sampled identities can differ. publishCommand repeats identity, epoch, state, world, bundle and command lease checks at exposure. | Static path; store tests cover selected races. |
| ROS publish / PX4 accept | ROS API publication is not adapter receipt. Adapter independently checks command contract, header/receive age, identity, world generation, frame, health, state freshness and tracking support. | No paired processes or PX4 acceptance evidence. |
| Setpoint / measured progress | Adapter may retain, transition, reject, safety-stop, stream stationary setpoints or schedule Hold. Mission progress has separate measured crossing and continuation gates. | Native MissionController counterexample only; no target trace. |

The artifact scan was bounded to artifacts/ and common *.ulg, *.db3, *.mcap, *trace*.jsonl, *safety*.log and *px4*.log patterns; no provenance-matched target workload log was found. KHÔNG ĐỦ DỮ LIỆU XẾP HẠNG BOTTLENECK THỰC TẾ. Source-path reachability and tests cannot rank frequency or CPU cost.

