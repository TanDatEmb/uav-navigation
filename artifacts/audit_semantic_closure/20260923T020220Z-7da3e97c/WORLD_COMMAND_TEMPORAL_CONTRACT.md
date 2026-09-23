# Blocker D — certified world to command lease

**Verdict: BLOCKED_BY_RUNTIME_EVIDENCE.** Exact target gates and a candidate temporal contract are identified; callback/transport/recertification tail distributions are absent. A heartbeat policy change is therefore not approved.

## Distinct current clocks and bounds

| Boundary | TARGET value / source | Clock and owner | Current failure action |
|---|---|---|---|
| mapping snapshot cadence | 0.10 s `planning_timing.hpp:21` | world observation source ROS time; mapping worker | admission blocks if stale |
| latest world source age | 0.50 s default `navigation_runtime_node.cpp:678-680,738-756` | ROS `now` minus observation stamp; runtime | `publishCommand` cancels worker, suspends episode command, publishes nothing (`:7696-7720`) |
| planning period / solve deadline | 0.10 s / 0.08 s `planning_timing.hpp:9-16` | timer/steady deadline; runtime/worker | fallback/deadline outcome, not world lease |
| command sample period | 0.02 s `planning_timing.hpp:17-19` | wall timer; runtime | no automatic proof of WCET |
| candidate validity | `CandidateBundle::valid_from_ns/valid_until_ns`; admission sets end to min(declared end, activation + maximum age) (`navigation_runtime_node.cpp:2646-2648`, `candidate_bundle.hpp:126-132`) | ROS execution validity; bundle owner | reject expired sample/admission; latest-world freshness checked separately |
| published `NavigationCommand.valid_until` | command stamp + 0.10 s `planning_timing.hpp:19`, runtime `:8500-8504` | ROS clock; runtime | adapter rejects expired command |
| adapter receive lease | 0.10 s fixed `navigation_mode_node.cpp:118-155,2639-2659` | local ROS receive time; adapter | safety stop / PX4 handover path |
| adapter state age | 0.20 s fixed `navigation_mode_node.cpp:120-155` | source + receive witnesses; adapter | invalid state, no setpoint |
| Hold retry | 250 ms after callback failure `navigation_mode_node.cpp:2924-2931` | steady clock; PX4 boundary | retry if pending |

`FACT_FROM_TARGET_CODE` latest world and active bundle certificate are distinct. Mapping callback validates active/pending against a new world, atomically publishes world identity and retains or revokes bundles (`navigation_runtime_node.cpp:~940-1305`). On valid recertification of a suspended exact bundle it resumes without replacing bundle identity (`:1237-1294`); on invalidation it fails closed to PX4 Hold if no safe brake remains (`:1298-1304`). `ExecutionEpisode::suspendCommand` only clears availability (`execution_episode.hpp:~230-235`). New world arrival does not semantically require invalidating a safe active trajectory before validation; current code performs synchronous decision at world publication, not the proposed asynchronous lease policy.

## Candidate temporal contract (policy, unapproved)

`t_w` world source stamp → certified interval `[t_c_start,t_c_expiry]` on active immutable bundle → candidate admission at `t_a` → sampled reference at `t_r` with short `valid_until` → adapter receive `t_rx` and own state freshness → PX4 setpoint execution. Preserve source and receive timestamps; do not compare ROS, steady, and PX4 boot-time values without explicit conversion and clock-reset handling.

While Core owns command authority and a certificate is valid, every command period should produce an explicit reference or an explicit transfer/failure state; transport silence must mean upstream unavailable, not merely waiting for mapping. At certificate expiry or invalidation, transition atomically to a *certified* recoverable brake, committed stop, or PX4 Hold request. A newer world can recertify concurrently only while the old certificate remains valid under its declared scope. This is a proposed contract, not supported by measured WCET or the current `suspendCommandForWorldFreshness()` behavior.

**Q-D1:** TARGET latest-world gate allows up to 0.50 s source age, plus active bundle `valid_until`; no evidence supports a longer continuation. **Q-D2:** No automatic immediate revocation just because revision increments; validate existing trajectory against new world, and revoke if unsafe. **Q-D3:** Conceptually yes, while old certificate is still valid; TARGET publication is transaction-coupled, so async design needs new proof. **Q-D4:** explicit certified brake/stop or Hold transfer, not silent lease expiry; current code may publish nothing. **Q-D5:** under the proposed invariant, no; current TARGET intentionally does on stale world. This conflict is a policy change. **Q-D6:** downstream 100 ms means maximum source command age/receive silence for *last admitted reference*, never proof that world evidence is current or PX4 has assumed Hold.

Required inequality, with no invented values: `p99.9/worst credible world detection + mapping queue + recertification + commit + command scheduling + transport + adapter admission < remaining certified interval − stop/handover reserve`. Likewise `p99.9 command callback + queue + transport jitter < 100 ms receive lease` and `state pipeline age < 200 ms state gate`. These are necessary relationships, not demonstrated bounds. A single 0.08 s solve deadline does not bound map recertification or all callbacks.

- **Current behavior:** stale latest world suppresses publication; new mapping snapshot may recertify and resume exact bundle.
- **Desired target behavior:** explicit certificate-backed authority/heartbeat and explicit expiry transition, conditional on timing proof and braking policy.
- **Information preserved:** latest immutable world, bundle certificate identity/expiry, recertification job identity, command source/receive lease, state freshness, handover disposition.
- **Information derivable:** `world_valid`, `recertify_pending`, `command_suspended` only when certificate/job/execution variants cover their independent temporal meaning; otherwise unresolved.
- **Behavior intentionally lost:** silence as a recoverable mapping-wait signal would be removed if policy approved.
- **New risk:** publishing a heartbeat against expired evidence; requiring explicit stop can exceed remaining safe window unless reserve and scheduler tails are measured.
- **Required product tests:** clock jumps/reset, delayed/reordered world, stale mapping while odometry fresh, recertification success/failure at expiry boundary, queue/transport saturation, 100 ms lease loss, brake/Hold handover with real PX4.

`RUNTIME_UNVERIFIED`: no representative recertification/queue/transport tail distribution in this audit; no full SITL or flight qualification performed. Thresholds remain unchanged.
