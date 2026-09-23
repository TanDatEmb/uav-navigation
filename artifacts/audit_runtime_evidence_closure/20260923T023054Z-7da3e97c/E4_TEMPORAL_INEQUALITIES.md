# E4 — temporal inequalities and unmeasured reserve

The following are **necessary candidate relationships**, not approved bounds. Keep ROS source, local steady, recorder wall and PX4 boot time separate. An observed p99 cannot prove worst-case timing or a safe reserve.

| Boundary | Inequality | Evidence status |
|---|---|---|
| 100 ms downstream receive lease | command scheduling tail + publication/transport tail + adapter callback tail **< 100 ms − reserve** | `MEASURED`: command recorder gaps and diagnostic publish durations only; `UNOBSERVABLE`: publisher→adapter receive and callback tail; reserve is `POLICY_ASSUMPTION` not chosen. |
| certified world | detection + queue + recertification + commit + next command publish **< remaining certified authority − stop/Hold reserve** | `MEASURED`: mapping/revalidation diagnostic duration samples; exact snapshot→recert start/commit and certificate remaining window at event are unavailable. |
| 200 ms state gate | state producer + transport + queue **< 200 ms − reserve** | `MEASURED`: state source age at emitted adapter tracking updates; missing rejected attempts and source→receive pair. |

`FACT_FROM_PINNED_PRODUCT_SOURCE` A newer world does not automatically make an active candidate unsafe; TARGET validates/recertifies identity, and latest-world freshness is a separate 500 ms gate. At its failure, publication can intentionally stop while execution identity is retained. Command silence therefore presently has at least two meanings: recoverable wait for a fresh world and unavailable upstream command authority. That ambiguity must be resolved by a policy decision **after** lease/transport tails are known. The downstream 100 ms lease is the maximum permitted age/silence for an admitted command in the adapter; it is not a certificate lifetime or proof PX4 has entered Hold.

`RUNTIME_UNVERIFIED` No selected run increments the world-suspend counter, so the causal chain `last certified world → silence → adapter receive lease expiry → safety action → new world/recertification` has no runtime observation. A controlled L6 input-gap/recovery run must log world source and certificate expiry, runtime suppression reason, last command publish and adapter receive steady stamps, adapter transition/setpoint, new world publication and exact recertification commit. L5 contention and repeated same-profile timing runs are needed for a credible reserve. Do not inject a relaxed gate to manufacture completion.

Decision: `INSUFFICIENT_EVIDENCE` for “while navigation owns authority, emit an explicit ControlReference heartbeat; silence means upstream unavailable.” It remains an architecture proposal, not an implementation instruction or safety invariant of current TARGET.
