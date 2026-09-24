# Pre-admission `valid=0` evidence

The pinned prior nominal session `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260924T014351-704845` contains two adapter warnings:

- line 50 of the PX4 adapter log, ROS time `1790214293.235009879`: `planner backend command rejected before acceptance validation: valid=0 command_present=1`;
- line 97, ROS time `1790214322.249237958`: the same ambiguous log class.

That log combined a null message, contract-shape failure and time-window failure into one boolean. It does **not** prove which exact reason occurred. The new precheck runs `assessCommandContract` and `assessCommandTemporalLease` separately and emits `NavigationCommandRejection(stage, reason_code, disposition)` plus callback ROS/steady clocks and command identity. A malformed replacement still retains the prior accepted command. Focused runtime runs must report whether this historical class recurs and, if so, its exact typed reason; absence of recurrence is not claimed as a fix.

The new nominal N3 session `external-mode-check-20260924T031122-825633` recorded four typed rejections on `/navigation/command_rejection`:

| Request / bundle / sample | Stage | Reason | Disposition | Callback ROS ns | Command stamp ns | Valid-until ns |
| --- | --- | --- | --- | ---: | ---: | ---: |
| 1 / 2 / 79 | TEMPORAL_LEASE | NOT_YET_VALID | REJECT_RETAIN_PREVIOUS | 21848000000 | 21852000000 | 21952000000 |
| 2 / 3 / 310 | TEMPORAL_LEASE | NOT_YET_VALID | REJECT_RETAIN_PREVIOUS | 25544000000 | 25547999999 | 25648000000 |
| 2 / 4 / 667 | TEMPORAL_LEASE | NOT_YET_VALID | REJECT_RETAIN_PREVIOUS | 31256000000 | 31260000000 | 31360000000 |
| 4 / 10 / 1858 | TEMPORAL_LEASE | NOT_YET_VALID | REJECT_RETAIN_PREVIOUS | 50312000000 | 50316000000 | 50416000000 |

Each command arrived about 4 ms before its header stamp by the adapter callback ROS clock. The adapter retained its previous admission; 2817 later/other samples were admitted, mission completed, and the maximum admission inter-arrival gap was 40.328 ms. This explains **the new N3 rejection class**, not the exact two historical baseline events: the old log lacks the fields needed to prove they had this same cause. The branch closes the *ambiguity* by emitting stage, code, disposition and time/identity witnesses; it does not change the inclusive temporal predicate or make these commands valid.

Source proof: `navigation_command_contract.hpp` frozen-predicate unit comparison and `NavigationMode::onNavigationCommand` typed precheck. Runtime proof: N3 rejection topic and adapter log. Firmware consumption is not inferred from adapter admission.
