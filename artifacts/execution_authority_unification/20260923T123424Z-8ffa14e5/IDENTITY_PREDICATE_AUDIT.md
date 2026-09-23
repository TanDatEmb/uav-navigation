# Identity predicate audit (base `8ffa14e5`)

Every call to `desiredGoalIdentityMatchesLocked`, `executingCommandIdentityMatchesLocked`, or `sameGoalIdentity` in `navigation_runtime_node.cpp` is listed. Definitions at 1753 and 1773 are excluded. Line numbers are base-source anchors. `BOTH_REQUIRED` does not mean a generic equality guard: it is confined to terminal/renewal/admission policy where both identities participate.

| Base line | Class | Reason |
|---:|---|---|
| 1267 | EXECUTION_ONLY | recertified retained bundle identity |
| 2451 | EXECUTION_ONLY | current physical continuation |
| 2757 | DESIRED_ONLY | candidate admission scope |
| 2770 | BOTH_REQUIRED | terminal replacement requires exact predecessor and current desired goal |
| 3397 | BOTH_REQUIRED | terminal monitor is only for same desired and executing STOP |
| 4059 | DESIRED_ONLY | mission start heading scope |
| 4155 | EXECUTION_ONLY | stopped execution observation |
| 4191 | EXECUTION_ONLY | measured stop recovery |
| 4356 | EXECUTION_ONLY | stopped execution identity |
| 4519 | DESIRED_ONLY | mission completion target still current |
| 4625 | EXECUTION_ONLY | completion witness for committed bundle |
| 4663 | BOTH_REQUIRED | terminal successor transfer checks desired and completed execution separately |
| 4743 | BOTH_REQUIRED | new desired handover checked against completed predecessor |
| 4748 | BOTH_REQUIRED | restart for terminal handover only while both remain current |
| 4838 | BOTH_REQUIRED | terminal monitor requires desired and executing same goal |
| 5013 | DESIRED_ONLY | planned transition target |
| 5062 | BOTH_REQUIRED | same-identity renewal only when desired equals executing |
| 5662 | SUSPICIOUS_COUPLING | planner failure failClosed is gated by desired identity; inspect active predecessor retention |
| 5676 | DESIRED_ONLY | PlanFromRest restart request scope |
| 5695 | EXECUTION_ONLY | completion witness for active bundle |
| 5739 | SUSPICIOUS_COUPLING | planner failure handling checks desired identity while predecessor may still execute |
| 5868 | BOTH_REQUIRED | immediate candidate must equal desired and newly active execution |
| 5909 | BOTH_REQUIRED | committed terminal endpoint witness for desired goal |
| 5944 | DESIRED_ONLY | post-solve bookkeeping |
| 7112 | DESIRED_ONLY | retained validation callback target |
| 7132 | BOTH_REQUIRED | terminal monitor exact desired and execution equality |
| 7822 | EXECUTION_ONLY | retained active snapshot stability |
| 7835 | DESIRED_ONLY | retained desired snapshot stability |
| 7840 | BOTH_REQUIRED | terminal monitor exact same-identity scope |
| 7901 | EXECUTION_ONLY | retained bridge observation |
| 7928 | EXECUTION_ONLY | safety suffix observation |
| 7943 | EXECUTION_ONLY | retained BACKUP recovery event |
| 8507 | EXECUTION_ONLY | stopped hold sample |
| 8529 | EXECUTION_ONLY | terminal command invalidation |
| 8580 | EXECUTION_ONLY | sampled BACKUP transition |
| 8635 | EXECUTION_ONLY | completion active bundle witness |
| 8640 | BOTH_REQUIRED | mission terminal acceptance requires current desired plus active execution |
| 8682 | EXECUTION_ONLY | sampled execution completion |
| 8686 | BOTH_REQUIRED | desired mission completion requires exact active execution |

The two `SUSPICIOUS_COUPLING` planner failure sites require adversarial tests with desired N+1 and active N. This cut will not silently alter recovery semantics. Manual execution-identity comparisons in mapping/world callbacks and publication also need migration to one owner snapshot/token; they are included in the read/write mapping checkpoint.
