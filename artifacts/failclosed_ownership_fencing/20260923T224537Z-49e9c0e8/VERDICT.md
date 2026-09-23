# Repair verdict

**Pending validation.** The final verdict will be one of `FAILCLOSED_OWNERSHIP_REPAIRED`, `PARTIAL_REPAIR`, `REPAIR_BLOCKED`, or `REGRESSION`, selected only after clean build, deterministic tests, static guards and three matched nominal SITL runs. Source changes alone do not satisfy the merge gate.

The branch preserves the five independent owners and changes only the right of an asynchronous callback to destroy current execution. Exact execution and causal state/world/solve evidence must still match at mutation; otherwise the callback discards its result. Policy and threshold gates remain with existing owners.
