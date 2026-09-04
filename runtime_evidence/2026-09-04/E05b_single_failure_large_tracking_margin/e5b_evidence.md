# E5b evidence

Status: **BLOCKED / not a valid E5b witness**.

The run retained MAIN commands and a bag, but `injected_replan_failure=0` in
the captured planner trace. The runtime stopped after a native hot-replan
failure with no valid retained suffix (`backup=0`, `anchor_error=inf` in the
node log). Therefore the requested large-margin single-failure causal claim
was not tested.

Raw evidence: `metadata.json`, `samples.jsonl`, `scenario.jsonl`,
`rosbag/`, and `logs/navigation_runtime_node_*.log` in this directory.
