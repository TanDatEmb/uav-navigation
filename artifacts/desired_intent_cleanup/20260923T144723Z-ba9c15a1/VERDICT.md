# Desired intent cleanup verdict

Runtime validation is pending for commit `c9006768`. The final verdict will be set only after the matched nominal and fault SITL runs are analyzed against the pinned `ba9c15a1` reference.

Source and deterministic gates currently show one `DesiredPlanningIntent` value under the existing RuntimeNode input lock; its independent facts are goal, monotonic revision, and one planning transition enum. `ExecutionAuthority` remains the sole active/staged command owner. `MissionProgress`, WorldModel, and PX4 adapter ownership did not move. The old RuntimeNode desired/active names and execution compatibility projections have been removed. The initial full Release build passed 23 packages, focused CTest passed 31 targets, Python passed 390 tests with one skip, and the three static guards passed. These are component and static evidence only.

The runtime gate remains open: three consecutive `long_featured` seed-0, tracking-off mission completions; nominal command-admission gap relative to the existing `19.894/20.030/20.093 ms` reference; no nominal lease, Hold, identity, or continuity regression; Core pause, BACKUP recovery, and terminal STOP fault observations. The focused runs are not flight qualification.
