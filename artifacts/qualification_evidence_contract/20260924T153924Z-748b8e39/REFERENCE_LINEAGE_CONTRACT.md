# Reference lineage contract

The tracking reference is a sampled NavigationCommand, not the desired goal, nominal route or current planner result. A reference record must retain source session/runtime, mission/request, localization and goal epoch, active bundle generation, sample ID, frame/source clock and the exact world/certificate identity. Its producer authorization and downstream PX4 input observation are separate witnesses.

`reduce_lifecycle()` grants a valid raw reference ID `(request_id, bundle_generation, sample_id)` only from a complete exact request/export/activate/authorize/publish transaction. `_reference_lineage_status()` checks **every raw executable reference**, including duplicate-source-time heartbeats, against that set. The time-indexed tracking copy may collapse exact heartbeats; lineage never does.

No reference can be assigned a current route/world revision merely because an observer saw one later. Ground truth is an independent measured stream with its own frame and clock witness; it is not a NavigationCommand generation. A nominal route/guidance polyline is descriptive and does not replace active execution lineage.
