# Fault-gate isolation proof

## 430 ms recovery run

Session: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260925T091243-356218`. The runtime parameter snapshot remapped only `navigation_runtime.registered_scan_topic` to `/test/world_gate/mapping_observation`; runner gate validation is `PASS`.

The gate scheduled `DROP_MAPPING_ONLY` in simulation time `[22.615999999 s, 23.045999999 s)`. It observed 670 original input scans, forwarded 666, dropped 4 (source stamps 22.7, 22.8, 22.9, 23.0 s), then forwarded again at source stamp 23.1 s. Thus original RegisteredScan traffic continued while the Core mapping consumer stopped receiving it.

During the dropped interval, /clock, propagated odometry, typed health, Core diagnostics and adapter command admissions all advanced. At the first dropped scan the mode was ACTIVE and 168 adapter admissions had been observed; by forwarding restoration there were 192 admissions. Gate-final counts: clock 17,731; odometry 670; health 4,126 (4,117 valid); Core diagnostics 495; adapter admissions 2,905; mode status 1,169. Runtime topic remap was verified from the emitted parameter file. These are stream witnesses, not process-presence inference.

The fault gate forwards/drops RegisteredScan only. It does not publish or rewrite odometry, health, /clock, NavigationCommand or adapter admission. This run proves the isolation wiring used by the World stale recovery scenario.
