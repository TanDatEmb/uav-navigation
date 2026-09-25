# Isolated World source fault gate

The opt-in runner argument `--world-observation-fault-ms {520,700}` is test/SITL-only and defaults to disabled. When enabled, the runner changes only the generated `navigation_runtime.registered_scan_topic` to `/test/world_gate/mapping_observation` and starts a separate relay before NavigationRuntime.

The relay subscribes to `/lio/mapping_observation` (`RegisteredScan`), `/navigation/navigation_command`, `/clock`, `/lio/odometry_propagated`, `/lio/health`, and `/diagnostics`. It republishes only `RegisteredScan`. It schedules one fault after 100 distinct `STATUS_READY` command sample IDs, with a one-second simulation-time delay, then drops scans for the requested simulation-time duration and resumes forwarding. The source scan sequence/stamp is logged for each dropped scan. All host steady timestamps are recorded separately.

The fault is disabled for all ordinary runner calls. It cannot publish a command, alter health, pause any process, block DDS, or invoke product authority APIs. Runner result finalization checks that the gate armed once, dropped source observations, observed unaffected streams progressing, restored forwarding for the 520 ms case, and that runtime mapping input was configured to the gated topic.
