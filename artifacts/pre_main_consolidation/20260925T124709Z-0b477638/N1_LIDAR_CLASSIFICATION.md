# N1 LiDAR classification

Classification: **SIMULATION_INFRASTRUCTURE_LIMITATION**.

Session: `external-mode-check-20260925T092305-366082`; mission outcome COMPLETE; runtime input-stream verdict FAIL due a measured LiDAR timestamp/freshness violation. This is not hidden by the corrected C0-SW/C0-IFP report separation: C0-SW is PASS/eligible, C0-IFP is NOT_EVALUABLE, while runtime remains FAIL.

The first recorded temporal violation is the simulated LiDAR/registered-scan source stream: source stamps advance from 7.9 s to 8.5 s (600 ms), with 751.851 ms wall arrival gap. The simulator clock remained regular (250 Hz, 4 ms maximum observed gap); source timestamps had no regression or invalid values. LiDAR and registered-scan observer streams each reported one stale event. The bridge log reports 892 ingress, 892 published, zero drops, and zero conversion/publish failures. FAST-LIO diagnostics reported TRACKING/navigation valid and zero LiDAR drops; received and processed counts matched in the captured diagnostic witness.

The observed problem is missing simulated sensor updates across a source interval, not a malformed/stale LiDAR message consumed by the product, a product authority defect, or an evaluator-only rejection. The exact split between Gazebo sensor update scheduling and sensor-to-bridge transport is not available from this historical session because there is no independent per-sensor generation sequence/timestamp witness before bridge ingress. The first causal failure is nevertheless bounded to the simulated input boundary; do not attribute it more narrowly to Gazebo or DDS.

No product or evaluator change is made to suppress the runtime stream failure. No raw evidence is rewritten. N1 remains in the runtime denominator as a FAIL and is recorded as a simulation-input limitation for architecture integration; it is not C0-SW failure evidence.
