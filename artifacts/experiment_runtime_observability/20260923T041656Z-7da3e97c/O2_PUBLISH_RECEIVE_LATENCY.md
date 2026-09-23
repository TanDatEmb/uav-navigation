# O2 exact command correlation

**Observed:** C's raw audit bag contains 2701 `COMMAND_PUBLISH` and 2683 `COMMAND_RECEIVE` records. The seven-field command key in `COMMAND_KEY_CONTRACT.md` gives **2680 exact one-to-one pairs**. The other **21 publisher keys** are ambiguous or unpaired (including initial recorder loss and the 4 runtime audit drops); they carry no latency conclusion. Of the exact pairs, **2340** have an exact-key later `PX4_INPUT_SETPOINT` first-use diagnostic witness and **340** do not; the entire publish-key table has **361** without first use. Neither receive admission nor diagnostic first use by itself proves PX4 flight acceptance.

Both relevant launch paths set `use_sim_time=true`. C's bag contains **11,008** `/clock` samples from **33.024 s** to **77.052 s**, with no zero or backward sample. `C_SHARED_SIM_CLOCK_11008_MONOTONE_SAMPLES` is the run-specific proof ID passed to `normalize.py`. The parser never subtracts steady-clock timestamps from different processes. Numeric values below are ROS simulation-clock deltas, with coarse clock updates; **0 ms means the events fell in one simulated tick, not zero transport time**.

| C exact-key boundary | n | p50 | p95 | p99 | max | ≥100 ms |
|---|---:|---:|---:|---:|---:|---:|
| Publish enter → receive entry | 2680 | 0 ms | 0 ms | 0 ms | 12 ms | 0 |
| Publish return → receive entry | 2680 | 0 ms | 0 ms | 0 ms | 12 ms | 0 |
| Receive entry → first diagnostic setpoint use | 2340 | 12 ms | 12 ms | 12 ms | 12 ms | 0 |

The local adapter receive-entry → callback-return/admission-result duration, measured on **one process-local steady clock**, was p50/p95/p99/max **10.608/22.512/46.909/150.938 µs** over 2683 receive records. This is callback duration up to the emitted admission outcome, not cross-process transport latency.

`O2_PUBLISH_RECEIVE.csv` preserves each exact/ambiguous key, process incarnations, admission result and first-use status. `COMMAND_PUBLISH.outcome=1` means the `rclcpp::publish()` call returned; `COMMAND_RECEIVE.outcome=1` means the adapter admitted the product command. The first-use witness comes from the existing `PX4_INPUT_SETPOINT` diagnostic, not a new authority path. The raw command topic is also in the recorder.

The report's p99/max are measured only over paired, valid-clock events. They are not a transport latency distribution for all 2701 publications. Four internal runtime audit drops and initial recorder startup gaps prevent whole-run completeness. No shared-clock claim is made for the F1/F2/O1 repeats, whose normalizers intentionally retain identity-only pairing. O2 is sufficiently observable for exact diagnostic pairing in the captured C interval; quantitative authority cutover timing remains unqualified.
