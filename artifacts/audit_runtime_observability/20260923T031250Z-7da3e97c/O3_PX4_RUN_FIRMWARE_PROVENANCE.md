# O3 — PX4 run firmware provenance

| Requested field | Exact observation |
|---|---|
| `RUN_FIRMWARE_SHA` | `deaff86ee335dd697677bcfc2415a23878e1b895` in all 18 analyzed manifests (14 historical plus four focused smoke runs) and live `/home/letandat/Dev/Autopilot` checkout |
| `RUN_FIRMWARE_DIRTY_PATCH_HASH` | `d492bca20bd947a3d24c6657197d3cb84baf534a069825df5d320ff6139758b7` (SHA-256 of captured `provenance/external_px4_tracked.diff`) |
| Live recursive tracked diff hash | **same** `d492bca...` with `git diff --submodule=diff`; ordinary `git diff` is only 1,496 bytes and misses submodule working-tree content |
| Captured dirty status hash | `4165f8bc803e4dbc91afb1a5b198c5d914bea455e5d91ec0ac3165983cf02546` |
| `RUN_BINARY_HASH` | `e440bd77fbacdc422eaafdb168fec01554298d545f11e2b004a640b04e324ff9`, captured and live `build/px4_sitl_default/bin/px4` match |
| `REPRODUCIBLE_SOURCE` | **NO** for complete build-input closure |
| Product firmware baseline pinned | **NO / UNRESOLVED** |

`FACT_FROM_EXISTING_RUNTIME_ARTIFACT` Each run records SHA, status, recursive tracked patch and executable hash in `metadata.json:build_provenance.external_px4`. The recorded binary identity strongly pins *which executable file* was used in the run. The checkout's tracked recursive diff still matches captured bytes. But untracked directories (`boards/modalai/voxl2/src/lib/`, `src/lib/rl_tools/`, `src/modules/mc_raptor/`, optical-flow plugin, Micro-XRCE-DDS-Client-v3) were not snapshotted as a complete immutable source closure. The binary mtime predates the run; the manifest does not prove it was compiled from the checkout's current dirty diff. It is therefore incorrect to claim exact rebuildable firmware source from SHA+dirty patch+binary hash. We can analyze the *associated run checkout* and attach a binary-specific uncertainty; any behavior depending on an unrecorded build input remains unproved. No PX4 source or binary was changed.

`FACT_FROM_TARGET_CODE` The product repo pins `px4_msgs` and `px4_ros2_interface_lib` gitlinks, but `tools/runtime/runner.py` selects a local `PX4_DIR`; no product-wide PX4 firmware SHA/build contract was found (also documented in the preceding audit's `PROVENANCE.md`). This unresolved product baseline is separate from the strong run-specific binary identity. The associated checkout's DDS YAML publishes `/fmu/out/mode_completed`, `/fmu/out/vehicle_command_ack`, `/fmu/out/vehicle_status` and receives `/fmu/in/vehicle_command`; existing bag selection omitted the first three protocol topics. Its dirty DDS patch can change telemetry topic availability.
