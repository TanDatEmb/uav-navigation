# M1 acceptance checklist

This checklist defines repeatable gates. It does not record results from a
particular host or run.

| Gate | Command | Pass/fail condition | Generated artifact |
| --- | --- | --- | --- |
| Clean build | `make clean && make build` | Build exits zero with no compiler error | `build/`, `install/`, `log/` |
| Unit tests | `make test` | All discovered unit tests pass | `build/*/test_results/` |
| Static checks | `make check` | Linters and supported sanitizers exit zero | Tool-specific output under `build/` |
| Dataset inspection | `make dataset-inspect DATASET=<dataset>` | Manifest, bag, topics, types, and config validate | `.artifacts/datasets/<name>/<run>/run_manifest.json` |
| Dataset smoke | `make dataset-smoke DATASET=<dataset>` | Adapter, synchronization, deskew, and correction smoke gates pass | Run directory under `.artifacts/datasets/<name>/` |
| Full offline replay | `make dataset-run DATASET=<dataset>` | Replay completes, state remains finite, and required outputs exist | Summary, CSV files, logs, trajectory, and local registration map |
| Realtime ROS replay | `make dataset-ros DATASET=<dataset> RATE=1.0` | Replay completes without unbounded processing backlog | ROS run directory under `.artifacts/datasets/<name>/` |
| Map stress | Run stationary, pure-yaw, translation, and long-replay scenarios | Map respects the hard limit after maintenance without continuous pruning | Diagnostics, summary, and final local registration map |
