# PX4 MID-360 Flight Observability Harness

Each start creates an isolated directory under
`.artifacts/simulation/px4-mid360-YYYYMMDD-HHMMSS/`; `latest` is an atomic
symlink to the newest session. Historical sessions are not removed on start.

```bash
make sim-px4-mid360
make sim-px4-mid360-check
make sim-px4-mid360-stop
make sim-px4-mid360-report
```

Use `make sim-px4-mid360-headless` without Gazebo GUI/RViz. Prune explicitly
with `make sim-px4-mid360-clean KEEP_SESSIONS=10`.

The implementation is split by responsibility:

- `session_manager.py` and the start/stop scripts own lifecycle and PID groups.
- `sim_observer.py` is the ROS boundary and continuously writes stream/sync CSV.
- `pointcloud_probe.py` performs bounded layout-aware XYZ sampling.
- `gazebo_probe.py`, `process_probe.py`, and `ros_graph_probe.py` collect
  subsystem-specific evidence.
- `observer_core.py` contains the pure state machine and event lifecycle.
- `snapshot_collector.py` captures best-effort evidence without GDB by default.
- `report_generator.py` creates `summary.json` and `REPORT.md`.

Thresholds are centralized in `config/px4_mid360_observer.yaml`. Generated
sessions are machine-local evidence and remain intentionally untracked.
