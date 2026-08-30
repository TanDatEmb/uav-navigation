# Runtime validation

This is the copy-and-run guide for ROS 2, FAST-LIO, Gazebo and PX4 External
Mode validation in this repository. A successful launch or a single completed
flight is not, by itself, an acceptance result.

## 1. Required setup

The supported environment is Ubuntu with ROS 2 Jazzy, Gazebo Harmonic, Python
3 and a PX4 checkout with SITL available. The default PX4 path is
`$HOME/Dev/Autopilot`; set `PX4_DIR` explicitly if yours differs.

```bash
cd /home/letandat/Dev/uav-navigation
source /opt/ros/jazzy/setup.bash
export PX4_DIR="$HOME/Dev/Autopilot"
test -d "$PX4_DIR" || { echo "PX4_DIR does not exist: $PX4_DIR" >&2; exit 2; }
command -v gz
command -v python3
```

Do not run two SITL sessions in this workspace at the same time. The runner
uses one workspace-owned process group and the canonical runtime lock. Dataset
replay uses ROS domain 43 by default; SITL uses domain 42 and XRCE UDP port
8892. For deliberate isolation, use the `UAV_NAV_*` variables below.

## 2. Build and local tests

Always rebuild after changing C++, launch files, runtime configuration or
message contracts. `make build` writes the authoritative install manifest;
this prevents a stale-manifest/provenance error from being confused with a UDP
or simulator failure.

```bash
cd /home/letandat/Dev/uav-navigation
source /opt/ros/jazzy/setup.bash
make build
make test
```

`make test` is a component/contract suite, not an end-to-end flight verdict.
Before a map run, confirm the manifest and workspace state:

```bash
git rev-parse HEAD
git status --short
python3 - <<'PY'
import json
from pathlib import Path
p = Path("install/.uav_navigation_build_manifest.json")
print(json.dumps(json.loads(p.read_text()), indent=2))
PY
```

## 3. Which command to use

`make sim` starts an interactive PX4/Gazebo/RViz session and does not publish
an automatic mission. `make sim-check` is a legacy headless offboard smoke
workflow. The product External Mode acceptance command is:

```bash
PX4_DIR="$HOME/Dev/Autopilot" make external-mode-check
```

The GUI equivalent is:

```bash
PX4_DIR="$HOME/Dev/Autopilot" make external-mode-gui
```

The GUI command accepts the same selectors and launches RViz. To require a
manual takeoff/arm in the GUI session:

```bash
PX4_DIR="$HOME/Dev/Autopilot" MANUAL_TAKEOFF=1 \
  MAP_SCENE=sanity_open TEST_CASE=positive MOTION_PRESET=nominal \
  make external-mode-gui
```

Useful lifecycle commands are:

```bash
make status
make stop
make clean
```

`make stop` only stops process groups recorded as owned by this workspace.

## 4. Canonical scene selectors

The Make interface exposes stable scene families. `TEST_CASE` and
`MOTION_PRESET` select a variant when that combination exists; otherwise the
runner resolves the declared fallback in `config/runtime/map_profiles.yaml`.

| Variable | Accepted values | Purpose |
|---|---|---|
| `MAP_SCENE` | `sanity_open`, `structured_obstacle`, `long_route`, `tunnel`, `clutter`, `planner_negative`, `navigation_generalization` | Canonical scene family |
| `TEST_CASE` | `positive`, `degenerate`, `detour`, `no_path`, `comprehensive` | Mission/behavior variant |
| `MOTION_PRESET` | `nominal`, `slow`, `fast` | Motion variant |
| `MAP_SEED` | Integer | Deterministic stochastic-map seed |
| `SPEED_CAP_MPS` | Finite positive number | Per-run planner/tracker cap |
| `PX4_DIR` | Absolute path | PX4 source checkout |
| `UAV_NAV_ROS_DOMAIN_ID` | `0..232` | Explicit SITL ROS domain |
| `UAV_NAV_XRCE_PORT` | `1024..65535` | Explicit MicroXRCEAgent UDP port |

The runner also supports equivalent `--map-scene`, `--test-case`,
`--motion-preset`, `--map-seed`, `--speed-cap-mps`, `--ros-domain-id` and
`--xrce-port` options. `MAP_PROFILE` is retained for legacy profile names.

## 5. Complete positive map matrix

The following is the full `mission_complete` profile matrix declared by the
map registry. Run profiles sequentially from the repository root. The explicit
`SPEED_CAP_MPS` values are reproducible screening values, not claims that the
vehicle achieved those speeds.

### Canonical scene matrix

```bash
cd /home/letandat/Dev/uav-navigation
source /opt/ros/jazzy/setup.bash
export PX4_DIR="$HOME/Dev/Autopilot"
make build

MAP_SCENE=sanity_open TEST_CASE=positive MOTION_PRESET=nominal \
  SPEED_CAP_MPS=1 PX4_DIR="$PX4_DIR" make external-mode-check

MAP_SCENE=structured_obstacle TEST_CASE=positive MOTION_PRESET=nominal \
  SPEED_CAP_MPS=1 PX4_DIR="$PX4_DIR" make external-mode-check
MAP_SCENE=structured_obstacle TEST_CASE=detour MOTION_PRESET=nominal \
  SPEED_CAP_MPS=1 PX4_DIR="$PX4_DIR" make external-mode-check

MAP_SCENE=long_route TEST_CASE=positive MOTION_PRESET=nominal \
  SPEED_CAP_MPS=2 PX4_DIR="$PX4_DIR" make external-mode-check
MAP_SCENE=long_route TEST_CASE=positive MOTION_PRESET=slow \
  SPEED_CAP_MPS=2 PX4_DIR="$PX4_DIR" make external-mode-check
MAP_SCENE=long_route TEST_CASE=positive MOTION_PRESET=fast \
  SPEED_CAP_MPS=5 PX4_DIR="$PX4_DIR" make external-mode-check

MAP_SCENE=tunnel TEST_CASE=positive MOTION_PRESET=nominal \
  SPEED_CAP_MPS=1 PX4_DIR="$PX4_DIR" make external-mode-check
MAP_SCENE=clutter TEST_CASE=positive MOTION_PRESET=nominal MAP_SEED=11 \
  SPEED_CAP_MPS=1 PX4_DIR="$PX4_DIR" make external-mode-check

MAP_SCENE=navigation_generalization TEST_CASE=comprehensive \
  MOTION_PRESET=nominal SPEED_CAP_MPS=3 PX4_DIR="$PX4_DIR" \
  make external-mode-check
```

### Direct profile matrix

Use this matrix when the benchmark requires every concrete profile, including
three-pillar, ablation and long-leg profiles kept behind the legacy
`MAP_PROFILE` surface:

```bash
cd /home/letandat/Dev/uav-navigation
source /opt/ros/jazzy/setup.bash
export PX4_DIR="$HOME/Dev/Autopilot"
make build

for profile in \
  occlusion_featured tunnel_irregular forest_clutter long_featured long_open \
  long_open_slow long_three_pillars long_three_pillars_speed \
  long_three_pillars_multiwaypoint long_open_featured_speed \
  long_open_featured_core_60 long_open_featured_core_60_pv \
  single_pillar_speed single_pillar_speed_pv navigation_generalization; do
  MAP_PROFILE="$profile" TEST_CASE=positive MOTION_PRESET=nominal \
    MAP_SEED=11 SPEED_CAP_MPS=1 PX4_DIR="$PX4_DIR" \
    make external-mode-check
done
```

For the declared high-speed three-pillar matrix:

```bash
for speed in 2 3 4 5 6 8; do
  MAP_PROFILE=long_three_pillars_speed TEST_CASE=positive \
    MOTION_PRESET=fast SPEED_CAP_MPS="$speed" PX4_DIR="$PX4_DIR" \
    make external-mode-check
done
```

For the nine-waypoint three-pillar mission:

```bash
for speed in 3 4 5; do
  MAP_PROFILE=long_three_pillars_multiwaypoint TEST_CASE=positive \
    MOTION_PRESET=fast SPEED_CAP_MPS="$speed" PX4_DIR="$PX4_DIR" \
    make external-mode-check
done
```

## 6. GUI commands for each map family

Run only one GUI session at a time:

```bash
PX4_DIR="$HOME/Dev/Autopilot" MAP_SCENE=sanity_open \
  TEST_CASE=positive MOTION_PRESET=nominal SPEED_CAP_MPS=1 \
  make external-mode-gui

PX4_DIR="$HOME/Dev/Autopilot" MAP_SCENE=structured_obstacle \
  TEST_CASE=detour MOTION_PRESET=nominal SPEED_CAP_MPS=1 \
  make external-mode-gui

PX4_DIR="$HOME/Dev/Autopilot" MAP_SCENE=long_route \
  TEST_CASE=positive MOTION_PRESET=slow SPEED_CAP_MPS=2 \
  make external-mode-gui

PX4_DIR="$HOME/Dev/Autopilot" MAP_SCENE=tunnel \
  TEST_CASE=positive MOTION_PRESET=nominal SPEED_CAP_MPS=1 \
  make external-mode-gui

PX4_DIR="$HOME/Dev/Autopilot" MAP_SCENE=clutter MAP_SEED=11 \
  TEST_CASE=positive MOTION_PRESET=nominal SPEED_CAP_MPS=1 \
  make external-mode-gui

PX4_DIR="$HOME/Dev/Autopilot" MAP_SCENE=navigation_generalization \
  TEST_CASE=comprehensive MOTION_PRESET=nominal SPEED_CAP_MPS=3 \
  make external-mode-gui
```

For a manually isolated GUI session:

```bash
UAV_NAV_ROS_DOMAIN_ID=52 UAV_NAV_XRCE_PORT=8893 \
  PX4_DIR="$HOME/Dev/Autopilot" MAP_SCENE=structured_obstacle \
  TEST_CASE=detour MOTION_PRESET=nominal SPEED_CAP_MPS=1 \
  make external-mode-gui
```

## 7. Negative and malformed-input checks

Negative scenarios are safety checks, not positive map acceptance. They must
stop safely and must never be included in the positive matrix:

```bash
MAP_SCENE=planner_negative TEST_CASE=no_path MOTION_PRESET=nominal \
  PX4_DIR="$HOME/Dev/Autopilot" make external-mode-check

MAP_SCENE=tunnel TEST_CASE=degenerate MOTION_PRESET=nominal \
  PX4_DIR="$HOME/Dev/Autopilot" make external-mode-check

MAP_SCENE=structured_obstacle TEST_CASE=degenerate MOTION_PRESET=nominal \
  PX4_DIR="$HOME/Dev/Autopilot" make external-mode-check
```

Expected negative behavior is a verified fail-closed transition, safe PX4
handover/hold and zero collision. A negative run without evidence is
inconclusive, not PASS.

## 8. Runtime graph and frame contract

```text
/lidar/points + /lidar/imu
  -> FAST-LIO
  -> /lio/odometry_corrected
  -> /lio/odometry_propagated
  -> /lio/mapping_observation
  -> navigation_runtime_node (mapping + planning)
  -> /navigation/navigation_command
  -> px4_navigation_external_mode
  -> /fmu/in/trajectory_setpoint
```

The planner state frame is `lio_odom`. FAST-LIO and the bridge use
`lio_odom -> base_link`; the PX4 boundary converts ENU/FLU to NED/FRD. Truth
odometry is evaluation-only and must never be used as a control input.

Required diagnostic streams include `/lio/health`, `/lio/diagnostics`,
`/navigation/diagnostics`, `/navigation/navigation_command`,
`/navigation/mode_status`, `/navigation/mission_complete`, and PX4
trajectory setpoints. Propagated odometry is required for active navigation.

## 9. Artifacts and report commands

Every runner invocation creates:

```text
.artifacts/runtime/external-mode-check-<timestamp>-<pid>/
```

Important files are `scenario.json`, `scenario_config.yaml`, `samples.jsonl`,
`monitor.json`, `runtime.json`, `processes.json`, `report.json`,
`REPORT.html`, and component logs. The only public report tool is:

```bash
python3 tools/runtime/report.py \
  --session .artifacts/runtime/external-mode-check-<timestamp>-<pid> \
  --workflow external-mode \
  --config config/runtime/sim.yaml \
  --workspace /home/letandat/Dev/uav-navigation \
  --px4-dir "$HOME/Dev/Autopilot"
```

`make status` shows the latest session. `make stop` is the safe cleanup path.
There is no `REPORT.md` artifact.

## 10. Acceptance rules

A positive map is accepted only when all of the following are present in the
same artifact and consistent with the manifest:

- all expected waypoints and mission-complete evidence;
- finite, fresh and correctly framed corrected/propagated LIO streams;
- no timestamp regression, unexplained transport blackout or continuity fault;
- no collision and sufficient truth clearance;
- bounded cross-track error, altitude error and command gaps;
- valid planner V/A/J/flatness/corridor certificates;
- no unexpected PX4 failsafe or External Mode exit; and
- clean process-group cleanup and report provenance.

The locked qualification matrix is
`config/runtime/planning_stability_qualification.yaml`. Each deterministic
open/corner/obstacle scenario must pass 10 consecutive runs at 1, 3 and 5 m/s;
stochastic clutter requires at least 30 seeds. Pass-through coverage is
0/30/45/60/90 degrees plus the nine-waypoint chain. Runs at 6 or 8 m/s are
characterization only. Keep p50/p95/p99 and maximum planner, mapping, transport
and tracking metrics; do not retune a hard gate from one run.

The fault matrix covers perception presence semantics, disjoint/intersecting
world revision, route/epoch supersede, forced cancellation, worker overload,
one-shot brake, stale propagated state and moving command loss. Its named
component/integration test is the minimum RED/GREEN evidence; qualification
also requires the corresponding SITL injection artifact where the case crosses
the PX4 boundary.

Every finalized report writes three owner-separated machine-readable files:
`perception_timeline.jsonl`, `planning_timeline.jsonl`, and
`execution_timeline.jsonl`. `report.json.qualification_timelines` records their
paths and event counts. Missing events remain an empty timeline and must never
be synthesized as successful evidence.

When reading LIO plots, keep these streams separate:

- `corrected_odometry`: lower-rate LiDAR correction, commonly about 10 Hz;
- `propagated_odometry`: high-rate execution stream, commonly about 50 Hz;
- `ground_truth_odometry`: simulator truth for evaluation only.

Connecting sparse corrected samples can visually produce a sawtooth. That is
not sufficient evidence of an estimator or frame-conversion bug. Investigate
LIO only when the sawtooth is accompanied by timestamp regression/gaps, large
correction jumps, propagated-vs-truth residual growth, or a freshness/
continuity gate failure. Correlate `samples.jsonl`, `report.json`,
`monitor.json`, PX4 logs, LIO logs and planner logs by source timestamp.

## 11. Dataset replay

Dataset replay is a separate estimator/mapping health check; it does not
certify PX4 tracking or flight safety:

```bash
DATASET=aist-mid360-drive RATE=1.0 \
  DATASET_SHADOW_GOAL_M=5.0 make dataset-check

DATASET=aist-mid360-drive RATE=1.0 \
  DATASET_SHADOW_GOAL_M=5.0 make replay
```

Use `DATASET_ROS_DOMAIN_ID=43` or another isolated domain for parallel replay.
Do not start `make build` while a dataset or SITL session is active.
