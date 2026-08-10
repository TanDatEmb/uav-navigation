# P1 acceptance results

Current verdict: `P1 NOT YET ACCEPTED`.

The automated dataset and SITL gates are complete on the current workspace
state. The remaining gate is a live interactive flight observation: rotate the
vehicle in `lio_odom` while watching the occupied/inflated voxel displays and
confirm that the map remains in the fixed `lio_odom` frame. The interactive
startup/RViz wiring was verified without arming or taking off.

## AIST MID360, rate 1.0

All three runs used dataset `aist-mid360-drive`, the same runtime profiles and
the same workspace state. They all reached `TRACKING`, completed cleanup, and
accepted 2756 corrections.

| Mode | Corrections | Deskewed clouds | Paired | Integrated | Scan p99 | Map update p99 | Verdict |
|---|---:|---:|---:|---:|---:|---:|---|
| `off` | 2756 | 0 | n/a | n/a | 78.1 ms | n/a | PASS |
| `publisher` | 2756 | 2756 | n/a | n/a | 72.4 ms | n/a | PASS |
| `full` | 2756 | 2756 | 2756 | 2756 | 70.8 ms | 58.4 ms | PASS |

The full mapping queue high-water bound was 1 and its maximum map update was
70.4 ms. LIO RSS stayed around 41--42 MiB; the reports also contain monitor
and replay RSS after the 30-second warm-up.

Artifacts:

- `off`: `.artifacts/runtime/dataset-20260810T040915-31957/REPORT.md`
- `publisher`: `.artifacts/runtime/dataset-20260810T042512-39777/REPORT.md`
- `full`: `.artifacts/runtime/dataset-20260810T043007-40524/REPORT.md`

The separate `RATE=2.0` stress attempt intentionally failed closed with input
queue pressure and drops; it is not part of the acceptance comparison.

## Headless SITL

`sim-check --mapping-mode full` passed the deterministic offboard scenario:
arm, offboard, takeoff, translation, yaw, return, landing and disarm all
completed with zero offboard loss events.

Artifact: `.artifacts/runtime/sim-check-20260810T043634-41280/REPORT.md`.

## Interactive RViz wiring

The interactive session reached `startup_complete=true`. RViz subscribed to
`/rog_map/occupied_voxels` and `/rog_map/inflated_voxels`; each topic had one
publisher. The session was stopped before manual flight, so the fixed-frame
map-under-yaw observation remains pending.
