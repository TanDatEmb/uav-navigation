# P1 acceptance results

Current verdict: `P1 NOT YET ACCEPTED`.

The implementation and automated unit/build gates are complete for this
checkout. Post-change AIST OFF and RViz-backed visualization runs pass. The
headless SITL attempt failed because of simulator/external-odometry freshness
and OFFBOARD loss in both mapping-on and mapping-off runs. Manual yaw/frame
and lifecycle-clear observations also remain required.

## AIST MID360, rate 1.0

All three runs used dataset `aist-mid360-drive`, the same runtime profiles and
the same workspace state. They all reached `TRACKING`, completed cleanup, and
accepted 2756 corrections.

| Mode | Corrections | Deskewed clouds | Paired | Integrated | Scan p99 | Map update p99 | Verdict |
|---|---:|---:|---:|---:|---:|---:|---|
| `off` | 2756 | 0 | n/a | n/a | 78.1 ms | n/a | BASELINE |
| `publisher` | 2756 | 2756 | n/a | n/a | 72.4 ms | n/a | BASELINE |
| `full` | 2756 | 2756 | 2756 | 2756 | 70.8 ms | 58.4 ms | BASELINE |

The full mapping queue high-water bound was 1 and its maximum map update was
70.4 ms. LIO RSS stayed around 41--42 MiB; the reports also contain monitor
and replay RSS after the 30-second warm-up.

Artifacts:

- `off`: `.artifacts/runtime/dataset-20260810T040915-31957/REPORT.md`
- `publisher`: `.artifacts/runtime/dataset-20260810T042512-39777/REPORT.md`
- `full`: `.artifacts/runtime/dataset-20260810T043007-40524/REPORT.md`

The separate `RATE=2.0` stress attempt intentionally failed closed with input
queue pressure and drops; it is not part of the acceptance comparison.

Post-change artifacts:

- visualization OFF: `.artifacts/runtime/dataset-20260810T072954-26445/REPORT.md`
  — 2756 corrected/deskewed/paired/integrated, zero drops/mismatches, map
  update p99 54.054 ms, visualization built/published 0.
- visualization ON with RViz subscriber:
  `.artifacts/runtime/dataset-20260810T073455-27290/REPORT.md` — 2756 paired
  and integrated, zero drops/mismatches, queue bound 1, map update p99 65.022
  ms, 552 visualization frames built/published, 0 skipped, exact inflated
  snapshot 36109, surface 23709, build p95/p99/max 12736/15101/20570 us.

## Headless SITL

Post-change `sim-check --mapping-mode full` did not pass the deterministic
offboard scenario: it completed landing/disarm but recorded 38 OFFBOARD losses
and external-odometry gaps up to 688 ms. A mapping-off control run showed the
same issue (45 OFFBOARD losses, 604 ms gap), so this is not evidence of a ROG
visualization regression.

Artifacts:

- `.artifacts/runtime/sim-check-20260810T073957-28195/REPORT.md`
- `.artifacts/runtime/sim-check-20260810T074137-29653/REPORT.md`

## Visualization contract and remaining gates

The visualization outputs are `/rog_map/occupied_voxels` (measured occupied
centers), `/rog_map/inflated_voxels` (derived full keep-out debug set),
`/rog_map/inflation_surface` (derived six-connected boundary), and
`/rog_map/local_bounds` (LINE_LIST bounds). They share `frame_id=lio_odom` and
the last integrated observation stamp. Production/headless visualization is
OFF; interactive mode is subscriber-gated at `2.0 Hz`.

The fixed-frame manual gate remains pending: rotate an asymmetric object/vehicle
through approximately 0, +90, -90 and 180 degrees, then record timestamped
screenshots or a checklist showing stable occupied geometry, surface coverage,
axis-aligned bounds, unchanged continuity epoch, no yaw-following ring, and
correct min-range behavior. No manual PASS is claimed here.

Lifecycle clear was not independently observed after a controlled reset in
these sessions.

Current verdict: `P1 NOT YET ACCEPTED`.
