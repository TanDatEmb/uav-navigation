# Odometry lifecycle and PX4-LIO alignment baseline

## Scope

This is the Phase 0 baseline for branch
`fix/odometry-lifecycle-and-alignment`, started from
`57e437496d3ed914e62a94c74c63078ad76aac57`. It records the current behavior
before production changes. Existing focused tests passing is not evidence that
the lifecycle contract is implemented; the runtime findings below are the
reproductions that drive the fixes.

## Provenance

| Item | Value |
|---|---|
| Repository HEAD at baseline | `57e437496d3ed914e62a94c74c63078ad76aac57` |
| Working branch | `fix/odometry-lifecycle-and-alignment` |
| Worktree | clean at capture |
| PX4 source | `/home/letandat/Dev/Autopilot-p0.7-v1.17` |
| PX4 SHA | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| px4_msgs SHA | `86d8239e962f6939e05c3737784f60c02fa884db` |
| Compiler | `/usr/bin/g++` 13.3.0 |
| CMake build type | `RelWithDebInfo` |
| PX4-LIO config SHA-256 | `2690d584703d709e24d7159759ddffd1ab6e5d6eb165870058b98af61c587f6f` |
| Supervisor config SHA-256 | `8b2155ed3c3abb65905770086654178cedb7f906eeb4c8e02b33a85f35eee308` |
| Observer config SHA-256 | `4bc7486a0e417aec9e18080d63cfed13bcf118a742ca937b89327dd2f9403a67` |
| PX4 compatibility lock SHA-256 | `aeb97ddd0a740b11b0d8e1c9aa2b7b22ebfd2370f8d3a6cbd0acad9ea6167944` |

## Commands and results

The workspace was sourced with `/opt/ros/jazzy/setup.bash` and
`install/setup.bash`.

| Command | Result |
|---|---|
| `ctest --test-dir build/px4_odometry_bridge --output-on-failure` | 1/1 passed |
| `ctest --test-dir build/odometry_supervisor --output-on-failure` | 1/1 passed |
| `ctest --test-dir build/fast_lio_core -R 'test_(initial_state_prior\|initial_state_prior_pipeline)' --output-on-failure` | 2/2 passed |
| `ctest --test-dir build/fast_lio_ros -R 'test_(parameter_loader\|fast_lio_node_fanout)' --output-on-failure` | 2/2 passed |
| `make check` | 347 tests, 0 errors, 0 failures, dataset guard OK |
| `make vendor-check` | vendor freeze OK: 18 files, 2 pinned SHAs, 3 patched files |

## Runtime reproduction

The canonical pinned PX4 MID-360 session was run with
`SIM_PROFILE=runtime`, supervisor enabled, and the existing 60-second full
motion driver. The development artifact is:

```text
.artifacts/verification/odometry-lifecycle-baseline/
  px4-mid360-20260804-064905/
  20260804-064905-motion.jsonl
```

The motion driver completed `60.02 s` and received `5981` PX4 odometry,
`5981` converted PX4 odometry, `556` corrected LIO odometry, `2990`
propagated LIO odometry, and `1199` supervisor status samples. The relevant
machine-readable observations were:

| Observation | Baseline value |
|---|---|
| Alignment source | `initial_prior.startup_coincident_identity` |
| Alignment representation | identity default, not an estimated 4-DOF transform |
| Supervisor health samples | `1199/1199 HEALTHY` |
| `external_odometry_allowed=false` samples | `0` |
| Samples with `alignment_valid=false` | `1` |
| Samples with `comparison_valid=false` | `13` |
| PX4 reset generation | `2` at the end of the run |
| PX4 reset suppressed samples | `3` |
| LIO propagated status | `READY`, `requires_reanchor=false`, queue overflow `0` |

This is a safety reproduction: the supervisor opened the external-odometry
gate while its own runtime stream included an invalid/unavailable alignment
sample, and the only recorded alignment provenance was the startup identity
contract. It is not treated as a qualification result because this branch is
dirty only through generated artifacts and the scenario is a baseline, not a
fixed-result acceptance run.

## Confirmed pre-fix hypotheses

1. `Px4OdometryBridgeNode::on_odometry` falls back to
   `clear(); observe(sample)` whenever a reset transition is suppressed before
   the first output. This combines metadata-pending, invalid metadata, and
   discontinuity cases behind one optional result.
2. `OdometrySupervisorNode::initialize_alignment_if_ready` creates a valid
   `WorldAlignment` with identity rotation/translation after a topic prior;
   no paired fixed-window 4-DOF estimate or observability gate is used.
3. `SupervisorStateMachine::applyActions` makes the external gate true from
   `HEALTHY`/`SUSPECT` without independently requiring every safety input in
   the requested gate expression.
4. `FastLioPipeline` accepts a non-future topic prior by age and applies it at
   the current application epoch without propagating the state from the prior
   timestamp; `InitialStatePrior` carries no covariance or lifecycle
   generation.
5. The PX4 external-odometry topic is present in the PX4 message contract, but
   this repository has no production publisher from `lio_odom` to
   `/fmu/in/vehicle_visual_odometry`.

## Thresholds for later phases

No runtime acceptance threshold is changed based on this run. Later phase
reports must publish their thresholds before running fixed-result scenarios.
The baseline artifact is retained for provenance and root-cause comparison;
it must not be used to claim PASS, fixed, validated, or no-regression.
