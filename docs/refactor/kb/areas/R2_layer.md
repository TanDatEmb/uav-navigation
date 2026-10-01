# R2 — Planning backend (outside planner_core)

## Role
Planning backend numerical core outside planner_core: builds and certifies polynomial position/yaw trajectories for the planner (A* guide -> corridor -> MINCO), stores committed command bundles (CmdTraj) and exposes the product facade (`PlannerFacade`) to runtime. Layer: planning / math library. No ROS nodes, timers or executors here; everything runs on the caller's (planner worker) thread, except CmdTraj getters that the runtime can also call.

## Components
| Class / file | Responsibility |
|---|---|
| `geometry_utils::Piece` (piece.h/.cpp) | One polynomial piece: 3xD+1 coeffs, col(0)=t^D; derivatives up to snap; exact max/limit checks for V/A/J via roots/Sturm |
| `geometry_utils::Trajectory` (trajectory.h/.cpp) | Piece list + `start_WT`; time lookup (clamps past the end), Taylor re-expansion slicing (deg 5/7), max-rate checks |
| `traj_opt::MINCO_S4NU` (traj_opt/minco.h, minco.cpp) | Minimum-snap degree-7 banded solve; energy + gradients; adjoint propagation to points/times/head/tail. S2/S3 and utils/optimization/minco.h are dead |
| `BandedSystem` | LU with no pivoting, plus solve and adjoint solve |
| `traj_opt::ExpTrajOpt` (nominal_trajectory_optimizer.*) | Nominal solve: SimplifySFC -> corridor V-rep/overlaps -> guide time seed -> Bezier/MINCO certified seed -> L-BFGS -> retry ladder (bounded time stretch, penalty escalation) -> independent certificate -> checkpoint/seed fallback; diagnostics + snapshot capture |
| `traj_opt::BackupTrajOpt` | Two-piece uniform-time braking refinement with optimized switch time ts. Disabled in product (`backup_refinement_enabled: false`) |
| `traj_opt::YawTrajOpt` | One quintic over the position duration: full turn -> hold -> free-terminal stop -> bisection on the fraction of the turn |
| `traj_opt::Config` | YAML loader + validation for optimizer limits/weights/flatness |
| `trajectory_dynamics.hpp` | Sampled flatness gate (body rate, thrust) at uniform period + junctions |
| `flatness::FlatnessMap` | Differential flatness forward/backward (thrust N, quaternion, body rate), with drag |
| `lbfgs` | L-BFGS with Lewis-Overton line search; callback cancellation |
| `optimization_utils::Gcopter` | tau<->T map, sin interval map, V-polytope spatial parameterization, smoothed L1 |
| `geometry_utils.h/.cpp` | Guide time allocators, path subdivision, polytope LP/vertex enumeration, angles |
| `Polytope` + `SimplifySFC` (polytope.h) | H-polytope + route-gate metadata; corridor shortcut/trim |
| `ExpTraj`, `BackupTraj` | Candidate containers (pos, yaw, time origin, SFC, flags) |
| `CmdTraj` (cmd_traj.h) | Mutex-protected committed bundle/history; role partition MAIN/BACKUP; monotonic ns start check; generation |
| `PlannerFacade` | Pimpl product API; delegates to `Planner`; converts diagnostics; builds `TrajectorySnapshot` evaluators; unchanged-region certificate reuse |
| `PlannerRuntimeContext` | Clock provider (falls back to steady_clock), fmt logging to stdout, viz no-ops |
| `path_search::Astar` (header) | A* node/open-set types; node pool rebuilt on every search |
| vendored: quickhull, sdlp, sdqp, mvie, root_finder, ellipsoid | Geometry/LP/root numerics (reviewed at the interface only) |

## Processes & threads
- No threads, timers or callback groups are created in this area. ExpTrajOpt, BackupTrajOpt, YawTrajOpt, A* and SimplifySFC run synchronously on the planner worker thread.
- Cancellation is cooperative only: `std::atomic_bool* solve_cancelled` plus steady-clock deadlines (refinement / hard), checked inside the L-BFGS `monitorProgress` callback once per accepted iterate and around certificates. Nothing preempts a single objective evaluation. Nothing bounds SimplifySFC or the root finders (R2-24).
- `CmdTraj` holds one `std::mutex` (LOCK_G). It is taken by commit/snapshot/getters, and the snapshot copies whole trajectories while holding the lock.
- `PlannerFacade` adds no synchronization; the `Planner` it wraps owns threading.

## Flows
1. **Nominal**: `Planner` -> `ExpTrajOpt::solve(headPVAJ, tailPVAJ, guide_path, guide_t, sfcs)` -> validate/normalize planes -> `SimplifySFC` -> `setupProblemAndCheck` (`processCorridorWithGuideTraj`: V-rep, overlap interiors, route-gate seeds, guide timestamps -> `times`) -> `optimize()`:
   - build seeds: `minco.setParameters` and `buildCorridorContainedBezierSeed`, each certified with `certifyDeterministicNominalSeed`, plus duration retries;
   - run L-BFGS (`costFunctional`: tau->T + lower bound, xi->points, MINCO energy + `constraintsFunctional` penalties sampled at K+1 points per piece, adjoint gradients);
   - rebuild the candidate, check corridor/route gates and V/A/J (with velocity recovery), run the retry ladder, run `certifyOptimizedNominalCandidate` (corridor continuous, route ball, V/A/J continuous, flatness sampled), fall back to the checkpoint or the certified seed.
   - Output: `Trajectory out_traj` (start_WT overwritten by the planner) + `NominalSolveResult`.
2. **Yaw**: `YawTrajOpt::optimizeToTarget(initial yaw state, target, position traj)` -> one quintic.
3. **Backup** (disabled): `BackupTrajOpt::optimize(exp_traj, t_0, t_e, heu_ts, sfc, init)` -> L-BFGS over (T_total, points, ts) -> gates.
4. **Commit**: `CmdTraj::buildCandidate(ExpTraj, BackupTraj*)` (prefix slice + backup, role partition) -> `commitCandidate` (structural checks, monotonic ns start, generation) -> `snapshot()`.
5. **Product boundary**: `PlannerFacade::committedSnapshot()` -> `TrajectorySnapshot{evaluator, role_evaluator}` -> runtime `sample()` (guards t in [0, duration]). `validateCommittedTrajectory` either reuses the certificate when the world's changed region does not intersect the protected box, or re-sweeps.

## State machines
- `NominalSolveStatus {kRefined, kCertifiedSeed, kFailed}`: set by `classifyNominalSolveResult` (nominal_trajectory_optimizer.hpp:381-393) from optimizer success and `diagnostics_.used_certified_seed`.
- Implicit solve ladder in `ExpTrajOpt::optimize` (nominal_trajectory_optimizer.cpp):
  - seed selection 2072-2300 -> early certified-seed return 2404-2421 (baseline/suppress);
  - L-BFGS 2453 -> CANCELED -> seed or fail 2454-2497;
  - line-search stop accept 2663-2706 -> corridor reject -> seed 2710-2760;
  - bounded time stretch 3090-3100 -> feasibility retries (max 2) 3102-3284 -> final certificate 3308-3354 -> checkpoint 3356-3391 -> seed 3397-3427 -> revocation 3431-3443.
  - `retry_stop_reason` 0..6 (hpp:56-58).
- `lbfgs` return codes: LBFGS_STOP/CONVERGENCE/CANCELED/ERR* (lbfgs.cpp). ls<0 restores x.
- `YawOptimizationFailure {kNone, kInvalidInput, kNoFeasibleHold}` (yaw_traj_opt.cpp:40, 90).
- `CandidateTrajectoryRole {MAIN, BACKUP}` and `BackupDisposition` (cmd_traj.h:29-33). Transitions happen at `commitCandidate` (378-529) and `setEmpty` (219-235).
- Container flags `flag_empty_` in ExpTraj/BackupTraj/CmdTraj: ExpTraj::setEmpty leaves stale data behind (R2-19).

## Behavior per branch
- **Piece::check*Rate**: invalid piece/limit -> false; endpoint >= limit -> false; else Sturm count on [0,1]; countRoots -1 -> false (fail-closed).
- **Trajectory::locatePieceIdx**: NaN/negative t -> -1 (NaN outputs); exact boundary -> earlier piece; t > T -> clamps to the end (R2-01).
- **getPartialTrajectoryByTime**: invalid -> false; start==0 -> truncate the last piece; start degree not 5/7 -> false; else Taylor re-expansion + interior pieces + truncated end.
- **ExpTrajOpt::costFunctional**: non-finite x / T / points / MINCO / objective / gradient -> +inf with the stage recorded (1..6); otherwise cost + gradient.
- **ExpTrajOpt::optimize**:
  - bad tolerance, times < 1e-3, bad reserve -> INF (returns with a stale out_traj in some branches);
  - baseline/suppress + certified seed -> the seed;
  - CANCELED -> the seed if certified, else clear;
  - otherwise the retry ladder, then the final typed certificate, then checkpoint or seed; revocation at the end clears.
- **ExpTrajOpt::optimize (public)**: input/plane/SimplifySFC/setup errors -> false before `out_traj.clear()` (R2-09); success stamps start_WT = sim time.
- **BackupTrajOpt::optimize**: invalid inputs -> false; CANCELED -> -1.0, treated as "out of corridor" (R2-15); corridor/VAJ/flatness/floor gates.
- **YawTrajOpt::optimizeToTarget**: invalid -> kInvalidInput; full turn OK -> full; else hold OK -> bisection partial turn; else stopping displacement OK -> stop; else kNoFeasibleHold.
- **SimplifySFC**: route gates present -> trim only the tail suffix; no head/tail cell -> false; else greedy shortcut. It loops forever on non-representable adjacency (R2-24).
- **CmdTraj::buildCandidate**: bad backup start / prefix / non-finite / main-only non-rest terminal -> nullopt; builds the role partition; backup without a positive suffix -> nullopt.
- **CmdTraj::commitCandidate**: non-canonical start, gaps in the role partition, missing identity/certificate (non-emergency), start regression, generation regression -> false.
- **PlannerFacade::validateCommittedTrajectory**: no world / generation mismatch / empty -> invalid; unchanged protected region and not expired -> reuse; else full re-sweep.

## Information needs
| Input | Source | Frame | Clock | Freshness bound | Authority |
|---|---|---|---|---|---|
| head/tail PVAJ | Planner (execution anchor / goal) | world ENU | trajectory-local | per request | Planner |
| guide_path + guide_t | A* + geometry_utils allocators | world | relative s | per request | Planner (seed only) |
| SFC polytopes (+route gates) | CorridorGenerator/CIRI (R1) | world | - | per request | corridor generator; certified again by the optimizer |
| V/A/J/omega/thrust limits | planner.yaml traj_opt/boundary + mission `setMaximumVelocity` | - | - | per request | Config (mission may lower) |
| flatness params (mass, g, drag) | planner.yaml traj_opt/flatness | - | - | static; silent defaults if missing (R2-16) | Config |
| tilt limit (PX4) | **missing** | - | - | - | none (R2-13) |
| yaw limits | planner.yaml yaw_rate_max/accel | - | - | static | Planner |
| solve deadlines / cancel flag | Planner `setSolveBudget` | - | steady_clock ns | per solve | Planner |
| start_WT | ROS time via PlannerRuntimeContext (steady fallback R2-22) | - | ROS s (double) | set by Planner (overrides ExpTrajOpt, R2-09) | Planner/execution |
| world view | WorldModelView (R3/R5) | world | observation ns + epoch | epoch/revision checked | world model |

## Bottlenecks
- **L-BFGS objective** runs (K+1)*N samples per evaluation, each with a flatness forward/backward. It makes about 7 heap allocations per evaluation (R2-05) plus a propogateGrad copy. The first attempt has no iteration cap (R2-18).
- **Certificates**:
  - `certifyOptimizedNominalCandidate` runs a root-based V/A/J check plus a flatness sample every 10 ms. It is also called from inside `monitorProgress` for checkpoints.
  - `trajectoryFinite` repeats 101 samples x 2 trajectories x 2 (R2-20).
  - `getState` does 4-5 linear piece scans per sample (R2-03).
- **A\*** allocates one node per visited cell on every search (R2-21).
- **SimplifySFC** has no bound (R2-24). The nominal optimize() is a single ~1680-line function (R-09).
- **Logging**: `fmt::print` to stdout on the planner thread for every warn/info, including very long diagnostic lines in the retry ladder. `cout` precision is mutated globally (R2-06).

## Notes for target architecture
**Keep:**
- The piece/trajectory math: coefficients, derivatives and Taylor slicing were verified correct.
- Exact root/Sturm continuous V/A/J certificates that fail closed on invalid input.
- MINCO S4 matrices and gradients (verified term by term), and the tau diffeomorphism.
- Independent typed certificates (`certifyOptimizedNominalCandidate`) that are never the optimizer's own penalty.
- Immutable certified-seed fallback, cooperative cancellation with revocation re-checks, and nonfinite stage diagnostics.
- CmdTraj ns-canonical start monotonicity and role partition.
- The roundoff-bounded terminal-rest check.
- Facade pimpl isolating backend types.

**Over-engineered / unnecessary:**
- The retry ladder: seed Bezier + MINCO seed + duration retries + time stretch x3 + 2 penalty retries + checkpoint + seed fallback. Several duplicated lambdas; two different "normalized violation" metrics (R2-07/08).
- ~480 LOC of unused MINCO S2/S3 plus a dead duplicate header (R2-04).
- BackupTrajOpt (933 LOC) is disabled in product.
- Snapshot JSON writer (~700 LOC) inside the optimizer TU.
- A legacy dual plan API on the facade (R-06/R2-23).
- Three copies of the time origin per container (R2-19).
- The unused `is_known_free` flag.
- Viz no-op methods in the runtime context.

**Target:**
- One typed solve pipeline (seed -> refine -> certify) with a status enum, no double-return codes.
- A single sampling/certificate config (period, tolerances) carried in reports (R2-12/25).
- Add a tilt certificate (R2-13) and an inter-sample margin for flatness (R2-11).
- Fix the PM allocator (R2-17).
- Make flatness params required (R2-16).
- Add finite-difference gradient tests (R2-26).
