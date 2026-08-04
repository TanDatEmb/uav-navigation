# Odometry decision ledger

Status: P0.9-A external odometry contract and non-fusing dry run, maintained on
`feat/p0.9-a-external-odometry-contract`.

This ledger records decisions that are closed for the current odometry
lifecycle. They are not reopened by ordinary runtime noise; reopening requires
contradictory evidence from an exact-time, generation-qualified observation or
a reproducible contract/test failure.

| ID | Closed fact | Evidence and consequence |
| --- | --- | --- |
| D-001 | The PointCloud finite ratio was not the root cause of the former zero-corrected regression. | R22 replay retained the finite ratio at `0.40478515625` while isolating the regression in the propagation start epoch. It remains a separate sensor/data-quality diagnostic. |
| D-002 | The former zero-corrected regression was caused by prediction starting at scan time while the state-time prior was at an earlier epoch. | The exact-time prior propagation path now starts from the prior state epoch and was covered by physical-state tests; a handwritten IMU integrator is not permitted. |
| D-003 | Exact-time prior propagation is fixed and physically state-tested. | R22/R23 implementation and regression evidence are the basis for keeping this fix closed. Future changes must preserve the state-time contract and rollback/PSD checks. |
| D-004 | `px4_odom` and `lio_odom` are independent public frames. | Lifecycle and frame-converter contracts keep the producers separate. Alignment is an explicit comparison transform, never a silent public-frame alias. |
| D-005 | PX4-LIO alignment is yaw plus XYZ only. | The alignment math contract uses `p_lio = Rz(yaw) p_px4 + t`; roll and pitch remain residual quality checks and are rejected rather than absorbed into the public transform. |
| D-006 | `PROVISIONAL` is not production-valid; `LOCKED` is comparison-valid only. | Candidate stability, novel-pair, covariance/PSD, generation, and revalidation gates are separate from the state-machine production gates. |
| D-007 | A compensated PX4 reset does not change the public `px4_odom` frame. | The reset event starts frozen-transform revalidation while frame generation remains unchanged. An uncompensated reset invalidates the locked transform and requires a new public frame. |
| D-008 | In-flight restart semantics remain deferred. | In-flight restart is `DEFERRED_P0.10`; P0.8 closes lifecycle correctness without authorizing restart integration. |
| D-009 | The LIO producer owns the public external-frame generation. | `FastLioNode` publishes the authoritative generation and validity fields. Internal LIO generation changes, corrected/propagated handoff, PX4 reset, supervisor gate changes, and geometric jumps do not increment it. |
| D-010 | `VehicleOdometry.reset_counter` is derived from the LIO public generation. | The bridge publishes `uint8_t(public_generation % 256)`. PX4 frame generation is diagnostic-only and cannot author the external reset counter. |
| D-011 | External pose and velocity frames are explicit and separate. | Pose uses ROS local Z-up to world FRD; body velocity uses body FLU to body FRD. The quaternion is `C_world * R_lio * inverse(C_body)`; no ENU/FRD alias is permitted. |
| D-012 | Covariance is transformed and fail-closed without a synthetic floor. | Position XYZ, orientation small-angle XYZ, and body velocity XYZ are validated as finite, symmetric, PSD, and positive; full 3x3 blocks are transformed before publishing diagonal variances. Cross-covariances are ignored explicitly. |
| D-013 | P0.9-A supports only proven SITL timestamp equivalence. | `ROS_SIMULATION_TIME -> PX4_SIMULATION_TIME` is accepted with explicit measurement/publication timestamps. Unresolved real-hardware conversion remains fail-closed and deferred to P0.9-B. |
| D-014 | Publication and fusion are separate states. | The bridge may publish only after its explicit gate is ready; P0.9-A does not enable EKF2 external-vision fusion and makes no aiding, innovation, or estimator-improvement claim. |

## Query and revalidation contract

Both alignment acquisition and locked comparison select the newest propagated
epoch at or before the latest PX4 epoch. They do not extrapolate. A query key is
the tuple `(epoch_ns, lio_generation, frame_generation, time_generation)` and
is submitted at most once per generation. Eligibility, expiry, duplicate
suppression, transport failure, and geometric failure are exposed separately
in supervisor diagnostics/status.

Transport, timeout, service-unavailable, missing-response, and generation
contract failures retain a locked transform and close comparison evidence
without incrementing geometric revalidation failures. Only finite,
exact-time, generation-matching residual evidence is geometric evidence;
position, velocity, orientation, and yaw gates are explicit, with optional NIS
when covariance is available. Revalidation epochs and evidence IDs are
strictly increasing, and duplicate epochs do not count.

## Generation contract

The first public output starts at frame generation `1`. A source/time restart
before any public output reinitializes startup state without manufacturing a
frame transition. A restart after public output increments frame generation,
increments the time generation through the timestamp validator, clears the
history, and invalidates prior frame-bound alignment. Reset-event generation is
separate from frame generation. Ring-buffer interpolation requires identical
explicit frame and time generations; there is no reset-generation fallback.

## Reopen rule

Reopen a decision only with a contradictory artifact that includes the exact
HEAD, protected configuration SHA, input/output frame IDs, timestamp and
generation tuple, relevant diagnostics, and a reproducible test or replay
command. Assertion-only changes, sleeps, threshold relaxation, and frame-name
aliasing are not contradictory evidence.
