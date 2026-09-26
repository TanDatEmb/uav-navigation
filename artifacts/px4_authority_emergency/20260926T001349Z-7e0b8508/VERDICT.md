# Verdict: EVIDENCE_BLOCKED

The pinned `px4_ros2_interface_lib` callback contract is verified from source. A code defect was repaired: callback `Success` and `Deactivated` no longer clear pending Hold; only `VehicleStatus.AUTO_LOITER` confirms it. Old attempts and old activations are fenced. An accepted request with no completion callback is retired and retried on the 250 ms request timeout. The final-source Release build, CTest (94/94), Python (459 passed, 1 skipped), and permanent guards pass. A final-source mapping-only 700 ms outage reached safety pause, continued stationary PX4 inputs, and the observer saw `AUTO_LOITER`.

Do not claim `PX4_AUTHORITY_EMERGENCY_CLOSED`: total retry exhaustion policy is awaiting owner input; producer-correlated C0-SW request/callback/status evidence is missing; operator/failsafe and reactivation race SITL are not captured; no controlled adapter-state stale SITL was run; and no end-to-end Emergency SITL proves the PX4 input boundary. The 700 ms injected fault run's runtime verdict is `BLOCKED` and C0-SW is `NOT_EVALUABLE` because it intentionally stopped before mission completion and lacks required producer-owned handover evidence.

No PR/merge readiness claim is made. Continue only after the retry policy is decided and the remaining handover/emergency evidence is collected.
