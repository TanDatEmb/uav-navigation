# Open findings

1. **Runtime C2 scenario not observed**: the nominal cohort did not deliberately force an emergency predecessor followed by its bounded correction. The exact-source unit/request-contract and source-path evidence pass; a dedicated SITL emergency recovery trace remains an integration evidence gap.
2. **Exact `kOptimizationFailed` on this source not observed**: both targeted attempts explicitly failed to arm before the hot handoff. Do not substitute the prior SHA runtime proof.
3. **Nominal runtime repeatability remains limited**: repair and exact-base cohorts each completed 1/3; outcomes differ. This is not enough to assert strong behavioral parity or C0-SW qualification.
4. **Full workspace CTest external debt**: 13 pinned `px4_ros2_interface_lib`/example targets failed in earlier full run due external lint/FMU wait setup. Product-relevant selected tests pass.
5. PX4/Gazebo tracking and evidence provenance findings remain outside this convergence repair.

No missing C1/C2 source behavior remains in the canonical tree. These are runtime/integration and external validation limitations.
